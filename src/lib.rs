#![no_std]
use embedded_io_async::{Read, ReadReady, Write};

const START_BYTE: u8 = 0x7E;
const END_BYTE: u8 = 0xEF;
const VERSION: u8 = 0x7F;
const MSG_LEN: u8 = 0x06;
const DATA_FRAME_SIZE: usize = 10;

const INDEX_START_BYTE: usize = 0;
const INDEX_VERSION: usize = 1;
const INDEX_CMD: usize = 3;
const INDEX_FEEDBACK_ENABLE: usize = 4;
const INDEX_PARAM1: usize = 5;
const INDEX_PARAM2: usize = 5;
const INDEX_CHECKSUM_H: usize = 7;
const INDEX_CHECKSUM_L: usize = 8;
const INDEX_END_BYTE: usize = 9;

#[derive(Debug)]
pub enum DFPlayerError<E> {
    Init,
    SerialPort(E),
    FailedAck,
    Connection,
    ModuleBusy,
    DataFrame,
    Verification,
    Unknown,
    BrokenMessage,
    UserTimeout,
}

#[derive(PartialEq)]
pub struct MessageData {
    command: Command,
    param1: u8,
    param2: u8,
}

impl MessageData {
    pub fn new(command: Command, param1: u8, param2: u8) -> Self {
        Self {
            command,
            param1,
            param2,
        }
    }
}

#[derive(PartialEq)]
#[repr(u8)]
pub enum Command {
    Nothing = 0x00,
    Next = 0x01,
    Previous = 0x02,
    SpecifyTracking = 0x03,
    IncreaseVolume = 0x04,
    DecreaseVolume = 0x05,
    SpecifyVolume = 0x06,
    SpecifyEQ = 0x07,
    SpecifyPlaybackMode = 0x08,
    SpecifyPlaybackSource = 0x09,
    EnterStandbyMode = 0x0A,
    NormalWorking = 0x0B,
    Reset = 0x0C, // triggers a message
    Playback = 0x0D,
    Pause = 0x0E,
    SpecifyFolder = 0x0F,
    DACVolumeAdjustSet = 0x10,
    RepeatPlay = 0x11,
    #[cfg(feature = "extended")]
    PlayMp3Folder = 0x12,
    #[cfg(feature = "extended")]
    Advertise = 0x13,
    #[cfg(feature = "extended")]
    PlayLargeFolder = 0x14,
    #[cfg(feature = "extended")]
    StopAdvertise = 0x15,
    #[cfg(feature = "extended")]
    Stop = 0x16,
    #[cfg(feature = "extended")]
    LoopFolder = 0x17,
    #[cfg(feature = "extended")]
    RandomPlayback = 0x18,
    #[cfg(feature = "extended")]
    ControlLoop = 0x19,
    #[cfg(feature = "extended")]
    ControlDAC = 0x1a,
    //Queries
    QueryStay0 = 0x3C,
    QueryStay1 = 0x3D,
    QueryStay2 = 0x3E,
    QueryInit = 0x3F,
    QuerySendError = 0x40,
    QueryReply = 0x41,
    QueryStatus = 0x42,
    QueryVolume = 0x43,
    QueryEQ = 0x44,
    QueryPlaybackMode = 0x45,
    QuerySwVersion = 0x46,
    QueryFileCntSD = 0x47,
    QueryFileCntUdisk = 0x48,
    QueryFileCntFlash = 0x49,
    QueryKeepOn = 0x4A,
    QueryCurrentTrackSD = 0x4B,
    QueryCurrentTrackUdisk = 0x4C,
    QueryCurrentTrackFlash = 0x4D,
}

impl TryFrom<u8> for Command {
    type Error = ();
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        #[cfg(not(feature = "extended"))]
        match value {
            0..=0x11 | 0x3C..=0x4D => {
                // Safety: transmutation is only done after confirming the given
                // value is a discriminant of the Command enum
                Ok(unsafe { core::mem::transmute(value) })
            }
            _ => Err(()),
        }
        #[cfg(feature = "extended")]
        match value {
            0..=0x1a | 0x3C..=0x4D => {
                // Safety: transmutation is only done after confirming the given
                // value is a discriminant of the Command enum
                Ok(unsafe { core::mem::transmute(value) })
            }
            _ => Err(()),
        }
    }
}

pub fn checksum(buffer: &[u8]) -> u16 {
    let mut checksum = 0;
    for &b in buffer {
        checksum += &b.into();
    }
    0u16.wrapping_sub(checksum)
}

pub struct DfPlayer<S>
where
    S: Read + Write + ReadReady,
{
    port: S,
    last_command: MessageData,
    last_response: MessageData,
    last_cmd_acknowledge: bool,
    /// timeout in case there are serial communication errors
    delay_ms: fn(u64) -> (),
    /// function that starts a timeout, waiting time depends on user
    config_timeout: fn() -> (),
    timeout_expired: fn() -> bool,
}


/// Structure for interacting with the device
impl<S> DfPlayer<S>
where
    S: Read + Write + ReadReady,
{
    /// Assumes port is configured 8N1 baud rate 9600, rx fifo reset on init
    pub async fn try_new(
        port: S,
        delay_ms: fn(u64) -> (),
        // timeout config function. User can choose whatever they prefer, but a
        // timeout of around 0.5s is advised
        config_timeout: fn() -> (),
        timeout_expired: fn() -> bool,
    ) -> Result<Self, DFPlayerError<S::Error>> {
        // wait for module to turn on
        delay_ms(3000);
        // module sends error message after boot, it can be discarded. We also
        // ignore a possible serial/timeout error
        let mut player = Self {
            port: port,
            last_command: MessageData::new(Command::Nothing, 0, 0),
            last_response: MessageData::new(Command::Nothing, 0, 0),
            last_cmd_acknowledge: false,
            delay_ms: delay_ms,
            config_timeout: config_timeout,
            timeout_expired: timeout_expired,
        };
        player.read_last_message().await?;
        player.reset().await?;
        Ok(player)
    }

    pub async fn read_last_message(
        &mut self,
    ) -> Result<(), DFPlayerError<S::Error>> {
        (self.config_timeout)();
        // wait_ack loop
        let mut current_index = 0;
        let mut receive_buffer = [0u8; 30];
        let mut message = [0u8; DATA_FRAME_SIZE];
        loop {
            if !self
                .port
                .read_ready()
                .map_err(|e| DFPlayerError::SerialPort(e))?
            {
                if (self.timeout_expired)() {
                    return Err(DFPlayerError::UserTimeout);
                }
                (self.delay_ms)(5);
            } else {
                let cnt = self
                    .port
                    .read(&mut receive_buffer)
                    .await
                    .map_err(|e| DFPlayerError::SerialPort(e))?;
                for i in 0..cnt {
                    let byte = receive_buffer[i];
                    message[current_index] = byte;

                    match current_index {
                        INDEX_START_BYTE => {
                            if byte == START_BYTE {
                                current_index = 1;
                            }
                        }
                        INDEX_END_BYTE => {
                            if byte != END_BYTE {
                                current_index = 0;
                            } else {
                                // found the end byte, but is there more info to
                                // read? Discard old info as it is not longer
                                // relevant. We'll check if the message it's an
                                // ACK message just in case

                                let read_checksum =
                                    ((message[INDEX_CHECKSUM_H] as u16) << 8)
                                        | (message[INDEX_CHECKSUM_L] as u16);
                                let calc_checksum = checksum(
                                    &message[INDEX_VERSION..INDEX_CHECKSUM_H],
                                );
                                let valid_checksum =
                                    read_checksum == calc_checksum;
                                if valid_checksum {
                                    self.last_response.command = message
                                        [INDEX_CMD]
                                        .try_into()
                                        .map_err(|_| DFPlayerError::Unknown)?;
                                    self.last_response.param1 = message
                                        [INDEX_PARAM1]
                                        .try_into()
                                        .map_err(|_| DFPlayerError::Unknown)?;
                                    self.last_response.param2 = message
                                        [INDEX_PARAM2]
                                        .try_into()
                                        .map_err(|_| DFPlayerError::Unknown)?;
                                    if self.last_command == self.last_response {
                                        self.last_cmd_acknowledge = true;
                                    }
                                }

                                if i < (cnt - 1)
                                    || self.port.read_ready().map_err(|e| {
                                        DFPlayerError::SerialPort(e)
                                    })?
                                {
                                    current_index = 0;
                                } else {
                                    // no more data
                                    if valid_checksum {
                                        return Ok(());
                                    } else {
                                        return Err(
                                            DFPlayerError::BrokenMessage,
                                        );
                                    }
                                }
                                break;
                            }
                        }
                        _ => {
                            current_index += 1;
                        }
                    }
                }
            }
        }
    }

    /// Sends a command, if ACK is enabled the paramet er value will be returned
    /// as Ok(Some(value))
    pub async fn send_command(
        &mut self,
        command_data: MessageData,
    ) -> Result<(), DFPlayerError<S::Error>> {
        let mut out_buffer = [
            START_BYTE, VERSION, MSG_LEN, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
            END_BYTE,
        ];
        out_buffer[INDEX_CMD] = command_data.command as u8;
        out_buffer[INDEX_PARAM1] = command_data.param1;
        out_buffer[INDEX_PARAM2] = command_data.param2;
        let checksum = checksum(&out_buffer[INDEX_VERSION..INDEX_CHECKSUM_H]);
        out_buffer[INDEX_CHECKSUM_H] = (checksum >> 8) as u8;
        out_buffer[INDEX_CHECKSUM_L] = checksum as u8;
        self.port
            .write_all(&out_buffer)
            .await
            .map_err(|e| DFPlayerError::SerialPort(e))?;
        // Orden mensajes:
        // - ACK
        if out_buffer[INDEX_FEEDBACK_ENABLE] != 0x1 {
            self.last_cmd_acknowledge = false;
            self.read_last_message().await?;
            if self.last_cmd_acknowledge != true {
                Err(DFPlayerError::FailedAck)
            } else {
                Ok(())
            }
        } else {
            (self.delay_ms)(out_buffer.len() as u64);
            Ok(())
        }
    }
    pub async fn reset(&mut self) -> Result<(), DFPlayerError<S::Error>> {
        self.send_command(MessageData::new(Command::Reset, 0, 0))
            .await?;
        (self.delay_ms)(3000); // wait 3 seconds after reset
        self.read_last_message().await?;
        todo!("Check last message to confirm state of the device");
    }
}
