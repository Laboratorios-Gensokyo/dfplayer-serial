#![no_std]
use embassy_time::{Duration, Instant, Timer};
use embedded_io_async::{Read, ReadReady, Write};
use defmt::*;

const START_BYTE: u8 = 0x7E;
const END_BYTE: u8 = 0xEF;
const VERSION: u8 = 0x7F;
const MSG_LEN: u8 = 0x06;
const DATA_FRAME_SIZE: usize = 10;

const INDEX_START_BYTE: usize = 0;
const INDEX_VERSION: usize = 1;
const INDEX_CMD: usize = 3;
const INDEX_FEEDBACK_ENABLE: usize = 4;
const INDEX_PARAM_H: usize = 5;
const INDEX_PARAM_L: usize = 6;
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
    BadParameter,
}

#[derive(PartialEq, Debug, Clone, Copy)]
pub struct MessageData {
    command: Command,
    param_h: u8,
    param_l: u8,
}

impl MessageData {
    pub fn new(command: Command, param_h: u8, param_l: u8) -> Self {
        Self {
            command,
            param_h,
            param_l,
        }
    }
}

#[derive(PartialEq, Debug, Clone, Copy)]
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
    LoopAll = 0x11,
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
    LoopCurrentTrack = 0x19,
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

#[repr(u8)]
pub enum Equalizer {
    Normal = 0x0,
    Pop = 0x1,
    Rock = 0x2,
    Jazz = 0x3,
    Classic = 0x4,
    Bass = 0x5,
}

#[repr(u8)]
pub enum PlayBackMode {
    Repeat = 0x0,
    FolderRepeat = 0x1,
    SingleRepeat = 0x2,
    Random = 0x3,
}

#[repr(u8)]
pub enum PlayBackSource {
    Udisk = 0x0,
    SDCard = 0x1,
    Aux = 0x2,
    Sleep = 0x3,
    Flash = 0x4,
}

pub fn checksum(buffer: &[u8]) -> u16 {
    let mut checksum = 0;
    for &b in buffer {
        checksum += &b.into();
    }
    0u16.wrapping_sub(checksum)
}

pub struct DfPlayer<'a, S>
where
    S: Read + Write + ReadReady,
{
    port: &'a mut S,
    feedback_enable: bool,
    last_command: MessageData,
    last_response: MessageData,
    last_cmd_acknowledged: bool,
    timeout: Duration,
}

/// Structure for interacting with the device
impl<'a, S> DfPlayer<'a, S>
where
    S: Read + Write + ReadReady,
{
    /// Assumes port is configured 8N1 baud rate 9600, rx fifo reset on init
    pub async fn try_new(
        port: &'a mut S,
        feedback_enable: bool,
        timeout: Duration,
        // in case you want to modify the reset delay
        reset_duration_override: Option<Duration>,
    ) -> Result<Self, DFPlayerError<S::Error>> {
        // wait for module to turn on
        //? Timer::after_millis(3000).await; // not needed (?
        // module sends error message after boot, it can be discarded. We also
        // ignore a possible serial/timeout error
        let mut player = Self {
            port: port,
            feedback_enable: feedback_enable,
            last_command: MessageData::new(Command::Nothing, 0, 0),
            last_response: MessageData::new(Command::Nothing, 0, 0),
            last_cmd_acknowledged: false,
            timeout: timeout,
        };
        // discard first bytes
        let _ = player.read_last_message().await;
        player.reset(reset_duration_override).await?;
        Ok(player)
    }

    pub async fn read_last_message(
        &mut self,
    ) -> Result<(), DFPlayerError<S::Error>> {
        let timeout_start = Instant::now();
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
                if self.timeout < Instant::now().duration_since(timeout_start) {
                    return Err(DFPlayerError::UserTimeout);
                }
                Timer::after_millis(10).await;
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
                            info!("Received message: {:?}", &message);
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
                                    self.last_response.param_h = message
                                        [INDEX_PARAM_H]
                                        .try_into()
                                        .map_err(|_| DFPlayerError::Unknown)?;
                                    self.last_response.param_l = message
                                        [INDEX_PARAM_L]
                                        .try_into()
                                        .map_err(|_| DFPlayerError::Unknown)?;
                                    if self.last_command == self.last_response {
                                        self.last_cmd_acknowledged = true;
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
        if self.feedback_enable {
            out_buffer[INDEX_FEEDBACK_ENABLE] = 0x1;
        }
        out_buffer[INDEX_CMD] = command_data.command as u8;
        out_buffer[INDEX_PARAM_H] = command_data.param_h;
        out_buffer[INDEX_PARAM_L] = command_data.param_l;
        let checksum = checksum(&out_buffer[INDEX_VERSION..INDEX_CHECKSUM_H]);
        out_buffer[INDEX_CHECKSUM_H] = (checksum >> 8) as u8;
        out_buffer[INDEX_CHECKSUM_L] = checksum as u8;
        info!("Sending command: {:?}", out_buffer);
        self.port
            .write_all(&out_buffer)
            .await
            .map_err(|e| DFPlayerError::SerialPort(e))?;
        self.last_command = command_data;
        // Orden mensajes:
        // - ACK
        if self.feedback_enable {
            self.last_cmd_acknowledged = false;
            self.read_last_message().await?;
            if self.last_cmd_acknowledged != true {
                Err(DFPlayerError::FailedAck)
            } else {
                Ok(())
            }
        } else {
            Timer::after_millis(out_buffer.len() as u64).await;
            Ok(())
        }
    }
    pub async fn reset(
        &mut self,
        reset_duration_override: Option<Duration>,
    ) -> Result<(), DFPlayerError<S::Error>> {
        self.send_command(MessageData::new(Command::Reset, 0, 0))
            .await?;
        let wait = if let Some(duration) = reset_duration_override {
            duration
        } else {
            Duration::from_millis(3000)
        };
        Timer::after(wait).await; // wait 3 seconds after reset
        self.read_last_message().await?;
        //todo!("Check last message to confirm state of the device");
        Ok(())
    }

    pub async fn playback_source(
        &mut self,
        playback_source: PlayBackSource,
    ) -> Result<(), DFPlayerError<S::Error>> {
        self.send_command(MessageData::new(
            Command::SpecifyPlaybackSource,
            0,
            playback_source as u8,
        ))
        .await
    }
    pub async fn volume(
        &mut self,
        volume: u8,
    ) -> Result<(), DFPlayerError<S::Error>> {
        if volume > 30 {
            return Err(DFPlayerError::BadParameter);
        }
        self.send_command(MessageData::new(Command::SpecifyVolume, 0, volume))
            .await
    }

    pub async fn play(
        &mut self,
        track: u16,
    ) -> Result<(), DFPlayerError<S::Error>> {
        if track > 2999 {
            return Err(DFPlayerError::BadParameter);
        }
        self.send_command(MessageData::new(
            Command::SpecifyTracking,
            (track >> 8) as u8,
            track as u8,
        ))
        .await
    }

    pub async fn equalizer(
        &mut self,
        equalizer: Equalizer,
    ) -> Result<(), DFPlayerError<S::Error>> {
        self.send_command(MessageData::new(
            Command::SpecifyEQ,
            0,
            equalizer as u8,
        ))
        .await
    }
    pub async fn loop_all(
        &mut self,
        enable: bool,
    ) -> Result<(), DFPlayerError<S::Error>> {
        self.send_command(MessageData::new(
            Command::LoopAll,
            0,
            if enable { 1 } else { 0 },
        ))
        .await
    }
}
