#![no_std]
/// The _actual_ module is https://www.flyrontech.com/en/product/fn-m16p-mp3-module.html
/// _Or is it?_
/// In reality there are N+1 modules that seem to use different chips but they use the same
/// protocol
/// All use flyron chips it seems. YX5200 is the "main one" but some modules
/// use upgraded versions or even better, a chip that has the model number erased
/// The M16P-MP3 Datasheet it's way more readable than the others and seems to
/// have the least inconsistencies when checked against official libraries and 
/// other sources
use embassy_time::{Duration, Instant, Timer};
use embedded_io_async::{Read, ReadReady, Write};
use defmt::*;

const START_BYTE: u8 = 0x7E;
const END_BYTE: u8 = 0xEF;
const VERSION: u8 = 0xFF;
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
#[repr(u8)]
pub enum Source {
    USBFlash = 0b001,
    SDCard = 0b010,
    USBHost = 0b100,
}

impl TryFrom<u8> for Source {
    type Error = ();
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0b001 => Ok(Source::USBFlash),
            0b010 => Ok(Source::SDCard),
            0b100 => Ok(Source::USBHost),
            _ => Err(()),
        }
    }
}

#[derive(Debug)]
#[repr(u8)]
pub enum ModuleError {
    Busy = 1,
    Sleeping = 2,
    SerialRxError = 3,
    Checksum = 4,
    TrackNotInScope = 5,
    TrackNotFound = 6,
    InsertionError = 7,
    EnterSleep = 8,
}

impl TryFrom<u8> for ModuleError {
    type Error = ();
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            1..=8 => {
                // Safety: transmutation is only done after confirming the given
                // value is a discriminant of the ModuleError enum
                Ok(unsafe { core::mem::transmute(value) })
            }
            _ => Err(()),
        }
    }
}

#[derive(Debug)]
pub enum Error<E> {
    Init,
    SerialPort(E),
    FailedAck,
    Connection,
    ModuleError(ModuleError),
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
    pub const fn new(command: Command, param_h: u8, param_l: u8) -> Self {
        Self {
            command,
            param_h,
            param_l,
        }
    }
}

const ACK_MESSAGE_DATA: MessageData =
    MessageData::new(Command::NotifyReply, 0, 0);

#[derive(PartialEq, Debug, Clone, Copy)]
#[repr(u8)]
pub enum Command {
    /// Play next file
    Next = 0x01,
    /// Play previous file
    Previous = 0x02,
    /// Specify Track number to play (1-2999)
    PlayTrack = 0x03,
    /// Increase Volume
    VolumeUp = 0x04,
    /// Decrease Volume
    VolumeDown = 0x05,
    /// Set specific volume value, 0-30
    SetVolume = 0x06,
    /// Select equalizer
    SetEQ = 0x07,
    /// Select track to play in loop
    PlayLoopTrack = 0x08,
    /// Select the Playback source (USDB/SD)
    SetPlaybackSource = 0x09,
    /// Enter Sleep/StandBy Mode
    EnterSleepMode = 0x0A,
    /// Normal mode per DFRobot, but it's reported it does nothing
    EnterNormalMode = 0x0B,
    /// Reset the device
    Reset = 0x0C,
    /// Start Playback
    Play = 0x0D,
    /// Pause Current Playback
    Pause = 0x0E,
    /// Specify Track to play in a folder, 99 folders 255 tracks each max
    PlayTrackInFolder = 0x0F,
    /// Configure the audio amplifier gain settings, MSB enables amp, LS sets
    /// gain 0-31
    ConfigAudioAmp = 0x10, 
    /// Play all tracks in a loop
    PlayLoopAll = 0x11,
    /// Play track number in MP3 folder, max 65536 tracks, 3000 recomended max
    PlayTrackInMp3Folder = 0x12,
    /// Play track number in ADVERT folder, max 3000 tracks
    StartAdvertisement = 0x13,
    /// Play track of large folder, 1-3000 valid names
    PlayTrackLargeFolder = 0x14,
    /// Stop playing ADVERT track if one is active
    StopAdvertisement = 0x15,
    /// Stop all playback including advertisement
    Stop = 0x16,
    /// Play tracks from a folder on repeat, max 99 folders, 255 files each
    PlayLoopFolder = 0x17,
    /// Play random tracks from all media available in the current source
    PlayRandom = 0x18,
    /// If a track is playing, control loop playback enable (0x1 enable, 0x0 disable)
    LoopCurrentTrack = 0x19,
    /// Control whether the DAC is powered on or off  
    SetDAC = 0x1a,
    // Only sent by module when media is connected
    NotifyPushMedia = 0x3A,
    // Only sent by module when media is removed
    NotifyPullOutMedia = 0x3B,
    // Only sent by module when track in USB Flash finished playing
    NotifyFinishTrackUSBFlash = 0x3C,
    // Only sent by module when track in SD card finished playing
    NotifyFinishTrackSD = 0x3D,
    // Only sent by module when track in USB Host link stopped playing
    NotifyFinishTrackUSBHost = 0x3E,

    // List the available sources. For the DFPlayer Mini, it's essentially
    // only the SD card
    QueryAvailableSources = 0x3F,
    // Only sent by module when an error occurs
    NotifyError = 0x40,
    // Sent as ACK response when feedback is enabled
    NotifyReply = 0x41,
    // Returns status of module
    QueryStatus = 0x42,
    QueryVolume = 0x43,
    QueryEQ = 0x44,
    ReservedQueryPlaybackMode = 0x45,
    ReservedQuerySwVersion = 0x46,
    QueryTrackCntUSB = 0x47,
    QueryTrackCntSD = 0x48,
    ReservedQueryTrackCntPC = 0x49,
    ReservedQueryKeepOn = 0x4A,
    QueryCurrentTrackUSBFlash = 0x4B,
    QueryCurrentTrackSD = 0x4C,
    QueryCurrentTrackUSBHost = 0x4D,
    QueryFolderTrackCnt = 0x4E,
    QueryFolderCnt = 0x4F,
}

impl TryFrom<u8> for Command {
    type Error = ();
    fn try_from(value: u8) -> Result<Self, Self::Error> {
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
    USB = 0x0,
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
    if buffer[2] == 0x0 {
        checksum += 2
    };
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
    ) -> Result<Self, Error<S::Error>> {
        // wait for module to turn on
        //? Timer::after_millis(3000).await; // not needed (?
        // module sends error message after boot, it can be discarded. We also
        // ignore a possible serial/timeout error
        let mut player = Self {
            port: port,
            feedback_enable: feedback_enable,
            last_command: MessageData::new(Command::EnterNormalMode, 0, 0),
            last_response: MessageData::new(Command::EnterNormalMode, 0, 0),
            last_cmd_acknowledged: false,
            timeout: timeout,
        };
        // discard first bytes
        let _ = player.read_last_message().await;
        player.reset(reset_duration_override).await?;
        Ok(player)
    }

    pub async fn read_last_message(&mut self) -> Result<(), Error<S::Error>> {
        let timeout_start = Instant::now();
        // wait_ack loop
        let mut current_index = 0;
        let mut receive_buffer = [0u8; 30];
        let mut message = [0u8; DATA_FRAME_SIZE];
        loop {
            if !self.port.read_ready().map_err(|e| Error::SerialPort(e))? {
                if self.timeout < Instant::now().duration_since(timeout_start) {
                    return Err(Error::UserTimeout);
                }
                Timer::after_millis(10).await;
            } else {
                let cnt = self
                    .port
                    .read(&mut receive_buffer)
                    .await
                    .map_err(|e| Error::SerialPort(e))?;
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
                                        .map_err(|_| Error::Unknown)?;
                                    self.last_response.param_h = message
                                        [INDEX_PARAM_H]
                                        .try_into()
                                        .map_err(|_| Error::Unknown)?;
                                    self.last_response.param_l = message
                                        [INDEX_PARAM_L]
                                        .try_into()
                                        .map_err(|_| Error::Unknown)?;
                                    if self.last_response == ACK_MESSAGE_DATA {
                                        self.last_cmd_acknowledged = true;
                                    }
                                }

                                if i < (cnt - 1)
                                    || self
                                        .port
                                        .read_ready()
                                        .map_err(|e| Error::SerialPort(e))?
                                {
                                    current_index = 0;
                                } else {
                                    // no more data
                                    if valid_checksum {
                                        if self.last_response.command
                                            == Command::NotifyError
                                        {
                                            if let Ok(err) = self
                                                .last_response
                                                .param_l
                                                .try_into()
                                            {
                                                return Err(
                                                    Error::ModuleError(err),
                                                );
                                            } else {
                                                return Err(Error::Unknown);
                                            }
                                        } else {
                                            return Ok(());
                                        }
                                    } else {
                                        return Err(Error::BrokenMessage);
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
    ) -> Result<(), Error<S::Error>> {
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
            .map_err(|e| Error::SerialPort(e))?;
        self.last_command = command_data;

        // wait for possible error messages or ack
        // If you are an user of this crate and find this to be not enough,
        // please open an issue and attach photos and information about your
        // module

        //Timer::after_millis(60).await;
        self.last_cmd_acknowledged = false;
        self.read_last_message().await?;
        
        if self.feedback_enable && (command_data.command != Command::Reset) {
            if self.last_cmd_acknowledged != true {
                info!("wut {:?} {:?}",
                    Debug2Format(&self.last_command),
                    Debug2Format(&self.last_response));
                Err(Error::FailedAck)
            } else {
                Ok(())
            }
        } else {
            if self.port.read_ready().map_err(|e| Error::SerialPort(e))? {
                let _ = self.read_last_message().await; // prueba na mas
            }
            Ok(())
        }
    }
    pub async fn next(&mut self) -> Result<(), Error<S::Error>> {
        self.send_command(MessageData::new(Command::EnterNormalMode, 0, 0))
            .await
    }
    pub async fn reset(
        &mut self,
        reset_duration_override: Option<Duration>,
    ) -> Result<(), Error<S::Error>> {
        self.send_command(MessageData::new(Command::Reset, 0, 0))
            .await?;
        let wait = if let Some(duration) = reset_duration_override {
            duration
        } else {
            Duration::from_millis(1500) // max M16P init time
        };
        Timer::after(wait).await; // wait 3 seconds after reset
        self.read_last_message().await?;
        //todo!("Check last message to confirm state of the device");
        Ok(())
    }

    pub async fn playback_source(
        &mut self,
        playback_source: PlayBackSource,
    ) -> Result<(), Error<S::Error>> {
        self.send_command(MessageData::new(
            Command::SetPlaybackSource,
            0,
            playback_source as u8,
        ))
        .await?;
        Timer::after_millis(200).await; // max M16P file system init time
        Ok(())
    }

    pub async fn volume(&mut self, volume: u8) -> Result<(), Error<S::Error>> {
        if volume > 30 {
            return Err(Error::BadParameter);
        }
        self.send_command(MessageData::new(Command::SetVolume, 0, volume))
            .await
    }

    pub async fn play(&mut self, track: u16) -> Result<(), Error<S::Error>> {
        if track > 2999 {
            return Err(Error::BadParameter);
        }
        self.send_command(MessageData::new(
            Command::PlayTrack,
            (track >> 8) as u8,
            track as u8,
        ))
        .await
    }

    pub async fn set_equalizer(
        &mut self,
        equalizer: Equalizer,
    ) -> Result<(), Error<S::Error>> {
        self.send_command(MessageData::new(Command::SetEQ, 0, equalizer as u8))
            .await
    }
    pub async fn loop_all(
        &mut self,
        enable: bool,
    ) -> Result<(), Error<S::Error>> {
        self.send_command(MessageData::new(
            Command::PlayLoopAll,
            0,
            if enable { 1 } else { 0 },
        ))
        .await
    }
}
