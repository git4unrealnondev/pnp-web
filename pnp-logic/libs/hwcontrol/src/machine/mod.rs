use crate::protocol::{Frame, MoveType};
use crate::transport::{Transport, TransportError};
use shared_types::*;

pub type Result<T> = std::result::Result<T, Error>;

#[derive(Debug)]
pub enum Error {
    Protocol(crate::protocol::ProtocolError),
    Transport(TransportError),
    UnsupportedLightColor(LightColor),
    /// The requested nozzle has no axis on this head.
    UnsupportedNozzle(NozzleId),
}

impl From<crate::protocol::ProtocolError> for Error {
    fn from(e: crate::protocol::ProtocolError) -> Self {
        Error::Protocol(e)
    }
}

impl From<TransportError> for Error {
    fn from(e: TransportError) -> Self {
        Error::Transport(e)
    }
}

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Error::Protocol(e) => write!(f, "protocol: {}", e),
            Error::Transport(e) => write!(f, "transport: {}", e),
            Error::UnsupportedLightColor(color) => {
                write!(f, "unsupported light color: {:?}", color)
            }
            Error::UnsupportedNozzle(nozzle) => {
                write!(f, "this head has no nozzle {}", nozzle.id())
            }
        }
    }
}

impl std::error::Error for Error {}

pub mod openkayo;
pub use openkayo::{OpenKayoMachine, Position};

#[derive(Debug, Clone, Copy)]
pub struct JogRequest {
    pub x: i8,
    pub y: i8,
    pub speed: u8,
}

pub trait Machine: Send {
    fn transport(&self) -> &dyn Transport;

    fn send_raw(&self, frame: Frame) -> Result<()> {
        self.transport().send_frame(frame)?;
        Ok(())
    }

    fn home_all(&self) -> Result<()> {
        self.send_raw(self.cmd_home_all())
    }
    fn home_z(&self) -> Result<()> {
        self.send_raw(self.cmd_home_z())
    }
    fn reset(&self) -> Result<()> {
        self.send_raw(self.cmd_reset())
    }
    fn report_position(&self) -> Result<Position>;
    fn cached_position(&self) -> Position;

    fn move_xy(&self, target: (f64, f64), speed: u8, kind: MoveType) -> Result<()>;
    fn jog_xy(&self, request: JogRequest) -> Result<()>;
    fn move_z(&self, nozzle: NozzleId, height: f64, speed: u8) -> Result<()>;

    fn rotate_nozzle(
        &self,
        nozzle: NozzleId,
        angle_deg: f64,
        clockwise: bool,
        speed: u8,
    ) -> Result<()>;
    fn rotate_nozzles(&self, directions: &[bool], speed: u8) -> Result<()>;

    fn open_vacuum(&self, nozzle: NozzleId) -> Result<()>;
    fn close_vacuum(&self, nozzle: NozzleId) -> Result<()>;
    fn open_blow(&self, nozzle: NozzleId) -> Result<()>;
    fn close_blow(&self, nozzle: NozzleId) -> Result<()>;
    fn close_vacuum_open_blow(&self, nozzle: NozzleId) -> Result<()>;
    fn close_vacuum_blow_all(&self) -> Result<()>;

    fn open_supplement_valve(&self) -> Result<()>;
    fn close_supplement_valve(&self) -> Result<()>;

    fn open_feeder(&self, id: u8) -> Result<()>;
    fn close_feeders(&self) -> Result<()>;
    fn tube_feeder_action(&self, p1: u8, p2: u8) -> Result<()>;

    fn clip_pcb(&self) -> Result<()>;
    fn release_pcb(&self) -> Result<()>;

    fn conveyor_run(&self) -> Result<()>;
    fn conveyor_stop(&self) -> Result<()>;
    fn conveyor_stop_pin_engage(&self) -> Result<()>;
    fn conveyor_clip_pcb(&self) -> Result<()>;
    fn conveyor_unclip_and_move(&self) -> Result<()>;
    fn conveyor_clear(&self) -> Result<()>;
    fn conveyor_check_pcb(&self) -> Result<()>;
    fn track_set_width(&self, units: u16) -> Result<()>;
    fn track_reset(&self) -> Result<()>;

    fn light_on(&self, id: u8, brightness: u8) -> Result<()>;
    fn light_off(&self) -> Result<()>;
    fn error_light(&self, color: shared_types::LightColor) -> Result<()>;

    fn canned_pnp(
        &self,
        nozzle: NozzleId,
        depth: f64,
        feeder: u8,
        delay: u8,
        kind: u8,
        speed: u8,
    ) -> Result<()>;

    fn cmd_home_all(&self) -> Frame;
    fn cmd_home_z(&self) -> Frame;
    fn cmd_reset(&self) -> Frame;
    fn cmd_track_reset(&self) -> Frame;
}
