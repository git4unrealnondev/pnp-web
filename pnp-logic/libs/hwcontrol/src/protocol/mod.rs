use std::fmt;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Frame(pub [u8; 8]);

impl Frame {
    pub const fn new(bytes: [u8; 8]) -> Self {
        Self(bytes)
    }

    pub fn bytes(&self) -> &[u8; 8] {
        &self.0
    }

    pub fn command(&self) -> u8 {
        self.0[0]
    }

    pub fn response(&self) -> Option<KayoResponse> {
        if self.0[0] != 0xAA || self.0[7] != 0x55 {
            return None;
        }
        Some(match self.0[1] {
            0xA1 | 0xA8 => KayoResponse::Position {
                x: -(u16::from_le_bytes([self.0[2], self.0[3]]) as f64) / 100.0,
                y: -(u16::from_le_bytes([self.0[4], self.0[5]]) as f64) / 100.0,
            },
            0xA2 => KayoResponse::ZPosition {
                microsteps: u16::from_le_bytes([self.0[2], self.0[3]]),
                nozzle: self.0[6],
            },
            0xA7 => KayoResponse::ConveyorSensor {
                present: self.0[3] != 0,
            },
            0xCC => KayoResponse::Conveyor { command: self.0[3] },
            code => KayoResponse::Generic {
                code,
                frame: self.0,
            },
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum KayoResponse {
    Position { x: f64, y: f64 },
    ZPosition { microsteps: u16, nozzle: u8 },
    ConveyorSensor { present: bool },
    Conveyor { command: u8 },
    Generic { code: u8, frame: [u8; 8] },
}

impl fmt::Display for Frame {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let parts: Vec<String> = self.0.iter().map(|b| format!("{:02X}", b)).collect();
        write!(f, "{}", parts.join(" "))
    }
}

pub mod messages;

pub use messages::{ErrorLight, JogDir, MessageBuilder, MoveType};

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ProtocolError {
    InvalidArgument(String),
    OutOfRange(String),
}

impl fmt::Display for ProtocolError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ProtocolError::InvalidArgument(s) => write!(f, "invalid argument: {}", s),
            ProtocolError::OutOfRange(s) => write!(f, "value out of range: {}", s),
        }
    }
}

impl std::error::Error for ProtocolError {}

pub type Result<T> = std::result::Result<T, ProtocolError>;
