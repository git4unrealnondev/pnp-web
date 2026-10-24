pub mod machine;
pub mod protocol;
pub mod transport;

use std::time::Duration;

pub use machine::{Error, JogRequest, Machine, OpenKayoMachine, Position};
pub use protocol::{
    ErrorLight, Frame, JogDir, KayoResponse, MessageBuilder, MoveType, ProtocolError,
};
pub use shared_types::{LightColor, NozzleId};
pub use transport::{
    LoggingTransport, LoopbackTransport, SerialTransport, Transport, TransportError,
};

pub fn connect_serial_kayo(
    path: &str,
    baud_rate: u32,
) -> Result<Box<dyn Machine>, Box<dyn std::error::Error>> {
    let frame_dump_dir =
        std::env::var_os("PNP_FRAME_DUMP_DIR").unwrap_or_else(|| "frame-dumps".into());
    std::fs::create_dir_all(&frame_dump_dir)?;
    // A move completion response can arrive several seconds after the
    // command, especially across the remote PTY/SSH connection.
    let port = serialport::new(path, baud_rate)
        .timeout(Duration::from_secs(10))
        .open()?;

    let transport = SerialTransport::new(port);
    let machine = OpenKayoMachine::new(transport, MessageBuilder::default());

    Ok(Box::new(machine))
}
