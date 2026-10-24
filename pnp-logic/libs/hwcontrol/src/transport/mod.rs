use crate::protocol::Frame;
use std::io;
use std::sync::Mutex;

#[derive(Debug)]
pub enum TransportError {
    Io(io::Error),
    Closed,
    Unsupported(&'static str),
}

impl From<io::Error> for TransportError {
    fn from(e: io::Error) -> Self {
        TransportError::Io(e)
    }
}

impl std::fmt::Display for TransportError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            TransportError::Io(e) => write!(f, "io error: {}", e),
            TransportError::Closed => write!(f, "transport closed"),
            TransportError::Unsupported(operation) => {
                write!(f, "unsupported transport operation: {}", operation)
            }
        }
    }
}

impl std::error::Error for TransportError {}

pub trait Transport: Send {
    fn send_frame(&self, frame: Frame) -> Result<(), TransportError>;
    fn recv_frame(&self) -> Result<Frame, TransportError>;

    fn send_text(&self, _command: &str) -> Result<(), TransportError> {
        Err(TransportError::Unsupported("send_text"))
    }

    fn recv_line(&self) -> Result<String, TransportError> {
        Err(TransportError::Unsupported("recv_line"))
    }

    fn wait_for_move(&self) -> Result<Option<(f64, f64)>, TransportError> {
        Ok(None)
    }
}

pub struct LoggingTransport<T: Transport> {
    inner: Mutex<T>,
    log_tx: bool,
    log_rx: bool,
}

impl<T: Transport> LoggingTransport<T> {
    pub fn new(inner: T) -> Self {
        Self {
            inner: Mutex::new(inner),
            log_tx: true,
            log_rx: true,
        }
    }
}

impl<T: Transport> Transport for LoggingTransport<T> {
    fn send_frame(&self, frame: Frame) -> Result<(), TransportError> {
        if self.log_tx {
            println!(">> {}", frame);
        }
        self.inner.lock().unwrap().send_frame(frame)
    }

    fn recv_frame(&self) -> Result<Frame, TransportError> {
        let f = self.inner.lock().unwrap().recv_frame()?;
        if self.log_rx {
            println!("<< {}", f);
        }
        Ok(f)
    }

    fn send_text(&self, command: &str) -> Result<(), TransportError> {
        self.inner.lock().unwrap().send_text(command)
    }

    fn recv_line(&self) -> Result<String, TransportError> {
        let line = self.inner.lock().unwrap().recv_line()?;
        if self.log_rx {
            println!("<< {}", line.trim_end());
        }
        Ok(line)
    }

    fn wait_for_move(&self) -> Result<Option<(f64, f64)>, TransportError> {
        self.inner.lock().unwrap().wait_for_move()
    }
}

pub mod serial;
pub use serial::{LoopbackTransport, SerialTransport};
