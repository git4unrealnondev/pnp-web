use crate::protocol::Frame;
use crate::transport::{Transport, TransportError};
use std::io::{Read, Write};
use std::path::PathBuf;
use std::sync::{Mutex, OnceLock};

static FRAME_DUMP_DIR: OnceLock<Option<PathBuf>> = OnceLock::new();
static FRAME_DUMP_INDEX: Mutex<u64> = Mutex::new(0);

fn frame_dump_dir() -> Option<&'static PathBuf> {
    FRAME_DUMP_DIR
        .get_or_init(|| std::env::var_os("PNP_FRAME_DUMP_DIR").map(PathBuf::from))
        .as_ref()
}

fn dump_frame(bytes: &[u8; 8]) {
    let Some(directory) = frame_dump_dir() else {
        return;
    };
    if std::fs::create_dir_all(directory).is_err() {
        return;
    }
    let mut index = FRAME_DUMP_INDEX.lock().unwrap();
    let path = directory.join(format!("frame-{index:06}.bin"));
    *index += 1;
    let _ = std::fs::write(path, bytes);
}

pub struct SerialTransport<S: Read + Write + Send> {
    stream: Mutex<S>,
}

impl<S: Read + Write + Send> SerialTransport<S> {
    pub fn new(stream: S) -> Self {
        Self {
            stream: Mutex::new(stream),
        }
    }

    pub fn into_inner(self) -> S {
        self.stream.into_inner().unwrap()
    }
}

impl<S: Read + Write + Send> Transport for SerialTransport<S> {
    fn send_frame(&self, frame: Frame) -> Result<(), TransportError> {
        let mut stream = self.stream.lock().unwrap();
        stream.write_all(frame.bytes())?;
        stream.flush()?;
        Ok(())
    }

    fn recv_frame(&self) -> Result<Frame, TransportError> {
        let mut buf = [0u8; 8];
        self.stream.lock().unwrap().read_exact(&mut buf)?;
        dump_frame(&buf);
        Ok(Frame(buf))
    }

    fn send_text(&self, command: &str) -> Result<(), TransportError> {
        let mut stream = self.stream.lock().unwrap();
        stream.write_all(command.as_bytes())?;
        stream.flush()?;
        Ok(())
    }

    fn recv_line(&self) -> Result<String, TransportError> {
        let mut line = Vec::new();
        loop {
            let mut byte = [0u8; 1];
            self.stream.lock().unwrap().read_exact(&mut byte)?;
            line.push(byte[0]);
            if byte[0] == b'\n' {
                return String::from_utf8(line).map_err(|_| {
                    TransportError::Io(std::io::Error::new(
                        std::io::ErrorKind::InvalidData,
                        "position response is not UTF-8",
                    ))
                });
            }
        }
    }

    fn wait_for_move(&self) -> Result<Option<(f64, f64)>, TransportError> {
        loop {
            let mut bytes = [0u8; 8];
            self.stream.lock().unwrap().read_exact(&mut bytes)?;
            dump_frame(&bytes);
            if let Some(crate::protocol::KayoResponse::Position { x, y }) = Frame(bytes).response()
            {
                return Ok(Some((x, y)));
            }
        }
    }
}

pub struct LoopbackTransport {
    last_sent: Mutex<Option<Frame>>,
    response_frame: Mutex<Option<Frame>>,
    response: Mutex<Option<String>>,
    last_text: Mutex<Option<String>>,
}

impl LoopbackTransport {
    pub fn new() -> Self {
        Self {
            last_sent: Mutex::new(None),
            response_frame: Mutex::new(None),
            response: Mutex::new(None),
            last_text: Mutex::new(None),
        }
    }

    pub fn last_sent(&self) -> Option<Frame> {
        *self.last_sent.lock().unwrap()
    }

    pub fn set_response(&mut self, response: impl Into<String>) {
        *self.response.lock().unwrap() = Some(response.into());
    }

    pub fn set_frame_response(&mut self, response: Frame) {
        *self.response_frame.lock().unwrap() = Some(response);
    }

    pub fn last_text(&self) -> Option<String> {
        self.last_text.lock().unwrap().clone()
    }
}

impl Default for LoopbackTransport {
    fn default() -> Self {
        Self::new()
    }
}

impl Transport for LoopbackTransport {
    fn send_frame(&self, frame: Frame) -> Result<(), TransportError> {
        *self.last_sent.lock().unwrap() = Some(frame);
        Ok(())
    }

    fn recv_frame(&self) -> Result<Frame, TransportError> {
        self.response_frame
            .lock()
            .unwrap()
            .take()
            .ok_or(TransportError::Closed)
    }

    fn send_text(&self, command: &str) -> Result<(), TransportError> {
        *self.last_text.lock().unwrap() = Some(command.to_owned());
        Ok(())
    }

    fn recv_line(&self) -> Result<String, TransportError> {
        let mut response_guard = self.response.lock().unwrap();
        let response = response_guard.as_mut().ok_or(TransportError::Closed)?;
        if let Some((line, rest)) = response.split_once('\n') {
            let line = format!("{line}\n");
            *response = rest.to_owned();
            Ok(line)
        } else {
            Ok(response_guard.take().unwrap())
        }
    }
}
