use std::sync::Mutex;

use super::{Error, JogRequest, Machine, Result};
use crate::protocol::{
    Frame, JogDir, KayoResponse, MessageBuilder, MoveType, messages::ErrorLight,
};
use crate::transport::Transport;

use shared_types::*;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Position {
    pub x: f64,
    pub y: f64,
    pub z: [f64; 4],
    pub rotation: [f64; 4],
}

impl Position {
    pub fn parse_report(report: &str) -> crate::protocol::Result<Self> {
        let mut values = [None; 10];
        for token in report.split_whitespace() {
            let (key, value) = if let Some(value) = token.strip_prefix("MPos:X:") {
                ("X", value)
            } else if let Some((key, value)) = token.split_once(':') {
                (key, value)
            } else {
                continue;
            };
            let index = match key {
                "X" => 0,
                "Y" => 1,
                "I" => 2,
                "J" => 3,
                "K" => 4,
                "L" => 5,
                "A" => 6,
                "B" => 7,
                "C" => 8,
                "D" => 9,
                _ => continue,
            };
            values[index] = Some(value.parse::<f64>().map_err(|_| {
                crate::protocol::ProtocolError::InvalidArgument(format!(
                    "invalid position value for {}",
                    key
                ))
            })?);
        }
        if values.iter().any(Option::is_none) {
            return Err(crate::protocol::ProtocolError::InvalidArgument(
                "incomplete M114 position report".into(),
            ));
        }
        Ok(Self {
            x: values[0].unwrap(),
            y: values[1].unwrap(),
            z: [
                values[2].unwrap(),
                values[3].unwrap(),
                values[4].unwrap(),
                values[5].unwrap(),
            ],
            rotation: [
                values[6].unwrap(),
                values[7].unwrap(),
                values[8].unwrap(),
                values[9].unwrap(),
            ],
        })
    }
}

pub struct OpenKayoMachine<T: Transport> {
    transport: T,
    pub builder: MessageBuilder,
    position: Mutex<Position>,
}

impl<T: Transport> OpenKayoMachine<T> {
    pub fn new(transport: T, builder: MessageBuilder) -> Self {
        Self {
            transport,
            builder,
            position: Position {
                x: 600.0,
                y: 600.0,
                z: [0.0, 0.0, 0.0, 0.0],
                rotation: [0.0, 0.0, 0.0, 0.0],
            }
            .into(),
        }
    }

    pub fn with_defaults(transport: T) -> Self {
        Self::new(transport, MessageBuilder::default())
    }

    pub fn into_transport(self) -> T {
        self.transport
    }

    pub fn transport_ref(&self) -> &T {
        &self.transport
    }
}

impl<T: Transport> Machine for OpenKayoMachine<T> {
    fn transport(&self) -> &dyn Transport {
        &self.transport
    }

    fn home_all(&self) -> Result<()> {
        let mut position = self.position.lock().unwrap();
        position.rotation = [0.0; 4];
        position.z = [0.0; 4];
        position.x = 600.0;
        position.y = 600.0;
        self.send_raw(self.cmd_home_all())
    }

    fn report_position(&self) -> Result<Position> {
        loop {
            let frame = self.transport.recv_frame()?;
            if let Some(KayoResponse::Position { x, y }) = frame.response() {
                let mut position = self.position.lock().unwrap();
                position.x = x;
                position.y = y;
                return Ok(*position);
            }
        }
    }

    fn cached_position(&self) -> Position {
        *self.position.lock().unwrap()
    }

    fn move_xy(&self, target: (f64, f64), speed: u8, kind: MoveType) -> Result<()> {
        let f = self.builder.move_xy_abs(target, speed, kind)?;
        self.send_raw(f)?;
        if let Some((x, y)) = self.transport.wait_for_move()? {
            let mut position = self.position.lock().unwrap();
            position.x = x;
            position.y = y;
        }
        Ok(())
    }

    fn jog_xy(&self, request: JogRequest) -> Result<()> {
        let x = if request.x > 0 {
            JogDir::Plus
        } else if request.x < 0 {
            JogDir::Minus
        } else {
            JogDir::None
        };
        let y = if request.y > 0 {
            JogDir::Plus
        } else if request.y < 0 {
            JogDir::Minus
        } else {
            JogDir::None
        };
        let mut position = self.position.lock().unwrap();
        position.x = request.x.into();
        position.y = request.y.into();
        self.send_raw(self.builder.jog_xy(x, y, request.speed))
    }

    fn move_z(&self, nozzle: NozzleId, height: f64, speed: u8) -> Result<()> {
        let f = self.builder.move_z_abs(nozzle.id(), height, speed)?;
        self.send_raw(f)
    }

    fn rotate_nozzle(
        &self,
        nozzle: NozzleId,
        angle_deg: f64,
        clockwise: bool,
        speed: u8,
    ) -> Result<()> {
        // `rotation` has one slot per axis this head actually has. A
        // nozzle outside that range would index past the end and panic
        // while holding the lock, so reject it here instead.
        let idx = nozzle.id().saturating_sub(1) as usize;
        {
            let position = self.position.lock().unwrap();
            if idx >= position.rotation.len() {
                return Err(Error::UnsupportedNozzle(nozzle));
            }
        }
        let mut position = self.position.lock().unwrap();
        let delta = angle_deg - position.rotation[idx];
        position.rotation[idx] = angle_deg;
        let f = self
            .builder
            .nozzle_rotation(nozzle.id(), delta, clockwise, speed);
        self.send_raw(f)
    }

    fn rotate_nozzles(&self, directions: &[bool], speed: u8) -> Result<()> {
        let mut b = [0xDD, 0xB2, 0x00, 0x80, 0x00, 0x80, 0x00, 0x55];
        let mut dirfield = 0u8;
        for (i, &cw) in directions.iter().enumerate().take(4) {
            dirfield <<= 1;
            if cw {
                dirfield |= 1;
            }
            let _ = i;
        }
        b[6] = 0x80 | dirfield | (speed & 0x0F);
        self.send_raw(Frame(b))
    }

    fn open_vacuum(&self, nozzle: NozzleId) -> Result<()> {
        self.send_raw(self.builder.open_vac(nozzle.id()))
    }
    fn close_vacuum(&self, nozzle: NozzleId) -> Result<()> {
        self.send_raw(self.builder.close_vac(nozzle.id()))
    }
    fn open_blow(&self, nozzle: NozzleId) -> Result<()> {
        self.send_raw(self.builder.open_blow(nozzle.id()))
    }
    fn close_blow(&self, nozzle: NozzleId) -> Result<()> {
        self.send_raw(self.builder.close_blow(nozzle.id()))
    }
    fn close_vacuum_open_blow(&self, nozzle: NozzleId) -> Result<()> {
        self.send_raw(self.builder.close_vac_open_blow(nozzle.id()))
    }
    fn close_vacuum_blow_all(&self) -> Result<()> {
        self.send_raw(self.builder.close_vac_blow_all())
    }

    fn open_supplement_valve(&self) -> Result<()> {
        self.send_raw(self.builder.open_supplement_valve())
    }
    fn close_supplement_valve(&self) -> Result<()> {
        self.send_raw(self.builder.close_supplement_valve())
    }

    fn open_feeder(&self, id: u8) -> Result<()> {
        self.send_raw(self.builder.feeder_control(id, true))
    }
    fn close_feeders(&self) -> Result<()> {
        self.send_raw(self.builder.feeder_control(0, false))
    }
    fn tube_feeder_action(&self, p1: u8, p2: u8) -> Result<()> {
        self.send_raw(self.builder.tube_feeder(p1, p2))
    }

    fn clip_pcb(&self) -> Result<()> {
        self.send_raw(self.builder.clip_pcb())
    }
    fn release_pcb(&self) -> Result<()> {
        self.send_raw(self.builder.release_pcb())
    }

    fn conveyor_run(&self) -> Result<()> {
        self.send_raw(self.builder.conveyor_run())
    }
    fn conveyor_stop(&self) -> Result<()> {
        self.send_raw(self.builder.conveyor_stop())
    }
    fn conveyor_stop_pin_engage(&self) -> Result<()> {
        self.send_raw(self.builder.conveyor_stop_pin_engage())
    }
    fn conveyor_clip_pcb(&self) -> Result<()> {
        self.send_raw(self.builder.conveyor_clip_pcb())
    }
    fn conveyor_unclip_and_move(&self) -> Result<()> {
        self.send_raw(self.builder.conveyor_unclip_and_move())
    }
    fn conveyor_clear(&self) -> Result<()> {
        self.send_raw(self.builder.conveyor_clear())
    }
    fn conveyor_check_pcb(&self) -> Result<()> {
        self.send_raw(self.builder.conveyor_check_pcb())
    }
    fn track_set_width(&self, units: u16) -> Result<()> {
        // Manual conversion from mm to kayo units
        let adjust = ((units as f32 + 5.0 + 0.5) * 10.0) as u16;
        self.send_raw(self.builder.track_set_width(adjust - 565))
    }
    fn track_reset(&self) -> Result<()> {
        self.send_raw(self.builder.track_reset())
    }

    fn light_on(&self, id: u8, brightness: u8) -> Result<()> {
        self.send_raw(self.builder.light_on(id, brightness))
    }
    fn light_off(&self) -> Result<()> {
        self.send_raw(self.builder.light_off())
    }
    fn error_light(&self, color: shared_types::LightColor) -> Result<()> {
        let c = match color {
            LightColor::Red => ErrorLight::Red,
            LightColor::Yellow => ErrorLight::Yellow,
            LightColor::Off => ErrorLight::Off,
            LightColor::Green => return Err(super::Error::UnsupportedLightColor(color)),
        };
        self.send_raw(self.builder.error_light(c))
    }

    fn canned_pnp(
        &self,
        nozzle: NozzleId,
        depth: f64,
        feeder: u8,
        delay: u8,
        kind: u8,
        speed: u8,
    ) -> Result<()> {
        let f = self
            .builder
            .canned_pnp(nozzle.id(), depth, feeder, delay, kind, speed)?;
        self.send_raw(f)
    }

    fn cmd_home_all(&self) -> Frame {
        self.builder.reset_and_home()
    }
    fn cmd_home_z(&self) -> Frame {
        self.builder.home_z()
    }
    fn cmd_reset(&self) -> Frame {
        self.builder.reset_button()
    }
    fn cmd_track_reset(&self) -> Frame {
        self.builder.track_reset()
    }
}
