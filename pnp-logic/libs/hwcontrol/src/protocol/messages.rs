use crate::protocol::{Frame, ProtocolError, Result};

const XY_SCALE: f64 = 100.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MoveType {
    Normal,
    T1,
    T2,
    T3,
    Throw,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JogDir {
    None,
    Plus,
    Minus,
}

impl MoveType {
    fn code(self) -> u8 {
        match self {
            MoveType::Normal => 0,
            MoveType::T1 => 1,
            MoveType::T2 => 2,
            MoveType::T3 => 3,
            MoveType::Throw => 4,
        }
    }
}

pub struct MessageBuilder {
    pub xy_home: (f64, f64),
    pub z_max: f64,
    pub z_min: f64,
    pub z_microsteps: u32,
    pub max_z_microsteps: u32,
    pub rotation_scale: f64,
}

impl Default for MessageBuilder {
    fn default() -> Self {
        Self {
            xy_home: (600.0, 600.0),
            z_max: 50.0,
            z_min: 0.0,
            z_microsteps: 16,
            max_z_microsteps: 0xFFFF,
            rotation_scale: 3200.0 / 360.0,
        }
    }
}

impl MessageBuilder {
    pub fn reset_and_home(&self) -> Frame {
        Frame([0xEE, 0xBB, 0x05, 0x00, 0x00, 0x02, 0x00, 0x55])
    }

    pub fn reset_button(&self) -> Frame {
        Frame([0xEE, 0xBB, 0x04, 0x00, 0x00, 0x02, 0x00, 0x55])
    }

    pub fn home_z(&self) -> Frame {
        Frame([0xEE, 0xBB, 0x02, 0x00, 0x00, 0x00, 0x00, 0x55])
    }

    pub fn track_reset(&self) -> Frame {
        Frame([0xEE, 0xBB, 0x06, 0x00, 0x00, 0x02, 0x00, 0x55])
    }

    pub fn clip_pcb(&self) -> Frame {
        Frame([0xEE, 0xCC, 0x00, 0x01, 0x00, 0x00, 0x00, 0x55])
    }

    pub fn release_pcb(&self) -> Frame {
        Frame([0xEE, 0xCC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x55])
    }

    pub fn move_xy_abs(&self, target: (f64, f64), speed: u8, move_type: MoveType) -> Result<Frame> {
        let mut b = [0xAA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x55];
        let mx = self.xy_home.0 - target.0;
        let my = self.xy_home.1 - target.1;
        let xb = scale_xy(mx * XY_SCALE)?;
        let yb = scale_xy(my * XY_SCALE)?;
        b[1..3].copy_from_slice(&xb.to_be_bytes());
        b[3..5].copy_from_slice(&yb.to_be_bytes());
        b[6] = (move_type.code() << 4) | (speed & 0x0F);
        Ok(Frame(b))
    }

    pub fn jog_xy(&self, x: JogDir, y: JogDir, speed: u8) -> Frame {
        let mut b = [0xEE, 0xEE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x55];
        if speed != 0 {
            b[2] = 0x01;
            b[6] = speed;
        }
        b[3] = match x {
            // KayoMsg.makeXYjogMsg negates the requested direction for the
            // machine's transposed axes: requested + is encoded as 0x02.
            JogDir::Plus => 0x02,
            JogDir::Minus => 0x01,
            JogDir::None => 0x00,
        };
        b[4] = match y {
            JogDir::Plus => 0x02,
            JogDir::Minus => 0x01,
            JogDir::None => 0x00,
        };
        Frame(b)
    }

    pub fn move_z_abs(&self, z_axis_index: u8, height_above_deck: f64, speed: u8) -> Result<Frame> {
        let mut b = [0xCC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x55];
        let micro = self.z_depth_microsteps(height_above_deck)?;
        b[1..3].copy_from_slice(&(micro as u16).to_be_bytes());
        b[6] = ((z_axis_index & 0x0F) << 4) | (speed & 0x0F);
        Ok(Frame(b))
    }

    pub fn send_z_steps(&self, z_axis_index: u8, microsteps: u16, speed: u8) -> Frame {
        let mut b = [0xCC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x55];
        b[1..3].copy_from_slice(&microsteps.to_be_bytes());
        b[6] = ((z_axis_index & 0x0F) << 4) | (speed & 0x0F);
        Frame(b)
    }

    pub fn canned_pnp(
        &self,
        z_axis_index: u8,
        depth: f64,
        feeder: u8,
        delay: u8,
        kind: u8,
        speed: u8,
    ) -> Result<Frame> {
        let mut b = [0xCC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x55];
        let micro = self.z_depth_microsteps(depth)?;
        b[1..3].copy_from_slice(&(micro as u16).to_be_bytes());
        b[3] = feeder;
        b[4] = delay;
        b[5] = kind;
        b[6] = ((z_axis_index & 0x0F) << 4) | (speed & 0x0F);
        Ok(Frame(b))
    }

    pub fn nozzle_rotation(
        &self,
        z_axis_index: u8,
        delta_deg: f64,
        clockwise: bool,
        speed: u8,
    ) -> Frame {
        let mut b = [0xDD, 0xBB, 0x00, 0x00, 0x00, 0x00, 0x00, 0x55];
        let raw = (delta_deg.abs() * self.rotation_scale).round() as i64;
        let raw = raw.clamp(0, u16::MAX as i64) as u16;
        b[2..4].copy_from_slice(&raw.to_be_bytes());
        b[5] = if clockwise { 1 } else { 0 };
        b[6] = ((speed & 0x0F) << 4) | (z_axis_index & 0x0F);
        Frame(b)
    }

    pub fn open_vac(&self, nozzle: u8) -> Frame {
        let mut b = [0xDD, 0xCC, 0x06, 0x00, 0x00, 0x00, 0x00, 0x55];
        b[6] = nozzle;
        Frame(b)
    }

    pub fn close_vac(&self, nozzle: u8) -> Frame {
        let mut b = [0xDD, 0xCC, 0x07, 0x00, 0x00, 0x00, 0x00, 0x55];
        b[6] = nozzle;
        Frame(b)
    }

    pub fn open_blow(&self, nozzle: u8) -> Frame {
        let mut b = [0xDD, 0xCC, 0x04, 0x00, 0x00, 0x00, 0x00, 0x55];
        b[6] = nozzle;
        Frame(b)
    }

    pub fn close_blow(&self, nozzle: u8) -> Frame {
        let mut b = [0xDD, 0xCC, 0x05, 0x00, 0x00, 0x00, 0x00, 0x55];
        b[6] = nozzle;
        Frame(b)
    }

    pub fn close_vac_open_blow(&self, nozzle: u8) -> Frame {
        let mut b = [0xDD, 0xCC, 0x02, 0x00, 0x00, 0x00, 0x00, 0x55];
        b[6] = nozzle;
        Frame(b)
    }

    pub fn close_vac_blow_all(&self) -> Frame {
        Frame([0xDD, 0xCC, 0x03, 0x00, 0x00, 0x00, 0x00, 0x55])
    }

    pub fn open_supplement_valve(&self) -> Frame {
        Frame([0xEE, 0xCC, 0x00, 0x08, 0x00, 0x00, 0x00, 0x55])
    }

    pub fn close_supplement_valve(&self) -> Frame {
        Frame([0xEE, 0xCC, 0x00, 0x09, 0x00, 0x00, 0x00, 0x55])
    }

    pub fn feeder_control(&self, feeder_number: u8, open: bool) -> Frame {
        let mut b = [0xBB, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x55];
        b[2] = feeder_number;
        if open {
            b[1] = 0x01;
        }
        Frame(b)
    }

    pub fn tube_feeder(&self, p1: u8, p2: u8) -> Frame {
        let mut b = [0xBB, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x55];
        b[4] = p1;
        b[6] = p2;
        Frame(b)
    }

    pub fn conveyor_run(&self) -> Frame {
        Frame([0xEE, 0xCC, 0x01, 0x00, 0x00, 0x00, 0x00, 0x55])
    }

    pub fn conveyor_stop(&self) -> Frame {
        Frame([0xEE, 0xCC, 0x00, 0x07, 0x00, 0x00, 0x00, 0x55])
    }

    pub fn conveyor_stop_pin_engage(&self) -> Frame {
        Frame([0xEE, 0xCC, 0x00, 0x06, 0x00, 0x00, 0x00, 0x55])
    }

    pub fn conveyor_clip_pcb(&self) -> Frame {
        Frame([0xEE, 0xCC, 0x00, 0x01, 0x00, 0x00, 0x00, 0x55])
    }

    pub fn conveyor_unclip_and_move(&self) -> Frame {
        Frame([0xEE, 0xCC, 0x02, 0x00, 0x00, 0x00, 0x00, 0x55])
    }

    pub fn conveyor_step(&self, step: u8) -> Frame {
        let mut b = [0xEE, 0xCC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x55];
        b[3] = step;
        Frame(b)
    }

    pub fn conveyor_clear(&self) -> Frame {
        Frame([0xEE, 0xCC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x55])
    }

    pub fn conveyor_check_pcb(&self) -> Frame {
        Frame([0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x55])
    }

    pub fn track_set_width(&self, width_units: u16) -> Frame {
        let mut b = [0xEE, 0xDD, 0x00, 0x00, 0x00, 0x00, 0x00, 0x55];
        b[2..4].copy_from_slice(&width_units.to_be_bytes());
        Frame(b)
    }

    pub fn light_on(&self, light_id: u8, brightness: u8) -> Frame {
        let mut b = [0xBB, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x55];
        b[3] = brightness;
        b[6] = light_id;
        Frame(b)
    }

    pub fn light_off(&self) -> Frame {
        Frame([0xBB, 0x00, 0x00, 0x01, 0x00, 0x00, 0x08, 0x55])
    }

    pub fn error_light(&self, color: ErrorLight) -> Frame {
        let mut b = [0xBB, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x55];
        b[1] = match color {
            ErrorLight::Red => 9,
            ErrorLight::Yellow => 8,
            ErrorLight::Off => 7,
        };
        Frame(b)
    }

    fn z_depth_microsteps(&self, height_above_deck: f64) -> Result<u32> {
        let z_clamped = height_above_deck.clamp(self.z_min, self.z_max);
        let z_conv = (self.z_max - z_clamped).max(0.0);
        let depth_deg_90 = 10.54_f64;
        let depth_deg_135 = 18.6_f64;
        let motor_steps = 1.8_f64;
        let angle = if z_conv <= depth_deg_90 {
            (z_conv / depth_deg_90).asin()
        } else {
            std::f64::consts::FRAC_PI_2
                + ((z_conv - depth_deg_90) / ((depth_deg_135 - depth_deg_90) * 1.414)).asin()
        };
        let full_steps = angle.to_degrees() / motor_steps;
        let micro = ((full_steps * self.z_microsteps as f64) + 0.5).floor() as i64;
        Ok(micro.clamp(0, self.max_z_microsteps as i64) as u32)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ErrorLight {
    Red,
    Yellow,
    Off,
}

fn scale_xy(value: f64) -> Result<u16> {
    let v = value.round();
    if !(0.0..=65535.0).contains(&v) {
        return Err(ProtocolError::OutOfRange(format!("{} not in 0..=65535", v)));
    }
    Ok(v as u16)
}
