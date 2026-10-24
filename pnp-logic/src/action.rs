//! Machine-agnostic commands.
//!
//! The runner produces a stream of `Action`s; the caller dispatches them
//! to whatever hardware backend is in use (e.g. an `hwcontrol::Machine`).
//! This keeps `pnp-logic` decoupled from any specific protocol and makes
//! the runner trivial to log, replay, or mock in tests.

use shared_types::*;

/// Kind of XY move. Different machines expose these as different speed
/// profiles (linear, slow approach, fast traverse, …).
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum MoveKind {
    Linear,
    PickApproach,
    PlaceApproach,
    Fast,
}

/// Tower-light colour.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum LightColor {
    Off,
    Green,
    Yellow,
    Red,
}

/// One command to dispatch.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub enum Action {
    Home,
    Reset,
    HomeZ,

    Move {
        target: Point,
        speed: u8,
        kind: MoveKind,
    },
    MoveZ {
        nozzle: u8,
        height_mm: f64,
        speed: u8,
    },

    /// Rotate the nozzle by `delta_deg` from its current angle.
    RotateNozzle {
        nozzle: u8,
        delta_deg: f64,
        speed: u8,
    },
    /// Rotate the nozzle to an absolute angle.
    SetNozzleRotation {
        nozzle: u8,
        abs_deg: f64,
        speed: u8,
    },

    Vacuum {
        nozzle: u8,
        on: bool,
    },
    Blow {
        nozzle: u8,
        on: bool,
    },

    Feeder {
        id: u8,
        open: bool,
    },
    AdvanceFeeder {
        id: u8,
        pulses: u8,
    },

    TrackWidth {
        units: u16,
    },
    Light(LightColor),

    /// Pause and wait for the up-camera; call `Runner::feed_camera`.
    AwaitCamera,
    /// Pause and wait for the operator; call `Runner::feed_prompt_response`.
    Prompt(String),
}

impl Action {
    /// Short label for logs and trace UIs.
    pub fn label(&self) -> String {
        match self {
            Action::Home => "home".into(),
            Action::Reset => "reset".into(),
            Action::HomeZ => "homeZ".into(),
            Action::Move { target, kind, .. } => {
                format!("move({:.2},{:.2},{:?})", target.x, target.y, kind)
            }
            Action::MoveZ {
                nozzle, height_mm, ..
            } => {
                format!("moveZ(n{},{:.2}mm)", nozzle, height_mm)
            }
            Action::RotateNozzle {
                nozzle, delta_deg, ..
            } => {
                format!("rot(n{},{:+.2}°)", nozzle, delta_deg)
            }
            Action::SetNozzleRotation {
                nozzle, abs_deg, ..
            } => {
                format!("setRot(n{},{:.2}°)", nozzle, abs_deg)
            }
            Action::Vacuum { nozzle, on } => {
                format!("vac(n{},{})", nozzle, if *on { "on" } else { "off" })
            }
            Action::Blow { nozzle, on } => {
                format!("blow(n{},{})", nozzle, if *on { "on" } else { "off" })
            }
            Action::Feeder { id, open } => {
                format!("feeder({:02}{})", id, if *open { "+" } else { "-" })
            }
            Action::AdvanceFeeder { id, pulses } => {
                format!("feeder++({:02},{}x)", id, pulses)
            }
            Action::TrackWidth { units } => format!("track({})", units),
            Action::Light(c) => format!("light({:?})", c),
            Action::AwaitCamera => "awaitCamera".into(),
            Action::Prompt(s) => format!("prompt({})", s),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn labels_are_stable() {
        assert_eq!(Action::Home.label(), "home");
        assert_eq!(
            Action::Move {
                target: Point::new(1.0, 2.0),
                speed: 5,
                kind: MoveKind::Linear
            }
            .label(),
            "move(1.00,2.00,Linear)"
        );
        assert_eq!(
            Action::Vacuum {
                nozzle: 0,
                on: true
            }
            .label(),
            "vac(n0,on)"
        );
    }
}
