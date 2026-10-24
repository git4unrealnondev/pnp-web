/// 2D point / vector in machine coordinates.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Point {
    pub x: f64,
    pub y: f64,
}

impl Point {
    pub const ORIGIN: Self = Self { x: 0.0, y: 0.0 };

    pub const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    pub fn distance(self, other: Self) -> f64 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        (dx * dx + dy * dy).sqrt()
    }
}

impl std::ops::Add for Point {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self::new(self.x + rhs.x, self.y + rhs.y)
    }
}

impl std::ops::Sub for Point {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self::new(self.x - rhs.x, self.y - rhs.y)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LightColor {
    Green,
    Red,
    Yellow,
    Off,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum NozzleId {
    N0,
    N1,
    N2,
    N3,
    N4,
    N5,
    N6,
    N7,
    N8,
}

impl NozzleId {
    /// Physical nozzle number, as the hardware addresses it (1-based).
    pub fn id(self) -> u8 {
        match self {
            NozzleId::N0 => 1,
            NozzleId::N1 => 2,
            NozzleId::N2 => 3,
            NozzleId::N3 => 4,
            NozzleId::N4 => 5,
            NozzleId::N5 => 6,
            NozzleId::N6 => 7,
            NozzleId::N7 => 8,
            NozzleId::N8 => 9,
        }
    }

    /// Every nozzle, ordered by physical number.
    pub const ALL: [NozzleId; 9] = [
        NozzleId::N0,
        NozzleId::N1,
        NozzleId::N2,
        NozzleId::N3,
        NozzleId::N4,
        NozzleId::N5,
        NozzleId::N6,
        NozzleId::N7,
        NozzleId::N8,
    ];

    /// Inverse of [`Self::id`]: physical number to nozzle.
    ///
    /// Exists so callers converting user input never hand-roll a match
    /// that can drift from `id()`. `id` is 1-based, so 0 is not a valid
    /// nozzle and returns `None`.
    pub fn from_id(id: u8) -> Option<Self> {
        match id {
            1 => Some(NozzleId::N0),
            2 => Some(NozzleId::N1),
            3 => Some(NozzleId::N2),
            4 => Some(NozzleId::N3),
            5 => Some(NozzleId::N4),
            6 => Some(NozzleId::N5),
            7 => Some(NozzleId::N6),
            8 => Some(NozzleId::N7),
            9 => Some(NozzleId::N8),
            _ => None,
        }
    }
}

/// Different types of lighting on the machine
pub enum LightingSource {
    StackLight(LightColor),
    PreciseVision,
    FastVision,
}

/// Seperate Camera Types
#[derive(Hash, Eq, PartialEq, Copy, Clone, Debug, serde::Serialize, serde::Deserialize)]
pub enum CameraTypes {
    FastVision(NozzleId),
    PreciseVision,
    Head,
}

/// Every camera the UI can address, in display order.
pub const CAMERA_NAMES: [&str; 10] = [
    "head",
    "precise",
    "fast-1",
    "fast-2",
    "fast-3",
    "fast-4",
    "fast-5",
    "fast-6",
    "fast-7",
    "fast-8",
];

impl CameraTypes {
    /// Every camera, in the same order as [`CAMERA_NAMES`].
    pub const ALL: [CameraTypes; 10] = [
        CameraTypes::Head,
        CameraTypes::PreciseVision,
        CameraTypes::FastVision(NozzleId::N0),
        CameraTypes::FastVision(NozzleId::N1),
        CameraTypes::FastVision(NozzleId::N2),
        CameraTypes::FastVision(NozzleId::N3),
        CameraTypes::FastVision(NozzleId::N4),
        CameraTypes::FastVision(NozzleId::N5),
        CameraTypes::FastVision(NozzleId::N6),
        CameraTypes::FastVision(NozzleId::N7),
    ];

    /// The name this camera is addressed by in the UI and API.
    pub const fn name(self) -> &'static str {
        match self {
            CameraTypes::Head => "head",
            CameraTypes::PreciseVision => "precise",
            CameraTypes::FastVision(nozzle) => match nozzle {
                NozzleId::N0 => "fast-1",
                NozzleId::N1 => "fast-2",
                NozzleId::N2 => "fast-3",
                NozzleId::N3 => "fast-4",
                NozzleId::N4 => "fast-5",
                NozzleId::N5 => "fast-6",
                NozzleId::N6 => "fast-7",
                NozzleId::N7 => "fast-8",
                NozzleId::N8 => "fast-9",
            },
        }
    }
}

impl std::str::FromStr for CameraTypes {
    type Err = ();

    fn from_str(name: &str) -> Result<Self, Self::Err> {
        Self::ALL
            .into_iter()
            .find(|camera| camera.name() == name)
            .ok_or(())
    }
}

/// Used to store the machines state used to determine movement of machine
#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize, serde::Deserialize, Default)]
pub enum MachineState {
    // Machine is processing a job
    #[default]
    Unknown,
    // Machine is ready for something
    Ready,
    // Machine is running a job
    RunningJob(JobState),
}
/*
impl Default for MachineState {
    fn default() -> Self {
        MachineState::Unknown
    }
}*/

/// Job-level state machine. Drives the overall run: setup → load board
/// → locate fiducials → for each placement, pick then place → done.
#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize, serde::Deserialize, Default)]
pub enum JobState {
    #[default]
    Setup,
    LoadingBoard,
    LocatingFiducials,
    Picking {
        index: usize,
    },
    Placing {
        index: usize,
    },
    Done,
    Aborted {
        reason: String,
    },
    Unloading,
}
/*
impl Default for JobState {
    fn default() -> Self {
        JobState::Setup
    }
}*/

/// A summary of a part, containing only the fields needed for display
/// and transfer between server and client without requiring the full `pnp_logic`
/// crate (which depends on OpenCV).
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct PartSummary {
    /// Part identifier.
    pub id: u32,
    /// Manufacturer part number.
    pub mpn: String,
    /// Footprint/designator name.
    pub footprint: String,
    /// Human-readable description.
    pub description: String,
    /// Nozzle allowlist as a JSON string.
    pub nozzles: String,
    /// Pickup rotation in degrees.
    pub rotation: f64,
    /// Pickup offset X in mm.
    pub offset_x: f64,
    /// Pickup offset Y in mm.
    pub offset_y: f64,
    /// Trust status of the part parameters.
    pub status: String,
    /// Camera assignment: "auto", "fast", "precise" or "head".
    pub camera: String,
}
