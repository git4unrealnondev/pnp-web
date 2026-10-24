// `Mat` is re-exported so callers can hold a captured frame without
// depending on `opencv` directly.
pub use opencv::core::Mat;
use opencv::{
    Result,
    core::{self, Point, Point2f, Scalar, Size, Vector},
    geometry::{arc_length, bounding_rect, contour_area, min_enclosing_circle},
    imgcodecs, imgproc,
    prelude::*,
    videoio,
};
use std::sync::OnceLock;
use std::{
    collections::HashMap,
    path::PathBuf,
    sync::{Arc, Mutex, atomic::AtomicBool},
    thread,
    time::Duration,
};

static CAMERA_FRAME_DUMP_DIR: OnceLock<Option<PathBuf>> = OnceLock::new();
static CAMERA_FRAME_DUMP_INDEX: Mutex<u64> = Mutex::new(0);

fn dump_camera_frame(camera_path: &str, frame: &Mat) {
    let directory = CAMERA_FRAME_DUMP_DIR
        .get_or_init(|| {
            Some(
                std::env::var_os("PNP_CAMERA_FRAME_DIR")
                    .map(PathBuf::from)
                    .unwrap_or_else(|| PathBuf::from("camera-frame-dumps")),
            )
        })
        .as_ref()
        .expect("camera frame dump directory is configured");
    if std::fs::create_dir_all(directory).is_err() {
        return;
    }
    let camera_path = PathBuf::from(camera_path);
    let camera_name = camera_path
        .file_name()
        .and_then(|name| name.to_str())
        .unwrap_or("camera");
    let mut index = CAMERA_FRAME_DUMP_INDEX.lock().unwrap();
    let path = directory.join(format!("frame-{index:06}-{camera_name}.jpg"));
    *index += 1;
    let _ = imgcodecs::imwrite(&path, frame, &Vector::new());
}

pub struct CameraManager {
    cameras: HashMap<String, Arc<Mutex<Mat>>>,
    camera_calibration: Mutex<HashMap<String, PixelToMachineCalibration>>,
    feeder_pocket_settings: Mutex<HashMap<shared_types::CameraTypes, FeederPocketVisionSettings>>,
    camera_kind: HashMap<shared_types::CameraTypes, String>,
    should_exit: Arc<AtomicBool>,
}

/// Camera-specific geometry and tolerance settings for feeder-pocket detection.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct FeederPocketVisionSettings {
    pub size_mm: f32,
    pub position_tolerance_mm: f32,
    pub size_tolerance_mm: f32,
    pub px_per_mm: f32,
}

impl Default for FeederPocketVisionSettings {
    fn default() -> Self {
        Self {
            size_mm: 10.0,
            position_tolerance_mm: 3.0,
            size_tolerance_mm: 6.0,
            px_per_mm: 20.0,
        }
    }
}

// The captured head-camera fiducials are approximately 17-20 px in diameter
// at the edge of the view. Keep enough margin for blur and JPEG variation.
const FEDUCIAL_MINIMUM_PX: f32 = 8.0;
const FEDUCIAL_MAXIMUM_PX: f32 = 80.0;

impl Default for CameraManager {
    fn default() -> Self {
        Self::new()
    }
}
impl CameraManager {
    pub fn new() -> Self {
        CameraManager {
            cameras: HashMap::new(),
            camera_calibration: Mutex::new(HashMap::new()),
            feeder_pocket_settings: Mutex::new(HashMap::new()),
            camera_kind: HashMap::new(),
            should_exit: AtomicBool::default().into(),
        }
    }

    /// Gets the feducial point from specified camera and outputs the correction in world space
    pub fn feducial_correction_in_world_space(
        &self,
        camera_type: &shared_types::CameraTypes,
    ) -> Option<shared_types::Point> {
        self.landmark_correction_in_world_space(
            camera_type,
            FEDUCIAL_MINIMUM_PX,
            FEDUCIAL_MAXIMUM_PX,
        )
    }

    /// Returns the machine-space correction needed to center a detected
    /// circular landmark, such as a feeder pocket, in the camera image.
    pub fn landmark_correction_in_world_space(
        &self,
        camera_type: &shared_types::CameraTypes,
        min_diameter: f32,
        max_diameter: f32,
    ) -> Option<shared_types::Point> {
        if let Some(camera_path) = self.camera_kind.get(camera_type)
            && let Some(camera_frame) = self.cameras.get(camera_path)
        {
            let camera_frame = camera_frame.lock().unwrap();
            dump_camera_frame(camera_path, &camera_frame);
            if let Ok(Some(point)) = detect_fiducial_in_frame_near(
                &camera_frame,
                min_diameter,
                max_diameter,
                Some((
                    camera_frame.cols() as f32 / 2.0,
                    camera_frame.rows() as f32 / 2.0,
                )),
            ) {
                let (image_width, image_height) = frame_dimensions(&camera_frame);

                let pixel_error_x = point.x as f64 - image_width as f64 / 2.0;
                let pixel_error_y = point.y as f64 - image_height as f64 / 2.0;

                let calibrations = self.camera_calibration.lock().unwrap();
                let bootstrap = PixelToMachineCalibration::bootstrap();
                let calibration = calibrations.get(camera_path).unwrap_or(&bootstrap);

                let (correction_x_mm, correction_y_mm) =
                    calibration.pixel_delta_to_machine(-pixel_error_x, -pixel_error_y);

                return Some(shared_types::Point {
                    x: correction_x_mm,
                    y: correction_y_mm,
                });
            }
        }

        None
    }

    /// Detect a feeder pocket using the dedicated pnp-capture edge pipeline
    /// and return the correction in machine coordinates.
    pub fn feeder_pocket_correction_in_world_space(
        &self,
        camera_type: &shared_types::CameraTypes,
    ) -> Option<shared_types::Point> {
        let camera_path = self.camera_kind.get(camera_type)?;
        let frame = self.cameras.get(camera_path)?.lock().ok()?.clone();
        dump_camera_frame(camera_path, &frame);
        let settings = self
            .feeder_pocket_settings
            .lock()
            .ok()
            .and_then(|settings| settings.get(camera_type).copied())
            .unwrap_or_default();
        let hint = pnp_capture::PocketHint {
            size_mm: pnp_capture::Size {
                width: settings.size_mm,
                height: settings.size_mm,
            },
            center_px: None,
            position_tolerance_mm: settings.position_tolerance_mm,
            size_tolerance_mm: settings.size_tolerance_mm,
            px_per_mm: settings.px_per_mm,
            pickup_offset_mm: pnp_capture::Point::ORIGIN,
        };
        let center = pnp_capture::detect_pocket_center_in_frame(&frame, hint).ok()??;
        let image_center_x = frame.cols() as f64 / 2.0;
        let image_center_y = frame.rows() as f64 / 2.0;
        let calibration = self.calibration(camera_type)?;
        let (x, y) = calibration.pixel_delta_to_machine(
            -(center.x as f64 - image_center_x),
            -(center.y as f64 - image_center_y),
        );
        Some(shared_types::Point::new(x, y))
    }

    pub fn feeder_pocket_settings(
        &self,
        camera_type: &shared_types::CameraTypes,
    ) -> FeederPocketVisionSettings {
        self.feeder_pocket_settings
            .lock()
            .ok()
            .and_then(|settings| settings.get(camera_type).copied())
            .unwrap_or_default()
    }

    pub fn set_feeder_pocket_settings(
        &self,
        camera_type: &shared_types::CameraTypes,
        settings: FeederPocketVisionSettings,
    ) -> Result<(), String> {
        if !settings.size_mm.is_finite()
            || !settings.position_tolerance_mm.is_finite()
            || !settings.size_tolerance_mm.is_finite()
            || !settings.px_per_mm.is_finite()
            || settings.size_mm <= 0.0
            || settings.position_tolerance_mm < 0.0
            || settings.size_tolerance_mm < 0.0
            || settings.px_per_mm <= 0.0
        {
            return Err("feeder-pocket vision settings contain invalid values".to_string());
        }
        self.feeder_pocket_settings
            .lock()
            .map_err(|_| "feeder-pocket settings lock poisoned".to_string())?
            .insert(*camera_type, settings);
        Ok(())
    }

    // Adds camera into the manager
    pub fn add_camera(
        &mut self,
        camera_path: &str,
        camera_type: shared_types::CameraTypes,
        camera_calibration: Option<PixelToMachineCalibration>,
    ) -> Result<()> {
        let camera = videoio::VideoCapture::from_file(camera_path, videoio::CAP_V4L2)?;
        let latest_frame = Arc::new(Mutex::new(Mat::default()));
        let should_exit = self.should_exit.clone();
        let latest_frame_clone = latest_frame.clone();
        thread::spawn(move || {
            let mut temp_camera = camera;
            loop {
                if should_exit.load(std::sync::atomic::Ordering::SeqCst) {
                    break;
                }

                if let Ok(true) = temp_camera.grab()
                    && let Ok(mut frame_lock) = latest_frame_clone.lock()
                    && temp_camera.retrieve(&mut *frame_lock, 0).is_ok()
                    && !frame_lock.empty()
                {}

                // Targeting ~30 FPS
                thread::sleep(Duration::from_millis(30));
            }
        });

        self.cameras.insert(camera_path.to_string(), latest_frame);
        self.camera_kind
            .insert(camera_type, camera_path.to_string());
        if let Some(camera_calibration) = camera_calibration {
            self.camera_calibration
                .lock()
                .unwrap()
                .insert(camera_path.to_string(), camera_calibration);
        }

        Ok(())
    }

    /// Gets PNG bytes from cameratype
    pub fn get_png_bytes_kind(&self, camera_type: &shared_types::CameraTypes) -> Result<Vec<u8>> {
        if let Some(camera_path) = self.camera_kind.get(camera_type) {
            self.get_png_bytes_path(camera_path)
        } else {
            Err(opencv::Error {
                code: 1,
                message: format!("Failed to find cached camera at: {:?}", camera_type),
            })
        }
    }

    pub fn get_png_bytes_path(&self, camera_path: &str) -> Result<Vec<u8>> {
        let frame = if let Some(mat) = self.cameras.get(camera_path) {
            let guard = mat.lock().map_err(|_| opencv::Error {
                code: 1,
                message: format!("Mutex poisoned for camera at: {}", camera_path),
            })?;

            if guard.empty() {
                return Err(opencv::Error {
                    code: 1,
                    message: format!("Camera frame is empty at: {}", camera_path),
                });
            }

            guard.clone()
        } else {
            return Err(opencv::Error {
                code: 1,
                message: format!("Failed to find cached camera at: {}", camera_path),
            });
        };

        let mut encoded = Vector::<u8>::new();
        imgcodecs::imencode(".png", &frame, &mut encoded, &Vector::new())?;

        Ok(encoded.to_vec())
    }

    /// Fit and store the pixel-to-machine transform for a camera.
    pub fn set_calibration(
        &self,
        camera_type: &shared_types::CameraTypes,
        samples: &[CalibrationSample],
    ) -> std::result::Result<PixelToMachineCalibration, String> {
        let camera_path = self
            .camera_kind
            .get(camera_type)
            .ok_or_else(|| format!("camera is not configured: {camera_type:?}"))?;
        let calibration = calibrate_pixel_to_machine(samples)?;
        self.camera_calibration
            .lock()
            .map_err(|_| "camera calibration lock poisoned".to_string())?
            .insert(camera_path.clone(), calibration);
        Ok(calibration)
    }

    pub fn calibration(
        &self,
        camera_type: &shared_types::CameraTypes,
    ) -> Option<PixelToMachineCalibration> {
        let path = self.camera_kind.get(camera_type)?;
        self.camera_calibration.lock().ok()?.get(path).copied()
    }

    pub fn set_calibration_value(
        &self,
        camera_type: &shared_types::CameraTypes,
        calibration: PixelToMachineCalibration,
    ) -> Result<(), String> {
        let path = self
            .camera_kind
            .get(camera_type)
            .ok_or_else(|| format!("camera is not configured: {camera_type:?}"))?;
        self.camera_calibration
            .lock()
            .map_err(|_| "camera calibration lock poisoned".to_string())?
            .insert(path.clone(), calibration);
        Ok(())
    }

    /// Encodes a camera frame as a lower-cost JPEG for the web preview.
    pub fn get_jpeg_bytes_kind(&self, camera_type: &shared_types::CameraTypes) -> Result<Vec<u8>> {
        let camera_path = self
            .camera_kind
            .get(camera_type)
            .ok_or_else(|| opencv::Error {
                code: 1,
                message: format!("Failed to find cached camera at: {:?}", camera_type),
            })?;
        let frame = self.cameras.get(camera_path).ok_or_else(|| opencv::Error {
            code: 1,
            message: format!("Failed to find cached camera at: {}", camera_path),
        })?;
        let frame = {
            let frame = frame.lock().map_err(|_| opencv::Error {
                code: 1,
                message: format!("Mutex poisoned for camera at: {}", camera_path),
            })?;
            if frame.empty() {
                return Err(opencv::Error {
                    code: 1,
                    message: format!("Camera frame is empty at: {}", camera_path),
                });
            }
            frame.clone()
        };

        let preview = if frame.cols() > 640 {
            let height = frame.rows() * 640 / frame.cols();
            let mut resized = Mat::default();
            imgproc::resize(
                &frame,
                &mut resized,
                Size::new(640, height),
                0.0,
                0.0,
                imgproc::INTER_AREA,
            )?;
            resized
        } else {
            frame
        };

        let mut encoded = Vector::<u8>::new();
        let params = Vector::from_slice(&[imgcodecs::IMWRITE_JPEG_QUALITY, 75]);
        imgcodecs::imencode(".jpg", &preview, &mut encoded, &params)?;
        Ok(encoded.to_vec())
    }

    /// Clone the latest frame for vision processing.
    pub fn camera_frame_kind(&self, camera_type: &shared_types::CameraTypes) -> Result<Mat> {
        let camera_path = self
            .camera_kind
            .get(camera_type)
            .ok_or_else(|| opencv::Error {
                code: 1,
                message: format!("Failed to find cached camera at: {:?}", camera_type),
            })?;
        let frame = self.cameras.get(camera_path).ok_or_else(|| opencv::Error {
            code: 1,
            message: format!("Failed to find cached camera at: {}", camera_path),
        })?;
        let frame = frame.lock().map_err(|_| opencv::Error {
            code: 1,
            message: format!("Mutex poisoned for camera at: {}", camera_path),
        })?;
        if frame.empty() {
            return Err(opencv::Error {
                code: 1,
                message: format!("Camera frame is empty at: {}", camera_path),
            });
        }
        dump_camera_frame(camera_path, &frame);
        Ok(frame.clone())
    }
}

/// Return the width and height of a captured image in pixels.
pub fn image_dimensions(image_path: &str) -> Result<(i32, i32)> {
    let image = imgcodecs::imread(image_path, imgcodecs::IMREAD_GRAYSCALE)?;
    if image.empty() {
        return Err(opencv::Error::new(1, "Failed to load image"));
    }
    Ok((image.cols(), image.rows()))
}

/// Capture one frame from a V4L2 camera device.
pub fn capture_frame(camera_path: &str) -> Result<Mat> {
    let mut camera = videoio::VideoCapture::from_file(camera_path, videoio::CAP_V4L2)?;
    if !camera.is_opened()? {
        return Err(opencv::Error::new(
            1,
            format!("failed to open camera {camera_path}"),
        ));
    }
    let _ = camera.set(videoio::CAP_PROP_BUFFERSIZE, 1.0);
    let mut frame = Mat::default();
    if !camera.read(&mut frame)? || frame.empty() {
        return Err(opencv::Error::new(1, "camera returned an empty frame"));
    }
    Ok(frame)
}

/// Reusable camera handle. V4L2 devices are kept open between captures so
/// each request does not race camera initialization.
pub struct CameraCapture {
    camera_path: String,
    camera: videoio::VideoCapture,
    frame_size: Option<(i32, i32, i32)>,
}

impl CameraCapture {
    pub fn path(&self) -> &str {
        &self.camera_path
    }

    pub fn open(camera_path: &str) -> Result<Self> {
        let mut camera = videoio::VideoCapture::from_file(camera_path, videoio::CAP_V4L2)?;
        if !camera.is_opened()? {
            return Err(opencv::Error::new(
                1,
                format!("failed to open camera {camera_path}"),
            ));
        }
        let _ = camera.set(videoio::CAP_PROP_BUFFERSIZE, 1.0);
        let mut capture = Self {
            camera_path: camera_path.to_string(),
            camera,
            frame_size: None,
        };
        capture.warm_up(3)?;
        Ok(capture)
    }

    pub fn read(&mut self) -> Result<Mat> {
        for _ in 0..3 {
            if let Some(frame) = Self::read_latest(&mut self.camera)? {
                if self.accept_frame(&frame) {
                    return Ok(frame);
                }
            }
        }

        // Reopen once if the device temporarily returned empty frames.
        self.camera = videoio::VideoCapture::from_file(&self.camera_path, videoio::CAP_V4L2)?;
        self.frame_size = None;
        if !self.camera.is_opened()? {
            return Err(opencv::Error::new(
                1,
                format!("failed to reopen camera {}", self.camera_path),
            ));
        }
        let _ = self.camera.set(videoio::CAP_PROP_BUFFERSIZE, 1.0);
        self.warm_up(3)?;
        for _ in 0..3 {
            if let Some(frame) = Self::read_latest(&mut self.camera)? {
                if self.accept_frame(&frame) {
                    return Ok(frame);
                }
            }
        }
        Err(opencv::Error::new(
            1,
            "camera returned invalid frame sizes after reopen",
        ))
    }

    fn accept_frame(&mut self, frame: &Mat) -> bool {
        let size = (frame.cols(), frame.rows(), frame.channels());
        match self.frame_size {
            Some(expected) => expected == size,
            None => {
                self.frame_size = Some(size);
                true
            }
        }
    }

    fn read_latest(camera: &mut videoio::VideoCapture) -> Result<Option<Mat>> {
        // `read` both grabs and decodes a frame. With some V4L2 capture cards
        // that returns queued buffers in order, even when BUFFERSIZE=1 was
        // accepted. Grab and discard a burst first, then retrieve only the
        // newest grabbed frame.
        let mut grabbed = false;
        for _ in 0..25 {
            if camera.grab()? {
                grabbed = true;
            } else {
                break;
            }
        }
        if !grabbed {
            return Ok(None);
        }
        let mut frame = Mat::default();
        if !camera.retrieve(&mut frame, 0)? || frame.empty() {
            return Ok(None);
        }
        Ok(Some(frame))
    }

    fn warm_up(&mut self, count: usize) -> Result<()> {
        for _ in 0..count {
            let mut frame = Mat::default();
            let _ = self.camera.read(&mut frame)?;
        }
        Ok(())
    }
}

/// Return the width and height of an OpenCV frame in pixels.
pub fn frame_dimensions(frame: &Mat) -> (i32, i32) {
    (frame.cols(), frame.rows())
}

/// Encode an OpenCV frame as a PNG for transport to the web UI.
pub fn encode_png(frame: &Mat) -> Result<Vec<u8>> {
    let mut encoded = Vector::<u8>::new();
    imgcodecs::imencode(".png", frame, &mut encoded, &Vector::new())?;
    Ok(encoded.to_vec())
}

/// Save an OpenCV frame to a local image file.
pub fn save_frame(frame: &Mat, path: impl AsRef<std::ffi::OsStr>) -> Result<()> {
    if !imgcodecs::imwrite(path, frame, &Vector::new())? {
        return Err(opencv::Error::new(1, "OpenCV failed to write image"));
    }
    Ok(())
}

/// Draw the detected fiducial and the optical centre on a frame.
pub fn render_fiducial_debug_frame(frame: &Mat, point: Option<Point>) -> Result<Mat> {
    render_fiducial_debug_frame_with_box(frame, point, None)
}

/// Render a fiducial, optical centre, and optional refinement box.
pub fn render_fiducial_debug_frame_with_box(
    frame: &Mat,
    point: Option<Point>,
    refinement_box: Option<core::Rect>,
) -> Result<Mat> {
    let mut debug = frame.clone();
    let center = Point::new(frame.cols() / 2, frame.rows() / 2);
    if let Some(rect) = refinement_box {
        imgproc::rectangle(
            &mut debug,
            rect,
            Scalar::new(255.0, 0.0, 255.0, 0.0),
            2,
            imgproc::LINE_AA,
            0,
        )?;
    }
    imgproc::draw_marker(
        &mut debug,
        center,
        Scalar::new(255.0, 0.0, 0.0, 0.0),
        imgproc::MARKER_CROSS,
        30,
        2,
        imgproc::LINE_AA,
    )?;
    if let Some(point) = point {
        imgproc::draw_marker(
            &mut debug,
            point,
            Scalar::new(0.0, 255.0, 0.0, 0.0),
            imgproc::MARKER_CROSS,
            30,
            2,
            imgproc::LINE_AA,
        )?;
        imgproc::line(
            &mut debug,
            center,
            point,
            Scalar::new(0.0, 0.0, 255.0, 0.0),
            1,
            imgproc::LINE_AA,
            0,
        )?;
    }
    Ok(debug)
}

/// Detect one ink mark near an expected image position.
///
/// Thin wrapper over [`detect_fiducial_in_frame_near`] that exists so
/// callers holding a calibration can seed the search from a machine
/// position. Ink on paper is a small dark disk, which is the same shape
/// the fiducial detector looks for, so no new detector is needed.
pub fn detect_ink_mark_in_frame(
    frame: &Mat,
    expected_pixel: (f32, f32),
    min_diameter_px: f32,
    max_diameter_px: f32,
) -> Result<Option<Point>> {
    detect_fiducial_in_frame_near(frame, min_diameter_px, max_diameter_px, Some(expected_pixel))
}

/// Detect a fiducial directly in a captured camera frame.
pub fn detect_fiducial_in_frame(
    frame: &Mat,
    min_diameter: f32,
    max_diameter: f32,
) -> Result<Option<Point>> {
    detect_fiducial_in_frame_near(
        frame,
        min_diameter,
        max_diameter,
        Some((frame.cols() as f32 / 2.0, frame.rows() as f32 / 2.0)),
    )
}

/// Detect a fiducial, preferring candidates closest to `expected_position`.
/// The expected position is in camera pixels and is useful when several
/// circular features are visible in the same frame.
pub fn detect_fiducial_in_frame_near(
    frame: &Mat,
    min_diameter: f32,
    max_diameter: f32,
    expected_position: Option<(f32, f32)>,
) -> Result<Option<Point>> {
    Ok(detect_fiducial_in_image_with_box_near(
        frame,
        min_diameter,
        max_diameter,
        expected_position,
    )?
    .map(|(point, _)| point))
}

/// Detect a fiducial and return the local refinement box used for the final
/// position. The first pass finds a candidate in the full image; the second
/// pass searches only inside a box around that candidate and translates the
/// result back into full-image coordinates.
pub fn detect_fiducial_in_image_with_box(
    image: &Mat,
    min_diameter: f32,
    max_diameter: f32,
) -> Result<Option<(Point, core::Rect)>> {
    detect_fiducial_in_image_with_box_near(image, min_diameter, max_diameter, None)
}

pub fn detect_fiducial_in_image_with_box_near(
    image: &Mat,
    min_diameter: f32,
    max_diameter: f32,
    expected_position: Option<(f32, f32)>,
) -> Result<Option<(Point, core::Rect)>> {
    let coarse =
        detect_fiducial_in_image_coarse_near(image, min_diameter, max_diameter, expected_position)?;
    let Some(coarse) = coarse else {
        return Ok(None);
    };

    let half_box = (max_diameter.ceil() as i32).max(8);
    let x0 = (coarse.x - half_box).max(0);
    let y0 = (coarse.y - half_box).max(0);
    let x1 = (coarse.x + half_box + 1).min(image.cols());
    let y1 = (coarse.y + half_box + 1).min(image.rows());
    let rect = core::Rect::new(x0, y0, (x1 - x0).max(1), (y1 - y0).max(1));
    let roi = Mat::roi(image, rect)?.try_clone()?;
    let refined = detect_fiducial_in_image_coarse_near(&roi, min_diameter, max_diameter, None)?
        .map(|point| Point::new(point.x + rect.x, point.y + rect.y))
        .unwrap_or(coarse);
    Ok(Some((refined, rect)))
}

/// One observation of a fixed fiducial during pixel-to-machine calibration.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CalibrationSample {
    pub machine_x_mm: f64,
    pub machine_y_mm: f64,
    pub pixel_x: f64,
    pub pixel_y: f64,
}

/// Affine transform from image pixels to machine XY coordinates.
///
/// The transform is fitted as:
///
/// ```text
/// machine_x = a * pixel_x + b * pixel_y + c
/// machine_y = d * pixel_x + e * pixel_y + f
/// ```
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct PixelToMachineCalibration {
    pub a: f64,
    pub b: f64,
    pub c: f64,
    pub d: f64,
    pub e: f64,
    pub f: f64,
    pub rms_error_mm: f64,
}

// Lets us run functions for feducaials without cal data
impl Default for PixelToMachineCalibration {
    fn default() -> Self {
        PixelToMachineCalibration {
            a: 0.0,
            b: 0.0,
            c: 0.0,
            d: 0.0,
            e: 0.0,
            f: 0.0,
            rms_error_mm: 0.0,
        }
    }
}

impl PixelToMachineCalibration {
    /// Approximate transform used only to center a fiducial before the first
    /// real calibration has been measured. The calibration flow replaces it.
    pub fn bootstrap() -> Self {
        Self {
            a: 1.0 / 20.0,
            b: 0.0,
            c: 0.0,
            d: 0.0,
            e: 1.0 / 20.0,
            f: 0.0,
            rms_error_mm: 0.0,
        }
    }

    /// Image centre in pixels, as (x, y).
    pub fn image_center(frame: &Mat) -> (f64, f64) {
        (frame.cols() as f64 * 0.5, frame.rows() as f64 * 0.5)
    }

    /// Image scale along the pixel-x axis, in pixels per millimetre.
    ///
    /// Uses the norm of the (a, d) column, so it stays correct for a
    /// rotated or skewed calibration instead of assuming an axis-aligned
    /// camera.
    pub fn pixels_per_mm_x(&self) -> f64 {
        (self.a * self.a + self.d * self.d).sqrt()
    }

    /// Image scale along the pixel-y axis, in pixels per millimetre.
    pub fn pixels_per_mm_y(&self) -> f64 {
        (self.b * self.b + self.e * self.e).sqrt()
    }

    /// Inverse of [`Self::pixel_to_machine`]: where a machine point falls
    /// in the image, given the image centre.
    ///
    /// Solves the 2x2 system forward rather than inverting it, so a
    /// singular or near-singular calibration returns `None` instead of
    /// producing infinities that would silently mis-seed a search.
    pub fn machine_to_pixel(
        &self,
        machine_x: f64,
        machine_y: f64,
        image_center: (f64, f64),
    ) -> Option<(f32, f32)> {
        let det = self.a * self.e - self.b * self.d;
        if !det.is_finite() || det.abs() < 1e-12 {
            return None;
        }
        let dx = machine_x - self.c;
        let dy = machine_y - self.f;
        let pixel_x = (self.e * dx - self.b * dy) / det + image_center.0;
        let pixel_y = (self.a * dy - self.d * dx) / det + image_center.1;
        if !pixel_x.is_finite() || !pixel_y.is_finite() {
            return None;
        }
        Some((pixel_x as f32, pixel_y as f32))
    }

    /// Convert a pixel displacement into a machine displacement.
    pub fn pixel_delta_to_machine(&self, pixel_x: f64, pixel_y: f64) -> (f64, f64) {
        (
            self.a * pixel_x + self.b * pixel_y,
            self.d * pixel_x + self.e * pixel_y,
        )
    }

    /// Convert an absolute image point into a machine point.
    pub fn pixel_to_machine(&self, pixel_x: f64, pixel_y: f64) -> (f64, f64) {
        (
            self.a * pixel_x + self.b * pixel_y + self.c,
            self.d * pixel_x + self.e * pixel_y + self.f,
        )
    }
}

/// Fit a pixel-to-machine affine transform from observations of one fixed
/// fiducial at known machine positions.
///
/// At least three non-collinear machine positions are required. Additional
/// samples are handled by least squares, which makes the result less
/// sensitive to fiducial detection noise.
pub fn calibrate_pixel_to_machine(
    samples: &[CalibrationSample],
) -> std::result::Result<PixelToMachineCalibration, String> {
    if samples.len() < 3 {
        return Err("at least three calibration samples are required".into());
    }
    if samples.iter().any(|sample| {
        !sample.machine_x_mm.is_finite()
            || !sample.machine_y_mm.is_finite()
            || !sample.pixel_x.is_finite()
            || !sample.pixel_y.is_finite()
    }) {
        return Err("calibration samples must contain finite values".into());
    }

    // Normal equations for the two independent affine output coordinates.
    let mut normal = [[0.0; 3]; 3];
    let mut x_rhs = [0.0; 3];
    let mut y_rhs = [0.0; 3];
    for sample in samples {
        let row = [sample.pixel_x, sample.pixel_y, 1.0];
        for i in 0..3 {
            for j in 0..3 {
                normal[i][j] += row[i] * row[j];
            }
            x_rhs[i] += row[i] * sample.machine_x_mm;
            y_rhs[i] += row[i] * sample.machine_y_mm;
        }
    }

    let x = solve_3x3(normal, x_rhs)?;
    let y = solve_3x3(normal, y_rhs)?;
    let mut squared_error = 0.0;
    for sample in samples {
        let predicted_x = x[0] * sample.pixel_x + x[1] * sample.pixel_y + x[2];
        let predicted_y = y[0] * sample.pixel_x + y[1] * sample.pixel_y + y[2];
        squared_error += (predicted_x - sample.machine_x_mm).powi(2)
            + (predicted_y - sample.machine_y_mm).powi(2);
    }

    Ok(PixelToMachineCalibration {
        a: x[0],
        b: x[1],
        c: x[2],
        d: y[0],
        e: y[1],
        f: y[2],
        rms_error_mm: (squared_error / samples.len() as f64).sqrt(),
    })
}

fn solve_3x3(
    mut matrix: [[f64; 3]; 3],
    mut rhs: [f64; 3],
) -> std::result::Result<[f64; 3], String> {
    for column in 0..3 {
        let pivot = (column..3)
            .max_by(|&left, &right| {
                matrix[left][column]
                    .abs()
                    .total_cmp(&matrix[right][column].abs())
            })
            .unwrap();
        if matrix[pivot][column].abs() < 1e-12 {
            return Err("calibration positions are degenerate".into());
        }
        matrix.swap(column, pivot);
        rhs.swap(column, pivot);

        let divisor = matrix[column][column];
        for value in &mut matrix[column][column..] {
            *value /= divisor;
        }
        rhs[column] /= divisor;

        for row in 0..3 {
            if row == column {
                continue;
            }
            let factor = matrix[row][column];
            for index in column..3 {
                matrix[row][index] -= factor * matrix[column][index];
            }
            rhs[row] -= factor * rhs[column];
        }
    }
    Ok(rhs)
}

#[cfg(test)]
mod calibration_tests {
    use super::*;

    #[test]
    fn fits_rotated_affine_transform() {
        let samples = [
            sample(10.0, 20.0),
            sample(20.0, 20.0),
            sample(10.0, 30.0),
            sample(25.0, 35.0),
        ];
        let calibration = calibrate_pixel_to_machine(&samples).unwrap();
        let (x, y) = calibration.pixel_to_machine(90.0, 130.0);
        assert!((x - 10.0).abs() < 1e-9);
        assert!((y - 20.0).abs() < 1e-9);
        assert!(calibration.rms_error_mm < 1e-9);
    }

    #[test]
    fn rejects_collinear_samples() {
        let samples = [sample(10.0, 20.0), sample(20.0, 30.0), sample(30.0, 40.0)];
        assert!(calibrate_pixel_to_machine(&samples).is_err());
    }

    fn sample(machine_x_mm: f64, machine_y_mm: f64) -> CalibrationSample {
        // pixel_x = 2 * machine_x + machine_y + 50
        // pixel_y = -machine_x + 3 * machine_y + 80
        CalibrationSample {
            machine_x_mm,
            machine_y_mm,
            pixel_x: 2.0 * machine_x_mm + machine_y_mm + 50.0,
            pixel_y: -machine_x_mm + 3.0 * machine_y_mm + 80.0,
        }
    }
}

#[cfg(test)]
mod test_image_review {
    use super::*;
    use opencv::highgui;
    use std::path::{Path, PathBuf};

    fn expected_fiducial(file_name: &str) -> Option<(i32, i32)> {
        match file_name {
            "sample-00-x-225.000-y-195.000.png" => Some((519, 47)),
            "sample-01-x-225.000-y-205.000.png" => Some((523, 237)),
            "sample-02-x-225.000-y-215.000.png" => Some((523, 430)),
            "sample-03-x-235.000-y-195.000.png" => Some((365, 40)),
            "sample-04-x-235.000-y-205.000.png" => Some((366, 236)),
            "sample-05-x-235.000-y-215.000.png" => Some((367, 434)),
            "sample-06-x-245.000-y-195.000.png" => Some((205, 43)),
            "sample-07-x-245.000-y-205.000.png" => Some((205, 238)),
            "sample-08-x-245.000-y-215.000.png" => Some((206, 434)),
            "sample-02-x-228.000-y-219.000.png" => None,
            "sample-05-x-238.000-y-219.000.png" => None,
            "bot-black.jpg" => Some((419, 326)),
            "bot-green.jpg" => Some((459, 360)),
            "top-black.jpg" => Some((394, 325)),
            "top-green.jpg" => Some((398, 238)),
            // These are the detections for the copies checked into
            // test-images. The live frame-dumps are separate captures and
            // have their own manually recorded positions.
            "frame-000014-video0.jpg" => Some((160, 285)),
            "frame-000015-video0.jpg" => Some((479, 479)),
            "frame-000016-video0.jpg" => Some((322, 481)),
            "frame-000017-video0.jpg" => Some((161, 482)),
            "frame-000018-video0.jpg" => Some((322, 283)),
            "frame-000019-video0.jpg" => Some((160, 285)),
            "frame-000020-video0.jpg" => Some((344, 243)),
            "frame-000021-video0.jpg" => Some((322, 481)),
            "frame-000022-video0.jpg" => Some((164, 481)),
            _ => None,
        }
    }
    const PIXEL_TOLERANCE: i32 = 4;

    fn image_paths() -> Vec<PathBuf> {
        let directory = Path::new(env!("CARGO_MANIFEST_DIR")).join("test-images");
        let mut paths = std::fs::read_dir(directory)
            .expect("read test-images")
            .map(|entry| entry.expect("read image entry").path())
            .filter(|path| {
                matches!(
                    path.extension().and_then(|ext| ext.to_str()),
                    Some("png") | Some("jpg") | Some("jpeg")
                ) && path.file_name().and_then(|name| name.to_str()) != Some("center.png")
            })
            .collect::<Vec<_>>();
        paths.sort();
        paths
    }

    fn review_image_paths() -> Vec<PathBuf> {
        let mut paths = image_paths();
        paths.push(
            Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("test-images")
                .join("center.png"),
        );
        paths
    }

    #[test]
    fn detects_dark_center_fiducial() {
        let path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("test-images")
            .join("center.png");
        let image = imgcodecs::imread(path.to_str().unwrap(), imgcodecs::IMREAD_GRAYSCALE)
            .expect("read center image");
        let point = detect_fiducial_in_image(&image, 15.0, 80.0)
            .expect("detect center fiducial")
            .expect("center fiducial should be detected");
        let expected = Point::new(314, 276);
        assert!(
            (point.x - expected.x).abs() <= 5 && (point.y - expected.y).abs() <= 5,
            "center.png: detected ({}, {}), expected ({}, {}) +/- 5 px",
            point.x,
            point.y,
            expected.x,
            expected.y
        );
        println!("center.png: detected ({}, {})", point.x, point.y);
    }

    fn detect(path: &Path) -> Option<Point> {
        let name = path.file_name().and_then(|name| name.to_str());
        let expected = name
            .and_then(expected_fiducial)
            .map(|(x, y)| (x as f32, y as f32));
        let image = imgcodecs::imread(path.to_str().unwrap(), imgcodecs::IMREAD_GRAYSCALE)
            .expect("read fiducial image");
        detect_fiducial_in_image_with_box_near(&image, 8.0, 80.0, expected)
            .expect("detect fiducial")
            .map(|(point, _)| point)
    }

    #[test]
    fn report_test_images() {
        let paths = image_paths();
        assert_eq!(paths.len(), 24);
        for path in paths {
            let file_name = path.file_name().unwrap().to_str().unwrap();
            let detected = detect(&path);
            println!("{}: {:?}", path.display(), detected);
            if let Some((expected_x, expected_y)) = expected_fiducial(file_name) {
                let point = detected.expect("fiducial should be detected");
                let tolerance = if file_name.starts_with("frame-") {
                    8
                } else {
                    PIXEL_TOLERANCE
                };
                assert!(
                    (point.x - expected_x).abs() <= tolerance
                        && (point.y - expected_y).abs() <= tolerance,
                    "{}: detected ({}, {}), expected ({}, {}) +/- {} px",
                    path.display(),
                    point.x,
                    point.y,
                    expected_x,
                    expected_y,
                    tolerance
                );
            }
        }
    }

    #[test]
    #[ignore = "requires a graphical display; inspect each image and fill EXPECTED_FIDUCIALS"]
    fn review_test_images_with_highgui() {
        let window = "fiducial review - press any key for next image, q to quit";
        highgui::named_window(window, highgui::WINDOW_NORMAL).expect("create review window");
        for path in review_image_paths() {
            let image = imgcodecs::imread(path.to_str().unwrap(), imgcodecs::IMREAD_COLOR)
                .expect("read image");
            let detection =
                detect_fiducial_in_image_with_box(&image, 15.0, 80.0).expect("detect fiducial");
            let (point, refinement_box) = detection
                .map(|(point, rect)| (Some(point), Some(rect)))
                .unwrap_or((None, None));
            let debug = render_fiducial_debug_frame_with_box(&image, point, refinement_box)
                .expect("render debug");
            highgui::imshow(window, &debug).expect("show image");
            println!("{}: detected {:?}", path.display(), point);
            if highgui::wait_key(0).expect("wait for key") == 'q' as i32 {
                break;
            }
        }
        highgui::destroy_window(window).expect("destroy review window");
    }

    #[test]
    #[ignore = "requires a graphical display"]
    fn review_new_jpg_images_with_highgui() {
        let directory = Path::new(env!("CARGO_MANIFEST_DIR")).join("test-images");
        let mut paths = [
            "bot-black.jpg",
            "bot-green.jpg",
            "top-black.jpg",
            "top-green.jpg",
        ]
        .into_iter()
        .map(|name| directory.join(name))
        .collect::<Vec<_>>();
        paths.sort();

        let window = "new fiducial images - press any key for next image, q to quit";
        highgui::named_window(window, highgui::WINDOW_NORMAL).expect("create review window");
        for path in paths {
            let image = imgcodecs::imread(path.to_str().unwrap(), imgcodecs::IMREAD_COLOR)
                .expect("read image");
            let detection =
                detect_fiducial_in_image_with_box(&image, 15.0, 80.0).expect("detect fiducial");
            let (point, refinement_box) = detection
                .map(|(point, rect)| (Some(point), Some(rect)))
                .unwrap_or((None, None));
            let debug = render_fiducial_debug_frame_with_box(&image, point, refinement_box)
                .expect("render debug");
            highgui::imshow(window, &debug).expect("show image");
            println!("{}: detected {:?}", path.display(), point);
            if highgui::wait_key(0).expect("wait for key") == 'q' as i32 {
                break;
            }
        }
        highgui::destroy_window(window).expect("destroy review window");
    }
}

/// Detects fecucials from top down camera
pub fn detect_fiducial_with_diameter(
    image_path: &str,
    min_diameter: f32,
    max_diameter: f32,
) -> Result<Option<Point>> {
    let image = imgcodecs::imread(image_path, imgcodecs::IMREAD_GRAYSCALE)?;
    if image.empty() {
        return Err(opencv::Error::new(1, "Failed to load image"));
    }
    detect_fiducial_in_image(&image, min_diameter, max_diameter)
}

/// Detect a fiducial directly from an OpenCV image without filesystem I/O.
pub fn detect_fiducial_in_image(
    image: &Mat,
    min_diameter: f32,
    max_diameter: f32,
) -> Result<Option<Point>> {
    Ok(
        detect_fiducial_in_image_with_box(image, min_diameter, max_diameter)?
            .map(|(point, _)| point),
    )
}

fn detect_fiducial_in_image_coarse_near(
    image: &Mat,
    min_diameter: f32,
    max_diameter: f32,
    expected_position: Option<(f32, f32)>,
) -> Result<Option<Point>> {
    if image.empty() {
        return Err(opencv::Error::new(1, "Image is empty"));
    }
    if !(min_diameter.is_finite() && max_diameter.is_finite())
        || min_diameter <= 0.0
        || max_diameter < min_diameter
    {
        return Err(opencv::Error::new(1, "Invalid fiducial diameter range"));
    }

    let mut grayscale = Mat::default();
    if image.channels() > 1 {
        imgproc::cvt_color(
            image,
            &mut grayscale,
            imgproc::COLOR_BGR2GRAY,
            0,
            core::AlgorithmHint::ALGO_HINT_DEFAULT,
        )?;
    } else {
        grayscale = image.clone();
    }
    let mut blurred = Mat::default();
    imgproc::gaussian_blur(
        &grayscale,
        &mut blurred,
        core::Size::new(5, 5),
        0.0,
        0.0,
        core::BORDER_REPLICATE,
        core::AlgorithmHint::ALGO_HINT_DEFAULT,
    )?;
    let mut masks = Vec::new();
    for threshold in [0.0, 45.0, 85.0, 125.0, 165.0, 205.0] {
        for mode in [imgproc::THRESH_BINARY, imgproc::THRESH_BINARY_INV] {
            let mut mask = Mat::default();
            let threshold_mode = if threshold == 0.0 {
                mode | imgproc::THRESH_OTSU
            } else {
                mode
            };
            imgproc::threshold(&blurred, &mut mask, threshold, 255.0, threshold_mode)?;
            let kernel = imgproc::get_structuring_element(
                imgproc::MORPH_ELLIPSE,
                core::Size::new(3, 3),
                core::Point::new(-1, -1),
            )?;
            let mut cleaned = Mat::default();
            imgproc::morphology_ex(
                &mask,
                &mut cleaned,
                imgproc::MORPH_OPEN,
                &kernel,
                core::Point::new(-1, -1),
                1,
                core::BORDER_CONSTANT,
                Scalar::all(0.0),
            )?;
            // Bridge thin crosshair lines that cut through the fiducial so
            // the marker remains one candidate when its centre is covered.
            let close_kernel = imgproc::get_structuring_element(
                imgproc::MORPH_ELLIPSE,
                core::Size::new(5, 5),
                core::Point::new(-1, -1),
            )?;
            let mut closed = Mat::default();
            imgproc::morphology_ex(
                &cleaned,
                &mut closed,
                imgproc::MORPH_CLOSE,
                &close_kernel,
                core::Point::new(-1, -1),
                1,
                core::BORDER_CONSTANT,
                Scalar::all(0.0),
            )?;
            masks.push((closed, mode == imgproc::THRESH_BINARY_INV));
        }
    }

    let min_radius = min_diameter * 0.5;
    let max_radius = max_diameter * 0.5;
    let image_center = Point2f::new(image.cols() as f32 * 0.5, image.rows() as f32 * 0.5);
    let ranking_position = expected_position
        .map(|(x, y)| Point2f::new(x, y))
        .unwrap_or(image_center);
    let mut best_bright: Option<(FiducialDetection, f32, f32)> = None;
    let mut best_dark: Option<(FiducialDetection, f32, f32)> = None;
    let mut candidates = Vec::new();
    for (mask, _) in masks {
        let mut contours = Vector::<Vector<core::Point>>::new();
        imgproc::find_contours(
            &mask,
            &mut contours,
            imgproc::RETR_EXTERNAL,
            imgproc::CHAIN_APPROX_NONE,
            core::Point::new(0, 0),
        )?;
        for index in 0..contours.len() {
            let contour = contours.get(index)?;
            if contour.len() < 8 {
                continue;
            }
            let area = contour_area(&contour, false)?.abs() as f32;
            let perimeter = arc_length(&contour, true)? as f32;
            if area <= 0.0 || perimeter <= 0.0 {
                continue;
            }
            let mut center = core::Point2f::default();
            let mut radius = 0.0_f32;
            min_enclosing_circle(&contour, &mut center, &mut radius)?;
            if radius < min_radius || radius > max_radius {
                continue;
            }
            let rect = bounding_rect(&contour)?;
            let aspect = rect.width.min(rect.height) as f32 / rect.width.max(rect.height) as f32;
            let circularity = 4.0 * std::f32::consts::PI * area / (perimeter * perimeter);
            let fill = area / (std::f32::consts::PI * radius * radius);
            if aspect < 0.72 || circularity < 0.42 || fill < 0.48 {
                continue;
            }
            let x = center.x.round() as i32;
            let y = center.y.round() as i32;
            let margin = (radius * 1.5).ceil() as i32;
            if x < margin || y < margin || x >= image.cols() - margin || y >= image.rows() - margin
            {
                continue;
            }

            // Compare the candidate disk with a surrounding annulus. This
            // rejects isolated threshold blobs in textured or uniformly lit
            // areas while requiring a bright marker on a darker background.
            let inner = mean_disk(&blurred, x, y, radius * 0.65)?;
            let outer = mean_annulus(&blurred, x, y, radius * 1.15, radius * 1.8)?;
            let contrast = (inner - outer).abs() as f32;
            if contrast < 12.0 {
                continue;
            }
            let polarity = if inner > outer { -1 } else { 1 };
            // Shape is more stable than absolute brightness across the JPG
            // lighting variants, so contrast is deliberately a small tie
            // breaker rather than the dominant score component.
            let score = circularity * 3.0 + aspect + fill + (contrast / 512.0).min(0.5);
            let detection = FiducialDetection {
                center: Point2f::new(center.x, center.y),
                radius_px: radius,
                polarity,
                confidence: score,
            };
            if expected_position.is_some() {
                candidates.push(detection);
                continue;
            }
            if std::env::var_os("VISION_DEBUG_CANDIDATES").is_some() {
                eprintln!(
                    "candidate ({}, {}) radius={:.2} score={:.3} contrast={:.2} aspect={:.3} circularity={:.3} fill={:.3}",
                    x, y, radius, score, contrast, aspect, circularity, fill
                );
            }
            let dx = detection.center.x - ranking_position.x;
            let dy = detection.center.y - ranking_position.y;
            let distance_squared = dx * dx + dy * dy;
            // Do not prefer the blob nearest the optical centre. During
            // calibration the real fiducial intentionally moves through the
            // image, so that rule can randomly switch to another circular
            // feature. Confidence is the primary discriminator; distance is
            // only a deterministic tie-breaker.
            let best = if polarity < 0 {
                &mut best_bright
            } else {
                &mut best_dark
            };
            let better = best
                .as_ref()
                .map_or(true, |(old_detection, old_distance, old_score)| {
                    if expected_position.is_some()
                        && score + 0.35 >= *old_score
                        && detection.radius_px >= old_detection.radius_px * 0.75
                    {
                        distance_squared < *old_distance
                    } else {
                        score > *old_score
                            || (score == *old_score && distance_squared < *old_distance)
                    }
                });
            if better {
                *best = Some((detection, distance_squared, score));
            }
        }
    }
    let selected = if let Some((expected_x, expected_y)) = expected_position {
        candidates
            .into_iter()
            .min_by(|left, right| {
                let left_distance =
                    (left.center.x - expected_x).powi(2) + (left.center.y - expected_y).powi(2);
                let right_distance =
                    (right.center.x - expected_x).powi(2) + (right.center.y - expected_y).powi(2);
                left_distance
                    .total_cmp(&right_distance)
                    .then_with(|| right.confidence.total_cmp(&left.confidence))
            })
            .map(|detection| (detection, 0.0, 0.0))
    } else {
        best_bright
    };
    if let Some((detection, _, _)) = selected {
        return Ok(Some(Point::new(
            detection.center.x.round() as i32,
            detection.center.y.round() as i32,
        )));
    }
    // A bright-ring/dark-centre fiducial can split into separate contours.
    // Hough circles examines the edge structure directly, so use it only
    // when contour candidates found nothing. Its radius comes from the
    // caller's configured diameter range.
    let mut circles = Vector::<core::Vec3f>::new();
    imgproc::hough_circles(
        &blurred,
        &mut circles,
        imgproc::HOUGH_GRADIENT,
        1.0,
        min_diameter as f64,
        100.0,
        12.0,
        (min_diameter * 0.5).floor() as i32,
        (max_diameter * 0.5).ceil() as i32,
    )?;
    let circle_candidates = circles
        .iter()
        .filter_map(|circle| {
            let center_x = circle[0].round() as i32;
            let center_y = circle[1].round() as i32;
            let radius = circle[2];
            let center = mean_disk(&blurred, center_x, center_y, radius * 0.3).ok()?;
            let ring =
                mean_annulus(&blurred, center_x, center_y, radius * 0.45, radius * 0.9).ok()?;
            let outer =
                mean_annulus(&blurred, center_x, center_y, radius * 1.2, radius * 1.8).ok()?;
            let ring_score = (ring - center).max(0.0) + (ring - outer).max(0.0);
            if ring_score < 18.0 {
                return None;
            }
            Some((circle, ring_score as f32))
        })
        .collect::<Vec<_>>();
    let image_center = Point2f::new(image.cols() as f32 * 0.5, image.rows() as f32 * 0.5);
    let best_circle =
        circle_candidates
            .iter()
            .min_by(|(left, left_score), (right, right_score)| {
                let left_distance =
                    (left[0] - image_center.x).powi(2) + (left[1] - image_center.y).powi(2);
                let right_distance =
                    (right[0] - image_center.x).powi(2) + (right[1] - image_center.y).powi(2);
                left_distance
                    .total_cmp(&right_distance)
                    .then_with(|| right_score.total_cmp(left_score))
            });
    let Some((circle, _)) = best_circle else {
        return Ok(None);
    };
    let radius = circle[2];
    let mut best_point = Point::new(circle[0].round() as i32, circle[1].round() as i32);
    let mut best_score = f32::MIN;
    let search = (radius * 0.5).ceil() as i32;
    for y in (best_point.y - search)..=(best_point.y + search) {
        for x in (best_point.x - search)..=(best_point.x + search) {
            let inner = mean_disk(&blurred, x, y, radius * 0.65)?;
            let outer = mean_annulus(&blurred, x, y, radius * 1.15, radius * 1.8)?;
            let score = (outer - inner) as f32;
            if score > best_score {
                best_score = score;
                best_point = Point::new(x, y);
            }
        }
    }
    // Refine the ring centre from the bright-ring pixels. This removes the
    // small bias introduced by the Hough circle's quantized centre.
    let mut weighted_x = 0.0;
    let mut weighted_y = 0.0;
    let mut weight_total = 0.0;
    let radius = radius.max(1.0);
    for y in (best_point.y - search)..=(best_point.y + search) {
        for x in (best_point.x - search)..=(best_point.x + search) {
            let inner = mean_disk(&blurred, x, y, radius * 0.3)?;
            let ring = mean_annulus(&blurred, x, y, radius * 0.45, radius * 0.9)?;
            let weight = (ring - inner).max(0.0);
            weighted_x += x as f64 * weight;
            weighted_y += y as f64 * weight;
            weight_total += weight;
        }
    }
    if weight_total > 0.0 {
        best_point = Point::new(
            (weighted_x / weight_total).round() as i32,
            (weighted_y / weight_total).round() as i32,
        );
    }

    // For the alternate dark-centre marker, use the centre of the dark core
    // as a second refinement. The Hough result follows the outer ring, which
    // can be biased when the ring is unevenly illuminated.
    let outer_level = mean_annulus(
        &blurred,
        best_point.x,
        best_point.y,
        radius * 1.15,
        radius * 1.8,
    )?;
    let core_radius = (radius * 0.65).ceil() as i32;
    let mut dark_x = 0.0;
    let mut dark_y = 0.0;
    let mut dark_weight = 0.0;
    for y in (best_point.y - core_radius)..=(best_point.y + core_radius) {
        for x in (best_point.x - core_radius)..=(best_point.x + core_radius) {
            let dx = x - best_point.x;
            let dy = y - best_point.y;
            if (dx * dx + dy * dy) as f32 > (radius * 0.65).powi(2) {
                continue;
            }
            let pixel = *blurred.at_2d::<u8>(y, x)? as f64;
            let weight = (outer_level - pixel).max(0.0);
            dark_x += x as f64 * weight;
            dark_y += y as f64 * weight;
            dark_weight += weight;
        }
    }
    if dark_weight > 0.0 {
        best_point = Point::new(
            (dark_x / dark_weight).round() as i32,
            (dark_y / dark_weight).round() as i32,
        );
    }
    Ok(Some(best_point))
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FiducialDetection {
    pub center: Point2f,
    pub radius_px: f32,
    pub polarity: i32,
    pub confidence: f32,
}

fn mean_disk(image: &Mat, center_x: i32, center_y: i32, radius: f32) -> Result<f64> {
    let mut mask = Mat::new_rows_cols_with_default(
        image.rows(),
        image.cols(),
        core::CV_8UC1,
        Scalar::all(0.0),
    )?;
    imgproc::circle(
        &mut mask,
        Point::new(center_x, center_y),
        radius.max(1.0).ceil() as i32,
        Scalar::all(255.0),
        -1,
        imgproc::LINE_8,
        0,
    )?;
    Ok(core::mean(image, &mask)?.0[0])
}

fn mean_annulus(
    image: &Mat,
    center_x: i32,
    center_y: i32,
    inner_radius: f32,
    outer_radius: f32,
) -> Result<f64> {
    let mut mask = Mat::new_rows_cols_with_default(
        image.rows(),
        image.cols(),
        core::CV_8UC1,
        Scalar::all(0.0),
    )?;
    let center = Point::new(center_x, center_y);
    imgproc::circle(
        &mut mask,
        center,
        outer_radius.max(1.0).ceil() as i32,
        Scalar::all(255.0),
        -1,
        imgproc::LINE_8,
        0,
    )?;
    imgproc::circle(
        &mut mask,
        center,
        inner_radius.max(0.0).floor() as i32,
        Scalar::all(0.0),
        -1,
        imgproc::LINE_8,
        0,
    )?;
    Ok(core::mean(image, &mask)?.0[0])
}

/// Render a review image showing the detected fiducial and its measurements.
pub fn render_fiducial_debug(
    image_path: &str,
    detection: Option<FiducialDetection>,
) -> Result<Mat> {
    let source = imgcodecs::imread(image_path, imgcodecs::IMREAD_COLOR)?;
    if source.empty() {
        return Err(opencv::Error::new(1, "Failed to load image"));
    }
    let mut debug = source.clone();
    if let Some(detection) = detection {
        let center = core::Point::new(
            detection.center.x.round() as i32,
            detection.center.y.round() as i32,
        );
        let radius = detection.radius_px.round().max(1.0) as i32;
        imgproc::circle(
            &mut debug,
            center,
            (detection.radius_px * 1.8).round().max(2.0) as i32,
            Scalar::new(0.0, 180.0, 255.0, 0.0),
            1,
            imgproc::LINE_AA,
            0,
        )?;
        imgproc::circle(
            &mut debug,
            center,
            radius,
            Scalar::new(0.0, 255.0, 0.0, 0.0),
            2,
            imgproc::LINE_AA,
            0,
        )?;
        imgproc::draw_marker(
            &mut debug,
            center,
            Scalar::new(0.0, 0.0, 255.0, 0.0),
            imgproc::MARKER_CROSS,
            (radius * 2).max(12),
            1,
            imgproc::LINE_AA,
        )?;
        let polarity = if detection.polarity > 0 {
            "dark-on-light"
        } else {
            "light-on-dark"
        };
        draw_fiducial_labels(
            &mut debug,
            &[
                format!(
                    "FIDUCIAL center=({:.2}, {:.2})",
                    detection.center.x, detection.center.y
                ),
                format!(
                    "radius={:.2}px diameter={:.2}px",
                    detection.radius_px,
                    detection.radius_px * 2.0
                ),
                format!(
                    "polarity={} confidence={:.2}",
                    polarity, detection.confidence
                ),
            ],
        )?;
    } else {
        draw_fiducial_labels(&mut debug, &["FIDUCIAL: no detection".to_owned()])?;
    }
    Ok(debug)
}

fn draw_fiducial_labels(image: &mut Mat, labels: &[String]) -> Result<()> {
    let line_height = 22;
    let panel_height = labels.len() as i32 * line_height + 12;
    imgproc::rectangle(
        image,
        core::Rect::new(8, 8, (image.cols() - 16).min(560).max(1), panel_height),
        Scalar::all(0.0),
        -1,
        imgproc::LINE_8,
        0,
    )?;
    for (index, label) in labels.iter().enumerate() {
        imgproc::put_text(
            image,
            label,
            core::Point::new(16, 29 + index as i32 * line_height),
            imgproc::FONT_HERSHEY_SIMPLEX,
            0.55,
            Scalar::all(255.0),
            1,
            imgproc::LINE_AA,
            false,
        )?;
    }
    Ok(())
}
