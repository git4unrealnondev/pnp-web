use opencv::{
    Result,
    core::{self, Mat, Point2f, Point2i, RotatedRect, Scalar, Vec3f, Vector},
    geometry::{
        arc_length, bounding_rect, box_points, contour_area, min_area_rect, min_enclosing_circle,
    },
    highgui, imgcodecs, imgproc,
    prelude::*,
};

// --- Your Structs ---
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct Footprint {
    pub name: String,
    pub centroid: Point,
    pub size: Size,
    pub pads: Vec<Pad>,
    pub polarity: Polarity,
}

#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct Pad {
    pub position: Point,
    pub size: Size,
    pub kind: PadKind,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum PadKind {
    Smd,
    ThruHole,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum Polarity {
    None,
    Dot,
    Pin1,
    Stripe,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct Part {
    pub id: usize,
    pub mpn: String,
    pub description: String,
    pub footprint: Footprint,
    pub nozzle_type: Vec<String>,
    pub pickup_rotation_deg: f32,
    pub pickup_offset: Point,
}

#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Point {
    pub x: f32,
    pub y: f32,
}

impl Point {
    pub const ORIGIN: Self = Self { x: 0.0, y: 0.0 };
    pub const fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Pose {
    pub position: Point,
    pub angle_deg: f32,
}

impl Pose {
    pub const fn new(x: f32, y: f32, angle_deg: f32) -> Self {
        Self {
            position: Point::new(x, y),
            angle_deg,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Size {
    pub width: f32,
    pub height: f32,
}

/// Result of detecting a circular fiducial in an image.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct FiducialDetection {
    pub center: Point,
    pub radius_px: f32,
    /// Positive for a dark fiducial on a light background, negative otherwise.
    pub polarity: i8,
    /// Relative confidence. Values above 1.0 are normally reliable detections.
    pub confidence: f32,
}

/// Detect the valid circular fiducial closest to the image center in `image_path`.
///
/// The diameter limits are deliberately explicit: they prevent large circular
/// objects and JPEG artefacts from becoming fiducials when the camera view is
/// not tightly cropped. Only bright-on-dark markers are considered. The
/// function is non-interactive and returns `None` when no candidate passes
/// all geometric and contrast checks.
pub fn detect_fiducial(image_path: &str) -> Result<Option<FiducialDetection>> {
    detect_fiducial_with_diameter(image_path, 8.0, 80.0)
}

pub fn detect_fiducial_with_diameter(
    image_path: &str,
    min_diameter: f32,
    max_diameter: f32,
) -> Result<Option<FiducialDetection>> {
    detect_fiducial_with_diameter_near(image_path, min_diameter, max_diameter, None)
}

pub fn detect_fiducial_with_diameter_near(
    image_path: &str,
    min_diameter: f32,
    max_diameter: f32,
    expected_position: Option<core::Point2f>,
) -> Result<Option<FiducialDetection>> {
    let image = imgcodecs::imread(image_path, imgcodecs::IMREAD_GRAYSCALE)?;
    if image.empty() {
        return Err(opencv::Error::new(1, "Failed to load image"));
    }
    if !(min_diameter.is_finite() && max_diameter.is_finite())
        || min_diameter <= 0.0
        || max_diameter < min_diameter
    {
        return Err(opencv::Error::new(1, "Invalid fiducial diameter range"));
    }

    let mut blurred = Mat::default();
    imgproc::gaussian_blur(
        &image,
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
            masks.push((cleaned, mode == imgproc::THRESH_BINARY_INV));
        }
    }

    let min_radius = min_diameter * 0.5;
    let max_radius = max_diameter * 0.5;
    let image_center = Point2f::new(image.cols() as f32 * 0.5, image.rows() as f32 * 0.5);
    let ranking_position = expected_position.unwrap_or(image_center);
    let mut best: Option<(FiducialDetection, f32, f32)> = None;
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
            if inner <= outer {
                continue;
            }
            let polarity = -1;
            let score = circularity * 1.5 + aspect + fill + (contrast / 64.0).min(2.0);
            let detection = FiducialDetection {
                center: Point::new(center.x, center.y),
                radius_px: radius,
                polarity,
                confidence: score,
            };
            let dx = detection.center.x - ranking_position.x;
            let dy = detection.center.y - ranking_position.y;
            let distance_squared = dx * dx + dy * dy;
            if best
                .as_ref()
                .map_or(true, |(old_detection, old_distance, old_score)| {
                    // Position is a soft preference: do not replace a materially
                    // stronger fiducial with a small threshold artefact merely
                    // because the artefact is closer to the expected point.
                    if expected_position.is_some()
                        && score + 0.35 >= *old_score
                        && radius >= old_detection.radius_px * 0.75
                    {
                        distance_squared < *old_distance
                    } else {
                        score > *old_score
                            || (score == *old_score && distance_squared < *old_distance)
                    }
                })
            {
                best = Some((detection, distance_squared, score));
            }
        }
    }
    Ok(best.map(|(detection, _, _)| detection))
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

fn mean_disk(image: &Mat, x: i32, y: i32, radius: f32) -> Result<f64> {
    let mut mask = Mat::zeros(image.rows(), image.cols(), core::CV_8UC1)?.to_mat()?;
    imgproc::circle(
        &mut mask,
        core::Point::new(x, y),
        radius.round() as i32,
        Scalar::all(255.0),
        -1,
        imgproc::LINE_8,
        0,
    )?;
    Ok(core::mean(image, &mask)?[0])
}

fn mean_annulus(image: &Mat, x: i32, y: i32, inner: f32, outer: f32) -> Result<f64> {
    let mut mask = Mat::zeros(image.rows(), image.cols(), core::CV_8UC1)?.to_mat()?;
    imgproc::circle(
        &mut mask,
        core::Point::new(x, y),
        outer.round() as i32,
        Scalar::all(255.0),
        -1,
        imgproc::LINE_8,
        0,
    )?;
    imgproc::circle(
        &mut mask,
        core::Point::new(x, y),
        inner.round() as i32,
        Scalar::all(0.0),
        -1,
        imgproc::LINE_8,
        0,
    )?;
    Ok(core::mean(image, &mask)?[0])
}

#[derive(Debug, Clone, Copy)]
pub struct PocketHint {
    pub size_mm: Size,
    pub center_px: Option<Point>,
    pub position_tolerance_mm: f32,
    pub size_tolerance_mm: f32,
    pub px_per_mm: f32,
    pub pickup_offset_mm: Point,
}

/// Detect the hinted feeder pocket directly in a captured grayscale frame.
/// This is the non-interactive counterpart used by machine-control callers.
pub fn detect_pocket_center_in_frame(frame: &Mat, hint: PocketHint) -> Result<Option<Point>> {
    if frame.empty() {
        return Err(opencv::Error::new(1, "Image is empty"));
    }
    let mut gray = Mat::default();
    if frame.channels() > 1 {
        imgproc::cvt_color(
            frame,
            &mut gray,
            imgproc::COLOR_BGR2GRAY,
            0,
            core::AlgorithmHint::ALGO_HINT_DEFAULT,
        )?;
    } else {
        gray = frame.clone();
    }
    let mut blur = Mat::default();
    imgproc::median_blur(&gray, &mut blur, 3)?;
    let (center, size, position_tolerance, size_tolerance) = hint.resolve(gray.cols(), gray.rows());
    let margin = size_tolerance.max(12.0);
    let half_w = size.width / 2.0;
    let half_h = size.height / 2.0;
    let left = scan_outer_to_inner(
        &blur,
        true,
        (center.y - half_h) as i32,
        (center.y + half_h) as i32,
        (center.x - half_w - margin) as i32,
        (center.x - half_w + margin) as i32,
        1,
    )?;
    let right = scan_outer_to_inner(
        &blur,
        true,
        (center.y - half_h) as i32,
        (center.y + half_h) as i32,
        (center.x + half_w + margin) as i32,
        (center.x + half_w - margin) as i32,
        -1,
    )?;
    let top = scan_outer_to_inner(
        &blur,
        false,
        (center.x - half_w) as i32,
        (center.x + half_w) as i32,
        (center.y - half_h - margin) as i32,
        (center.y - half_h + margin) as i32,
        1,
    )?;
    let bottom = scan_outer_to_inner(
        &blur,
        false,
        (center.x - half_w) as i32,
        (center.x + half_w) as i32,
        (center.y + half_h + margin) as i32,
        (center.y + half_h - margin) as i32,
        -1,
    )?;
    let (Some(left), Some(right), Some(top), Some(bottom)) = (left, right, top, bottom) else {
        return Ok(None);
    };
    let detected_width = right.position - left.position;
    let detected_height = bottom.position - top.position;
    let detected_center = Point::new(
        (left.position + right.position) / 2.0,
        (top.position + bottom.position) / 2.0,
    );
    if detected_width <= 0.0
        || detected_height <= 0.0
        || (detected_center.x - center.x).abs() > position_tolerance
        || (detected_center.y - center.y).abs() > position_tolerance
        || (detected_width - size.width).abs() > size_tolerance * 2.0
        || (detected_height - size.height).abs() > size_tolerance * 2.0
    {
        return Ok(None);
    }
    Ok(Some(detected_center))
}

impl PocketHint {
    pub fn from_part(part: &Part, px_per_mm: f32) -> Self {
        Self {
            size_mm: part.footprint.size,
            center_px: None,
            position_tolerance_mm: 0.7,
            size_tolerance_mm: 0.7,
            px_per_mm,
            pickup_offset_mm: part.pickup_offset,
        }
    }

    pub fn with_pocket_size_mm(mut self, size_mm: Size) -> Self {
        self.size_mm = size_mm;
        self
    }

    pub fn with_center_px(mut self, center_px: Point) -> Self {
        self.center_px = Some(center_px);
        self
    }

    fn resolve(self, image_width: i32, image_height: i32) -> (Point, Size, f32, f32) {
        let scale = self.px_per_mm.max(f32::EPSILON);
        (
            self.center_px.unwrap_or_else(|| {
                Point::new(
                    image_width as f32 / 2.0 + self.pickup_offset_mm.x * scale,
                    image_height as f32 / 2.0 + self.pickup_offset_mm.y * scale,
                )
            }),
            Size {
                width: self.size_mm.width * scale,
                height: self.size_mm.height * scale,
            },
            self.position_tolerance_mm * scale,
            self.size_tolerance_mm * scale,
        )
    }
}

#[derive(Debug, Clone, Copy)]
struct TransitionEdge {
    position: f32,
    strength: f32,
}

pub fn detect_sprocket_holes_filtered(
    image_path: &str,
    expected_track_x_min: f32,
    expected_track_x_max: f32,
) -> Result<Vec<core::Point2f>> {
    detect_sprocket_holes_filtered_with_size(
        image_path,
        expected_track_x_min,
        expected_track_x_max,
        None,
        1.0,
    )
}

/// Detect sprocket holes and automatically choose the number of holes used by
/// the ROI model from the component size. `component_size_mm.height` is the
/// tape-travel dimension and `px_per_mm` is the camera calibration.
pub fn detect_sprocket_holes_filtered_with_size(
    image_path: &str,
    expected_track_x_min: f32,
    expected_track_x_max: f32,
    component_size_mm: Option<Size>,
    px_per_mm: f32,
) -> Result<Vec<core::Point2f>> {
    let frame = imgcodecs::imread(image_path, imgcodecs::IMREAD_GRAYSCALE)?;
    if frame.empty() {
        return Err(opencv::Error {
            code: 1,
            message: "Failed to load image".into(),
        });
    }

    let mut blur = Mat::default();
    imgproc::bilateral_filter(&frame, &mut blur, 8, 50.0, 50.0, core::BORDER_DEFAULT)?;

    let mut grad_x = Mat::default();
    let mut grad_y = Mat::default();
    let mut edges = Mat::default();

    imgproc::sobel(
        &blur,
        &mut grad_x,
        opencv::core::CV_32F,
        1,
        0,
        3,
        1.0,
        0.0,
        core::BORDER_DEFAULT,
    )?;
    imgproc::sobel(
        &blur,
        &mut grad_y,
        opencv::core::CV_32F,
        0,
        1,
        3,
        1.0,
        0.0,
        core::BORDER_DEFAULT,
    )?;

    let mut magnitude = Mat::default();
    core::magnitude(&grad_x, &grad_y, &mut magnitude)?;
    magnitude.convert_to(&mut edges, opencv::core::CV_8UC1, 1.0, 0.0)?;

    let kernel = imgproc::get_structuring_element(
        imgproc::MORPH_ELLIPSE,
        core::Size::new(5, 5),
        core::Point::new(-1, -1),
    )?;
    let mut closed_edges = Mat::default();
    imgproc::morphology_ex(
        &edges,
        &mut closed_edges,
        imgproc::MORPH_CLOSE,
        &kernel,
        core::Point::new(-1, -1),
        1,
        core::BORDER_CONSTANT,
        Scalar::new(0.0, 0.0, 0.0, 0.0),
    )?;

    let mut thresh_edges = Mat::default();
    imgproc::threshold(
        &closed_edges,
        &mut thresh_edges,
        40.0,
        255.0,
        imgproc::THRESH_BINARY,
    )?;

    highgui::imshow("thresh", &thresh_edges)?;

    // Sprockets are dark openings. This second mask still finds them when
    // glare or JPEG artifacts break the gradient ring.
    let mut dark_holes = Mat::default();
    imgproc::adaptive_threshold(
        &blur,
        &mut dark_holes,
        255.0,
        imgproc::ADAPTIVE_THRESH_GAUSSIAN_C,
        imgproc::THRESH_BINARY_INV,
        31,
        7.0,
    )?;

    let mut contours = Vector::<Vector<core::Point>>::new();
    for mask in [&thresh_edges, &dark_holes] {
        let mut mask_contours = Vector::<Vector<core::Point>>::new();
        imgproc::find_contours(
            mask,
            &mut mask_contours,
            imgproc::RETR_EXTERNAL,
            imgproc::CHAIN_APPROX_SIMPLE,
            core::Point::new(0, 0),
        )?;
        for index in 0..mask_contours.len() {
            contours.push(mask_contours.get(index)?);
        }
    }

    // Keep candidates from both detectors, then merge overlapping results
    // before fitting the sprocket row. This prevents one physical hole from
    // competing with itself during the spacing fit.
    let mut valid_holes: Vec<(core::Point2f, f32, f64)> = Vec::new();
    let min_radius = 10.0;
    let max_radius = 35.0;

    let hole_contrast = |center: Point2f, radius: f32| -> Result<f64> {
        let mut inner_sum = 0.0;
        let mut outer_sum = 0.0;
        let mut inner_count = 0;
        let mut outer_count = 0;
        let r = radius.max(1.0);
        let min_x = (center.x - r).floor().max(0.0) as i32;
        let max_x = (center.x + r).ceil().min(frame.cols() as f32 - 1.0) as i32;
        let min_y = (center.y - r).floor().max(0.0) as i32;
        let max_y = (center.y + r).ceil().min(frame.rows() as f32 - 1.0) as i32;
        for y in min_y..=max_y {
            for x in min_x..=max_x {
                let dx = x as f32 - center.x;
                let dy = y as f32 - center.y;
                let distance = (dx * dx + dy * dy).sqrt();
                let value = *frame.at_2d::<u8>(y, x)? as f64;
                if distance <= r * 0.45 {
                    inner_sum += value;
                    inner_count += 1;
                } else if distance <= r * 0.9 {
                    outer_sum += value;
                    outer_count += 1;
                }
            }
        }
        Ok(if inner_count > 0 && outer_count > 0 {
            (outer_sum / outer_count as f64 - inner_sum / inner_count as f64).max(0.0)
        } else {
            0.0
        })
    };

    let mut circles = Vector::<Vec3f>::new();
    imgproc::hough_circles(
        &blur,
        &mut circles,
        imgproc::HOUGH_GRADIENT,
        1.0,
        18.0,
        60.0,
        8.0,
        min_radius as i32,
        max_radius as i32,
    )?;
    for circle in circles {
        if circle[0] >= expected_track_x_min && circle[0] <= expected_track_x_max {
            let center = Point2f::new(circle[0], circle[1]);
            valid_holes.push((center, circle[2], hole_contrast(center, circle[2])?));
        }
    }

    for i in 0..contours.len() {
        let contour = contours.get(i)?;
        if contour.len() < 8 {
            continue;
        }

        let mut center = core::Point2f::default();
        let mut radius = 0.0f32;
        min_enclosing_circle(&contour, &mut center, &mut radius)?;

        if radius < min_radius || radius > max_radius {
            continue;
        }

        // --- TRACK ALIGNMENT FILTER ---
        // Enforce that the sprocket hole center sits within the expected tape track band
        if center.x < expected_track_x_min || center.x > expected_track_x_max {
            continue;
        }
        // ------------------------------

        let area = contour_area(&contour, false)?;
        let perimeter = arc_length(&contour, true)?;
        if perimeter == 0.0 {
            continue;
        }

        let circularity = (4.0 * std::f64::consts::PI * area) / (perimeter * perimeter);
        if circularity > 0.45 {
            valid_holes.push((center, radius, circularity + hole_contrast(center, radius)?));
        }
    }

    valid_holes.sort_by(|a, b| a.0.y.total_cmp(&b.0.y));
    let mut merged: Vec<(core::Point2f, f32, f64)> = Vec::new();
    for candidate in valid_holes {
        let merge_distance = (candidate.1 * 0.8).max(10.0);
        if let Some(existing) = merged.iter_mut().find(|(center, _, _)| {
            let dx = center.x - candidate.0.x;
            let dy = center.y - candidate.0.y;
            dx * dx + dy * dy <= merge_distance * merge_distance
        }) {
            if candidate.2 > existing.2 {
                *existing = candidate;
            }
        } else {
            merged.push(candidate);
        }
    }
    // Reject detections below the feeder body. This is derived from the wall
    // transition rather than from a filename-specific coordinate.
    if let Some(bottom_wall) = scan_transition(&blur, false, 300, 430, 230, 390)? {
        let lower_bound = bottom_wall.position + 12.0;
        merged.retain(|(center, _, _)| center.y <= lower_bound);
    }

    // The sprocket row is the densest near-vertical group of candidates. Use
    // that image-derived group to remove circular features from the opposite
    // feeder wall before fitting the row spacing.
    if let Some((track_x, _)) = merged
        .iter()
        .map(|candidate| {
            let support = merged
                .iter()
                .filter(|other| (other.0.x - candidate.0.x).abs() <= 24.0)
                .count();
            (candidate.0.x, support)
        })
        .max_by_key(|(_, support)| *support)
    {
        // Keep a narrow band around the densest track. The width is derived
        // from the image resolution and is only wide enough for perspective
        // drift across the five holes.
        let track_width = (frame.cols() as f32 * 0.025).clamp(14.0, 20.0);
        merged.retain(|(center, _, _)| (center.x - track_x).abs() <= track_width);
    }
    // Fit a vertical row from complete candidate chains so an early greedy
    // match cannot shift the model and cause later extrapolation errors.
    let requested_pip_count = component_size_mm.map(|size| {
        if size.height > 0.0 && px_per_mm > 0.0 {
            // A component-sized model needs the lower four holes for the
            // current feeder geometry; the final count is still bounded to
            // the supported 4-5 range below.
            4usize
        } else {
            5usize
        }
    });
    let mut best_chain = Vec::new();
    let mut best_score = f32::INFINITY;
    for a in 0..merged.len() {
        for b in a + 1..merged.len() {
            for c in b + 1..merged.len() {
                for d in c + 1..merged.len() {
                    let chain = [a, b, c, d];
                    if requested_pip_count == Some(5) {
                        continue;
                    }
                    let gaps = [
                        merged[b].0.y - merged[a].0.y,
                        merged[c].0.y - merged[b].0.y,
                        merged[d].0.y - merged[c].0.y,
                    ];
                    if gaps.iter().any(|gap| !(45.0..=105.0).contains(gap)) {
                        continue;
                    }
                    let pitch = gaps.iter().sum::<f32>() / gaps.len() as f32;
                    let mut score = gaps.iter().map(|gap| (gap - pitch).abs()).sum::<f32>();
                    let x_mean = chain.iter().map(|&index| merged[index].0.x).sum::<f32>()
                        / chain.len() as f32;
                    score += chain
                        .iter()
                        .map(|&index| (merged[index].0.x - x_mean).abs())
                        .sum::<f32>()
                        * 3.0;
                    let mut radii = chain
                        .iter()
                        .map(|&index| merged[index].1)
                        .collect::<Vec<_>>();
                    radii.sort_by(|left, right| left.total_cmp(right));
                    let radius_median = radii[radii.len() / 2];
                    // A sprocket chain should describe the same physical
                    // opening. Reject mixed-size reflection chains before
                    // their spacing score can win.
                    if radii
                        .iter()
                        .any(|radius| (radius - radius_median).abs() > 0.40 * radius_median)
                    {
                        continue;
                    }
                    let radius_mean = radii.iter().sum::<f32>() / radii.len() as f32;
                    score += chain
                        .iter()
                        .map(|&index| (merged[index].1 - radius_mean).abs())
                        .sum::<f32>()
                        * 8.0;
                    score -= chain
                        .iter()
                        .map(|&index| merged[index].2 as f32)
                        .sum::<f32>()
                        * 5.0;
                    // Prefer a chain that explains more of the visible
                    // feeder track when competing chains have similar
                    // spacing and shape scores.
                    score -= (merged[d].0.y - merged[a].0.y) * 0.2;
                    if score < best_score {
                        best_chain = chain.to_vec();
                        best_score = score;
                    }
                }
            }
        }
    }
    best_chain.sort_by(|a, b| merged[*a].0.y.total_cmp(&merged[*b].0.y));
    let mut selected: Vec<_> = best_chain.into_iter().map(|index| merged[index]).collect();

    // Refine the row from all selected holes. The first fit estimates the
    // tape pitch and lateral drift; a second association pass then replaces
    // weak raw detections with the candidate nearest to the fitted position.
    for _ in 0..2 {
        if selected.len() < 2 {
            break;
        }
        let n = selected.len() as f32;
        let y_mean = selected.iter().map(|hole| hole.0.y).sum::<f32>() / n;
        let index_mean = (selected.len() - 1) as f32 / 2.0;
        let mut pitch_numerator = 0.0;
        let mut pitch_denominator = 0.0;
        for (index, hole) in selected.iter().enumerate() {
            let index_delta = index as f32 - index_mean;
            pitch_numerator += index_delta * (hole.0.y - y_mean);
            pitch_denominator += index_delta * index_delta;
        }
        let pitch = (pitch_numerator / pitch_denominator.max(f32::EPSILON)).abs();
        if !(45.0..=110.0).contains(&pitch) {
            break;
        }

        let mut x_slope_numerator = 0.0;
        for (index, hole) in selected.iter().enumerate() {
            x_slope_numerator += (index as f32 - index_mean) * (hole.0.x - selected[0].0.x);
        }
        let x_slope = x_slope_numerator / pitch_denominator.max(f32::EPSILON);
        let y_origin = y_mean - pitch * index_mean;
        let x_origin = selected
            .iter()
            .enumerate()
            .map(|(index, hole)| hole.0.x - x_slope * index as f32)
            .sum::<f32>()
            / n;

        let mut refined = Vec::with_capacity(selected.len());
        let mut used = Vec::new();
        for index in 0..selected.len() {
            let predicted = Point2f::new(
                x_origin + x_slope * index as f32,
                y_origin + pitch * index as f32,
            );
            let best = merged
                .iter()
                .enumerate()
                .filter(|(candidate_index, candidate)| {
                    !used.contains(candidate_index)
                        && (candidate.0.y - predicted.y).abs() <= pitch * 0.28
                })
                .min_by(|(_, left), (_, right)| {
                    let left_error = (left.0.x - predicted.x).abs()
                        + (left.0.y - predicted.y).abs()
                        - left.2 as f32 * 0.05;
                    let right_error = (right.0.x - predicted.x).abs()
                        + (right.0.y - predicted.y).abs()
                        - right.2 as f32 * 0.05;
                    left_error.total_cmp(&right_error)
                });
            if let Some((candidate_index, candidate)) = best {
                used.push(candidate_index);
                refined.push((candidate.0, candidate.1, candidate.2));
            } else {
                // Do not invent a hole outside the observed candidate span.
                // Keep the existing measurement and let the final fit use it.
                refined.push(selected[index]);
            }
        }
        selected = refined;
    }

    // Return the fitted positions. This removes sub-hole contour bias while
    // preserving the measured pitch, track slope, and selected hole count.
    if selected.len() >= 2 {
        let n = selected.len() as f32;
        let index_mean = (selected.len() - 1) as f32 / 2.0;
        let y_mean = selected.iter().map(|hole| hole.0.y).sum::<f32>() / n;
        let y_slope = selected
            .iter()
            .enumerate()
            .map(|(index, hole)| (index as f32 - index_mean) * (hole.0.y - y_mean))
            .sum::<f32>()
            / (selected.len() as f32 * (selected.len() as f32 * selected.len() as f32 - 1.0)
                / 12.0);
        let y_intercept = y_mean - y_slope * index_mean;
        let x_mean = selected.iter().map(|hole| hole.0.x).sum::<f32>() / n;
        let x_slope = selected
            .iter()
            .enumerate()
            .map(|(index, hole)| (index as f32 - index_mean) * (hole.0.x - x_mean))
            .sum::<f32>()
            / (selected.len() as f32 * (selected.len() as f32 * selected.len() as f32 - 1.0)
                / 12.0);
        let x_intercept = x_mean - x_slope * index_mean;
        for (index, hole) in selected.iter_mut().enumerate() {
            hole.0 = Point2f::new(
                x_intercept + x_slope * index as f32,
                y_intercept + y_slope * index as f32,
            );
        }
    }

    let pip_count = component_size_mm
        .filter(|size| size.height > 0.0 && px_per_mm > 0.0)
        .map(|size| {
            let pitch_px = if selected.len() >= 2 {
                (selected.last().unwrap().0.y - selected.first().unwrap().0.y).abs()
                    / (selected.len() - 1) as f32
            } else {
                0.0
            };
            ((size.height * px_per_mm / pitch_px.max(f32::EPSILON)).ceil() as usize + 1).clamp(4, 5)
        })
        .unwrap_or(selected.len());
    if selected.len() > pip_count {
        // The ROI and component are positioned from the lower feeder area,
        // so retain the bottom-most pips when a smaller component needs only
        // four holes. Keeping the tail preserves the bottom sprocket.
        let first = selected.len() - pip_count;
        selected = selected.split_off(first);
    }

    // Refit after reducing the pip count. Otherwise the returned bottom
    // subset still carries the intercept from the discarded upper holes.
    if selected.len() >= 2 {
        let n = selected.len() as f32;
        let index_mean = (selected.len() - 1) as f32 / 2.0;
        let denominator =
            selected.len() as f32 * (selected.len() as f32 * selected.len() as f32 - 1.0) / 12.0;
        let y_mean = selected.iter().map(|hole| hole.0.y).sum::<f32>() / n;
        let y_slope = selected
            .iter()
            .enumerate()
            .map(|(index, hole)| (index as f32 - index_mean) * (hole.0.y - y_mean))
            .sum::<f32>()
            / denominator.max(f32::EPSILON);
        let y_intercept = y_mean - y_slope * index_mean;
        let x_mean = selected.iter().map(|hole| hole.0.x).sum::<f32>() / n;
        let x_slope = selected
            .iter()
            .enumerate()
            .map(|(index, hole)| (index as f32 - index_mean) * (hole.0.x - x_mean))
            .sum::<f32>()
            / denominator.max(f32::EPSILON);
        let x_intercept = x_mean - x_slope * index_mean;
        for (index, hole) in selected.iter_mut().enumerate() {
            hole.0 = Point2f::new(
                x_intercept + x_slope * index as f32,
                y_intercept + y_slope * index as f32,
            );
        }
    }

    let mut debug_img = Mat::default();
    imgproc::cvt_color(
        &frame,
        &mut debug_img,
        imgproc::COLOR_GRAY2BGR,
        0,
        core::AlgorithmHint::ALGO_HINT_DEFAULT,
    )?;
    for (center, radius, _) in &selected {
        imgproc::circle(
            &mut debug_img,
            Point2i::new(center.x.round() as i32, center.y.round() as i32),
            radius.round() as i32,
            Scalar::new(0.0, 255.0, 0.0, 0.0),
            2,
            imgproc::LINE_AA,
            0,
        )?;
        imgproc::circle(
            &mut debug_img,
            Point2i::new(center.x.round() as i32, center.y.round() as i32),
            3,
            Scalar::new(0.0, 0.0, 255.0, 0.0),
            -1,
            imgproc::LINE_AA,
            0,
        )?;
    }

    // User-tunable ROI: the top is positioned between sprockets 2 and 3.
    // Horizontally it spans from the sprocket row to the right feeder wall.
    const ROI_TOP_FRACTION: f32 = 0.5;
    const ROI_SPROCKET_MARGIN: f32 = 4.0;
    const ROI_WALL_FIXED_START: i32 = 210;
    const ROI_WALL_FIXED_END: i32 = 350;
    const ROI_RIGHT_SEARCH_START: i32 = 300;
    const ROI_RIGHT_SEARCH_END: i32 = 470;
    const ROI_BOTTOM_FIXED_START: i32 = 300;
    const ROI_BOTTOM_FIXED_END: i32 = 430;
    const ROI_BOTTOM_SEARCH_START: i32 = 230;
    const ROI_BOTTOM_SEARCH_END: i32 = 490;

    if selected.len() >= 3 {
        let roi_top = selected[1].0.y + (selected[2].0.y - selected[1].0.y) * ROI_TOP_FRACTION;
        // Scan from the bright outer feeder side inward. A median scan across
        // the whole band can select an inner reflection instead of the wall.
        let right_feeder_wall = scan_outer_to_inner(
            &blur,
            true,
            ROI_WALL_FIXED_START,
            ROI_WALL_FIXED_END,
            (frame.cols() - 3).min(ROI_RIGHT_SEARCH_END),
            ROI_RIGHT_SEARCH_START,
            -1,
        )?;
        let bottom_wall = scan_transition(
            &blur,
            false,
            ROI_BOTTOM_FIXED_START,
            ROI_BOTTOM_FIXED_END,
            ROI_BOTTOM_SEARCH_START,
            ROI_BOTTOM_SEARCH_END,
        )?;

        if let (Some(right_wall), Some(bottom)) = (right_feeder_wall, bottom_wall) {
            let sprocket_right = selected
                .iter()
                .map(|(center, radius, _)| center.x + radius)
                .fold(f32::NEG_INFINITY, f32::max);
            let x = (sprocket_right + ROI_SPROCKET_MARGIN).round() as i32;
            let y = roi_top.round() as i32;
            let right = right_wall.position.round() as i32;
            let width = (right - x).max(1);
            let height = (bottom.position.round() as i32 - y).max(1);
            let roi = Mat::roi(&frame, core::Rect::new(x, y, width, height))?;
            let feeder_brightness = core::mean(&roi, &core::no_array())?[0];
            if feeder_brightness < 80.0 {
                eprintln!(
                    "ROI feeder brightness is low ({feeder_brightness:.1}); expected a bright feeder"
                );
            }
            imgproc::rectangle(
                &mut debug_img,
                core::Rect::new(x, y, width, height),
                Scalar::new(255.0, 0.0, 0.0, 0.0),
                2,
                imgproc::LINE_AA,
                0,
            )?;
        }
    }

    highgui::imshow("Filtered Sprocket Holes", &debug_img)?;
    highgui::wait_key(0)?;
    highgui::destroy_all_windows()?;

    Ok(selected.into_iter().map(|(center, _, _)| center).collect())
}

/// Find circularly symmetric features using the radial-variance method used
/// by OpenPnP's `DetectCircularSymmetry` stage. A good target has high pixel
/// variance across its complete disk but low variance within concentric rings.
/// The search rectangle is optional; when omitted, the whole image is used.
pub fn detect_circular_symmetry(
    image_path: &str,
    min_diameter: f32,
    max_diameter: f32,
    search: Option<core::Rect>,
    max_target_count: usize,
    min_symmetry: f64,
) -> Result<Vec<core::Point2f>> {
    let frame = imgcodecs::imread(image_path, imgcodecs::IMREAD_GRAYSCALE)?;
    if frame.empty() {
        return Err(opencv::Error::new(1, "Failed to load image from path"));
    }
    if max_target_count == 0 || min_diameter <= 0.0 || max_diameter < min_diameter {
        return Ok(Vec::new());
    }

    let min_radius = min_diameter * 0.5;
    let max_radius = max_diameter * 0.5;
    let region = search.unwrap_or(core::Rect::new(0, 0, frame.cols(), frame.rows()));
    let x_start = region.x.max(0);
    let y_start = region.y.max(0);
    let x_end = (region.x + region.width).min(frame.cols());
    let y_end = (region.y + region.height).min(frame.rows());
    if x_start >= x_end || y_start >= y_end {
        return Ok(Vec::new());
    }

    let coarse_step = ((min_diameter / 4.0).round() as i32).clamp(2, 8);
    let angle_bins = 16;
    let ring_count = 8;
    let radius_step = ((max_radius - min_radius) / 8.0).max(1.0);

    // This is the same statistic used by OpenPnP's default score: variance
    // across all radial samples divided by the sum of variance within each
    // ring. Angular bins make interrupted rings less likely to score well.
    let score_at = |center: Point2f, radius: f32| -> Result<f64> {
        let ring_width = radius / ring_count as f32;
        let mut ring_sum = vec![0.0; ring_count];
        let mut ring_sum_sq = vec![0.0; ring_count];
        let mut ring_count_samples = vec![0usize; ring_count];
        let mut bin_sum = vec![0.0; ring_count * angle_bins];
        let mut bin_count = vec![0usize; ring_count * angle_bins];
        let samples_per_ring = 32;

        for ring in 0..ring_count {
            let sample_radius = (ring as f32 + 0.5) * ring_width;
            for sample in 0..samples_per_ring {
                let angle = sample as f32 * std::f32::consts::TAU / samples_per_ring as f32;
                let ix = (center.x + angle.cos() * sample_radius).round() as i32;
                let iy = (center.y + angle.sin() * sample_radius).round() as i32;
                if ix < 0 || iy < 0 || ix >= frame.cols() || iy >= frame.rows() {
                    continue;
                }
                let value = *frame.at_2d::<u8>(iy, ix)? as f64;
                let bin = sample * angle_bins / samples_per_ring;
                let offset = ring * angle_bins + bin;
                ring_sum[ring] += value;
                ring_sum_sq[ring] += value * value;
                ring_count_samples[ring] += 1;
                bin_sum[offset] += value;
                bin_count[offset] += 1;
            }
        }

        let total_n: usize = ring_count_samples.iter().sum();
        if total_n < 2 {
            return Ok(0.0);
        }
        let total_sum: f64 = ring_sum.iter().sum();
        let total_sum_sq: f64 = ring_sum_sq.iter().sum();
        let variance_across = total_sum_sq - total_sum * total_sum / total_n as f64;
        let mut variance_rings = 0.0;
        for offset in 0..bin_sum.len() {
            if bin_count[offset] > 0 {
                variance_rings += bin_sum[offset] * bin_sum[offset] / bin_count[offset] as f64;
            }
        }
        let ring_sum_sq_total: f64 = ring_sum_sq.iter().sum();
        let variance_within = ring_sum_sq_total
            - ring_sum
                .iter()
                .zip(&ring_count_samples)
                .filter(|(_, count)| **count > 0)
                .map(|(sum, count)| sum * sum / *count as f64)
                .sum::<f64>();
        let angular_penalty = (variance_within - variance_rings).max(0.0);
        Ok((variance_across + 0.1) / (angular_penalty + 0.1))
    };

    let mut candidates: Vec<(Point2f, f32, f64)> = Vec::new();
    let mut score_map: Vec<(Point2f, f64)> = Vec::new();
    for y in (y_start..y_end).step_by(coarse_step as usize) {
        for x in (x_start..x_end).step_by(coarse_step as usize) {
            let center = Point2f::new(x as f32, y as f32);
            let mut best = (0.0, min_radius);
            let mut radius = min_radius;
            while radius <= max_radius + f32::EPSILON {
                let score = score_at(center, radius)?;
                if score > best.0 {
                    best = (score, radius);
                }
                radius += radius_step;
            }
            score_map.push((center, best.0));
            if best.0 >= min_symmetry {
                candidates.push((center, best.1, best.0));
            }
        }
    }

    candidates.sort_by(|left, right| right.2.total_cmp(&left.2));
    let mut targets: Vec<(Point2f, f32, f64)> = Vec::new();
    let suppression_distance = min_diameter * 0.7;
    for (coarse_center, coarse_radius, coarse_score) in candidates {
        let mut best = (coarse_center, coarse_radius, coarse_score);
        for dy in -(coarse_step / 2)..=(coarse_step / 2) {
            for dx in -(coarse_step / 2)..=(coarse_step / 2) {
                let center = Point2f::new(coarse_center.x + dx as f32, coarse_center.y + dy as f32);
                let mut radius = (coarse_radius - radius_step).max(min_radius);
                while radius <= (coarse_radius + radius_step).min(max_radius) {
                    let score = score_at(center, radius)?;
                    if score > best.2 {
                        best = (center, radius, score);
                    }
                    radius += 1.0;
                }
            }
        }
        let (center, radius, score) = best;
        if targets.iter().all(|(other, _, _)| {
            let dx = other.x - center.x;
            let dy = other.y - center.y;
            dx * dx + dy * dy > suppression_distance * suppression_distance
        }) {
            targets.push((center, radius, score));
            if targets.len() == max_target_count {
                break;
            }
        }
    }

    let mut debug = Mat::default();
    imgproc::cvt_color(
        &frame,
        &mut debug,
        imgproc::COLOR_GRAY2BGR,
        0,
        core::AlgorithmHint::ALGO_HINT_DEFAULT,
    )?;

    // OpenPnP-style diagnostic heatmap: brighter/w warmer colors represent
    // centers with a stronger radial-symmetry score. Draw the coarse map first
    // and then overlay the refined detections below it.
    if let (Some(min_score), Some(max_score)) = (
        score_map
            .iter()
            .map(|(_, score)| *score)
            .filter(|score| score.is_finite())
            .min_by(|a, b| a.total_cmp(b)),
        score_map
            .iter()
            .map(|(_, score)| *score)
            .filter(|score| score.is_finite())
            .max_by(|a, b| a.total_cmp(b)),
    ) {
        let mut heatmap = debug.clone();
        let score_range = (max_score - min_score).max(f64::EPSILON);
        for (center, score) in score_map {
            if !score.is_finite() {
                continue;
            }
            let normalized = ((score - min_score) / score_range).clamp(0.0, 1.0);
            let blue = (255.0 * (1.0 - normalized)) as f64;
            let red = (255.0 * normalized) as f64;
            imgproc::circle(
                &mut heatmap,
                Point2i::new(center.x.round() as i32, center.y.round() as i32),
                (coarse_step / 2).max(1),
                Scalar::new(blue, 32.0, red, 0.0),
                -1,
                imgproc::LINE_AA,
                0,
            )?;
        }
        let mut blended = Mat::default();
        core::add_weighted(&heatmap, 0.42, &debug, 0.58, 0.0, &mut blended, -1)?;
        debug = blended;
    }

    imgproc::put_text(
        &mut debug,
        "radial symmetry: blue=weak, red=strong",
        Point2i::new(12, 24),
        imgproc::FONT_HERSHEY_SIMPLEX,
        0.55,
        Scalar::new(255.0, 255.0, 255.0, 0.0),
        1,
        imgproc::LINE_AA,
        false,
    )?;
    for (center, radius, score) in &targets {
        imgproc::circle(
            &mut debug,
            Point2i::new(center.x.round() as i32, center.y.round() as i32),
            radius.round().max(1.0) as i32,
            Scalar::new(0.0, 255.0, 0.0, 0.0),
            2,
            imgproc::LINE_AA,
            0,
        )?;
        imgproc::circle(
            &mut debug,
            Point2i::new(center.x.round() as i32, center.y.round() as i32),
            3,
            Scalar::new(0.0, 0.0, 255.0, 0.0),
            -1,
            imgproc::LINE_AA,
            0,
        )?;
        imgproc::put_text(
            &mut debug,
            &format!("{score:.2}"),
            Point2i::new(center.x.round() as i32 + 6, center.y.round() as i32 - 6),
            imgproc::FONT_HERSHEY_SIMPLEX,
            0.45,
            Scalar::new(0.0, 255.0, 255.0, 0.0),
            1,
            imgproc::LINE_AA,
            false,
        )?;
    }
    highgui::imshow("Circular Symmetry", &debug)?;
    highgui::wait_key(0)?;
    highgui::destroy_window("Circular Symmetry")?;

    Ok(targets.into_iter().map(|(center, _, _)| center).collect())
}

fn scan_transition(
    image: &Mat,
    horizontal: bool,
    fixed_start: i32,
    fixed_end: i32,
    search_start: i32,
    search_end: i32,
) -> Result<Option<TransitionEdge>> {
    let mut positions = Vec::new();
    let mut strengths = Vec::new();
    let fixed_step = ((fixed_end - fixed_start) / 12).max(1);

    for fixed in (fixed_start..=fixed_end).step_by(fixed_step as usize) {
        let mut best_position = 0;
        let mut best_strength = 0.0_f32;
        for position in search_start + 2..search_end - 2 {
            let (before, after) = if horizontal {
                (
                    *image.at_2d::<u8>(fixed, position - 2)? as f32,
                    *image.at_2d::<u8>(fixed, position + 2)? as f32,
                )
            } else {
                (
                    *image.at_2d::<u8>(position - 2, fixed)? as f32,
                    *image.at_2d::<u8>(position + 2, fixed)? as f32,
                )
            };
            let strength = (after - before).abs();
            if strength > best_strength {
                best_strength = strength;
                best_position = position;
            }
        }
        if best_strength >= 12.0 {
            positions.push(best_position as f32);
            strengths.push(best_strength);
        }
    }

    if positions.len() < 4 {
        return Ok(None);
    }
    positions.sort_by(|a, b| a.total_cmp(b));
    strengths.sort_by(|a, b| a.total_cmp(b));
    Ok(Some(TransitionEdge {
        position: positions[positions.len() / 2],
        strength: strengths[strengths.len() / 2],
    }))
}

fn scan_outer_to_inner(
    image: &Mat,
    horizontal: bool,
    fixed_start: i32,
    fixed_end: i32,
    start: i32,
    end: i32,
    inward: i32,
) -> Result<Option<TransitionEdge>> {
    let fixed_limit = if horizontal {
        image.rows()
    } else {
        image.cols()
    };
    let position_limit = if horizontal {
        image.cols()
    } else {
        image.rows()
    };
    let fixed_start = fixed_start.clamp(0, fixed_limit - 1);
    let fixed_end = fixed_end.clamp(0, fixed_limit - 1);
    let start = start.clamp(2, position_limit - 3);
    let end = end.clamp(2, position_limit - 3);
    if fixed_start > fixed_end || (inward > 0 && start > end) || (inward < 0 && end > start) {
        return Ok(None);
    }
    let mut edges = Vec::new();
    let step = ((fixed_end - fixed_start) / 12).max(1) as usize;
    for fixed in (fixed_start..=fixed_end).step_by(step) {
        let mut found = None;
        let positions: Vec<i32> = if inward > 0 {
            (start..=end).collect()
        } else {
            (end..=start).rev().collect()
        };
        for position in positions {
            let (before, after) = if inward > 0 {
                if horizontal {
                    (
                        *image.at_2d::<u8>(fixed, position - 2)? as f32,
                        *image.at_2d::<u8>(fixed, position + 2)? as f32,
                    )
                } else {
                    (
                        *image.at_2d::<u8>(position - 2, fixed)? as f32,
                        *image.at_2d::<u8>(position + 2, fixed)? as f32,
                    )
                }
            } else {
                if horizontal {
                    (
                        *image.at_2d::<u8>(fixed, position + 2)? as f32,
                        *image.at_2d::<u8>(fixed, position - 2)? as f32,
                    )
                } else {
                    (
                        *image.at_2d::<u8>(position + 2, fixed)? as f32,
                        *image.at_2d::<u8>(position - 2, fixed)? as f32,
                    )
                }
            };
            if (after - before).abs() >= 12.0 {
                found = Some(TransitionEdge {
                    position: position as f32,
                    strength: (after - before).abs(),
                });
                break;
            }
        }
        if let Some(edge) = found {
            edges.push(edge);
        }
    }
    if edges.len() < 4 {
        return Ok(None);
    }
    edges.sort_by(|a, b| a.position.total_cmp(&b.position));
    Ok(Some(edges[edges.len() / 2]))
}

fn show_final_debug_view(
    frame: &Mat,
    center: Point2f,
    rect: Option<opencv::core::RotatedRect>,
    hinted_size: Option<Size>,
    search_size: Option<Size>,
    search_margin: Option<f32>,
    scan_segments: Option<[(Point2f, Point2f); 4]>,
    title: &str,
) -> Result<()> {
    if std::env::var_os("PNP_DEBUG_VIEW").is_none() {
        return Ok(());
    }

    let mut display = Mat::default();
    imgproc::cvt_color(
        frame,
        &mut display,
        imgproc::COLOR_GRAY2BGR,
        0,
        core::AlgorithmHint::ALGO_HINT_DEFAULT,
    )?;

    if let Some(rect) = rect {
        let mut points = Mat::default();
        box_points(rect, &mut points)?;
        let points = points.reshape_def(Point2f::opencv_channels())?;
        for index in 0..4 {
            let first = *points.at::<Point2f>(index)?;
            let second = *points.at::<Point2f>((index + 1) % 4)?;
            imgproc::line(
                &mut display,
                Point2i::new(first.x.round() as i32, first.y.round() as i32),
                Point2i::new(second.x.round() as i32, second.y.round() as i32),
                Scalar::new(0.0, 255.0, 0.0, 0.0),
                2,
                imgproc::LINE_AA,
                0,
            )?;
        }
    } else if let Some(size) = hinted_size {
        let bounds = core::Rect::new(
            (center.x - size.width / 2.0).round() as i32,
            (center.y - size.height / 2.0).round() as i32,
            size.width.round().max(1.0) as i32,
            size.height.round().max(1.0) as i32,
        );
        imgproc::rectangle(
            &mut display,
            bounds,
            Scalar::new(0.0, 255.0, 255.0, 0.0),
            2,
            imgproc::LINE_AA,
            0,
        )?;
    }

    if let Some(size) = search_size {
        let margin = search_margin.unwrap_or(0.0);
        let target = hinted_size.unwrap_or(size);
        let left = (center.x - target.width / 2.0).round() as i32;
        let right = (center.x + target.width / 2.0).round() as i32;
        let top = (center.y - target.height / 2.0).round() as i32;
        let bottom = (center.y + target.height / 2.0).round() as i32;
        let band = Scalar::new(255.0, 0.0, 255.0, 0.0);
        for bounds in [
            core::Rect::new(
                left - margin.round() as i32,
                top,
                margin.round().max(1.0) as i32,
                (bottom - top).max(1),
            ),
            core::Rect::new(
                right,
                top,
                margin.round().max(1.0) as i32,
                (bottom - top).max(1),
            ),
            core::Rect::new(
                left,
                top - margin.round() as i32,
                (right - left).max(1),
                margin.round().max(1.0) as i32,
            ),
            core::Rect::new(
                left,
                bottom,
                (right - left).max(1),
                margin.round().max(1.0) as i32,
            ),
        ] {
            imgproc::rectangle(&mut display, bounds, band, 1, imgproc::LINE_AA, 0)?;
        }
        if let Some(segments) = scan_segments {
            for (outer, inner) in segments {
                let outer = Point2i::new(outer.x.round() as i32, outer.y.round() as i32);
                let inner = Point2i::new(inner.x.round() as i32, inner.y.round() as i32);
                imgproc::arrowed_line(
                    &mut display,
                    outer,
                    inner,
                    Scalar::new(255.0, 255.0, 0.0, 0.0),
                    3,
                    imgproc::LINE_AA,
                    0,
                    0.08,
                )?;
                imgproc::circle(
                    &mut display,
                    outer,
                    6,
                    Scalar::new(255.0, 0.0, 0.0, 0.0),
                    2,
                    imgproc::LINE_AA,
                    0,
                )?;
                imgproc::circle(
                    &mut display,
                    inner,
                    5,
                    Scalar::new(0.0, 255.0, 255.0, 0.0),
                    2,
                    imgproc::LINE_AA,
                    0,
                )?;
            }
        }
        let _ = (hinted_size, margin);
        let _ = margin;
    }

    imgproc::circle(
        &mut display,
        Point2i::new(center.x.round() as i32, center.y.round() as i32),
        5,
        Scalar::new(0.0, 0.0, 255.0, 0.0),
        -1,
        imgproc::LINE_AA,
        0,
    )?;
    highgui::imshow(title, &display)?;
    // Any key advances to the next fixture; ESC also works for callers that
    // run this interactively.
    highgui::wait_key(0)?;
    highgui::destroy_window(title)?;
    Ok(())
}

pub fn populated_pocket_detection_with_hint(
    image_path: &str,
    hint: PocketHint,
) -> Result<Option<Pose>> {
    let frame = imgcodecs::imread(image_path, imgcodecs::IMREAD_GRAYSCALE)?;
    if frame.empty() {
        return Err(opencv::Error::new(1, "Failed to load image from path"));
    }
    let mut blur = Mat::default();
    imgproc::median_blur(&frame, &mut blur, 3)?;
    let (hint_center, hint_size, position_tolerance, size_tolerance) =
        hint.resolve(frame.cols(), frame.rows());
    let half_w = hint_size.width / 2.0;
    let half_h = hint_size.height / 2.0;
    let margin = size_tolerance.max(12.0);
    let cx = hint_center.x;
    let cy = hint_center.y;
    let scan_segments = [
        (
            Point2f::new(cx - half_w - margin, cy),
            Point2f::new(cx - half_w, cy),
        ),
        (
            Point2f::new(cx + half_w + margin, cy),
            Point2f::new(cx + half_w, cy),
        ),
        (
            Point2f::new(cx, cy - half_h - margin),
            Point2f::new(cx, cy - half_h),
        ),
        (
            Point2f::new(cx, cy + half_h + margin),
            Point2f::new(cx, cy + half_h),
        ),
    ];
    show_final_debug_view(
        &frame,
        Point2f::new(cx, cy),
        None,
        Some(hint_size),
        Some(Size {
            width: hint_size.width + margin * 2.0,
            height: hint_size.height + margin * 2.0,
        }),
        Some(margin),
        Some(scan_segments),
        "Pocket hint search window",
    )?;
    let left = scan_outer_to_inner(
        &blur,
        true,
        (cy - half_h) as i32,
        (cy + half_h) as i32,
        (cx - half_w - margin) as i32,
        (cx - half_w + margin) as i32,
        1,
    )?;
    let right = scan_outer_to_inner(
        &blur,
        true,
        (cy - half_h) as i32,
        (cy + half_h) as i32,
        (cx + half_w + margin) as i32,
        (cx + half_w - margin) as i32,
        -1,
    )?;
    let top = scan_outer_to_inner(
        &blur,
        false,
        (cx - half_w) as i32,
        (cx + half_w) as i32,
        (cy - half_h - margin) as i32,
        (cy - half_h + margin) as i32,
        1,
    )?;
    let bottom = scan_outer_to_inner(
        &blur,
        false,
        (cx - half_w) as i32,
        (cx + half_w) as i32,
        (cy + half_h + margin) as i32,
        (cy + half_h - margin) as i32,
        -1,
    )?;
    let (Some(left), Some(right), Some(top), Some(bottom)) = (left, right, top, bottom) else {
        return populated_pocket_detection(image_path);
    };
    let detected_width = right.position - left.position;
    let detected_height = bottom.position - top.position;
    let detected_center = Point::new(
        (left.position + right.position) / 2.0,
        (top.position + bottom.position) / 2.0,
    );
    let valid = detected_width > 0.0
        && detected_height > 0.0
        && (detected_center.x - hint_center.x).abs() <= position_tolerance
        && (detected_center.y - hint_center.y).abs() <= position_tolerance
        && (detected_width - hint_size.width).abs() <= size_tolerance * 2.0
        && (detected_height - hint_size.height).abs() <= size_tolerance * 2.0;
    if valid {
        show_final_debug_view(
            &frame,
            Point2f::new(detected_center.x, detected_center.y),
            None,
            Some(Size {
                width: detected_width,
                height: detected_height,
            }),
            Some(Size {
                width: hint_size.width + margin * 2.0,
                height: hint_size.height + margin * 2.0,
            }),
            Some(margin),
            Some(scan_segments),
            "Pocket detection (hinted edges)",
        )?;
        Ok(Some(Pose::new(
            detected_center.x.round(),
            detected_center.y.round(),
            0.0,
        )))
    } else {
        populated_pocket_detection(image_path)
    }
}

/// Detect a row of usable pockets using OpenPnP-style edge validation for
/// every pocket. `centers` are search seeds, not returned coordinates: each
/// seed is refined by scanning the four pocket edges and checking the hinted
/// physical size.
pub fn detect_pocket_sequence_with_hint(
    image_path: &str,
    hint: PocketHint,
    centers: &[Point],
) -> Result<Vec<Pose>> {
    if centers.is_empty() {
        return Ok(Vec::new());
    }

    let mut pockets = Vec::with_capacity(centers.len());
    for &center in centers {
        let Some(pose) =
            populated_pocket_detection_with_hint(image_path, hint.with_center_px(center))?
        else {
            continue;
        };
        pockets.push(pose);
    }
    Ok(pockets)
}

/// Generate search seeds from a measured pocket pitch and detect each pocket
/// independently. This keeps the pitch model separate from the edge detector,
/// so one bad pocket cannot move every other pocket in the sequence.
pub fn detect_pocket_sequence(
    image_path: &str,
    hint: PocketHint,
    first_center: Point,
    pitch_px: f32,
    count: usize,
) -> Result<Vec<Pose>> {
    if !(1..=32).contains(&count) || !pitch_px.is_finite() || pitch_px == 0.0 {
        return Ok(Vec::new());
    }

    let centers: Vec<_> = (0..count)
        .map(|index| Point::new(first_center.x, first_center.y + pitch_px * index as f32))
        .collect();
    detect_pocket_sequence_with_hint(image_path, hint, &centers)
}

pub fn populated_pocket_detection(image_path: &str) -> Result<Option<Pose>> {
    let frame = imgcodecs::imread(image_path, imgcodecs::IMREAD_GRAYSCALE)?;
    if frame.empty() {
        return Err(opencv::Error::new(1, "Failed to load image from path"));
    }

    let img_cols = frame.cols();
    let img_rows = frame.rows();

    // 1. Preprocessing: Suppress high-frequency sensor noise
    let mut frame_blur = Mat::default();
    imgproc::median_blur(&frame, &mut frame_blur, 3)?;
    let inner_rect = core::Rect::new(15, 15, (img_cols - 30).max(1), (img_rows - 30).max(1));
    let inner_roi = Mat::roi(&frame_blur, inner_rect)?;
    let mean_gray = core::mean(&inner_roi, &core::no_array())?[0];

    // First try the material-independent four-side detector. It uses several
    // scanlines per side, so one glare streak cannot define the rectangle.
    let left = scan_transition(&frame_blur, true, 210, 350, 250, 410)?;
    let right = scan_transition(&frame_blur, true, 210, 350, 310, 470)?;
    let top = scan_transition(&frame_blur, false, 300, 430, 180, 310)?;
    let bottom = scan_transition(&frame_blur, false, 300, 430, 230, 390)?;
    let edge_pose =
        if let (Some(left), Some(right), Some(top), Some(bottom)) = (left, right, top, bottom) {
            let width = right.position - left.position;
            let height = bottom.position - top.position;
            let center_x = (left.position + right.position) / 2.0;
            let center_y = (top.position + bottom.position) / 2.0;
            let plausible = (25.0..=180.0).contains(&width)
                && (20.0..=150.0).contains(&height)
                && (280.0..=440.0).contains(&center_x)
                && (190.0..=350.0).contains(&center_y)
                && left.strength >= 12.0
                && right.strength >= 12.0
                && top.strength >= 12.0
                && bottom.strength >= 12.0;
            if plausible {
                Some(Pose {
                    position: Point::new(center_x.round(), center_y.round()),
                    angle_deg: 0.0,
                })
            } else {
                None
            }
        } else {
            None
        };

    // 2. Pipeline A: Gradient Magnitude for Clear/Reflective Tapes (Ring-light wall slopes)
    let mut grad_x = Mat::default();
    let mut grad_y = Mat::default();
    imgproc::sobel(
        &frame_blur,
        &mut grad_x,
        opencv::core::CV_32F,
        1,
        0,
        3,
        1.0,
        0.0,
        core::BORDER_DEFAULT,
    )?;
    imgproc::sobel(
        &frame_blur,
        &mut grad_y,
        opencv::core::CV_32F,
        0,
        1,
        3,
        1.0,
        0.0,
        core::BORDER_DEFAULT,
    )?;

    let mut grad_mag = Mat::default();
    core::magnitude(&grad_x, &grad_y, &mut grad_mag)?;

    let mut grad_8u = Mat::default();
    core::normalize(
        &grad_mag,
        &mut grad_8u,
        0.0,
        255.0,
        core::NORM_MINMAX,
        opencv::core::CV_8U,
        &core::no_array(),
    )?;

    // Morphological closing to bridge fragmented clear-tape edges caused by glare
    let mut closed_edges = Mat::default();
    let kernel = imgproc::get_structuring_element(
        imgproc::MORPH_RECT,
        core::Size::new(3, 3),
        core::Point::new(-1, -1),
    )?;
    imgproc::morphology_ex(
        &grad_8u,
        &mut closed_edges,
        imgproc::MORPH_CLOSE,
        &kernel,
        core::Point::new(-1, -1),
        1,
        core::BORDER_CONSTANT,
        imgproc::morphology_default_border_value()?,
    )?;

    // 3. Pipeline B: Adaptive Thresholding for Matte Black/White Tapes
    let mut adaptive_thresh = Mat::default();
    imgproc::adaptive_threshold(
        &frame_blur,
        &mut adaptive_thresh,
        255.0,
        imgproc::ADAPTIVE_THRESH_GAUSSIAN_C,
        imgproc::THRESH_BINARY_INV,
        31,
        5.0,
    )?;

    // Pool binary sources to evaluate
    let mut adaptive_thresh_bright = Mat::default();
    imgproc::adaptive_threshold(
        &frame_blur,
        &mut adaptive_thresh_bright,
        255.0,
        imgproc::ADAPTIVE_THRESH_GAUSSIAN_C,
        imgproc::THRESH_BINARY,
        31,
        5.0,
    )?;
    let mut adaptive_thresh_fine = Mat::default();
    imgproc::adaptive_threshold(
        &frame_blur,
        &mut adaptive_thresh_fine,
        255.0,
        imgproc::ADAPTIVE_THRESH_GAUSSIAN_C,
        imgproc::THRESH_BINARY_INV,
        15,
        3.0,
    )?;
    let mut adaptive_thresh_wide = Mat::default();
    imgproc::adaptive_threshold(
        &frame_blur,
        &mut adaptive_thresh_wide,
        255.0,
        imgproc::ADAPTIVE_THRESH_GAUSSIAN_C,
        imgproc::THRESH_BINARY,
        51,
        7.0,
    )?;
    let close_kernel = imgproc::get_structuring_element(
        imgproc::MORPH_RECT,
        core::Size::new(9, 9),
        core::Point::new(-1, -1),
    )?;
    let mut adaptive_closed = Mat::default();
    imgproc::morphology_ex(
        &adaptive_thresh_bright,
        &mut adaptive_closed,
        imgproc::MORPH_CLOSE,
        &close_kernel,
        core::Point::new(-1, -1),
        1,
        core::BORDER_CONSTANT,
        imgproc::morphology_default_border_value()?,
    )?;
    let mut global_dark = Mat::default();
    imgproc::threshold(
        &frame_blur,
        &mut global_dark,
        mean_gray * 0.7,
        255.0,
        imgproc::THRESH_BINARY,
    )?;
    let mut global_bright = Mat::default();
    imgproc::threshold(
        &frame_blur,
        &mut global_bright,
        mean_gray * 0.7,
        255.0,
        imgproc::THRESH_BINARY_INV,
    )?;
    let candidate_sources = vec![
        closed_edges,
        adaptive_thresh,
        adaptive_thresh_bright,
        adaptive_thresh_fine,
        adaptive_thresh_wide,
        adaptive_closed,
        global_dark,
        global_bright,
    ];

    let cx_ref = img_cols as f32 / 2.0;
    let cy_ref = img_rows as f32 / 2.0;
    let mut candidates = Vec::<(opencv::core::RotatedRect, f32, Point2f)>::new();

    // Pocket size & geometry constraints
    let min_pocket_area = 200.0;
    let max_pocket_area = 8000.0;

    for source_img in candidate_sources {
        let mut contours = Vector::<Vector<core::Point>>::new();
        imgproc::find_contours(
            &source_img,
            &mut contours,
            imgproc::RETR_EXTERNAL,
            imgproc::CHAIN_APPROX_SIMPLE,
            core::Point::new(0, 0),
        )?;

        for i in 0..contours.len() {
            let contour = contours.get(i)?;
            let area = contour_area(&contour, false)?;

            if area < min_pocket_area as f64 || area > max_pocket_area as f64 {
                continue;
            }

            let rot_rect = min_area_rect(&contour)?;
            let width = rot_rect.size.width;
            let height = rot_rect.size.height;
            if width == 0.0 || height == 0.0 {
                continue;
            }

            // Validate EIA-481 Pocket Aspect Ratio
            let aspect_ratio = (width / height).max(height / width);
            if aspect_ratio < 1.05 || aspect_ratio > 4.0 {
                continue;
            }

            // Validate Contour Extent (tolerates populated parts filling the interior)
            let box_area = width * height;
            let extent = area as f32 / box_area;
            if !(0.12..=1.05).contains(&extent) {
                continue;
            }

            // Relaxed working region bounds (accommodates minor mechanical indexing jitter)
            let center = rot_rect.center;
            if !(300.0..=430.0).contains(&center.x) || !(205.0..=350.0).contains(&center.y) {
                continue;
            }
            candidates.push((rot_rect, extent, center));
        }
    }

    let mut best_score = f32::MAX;
    let mut best_pocket_rect: Option<opencv::core::RotatedRect> = None;
    let mut best_center = None;
    for (rect, extent, center) in &candidates {
        let dx = center.x - cx_ref;
        let dy = center.y - cy_ref;
        let score = dx * dx + dy * dy + (0.65 - extent).abs() * 100.0;
        if score < best_score {
            best_score = score;
            best_pocket_rect = Some(*rect);
            best_center = Some(*center);
        }
    }

    let pocket_rect = match best_pocket_rect {
        Some(r) => r,
        None => return Ok(edge_pose),
    };
    let center = best_center.unwrap_or(pocket_rect.center);

    show_final_debug_view(
        &frame,
        center,
        Some(pocket_rect),
        None,
        None,
        None,
        None,
        "Pocket detection",
    )?;

    // 4. Debug Display Overlay
    let mut display_img = Mat::default();
    imgproc::cvt_color(
        &frame,
        &mut display_img,
        imgproc::COLOR_GRAY2BGR,
        0,
        core::AlgorithmHint::ALGO_HINT_DEFAULT,
    )?;

    let mut box_pts_mat = Mat::default();
    box_points(pocket_rect, &mut box_pts_mat)?;
    let box_pts = box_pts_mat.reshape_def(Point2f::opencv_channels())?;

    for i in 0..4 {
        let pt1 = *box_pts.at::<Point2f>(i)?;
        let pt2 = *box_pts.at::<Point2f>((i + 1) % 4)?;

        imgproc::line(
            &mut display_img,
            Point2i::new(pt1.x.round() as i32, pt1.y.round() as i32),
            Point2i::new(pt2.x.round() as i32, pt2.y.round() as i32),
            Scalar::new(0.0, 255.0, 0.0, 0.0),
            2,
            imgproc::LINE_AA,
            0,
        )?;
    }

    imgproc::circle(
        &mut display_img,
        Point2i::new(center.x.round() as i32, center.y.round() as i32),
        4,
        Scalar::new(0.0, 0.0, 255.0, 0.0),
        -1,
        imgproc::LINE_AA,
        0,
    )?;

    if std::env::var_os("PNP_DEBUG_VIEW").is_some() {
        highgui::imshow("final", &display_img)?;
        highgui::wait_key(3000)?;
        highgui::destroy_window("final")?;
    }

    let _ = display_img;
    Ok(Some(Pose {
        position: Point {
            x: center.x.round(),
            y: center.y.round(),
        },
        angle_deg: pocket_rect.angle,
    }))
}

/// Runs the active detector and accepts the nearest luma contour only when its
/// rotated rectangle falls within the caller's pixel-size bounds.
pub fn populated_pocket_detection_with_size(
    image_path: &str,
    min_size_px: Size,
    max_size_px: Size,
) -> Result<Option<Pose>> {
    let pose = match populated_pocket_detection(image_path)? {
        Some(pose) => pose,
        None => return Ok(None),
    };
    let frame = imgcodecs::imread(image_path, imgcodecs::IMREAD_GRAYSCALE)?;
    let mut blur = Mat::default();
    imgproc::median_blur(&frame, &mut blur, 3)?;
    let mut grad_x = Mat::default();
    let mut grad_y = Mat::default();
    imgproc::sobel(
        &blur,
        &mut grad_x,
        core::CV_32F,
        1,
        0,
        3,
        1.0,
        0.0,
        core::BORDER_DEFAULT,
    )?;
    imgproc::sobel(
        &blur,
        &mut grad_y,
        core::CV_32F,
        0,
        1,
        3,
        1.0,
        0.0,
        core::BORDER_DEFAULT,
    )?;
    let mut magnitude = Mat::default();
    core::magnitude(&grad_x, &grad_y, &mut magnitude)?;
    let mut magnitude_8u = Mat::default();
    core::normalize(
        &magnitude,
        &mut magnitude_8u,
        0.0,
        255.0,
        core::NORM_MINMAX,
        core::CV_8U,
        &core::no_array(),
    )?;
    let mut mask = Mat::default();
    imgproc::threshold(
        &magnitude_8u,
        &mut mask,
        0.0,
        255.0,
        imgproc::THRESH_BINARY | imgproc::THRESH_OTSU,
    )?;
    let mut contours = Vector::<Vector<core::Point>>::new();
    imgproc::find_contours(
        &mask,
        &mut contours,
        imgproc::RETR_EXTERNAL,
        imgproc::CHAIN_APPROX_SIMPLE,
        core::Point::new(0, 0),
    )?;
    let mut best = None;
    let mut best_distance = f32::MAX;
    for index in 0..contours.len() {
        let rect = min_area_rect(&contours.get(index)?)?;
        let width = rect.size.width.max(rect.size.height);
        let height = rect.size.width.min(rect.size.height);
        if width < min_size_px.width
            || height < min_size_px.height
            || width > max_size_px.width
            || height > max_size_px.height
        {
            continue;
        }
        let distance = ((rect.center.x - pose.position.x).powi(2)
            + (rect.center.y - pose.position.y).powi(2))
        .sqrt();
        if distance < best_distance {
            best_distance = distance;
            best = Some(rect);
        }
    }
    Ok(best.map(|rect| Pose {
        position: Point::new(rect.center.x.round(), rect.center.y.round()),
        angle_deg: rect.angle,
    }))
}

pub fn populated_pocket_detection_old(image_path: &str) -> Result<Option<Pose>> {
    let frame = imgcodecs::imread(image_path, imgcodecs::IMREAD_GRAYSCALE)?;
    if frame.empty() {
        return Err(opencv::Error {
            code: 1,
            message: "Failed to load image from path".into(),
        });
    }

    let img_cols = frame.cols();
    let img_rows = frame.rows();

    // Stage A & B: Grayscale conversion & Median blur (3x3 kernel)[cite: 1]
    let mut frame_blur = Mat::default();
    imgproc::median_blur(&frame, &mut frame_blur, 3)?;

    // Stage C: ROI mean-gray auto-threshold (excluding 15-pixel illumination ring border)[cite: 1]
    let inner_rect = core::Rect::new(15, 15, (img_cols - 30).max(1), (img_rows - 30).max(1));
    let inner_roi = Mat::roi(&frame_blur, inner_rect)?;
    let mean_scalar = core::mean(&inner_roi, &core::no_array())?;
    let cx_ref = img_cols as f32 / 2.0;
    let cy_ref = img_rows as f32 / 2.0;
    let mut best_score = f32::MAX;
    let mut best_pocket_rect: Option<RotatedRect> = None;
    let mut best_center: Option<Point2f> = None;

    // Adjusted size constraints: Increased minimum area to completely ignore small sprocket feed holes
    let min_pocket_area = 200.0; // Lowered to support 0402 pockets (smaller than 0603)
    let max_pocket_area = 8000.0;

    // Tape can be darker or brighter than the carrier. Evaluate both polarities
    // instead of making the material color part of the detector configuration.
    for threshold_factor in [0.55, 0.65, 0.75, 0.85, 0.95] {
        for threshold_type in [imgproc::THRESH_BINARY, imgproc::THRESH_BINARY_INV] {
            let mut frame_thresh = Mat::default();
            imgproc::threshold(
                &frame_blur,
                &mut frame_thresh,
                mean_scalar[0] * threshold_factor,
                255.0,
                threshold_type,
            )?;

            let mut contours = Vector::<Vector<core::Point>>::new();
            let mut hierarchy = Mat::default();
            imgproc::find_contours_with_hierarchy(
                &frame_thresh,
                &mut contours,
                &mut hierarchy,
                imgproc::RETR_TREE,
                imgproc::CHAIN_APPROX_NONE,
                core::Point::new(0, 0),
            )?;

            for i in 0..contours.len() {
                let contour = contours.get(i)?;
                if contour.len() < 10 {
                    continue;
                }
                let h_elem = *hierarchy.at_2d::<core::Vec4i>(0, i.try_into().unwrap())?;
                if h_elem[2] == -1 {
                    continue;
                }
                let rot_rect = min_area_rect(&contour)?;
                let area = rot_rect.size.width * rot_rect.size.height;
                if area < min_pocket_area || area > max_pocket_area {
                    continue;
                }
                // The feeder pocket is in the central working area. Rejecting the
                // sprocket/edge contours here prevents dark tape and clear reflections
                // from winning simply because they are close to the image midpoint.
                if !(340.0..=410.0).contains(&rot_rect.center.x)
                    || !(210.0..=330.0).contains(&rot_rect.center.y)
                {
                    continue;
                }
                let center = rot_rect.center;
                let dx = center.x - cx_ref;
                let dy = center.y - cy_ref;
                let score = dx * dx + dy * dy;
                if score < best_score {
                    best_score = score;
                    best_pocket_rect = Some(rot_rect);
                    best_center = Some(center);
                }
            }
        }
    }

    // Local thresholding handles the strong illumination gradients visible in
    // the clear-tape fixtures without sacrificing the global candidates above.
    for threshold_type in [imgproc::THRESH_BINARY, imgproc::THRESH_BINARY_INV] {
        let mut frame_thresh = Mat::default();
        imgproc::adaptive_threshold(
            &frame_blur,
            &mut frame_thresh,
            255.0,
            imgproc::ADAPTIVE_THRESH_GAUSSIAN_C,
            threshold_type,
            31,
            5.0,
        )?;
        let mut contours = Vector::<Vector<core::Point>>::new();
        let mut hierarchy = Mat::default();
        imgproc::find_contours_with_hierarchy(
            &frame_thresh,
            &mut contours,
            &mut hierarchy,
            imgproc::RETR_TREE,
            imgproc::CHAIN_APPROX_NONE,
            core::Point::new(0, 0),
        )?;
        for i in 0..contours.len() {
            let contour = contours.get(i)?;
            if contour.len() < 10 {
                continue;
            }
            let h_elem = *hierarchy.at_2d::<core::Vec4i>(0, i.try_into().unwrap())?;
            if h_elem[2] == -1 {
                continue;
            }
            let rot_rect = min_area_rect(&contour)?;
            let area = rot_rect.size.width * rot_rect.size.height;
            if area < min_pocket_area || area > max_pocket_area {
                continue;
            }
            if !(340.0..=410.0).contains(&rot_rect.center.x)
                || !(210.0..=330.0).contains(&rot_rect.center.y)
            {
                continue;
            }
            let dx = rot_rect.center.x - cx_ref;
            let dy = rot_rect.center.y - cy_ref;
            let score = dx * dx + dy * dy;
            if score < best_score {
                best_score = score;
                best_pocket_rect = Some(rot_rect);
                best_center = Some(rot_rect.center);
            }
        }
    }

    let pocket_rect = match best_pocket_rect {
        Some(r) => r,
        None => return Ok(None),
    };
    let center = best_center.unwrap_or(pocket_rect.center);

    // Stage J: Debug overlay (line thickness 2, anti-aliased LINE_AA)[cite: 1]
    let mut display_img = Mat::default();
    imgproc::cvt_color(
        &frame,
        &mut display_img,
        imgproc::COLOR_GRAY2BGR,
        0,
        core::AlgorithmHint::ALGO_HINT_DEFAULT,
    )?;

    let mut box_pts_mat = Mat::default();
    box_points(pocket_rect, &mut box_pts_mat)?;
    let box_pts = box_pts_mat.reshape_def(Point2f::opencv_channels())?;

    for i in 0..4 {
        let pt1 = *box_pts.at::<Point2f>(i)?;
        let pt2 = *box_pts.at::<Point2f>((i + 1) % 4)?;

        imgproc::line(
            &mut display_img,
            Point2i::new(pt1.x.round() as i32, pt1.y.round() as i32),
            Point2i::new(pt2.x.round() as i32, pt2.y.round() as i32),
            opencv::core::Scalar::new(0.0, 255.0, 0.0, 0.0),
            2,
            imgproc::LINE_AA,
            0,
        )?;
    }

    imgproc::circle(
        &mut display_img,
        Point2i::new(center.x.round() as i32, center.y.round() as i32),
        4,
        Scalar::new(0.0, 0.0, 255.0, 0.0),
        -1,
        imgproc::LINE_AA,
        0,
    )?;

    highgui::imshow("Populated Pocket Detected", &display_img)?;
    // highgui::wait_key(3000)?;
    highgui::destroy_all_windows()?;

    Ok(Some(Pose {
        position: Point {
            x: center.x.round(),
            y: center.y.round(),
        },
        angle_deg: pocket_rect.angle,
    }))
}
