use opencv::{
    Result,
    core::{self, Mat, Point2f, Scalar, Vector},
    geometry::min_area_rect,
    imgcodecs, imgproc,
    prelude::*,
};

fn detect(path: &str, closing_kernel: i32, min_area: f32, max_area: f32) -> Result<()> {
    let frame = imgcodecs::imread(path, imgcodecs::IMREAD_GRAYSCALE)?;
    let cols = frame.cols();
    let rows = frame.rows();
    println!(
        "\n=== {} (close_k={}, area=[{},{}]) ===",
        path, closing_kernel, min_area, max_area
    );

    let mut frame_blur = Mat::default();
    imgproc::median_blur(&frame, &mut frame_blur, 3)?;

    let inner_rect = core::Rect::new(15, 15, (cols - 30).max(1), (rows - 30).max(1));
    let inner_roi = Mat::roi(&frame_blur, inner_rect)?;
    let mean_scalar = core::mean(&inner_roi, &core::no_array())?;
    let coarse_thresh_val = mean_scalar[0] * 0.8;

    let mut frame_thresh = Mat::default();
    imgproc::threshold(
        &frame_blur,
        &mut frame_thresh,
        coarse_thresh_val,
        255.0,
        imgproc::THRESH_BINARY,
    )?;

    if closing_kernel > 0 {
        let kernel = imgproc::get_structuring_element(
            imgproc::MORPH_RECT,
            core::Size::new(closing_kernel, closing_kernel),
            core::Point::new(-1, -1),
        )?;
        let mut closed = Mat::default();
        imgproc::morphology_ex(
            &frame_thresh,
            &mut closed,
            imgproc::MORPH_CLOSE,
            &kernel,
            core::Point::new(-1, -1),
            1,
            core::BORDER_CONSTANT,
            Scalar::new(0.0, 0.0, 0.0, 0.0),
        )?;
        frame_thresh = closed;
    }

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

    let cx_ref = cols as f32 / 2.0;
    let cy_ref = rows as f32 / 2.0;

    let mut cands: Vec<(f32, f32, f32, f32, f32, f32, i32, i32)> = Vec::new();
    for i in 0..contours.len() {
        let contour = contours.get(i)?;
        if contour.len() < 10 {
            continue;
        }
        let h_elem = *hierarchy.at_2d::<core::Vec4i>(0, i.try_into().unwrap())?;
        let first_child = h_elem[2];
        let rot_rect = min_area_rect(&contour)?;
        let area = rot_rect.size.width * rot_rect.size.height;
        let (child_area, _child_rect) = if first_child != -1 {
            let cc = contours.get(first_child as usize)?;
            let cr = min_area_rect(&cc)?;
            (cr.size.width * cr.size.height, cr)
        } else {
            (0.0, rot_rect)
        };
        let dx = rot_rect.center.x - cx_ref;
        let dy = rot_rect.center.y - cy_ref;
        let score = (dx * dx + dy * dy) as i32;
        cands.push((
            rot_rect.center.x,
            rot_rect.center.y,
            rot_rect.size.width,
            rot_rect.size.height,
            area,
            child_area,
            first_child,
            score,
        ));
    }
    cands.sort_by_key(|c| c.7);
    for c in cands.iter().take(8) {
        if c.4 < min_area || c.4 > max_area {
            continue;
        }
        println!(
            "  center=({:.1},{:.1}) pocket={:.0}x{:.0} (area={:.0}) child_area={:.0} child={} score={}",
            c.0, c.1, c.2, c.3, c.4, c.5, c.6, c.7
        );
    }
    Ok(())
}

fn main() -> Result<()> {
    for &(k, lo, hi) in &[
        (0, 800.0, 8000.0_f32),
        (5, 500.0, 8000.0),
        (9, 500.0, 8000.0),
        (15, 500.0, 8000.0),
    ] {
        detect("images/feeder-ident-black.jpg", k, lo, hi)?;
        detect("images/feeder-ident.jpg", k, lo, hi)?;
    }
    Ok(())
}
