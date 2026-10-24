use opencv::{core, imgcodecs, imgproc, prelude::*};

fn main() -> opencv::Result<()> {
    for path in &["images/feeder-ident-black.jpg", "images/feeder-ident.jpg"] {
        let frame = imgcodecs::imread(path, imgcodecs::IMREAD_GRAYSCALE)?;
        let cols = frame.cols() as i32;
        let rows = frame.rows() as i32;

        let mut frame_blur = Mat::default();
        imgproc::median_blur(&frame, &mut frame_blur, 3)?;

        let inner_rect = core::Rect::new(15, 15, (cols - 30).max(1), (rows - 30).max(1));
        let inner_roi = Mat::roi(&frame_blur, inner_rect)?;
        let mean_scalar = core::mean(&inner_roi, &core::no_array())?;
        let coarse_thresh_val = mean_scalar[0] * 0.8;

        println!(
            "{}: inner mean={:.1}, threshold={:.1}",
            path, mean_scalar[0], coarse_thresh_val
        );

        let mut frame_thresh = Mat::default();
        imgproc::threshold(
            &frame_blur,
            &mut frame_thresh,
            coarse_thresh_val,
            255.0,
            imgproc::THRESH_BINARY,
        )?;

        // Check pixel at (378, 367) in thresh
        if 367 < rows && 378 < cols {
            let orig = *frame.at_2d::<u8>(367, 378).unwrap();
            let blurred = *frame_blur.at_2d::<u8>(367, 378).unwrap();
            let threshed = *frame_thresh.at_2d::<u8>(367, 378).unwrap();
            println!(
                "  pixel at (378, 367): orig={}, blurred={}, threshed={}",
                orig, blurred, threshed
            );
        }
    }
    Ok(())
}
