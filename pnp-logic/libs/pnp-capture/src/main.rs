use opencv::{
    Result,
    core::{self, Point2i, Scalar},
    highgui, imgcodecs, imgproc,
    prelude::*,
};
use pnp_capture::{
    Part, Size, detect_circular_symmetry, detect_sprocket_holes_filtered,
    detect_sprocket_holes_filtered_with_size, populated_pocket_detection,
    populated_pocket_detection_with_size,
};

fn main() -> Result<()> {
    // 1. Load your test image from disk

    // Assume you have an instance of your `Part` struct loaded (e.g., via serde/JSON)
    let part: Part = Part {
        id: 0,
        mpn: "".into(),
        description: "".into(),
        footprint: pnp_capture::Footprint {
            name: "0603".into(),
            centroid: pnp_capture::Point { x: 0.0, y: 0.0 },
            size: pnp_capture::Size {
                width: 5.0,
                height: 5.0,
            },
            pads: vec![],
            polarity: pnp_capture::Polarity::None,
        },
        nozzle_type: vec!["503".into()],
        pickup_rotation_deg: 0.0,
        pickup_offset: pnp_capture::Point { x: 0.0, y: 0.0 },
    };

    // Calibration factor (e.g., 50 pixels per mm)
    let px_per_mm = 20.0;
    // User-tunable component dimensions. The travel direction is height.
    let component_size_mm = part.footprint.size;

    let test_paths = [
        /*     "images/feeder-ident-black-s0850-EMPTY.jpg",
        "images/feeder-ident-white.jpg",
        "images/feeder-ident-white-0402.jpg",
        "images/feeder-ident-clear-cap-EMPTY.jpg",
        "images/feeder-ident-clear-SS14-EMPTY.jpg",
        "images/feeder-ident-clear-USB1.jpg",
        "images/feeder-ident-clear-USB2.jpg",*/
        "images/fid/top-black.jpg",
    ];

    // User-tunable rectangle limits in pixels. The example minimum is the
    // expected 9.0 mm x 8.0 mm part at 20 px/mm; max values allow the outer
    // feeder pocket to be selected when the contour is larger than the part.
    let min_rect_width = 3.0 * px_per_mm;
    let min_rect_height = 3.0 * px_per_mm;
    let max_rect_width = 14.0 * px_per_mm;
    let max_rect_height = 13.0 * px_per_mm;

    for test_path in test_paths {
        // dbg!(populated_pocket_detection(test_path));
        /*  let pose = detect_sprocket_holes_filtered_with_size(
            test_path,
            240.0,
            360.0,
            Some(component_size_mm),
            px_per_mm,
        )?;*/

        let pose = detect_circular_symmetry(
            test_path,
            15.0,
            40.0,
            Some(core::Rect::new(220, 0, 220, 576)),
            10,
            1.2,
        );

        dbg!(pose);
    }

    Ok(())
}
