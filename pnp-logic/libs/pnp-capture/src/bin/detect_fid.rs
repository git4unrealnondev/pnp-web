use opencv::core::Point2f;
use opencv::{highgui, imgcodecs};
use pnp_capture::{detect_fiducial_with_diameter_near, render_fiducial_debug};

fn main() -> opencv::Result<()> {
    let mut args = std::env::args().skip(1);
    let path = args
        .next()
        .unwrap_or_else(|| "images/fid/bot-black.jpg".to_owned());
    let mut output = "fiducial-debug.jpg".to_owned();
    let mut show = false;
    let mut expected = None;
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--show" => show = true,
            "--output" => {
                output = args
                    .next()
                    .ok_or_else(|| opencv::Error::new(1, "--output requires a file path"))?;
            }
            "--expected" => {
                let x = args
                    .next()
                    .ok_or_else(|| opencv::Error::new(1, "--expected requires x y"))?;
                let y = args
                    .next()
                    .ok_or_else(|| opencv::Error::new(1, "--expected requires x y"))?;
                expected = Some(Point2f::new(
                    x.parse()
                        .map_err(|_| opencv::Error::new(1, "invalid expected x"))?,
                    y.parse()
                        .map_err(|_| opencv::Error::new(1, "invalid expected y"))?,
                ));
            }
            _ => return Err(opencv::Error::new(1, format!("Unknown argument: {arg}"))),
        }
    }

    let detection = detect_fiducial_with_diameter_near(&path, 8.0, 80.0, expected)?;
    match detection {
        Some(detection) => println!(
            "center=({:.2},{:.2}) radius={:.2}px polarity={} confidence={:.2}",
            detection.center.x,
            detection.center.y,
            detection.radius_px,
            detection.polarity,
            detection.confidence
        ),
        None => println!("no fiducial detected"),
    }

    let debug = render_fiducial_debug(&path, detection)?;
    if !imgcodecs::imwrite(&output, &debug, &opencv::core::Vector::new())? {
        return Err(opencv::Error::new(1, "Failed to write debug image"));
    }
    println!("debug view: {output}");

    if show {
        highgui::imshow("Fiducial Debug", &debug)?;
        highgui::wait_key(0)?;
        highgui::destroy_window("Fiducial Debug")?;
    }
    Ok(())
}
