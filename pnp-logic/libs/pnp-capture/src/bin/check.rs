use pnp_capture::populated_pocket_detection;

fn main() -> opencv::Result<()> {
    for path in &[
        "images/feeder-ident-black.jpg",
        "images/feeder-ident-black-led.jpg",
        "images/feeder-ident-clear-cap-EMPTY.jpg",
        "images/feeder-ident-clear-SS14-EMPTY.jpg",
        "images/feeder-ident-clear-SS14-led.jpg",
        "images/feeder-ident-white-0402.jpg",
        "images/feeder-ident-white.jpg",
    ] {
        match populated_pocket_detection(path) {
            Ok(Some(pose)) => println!("{}: {:?}", path, pose),
            Ok(None) => println!("{}: NO POCKET DETECTED", path),
            Err(e) => println!("{}: ERROR {}", path, e),
        }
    }
    Ok(())
}
