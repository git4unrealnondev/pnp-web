use opencv::{core, imgcodecs, imgproc, prelude::*};

fn main() -> opencv::Result<()> {
    let path = "images/feeder-ident-black.jpg";
    let frame = imgcodecs::imread(path, imgcodecs::IMREAD_GRAYSCALE)?;
    let cols = frame.cols() as i32;
    let rows = frame.rows() as i32;
    println!("Image: {}x{} for {}", cols, rows, path);

    // Print 1D intensity trace down x=378 (target x)
    let col: Vec<u8> = (0..rows)
        .map(|y| *frame.at_2d::<u8>(y as i32, 378).unwrap())
        .collect();
    // Find dark runs in col
    let mut runs: Vec<(usize, usize)> = Vec::new();
    let mut start = 0;
    let mut in_dark = false;
    for (y, &v) in col.iter().enumerate() {
        if v < 80 && !in_dark {
            start = y;
            in_dark = true;
        } else if v >= 80 && in_dark {
            runs.push((start, y));
            in_dark = false;
        }
    }
    if in_dark {
        runs.push((start, col.len()));
    }
    println!("Dark runs at x=378 (v<80):");
    for (a, b) in &runs {
        if b - a > 3 {
            println!("  y={}..{} (height={})", a, b, b - a);
        }
    }

    // Same for x=375
    let col: Vec<u8> = (0..rows)
        .map(|y| *frame.at_2d::<u8>(y as i32, 375).unwrap())
        .collect();
    let mut runs: Vec<(usize, usize)> = Vec::new();
    let mut start = 0;
    let mut in_dark = false;
    for (y, &v) in col.iter().enumerate() {
        if v < 80 && !in_dark {
            start = y;
            in_dark = true;
        } else if v >= 80 && in_dark {
            runs.push((start, y));
            in_dark = false;
        }
    }
    if in_dark {
        runs.push((start, col.len()));
    }
    println!("Dark runs at x=375 (v<80):");
    for (a, b) in &runs {
        if b - a > 3 {
            println!("  y={}..{} (height={})", a, b, b - a);
        }
    }
    Ok(())
}
