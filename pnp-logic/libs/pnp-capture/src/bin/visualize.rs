use opencv::{imgcodecs, prelude::*};

fn main() -> opencv::Result<()> {
    for path in &[
        "images/feeder-ident-clear-USB1.jpg",
        "images/feeder-ident-clear-USB2.jpg",
    ] {
        println!("\n=== {} ===", path);
        let frame = imgcodecs::imread(path, imgcodecs::IMREAD_GRAYSCALE)?;
        let cols = frame.cols() as i32;
        let rows = frame.rows() as i32;
        println!("Image: {}x{}", cols, rows);

        // Scan rows for dark runs around y=240
        for y in (220..280).step_by(4) {
            let row = frame.at_row::<u8>(y)?;
            // Count dark pixels and their x positions in middle range
            let dark: Vec<(usize, u8)> = row
                .iter()
                .enumerate()
                .filter(|(x, _)| *x >= 200 && *x <= 600)
                .filter(|&(_, &v)| v < 80)
                .map(|(x, &v)| (x, v))
                .collect();
            if dark.len() > 5 {
                let xs: Vec<usize> = dark.iter().map(|(x, _)| *x).collect();
                let min = *dark.iter().map(|(_, v)| v).min().unwrap();
                let max = *dark.iter().map(|(_, v)| v).max().unwrap();
                println!(
                    "y={:3}: n={} xs(0,1,2..)={:?}..{:?} v=[{},{}]",
                    y,
                    dark.len(),
                    xs.iter().take(5).collect::<Vec<_>>(),
                    xs.iter()
                        .rev()
                        .take(5)
                        .collect::<Vec<_>>()
                        .into_iter()
                        .rev()
                        .collect::<Vec<_>>(),
                    min,
                    max
                );
            }
        }
    }
    Ok(())
}
