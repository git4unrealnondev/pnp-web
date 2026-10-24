use pnp_capture::{PocketHint, Point, Size, populated_pocket_detection_with_hint};

struct Case {
    path: &'static str,
    expected: (f32, f32),
    pocket_mm: (f32, f32),
    tolerance: f32,
    /// If true, the pocket is expected to be EMPTY (no component). Function should
    /// still try to find the empty socket, but a failure here is acceptable.
    empty: bool,
}

fn main() -> opencv::Result<()> {
    let cases = [
        Case {
            path: "images/feeder-ident-black-5050-EMPTY.jpg",
            expected: (367.0, 288.0),
            pocket_mm: (5.0, 5.0),
            tolerance: 5.0,
            empty: true,
        },
        Case {
            path: "images/feeder-ident-black-5050-led.jpg",
            expected: (367.0, 288.0),
            pocket_mm: (5.0, 5.0),
            tolerance: 5.0,
            empty: false,
        },
        Case {
            path: "images/feeder-ident-black-s0850-EMPTY.jpg",
            expected: (360.0, 290.0),
            pocket_mm: (4.9, 4.5),
            tolerance: 5.0,
            empty: true,
        },
        Case {
            path: "images/feeder-ident-black-s0850.jpg",
            expected: (360.0, 290.0),
            pocket_mm: (4.0, 4.0),
            tolerance: 5.0,
            empty: false,
        },
        Case {
            path: "images/feeder-ident-black-SS14.jpg",
            expected: (360.0, 290.0),
            pocket_mm: (4.2, 2.5),
            tolerance: 5.0,
            empty: false,
        },
        Case {
            path: "images/feeder-ident-black-SOT23.jpg",
            expected: (378.0, 286.0),
            pocket_mm: (3.0, 2.5),
            tolerance: 5.0,
            empty: false,
        },
        Case {
            path: "images/feeder-ident-black-led.jpg",
            expected: (385.0, 286.0),
            pocket_mm: (1.55, 0.9),
            tolerance: 5.0,
            empty: false,
        },
        Case {
            path: "images/feeder-ident-clear-cap-EMPTY.jpg",
            expected: (388.0, 281.0),
            pocket_mm: (3.2, 2.0),
            tolerance: 5.0,
            empty: true,
        },
        Case {
            path: "images/feeder-ident-clear-cap.jpg",
            expected: (388.0, 281.0),
            pocket_mm: (3.2, 2.0),
            tolerance: 5.0,
            empty: false,
        },
        Case {
            path: "images/feeder-ident-clear-SS14-EMPTY.jpg",
            expected: (365.0, 288.0),
            pocket_mm: (4.0, 2.5),
            tolerance: 5.0,
            empty: true,
        },
        Case {
            path: "images/feeder-ident-clear-SS14-led.jpg",
            expected: (388.0, 288.0),
            pocket_mm: (4.0, 2.5),
            tolerance: 5.0,
            empty: false,
        },
        Case {
            path: "images/feeder-ident-clear-USB1.jpg",
            expected: (360.0, 240.0),
            pocket_mm: (9.0, 8.0),
            tolerance: 5.0,
            empty: false,
        },
        Case {
            path: "images/feeder-ident-clear-USB2.jpg",
            expected: (360.0, 240.0),
            pocket_mm: (9.0, 8.0),
            tolerance: 5.0,
            empty: false,
        },
        Case {
            path: "images/feeder-ident-white-0402.jpg",
            expected: (388.0, 287.0),
            pocket_mm: (0.6, 0.35),
            tolerance: 5.0,
            empty: false,
        },
        Case {
            path: "images/feeder-ident-white.jpg",
            expected: (375.0, 287.0),
            pocket_mm: (1.6, 0.8),
            tolerance: 5.0,
            empty: false,
        },
    ];

    let mut failed = 0;
    for case in &cases {
        let result = match populated_pocket_detection_with_hint(
            case.path,
            PocketHint {
                size_mm: Size {
                    width: case.pocket_mm.0,
                    height: case.pocket_mm.1,
                },
                position_tolerance_mm: 1.0,
                size_tolerance_mm: 0.4,
                px_per_mm: 20.0,
                pickup_offset_mm: Point::ORIGIN,
                center_px: Some(Point::new(case.expected.0, case.expected.1)),
            },
        ) {
            Ok(Some(pose)) => pose,
            Ok(None) => {
                if case.empty {
                    println!("SKIP {}: no pocket detected (EMPTY, expected)", case.path);
                } else {
                    println!("FAIL {}: no pocket detected", case.path);
                    failed += 1;
                }
                continue;
            }
            Err(e) => {
                println!("FAIL {}: error {}", case.path, e);
                failed += 1;
                continue;
            }
        };
        let dx = (result.position.x - case.expected.0).abs();
        let dy = (result.position.y - case.expected.1).abs();
        let ok = dx <= case.tolerance && dy <= case.tolerance;
        let status = if ok { "OK  " } else { "FAIL" };
        println!(
            "{} {}: got ({:.1},{:.1}), expected ({:.1},{:.1}), dx={:.2} dy={:.2}",
            status,
            case.path,
            result.position.x,
            result.position.y,
            case.expected.0,
            case.expected.1,
            dx,
            dy
        );
        if !ok {
            failed += 1;
        }
    }
    println!("\n{} failed of {} cases", failed, cases.len());
    if failed > 0 {
        std::process::exit(1);
    }
    Ok(())
}
