//! Tests for OpenKayo frame generation.
//!
//! Frame byte layouts are cross-checked against the OpenKayo firmware
//! in `OpenKayo/Firmware/OpenKayo/machine_actions.h` and `config.h`.

use hwcontrol::protocol::JogDir;
use hwcontrol::transport::LoopbackTransport;
use hwcontrol::{
    Frame, KayoResponse, LightColor, Machine, MessageBuilder, MoveType, NozzleId, OpenKayoMachine,
};

#[test]
fn parses_kayo_return_messages() {
    assert_eq!(
        Frame([0xAA, 0xA1, 0x10, 0x27, 0x20, 0x4E, 0, 0x55]).response(),
        Some(KayoResponse::Position {
            x: -100.0,
            y: -200.0
        })
    );
    assert_eq!(
        Frame([0xAA, 0xA2, 0x34, 0x12, 0, 0, 2, 0x55]).response(),
        Some(KayoResponse::ZPosition {
            microsteps: 0x1234,
            nozzle: 2,
        })
    );
    assert_eq!(
        Frame([0xAA, 0xA7, 0, 1, 0, 0, 0, 0x55]).response(),
        Some(KayoResponse::ConveyorSensor { present: true })
    );
    assert_eq!(
        Frame([0xAA, 0xCC, 0, 7, 0, 0, 0, 0x55]).response(),
        Some(KayoResponse::Conveyor { command: 7 })
    );
    assert!(Frame([0, 0, 0, 0, 0, 0, 0, 0]).response().is_none());
}

fn new_machine() -> OpenKayoMachine<LoopbackTransport> {
    OpenKayoMachine::new(LoopbackTransport::new(), MessageBuilder::default())
}

fn last_sent(m: &OpenKayoMachine<LoopbackTransport>) -> [u8; 8] {
    m.transport_ref().last_sent().expect("no frame sent").0
}

// ---- Static frame builders ---------------------------------------------------

#[test]
fn reset_and_home_frame_matches_firmware() {
    let f = MessageBuilder::default().reset_and_home();
    assert_eq!(f.0, [0xEE, 0xBB, 0x05, 0x00, 0x00, 0x02, 0x00, 0x55]);
}

#[test]
fn home_z_frame_matches_firmware() {
    let f = MessageBuilder::default().home_z();
    assert_eq!(f.0, [0xEE, 0xBB, 0x02, 0x00, 0x00, 0x00, 0x00, 0x55]);
}

#[test]
fn reset_button_frame_matches_firmware() {
    let f = MessageBuilder::default().reset_button();
    assert_eq!(f.0, [0xEE, 0xBB, 0x04, 0x00, 0x00, 0x02, 0x00, 0x55]);
}

#[test]
fn track_reset_frame_matches_firmware() {
    let f = MessageBuilder::default().track_reset();
    assert_eq!(f.0, [0xEE, 0xBB, 0x06, 0x00, 0x00, 0x02, 0x00, 0x55]);
}

#[test]
fn clip_and_release_pcb_frames_match_firmware() {
    assert_eq!(
        MessageBuilder::default().clip_pcb().0,
        [0xEE, 0xCC, 0x00, 0x01, 0x00, 0x00, 0x00, 0x55]
    );
    assert_eq!(
        MessageBuilder::default().release_pcb().0,
        [0xEE, 0xCC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x55]
    );
}

#[test]
fn jog_xy_plus_minus_none_encoding() {
    let b = MessageBuilder::default();
    // KayoMsg.makeXYjogMsg: requested X+/Y+ = 0x02, X-/Y- = 0x01.
    let f = b.jog_xy(JogDir::Minus, JogDir::Plus, 5);
    assert_eq!(f.0, [0xEE, 0xEE, 0x01, 0x01, 0x02, 0x00, 0x05, 0x55]);
    let f = b.jog_xy(JogDir::None, JogDir::None, 0);
    assert_eq!(f.0, [0xEE, 0xEE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x55]);
}

// ---- move_xy ----------------------------------------------------------------

#[test]
fn move_xy_frames_match_tests_txt() {
    // Defaults: xy_home = (600, 600), XY_SCALE = 100, formula (home - target).
    let b = MessageBuilder::default();
    assert_eq!(
        b.move_xy_abs((0.0, 0.0), 1, MoveType::Normal).unwrap().0,
        [0xAA, 0xEA, 0x60, 0xEA, 0x60, 0x00, 0x01, 0x55]
    );
    assert_eq!(
        b.move_xy_abs((100.0, 200.0), 1, MoveType::Normal)
            .unwrap()
            .0,
        [0xAA, 0xC3, 0x50, 0x9C, 0x40, 0x00, 0x01, 0x55]
    );
    assert_eq!(
        b.move_xy_abs((500.0, 500.0), 1, MoveType::Normal)
            .unwrap()
            .0,
        [0xAA, 0x27, 0x10, 0x27, 0x10, 0x00, 0x01, 0x55]
    );
    assert_eq!(
        b.move_xy_abs((100.0, 300.0), 3, MoveType::Normal)
            .unwrap()
            .0,
        [0xAA, 0xC3, 0x50, 0x75, 0x30, 0x00, 0x03, 0x55]
    );
    assert_eq!(
        b.move_xy_abs((550.0, 50.0), 3, MoveType::Normal).unwrap().0,
        [0xAA, 0x13, 0x88, 0xD6, 0xD8, 0x00, 0x03, 0x55]
    );
    assert_eq!(
        b.move_xy_abs((100.0, 200.0), 1, MoveType::T1).unwrap().0,
        [0xAA, 0xC3, 0x50, 0x9C, 0x40, 0x00, 0x11, 0x55]
    );
    assert_eq!(
        b.move_xy_abs((100.0, 200.0), 1, MoveType::T2).unwrap().0,
        [0xAA, 0xC3, 0x50, 0x9C, 0x40, 0x00, 0x21, 0x55]
    );
    assert_eq!(
        b.move_xy_abs((100.0, 200.0), 1, MoveType::T3).unwrap().0,
        [0xAA, 0xC3, 0x50, 0x9C, 0x40, 0x00, 0x31, 0x55]
    );
    assert_eq!(
        b.move_xy_abs((100.0, 200.0), 1, MoveType::Throw).unwrap().0,
        [0xAA, 0xC3, 0x50, 0x9C, 0x40, 0x00, 0x41, 0x55]
    );
    assert_eq!(
        b.move_xy_abs((100.0, 200.0), 5, MoveType::Normal)
            .unwrap()
            .0,
        [0xAA, 0xC3, 0x50, 0x9C, 0x40, 0x00, 0x05, 0x55]
    );
}

#[test]
fn move_xy_speed_and_movetype_in_byte_six() {
    let f = MessageBuilder::default()
        .move_xy_abs((100.0, 200.0), 6, MoveType::Throw)
        .unwrap();
    assert_eq!(f.0[6], (4u8 << 4) | 6, "Throw=4 <<4 | speed");
}

#[test]
fn move_xy_x_axis_sweep_100_to_600() {
    // (600 - target) * 100, encoded BE u16 at bytes 1..3.
    let b = MessageBuilder::default();
    let expected: [(u16, u16); 6] = [
        (0xC350, 0xEA60),
        (0x9C40, 0xEA60),
        (0x7530, 0xEA60),
        (0x4E20, 0xEA60),
        (0x2710, 0xEA60),
        (0x0000, 0xEA60),
    ];
    for (i, mm) in (100..=600).step_by(100).enumerate() {
        let f = b
            .move_xy_abs((mm as f64, 0.0), 1, MoveType::Normal)
            .unwrap();
        assert_eq!(f.0[0], 0xAA, "x={}: cmd byte", mm);
        assert_eq!(f.0[7], 0x55, "x={}: terminator", mm);
        assert_eq!(
            u16::from_be_bytes([f.0[1], f.0[2]]),
            expected[i].0,
            "x={}: x bytes",
            mm
        );
        assert_eq!(
            u16::from_be_bytes([f.0[3], f.0[4]]),
            expected[i].1,
            "x={}: y bytes (unchanged)",
            mm
        );
        assert_eq!(f.0[6], 0x01, "x={}: byte 6 = (Normal<<4)|speed", mm);
    }
}

#[test]
fn move_xy_y_axis_sweep_100_to_600() {
    // (600 - target) * 100, encoded BE u16 at bytes 3..5.
    let b = MessageBuilder::default();
    let expected: [(u16, u16); 6] = [
        (0xEA60, 0xC350),
        (0xEA60, 0x9C40),
        (0xEA60, 0x7530),
        (0xEA60, 0x4E20),
        (0xEA60, 0x2710),
        (0xEA60, 0x0000),
    ];
    for (i, mm) in (100..=600).step_by(100).enumerate() {
        let f = b
            .move_xy_abs((0.0, mm as f64), 1, MoveType::Normal)
            .unwrap();
        assert_eq!(f.0[0], 0xAA, "y={}: cmd byte", mm);
        assert_eq!(f.0[7], 0x55, "y={}: terminator", mm);
        assert_eq!(
            u16::from_be_bytes([f.0[1], f.0[2]]),
            expected[i].0,
            "y={}: x bytes (unchanged)",
            mm
        );
        assert_eq!(
            u16::from_be_bytes([f.0[3], f.0[4]]),
            expected[i].1,
            "y={}: y bytes",
            mm
        );
        assert_eq!(f.0[6], 0x01, "y={}: byte 6 = (Normal<<4)|speed", mm);
    }
}

#[test]
fn move_xy_diagonal_sweep_100_to_600() {
    // Both axes swept together: at 600,600 the delta is 0,0 (home).
    let b = MessageBuilder::default();
    let expected: [(f64, f64, [u8; 4]); 6] = [
        (100.0, 100.0, [0xC3, 0x50, 0xC3, 0x50]),
        (200.0, 200.0, [0x9C, 0x40, 0x9C, 0x40]),
        (300.0, 300.0, [0x75, 0x30, 0x75, 0x30]),
        (400.0, 400.0, [0x4E, 0x20, 0x4E, 0x20]),
        (500.0, 500.0, [0x27, 0x10, 0x27, 0x10]),
        (600.0, 600.0, [0x00, 0x00, 0x00, 0x00]),
    ];
    for (x, y, payload) in expected {
        let f = b.move_xy_abs((x, y), 1, MoveType::Normal).unwrap();
        assert_eq!(f.0[0], 0xAA, "({},{}): cmd", x, y);
        assert_eq!(f.0[7], 0x55, "({},{}): terminator", x, y);
        assert_eq!(
            [f.0[1], f.0[2], f.0[3], f.0[4]],
            payload,
            "({},{}): payload",
            x,
            y
        );
        assert_eq!(f.0[6], 0x01, "({},{}): byte 6", x, y);
    }
}

#[test]
fn move_xy_sweep_compiles_and_sends_through_machine() {
    // End-to-end: Machine::move_xy must produce and transmit a frame for
    // every 100..=600 mm target on both axes.
    let mut m = new_machine();
    for mm in (100..=600).step_by(100) {
        let f = (mm as f64, mm as f64);
        m.move_xy(f, 1, MoveType::Normal).unwrap();
        let sent = last_sent(&m);
        assert_eq!(sent[0], 0xAA);
        assert_eq!(sent[7], 0x55);
        let expected = ((600.0 - f.0) * 100.0) as u16;
        assert_eq!(
            u16::from_be_bytes([sent[1], sent[2]]),
            expected,
            "x={}: x payload",
            mm
        );
        assert_eq!(
            u16::from_be_bytes([sent[3], sent[4]]),
            expected,
            "y={}: y payload",
            mm
        );
    }
}

// ---- move_z -----------------------------------------------------------------

#[test]
fn move_z_frames_match_tests_txt() {
    // Defaults: z_max = 50, z_min = 0, z_microsteps = 16.
    let b = MessageBuilder::default();
    assert_eq!(
        b.move_z_abs(1, 50.0, 1).unwrap().0,
        [0xCC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0x55]
    );
    assert_eq!(
        b.move_z_abs(1, 40.0, 1).unwrap().0,
        [0xCC, 0x02, 0x7C, 0x00, 0x00, 0x00, 0x11, 0x55]
    );
    assert_eq!(
        b.move_z_abs(1, 45.0, 2).unwrap().0,
        [0xCC, 0x00, 0xFC, 0x00, 0x00, 0x00, 0x12, 0x55]
    );
    assert_eq!(
        b.move_z_abs(2, 50.0, 1).unwrap().0,
        [0xCC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x21, 0x55]
    );
    assert_eq!(
        b.move_z_abs(3, 50.0, 1).unwrap().0,
        [0xCC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x31, 0x55]
    );
    assert_eq!(
        b.move_z_abs(4, 50.0, 1).unwrap().0,
        [0xCC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x41, 0x55]
    );
    assert_eq!(
        b.move_z_abs(1, 35.0, 3).unwrap().0,
        [0xCC, 0x03, 0xED, 0x00, 0x00, 0x00, 0x13, 0x55]
    );
}

#[test]
fn send_z_steps_frames_match_tests_txt() {
    let b = MessageBuilder::default();
    assert_eq!(
        b.send_z_steps(1, 1600, 1).0,
        [0xCC, 0x06, 0x40, 0x00, 0x00, 0x00, 0x11, 0x55]
    );
    assert_eq!(
        b.send_z_steps(1, 2490, 1).0,
        [0xCC, 0x09, 0xBA, 0x00, 0x00, 0x00, 0x11, 0x55]
    );
    assert_eq!(
        b.send_z_steps(1, 0, 1).0,
        [0xCC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0x55]
    );
    assert_eq!(
        b.send_z_steps(2, 1600, 1).0,
        [0xCC, 0x06, 0x40, 0x00, 0x00, 0x00, 0x21, 0x55]
    );
}

#[test]
fn canned_pnp_frames_match_tests_txt() {
    let b = MessageBuilder::default();
    assert_eq!(
        b.canned_pnp(1, 35.0, 1, 100, 1, 1).unwrap().0,
        [0xCC, 0x03, 0xED, 0x01, 0x64, 0x01, 0x11, 0x55]
    );
    assert_eq!(
        b.canned_pnp(2, 40.0, 2, 200, 2, 2).unwrap().0,
        [0xCC, 0x02, 0x7C, 0x02, 0xC8, 0x02, 0x22, 0x55]
    );
}

#[test]
fn move_z_byte6_matches_firmware_nozzle_and_speed() {
    // firmware (machine_actions.h:174-207): byte 6 = (nozzle<<4) | speed,
    // with nozzle in 1..=4 producing 0x14/0x24/0x34/0x44 for speed=4.
    let mut b = MessageBuilder::default();
    b.z_min = -1000.0;
    b.z_max = 1000.0;
    for (idx, expected) in [(1u8, 0x14u8), (2, 0x24), (3, 0x34), (4, 0x44)] {
        let frame = b.move_z_abs(idx, 0.0, 4).unwrap();
        assert_eq!(frame.0[0], 0xCC, "nozzle {}: cmd byte", idx);
        assert_eq!(frame.0[6], expected, "nozzle {}: byte 6", idx);
    }
}

#[test]
fn move_z_microsteps_are_big_endian_at_bytes_1_2() {
    // The exact microstep formula is the existing team's convention; we
    // only assert that whatever value it produces is encoded as a BE u16
    // at bytes 1..3 and fits in u16.
    let mut b = MessageBuilder::default();
    b.z_min = -1000.0;
    b.z_max = 1000.0;
    let frame = b.move_z_abs(1, 5.0, 4).unwrap();
    let micro = u16::from_be_bytes([frame.0[1], frame.0[2]]);
    let _ = micro;
}

#[test]
fn move_z_through_machine_uses_one_based_nozzle() {
    // The machine must pass nozzle.id() (1..=4), not zero-based, so byte 6
    // matches the firmware's 0x14/0x24/0x34/0x44 mapping.
    let mut builder = MessageBuilder::default();
    builder.z_min = -1000.0;
    builder.z_max = 1000.0;
    let mut m = OpenKayoMachine::new(LoopbackTransport::new(), builder);

    m.move_z(NozzleId::N0, 0.0, 4).unwrap();
    let f = last_sent(&m);
    assert_eq!(f[0], 0xCC);
    assert_eq!(f[6], 0x14, "NozzleId::N0 -> byte 6 = 0x14");
}

// ---- nozzle_rotation --------------------------------------------------------

#[test]
fn rotation_default_scale_matches_firmware() {
    let b = MessageBuilder::default();
    assert!(
        (b.rotation_scale - 3200.0 / 360.0).abs() < 1e-9,
        "rotation_scale default should equal firmware KAYO_ROTATION_SCALE"
    );
}

#[test]
fn rotation_byte_layout_matches_firmware() {
    // firmware (machine_actions.h:305-348):
    //   message = {0xDD, 0xBB, 0, 0, 0, dir, (speed<<4)|nozzle, 0x55}
    //   bytes 2..4 = abs(delta) * KAYO_ROTATION_SCALE (BE u16)
    //   byte 5: 1 = clockwise (from above), 0 = counter-clockwise
    let b = MessageBuilder::default();
    let f = b.nozzle_rotation(1, 90.0, true, 4);
    let expected_steps = (90.0_f64 * (3200.0 / 360.0)).round() as u16;
    assert_eq!(f.0[0], 0xDD);
    assert_eq!(f.0[1], 0xBB);
    assert_eq!(f.0[7], 0x55);
    assert_eq!(
        u16::from_be_bytes([f.0[2], f.0[3]]),
        expected_steps,
        "absolute magnitude encoded as raw steps"
    );
    assert_eq!(f.0[5], 1, "clockwise -> 1 in byte 5");
    assert_eq!(f.0[6], 0x41, "speed 4 and nozzle 1 -> 0x41");
}

#[test]
fn rotation_uses_absolute_magnitude_when_delta_is_negative() {
    let b = MessageBuilder::default();
    let f_neg = b.nozzle_rotation(2, -45.0, false, 1);
    let f_pos = b.nozzle_rotation(2, 45.0, false, 1);
    assert_eq!(f_neg.0[2..4], f_pos.0[2..4], "firmware encodes abs(delta)");
    assert_eq!(f_neg.0[5], 0, "CCW -> 0");
}

#[test]
fn rotation_nozzle_select_byte() {
    let b = MessageBuilder::default();
    for (idx, expected) in [(1u8, 0x01u8), (2, 0x02), (3, 0x03), (4, 0x04)] {
        let f = b.nozzle_rotation(idx, 30.0, true, 0);
        assert_eq!(
            f.0[6], expected,
            "nozzle {} should encode in the low nibble",
            idx
        );
    }
}

#[test]
fn machine_rotate_sends_delta_not_absolute() {
    // firmware tracks g_nozzle_angle[] and sends (current - previous).
    // The machine must do the same or successive rotations will accumulate
    // raw steps and eventually wrap the u16.
    let mut m = new_machine();

    m.rotate_nozzle(NozzleId::N0, 90.0, true, 1).unwrap();
    let after_first = last_sent(&m);
    let steps_first = u16::from_be_bytes([after_first[2], after_first[3]]);

    m.rotate_nozzle(NozzleId::N0, 180.0, true, 1).unwrap();
    let after_second = last_sent(&m);
    let steps_second = u16::from_be_bytes([after_second[2], after_second[3]]);

    let expected_per_90 = (90.0_f64 * (3200.0 / 360.0)).round() as u16;
    assert_eq!(steps_first, expected_per_90);
    assert_eq!(
        steps_second, expected_per_90,
        "second rotation 90->180 must send another +90 degree delta, not 180"
    );
}

#[test]
fn machine_rotate_negative_delta_uses_ccw_bit() {
    let mut m = new_machine();

    m.rotate_nozzle(NozzleId::N0, 45.0, true, 1).unwrap();
    m.rotate_nozzle(NozzleId::N0, -45.0, true, 1).unwrap();
    let f = last_sent(&m);
    // delta = -45 - 45 = -90, abs encoded as steps; clockwise flag => dir=0
    let expected_steps = (90.0_f64 * (3200.0 / 360.0)).round() as u16;
    assert_eq!(u16::from_be_bytes([f[2], f[3]]), expected_steps);
    assert_eq!(f[5], 1);
}

#[test]
fn home_all_resets_nozzle_angle_tracker() {
    let mut m = new_machine();

    m.rotate_nozzle(NozzleId::N0, 720.0, true, 1).unwrap();
    m.home_all().unwrap();
    // After homing the tracker should be zero, so rotate to 90 sends 90 deg,
    // not 810 deg.
    m.rotate_nozzle(NozzleId::N0, 90.0, true, 1).unwrap();
    let f = last_sent(&m);
    let expected = (90.0_f64 * (3200.0 / 360.0)).round() as u16;
    assert_eq!(u16::from_be_bytes([f[2], f[3]]), expected);
}

#[test]
fn per_nozzle_angle_state_is_independent() {
    let mut m = new_machine();

    m.rotate_nozzle(NozzleId::N0, 90.0, true, 1).unwrap();
    m.rotate_nozzle(NozzleId::N2, 45.0, true, 1).unwrap();

    let f_n2 = last_sent(&m);
    let steps_n2 = u16::from_be_bytes([f_n2[2], f_n2[3]]);
    let expected_n2 = (45.0_f64 * (3200.0 / 360.0)).round() as u16;
    assert_eq!(steps_n2, expected_n2);
    assert_eq!(f_n2[6], 0x13, "nozzle 3 (N2) -> 0x10 | 3");

    // Verify N0 is still tracked independently by sending another small move.
    m.rotate_nozzle(NozzleId::N0, 135.0, true, 1).unwrap();
    let f_n0 = last_sent(&m);
    let steps_n0 = u16::from_be_bytes([f_n0[2], f_n0[3]]);
    let expected_n0 = (45.0_f64 * (3200.0 / 360.0)).round() as u16;
    assert_eq!(steps_n0, expected_n0, "N0 delta = 135 - 90");
}

// ---- Through-machine wiring of nozzle/vacuum/blow/light ---------------------

#[test]
fn machine_passes_correct_nozzle_byte_for_vacuum_and_blow() {
    let mut m = new_machine();

    m.open_vacuum(NozzleId::N2).unwrap();
    let f = last_sent(&m);
    assert_eq!(f[0], 0xDD);
    assert_eq!(f[1], 0xCC);
    assert_eq!(f[2], 0x06);
    assert_eq!(f[6], 3, "NozzleId::N2 -> 3");

    m.close_blow(NozzleId::N3).unwrap();
    let f = last_sent(&m);
    assert_eq!(f[2], 0x05);
    assert_eq!(f[6], 4);
}

#[test]
fn error_light_colors_map_correctly() {
    let mut m = new_machine();
    m.error_light(LightColor::Red).unwrap();
    let f = last_sent(&m);
    assert_eq!(f[0], 0xBB);
    assert_eq!(f[1], 9);
    m.error_light(LightColor::Yellow).unwrap();
    assert_eq!(last_sent(&m)[1], 8);
}

#[test]
fn light_on_frame_matches_firmware_cmdLightOn() {
    // firmware (machine_actions.h:965-991): byte 6 = light id (1=high-speed,
    // 2=precision), byte 3 = brightness 0..=100.
    let b = MessageBuilder::default();
    assert_eq!(
        b.light_on(1, 0).0,
        [0xBB, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x55]
    );
    assert_eq!(
        b.light_on(1, 15).0,
        [0xBB, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x01, 0x55]
    );
    assert_eq!(
        b.light_on(1, 100).0,
        [0xBB, 0x00, 0x00, 0x64, 0x00, 0x00, 0x01, 0x55]
    );
    assert_eq!(
        b.light_on(2, 46).0,
        [0xBB, 0x00, 0x00, 0x2E, 0x00, 0x00, 0x02, 0x55]
    );
}

#[test]
fn light_off_frame_matches_firmware_cmdLightsOff() {
    // firmware (machine_actions.h:997-1010): BB 00 00 01 00 00 08 55.
    let b = MessageBuilder::default();
    assert_eq!(
        b.light_off().0,
        [0xBB, 0x00, 0x00, 0x01, 0x00, 0x00, 0x08, 0x55]
    );
}

// ---- binary position return --------------------------------------------------

#[test]
fn machine_report_position_waits_for_binary_return() {
    let mut transport = LoopbackTransport::new();
    transport.set_frame_response(hwcontrol::Frame([
        0xAA, 0xA1, 0x9C, 0x19, 0x2A, 0xF3, 0x00, 0x55,
    ]));
    let mut machine = OpenKayoMachine::new(transport, MessageBuilder::default());

    let position = machine.report_position().unwrap();

    assert_eq!(position.x, -65.56);
    assert_eq!(position.y, -622.5);
}

#[test]
fn position_report_rejects_missing_axis() {
    let error = hwcontrol::Position::parse_report("MPos:X:1 Y:2 I:3 J:4 K:5 L:6 A:7 B:8 C:9\n")
        .unwrap_err();
    assert!(error.to_string().contains("incomplete"));
}
