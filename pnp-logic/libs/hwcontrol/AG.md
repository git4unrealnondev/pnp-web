# hwcontrol

Rust TUI + library for controlling a Kayo pick-and-place machine through an
[OpenKayo](../OpenKayo) protocol-converter board over serial.

## What it does

- Sends OpenKayo serial frames (8 bytes each, framed `0x?? ... 0x55`) to drive
  the Kayo's gantry, Z axes, nozzle rotation, vacuum/blow valves, feeders,
  conveyor, track width, and tower lights.
- Exposes a `Machine` trait (`src/machine/mod.rs`) implemented by
  `OpenKayoMachine<T: Transport>` (`src/machine/openkayo.rs`).
- Provides a `MessageBuilder` (`src/protocol/messages.rs`) that constructs each
  command frame, and `Transport` trait (`src/transport/mod.rs`) with
  `SerialTransport` and `LoopbackTransport` implementations.
- Runs a ratatui-based terminal UI (`src/main.rs`) for interactive control:
  home, reset, move XY/Z, jog, open/close vacuum+blow, rotate nozzles,
  feeder control, conveyor commands, track width, and tower lights.

## Layout

```
src/
  main.rs              TUI entry point and key bindings
  lib.rs               re-exports the public API
  machine/
    mod.rs             Machine trait + types (NozzleId, LightColor, JogRequest, Error)
    openkayo.rs        OpenKayoMachine impl, per-nozzle angle tracking, transport state
  protocol/
    mod.rs             Frame, ProtocolError, re-exports
    messages.rs        MessageBuilder (all frame constructors) + MoveType/JogDir/ErrorLight
  transport/
    mod.rs             Transport trait + LoggingTransport wrapper + TransportError
    serial.rs          SerialTransport (serialport-backed) and LoopbackTransport
tests/
  kayo.rs              21 tests verifying frame byte layouts match the OpenKayo firmware
```

## Public API surface

```rust
pub use machine::{Error, JogRequest, LightColor, Machine, NozzleId, OpenKayoMachine};
pub use protocol::{ErrorLight, Frame, JogDir, MessageBuilder, MoveType, ProtocolError};
pub use transport::{LoopbackTransport, LoggingTransport, SerialTransport, Transport, TransportError};
```

## Frame protocol summary

All frames are 8 bytes. The OpenKayo firmware source
(`../OpenKayo/Firmware/OpenKayo/machine_actions.h`, `config.h`) is the
authoritative reference for byte layouts.

| Command                | Byte 0 | Notes                                           |
|------------------------|--------|-------------------------------------------------|
| Reset + home           | 0xEE   | sub-cmd 0xBB 0x05                                |
| Reset                  | 0xEE   | sub-cmd 0xBB 0x04                                |
| Home Z                 | 0xEE   | sub-cmd 0xBB 0x02                                |
| Track reset            | 0xEE   | sub-cmd 0xBB 0x06                                |
| Move XY                | 0xAA   | x,y big-endian i16 at bytes 1..5; byte 6 = (move_type<<4) \| speed |
| Move Z                 | 0xCC   | microsteps BE u16 at bytes 1..2; byte 6 = (nozzle<<4) \| speed; nozzle is 1..=4 |
| Nozzle rotation        | 0xDD   | sub-cmd 0xBB; bytes 2..3 = abs(delta)*scale BE u16; byte 5: 0=CW, 1=CCW; byte 6 = 0x10\|nozzle |
| Vacuum / blow / valves | 0xDD   | sub-cmd 0xCC                                     |
| Jog XY                 | 0xEE   | sub-cmd 0xEE; byte 3 = x dir, byte 4 = y dir    |
| Feeder                 | 0xBB   | byte 2 = feeder id; byte 1 = 1 to open          |
| Conveyor               | 0xEE   | sub-cmd 0xCC 0x0x                                |
| Track width            | 0xEE   | sub-cmd 0xBB; BE u16 at bytes 2..3              |
| Lights                 | 0xEE   | sub-cmd 0xCC                                     |

## Conventions

- `XY_SCALE = 100.0` (KAYO_SCALE_FACTOR). Movement inputs are mm.
- `rotation_scale = 3200.0 / 360.0` (KAYO_ROTATION_SCALE). Rotation input is
  cumulative **absolute** degrees; `OpenKayoMachine` tracks per-nozzle angle
  state and sends the delta to the firmware (matches `g_nozzle_angle[]`).
- `OpenKayoMachine::home_all` zeroes the angle tracker.
- `MessageBuilder` exposes `xy_home`, `z_min`, `z_max`, `z_microsteps`,
  `max_z_microsteps`, and `rotation_scale` as public fields.

## Build / run / test

```sh
cargo build
cargo test              # 21 tests in tests/kayo.rs
cargo run               # launches the TUI
```

The TUI uses `[p]` to open a serial port, `[l]` for loopback mode, and the
single-letter keys shown in the in-app Keys panel.

## Related repos

- `../OpenKayo/` — the OpenKayo firmware (Arduino `.ino`) and board design
  that this software drives. Frame layouts in `messages.rs` mirror the
  firmware's command encoders in `Firmware/OpenKayo/machine_actions.h`.