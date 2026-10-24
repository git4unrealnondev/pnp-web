#!/usr/bin/env python3
"""Generate byte code tables for MakeXYabs, makeXYJogMsg, and z-axis movements."""

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import _globals_stub  # noqa: F401  -- provides g.md and logger
from KayoMsg import CKayoMsg


def fmt(b):
    return " ".join(f"{x:02X}" for x in b)


def main():
    km = CKayoMsg()

    print("=" * 88)
    print("makeXYabsMsg(xy=(X, Y), speed, movetype='NORMAL')")
    print("=" * 88)
    print(f"{'(X, Y)':<14} {'speed':<6} {'type':<8} -> bytes")
    print("-" * 88)
    # NB: machine coords are encoded as (home - input) * KSF in 16-bit unsigned.
    # KSF=100 and unsigned16 max ≈ 65535, so (home - input) must be in [0, 655].
    # Pick a generous home so the diff is non-negative and small.
    big_home = type("X", (), {"X": 600.0, "Y": 600.0})()
    import builtins
    g.md.XYrealhome = big_home

    print(f"{'(0, 0)':<14} {1:<6} {'NORMAL':<8} -> {fmt(km.makeXYabsMsg((0, 0), 1))}")
    print(f"{'(100, 200)':<14} {1:<6} {'NORMAL':<8} -> {fmt(km.makeXYabsMsg((100, 200), 1))}")
    print(f"{'(500, 500)':<14} {1:<6} {'NORMAL':<8} -> {fmt(km.makeXYabsMsg((500, 500), 1))}")
    print(f"{'(100, 300)':<14} {3:<6} {'NORMAL':<8} -> {fmt(km.makeXYabsMsg((100, 300), 3))}")
    print(f"{'(550, 50)':<14} {3:<6} {'NORMAL':<8} -> {fmt(km.makeXYabsMsg((550, 50), 3))}")
    print(f"{'(100, 200)':<14} {1:<6} {'T1':<8} -> {fmt(km.makeXYabsMsg((100, 200), 1, 'T1'))}")
    print(f"{'(100, 200)':<14} {1:<6} {'T2':<8} -> {fmt(km.makeXYabsMsg((100, 200), 1, 'T2'))}")
    print(f"{'(100, 200)':<14} {1:<6} {'T3':<8} -> {fmt(km.makeXYabsMsg((100, 200), 1, 'T3'))}")
    print(f"{'(100, 200)':<14} {1:<6} {'THROW':<8} -> {fmt(km.makeXYabsMsg((100, 200), 1, 'THROW'))}")
    print(f"{'(100, 200)':<14} {5:<6} {'NORMAL':<8} -> {fmt(km.makeXYabsMsg((100, 200), 5))}")

    print()
    print("=" * 88)
    print("makeXYjogMsg(isX, isY, speed)  -- speed=0 means STOP (no motion flag)")
    print("=" * 88)
    print(f"{'isX':<6} {'isY':<6} {'speed':<6} -> bytes")
    print("-" * 88)
    print(f"{0:<6} {0:<6} {0:<6} -> {fmt(km.makeXYjogMsg(0, 0, 0))}    (stop)")
    print(f"{0:<6} {0:<6} {1:<6} -> {fmt(km.makeXYjogMsg(0, 0, 1))}    (start, no dir)")
    print(f"{0:<6} {1:<6} {5:<6} -> {fmt(km.makeXYjogMsg(0, 1, 5))}    (Y+)")
    print(f"{0:<6} {-1:<6} {5:<6} -> {fmt(km.makeXYjogMsg(0, -1, 5))}    (Y-)")
    print(f"{1:<6} {0:<6} {3:<6} -> {fmt(km.makeXYjogMsg(1, 0, 3))}    (X+)")
    print(f"{-1:<6} {0:<6} {3:<6} -> {fmt(km.makeXYjogMsg(-1, 0, 3))}    (X-)")
    print(f"{1:<6} {1:<6} {7:<6} -> {fmt(km.makeXYjogMsg(1, 1, 7))}    (X+,Y+)")
    print(f"{1:<6} {-1:<6} {7:<6} -> {fmt(km.makeXYjogMsg(1, -1, 7))}    (X+,Y-)")
    print(f"{-1:<6} {1:<6} {7:<6} -> {fmt(km.makeXYjogMsg(-1, 1, 7))}    (X-,Y+)")
    print(f"{-1:<6} {-1:<6} {7:<6} -> {fmt(km.makeXYjogMsg(-1, -1, 7))}    (X-,Y-)")

    print()
    print("=" * 88)
    print("makeZabsMsg(toolname, Z, speed)  -- Z is height above deck (mm)")
    print("=" * 88)
    print(f"{'tool':<8} {'Z(mm)':<8} {'speed':<6} -> bytes")
    print("-" * 88)
    print(f"{'Z1':<8} {50.0:<8} {1:<6} -> {fmt(km.makeZabsMsg('Z1', 50.0, 1))}")
    print(f"{'Z1':<8} {40.0:<8} {1:<6} -> {fmt(km.makeZabsMsg('Z1', 40.0, 1))}")
    print(f"{'Z1':<8} {45.0:<8} {2:<6} -> {fmt(km.makeZabsMsg('Z1', 45.0, 2))}")
    print(f"{'Z2':<8} {50.0:<8} {1:<6} -> {fmt(km.makeZabsMsg('Z2', 50.0, 1))}")
    print(f"{'Z3':<8} {50.0:<8} {1:<6} -> {fmt(km.makeZabsMsg('Z3', 50.0, 1))}")
    print(f"{'Z4':<8} {50.0:<8} {1:<6} -> {fmt(km.makeZabsMsg('Z4', 50.0, 1))}")
    print(f"{'Z1':<8} {35.0:<8} {3:<6} -> {fmt(km.makeZabsMsg('Z1', 35.0, 3))}    (plunged)")

    print()
    print("=" * 88)
    print("makeZtestMsg(toolname, val)  -- raw microstep value")
    print("=" * 88)
    print(f"{'tool':<8} {'val':<8} -> bytes")
    print("-" * 88)
    print(f"{'Z1':<8} {1600:<8} -> {fmt(km.makeZtestMsg('Z1', 1600))}    (90 deg)")
    print(f"{'Z1':<8} {2490:<8} -> {fmt(km.makeZtestMsg('Z1', 2490))}    (max)")
    print(f"{'Z1':<8} {0:<8} -> {fmt(km.makeZtestMsg('Z1', 0))}    (top)")
    print(f"{'Z2':<8} {1600:<8} -> {fmt(km.makeZtestMsg('Z2', 1600))}")

    print()
    print("=" * 88)
    print("makedCannedPnP(depth, feeder, speed, delay, nozzle, typecanned)")
    print("=" * 88)
    print(f"{'depth':<8} {'feeder':<8} {'speed':<6} {'delay':<6} {'nozzle':<8} {'type':<6} -> bytes")
    print("-" * 88)
    print(f"{35.0:<8} {1:<8} {1:<6} {100:<6} {'Z1':<8} {1:<6} -> {fmt(km.makedCannedPnP(35.0, 1, 1, 100, 'Z1', 1))}")
    print(f"{40.0:<8} {2:<8} {2:<6} {200:<6} {'Z2':<8} {2:<6} -> {fmt(km.makedCannedPnP(40.0, 2, 2, 200, 'Z2', 2))}")


if __name__ == "__main__":
    main()
