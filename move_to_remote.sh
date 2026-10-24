#!/usr/bin/env bash
set -Eeuo pipefail

SITE="target/site"
EXE="target/release/leptos-tailwind"
CALIBRATION="camera-calibration.json"

cargo leptos build --release

REMOTE="${PNP_SSH_TARGET:-pnp@192.168.1.68}"
REMOTE_HOME="$(ssh "$REMOTE" 'printf %s "$HOME"')"
REMOTE_LOCATION="${PNP_REMOTE_LOCATION:-$REMOTE_HOME/Desktop/packed}"
REMOTE_FRAME_DUMP_DIR="${PNP_FRAME_DUMP_DIR:-$REMOTE_LOCATION/frame-dumps}"
REMOTE_CAMERA_FRAME_DIR="${PNP_CAMERA_FRAME_DIR:-$REMOTE_LOCATION/camera-frame-dumps}"

[[ -d "$SITE" ]] || { printf 'error: missing %s\n' "$SITE" >&2; exit 1; }
[[ -x "$EXE" ]] || { printf 'error: missing executable %s\n' "$EXE" >&2; exit 1; }

printf 'preparing %s:%s\n' "$REMOTE" "$REMOTE_LOCATION"
ssh "$REMOTE" "rm -rf -- '$REMOTE_LOCATION/target/site' '$REMOTE_LOCATION/leptos-tailwind' && mkdir -p -- '$REMOTE_LOCATION/target' '$REMOTE_FRAME_DUMP_DIR' '$REMOTE_CAMERA_FRAME_DIR'"

printf 'copying site files\n'
scp -r "$SITE" "$REMOTE:$REMOTE_LOCATION/target/"
printf 'copying executable\n'
scp "$EXE" "$REMOTE:$REMOTE_LOCATION/"
scp "startup.sh" "$REMOTE:$REMOTE_LOCATION/"
if [[ -f "$CALIBRATION" ]]; then
    printf 'copying camera calibration\n'
    scp "$CALIBRATION" "$REMOTE:$REMOTE_LOCATION/"
else
    printf 'warning: %s not found; feeder-pocket centering will be unavailable\n' "$CALIBRATION" >&2
fi

printf 'starting remote application\n'
ssh -t "$REMOTE" "cd '$REMOTE_LOCATION' && exec ./startup.sh"
