# Remote Deploy

`move_to_remote.sh` is the remote equivalent of `move_to_usb.sh`. It copies:

- `target/site` to `~/Desktop/packed/target/site`
- `target/release/leptos-tailwind` to `~/Desktop/packed/leptos-tailwind`
- `camera-calibration.json`, when present, to `~/Desktop/packed/camera-calibration.json`

It then starts the copied executable from that directory over SSH.

```sh
./move_to_remote.sh
```

Override the SSH destination or remote folder when needed:

```sh
PNP_SSH_TARGET=pnp@192.168.1.68 \
PNP_REMOTE_LOCATION=/home/pnp/Desktop/packed \
./move_to_remote.sh
```

Stop the remote application with `Ctrl-C`.

Every binary 8-byte frame received from the machine is written to
`~/Desktop/packed/frame-dumps/frame-NNNNNN.bin`. Override the directory with
`PNP_FRAME_DUMP_DIR` when deploying.

Camera images used for fiducial, feeder-pocket, or calibration inspection are
written as JPEGs to
`~/Desktop/packed/camera-frame-dumps/frame-NNNNNN-video0.jpg`. Override that
directory with `PNP_CAMERA_FRAME_DIR` when deploying.

Feeder-pocket centering requires a camera calibration. Run the camera
calibration flow first; the deployment script transfers the resulting
`camera-calibration.json` automatically.
