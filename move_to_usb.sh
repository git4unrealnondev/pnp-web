#!/usr/bin/env bash
set -e

SITE="target/site"
EXE="target/release/leptos-tailwind"

PROD_LOCATION="/run/media/linux/USB/packed"
rm -rf "$PROD_LOCATION/target/site"
rm "$PROD_LOCATION/leptos-tailwind"
cp -r "$SITE" "$PROD_LOCATION/target/site"
cp "$EXE" "$PROD_LOCATION"
