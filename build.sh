#!/bin/bash
set -e
CONFIG=debug
ARCH=armv7-unknown-linux-gnueabihf
cargo build --target=$ARCH
scp target/$ARCH/$CONFIG/rabid-control root@fpv-laptimer.local:/tmp

