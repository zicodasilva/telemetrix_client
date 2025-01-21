#!/bin/bash

TARGET="aarch64-unknown-linux-gnu"
RELEASE_PATH="target/$TARGET/release"
REMOTE_USER="mirte"
REMOTE_IP="mirte-21.home"
REMOTE_PATH="/home/mirte/zico/telemetrix_client"
EXECUTABLE="telemetrix_client"

echo "Building the project for $TARGET..."
cargo build --target $TARGET --release

echo "Transferring the binary to the Orange Pi..."
scp "$RELEASE_PATH/$EXECUTABLE" "$REMOTE_USER@$REMOTE_IP:$REMOTE_PATH"

echo "Running the binary on the Orange Pi..."
ssh "$REMOTE_USER@$REMOTE_IP" "$REMOTE_PATH/$EXECUTABLE"
