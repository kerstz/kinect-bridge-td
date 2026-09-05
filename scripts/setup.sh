#!/usr/bin/env bash
# One-shot installer: deps, MSVC headers (xwin), TD SDK headers, build, deploy, systemd.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
TD_PREFIX="${TD_PREFIX:-$HOME/.local/share/touchdesigner-linux/prefix}"

echo "== [1/6] System packages =="
sudo apt update
sudo apt install -y build-essential pkg-config git curl clang-19 lld-19 libfreenect-dev

echo "== [2/6] Rust + xwin (MSVC headers/libs on Linux) =="
command -v cargo >/dev/null 2>&1 || { curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y; }
. "$HOME/.cargo/env"
command -v xwin  >/dev/null 2>&1 || cargo install xwin --locked
[ -d "$HOME/.xwin/crt" ] || "$HOME/.cargo/bin/xwin" --accept-license --arch x86_64 splat --output "$HOME/.xwin"

echo "== [3/6] TouchDesigner C++ SDK headers (from your local TD install) =="
mkdir -p "$ROOT/sdk_reference"
TD_DIR=$(ls -d "$TD_PREFIX/drive_c/Program Files"/TouchDesigner* 2>/dev/null | head -n1)
SRC="$TD_DIR/Samples/CPlusPlus/CPUMemoryTOP"
cp -f "$SRC/TOP_CPlusPlusBase.h" "$SRC/CPlusPlus_Common.h" "$ROOT/sdk_reference/"

echo "== [4/6] Build server + plugin =="
"$ROOT/scripts/build.sh"

echo "== [5/6] systemd user service =="
mkdir -p "$HOME/.config/systemd/user"
cat > "$HOME/.config/systemd/user/kinect-bridge.service" <<UNIT
[Unit]
Description=Kinect Bridge Server (libfreenect -> TCP 9999/9998)
After=network.target
[Service]
ExecStart=$ROOT/kinect_bridge_server
Restart=always
RestartSec=2
[Install]
WantedBy=default.target
UNIT
systemctl --user daemon-reload
systemctl --user enable --now kinect-bridge.service || true

echo "== [6/6] Finished =="
echo "Optional: sudo loginctl enable-linger $USER   (start service at boot)"
echo "In TouchDesigner: CPlusPlus TOP -> C:\\TDPlugins\\KinectBridgeTOP.dll"
