#!/usr/bin/env bash
# Rebuild the Linux server and the Windows-ABI TD plugin, then deploy the DLL.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
XWIN="${XWIN:-$HOME/.xwin}"
TD_PREFIX="${TD_PREFIX:-$HOME/.local/share/touchdesigner-linux/prefix}"
find_lib(){ find "$XWIN" -type f -iname "$1" 2>/dev/null | head -n1; }

echo "[1/3] Building Linux server..."
gcc "$ROOT/server/kinect_bridge_server.c" -o "$ROOT/kinect_bridge_server" \
    $(pkg-config --cflags --libs libfreenect) -lpthread -O2

echo "[2/3] Building Windows-ABI plugin..."
clang++-19 \
  --target=x86_64-pc-windows-msvc \
  -fms-compatibility -fms-extensions -fdelayed-template-parsing \
  -fms-compatibility-version=19.33 -std=c++17 -fexceptions -frtti \
  -D_DLL -D_CRT_USE_BUILTIN_OFFSETOF=1 -DWIN32_LEAN_AND_MEAN -D_CRT_SECURE_NO_WARNINGS \
  -Wno-invalid-offsetof -Wno-deprecated-declarations \
  -I"$ROOT/plugin" -I"$ROOT/sdk_reference" \
  -isystem "$XWIN/crt/include" \
  -isystem "$XWIN/sdk/include/ucrt" \
  -isystem "$XWIN/sdk/include/shared" \
  -isystem "$XWIN/sdk/include/um" \
  -c "$ROOT/plugin/KinectBridgeTOP.cpp" -o "$ROOT/KinectBridgeTOP.obj"

lld-link-19 /DLL /MACHINE:X64 /OUT:"$ROOT/KinectBridgeTOP.dll" "$ROOT/KinectBridgeTOP.obj" \
  /NODEFAULTLIB /NOENTRY \
  "$(find_lib msvcrt.lib)" "$(find_lib vcruntime.lib)" "$(find_lib ucrt.lib)" \
  "$(find_lib kernel32.lib)" "$(find_lib ws2_32.lib)" "$(find_lib msvcprt.lib)" \
  /SUBSYSTEM:WINDOWS

echo "[3/3] Deploying into the Wine prefix..."
mkdir -p "$TD_PREFIX/drive_c/TDPlugins"
cp -f "$ROOT/KinectBridgeTOP.dll" "$TD_PREFIX/drive_c/TDPlugins/"
echo "Done. In TouchDesigner: CPlusPlus TOP -> C:\\TDPlugins\\KinectBridgeTOP.dll"
