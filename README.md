# KinectBridgeTD

A native-feeling "Kinect TOP" for TouchDesigner on Linux (Wine), using an Xbox 360 Kinect v1.

    Kinect v1 -> libfreenect (native Linux) -> TCP loopback -> C++ TOP plugin (MSVC ABI) -> TouchDesigner (Wine)

Color, registered depth (mm), infrared, silhouette mask, XYZ point cloud, colormaps,
LED and motor control from TouchDesigner, record/playback without the device,
auto-restarting systemd service. 30 fps.

## What is this?

TouchDesigner has no official Linux build; the community wrapper TouchDesigner-Linux
runs the Windows version under Wine. The stock Kinect TOP cannot work there because it
depends on the Microsoft Kinect SDK and Windows kernel drivers, which Wine cannot
provide.

This project rebuilds the missing operator in two halves:

1. kinect_bridge_server - a small native Linux daemon that owns the Kinect through
   libfreenect (registered depth in millimetres, RGB, IR) and serves frames over
   127.0.0.1:9999. A control port (9998) exposes LED / tilt / accelerometer commands,
   and sessions can be recorded or replayed.
2. KinectBridgeTOP.dll - a custom TouchDesigner C++ TOP, cross-compiled from Linux
   with clang --target=x86_64-pc-windows-msvc against real MSVC headers/libs (fetched
   by xwin) and linked with lld-link. Inside TouchDesigner (Wine) it pulls frames over
   the loopback TCP socket and republishes them with the official
   createOutputBuffer() / uploadBuffer() API.

Why not MinGW? TouchDesigner is an MSVC binary; C++ plugins must match the MSVC ABI
(vtable layout, name mangling). MinGW-built DLLs crash with access violations. The
clang/xwin/lld toolchain produces a binary-compatible DLL entirely on Linux.

## Architecture

        LINUX (native)                          WINE (TouchDesigner)
 +---------------------------+        +----------------------------------+
 | kinect_bridge_server      |        |  KinectBridgeTOP.dll             |
 |  libfreenect (v1)         | :9999  |   CPlusPlus TOP                  |
 |  RGB / Depth-mm / IR      |<-------|   Image: Color/Depth/Raw/Mask/   |
 |  record / playback        |  TCP   |          PointCloud/Infrared     |
 |  control LED/tilt/accel   | :9998  |   Device: LED, Tilt              |
 |  live RGB<->IR switch     |<-------|   Network: Host/Port/Reconnect   |
 +---------------------------+        +----------------------------------+

## Features

- Streams: Color (RGB 640x480), Registered depth (16-bit, mm), Raw depth,
  Infrared (live RGB/IR switch), Mask (background subtraction),
  Point Cloud XYZ (metres, float texture)
- Depth tools: Too Close / Too Far (mm), Auto Range, Denoise (3x3 median +
  hole filling), Invert, Unknown-pixel value
- Color tools: Gamma, Swap Red/Blue, Mirror, Vertical flip
- Colormaps: Grey / Jet / Viridis / Inferno
- Hardware: LED menu and motor Tilt (+-30 deg) driven from TouchDesigner
- Workflow: --record / --play session files (develop without the Kinect)
- System: systemd user service, auto-start and auto-restart

Hardware note: the Kinect v1 color camera does RGB OR IR, never both at once.
Selecting Infrared in the plugin switches the server live; switching back
restores RGB.

## Requirements

- Xbox 360 Kinect v1 on a USB 2.0 port
- Linux (tested on Ubuntu 24.04 / Linux Mint 22)
- TouchDesigner 2025.x installed via the TouchDesigner-Linux Wine wrapper
  (launched once, so the Wine prefix exists)
- Internet access during setup (xwin downloads MSVC headers/libs)

## Quick start

### 1. One-time host preparation

    echo "blacklist gspca_kinect" | sudo tee /etc/modprobe.d/blacklist-kinect-gspca.conf
    sudo modprobe -r gspca_kinect 2>/dev/null || true

### 2. Install everything

    git clone https://github.com/YOUR-USERNAME/kinect-bridge-td.git
    cd kinect-bridge-td
    ./scripts/setup.sh

setup.sh will:
1. install system packages (clang-19, lld-19, libfreenect-dev, ...),
2. install Rust + xwin and splat the MSVC CRT/SDK into ~/.xwin,
3. copy the two TouchDesigner C++ SDK headers from your local TD install
   into sdk_reference/ (never redistributed),
4. build the server and the DLL, deploy the DLL to C:\TDPlugins\,
5. install and enable the kinect-bridge systemd user service.

Optional, to start the service at boot without a login session:

    sudo loginctl enable-linger $USER

### 3. Use it in TouchDesigner

1. Check the bridge: systemctl --user status kinect-bridge (must be active).
2. In TouchDesigner: OP Create Dialog -> CPlusPlus TOP.
3. Set Plugin Path to C:\TDPlugins\KinectBridgeTOP.dll.
4. Pick a mode in the Kinect page: the image appears.

After rebuilding the DLL, fully restart TouchDesigner (Wine keeps the file
locked) and recreate the CPlusPlus TOP.

## Parameter reference

| Page    | Parameter      | Purpose                                        |
|---------|----------------|------------------------------------------------|
| Kinect  | Image          | Color / Depth / Raw Depth / Mask / Point Cloud XYZ / Infrared |
| Kinect  | Colormap       | Grey / Jet / Viridis / Inferno (depth views)   |
| Kinect  | Mirror         | horizontal mirror                              |
| Kinect  | Flipvertical   | vertical flip                                  |
| Kinect  | Autorange      | auto-fits the depth range (2-98 percentiles)   |
| Kinect  | Denoise        | 3x3 median + hole filling on depth             |
| Kinect  | Swapredblue    | R/B swap                                       |
| Kinect  | Invertdepth    | invert depth gradient                          |
| Kinect  | Unknownvalue   | value used for invalid depth pixels            |
| Kinect  | Gamma          | color gamma                                    |
| Kinect  | Close / Far    | depth window in millimetres                    |
| Kinect  | Capturebg      | pulse: capture background for Mask mode        |
| Device  | Led            | Kinect LED (off/green/red/yellow/blink)        |
| Device  | Tilt           | motor angle, -30 to +30 degrees                |
| Network | Host / Port    | bridge endpoint                                |
| Network | Reconnect      | pulse: force reconnect                         |

### Point cloud recipe

Set Image = Point Cloud XYZ. The TOP outputs a 640x480 float texture where
R,G,B = X,Y,Z in metres (A = 0 for invalid pixels). Feed it to a Lookup TOP +
SOP, or sample it in a GLSL TOP, to obtain a live point cloud.

## Server CLI

    ./kinect_bridge_server                  # live RGB + registered depth
    ./kinect_bridge_server --ir             # live IR + depth
    ./kinect_bridge_server --record s.kin   # live + record
    ./kinect_bridge_server --play s.kin     # replay a session, no Kinect needed

## Wire protocol (for developers)

- Client -> server, on connect: one selector byte (0=RGB, 1=Depth, 2=IR).
- Server -> client, per frame:
  u32be 'KIN1' | u32be width | u32be height | u32be format + payload
  (0=RGB24, 1=DEPTH16 mm, 2=IR8).
- Port 9998: one text command per connection: LED n, TILT x, ACCEL.

## Troubleshooting

| Symptom                          | Fix |
|----------------------------------|-----|
| LIBUSB_ERROR_BUSY                | another process owns the Kinect: pkill -9 -f kinect_bridge, then systemctl --user restart kinect-bridge |
| Black TOP in TD                  | restart TouchDesigner after every DLL rebuild; recreate the CPlusPlus TOP |
| Red/blue swapped                 | toggle Swapredblue |
| ~1 frame/minute                  | stale server build without the libfreenect event pump: restart the service |
| TD rejects a parameter name      | TD rule: one uppercase letter, then lowercase/digits only |
| STL1000 at compile time          | use clang-19 or newer |
| offsetof static_assert errors    | compile with -D_CRT_USE_BUILTIN_OFFSETOF=1 |

## Manual build (developers)

See scripts/build.sh. In short: clang++-19 --target=x86_64-pc-windows-msvc with
the xwin include dirs, then lld-link-19 /DLL /NODEFAULTLIB /NOENTRY with
msvcrt vcruntime ucrt kernel32 ws2_32 msvcprt.

## Repository layout

    server/kinect_bridge_server.c   native Linux bridge (libfreenect)
    plugin/KinectBridgeTOP.h        TouchDesigner C++ TOP (MSVC ABI)
    plugin/KinectBridgeTOP.cpp
    scripts/setup.sh                one-shot installer
    scripts/build.sh                rebuild + deploy

## Credits

Derivative (TouchDesigner C++ SDK), OpenKinect/libfreenect,
TouchDesigner-Linux wrapper, xwin, LLVM clang/lld.

## License

MIT
