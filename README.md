English | [繁體中文](README_TCH.md) | [簡體中文](README_SCH.md)

# Weshare RP2040 lcd Fluidsim 

 Based on waveshare RP2040-1.28-LCD

# Video
[![IMAGE ALT TEXT](http://img.youtube.com/vi/fz67tiyyCQg/0.jpg)](https://www.youtube.com/watch?v=fz67tiyyCQg "Shake change color")

# Features

- Simulated fluid
- Grid display
- Four-direction shake to change colors
- Sleeps after being idle for a while
- Wake on motion

### Changes from the original

```diff
+ Wake on motion
- The original woke via a pin
+ Color change on shake
- The original was single-color
+ Added documentation and setup steps
```

## How to Use

### Docs & environment setup

- [Waveshare official wiki: RP2040-LCD-1.28](https://www.waveshare.net/wiki/RP2040-LCD-1.28)

### References

- https://mitxela.com/shop/fluid-pendant
- https://mitxela.com/projects/fluid-pendant
- https://www.youtube.com/watch?v=XmzBREkK8kY
- [Fluid Pendant remake (Chinese)](https://www.bilibili.com/video/BV1PA4ZzDEAu)

### Steps

You can either build it yourself or flash a prebuilt image. (For security, you might prefer building it yourself; the release should include matching builds.)

#### Flashing a prebuilt image

- Download the corresponding prebuilt `.uf2` file from the GitHub release.
- Hold the **BOOT** button on the board while connecting it to your computer. A mass-storage drive should appear; copy the `.uf2` file onto it. (If it doesn’t appear, try again; you may also need a different cable or USB port.)
- After copying the `.uf2`, the board will reboot automatically and the app should start.

#### Building it yourself

- Get a **Waveshare RP2040-LCD-1.28** board **without touch** (the touch version uses different pins and code; you’d need to modify it yourself).
- Use a reliable data-capable USB cable and a stable USB 3.2 (or equivalent) port on your computer.
- Follow the Waveshare wiki to install the **Arduino IDE** (with the Raspberry Pi Pico support), and also install **VS Code** with the **PlatformIO** extension.
- Clone this repository from GitHub.
- After setup, click the checkmark (PlatformIO **Build**) in the top-right of VS Code to build.
- When the build finishes, a release/artifact folder will contain the `.uf2` firmware file.
- Hold the BOOT button while connecting the board; when the drive appears, copy the `.uf2` file onto it.
- The board will reboot automatically and start the app.

### What you can tweak in the code

- **Fluid behavior**
  - In `lib/FluidRenderer`. You can adjust fluid particle constants in `FluidRenderer.hpp`.
- **Sleep modes**
  - In `lib/LowPower`, which uses `LowPowerRP2040.h`. If you want to switch between wake-on-shake and pin-based wake, change it here.
- **Shake-to-color**
  - Implemented in the fluid code and partially in `main.cpp`.

### Optional enhancements

- **Enclosure**
  - Download the `shell` files in this repo and 3D-print a case.
- **Battery**
  - You can add a battery with a 1.25 mm connector (often called JST/MX 1.25) or solder your own leads.
- **Power switch**
  - Add a physical switch and set its pin in the code to sleep/wake for better power savings.
