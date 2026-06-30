# Pimax Native SteamVR driver

This project is a proof-of-concept of a SteamVR "native" output driver for Pimax headsets.

DISCLAIMER: This software is distributed as-is, without any warranties or conditions of any kind. Use at your own risks.

# THIS IS A WORK IN PROGRESS

This driver is a work-in-progress and is currently not usable by end users.

# Supported features

- Headset tracking
  - SLAM headsets only: Pimax Crystal, Pimax Crystal Light, Pimax Crystal Super, Pimax Dream Air, Pimax Dream Air SE. At this time, only Pimax Crystal Super has been tested.
  - Lighthouse headsets: see [sboys3's CustomHeadsetOpenVR](https://github.com/sboys3/CustomHeadsetOpenVR)
- Motion Controllers tracking (Pimax Crystal controllers only)
- Motion Controllers inputs (eg: triggers, buttons...)
- Motion Controllers haptics
- Optical hand tracking
- Eye tracking
- Video See Through (VST)
- Visibility mask
- Pimax Foveated Rendering ("LibMagic")

# Building from source

1. Use Visual Studio 2022-2026 (not Visual Studio Code).

1. Remember to fetch the Git submodules! `git submodule update --init` will do the trick!

1. After you build the Visual Studio solution, the driver is located under `bin\distribution`.

1. Register the driver with SteamVR by invoking (as regular user) the script `bin\distribution\Register-Driver.bat`.

# Extracting distortion

1. Build the Visual Studio solution.

1. Set environment variable `DISTORTION_EXTRACTOR_DUMP_FOR_DRIVER=1`.

1. Run `pi_server` with the `distortion_extractor.dll` injected. Use the `withdll.exe` tool in `utils`:

   ```
   utils\withdll.exe /d:bin\x64\Debug\distortion_extractor.dll  path\to\pi_server.exe
   ```

1. Wait until the process completes. The distortion file is stored in `%ProgramData%\Pimax-Native-SteamVR`, with the filename being the serial number of the headset.
