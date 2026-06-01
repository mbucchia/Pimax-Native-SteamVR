# Pimax Native SteamVR driver

This project is a proof-of-concept of a SteamVR "native" output driver for Pimax headsets.

DISCLAIMER: This software is distributed as-is, without any warranties or conditions of any kind. Use at your own risks.

# THIS IS A WORK IN PROGRESS

This driver is a work-in-progress and is currently not usable by end users.

# Supported features

- Headset tracking
  - SLAM headsets only: Pimax Crystal, Pimax Crystal Light, Pimax Crystal Super, and likely Pimax Dream Air. At this time, only Pimax Crystal Super has been tested.
  - Lighthouse headsets: see [sboys3's CustomHeadsetOpenVR](https://github.com/sboys3/CustomHeadsetOpenVR))
- Motion Controllers tracking (Pimax Crystal controllers only)
- Motion Controllers inputs (eg: triggers, buttons...)
- Motion Controllers haptics
- Eye tracking
- Video See Through (VST)
- Visibility mask

# Current blockers

1. Pimax Play shall relinquish Direct Mode exclusive access to the driver. Today, this driver implements a hack to take over Pimax Play's exclusive lock, however this requires a) a patched `pi_server.exe` and b) proper timing (aka "luck") at startup.

1. Pimax Play shall return the correct distortion profile through the `pvr_getHmdDistortedUV()` API. Today, this driver attempts to extract this information from `pi_server.exe`, but does not do a good job at it yet.

These requests have been sent to the Pimax team.
