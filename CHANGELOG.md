# Changelog

Historical motion results remain in the [original tuning record](docs/tuning-history.zh-CN.md).

## 0.1.0 — 2026-11-19

### Added

- English/Chinese README pages, setup and ROS topic-contract documentation.
- A four-panel preview edited from the existing teleoperation recording, with source timings and playback speeds documented.
- MIT terms for original source/documentation, explicit third-party scene exclusions, and software citation metadata.
- Contribution guidance, issue/PR templates, offline Python checks and a ROS 2 Humble package-build/import workflow.

### Fixed

- Public `walk` and `teleop_gui` imports no longer require the unpublished `autotune.py` extension. Explicit `auto_tune` use reports the missing extension.
- ROS metadata now declares Python runtime dependencies and a maintainer contact already present in public Git history.
- Removed the redundant install-time `sed` rewrite; the source already has the ROS system-Python shebang.

### Documentation and repository maintenance

- Preserved existing video links, profile parameters and May 2026 development history.
- Distinguished the bounded CoM tracker from classical optimal preview control; documented unused CoP inputs and contact fallback.
- Removed IDE metadata from version control while retaining local copies.

## Project history — May 2026

- 17–18 May: initial ROS 2 control stack; original notes record six-step walking and standing recovery.
- 19–25 May: keyboard profiles, backward walking, turning, standing yaw and faster-forward tuning.
- 27 May: modular code-reading tutorials added.
- 31 May: `MAN.V1.2.ttt` scene added.

These are development dates from existing records and Git history, not numbered releases.
