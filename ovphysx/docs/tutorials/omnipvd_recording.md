# OmniPVD Recording: Capture Physics Internals to .ovd

This tutorial shows how to record OmniPVD data from ovphysx to `.ovd` files for offline inspection in a Kit application. OmniPVD captures the full internal state of the PhysX simulation each frame — shapes, contacts, solver data — so you can debug and visualize physics behavior after the fact.

## Prerequisites

- Install ovphysx and confirm native libraries load.
- Prepare a USD file with physics objects.
- For offline inspection: a Kit application with the OmniPVD extension.

## Required Config

OmniPVD recording is controlled by two typed config fields:

| Config field (Python) | C builder | Description |
|---|---|---|
| `omnipvd_ovd_recording_directory` | `ovphysx_config_entry_omnipvd_ovd_recording_directory()` | Writable directory where `.ovd` files are saved |
| `omnipvd_output_enabled` | `ovphysx_config_entry_omnipvd_output_enabled()` | Enables OmniPVD data capture |

Both must be configured **before** the PhysX instance is created, because the recording pipeline is initialized during physics engine startup. Pass them through `PhysXConfig` (Python) or `config_entries` in `ovphysx_create_args` (C/C++).

The runtime auto-creates the recording directory if it does not exist.

## Code

### Python

```{literalinclude} ../../tests/python_samples/omnipvd_recording.py
:language: python
:start-after: [tutorial-start]
:end-before: [tutorial-end]
```

### C++

**CMakeLists.txt**

```{literalinclude} ../../tests/c_samples/omnipvd_recording_cpp/CMakeLists.txt
:language: cmake
```

**Source**

```{literalinclude} ../../tests/c_samples/omnipvd_recording_cpp/main.cpp
:language: cpp
:start-after: [tutorial-start]
:end-before: [tutorial-end]
```

## What Happens at Runtime

1. When `PhysX()` (or `ovphysx_create_instance`) is called with both config fields set, the runtime creates an OmniPVD writer backed by a file stream.
2. A temporary file `tmp.ovd` is created in the recording directory.
3. Each simulation step records a frame of physics state into `tmp.ovd`.
4. When the instance is destroyed (`release()` / `ovphysx_destroy_instance`), the runtime renames `tmp.ovd` to a timestamped file: `YYYY_MM_DD_HH_MM_SS_CC_rec.ovd` (where `CC` is a disambiguation counter).

If the recording directory is unset (empty string) or `omnipvd_output_enabled` is `false`, no recording takes place and no files are written.

## Inspecting .ovd Files in Kit

1. Open any Kit-based application (for example, USD Composer, Isaac Sim).
2. Enable the **OmniPVD** extension from **Window > Extensions**.
3. Use **File > Open** or the OmniPVD panel to load the `.ovd` file.
4. Use the timeline scrubber to step through recorded frames and inspect shapes, contacts, and solver state.

For more details on the Kit-side OmniPVD workflow, refer to the PhysX Visual Debugger documentation included with the Kit application you use for inspection.

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| No `.ovd` file after simulation | Config not set before instance creation | Pass both fields in `PhysXConfig` / `config_entries` at init |
| Empty recording directory | `omnipvd_output_enabled` is `false` | Set to `true` |
| `tmp.ovd` exists but no `*_rec.ovd` | Instance not properly destroyed | Ensure `release()` / `ovphysx_destroy_instance` is called |
| Runtime error about directory | Directory path is invalid or not writable | Use an absolute path to a writable location |

## Result

After this tutorial you can capture `.ovd` recordings from any ovphysx simulation and inspect them offline in Kit.
