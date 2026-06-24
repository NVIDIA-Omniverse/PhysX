# ovphysx

[![PyPI](https://img.shields.io/pypi/v/ovphysx)](https://pypi.org/project/ovphysx/)
[![Python 3.10+](https://img.shields.io/badge/python-3.10%2B-blue)](https://pypi.org/project/ovphysx/)
[![Linux | Windows](https://img.shields.io/badge/platform-linux%20%7C%20windows-lightgrey)](https://pypi.org/project/ovphysx/)

## 1. What is ovphysx?

**ovphysx** is a self-contained library for **USD-based physics simulation**, exposing a **C API with Python bindings**. It wraps NVIDIA PhysX SDK and loads USD scenes directly, runs the simulation, and exchanges state with your code via **DLPack tensors** — all with **no Omniverse Kit installation required**. It packages its own OV-namespaced OpenUSD runtime, so you get USD-native physics as a standalone dependency you can drop into any Python or C/C++ application.

**NVIDIA PhysX SDK** is the popular and stable **real-time physics simulation engine** underneath — the same GPU-accelerated rigid-body, articulation, and contact solver used across robotics, simulation, and interactive 3D for over a decade. PhysX is open source under BSD-3-Clause and ships in the same repository alongside the **Omniverse PhysX extensions** (for Kit-based apps). ovphysx is the path that brings that engine to developers as a lightweight, USD-first, kitless library.

---

## 2. What functionalities are available, and who are the target users?

**What you can do with it:**

- **Load and simulate USD scenes** — `add_usd("scene.usda")`, `step(dt, ...)`, `release()`. A full sim loop in three calls.
- **Tensor data exchange via DLPack** — read/write simulation state (e.g. rigid-body poses) as zero-copy tensors that interoperate with NumPy, PyTorch, and other ML frameworks.
- **Tensor bindings** — bind to scene patterns (e.g. `RIGID_BODY_POSE`) to stream state in and out efficiently.
- **Environment cloning for batched RL** — replicate environments for high-throughput reinforcement-learning workloads.
- **CPU or GPU simulation** — run with an NVIDIA GPU for acceleration, or fall back to CPU-only.
- **Standalone C/C++ SDK** — pre-built packages on GitHub Releases with ready-to-build samples (`find_package(ovphysx)`): hello world, tensor bindings, cloning, contacts, and **OmniPVD** recording for debugging.


**Who benefits:**

- **Robotics & RL developers** (Isaac Lab / Isaac Sim style workflows) who need fast, batched, headless physics with direct tensor access for policy training and inference.
- **Simulation & digital-twin engineers** who want USD-native physics without pulling in a full Omniverse/Kit stack.
- **C/C++ application developers** integrating real-time physics into non-Python engines and tools via the standalone SDK.
- **Researchers and ML practitioners** who live in NumPy/PyTorch and want simulation state as plain tensors.

---

## 3. Documentation and reference links

- **User Guide & Tutorials:** <https://nvidia-omniverse.github.io/PhysX/ovphysx/latest/index.html>
- **C API Reference:** <https://nvidia-omniverse.github.io/PhysX/ovphysx/latest/api.html>
- **Python API Reference:** <https://nvidia-omniverse.github.io/PhysX/ovphysx/latest/python_api.html>
- **PhysX project home:** <https://nvidia-omniverse.github.io/PhysX/>
- **Source (GitHub):** <https://github.com/NVIDIA-Omniverse/PhysX/tree/main/ovphysx>
- **PyPI:** <https://pypi.org/project/ovphysx/>
- **Pre-built C/C++ SDK (Releases):** <https://github.com/NVIDIA-Omniverse/PhysX/releases>
- **Community & support:** [Discussions](https://github.com/NVIDIA-Omniverse/PhysX/discussions) · [Issues](https://github.com/NVIDIA-Omniverse/PhysX/issues) · [#physics Discord](https://discord.com/invite/XWQNJDNuaC)

---

## 4. System requirements

- Python **3.10+**
- **Linux** (x86_64, aarch64) or **Windows** (x86_64)
- NVIDIA GPU + driver **recommended** for acceleration (CPU-only simulation also supported)
- Current release: **ovphysx 0.4.13**, compatible with **PhysX SDK 5.9.0**

---

## 5. Licensing

- **Source code:** BSD-3-Clause License — permissive, free for commercial and non-commercial use.
- **Pre-built binaries** (SDK packages and Python wheels): distributed under the **NVIDIA Omniverse License**.

> **Note:** ovphysx is pre-release and not yet mature. When sharing a process with other OV USD-aware subsystems, register each subsystem's schema paths before the first USD stage or schema-registry access. Parts of the API may change before 1.0.

---

> **Note:** Pre-release notice: ovphysx is pre-release software and not yet mature. ovphysx packages an OV namespaced OpenUSD runtime; when sharing a process with other OV USD-aware subsystems, register each subsystem's schema paths before the first USD stage or schema-registry access. Parts of the API are still being completed and may change before 1.0.

## Quick Start

```bash
pip install ovphysx
```

```python
from ovphysx import PhysX

physx = PhysX()
physx.add_usd("scene.usda")
physx.step(1.0 / 60.0, 0.0)
physx.release()
```

## Environment Cloning

Clone environments for batched reinforcement-learning workloads:

```python
from ovphysx import PhysX
from ovphysx.types import TensorType
import numpy as np

physx = PhysX(device="cpu")
usd_handle, _ = physx.add_usd("scene.usda")
physx.wait_all()

# Read rigid body poses via DLPack-compatible tensors
pose_binding = physx.create_tensor_binding(
    pattern="/World/envs/env0/table",
    tensor_type=TensorType.RIGID_BODY_POSE,
)
poses = np.zeros(pose_binding.shape, dtype=np.float32)
pose_binding.read(poses)

pose_binding.destroy()
physx.remove_usd(usd_handle)
physx.release()
```

## C/C++ SDK

A standalone C SDK is available for integration into non-Python applications.
Download pre-built packages from the [GitHub Releases](https://github.com/NVIDIA-Omniverse/PhysX/releases) page.

After extracting the SDK, you can build and run a bundled sample directly:

```bash
# /path/to/ovphysx-sdk is the extracted SDK package (pre-built binaries from GitHub Releases)
cmake -B build -S /path/to/ovphysx-sdk/samples/c_samples/hello_world_c -DCMAKE_PREFIX_PATH=/path/to/ovphysx-sdk
cmake --build build
./build/hello_world_c
```

The SDK includes ready-to-build samples in `samples/c_samples/` covering core
workflows (hello world, tensor bindings, cloning, contacts, OmniPVD recording).
Each sample has its own `CMakeLists.txt` that uses `find_package(ovphysx)`.

## Documentation

- [User Guide & Tutorials](https://nvidia-omniverse.github.io/PhysX/ovphysx/latest/index.html)
- [C API Reference](https://nvidia-omniverse.github.io/PhysX/ovphysx/latest/api.html)
- [Python API Reference](https://nvidia-omniverse.github.io/PhysX/ovphysx/latest/python_api.html)

## Links

- [PyPI](https://pypi.org/project/ovphysx/)
- [Documentation](https://nvidia-omniverse.github.io/PhysX/ovphysx/latest/index.html)
- [Issues](https://github.com/NVIDIA-Omniverse/PhysX/issues)
- [Discord](https://discord.com/invite/XWQNJDNuaC)
