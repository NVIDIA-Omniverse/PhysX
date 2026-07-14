# ovphysx

[![PyPI](https://img.shields.io/pypi/v/ovphysx)](https://pypi.org/project/ovphysx/)
[![Python 3.10+](https://img.shields.io/badge/python-3.10%2B-blue)](https://pypi.org/project/ovphysx/)
[![Linux | Windows](https://img.shields.io/badge/platform-linux%20%7C%20windows-lightgrey)](https://pypi.org/project/ovphysx/)

ovphysx is a self-contained library for USD-based physics simulation, offering
a C API with Python bindings. It wraps NVIDIA PhysX and Omni PhysX,
loads USD scenes, runs simulation, and exchanges data via DLPack tensors —
no Omniverse installation required.

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

## Requirements

- Python 3.10+
- Linux (x86_64, aarch64) or Windows (x86_64)
- NVIDIA GPU + driver recommended (CPU-only simulation also supported)

## License

The ovphysx source code is licensed under the BSD-3-Clause License — see [LICENSE.txt](LICENSE.txt).

Pre-built binary distributions (SDK packages and Python wheels) are licensed under the [NVIDIA Omniverse License](licenses/LICENSE-binary.txt).

## Links

- [PyPI](https://pypi.org/project/ovphysx/)
- [Documentation](https://nvidia-omniverse.github.io/PhysX/ovphysx/latest/index.html)
- [Issues](https://github.com/NVIDIA-Omniverse/PhysX/issues)
- [Discord](https://discord.com/invite/XWQNJDNuaC)
