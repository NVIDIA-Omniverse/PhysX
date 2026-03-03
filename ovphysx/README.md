# ovphysx

ovphysx is a self-contained library package offering the PhysX SDK and Omni PhysX capabilities behind a C API and corresponding Python bindings for inclusion in user applications.
It loads USD scenes, runs simulation, and allows reading and writing simulation data with DLPack interoperability.

> **Note:** Pre-release notice: ovphysx is pre-release software and not yet mature. Current limitations include a strict in-process USD requirement: ovphysx can only coexist with OpenUSD v25.11 that is non-monolithic, Python-enabled, and linked against oneTBB (in particular, ovphysx cannot be used together with usd-core in the same process today), and parts of the API are still being completed and may change before 1.0.

## Overview

If you consume prebuilt binaries (C SDK or Python wheel), start with the public docs: https://nvidia-omniverse.github.io/PhysX/ovphysx/index.html

## License

The ovphysx source code is licensed under the BSD-3-Clause License - see [LICENSE.txt](LICENSE.txt).

Pre-built binary distributions (SDK packages and Python wheels) are licensed under the [NVIDIA Omniverse License](licenses/LICENSE-binary.txt).
