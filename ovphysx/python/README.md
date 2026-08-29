<!-- SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: BSD-3-Clause -->

# ovphysx

ovphysx is a self-contained Python library for ovstage/USD-based physics simulation
with DLPack tensor interoperability. It wraps NVIDIA PhysX and provides:

- ovstage attachment plus rigid-body / articulation simulation
- Same-device zero-copy tensor exchange with transparent CPU/CUDA staging for NumPy, PyTorch, and other DLPack frameworks
- Environment cloning for batched reinforcement-learning workloads

## Quick start

```bash
pip install ovphysx
```

```python
import ovstage
from ovphysx import PhysX

stage = ovstage.Stage("scene")
ovstage.population.open_usd(
    stage, "scene.usda", ordinal=1, domains=ovstage.PopulationDomain.PHYSICS
)
# attach_ovstage() reads at a sealed ordinal.
stage.advance_write_floor(ordinal=1).wait()

physx = PhysX()
physx.attach_ovstage(stage, read_ordinal=1)
physx.step(1.0 / 60.0)
physx.detach_ovstage()
physx.release()
stage.destroy()
```

For full documentation and tutorials, see `ovphysx/docs/` inside the
installed package, or the [ovphysx repository](https://github.com/NVIDIA-Omniverse/PhysX).
