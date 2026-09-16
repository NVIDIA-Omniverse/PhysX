<!-- SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# ovphysx

ovphysx is a self-contained Python library for ovstage/USD-based physics simulation
with Warp-array output and downstream DLPack interoperability. It wraps NVIDIA
PhysX and provides:

- ovstage attachment plus rigid-body / articulation simulation
- Same-device `warp.array` output on CPU and CUDA through `PhysX.read()` / `read_tokens()`
- Downstream zero-copy framework interoperability through Warp's DLPack support
- Environment cloning for batched reinforcement-learning workloads

## Quick start

```bash
pip install ovphysx
```

```python
from pathlib import Path

import ovphysx
import ovstage
from ovphysx import PhysX

usd_path = (
    Path(ovphysx.__file__).resolve().parent
    / "samples"
    / "data"
    / "simple_physics_scene.usda"
)
if not usd_path.is_file():
    raise FileNotFoundError(f"ovphysx sample data is missing: {usd_path}")

# ovphysx ships its PhysX USD schemas as codeless resources and never registers
# them itself; register them with ovstage before the first population call.
ovstage.population.register_usd_schemas([str(ovphysx.codeless_schema_root())])
stage = ovstage.Stage("scene")
ovstage.population.open_usd(
    stage, str(usd_path), ordinal=1, domains=ovstage.PopulationDomain.PHYSICS
)
# attach_ovstage() reads at a sealed ordinal.
stage.advance_write_floor(ordinal=1).wait()

physx = PhysX()
physx.attach_ovstage(stage, read_ordinal=1)
physx.step_sync(1.0 / 60.0)
physx.detach_ovstage()
physx.destroy()
stage.destroy()
```

For full documentation and tutorials, see `ovphysx/docs/` inside the
installed package, or the [ovphysx repository](https://github.com/NVIDIA-Omniverse/PhysX).
