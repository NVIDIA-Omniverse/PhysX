<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Known limitations and gaps

Current boundaries of the session read/write API and unsupported combinations. See [the session model](index.md) for the API itself.

- **Capabilities gap.** A few things have no non-deprecated
  successor yet: articulation topology / metadata and DOF / body / joint **names**, rigid-body
  **wake / sleep** control, and the volume-deformable **`kinematicTarget`** (not yet served by the
  role-based read). Until a replacement lands, these require the deprecated tensor-binding API.
- **`ovphysx_update_articulations_kinematic` is a partial FK successor.** It refreshes *every*
  articulation in the instance, ignores position/velocity flags, and is a **no-op on CPU**. It is not
  a drop-in replacement for the deprecated per-binding kinematic update.
- **Very large articulation inverse dynamics.** A single inverse dynamics column wider than 65 535 lanes is dropped
  and the read errors (see the note under [Whole articulations](readable.md#whole-articulations)).
- **Unsupported device combinations.** Vehicles are CPU-only; deformables and particles require a
  CUDA context (they emit nothing on a sim without one).
- **`ovphysx_writability` is not the ovstage-drain coverage map.** It reports what the *imperative
  write session* accepts. The separate ovstage value-change drain deliberately does **not** apply
  some of those attributes (tendon knobs, the rigid mass frame, deformable bind-pose points,
  local-space-velocity bodies, multi-axis D6 joints), falling back to the parse path — and on a
  DirectGPU scene a tendon change via the drain does not reliably take effect. Do not treat the
  writability query as the drain's contract.

## See also

- [C API Reference](../api.md) — entry-point signatures and Doxygen
- [Python API Reference](../python_api.rst) — `PhysX.read` / `PhysX.write`
- [ovphysx overview](../ovphysx_overview.md) — the simulation loop and core concepts
- [Population](../population/index.md) and [Stage units](../population/stage_units.md) — how a stage
  is populated from schema, fallbacks, and unit scaling
