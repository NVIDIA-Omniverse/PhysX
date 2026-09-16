<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-USD-KINEMATIC-SUPPORT-001
maps_to: REQ-USD-KINEMATIC-SUPPORT-001
type: integration
---

## Scenario

ovstage transform control moves a support through the PhysX kinematic-target
path, and shipped samples distinguish transform motion, surface velocity, and
their combination through contact behavior.

## Given

- An ovphysx instance with a sample scene containing three high-friction
  kinematic platforms and one dynamic rider per platform: transform-only,
  surface-velocity-only, and combined.
- Sleeping disabled on the riders so a settled body observes later
  contact-driving motion.

## When

- The C and Python samples publish world transforms for the translating
  platforms with their authored scale at successive ovstage control ordinals,
  seal and drain each ordinal, step the simulation, and read each rider pose.
- The stationary and combined platforms also carry authored
  `PhysxSurfaceVelocityAPI` values.
- The C sample subscribes to physics-object creation and destruction events
  after initial scene population and before publishing transform updates.

## Then

- Both shipped samples require the transform-only and surface-only riders to
  displace by at least 0.5.
- Both samples require the combined rider displacement to exceed either
  independent displacement by at least 0.2.
- The C sample requires zero physics-object creation and destruction events
  while the transform updates are applied.
- Sample validation fails rather than reporting success if any contact-driving
  lane does not meet those checks.
