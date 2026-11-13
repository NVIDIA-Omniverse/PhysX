# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# [example-start]
import textwrap

from ovphysx import attributes

# The names that a RIGID_BODY read accepts. This needs no scene.
names = attributes.list_attributes(
    "RIGID_BODY", readable=True, representation="default"
)
print(textwrap.fill(", ".join(a.name for a in names), width=80))

# The unit and frame of one of them.
spin = attributes.get_attribute("RIGID_BODY", "angularVelocity")
print(f"{spin.name}: {spin.unit['display']}, {spin.frame} frame")
# [example-end]
