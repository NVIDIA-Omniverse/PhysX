# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

"""Verify ovphysx installation: ``python -m ovphysx``."""

import sys

import ovphysx

_skills_paths = ovphysx.ai_skills_path()

if "--skills-path" in sys.argv:
    for key, path in _skills_paths.items():
        print(f"{key}: {path}")
    sys.exit(0)

from ovphysx import PhysX

print(f"ovphysx {ovphysx.__version__}")

physx = PhysX()
physx.step(1.0 / 60.0, 0.0)
physx.release()

print("OK")
print("\nAI skills & samples:")
print(f"  Skills index: {_skills_paths['skills_index']}")
print(f"  Skills dir:   {_skills_paths['skills_dir']}")
print(f"  Samples dir:  {_skills_paths['samples_dir']}")
print(f"  Docs dir:     {_skills_paths['docs_dir']}")
