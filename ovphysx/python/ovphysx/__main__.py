# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

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
# step() rejects handles without an attached stage (NVBugs 6433668), and this
# smoke test has no stage to load. Constructing and destroying PhysX() already
# verifies that the native library loaded, linked and initialized.
physx.destroy()

print("OK")
print("\nAI skills & samples:")
print(f"  Skills index: {_skills_paths['skills_index']}")
print(f"  Skills dir:   {_skills_paths['skills_dir']}")
print(f"  Samples dir:  {_skills_paths['samples_dir']}")
print(f"  Docs dir:     {_skills_paths['docs_dir']}")
