# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

"""Verify that OVPHYSX_LIB can select the installed wheel's bundled library."""

from __future__ import annotations

import importlib.util
import os
import sys
from pathlib import Path


def main() -> None:
    package_spec = importlib.util.find_spec("ovphysx")
    if package_spec is None or package_spec.origin is None:
        raise RuntimeError("Installed ovphysx package was not found")

    library_name = "ovphysx.dll" if sys.platform == "win32" else "libovphysx.so"
    library_path = Path(package_spec.origin).parent / "lib" / library_name
    if not library_path.is_file():
        raise RuntimeError(f"Bundled ovphysx library was not found: {library_path}")

    os.environ["OVPHYSX_LIB"] = str(library_path)

    from ovphysx import PhysX

    PhysX.set_cpu_mode(True)
    physx = PhysX()
    physx.release()


if __name__ == "__main__":
    main()
