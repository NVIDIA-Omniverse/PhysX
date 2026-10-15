# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-PYTHON-001
# @covers AC-1 AC-2 AC-5

"""Optional companion utilities; importing this package initializes no runtime.

USD helpers live in :mod:`ovphysx_companion.usd` and use customer-installed pxr.
Native services are acquired from the host library through the native and preview modules.
"""

# The companion ships and is qualified with the core from the same tree, so it
# has no version of its own. Resolution order:
#   1. _version.py, stamped from (2) by setup.py into sdist and wheel outputs.
#      Never present in the source tree.
#   2. The repo VERSION file (companion/python/ovphysx_companion/__init__.py ->
#      ovphysx/VERSION), for source-tree and editable use.
#   3. The installed distribution metadata.
#   4. "unknown" when none is available.
def _resolve_version():
    from pathlib import Path
    try:
        from ._version import __version__ as stamped
        return stamped
    except ImportError:
        pass
    package = Path(__file__).resolve().parent
    if package.parents[1].name == "companion":
        try:
            return (package.parents[2] / "VERSION").read_text().strip()
        except OSError:
            pass
    try:
        from importlib.metadata import version
        return version("ovphysx-companion")
    except Exception:  # noqa: BLE001
        return "unknown"


__version__ = _resolve_version()
del _resolve_version
