# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-PYTHON-001
# @covers AC-5
"""Carry the shared ovphysx/VERSION into built artifacts.

pyproject.toml reads the version from ovphysx_companion.__version__, which
resolves ovphysx/VERSION in a source checkout. An sdist has no parent ovphysx/
tree, so both the sdist and the wheel get a generated _version.py. Building a
wheel from that sdist then resolves the same version. Nothing is written into
the source tree.
"""
from pathlib import Path

from setuptools import setup
from setuptools.command.build_py import build_py
from setuptools.command.sdist import sdist

_STAMP = Path("ovphysx_companion") / "_version.py"


def _write_stamp(root, version):
    if version in ("", "unknown", "0.0.0"):
        raise RuntimeError("ovphysx_companion version is unresolved; build from the ovphysx tree or its sdist")
    path = Path(root) / _STAMP
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.\n"
        "# SPDX-License-Identifier: Apache-2.0\n"
        "#\n"
        "# Generated from ovphysx/VERSION by companion/python/setup.py.\n"
        f'__version__ = "{version}"\n',
        encoding="utf-8",
    )


class _BuildPy(build_py):
    def run(self):
        super().run()
        _write_stamp(self.build_lib, self.distribution.get_version())


class _Sdist(sdist):
    def make_release_tree(self, base_dir, files):
        super().make_release_tree(base_dir, files)
        _write_stamp(base_dir, self.distribution.get_version())


setup(cmdclass={"build_py": _BuildPy, "sdist": _Sdist})
