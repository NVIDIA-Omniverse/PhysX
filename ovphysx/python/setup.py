# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

from setuptools import setup
from setuptools.dist import Distribution


class BinaryDistribution(Distribution):
    """Force platform-specific wheel (Root-Is-Purelib: false) while keeping
    the py3-none ABI tag (we bundle pre-built .so files, not cpython extensions)."""
    def has_ext_modules(self):
        return True


try:
    from setuptools.command.bdist_wheel import bdist_wheel
except ImportError:
    from wheel.bdist_wheel import bdist_wheel


class PlatWheel(bdist_wheel):
    def get_tag(self):
        impl, abi, plat = super().get_tag()
        return "py3", "none", plat


setup(
    distclass=BinaryDistribution,
    cmdclass={"bdist_wheel": PlatWheel},
)
