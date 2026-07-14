# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary

from setuptools import setup
from setuptools.command.develop import develop
from setuptools.command.editable_wheel import editable_wheel
import sys
from pathlib import Path

class CustomDevelop(develop):
    """Custom develop command to create .pth file with DLL directories"""
    def run(self):
        develop.run(self)
        self._create_pth_file()

    def _create_pth_file(self):
        """Create a .pth file to add bindings dir and DLL directories to sys.path"""
        import site
        site_packages = site.getsitepackages()[0] if site.getsitepackages() else None
        if not site_packages:
            # Fallback for venv
            site_packages = Path(sys.prefix) / "Lib" / "site-packages"

        pth_file = Path(site_packages) / "physics-umbrella.pth"
        bindings_dir = Path(__file__).parent.resolve()

        # Paths to DLL directories (staged bindings dir has all DLLs, install dirs as fallback)
        dll_dirs = [
            bindings_dir.parent / "_build" / "windows-x86_64" / "bindings",
            bindings_dir.parent / "_install" / "umbrella" / "release" / "bin",
            bindings_dir.parent / "_install" / "umbrella" / "debug" / "bin",
        ]

        with open(pth_file, 'w') as f:
            # Add DLL directories (Windows only)
            for dll_dir in dll_dirs:
                f.write(f"import os; os.add_dll_directory(r'{dll_dir}') if hasattr(os, 'add_dll_directory') and {dll_dir.exists()} else None\n")
            # Add bindings directory to sys.path
            f.write(f"{bindings_dir}\n")

class CustomEditableWheel(editable_wheel):
    """Custom editable_wheel command for modern setuptools/uv"""
    def run(self):
        super().run()
        # Create the .pth file after editable wheel install
        dev = CustomDevelop(self.distribution)
        dev._create_pth_file()

setup(
    cmdclass={
        'develop': CustomDevelop,
        'editable_wheel': CustomEditableWheel,
    },
)
