# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
# Ported from omni.physx.tests massTestBase.py for standalone ovruntime testing.
# Converted async def -> def for unittest.TestCase usage.

import math
from typing import Dict, Union
from pxr import Gf, UsdGeom, UsdPhysics, Usd
import _physx


class MassTestBase:
    _shape_types = ("cube", "sphere", "cylinder", "capsule")

    def set_units(self, stage: Usd.Stage, mpu: float = 0.01, kgpu: float = 1.0) -> None:
        """Sets the meters and kilograms per unit of a stage and calculates the default density."""
        self.metersPerUnit = mpu
        self.kilogramsPerUnit = kgpu
        self.defaultDensity = 1000.0 * self.metersPerUnit * self.metersPerUnit * self.metersPerUnit / self.kilogramsPerUnit
        UsdGeom.SetStageMetersPerUnit(stage, self.metersPerUnit)
        UsdPhysics.SetStageKilogramsPerUnit(stage, self.kilogramsPerUnit)

    def mass_test_stage_setup(self) -> Usd.Stage:
        """Sets up the stage for mass tests with the default values."""
        stage = self.new_stage()
        self.set_units(stage)
        return stage

    def calculate_shape_mass(self, shape_type, dimensions: Gf.Vec3f, density: float = None) -> float:
        """Calculate the mass of a shape with given dimensions and optional density."""
        self.assertTrue(shape_type in self._shape_types)
        if not density:
            self.assertTrue(hasattr(self, "defaultDensity"), msg="defaultDensity is None, please provide a density or use mass_test_stage_setup().")
        volume = 0
        if shape_type == "cube":
            volume = dimensions[0] * dimensions[1] * dimensions[2]
        elif shape_type == "sphere":
            radius = dimensions[0]
            volume = (4.0 * math.pi * radius * radius * radius) / 3.0
        elif shape_type == "cylinder":
            radius = dimensions[0]
            length = dimensions[1]
            volume = math.pi * radius * radius * length
        elif shape_type == "capsule":
            radius = dimensions[0]
            length = dimensions[1]
            volume = math.pi * radius * radius * (4.0 * radius / 3.0 + length)

        density = density if density else self.defaultDensity
        return density * volume

    def calculate_inertia_shift(self, inertia: Gf.Vec3f, offset: Gf.Vec3f, mass: float) -> Gf.Vec3f:
        """Shift an inertia to a new origin using parallel axis theorem."""
        inertia_matrix = Gf.Matrix3f()
        inertia_matrix.SetDiagonal(inertia)

        s = Gf.Matrix3f()
        s.SetColumn(0, Gf.Vec3f(0, offset[2], -offset[1]))
        s.SetColumn(1, Gf.Vec3f(-offset[2], 0, offset[0]))
        s.SetColumn(2, Gf.Vec3f(offset[1], -offset[0], 0))
        result = s.GetTranspose() * s * mass + inertia_matrix

        return Gf.Vec3f(result[0][0], result[1][1], result[2][2])

    def calculate_shape_inertia(self, shape_type, dimensions: Gf.Vec3f, density: float = None, mass: float = None, axes_offset: Gf.Vec3f = None) -> Gf.Vec3f:
        """Calculate the inertia of a shape with given dimensions."""
        self.assertTrue(shape_type in ("cube", "sphere", "capsule"))
        if not mass:
            mass = self.calculate_shape_mass(shape_type, dimensions, density)

        diag = Gf.Vec3f(0.0)

        if shape_type == "cube":
            mass_mod = mass / 12.0
            diag = Gf.Vec3f(
                mass_mod * (dimensions[1] * dimensions[1] + dimensions[2] * dimensions[2]),
                mass_mod * (dimensions[0] * dimensions[0] + dimensions[2] * dimensions[2]),
                mass_mod * (dimensions[0] * dimensions[0] + dimensions[1] * dimensions[1])
            )
        elif shape_type == "sphere":
            radius = dimensions[0]
            mass_mod = 2.0 * mass / 5.0
            diag = Gf.Vec3f(
                mass_mod * radius * radius,
                mass_mod * radius * radius,
                mass_mod * radius * radius
            )
        elif shape_type == "capsule":
            radius = dimensions[0]
            length = dimensions[1]
            if mass:
                volume = math.pi * radius * radius * (4.0 * radius / 3.0 + length)
                density = mass / volume
            mass_hemi = self.calculate_shape_mass("sphere", dimensions, density) * 0.5
            mass_cyl = self.calculate_shape_mass("cylinder", dimensions, density)
            first_term = mass_cyl * (length * length / 12.0 + radius * radius / 4.0) + 2.0 * mass_hemi * (
                2.0 * radius * radius / 5.0 + length * length / 4.0 + 3.0 * length * radius / 8.0)
            diag = Gf.Vec3f(
                first_term,
                mass_cyl * (radius * radius / 2.0) + 4.0 * mass_hemi * radius * radius / 5.0,
                first_term
            )

        if axes_offset:
            diag = self.calculate_inertia_shift(diag, axes_offset, mass)

        return diag

    def get_mass_information(self, prim_path: str) -> Dict[str, Union[float, Gf.Vec3f]]:
        """Get the mass, inertia and center of mass of a prim."""
        physxUT = _physx.acquire_physxunittests_interface()
        dt = 1.0 / 60.0
        physxUT.update(dt, 0)
        return physxUT.get_mass_information(prim_path)

    def check_shape_mass_properties(
        self, shape_type, prim_path: str, size: Gf.Vec3f, scale: Gf.Vec3f,
        mass: float = None, density: float = None, inertia: Gf.Vec3f = None,
        com: Gf.Vec3f = None, scale_com: bool = False
    ):
        """Checks the mass, inertia and optionally center of mass against the actual values of a prim."""
        self.assertTrue(shape_type in self._shape_types)
        dimensions = Gf.Vec3f(size[0] * scale[0], size[1] * scale[1], size[2] * scale[2])

        scaled_com = com
        if scale_com and com:
            scaled_com[0] = com[0] * scale[0]
            scaled_com[1] = com[1] * scale[1]
            scaled_com[2] = com[2] * scale[2]
        if not mass:
            mass = self.calculate_shape_mass(shape_type, dimensions, density)
        if not inertia:
            inertia = self.calculate_shape_inertia(shape_type, dimensions, density, mass, scaled_com)

        self.check_mass_properties(prim_path, mass, inertia, scaled_com)
        return mass

    def check_mass_properties(self, prim_path: str, mass: float, inertia: Gf.Vec3f = None, com: Gf.Vec3f = None) -> None:
        """Checks the mass and inertia and/or center of mass against the actual values of a prim."""
        massInfo = self.get_mass_information(prim_path)
        relative_tol = 0.001

        self.assertRelativeAlmostEqual(massInfo["mass"], mass, rel_tol=relative_tol)

        if inertia is not None:
            self.assertFloatIterableAlmostEqual(massInfo["inertia"], inertia, rel_tol=relative_tol)

        if com is not None:
            self.assertFloatIterableAlmostEqual(massInfo["com"], com, rel_tol=relative_tol)
