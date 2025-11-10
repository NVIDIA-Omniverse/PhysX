# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import math
from typing import Dict, Union
from typing_extensions import Literal
from omni.physx import get_physxunittests_interface
from pxr import Gf, UsdGeom, UsdPhysics, Usd


class MassTestBase:
    _shape_types = ("cube", "sphere", "cylinder", "capsule")

    def set_units(self, stage: Usd.Stage, mpu: float = 0.01, kgpu: float = 1.0) -> None:
        """Sets the meters and kilograms per unit of a stage and calculates the default density.

        Note: default density is assigned to self.defaultDensity

        Args:
            stage: the stage to change the units of.
            mpu: the meters per unit to set the stage to.
            kgpu: the kilograms per unit to set the stage to.
        """
        self.metersPerUnit = mpu
        self.kilogramsPerUnit = kgpu
        self.defaultDensity = 1000.0 * self.metersPerUnit * self.metersPerUnit * self.metersPerUnit / self.kilogramsPerUnit
        UsdGeom.SetStageMetersPerUnit(stage, self.metersPerUnit)
        UsdPhysics.SetStageKilogramsPerUnit(stage, self.kilogramsPerUnit)

    async def mass_test_stage_setup(self) -> Usd.Stage:
        """Sets up the stage for mass tests with the default values.

        Returns:
            the stage that was setup.
        """
        stage = await self.new_stage()
        self.set_units(stage)  # default scales
        return stage

    def calculate_shape_mass(self, shape_type: Literal["cube", "sphere", "cylinder", "capsule"], dimensions: Gf.Vec3f, density: float = None) -> float:
        """Calculate the mass of a shape with given dimensions and optional density.

        Note: if density is not provided then self.defaultDensity, which is set during the stage setup, will be used.

        Args:
            shape_type: the shape to compute the mass of (cube, sphere, cylinder or capsule).
            dimensions: the dimensions of the shape. Values for N/A fields are not used.
                cube: (length, height, width)
                sphere: (radius, N/A, N/A)
                cylinder: (radius, length, N/A)
                capsule: (radius, length of cylindrical part, N/A)
            density: the density of the shape.

        Returns:
            the mass of the shape.
        """
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
        """Shift an inertia to a new origin using parallel axis theorem.

        Args:
            inertia: the diagonal inertia at the COM.
            offset: the offset of the new origin P to the COM, i.e. the vector from the COM to P.
            mass: the mass of the object.

        Returns:
            the diagonalized inertia at the new origin.
        """
        inertia_matrix = Gf.Matrix3f()
        inertia_matrix.SetDiagonal(inertia)

        s = Gf.Matrix3f()
        s.SetColumn(0, Gf.Vec3f(0, offset[2], -offset[1]))
        s.SetColumn(1, Gf.Vec3f(-offset[2], 0, offset[0]))
        s.SetColumn(2, Gf.Vec3f(offset[1], -offset[0], 0))
        result = s.GetTranspose() * s * mass + inertia_matrix

        return Gf.Vec3f(result[0][0], result[1][1], result[2][2])

    def calculate_shape_inertia(self, shape_type: Literal["cube", "sphere", "capsule"], dimensions: Gf.Vec3f, density: float = None, mass: float = None, axes_offset: Gf.Vec3f = None) -> Gf.Vec3f:
        """Calculate the inertia of a shape with given dimensions and optional density, mass and offset from center of mass.

        Note: if mass is not provided it will be calculated using the dimensions and the density.
        If density is not provided then self.defaultDensity, which is set during the stage setup, will be used.

        Args:
            shape_type: the shape to compute the inertia of (cube, sphere or capsule).
            dimensions: the dimensions of the shape. Values for N/A fields are not used.
                cube: (width (in x), height (in y), depth (in z))
                sphere: (radius, N/A, N/A)
                capsule: (radius, length (in y) of cylindrical part, N/A)
            density: the density of the shape.
            mass: the mass of the shape.
            axes_offset: [optional] the offset to the origin of the parallel axes that the inertia is computed at.
            If it is not provided or zero, the inertia is computed at the center of mass.

        Returns:
            the diagonal inertia [Ixx, Iyy, Izz] at the center of gravity, or at the optional axes_offset.
        """
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
            # this is only valid when the capsule axis is Y
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

        # offset center of mass using parallel axis theorem
        if axes_offset:
            diag = self.calculate_inertia_shift(diag, axes_offset, mass)

        return diag

    async def get_mass_information(self, prim_path: str) -> Dict[str, Union[float, Gf.Vec3f]]:
        """Get the mass, inertia and center of mass of a prim

        Args:
            prim_path: the path of the prim to get the mass information of.

        Returns:
            the mass information (mass, inertia and com) as a dictionary.
        """
        physxUT = get_physxunittests_interface()
        dt = 1.0 / 60.0
        physxUT.update(dt, 0)  # update physx scene (from 0s to 1/60s) to parse and calculate mass properties
        return physxUT.get_mass_information(prim_path)

    async def check_shape_mass_properties(
        self,
        shape_type: Literal["cube", "sphere", "cylinder", "capsule"],
        prim_path: str,
        size: Gf.Vec3f,
        scale: Gf.Vec3f,
        mass: float = None,
        density: float = None,
        inertia: Gf.Vec3f = None,
        com: Gf.Vec3f = None,
        scale_com: bool = False
    ):
        """Checks the mass, inertia and optionally center of mass against the actual values of a prim.

        Note: a relative tolerance of 0.1% is used for comparing floats.
        Mass and inertia will be calculated if not provided using size and scale.

        Args:
            shape_type: the shape of the prim (cube, sphere, cylinder, capsule)
            prim_path: the path of the prim to check the mass properties of.
            size: the size of the prim. Will be scaled to create dimensions. Values for N/A fields are not used.
                cube: (width (in x), height (in y), depth (in z))
                sphere: (radius, N/A, N/A)
                capsule: (radius, length (in y) of cylindrical part, N/A)
            scale: the scale of the prim.
            mass: [optional] the expected mass of the prim.
            density: [optional] the density of the prim to calculate the mass.
            inertia: [optional] the expected inertia of the prim.
            com: [optional] the expected center of mass of the prim
            scale_com: if true will scale the center of mass to the scale provided.

        Raises:
            AssertionError: if a value doesn't match the expected value."""
        self.assertTrue(shape_type in self._shape_types)

        dimensions = Gf.Vec3f(
            size[0] * scale[0],
            size[1] * scale[1],
            size[2] * scale[2]
        )

        scaled_com = com
        if scale_com and com:
            scaled_com[0] = com[0] * scale[0]
            scaled_com[1] = com[1] * scale[1]
            scaled_com[2] = com[2] * scale[2]
        if not mass:
            mass = self.calculate_shape_mass(shape_type, dimensions, density)
        if not inertia:
            inertia = self.calculate_shape_inertia(shape_type, dimensions, density, mass, scaled_com)

        await self.check_mass_properties(prim_path, mass, inertia, scaled_com)
        return mass

    async def check_mass_properties(self, prim_path: str, mass: float, inertia: Gf.Vec3f = None, com: Gf.Vec3f = None) -> None:
        """Checks the mass and inertia and/or center of mass against the actual values of a prim.

        Note: if inertia or com are not provided they will not be checked.
        A relative tolerance of 0.1% is used due to floats.

        Args:
            prim_path: the path of the prim to check the mass properties of.
            mass: the expected mass of the prim.
            inertia: [optional] the expected inertia of the prim at it's center of mass.
            com: [optional] the expected center of mass of the prim in it's local space.

        Raises:
            AssertionError: if a value doesn't match the expected value."""
        massInfo = await self.get_mass_information(prim_path)
        relative_tol = 0.001

        self.assertRelativeAlmostEqual(massInfo["mass"], mass, rel_tol=relative_tol)

        if inertia is not None:
            self.assertFloatIterableAlmostEqual(massInfo["inertia"], inertia, rel_tol=relative_tol)

        if com is not None:
            self.assertFloatIterableAlmostEqual(massInfo["com"], com, rel_tol=relative_tol)
