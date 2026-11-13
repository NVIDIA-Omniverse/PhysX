# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PYTHON-ATTRIBUTES-001
# @covers AC-1 AC-2 AC-3 AC-4
"""Static discovery must work on a machine without the native SDK."""

import dataclasses
import subprocess
import sys
import unittest

from ovphysx import attributes


class AttributeMetadataTest(unittest.TestCase):
    def test_import_never_loads_native_dependencies(self):
        code = """
import importlib.abc
import sys
class RejectNative(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path, target=None):
        if fullname.split('.')[0] in {'warp', 'pxr', 'ovstage', 'numpy', 'ctypes'} or fullname in {'ovphysx.api', 'ovphysx._bindings', 'ovphysx._native'}:
            raise AssertionError('native dependency imported: ' + fullname)
sys.meta_path.insert(0, RejectNative())
from ovphysx import attributes
assert attributes.get_attribute('RIGID_BODY', 'angularVelocity').units == 'angular_velocity'
"""
        subprocess.run([sys.executable, "-c", code], check=True)

    def test_query_filters_and_unknown_values(self):
        all_attrs = attributes.list_attributes()
        self.assertGreater(len(all_attrs), 200)
        rigid = attributes.list_attributes("rigid_body", writable=True, representation="default")
        self.assertTrue(rigid)
        self.assertTrue(all(a.target == "RIGID_BODY" and a.write["supported"] for a in rigid))
        self.assertEqual(
            rigid, attributes.list_attributes(attributes.TARGETS["RIGID_BODY"], writable=True, representation="default")
        )
        self.assertEqual(attributes.list_attributes("SDF_SHAPE"), ())
        self.assertIs(
            attributes.get_attribute("PARTICLE_SET", "positions"), attributes.get_attribute("PARTICLE_SET", "points")
        )
        with self.assertRaises(ValueError):
            attributes.list_attributes("UNKNOWN")
        with self.assertRaises(ValueError):
            attributes.list_attributes(True)
        with self.assertRaises(TypeError):
            attributes.list_attributes(readable=1)
        with self.assertRaises(KeyError):
            attributes.get_attribute("RIGID_BODY", "unknown")
        for invalid in ("unknown", "", None):
            with self.assertRaises(ValueError):
                attributes.get_attribute("RIGID_BODY", "position", representation=invalid)
        with self.assertRaises(ValueError):
            attributes.list_attributes(representation="unknown")

    def test_query_target_enum_is_not_a_legacy_object_type(self):
        from enum import Enum, IntEnum

        from ovphysx.types import ObjectType, SimObjectType

        class Other(IntEnum):
            RIGID_BODY = 0

        class Text(str, Enum):
            RIGID_BODY = "RIGID_BODY"

        expected = attributes.list_attributes("RIGID_BODY")
        self.assertEqual(attributes.list_attributes(SimObjectType.RIGID_BODY), expected)
        self.assertEqual(attributes.list_attributes(int(SimObjectType.RIGID_BODY)), expected)
        for target in (ObjectType.RIGID_BODY, Other.RIGID_BODY, Text.RIGID_BODY):
            with self.subTest(target=target), self.assertRaises(ValueError):
                attributes.list_attributes(target)
        culling = attributes.get_attribute("DEBUG_RENDER", "cullingBox")
        self.assertEqual(culling.units, "length")
        self.assertEqual(attributes.UNITS[culling.write["units"]]["components"], ("dimensionless",) + ("length",) * 6)
        wake = attributes.get_attribute("RIGID_BODY", "wake")
        self.assertFalse(wake.read["supported"])
        self.assertNotIn("layout", wake.read)

    def test_target_specific_units_and_layout(self):
        angular = attributes.get_attribute("RIGID_BODY", "angularVelocity")
        self.assertEqual(angular.units, "angular_velocity")
        self.assertEqual(angular.read["layout"]["lanes"], 3)
        self.assertEqual(angular.frame, "world")
        joint = attributes.get_attribute("ARTICULATION_JOINT", "jointPosition")
        self.assertEqual(joint.units, "joint_coordinate")
        self.assertEqual(joint.read["layout"]["kind"], "fixed")  # one row per DOF (ADR-0059)
        self.assertEqual(joint.write["layout"]["kind"], "fixed")
        self.assertEqual(joint.unit["variants"]["angular"], "deg")
        self.assertEqual(joint.unit["variants"]["linear"], "distance")
        instancer = attributes.get_attribute("RIGID_BODY", "positions", representation="instancer")
        self.assertEqual(instancer.read["layout"]["kind"], "array")
        self.assertEqual(instancer.frame, "instancer_local")
        self.assertIs(instancer, attributes.get_attribute("RIGID_BODY", "position", representation="instancer"))
        self.assertEqual(instancer.read["request_name"], "position")
        self.assertEqual(instancer.write["request_name"], "position")
        self.assertEqual(instancer.read["aliases"], ())
        particles = attributes.get_attribute("PARTICLE_SET", "points")
        self.assertEqual(particles.read["aliases"], ("positions",))
        self.assertEqual(particles.write["aliases"], ())
        self.assertEqual(particles.write["request_name"], "points")
        scope = attributes.get_attribute("DEBUG_RENDER", "scope")
        self.assertNotEqual(scope.read["layout"], scope.write["layout"])

    def test_deep_immutability(self):
        attr = attributes.get_attribute("RIGID_BODY", "position")
        with self.assertRaises(dataclasses.FrozenInstanceError):
            attr.name = "other"
        with self.assertRaises(TypeError):
            attr.read["layout"]["lanes"] = 12
        with self.assertRaises(TypeError):
            attributes.UNITS["joint_coordinate"]["variants"]["angular"] = "rad"
        with self.assertRaises(TypeError):
            attributes.UNITS["wrench"]["components"][0] = "length"
        self.assertEqual(attributes.NAMES["position"], "position")
        self.assertEqual(hash(attr), hash(dataclasses.replace(attr)))
        self.assertEqual({attr: "position"}[dataclasses.replace(attr)], "position")

    def test_readable_runtime_units(self):
        expected = [
            ("RIGID_BODY", "position", "distance"),
            ("RIGID_BODY", "linearVelocity", "distance / sec"),
            ("RIGID_BODY", "angularVelocity", "rad / sec"),
            ("RIGID_BODY", "mass", "mass"),
            ("RIGID_BODY", "force", "mass * distance / sec^2"),
            ("VEHICLE_WHEEL", "tireLateralSlip", "rad"),
            ("VEHICLE_WHEEL", "driveTorque", "mass * distance^2 / sec^2"),
            ("SPATIAL_TENDON", "tendonOffset", "distance"),
            ("SPATIAL_TENDON", "tendonStiffness", "mass / sec^2"),
            ("SPATIAL_TENDON", "tendonDamping", "mass / sec"),
        ]
        for target, name, display in expected:
            with self.subTest(target=target, name=name):
                self.assertEqual(attributes.get_attribute(target, name).unit["display"], display)
        self.assertEqual(
            attributes.get_attribute("ARTICULATION", "jacobian").unit["variants"]["linear_row_angular_column"],
            "distance / rad",
        )
        self.assertEqual(
            attributes.get_attribute("ARTICULATION", "gravityForce").unit["variants"]["angular"],
            "mass * distance^2 / sec^2",
        )
        self.assertEqual(
            attributes.get_attribute("FIXED_TENDON", "tendonOffset").unit["variants"]["unit_authored_gearing_angular"],
            "deg",
        )
        self.assertEqual(
            attributes.get_attribute("RIGID_BODY", "wrench").unit["components"],
            ("force",) * 3 + ("torque",) * 3 + ("length",) * 3,
        )
        for unit in attributes.UNITS.values():
            self.assertNotIn("dimensions", unit)
            self.assertNotEqual(unit["display"], "1")
            self.assertTrue(all(isinstance(v, str) for v in unit.get("variants", {}).values()))

    def test_vehicle_value_constraints(self):
        gear = attributes.get_attribute("VEHICLE", "requestedGear")
        self.assertEqual(gear.write["value_constraints"]["minimum"], -1)
        self.assertEqual(gear.write["value_constraints"]["maximum"], 30)
        self.assertEqual(gear.write["value_constraints"]["additional_values"], (255,))
        self.assertEqual(gear.write["applicability"], "DriveBasic / DriveStandard.")
        brake = attributes.get_attribute("VEHICLE_WHEEL", "brakeTorque")
        self.assertEqual(brake.write["value_constraints"]["minimum"], 0)
        with self.assertRaises(TypeError):
            brake.write["value_constraints"]["minimum"] = -1


if __name__ == "__main__":
    unittest.main()
