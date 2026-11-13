# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

import importlib.metadata
import importlib.resources
import os
import re
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

import physx_usd_schemas
from physx_usd_schemas import PhysxSchema
from physx_usd_schemas import validation
from pxr import Usd, UsdGeom, UsdPhysics, UsdValidation

# A compiled PhysX schema plugin as a host such as Kit registers it. Its library is
# never loaded because the test only inspects plugin registration.
_HOST_PHYSX_SCHEMA_PLUGIN = """{
    "Plugins": [{
        "Info": {"Types": {"PhysxSchemaPhysxRigidBodyAPI": {
            "alias": {"UsdSchemaBase": "PhysxRigidBodyAPI"},
            "bases": ["UsdAPISchemaBase"],
            "schemaIdentifier": "PhysxRigidBodyAPI",
            "schemaKind": "singleApplyAPI"
        }}},
        "LibraryPath": "../../libphysxSchema.so",
        "Name": "physxSchema",
        "ResourcePath": "resources",
        "Root": "..",
        "Type": "library"
    }]
}
"""


class PhysxSchemaValidationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not physx_usd_schemas.register_validators():
            raise RuntimeError("OpenUSD 26.5 or newer is required for validator tests")
        cls.registry = UsdValidation.ValidationRegistry()
        cls.validator = cls.registry.GetOrLoadValidatorByName(validation.VALIDATOR_NAME)
        if cls.validator is None:
            raise RuntimeError(f"validator {validation.VALIDATOR_NAME!r} is missing after registration")

    def _run_fresh_python(self, source, **environment_overrides):
        environment = os.environ.copy()
        environment.pop("PYTHONPATH", None)
        environment.update(environment_overrides)
        with tempfile.TemporaryDirectory() as directory:
            result = subprocess.run(
                [sys.executable, "-c", source],
                cwd=directory,
                env=environment,
                capture_output=True,
                text=True,
            )
        self.assertEqual(result.returncode, 0, result.stderr)

    def _define_case(self, prim_type, *, collision=False, merge=False, applicable=True):
        stage = Usd.Stage.CreateInMemory()
        schema = getattr(UsdGeom, prim_type)
        prim = schema.Define(stage, "/Shape").GetPrim()
        if applicable:
            UsdPhysics.MeshCollisionAPI.Apply(prim)
        if collision:
            UsdPhysics.CollisionAPI.Apply(prim)
        if merge:
            PhysxSchema.PhysxMeshMergeCollisionAPI.Apply(prim)
        return stage, prim

    def _assert_errors(self, stage, prim, errors, expected):
        self.assertEqual(
            [(error.GetName(), error.GetMessage()) for error in errors],
            expected,
        )
        for error in errors:
            self.assertEqual(
                error.GetIdentifier(),
                f"{validation.VALIDATOR_NAME}.{error.GetName()}",
            )
            self.assertEqual(error.GetType(), UsdValidation.ValidationErrorType.Error)
            sites = error.GetSites()
            self.assertEqual(len(sites), 1)
            self.assertEqual(sites[0].GetStage(), stage)
            self.assertEqual(sites[0].GetPrim(), prim)

    def test_metadata_uses_explicit_registration_and_exact_schema_type(self):
        metadata = self.validator.GetMetadata()
        self.assertEqual(metadata.name, validation.VALIDATOR_NAME)
        self.assertIsNone(metadata.plugin)
        self.assertEqual(metadata.GetSchemaTypes(), ["UsdPhysicsMeshCollisionAPI"])

    def test_package_import_before_registry_supports_explicit_registration(self):
        self._run_fresh_python("""
import physx_usd_schemas
from pxr import UsdValidation
from physx_usd_schemas.validation import VALIDATOR_NAME
registry = UsdValidation.ValidationRegistry()
assert not registry.HasValidator(VALIDATOR_NAME)
assert physx_usd_schemas.register_validators()
assert registry.HasValidator(VALIDATOR_NAME)
""")

    def test_registry_before_package_import_supports_explicit_registration(self):
        self._run_fresh_python("""
from pxr import UsdValidation
registry = UsdValidation.ValidationRegistry()
assert not registry.HasValidator("physxSchemaValidators:MeshCollisionAPIChecker")
import physx_usd_schemas
from physx_usd_schemas.validation import VALIDATOR_NAME
assert not registry.HasValidator(VALIDATOR_NAME)
assert physx_usd_schemas.register_validators()
assert registry.HasValidator(VALIDATOR_NAME)
""")

    def test_missing_registration_api_keeps_schema_import_usable(self):
        self._run_fresh_python("""
from pxr import Usd, UsdValidation
def registry_must_not_be_created():
    raise AssertionError("unsupported OpenUSD must not create ValidationRegistry")

UsdValidation.ValidationRegistry = registry_must_not_be_created
import physx_usd_schemas
from physx_usd_schemas import PhysxSchema
assert physx_usd_schemas.register_validators() is False
stage = Usd.Stage.CreateInMemory()
prim = stage.DefinePrim("/Body", "Xform")
assert PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
assert prim.HasAPI("PhysxRigidBodyAPI")
""")

    def test_import_keeps_a_host_physx_schema_plugin(self):
        check = """
import os
import physx_usd_schemas
from pxr import Plug, Tf
registry = Plug.Registry()
host = registry.GetPluginWithName("physxSchema")
assert host is not None and not host.isResource, host
physx_usd_schemas.register_all()
provider = registry.GetPluginForType(Tf.Type.FindByName("PhysxSchemaPhysxRigidBodyAPI"))
assert provider.path == host.path, provider.path
deformable = registry.GetPluginWithName("omniUsdPhysicsDeformableSchema")
assert deformable.path.startswith(os.path.dirname(physx_usd_schemas.__file__)), deformable.path
"""
        with tempfile.TemporaryDirectory() as directory:
            resources = os.path.join(directory, "physxSchema", "resources")
            os.makedirs(resources)
            with open(os.path.join(resources, "plugInfo.json"), "w", encoding="utf-8") as plug_info:
                plug_info.write(_HOST_PHYSX_SCHEMA_PLUGIN)
            with self.subTest(host="RegisterPlugins"):
                self._run_fresh_python(
                    f"from pxr import Plug\nPlug.Registry().RegisterPlugins([{resources!r}])\n{check}"
                )
            with self.subTest(host="PXR_PLUGINPATH_NAME"):
                self._run_fresh_python(check, PXR_PLUGINPATH_NAME=resources)

    def test_late_schema_registration_is_a_tool_error(self):
        self._run_fresh_python("""
from pxr import Usd, UsdValidation

stage = Usd.Stage.CreateInMemory()
assert stage.GetRootLayer().ImportFromString('''
#usda 1.0
def Xform "Merged" (
    prepend apiSchemas = [
        "PhysicsCollisionAPI",
        "PhysicsMeshCollisionAPI",
        "PhysxMeshMergeCollisionAPI"
    ]
)
{
}
''')

import physx_usd_schemas
from physx_usd_schemas.validation import VALIDATOR_NAME

assert physx_usd_schemas.register_validators()
validator = UsdValidation.ValidationRegistry().GetOrLoadValidatorByName(
    VALIDATOR_NAME
)
try:
    validator.Validate(stage.GetPrimAtPath("/Merged"))
except RuntimeError as error:
    assert "Restart the process and import physx_usd_schemas" in str(error)
else:
    raise AssertionError("late schema registration must be a tool error")
""")

    def test_direct_validate_preserves_physx_col_002_behavior(self):
        invalid_message = (
            "Prim '/Shape' has MeshCollisionAPI but is not a UsdGeom Mesh "
            "nor a prim with PhysxMeshMergeCollisionAPI."
        )
        missing_collision_message = "Prim '/Shape' has MeshCollisionAPI but does not have CollisionAPI."
        cases = [
            ("Mesh", True, False, True, []),
            (
                "Mesh",
                False,
                False,
                True,
                [
                    (
                        validation.MESH_COLLISION_REQUIRES_COLLISION_API,
                        missing_collision_message,
                    )
                ],
            ),
            (
                "Cube",
                True,
                False,
                True,
                [
                    (
                        validation.MESH_COLLISION_API_INVALID_PRIM,
                        invalid_message,
                    )
                ],
            ),
            (
                "Xform",
                False,
                False,
                True,
                [
                    (
                        validation.MESH_COLLISION_API_INVALID_PRIM,
                        invalid_message,
                    ),
                    (
                        validation.MESH_COLLISION_REQUIRES_COLLISION_API,
                        missing_collision_message,
                    ),
                ],
            ),
            (
                "Xform",
                False,
                True,
                True,
                [
                    (
                        validation.MESH_COLLISION_REQUIRES_COLLISION_API,
                        missing_collision_message,
                    )
                ],
            ),
            ("Xform", True, True, True, []),
            ("Xform", False, False, False, []),
        ]
        for prim_type, collision, merge, applicable, expected in cases:
            with self.subTest(
                prim_type=prim_type,
                collision=collision,
                merge=merge,
                applicable=applicable,
            ):
                stage, prim = self._define_case(
                    prim_type,
                    collision=collision,
                    merge=merge,
                    applicable=applicable,
                )
                errors = self.validator.Validate(prim)
                self._assert_errors(stage, prim, errors, expected)

    def test_shipped_usda_examples_match_the_validator(self):
        text = importlib.resources.files(physx_usd_schemas).joinpath("VALIDATION.md").read_text(encoding="utf-8")
        examples = re.findall(r"```usda\n(.*?)```", text, re.DOTALL)
        self.assertEqual(len(examples), 2)
        expected_names = (
            [],
            [validation.MESH_COLLISION_REQUIRES_COLLISION_API],
        )
        for source, expected in zip(examples, expected_names, strict=True):
            with self.subTest(expected=expected):
                stage = Usd.Stage.CreateInMemory()
                self.assertTrue(stage.GetRootLayer().ImportFromString(source))
                prim = stage.GetPrimAtPath("/Collider")
                errors = self.validator.Validate(prim)
                self.assertEqual([error.GetName() for error in errors], expected)

    def test_validation_context_matches_direct_order_and_is_read_only(self):
        stage, prim = self._define_case("Xform")
        before = stage.GetRootLayer().ExportToString()
        direct_errors = self.validator.Validate(prim)
        context = UsdValidation.ValidationContext([self.validator])
        context_errors = context.Validate(stage)
        expected_names = [
            validation.MESH_COLLISION_API_INVALID_PRIM,
            validation.MESH_COLLISION_REQUIRES_COLLISION_API,
        ]
        self.assertEqual([error.GetName() for error in direct_errors], expected_names)
        self.assertEqual([error.GetName() for error in context_errors], expected_names)
        self.assertEqual(stage.GetRootLayer().ExportToString(), before)

    def test_registration_is_idempotent(self):
        original = self.registry.GetOrLoadValidatorByName(validation.VALIDATOR_NAME)
        self.assertTrue(physx_usd_schemas.register_validators())
        self.assertTrue(physx_usd_schemas.register_validators())
        self.assertEqual(
            self.registry.GetOrLoadValidatorByName(validation.VALIDATOR_NAME),
            original,
        )

    def test_callback_exception_is_a_tool_error(self):
        _stage, prim = self._define_case("Cube", collision=True)
        with mock.patch.object(
            validation,
            "_error",
            side_effect=RuntimeError("forced callback failure"),
        ):
            with self.assertRaisesRegex(RuntimeError, "forced callback failure"):
                self.validator.Validate(prim)

    def test_wheel_metadata_and_provider_files(self):
        requirements = importlib.metadata.distribution("physx-usd-schemas").requires or []
        usd_requirements = [requirement for requirement in requirements if requirement.lower().startswith("usd-core")]
        self.assertEqual(len(usd_requirements), 1)
        compact = usd_requirements[0].lower().replace(" ", "")
        self.assertIn("usd-core>=26.5", compact)
        self.assertIn('extra=="usd"', compact)

        usdex_requirements = [
            requirement for requirement in requirements if requirement.lower().startswith("usd-exchange")
        ]
        self.assertEqual(len(usdex_requirements), 1)
        compact = usdex_requirements[0].lower().replace(" ", "")
        self.assertIn("usd-exchange>=3.0.1", compact)
        self.assertIn('extra=="usdex"', compact)

        package_files = importlib.resources.files(physx_usd_schemas)
        provider_doc = package_files.joinpath("VALIDATION.md")
        self.assertTrue(provider_doc.is_file())
        self.assertIn(validation.VALIDATOR_NAME, provider_doc.read_text())
        self.assertTrue(package_files.joinpath("validation.py").is_file())


if __name__ == "__main__":
    unittest.main()
