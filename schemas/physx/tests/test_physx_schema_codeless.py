# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

import importlib.util
import shutil
import sys
import tempfile
import unittest
from pathlib import Path

from pxr import Plug, Sdf, Usd


SCHEMA_ROOT = Path(__file__).resolve().parents[1]
PHYSX_SCHEMA_SOURCE = SCHEMA_ROOT / "source" / "physxSchema"

# "Original" means the public PhysxSchema Python API generated before the schema
# was converted to codeless. These tests keep that API contract explicit while
# exercising the codeless replacement implementation.
#
# The pure-Python codeless module cannot safely inherit from upstream USD
# Boost.Python schema bases such as Usd.APISchemaBase or UsdPhysics.Joint:
# forcing that shape makes normal calls like GetPrim() fail with converter
# errors. Those upstream C++ base relationships are preserved in the generated
# header-only C++ wrappers; these Python tests cover the callable Python API and
# PhysxSchema-to-PhysxSchema inheritance that the pure-Python module can provide.
# Original PhysxSchema generated these Python classes and schema identifiers.
# Keep this list independent of the codeless Python module so tests verify the
# codeless replacement against the original API contract instead of itself.
ORIGINAL_SINGLE_APPLY_API_CLASSES = (
    ("PhysxArticulationAPI", "PhysxArticulationAPI"),
    ("PhysxCameraAPI", "PhysxCameraAPI"),
    ("PhysxCameraDroneAPI", "PhysxCameraDroneAPI"),
    ("PhysxCameraFollowAPI", "PhysxCameraFollowAPI"),
    ("PhysxCameraFollowLookAPI", "PhysxCameraFollowLookAPI"),
    ("PhysxCameraFollowVelocityAPI", "PhysxCameraFollowVelocityAPI"),
    ("PhysxCharacterControllerAPI", "PhysxCharacterControllerAPI"),
    ("PhysxCollisionAPI", "PhysxCollisionAPI"),
    ("PhysxContactReportAPI", "PhysxContactReportAPI"),
    ("PhysxConvexDecompositionCollisionAPI", "PhysxConvexDecompositionCollisionAPI"),
    ("PhysxConvexHullCollisionAPI", "PhysxConvexHullCollisionAPI"),
    ("PhysxDiffuseParticlesAPI", "PhysxDiffuseParticlesAPI"),
    ("PhysxForceAPI", "PhysxForceAPI"),
    ("PhysxJointAPI", "PhysxJointAPI"),
    ("PhysxMaterialAPI", "PhysxMaterialAPI"),
    ("PhysxMeshMergeCollisionAPI", "PhysxMeshMergeCollisionAPI"),
    ("PhysxPBDMaterialAPI", "PhysxPBDMaterialAPI"),
    ("PhysxParticleAPI", "PhysxParticleAPI"),
    ("PhysxParticleAnisotropyAPI", "PhysxParticleAnisotropyAPI"),
    ("PhysxParticleIsosurfaceAPI", "PhysxParticleIsosurfaceAPI"),
    ("PhysxParticleSamplingAPI", "PhysxParticleSamplingAPI"),
    ("PhysxParticleSetAPI", "PhysxParticleSetAPI"),
    ("PhysxParticleSmoothingAPI", "PhysxParticleSmoothingAPI"),
    ("PhysxPhysicsDistanceJointAPI", "PhysxPhysicsDistanceJointAPI"),
    ("PhysxRigidBodyAPI", "PhysxRigidBodyAPI"),
    ("PhysxSDFMeshCollisionAPI", "PhysxSDFMeshCollisionAPI"),
    ("PhysxSceneAPI", "PhysxSceneAPI"),
    ("PhysxSceneQuasistaticAPI", "PhysxSceneQuasistaticAPI"),
    ("PhysxSphereFillCollisionAPI", "PhysxSphereFillCollisionAPI"),
    ("PhysxSurfaceVelocityAPI", "PhysxSurfaceVelocityAPI"),
    ("PhysxTriangleMeshCollisionAPI", "PhysxTriangleMeshCollisionAPI"),
    ("PhysxTriangleMeshSimplificationCollisionAPI", "PhysxTriangleMeshSimplificationCollisionAPI"),
    ("PhysxTriggerAPI", "PhysxTriggerAPI"),
    ("PhysxTriggerStateAPI", "PhysxTriggerStateAPI"),
    ("PhysxVehicleAPI", "PhysxVehicleAPI"),
    ("PhysxVehicleAckermannSteeringAPI", "PhysxVehicleAckermannSteeringAPI"),
    ("PhysxVehicleAutoGearBoxAPI", "PhysxVehicleAutoGearBoxAPI"),
    ("PhysxVehicleClutchAPI", "PhysxVehicleClutchAPI"),
    ("PhysxVehicleContextAPI", "PhysxVehicleContextAPI"),
    ("PhysxVehicleControllerAPI", "PhysxVehicleControllerAPI"),
    ("PhysxVehicleDriveBasicAPI", "PhysxVehicleDriveBasicAPI"),
    ("PhysxVehicleDriveStandardAPI", "PhysxVehicleDriveStandardAPI"),
    ("PhysxVehicleEngineAPI", "PhysxVehicleEngineAPI"),
    ("PhysxVehicleGearsAPI", "PhysxVehicleGearsAPI"),
    ("PhysxVehicleMultiWheelDifferentialAPI", "PhysxVehicleMultiWheelDifferentialAPI"),
    ("PhysxVehicleSteeringAPI", "PhysxVehicleSteeringAPI"),
    ("PhysxVehicleSuspensionAPI", "PhysxVehicleSuspensionAPI"),
    ("PhysxVehicleSuspensionComplianceAPI", "PhysxVehicleSuspensionComplianceAPI"),
    ("PhysxVehicleTankControllerAPI", "PhysxVehicleTankControllerAPI"),
    ("PhysxVehicleTankDifferentialAPI", "PhysxVehicleTankDifferentialAPI"),
    ("PhysxVehicleTireAPI", "PhysxVehicleTireAPI"),
    ("PhysxVehicleWheelAPI", "PhysxVehicleWheelAPI"),
    ("PhysxVehicleWheelAttachmentAPI", "PhysxVehicleWheelAttachmentAPI"),
    ("PhysxVehicleWheelControllerAPI", "PhysxVehicleWheelControllerAPI"),
)

ORIGINAL_MULTIPLE_APPLY_API_CLASSES = (
    ("JointStateAPI", "PhysicsJointStateAPI", "state"),
    ("PhysxCookedDataAPI", "PhysxCookedDataAPI", "physxCookedData"),
    ("PhysxLimitAPI", "PhysxLimitAPI", "physxLimit"),
    ("PhysxMimicJointAPI", "PhysxMimicJointAPI", "physxMimicJoint"),
    ("PhysxTendonAttachmentAPI", "PhysxTendonAttachmentAPI", "physxTendon"),
    ("PhysxTendonAttachmentLeafAPI", "PhysxTendonAttachmentLeafAPI", "physxTendon"),
    ("PhysxTendonAttachmentRootAPI", "PhysxTendonAttachmentRootAPI", "physxTendon"),
    ("PhysxTendonAxisAPI", "PhysxTendonAxisAPI", "physxTendon"),
    ("PhysxTendonAxisRootAPI", "PhysxTendonAxisRootAPI", "physxTendon"),
    ("PhysxVehicleBrakesAPI", "PhysxVehicleBrakesAPI", "physxVehicleBrakes"),
    ("PhysxVehicleNonlinearCommandResponseAPI", "PhysxVehicleNonlinearCommandResponseAPI", "physxVehicleNCR"),
)

ORIGINAL_TYPED_SCHEMA_CLASSES = (
    ("PhysxParticleSystem", "PhysxParticleSystem"),
    ("PhysxPhysicsGearJoint", "PhysxPhysicsGearJoint"),
    ("PhysxPhysicsInstancer", "PhysxPhysicsInstancer"),
    ("PhysxPhysicsJointInstancer", "PhysxPhysicsJointInstancer"),
    ("PhysxPhysicsRackAndPinionJoint", "PhysxPhysicsRackAndPinionJoint"),
    ("PhysxVehicleTireFrictionTable", "PhysxVehicleTireFrictionTable"),
)

ORIGINAL_SCHEMA_BASE_CLASSES = (
    ("PhysxPhysicsJointInstancer", "PhysxPhysicsInstancer"),
)

ORIGINAL_CUSTOM_METHODS = (
    ("PhysxMeshMergeCollisionAPI", "GetCollisionMeshesCollectionAPI", "collisionmeshes"),
    ("PhysxSceneQuasistaticAPI", "GetQuasistaticActorsCollectionAPI", "quasistaticactors"),
)


def _load_source_physx_schema():
    module = sys.modules.get("PhysxSchema")
    if module is not None:
        return module

    spec = importlib.util.spec_from_file_location(
        "PhysxSchema",
        PHYSX_SCHEMA_SOURCE / "__init__.py",
        submodule_search_locations=[str(PHYSX_SCHEMA_SOURCE)],
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules["PhysxSchema"] = module
    spec.loader.exec_module(module)
    return module


def _all_original_class_names():
    names = [class_name for class_name, _ in ORIGINAL_SINGLE_APPLY_API_CLASSES]
    names += [class_name for class_name, _, _ in ORIGINAL_MULTIPLE_APPLY_API_CLASSES]
    names += [class_name for class_name, _ in ORIGINAL_TYPED_SCHEMA_CLASSES]
    return names


def _all_original_classes_with_kind():
    for class_name, schema_identifier in ORIGINAL_SINGLE_APPLY_API_CLASSES:
        yield class_name, schema_identifier, "single", None
    for class_name, schema_identifier, namespace_prefix in ORIGINAL_MULTIPLE_APPLY_API_CLASSES:
        yield class_name, schema_identifier, "multiple", namespace_prefix
    for class_name, schema_identifier in ORIGINAL_TYPED_SCHEMA_CLASSES:
        yield class_name, schema_identifier, "typed", None


def _camel(full_name):
    parts = full_name.split(":")
    return parts[0] + "".join(part[:1].upper() + part[1:] for part in parts[1:])


def _proper(name):
    return name[:1].upper() + name[1:]


def _original_declared_property_methods(schema_identifier):
    layer = Sdf.Layer.FindOrOpen(str(PHYSX_SCHEMA_SOURCE / "schema.usda"))
    if layer is None:
        raise RuntimeError("could not open PhysxSchema schema.usda")
    prim_spec = next(prim for prim in layer.rootPrims if prim.name == schema_identifier)
    for property_name, property_spec in prim_spec.properties.items():
        api_name = property_spec.customData.get("apiName") or _camel(property_name)
        suffix = _proper(api_name)
        if isinstance(property_spec, Sdf.RelationshipSpec):
            yield property_name, "rel", "Get" + suffix + "Rel", "Create" + suffix + "Rel"
        else:
            yield property_name, "attr", "Get" + suffix + "Attr", "Create" + suffix + "Attr"


class PhysxSchemaCodelessTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._plugin_tempdir = tempfile.TemporaryDirectory()
        plugin_root = Path(cls._plugin_tempdir.name)
        plugin_dir = plugin_root / "physxSchema"
        resource_dir = plugin_root / "resources"
        plugin_dir.mkdir()
        resource_dir.mkdir()
        shutil.copy2(PHYSX_SCHEMA_SOURCE / "plugInfo.json", plugin_dir / "plugInfo.json")
        shutil.copy2(PHYSX_SCHEMA_SOURCE / "generatedSchema.usda", resource_dir / "generatedSchema.usda")
        Plug.Registry().RegisterPlugins(str(plugin_dir))
        cls.PhysxSchema = _load_source_physx_schema()

    @classmethod
    def tearDownClass(cls):
        cls._plugin_tempdir.cleanup()

    def test_all_original_generated_classes_are_available(self):
        self.assertEqual(len(_all_original_class_names()), 71)
        for class_name in _all_original_class_names():
            with self.subTest(class_name=class_name):
                schema_cls = getattr(self.PhysxSchema, class_name, None)
                self.assertIsInstance(schema_cls, type)
                self.assertEqual(schema_cls._GetStaticTfType().typeName, "PhysxSchema" + class_name)

    def test_all_original_single_apply_apis_work_with_usd_overloads(self):
        for class_name, schema_identifier in ORIGINAL_SINGLE_APPLY_API_CLASSES:
            with self.subTest(class_name=class_name):
                schema_cls = getattr(self.PhysxSchema, class_name)
                stage = Usd.Stage.CreateInMemory()
                prim = stage.DefinePrim("/A")

                self.assertTrue(bool(schema_cls.CanApply(prim)))
                schema_api = schema_cls.Apply(prim)
                fetched_api = schema_cls.Get(stage, prim.GetPath())

                self.assertTrue(schema_api)
                self.assertEqual(schema_api.GetPrim(), prim)
                self.assertTrue(fetched_api)
                self.assertEqual(fetched_api.GetPrim(), prim)
                self.assertTrue(prim.HasAPI(schema_cls._schemaName))
                self.assertEqual(
                    schema_cls._GetStaticTfType(),
                    Usd.SchemaRegistry.GetTypeFromSchemaTypeName(schema_identifier),
                )
                self.assertIsInstance(schema_cls.GetSchemaAttributeNames(), list)
                self.assertTrue(prim.RemoveAPI(schema_cls._schemaName))
                self.assertFalse(prim.HasAPI(schema_cls._schemaName))

    def test_all_original_multiple_apply_apis_work_with_usd_overloads_and_helpers(self):
        instance_name = "inst"
        for class_name, schema_identifier, namespace_prefix in ORIGINAL_MULTIPLE_APPLY_API_CLASSES:
            with self.subTest(class_name=class_name):
                schema_cls = getattr(self.PhysxSchema, class_name)
                path_helper = getattr(schema_cls, "Is" + schema_identifier + "Path")
                stage = Usd.Stage.CreateInMemory()
                prim = stage.DefinePrim("/A")

                self.assertTrue(bool(schema_cls.CanApply(prim, instance_name)))
                schema_api = schema_cls.Apply(prim, instance_name)
                fetched_api = schema_cls.Get(prim, instance_name)
                path_api = schema_cls.Get(stage, Sdf.Path(f"/A.{namespace_prefix}:{instance_name}"))
                all_apis = schema_cls.GetAll(prim)
                template_names = schema_cls.GetSchemaAttributeNames(False)
                instance_names = schema_cls.GetSchemaAttributeNames(False, instance_name)

                self.assertTrue(schema_api)
                self.assertEqual(schema_api.GetName(), instance_name)
                self.assertTrue(fetched_api)
                self.assertTrue(path_api)
                self.assertEqual(path_api.GetName(), instance_name)
                self.assertEqual([api.GetName() for api in all_apis], [instance_name])
                self.assertTrue(prim.HasAPI(schema_cls._schemaName, instance_name))
                self.assertTrue(path_helper(Sdf.Path(f"/A.{namespace_prefix}:{instance_name}")))
                self.assertEqual(len(template_names), len(instance_names))
                self.assertGreater(len(template_names), 0)
                self.assertTrue(all("__INSTANCE_NAME__" in name for name in template_names))
                self.assertTrue(all("__INSTANCE_NAME__" not in name for name in instance_names))
                self.assertFalse(path_helper(Sdf.Path("/A." + instance_names[0])))
                self.assertFalse(schema_cls.Get(stage, Sdf.Path("/A." + instance_names[0])))
                self.assertEqual(
                    schema_cls._GetStaticTfType(),
                    Usd.SchemaRegistry.GetTypeFromSchemaTypeName(schema_identifier),
                )
                self.assertTrue(prim.RemoveAPI(schema_cls._schemaName, instance_name))
                self.assertFalse(prim.HasAPI(schema_cls._schemaName, instance_name))

    def test_all_original_typed_schemas_work_with_define_and_get(self):
        for class_name, schema_identifier in ORIGINAL_TYPED_SCHEMA_CLASSES:
            with self.subTest(class_name=class_name):
                schema_cls = getattr(self.PhysxSchema, class_name)
                stage = Usd.Stage.CreateInMemory()
                path = Sdf.Path("/" + class_name)

                schema_obj = schema_cls.Define(stage, path)
                fetched_obj = schema_cls.Get(stage, path)

                self.assertTrue(schema_obj)
                self.assertEqual(schema_obj.GetPrim().GetTypeName(), schema_identifier)
                self.assertTrue(fetched_obj)
                self.assertEqual(fetched_obj.GetPrim(), schema_obj.GetPrim())
                self.assertEqual(
                    schema_cls._GetStaticTfType(),
                    Usd.SchemaRegistry.GetTypeFromSchemaTypeName(schema_identifier),
                )
                self.assertIsInstance(schema_cls.GetSchemaAttributeNames(), list)

    def test_get_and_bool_reflect_applicability(self):
        # A schema object is truthy only when the prim actually has the API / is
        # the type -- matching the compiled bindings (UsdSchemaBase::operator bool
        # -> _IsCompatible). Get() on an unapplied or mismatched prim is falsy.
        for class_name, _schema_identifier in ORIGINAL_SINGLE_APPLY_API_CLASSES:
            with self.subTest(kind="single", class_name=class_name):
                schema_cls = getattr(self.PhysxSchema, class_name)
                stage = Usd.Stage.CreateInMemory()
                prim = stage.DefinePrim("/A")
                self.assertFalse(schema_cls.Get(stage, prim.GetPath()))
                schema_cls.Apply(prim)
                self.assertTrue(schema_cls.Get(stage, prim.GetPath()))
                prim.RemoveAPI(schema_cls._schemaName)
                self.assertFalse(schema_cls.Get(stage, prim.GetPath()))

        for class_name, _schema_identifier, _namespace_prefix in ORIGINAL_MULTIPLE_APPLY_API_CLASSES:
            with self.subTest(kind="multiple", class_name=class_name):
                schema_cls = getattr(self.PhysxSchema, class_name)
                stage = Usd.Stage.CreateInMemory()
                prim = stage.DefinePrim("/A")
                schema_cls.Apply(prim, "inst")
                self.assertTrue(schema_cls.Get(prim, "inst"))
                self.assertFalse(schema_cls.Get(prim, "other"))  # instance not applied

        for class_name, _schema_identifier in ORIGINAL_TYPED_SCHEMA_CLASSES:
            with self.subTest(kind="typed", class_name=class_name):
                schema_cls = getattr(self.PhysxSchema, class_name)
                stage = Usd.Stage.CreateInMemory()
                cube = stage.DefinePrim("/Cube", "Cube")
                self.assertFalse(schema_cls.Get(stage, cube.GetPath()))  # wrong type
                untyped = stage.DefinePrim("/Untyped")
                self.assertFalse(schema_cls.Get(stage, untyped.GetPath()))

    def test_create_attr_writes_sparsely_like_compiled_bindings(self):
        # writeSparsely must not author a value equal to the schema fallback, even
        # when the default is a double and the attribute is float32 -- matching
        # UsdSchemaBase::_CreateAttr, which coerces the default to the attr type
        # before comparing. sleepThreshold's fallback is float 0.00005.
        schema_cls = self.PhysxSchema.PhysxRigidBodyAPI
        stage = Usd.Stage.CreateInMemory()
        api = schema_cls.Apply(stage.DefinePrim("/A"))
        fallback = api.GetSleepThresholdAttr().Get()

        attr = api.CreateSleepThresholdAttr(0.00005, writeSparsely=True)
        self.assertFalse(attr.HasAuthoredValue())
        self.assertEqual(attr.Get(), fallback)

        attr = api.CreateSleepThresholdAttr(0.25, writeSparsely=True)
        self.assertTrue(attr.HasAuthoredValue())

        attr.Clear()
        self.assertFalse(attr.HasAuthoredValue())
        attr = api.CreateSleepThresholdAttr(0.00005, writeSparsely=False)
        self.assertTrue(attr.HasAuthoredValue())

    def _make_schema_object(self, schema_cls, kind, class_name, instance_name="inst"):
        stage = Usd.Stage.CreateInMemory()
        if kind == "typed":
            path = Sdf.Path("/" + class_name)
            return stage, schema_cls.Define(stage, path)

        prim = stage.DefinePrim("/A")
        if kind == "multiple":
            return stage, schema_cls.Apply(prim, instance_name)
        return stage, schema_cls.Apply(prim)

    def test_original_schema_base_classes_are_preserved(self):
        self.assertEqual(len(ORIGINAL_SCHEMA_BASE_CLASSES), 1)
        for derived_class_name, base_class_name in ORIGINAL_SCHEMA_BASE_CLASSES:
            with self.subTest(class_name=derived_class_name):
                derived_cls = getattr(self.PhysxSchema, derived_class_name)
                base_cls = getattr(self.PhysxSchema, base_class_name)
                stage = Usd.Stage.CreateInMemory()
                derived_obj = derived_cls.Define(stage, Sdf.Path("/" + derived_class_name))

                self.assertTrue(issubclass(derived_cls, base_cls))
                self.assertIsInstance(derived_obj, base_cls)
                for _, _, getter_name, creator_name in _original_declared_property_methods(base_cls._schemaName):
                    self.assertTrue(hasattr(derived_obj, getter_name), getter_name)
                    self.assertTrue(hasattr(derived_obj, creator_name), creator_name)

    def test_original_constructors_and_can_apply_result_shape_are_preserved(self):
        instance_name = "inst"
        for class_name, schema_identifier, kind, _ in _all_original_classes_with_kind():
            with self.subTest(class_name=class_name):
                schema_cls = getattr(self.PhysxSchema, class_name)
                stage = Usd.Stage.CreateInMemory()

                if kind == "typed":
                    schema_obj = schema_cls.Define(stage, Sdf.Path("/" + class_name))
                    from_prim = schema_cls(schema_obj.GetPrim())
                    from_schema = schema_cls(schema_obj)
                else:
                    prim = stage.DefinePrim("/A")
                    if kind == "multiple":
                        can_apply = schema_cls.CanApply(prim, instance_name)
                        schema_obj = schema_cls.Apply(prim, instance_name)
                        from_prim = schema_cls(prim, instance_name)
                        from_schema = schema_cls(schema_obj, instance_name)
                    else:
                        can_apply = schema_cls.CanApply(prim)
                        schema_obj = schema_cls.Apply(prim)
                        from_prim = schema_cls(prim)
                        from_schema = schema_cls(schema_obj)

                    self.assertTrue(bool(can_apply))
                    self.assertTrue(hasattr(can_apply, "whyNot"))
                    self.assertEqual(can_apply.whyNot, "")

                self.assertTrue(from_prim)
                self.assertTrue(from_schema)
                self.assertEqual(from_prim.GetPrim(), schema_obj.GetPrim())
                self.assertEqual(from_schema.GetPrim(), schema_obj.GetPrim())
                self.assertEqual(schema_obj.GetPrim().GetTypeName() or schema_identifier, schema_identifier)

    def test_original_declared_property_accessors_are_preserved(self):
        total_properties = 0
        for class_name, schema_identifier, kind, _ in _all_original_classes_with_kind():
            with self.subTest(class_name=class_name):
                schema_cls = getattr(self.PhysxSchema, class_name)
                _, schema_obj = self._make_schema_object(schema_cls, kind, class_name)
                expected_methods = list(_original_declared_property_methods(schema_identifier))
                total_properties += len(expected_methods)

                for property_name, property_kind, getter_name, creator_name in expected_methods:
                    with self.subTest(class_name=class_name, property_name=property_name):
                        self.assertTrue(hasattr(schema_cls, getter_name), getter_name)
                        self.assertTrue(hasattr(schema_cls, creator_name), creator_name)
                        created_property = getattr(schema_obj, creator_name)()
                        fetched_property = getattr(schema_obj, getter_name)()

                        self.assertTrue(created_property)
                        self.assertTrue(fetched_property)
                        self.assertEqual(created_property.GetPrim(), schema_obj.GetPrim())
                        self.assertEqual(fetched_property.GetPrim(), schema_obj.GetPrim())
                        expected_type_name = "Attribute" if property_kind == "attr" else "Relationship"
                        self.assertEqual(type(created_property).__name__, expected_type_name)
                        self.assertEqual(type(fetched_property).__name__, expected_type_name)

        self.assertEqual(total_properties, 412)

    def test_original_custom_methods_are_available(self):
        stage = Usd.Stage.CreateInMemory()
        prim = stage.DefinePrim("/A")
        for class_name, method_name, collection_name in ORIGINAL_CUSTOM_METHODS:
            with self.subTest(class_name=class_name):
                schema_api = getattr(self.PhysxSchema, class_name).Apply(prim)
                collection_api = getattr(schema_api, method_name)()

                self.assertTrue(collection_api)
                self.assertEqual(collection_api.GetName(), collection_name)


if __name__ == "__main__":
    unittest.main()
