# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#


from unittest.mock import patch, MagicMock
from omni.kit.test.async_unittest import AsyncTestCase

from omni.physics.isaacsimready import get_capability_manager, get_variant_switcher


class PhysXRegistryTests(AsyncTestCase):
    """Tests for PhysX extension capability and variant registration."""

    def setUp(self):
        """Set up test fixtures."""
        # Get the singleton instances
        self._capability_manager = get_capability_manager()
        self._variant_switcher = get_variant_switcher()

        # Clear any previous registrations
        self._capability_manager._schema_type_names.clear()
        self._capability_manager._api_schema_names.clear()
        self._variant_switcher.clear_mappings()

    def tearDown(self):
        """Clean up after tests."""
        # Clear registrations after tests
        self._capability_manager._schema_type_names.clear()
        self._capability_manager._api_schema_names.clear()
        self._variant_switcher.clear_mappings()

    # =========================================================================
    # Tests for PhysX schema registration
    # =========================================================================

    @patch('omni.physics.physxui.physx_schemas.get_physx_schema_names')
    def test_physx_schemas_are_registered(self, mock_get_schemas):
        """Should register PhysX prim type and API schema names."""
        # Mock schema names returned by get_physx_schema_names
        mock_prim_types = "PhysxPhysicsInstancer"
        mock_api_schemas = "PhysxTriggerAPI"
        mock_get_schemas.return_value = (mock_prim_types, mock_api_schemas)

        # Simulate extension startup
        from omni.physics.physxui import PhysicsPhysxUIExtension
        extension = PhysicsPhysxUIExtension()
        extension.on_startup("test_ext_id")

        # Verify prim types are registered
        registered_prim_types = self._capability_manager.get_registered_schema_type_names()
        self.assertIn(mock_prim_types, registered_prim_types)

        # Verify API schemas are registered
        registered_api_schemas = self._capability_manager.get_registered_api_schema_names()
        self.assertIn(mock_api_schemas, registered_api_schemas)

        # Clean up
        extension.on_shutdown()

    @patch('omni.physics.physxui.physx_schemas.get_physx_schema_names')
    def test_physx_schemas_persist_after_registration(self, mock_get_schemas):
        """Registered schemas should persist in capability manager."""
        mock_prim_types = {"PhysxPhysicsRackAndPinionJoint"}
        mock_api_schemas = {"PhysxTriggerAPI"}
        mock_get_schemas.return_value = (mock_prim_types, mock_api_schemas)

        from omni.physics.physxui import PhysicsPhysxUIExtension
        extension = PhysicsPhysxUIExtension()
        extension.on_startup("test_ext_id")

        # Schemas should still be retrievable after registration
        self.assertIn("PhysxPhysicsRackAndPinionJoint", self._capability_manager.get_registered_schema_type_names())
        self.assertIn("PhysxTriggerAPI", self._capability_manager.get_registered_api_schema_names())

        extension.on_shutdown()

    # =========================================================================
    # Tests for PhysX variant registration
    # =========================================================================

    @patch('omni.physics.physxui.physx_schemas.get_physx_schema_names')
    def test_physx_variant_is_registered(self, mock_get_schemas):
        """Should register PhysX simulator variant mapping."""
        mock_get_schemas.return_value = (set(), set())

        from omni.physics.physxui import PhysicsPhysxUIExtension
        extension = PhysicsPhysxUIExtension()
        extension.on_startup("test_ext_id")

        # Verify PhysX variant mapping is registered
        variant_name = self._variant_switcher.get_variant_for_simulator("PhysX")
        self.assertEqual(variant_name, "physx")

        extension.on_shutdown()

    @patch('omni.physics.physxui.physx_schemas.get_physx_schema_names')
    def test_physx_variant_mapping_persists(self, mock_get_schemas):
        """PhysX variant mapping should persist after registration."""
        mock_get_schemas.return_value = (set(), set())

        from omni.physics.physxui import PhysicsPhysxUIExtension
        extension = PhysicsPhysxUIExtension()
        extension.on_startup("test_ext_id")

        # Variant mapping should be retrievable through all mappings
        all_mappings = self._variant_switcher.get_all_mappings()
        self.assertIn("PhysX", all_mappings)
        self.assertEqual(all_mappings["PhysX"], "physx")

        extension.on_shutdown()

    @patch('omni.physics.physxui.physx_schemas.get_physx_schema_names')
    def test_physx_variant_in_all_mappings(self, mock_get_schemas):
        """PhysX variant should appear in all mappings list."""
        mock_get_schemas.return_value = (set(), set())

        from omni.physics.physxui import PhysicsPhysxUIExtension
        extension = PhysicsPhysxUIExtension()
        extension.on_startup("test_ext_id")

        # Get all mappings and verify PhysX is present
        all_mappings = self._variant_switcher.get_all_mappings()
        self.assertEqual(len(all_mappings), 1)
        self.assertIn("PhysX", all_mappings)
        self.assertEqual(all_mappings["PhysX"], "physx")

        extension.on_shutdown()

    # =========================================================================
    # Tests for combined registration
    # =========================================================================

    @patch('omni.physics.physxui.physx_schemas.get_physx_schema_names')
    def test_both_capabilities_and_variant_registered(self, mock_get_schemas):
        """Should register both capabilities and variant mapping together."""
        mock_prim_types = {"PhysxPhysicsRackAndPinionJoint"}
        mock_api_schemas = {"PhysxTriggerAPI"}
        mock_get_schemas.return_value = (mock_prim_types, mock_api_schemas)

        from omni.physics.physxui import PhysicsPhysxUIExtension
        extension = PhysicsPhysxUIExtension()
        extension.on_startup("test_ext_id")

        # Verify both registrations succeeded
        self.assertIn("PhysxPhysicsRackAndPinionJoint", self._capability_manager.get_registered_schema_type_names())
        self.assertIn("PhysxTriggerAPI", self._capability_manager.get_registered_api_schema_names())
        self.assertEqual(self._variant_switcher.get_variant_for_simulator("PhysX"), "physx")

        extension.on_shutdown()

    # =========================================================================
    # Tests for schema filtering (not_capabilities list)
    # =========================================================================

    @patch('omni.physics.physxui.physx_schemas._get_physx_schema_names')
    def test_filtered_schemas_not_registered(self, mock_get_schemas):
        """Schemas in not_capabilities list should be filtered out."""
        # Include schemas that should be filtered
        all_prim_types = {"PhysicsScene", "PhysxLimitAPI"}
        all_api_schemas = {"PhysxRigidBodyAPI", "PhysxArticulationAPI", "PhysxSceneAPI"}
        mock_get_schemas.return_value = (all_prim_types, all_api_schemas)

        from omni.physics.physxui.physx_schemas import get_physx_schema_names
        prim_types, api_schemas = get_physx_schema_names()

        # Verify filtered schemas are not present
        self.assertNotIn("PhysxLimitAPI", prim_types)
        self.assertNotIn("PhysxSceneAPI", api_schemas)
        self.assertNotIn("PhysxArticulationAPI", api_schemas)
        self.assertNotIn("PhysxRigidBodyAPI", api_schemas)

        # Verify non-filtered schemas are present
        self.assertIn("PhysicsScene", prim_types)


class PhysXSchemaRetrievalTests(AsyncTestCase):
    """Tests for PhysX schema retrieval functionality."""

    def setUp(self):
        """Set up test fixtures."""
        pass

    def tearDown(self):
        """Clean up after tests."""
        pass

    # =========================================================================
    # Tests for get_physx_schema_names function
    # =========================================================================

    @patch('omni.physics.physxui.physx_schemas._get_physx_schema_names')
    def test_get_physx_schema_names_returns_tuple(self, mock_get_schemas):
        """get_physx_schema_names should return tuple of (prim_types, api_schemas)."""
        mock_get_schemas.return_value = ({"PhysicsScene"}, {"PhysxRigidBodyAPI"})

        from omni.physics.physxui.physx_schemas import get_physx_schema_names
        result = get_physx_schema_names()

        self.assertIsInstance(result, tuple)
        self.assertEqual(len(result), 2)
        prim_types, api_schemas = result
        self.assertIsInstance(prim_types, set)
        self.assertIsInstance(api_schemas, set)

    @patch('omni.physics.physxui.physx_schemas._get_physx_schema_names')
    def test_get_physx_schema_names_handles_empty_results(self, mock_get_schemas):
        """Should handle empty schema results correctly."""
        mock_get_schemas.return_value = (set(), set())

        from omni.physics.physxui.physx_schemas import get_physx_schema_names
        prim_types, api_schemas = get_physx_schema_names()

        self.assertEqual(prim_types, set())
        self.assertEqual(api_schemas, set())

    @patch('omni.physics.physxui.physx_schemas._get_physx_schema_names')
    def test_get_physx_schema_names_queries_multiple_plugins(self, mock_get_schemas):
        """Should query both physxSchema and physxSchemaAddition plugins."""
        mock_get_schemas.return_value = (set(), set())

        from omni.physics.physxui.physx_schemas import get_physx_schema_names
        get_physx_schema_names()

        # Verify _get_physx_schema_names was called twice (once for each plugin)
        self.assertEqual(mock_get_schemas.call_count, 2)
        mock_get_schemas.assert_any_call("physxSchema")
        mock_get_schemas.assert_any_call("physxSchemaAddition")

    @patch('omni.physics.physxui.physx_schemas._get_physx_schema_names')
    def test_get_physx_schema_names_combines_results(self, mock_get_schemas):
        """Should combine results from both plugin queries."""
        # First call returns one set of schemas, second call returns another
        mock_get_schemas.side_effect = [
            ({"PhysicsScene"}, {"PhysxTriggerAPI"}),
            ({"PhysicsJoint"}, {"PhysxMeshMergeCollisionAPI"}),
        ]

        from omni.physics.physxui.physx_schemas import get_physx_schema_names
        prim_types, api_schemas = get_physx_schema_names()

        # Results should be combined
        self.assertIn("PhysicsScene", prim_types)
        self.assertIn("PhysicsJoint", prim_types)
        self.assertIn("PhysxTriggerAPI", api_schemas)
        self.assertIn("PhysxMeshMergeCollisionAPI", api_schemas)

    @patch('omni.physics.physxui.physx_schemas._get_physx_schema_names')
    def test_not_capabilities_list_filters_correctly(self, mock_get_schemas):
        """not_capabilities list should filter out specified schemas."""
        # Return schemas including ones that should be filtered
        mock_get_schemas.side_effect = [
            (set(), {"PhysxLimitAPI", "PhysxSceneAPI", "PhysxKeepThisAPI"}),
            (set(), set()),
        ]

        from omni.physics.physxui.physx_schemas import get_physx_schema_names
        prim_types, api_schemas = get_physx_schema_names()

        # Filtered schemas should not be present
        self.assertNotIn("PhysxLimitAPI", api_schemas)
        self.assertNotIn("PhysxSceneAPI", api_schemas)

        # Non-filtered schema should be present
        self.assertIn("PhysxKeepThisAPI", api_schemas)
