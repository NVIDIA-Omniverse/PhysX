# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#


from unittest.mock import patch, MagicMock
import omni.kit.viewport.utility as viewport_utility
from omni.kit.test.async_unittest import AsyncTestCase

from omni.physics.isaacsimready.scripts.capability_manager import CapabilityManager


class CheckStageCapabilitiesTests(AsyncTestCase):
    """Tests for CapabilityManager.check_stage_capabilities method."""

    def setUp(self):
        """Set up test fixtures."""
        self.manager = CapabilityManager()

    def tearDown(self):
        """Clean up after tests."""
        self.manager = None

    # =========================================================================
    # Tests for empty/null scenarios
    # =========================================================================

    def test_no_registered_schemas_returns_empty_list(self):
        """When no schemas are registered, should return empty list."""
        # Manager has no registered schemas by default
        mock_stage = MagicMock()

        result = self.manager.check_stage_capabilities(mock_stage)

        self.assertEqual(result, [])

    def test_null_stage_returns_empty_list(self):
        """When stage is None, should return empty list."""
        # Register some schemas first
        self.manager.register_schema_type_names({"PhysicsScene"})

        result = self.manager.check_stage_capabilities(None)

        self.assertEqual(result, [])

    def test_empty_schema_type_names_and_api_names_returns_empty_list(self):
        """When both schema type names and api schema names are empty, should return empty list."""
        mock_stage = MagicMock()

        result = self.manager.check_stage_capabilities(mock_stage)

        self.assertEqual(result, [])

    # =========================================================================
    # Tests for no active simulations scenario
    # =========================================================================

    @patch('omni.physics.isaacsimready.scripts.capability_manager.get_active_simulations')
    def test_no_active_simulations_returns_empty_list(self, mock_get_active_simulations):
        """When no active simulations are found, should return empty list."""
        mock_get_active_simulations.return_value = []
        self.manager.register_schema_type_names({"PhysicsScene"})

        mock_stage = MagicMock()
        with patch.object(self.manager, '_gather_stage_schemas', return_value=({"PhysicsScene"}, set())):
            result = self.manager.check_stage_capabilities(mock_stage)

        self.assertEqual(result, [])

    # =========================================================================
    # Tests for no physics schemas on stage scenario
    # =========================================================================

    @patch('omni.physics.isaacsimready.scripts.capability_manager.get_active_simulations')
    def test_no_physics_schemas_on_stage_returns_none_results(self, mock_get_active_simulations):
        """When no physics schemas are found on stage, should return None for results."""
        mock_get_active_simulations.return_value = [(1, "PhysX")]
        self.manager.register_schema_type_names({"PhysicsScene"})

        mock_stage = MagicMock()
        # Return empty sets - no schemas found on stage
        with patch.object(self.manager, '_gather_stage_schemas', return_value=(set(), set())):
            result = self.manager.check_stage_capabilities(mock_stage)

        self.assertEqual(len(result), 1)
        sim_name, sim_results, capability_available = result[0]
        self.assertEqual(sim_name, "PhysX")
        self.assertIsNone(sim_results)  # None indicates no physics schemas found
        self.assertTrue(capability_available)

    @patch('omni.physics.isaacsimready.scripts.capability_manager.get_active_simulations')
    def test_no_physics_schemas_multiple_simulations(self, mock_get_active_simulations):
        """When no physics schemas and multiple simulations, should return None for each."""
        mock_get_active_simulations.return_value = [(1, "PhysX"), (2, "IsaacSim")]
        self.manager.register_schema_type_names({"PhysicsScene"})

        mock_stage = MagicMock()
        with patch.object(self.manager, '_gather_stage_schemas', return_value=(set(), set())):
            result = self.manager.check_stage_capabilities(mock_stage)

        self.assertEqual(len(result), 2)
        for sim_name, sim_results, capability_available in result:
            self.assertIn(sim_name, ["PhysX", "IsaacSim"])
            self.assertIsNone(sim_results)
            self.assertTrue(capability_available)

    # =========================================================================
    # Tests for prim type schema checking
    # =========================================================================

    @patch('omni.physics.isaacsimready.scripts.capability_manager.check_simulation_capabilities')
    @patch('omni.physics.isaacsimready.scripts.capability_manager.get_active_simulations')
    def test_prim_types_only_all_supported(self, mock_get_active_simulations, mock_check_capabilities):
        """When only prim types are found and all are supported."""
        mock_get_active_simulations.return_value = [(1, "PhysX")]
        mock_check_capabilities.return_value = (True, [True, True])
        self.manager.register_schema_type_names({"PhysicsScene", "PhysicsJoint"})

        mock_stage = MagicMock()
        with patch.object(self.manager, '_gather_stage_schemas', return_value=({"PhysicsScene", "PhysicsJoint"}, set())):
            result = self.manager.check_stage_capabilities(mock_stage)

        self.assertEqual(len(result), 1)
        sim_name, sim_results, capability_available = result[0]
        self.assertEqual(sim_name, "PhysX")
        self.assertTrue(capability_available)
        self.assertIsNotNone(sim_results)
        self.assertEqual(len(sim_results), 2)

        # Check that all results are for prim types and are supported
        for schema_name, is_supported, schema_type in sim_results:
            self.assertIn(schema_name, ["PhysicsJoint", "PhysicsScene"])
            self.assertTrue(is_supported)
            self.assertEqual(schema_type, "Prim Type")

    @patch('omni.physics.isaacsimready.scripts.capability_manager.check_simulation_capabilities')
    @patch('omni.physics.isaacsimready.scripts.capability_manager.get_active_simulations')
    def test_prim_types_some_unsupported(self, mock_get_active_simulations, mock_check_capabilities):
        """When some prim types are not supported."""
        mock_get_active_simulations.return_value = [(1, "PhysX")]
        mock_check_capabilities.return_value = (True, [True, False])
        self.manager.register_schema_type_names({"PhysicsScene", "UnsupportedType"})

        mock_stage = MagicMock()
        with patch.object(self.manager, '_gather_stage_schemas', return_value=({"PhysicsScene", "UnsupportedType"}, set())):
            result = self.manager.check_stage_capabilities(mock_stage)

        self.assertEqual(len(result), 1)
        sim_name, sim_results, capability_available = result[0]
        self.assertTrue(capability_available)
        self.assertEqual(len(sim_results), 2)

        # Verify we have one supported and one unsupported
        supported_count = sum(1 for _, is_supported, _ in sim_results if is_supported)
        unsupported_count = sum(1 for _, is_supported, _ in sim_results if not is_supported)
        self.assertEqual(supported_count, 1)
        self.assertEqual(unsupported_count, 1)

    # =========================================================================
    # Tests for API schema checking
    # =========================================================================

    @patch('omni.physics.isaacsimready.scripts.capability_manager.check_simulation_capabilities')
    @patch('omni.physics.isaacsimready.scripts.capability_manager.get_active_simulations')
    def test_api_schemas_only_all_supported(self, mock_get_active_simulations, mock_check_capabilities):
        """When only API schemas are found and all are supported."""
        mock_get_active_simulations.return_value = [(1, "PhysX")]
        mock_check_capabilities.return_value = (True, [True, True])
        self.manager.register_api_schema_names({"PhysicsRigidBodyAPI", "PhysicsCollisionAPI"})

        mock_stage = MagicMock()
        with patch.object(self.manager, '_gather_stage_schemas', return_value=(set(), {"PhysicsRigidBodyAPI", "PhysicsCollisionAPI"})):
            result = self.manager.check_stage_capabilities(mock_stage)

        self.assertEqual(len(result), 1)
        sim_name, sim_results, capability_available = result[0]
        self.assertEqual(sim_name, "PhysX")
        self.assertTrue(capability_available)
        self.assertIsNotNone(sim_results)
        self.assertEqual(len(sim_results), 2)

        # Check that all results are for API schemas and are supported
        for schema_name, is_supported, schema_type in sim_results:
            self.assertIn(schema_name, ["PhysicsCollisionAPI", "PhysicsRigidBodyAPI"])
            self.assertTrue(is_supported)
            self.assertEqual(schema_type, "API Schema")

    @patch('omni.physics.isaacsimready.scripts.capability_manager.check_simulation_capabilities')
    @patch('omni.physics.isaacsimready.scripts.capability_manager.get_active_simulations')
    def test_api_schemas_some_unsupported(self, mock_get_active_simulations, mock_check_capabilities):
        """When some API schemas are not supported."""
        mock_get_active_simulations.return_value = [(1, "PhysX")]
        mock_check_capabilities.return_value = (True, [True, False])
        self.manager.register_api_schema_names({"PhysicsRigidBodyAPI", "UnsupportedAPI"})

        mock_stage = MagicMock()
        with patch.object(self.manager, '_gather_stage_schemas', return_value=(set(), {"PhysicsRigidBodyAPI", "UnsupportedAPI"})):
            result = self.manager.check_stage_capabilities(mock_stage)

        self.assertEqual(len(result), 1)
        sim_name, sim_results, capability_available = result[0]
        self.assertTrue(capability_available)
        self.assertEqual(len(sim_results), 2)

        # Verify we have one supported and one unsupported
        supported_count = sum(1 for _, is_supported, _ in sim_results if is_supported)
        unsupported_count = sum(1 for _, is_supported, _ in sim_results if not is_supported)
        self.assertEqual(supported_count, 1)
        self.assertEqual(unsupported_count, 1)

    # =========================================================================
    # Tests for combined prim types and API schemas
    # =========================================================================

    @patch('omni.physics.isaacsimready.scripts.capability_manager.check_simulation_capabilities')
    @patch('omni.physics.isaacsimready.scripts.capability_manager.get_active_simulations')
    def test_combined_prim_types_and_api_schemas(self, mock_get_active_simulations, mock_check_capabilities):
        """When both prim types and API schemas are found on stage."""
        mock_get_active_simulations.return_value = [(1, "PhysX")]
        # First call for prim types, second for API schemas
        mock_check_capabilities.side_effect = [
            (True, [True]),  # prim type check
            (True, [True]),  # api schema check
        ]
        self.manager.register_schema_type_names({"PhysicsScene"})
        self.manager.register_api_schema_names({"PhysicsRigidBodyAPI"})

        mock_stage = MagicMock()
        with patch.object(self.manager, '_gather_stage_schemas', return_value=({"PhysicsScene"}, {"PhysicsRigidBodyAPI"})):
            result = self.manager.check_stage_capabilities(mock_stage)

        self.assertEqual(len(result), 1)
        sim_name, sim_results, capability_available = result[0]
        self.assertEqual(sim_name, "PhysX")
        self.assertTrue(capability_available)
        self.assertEqual(len(sim_results), 2)

        # Verify we have both prim type and API schema results
        prim_type_results = [r for r in sim_results if r[2] == "Prim Type"]
        api_schema_results = [r for r in sim_results if r[2] == "API Schema"]
        self.assertEqual(len(prim_type_results), 1)
        self.assertEqual(len(api_schema_results), 1)

    @patch('omni.physics.isaacsimready.scripts.capability_manager.check_simulation_capabilities')
    @patch('omni.physics.isaacsimready.scripts.capability_manager.get_active_simulations')
    def test_combined_with_mixed_support(self, mock_get_active_simulations, mock_check_capabilities):
        """When both types are found with mixed support status."""
        mock_get_active_simulations.return_value = [(1, "PhysX")]
        mock_check_capabilities.side_effect = [
            (True, [True, False]),  # prim types: one supported, one not
            (True, [False, True]),  # api schemas: one not, one supported
        ]
        self.manager.register_schema_type_names({"PhysicsScene", "UnsupportedPrim"})
        self.manager.register_api_schema_names({"UnsupportedAPI", "PhysicsRigidBodyAPI"})

        mock_stage = MagicMock()
        with patch.object(self.manager, '_gather_stage_schemas',
                          return_value=({"PhysicsScene", "UnsupportedPrim"}, {"UnsupportedAPI", "PhysicsRigidBodyAPI"})):
            result = self.manager.check_stage_capabilities(mock_stage)

        self.assertEqual(len(result), 1)
        sim_name, sim_results, capability_available = result[0]
        self.assertTrue(capability_available)
        self.assertEqual(len(sim_results), 4)

        # Verify mixed support
        supported = [r for r in sim_results if r[1]]
        unsupported = [r for r in sim_results if not r[1]]
        self.assertEqual(len(supported), 2)
        self.assertEqual(len(unsupported), 2)

    # =========================================================================
    # Tests for multiple simulations
    # =========================================================================

    @patch('omni.physics.isaacsimready.scripts.capability_manager.check_simulation_capabilities')
    @patch('omni.physics.isaacsimready.scripts.capability_manager.get_active_simulations')
    def test_multiple_simulations_different_capabilities(self, mock_get_active_simulations, mock_check_capabilities):
        """When multiple simulations have different capability support."""
        mock_get_active_simulations.return_value = [(1, "PhysX"), (2, "IsaacSim")]
        # PhysX supports all, IsaacSim supports none
        mock_check_capabilities.side_effect = [
            (True, [True]),   # PhysX prim type check
            (True, [True]),   # IsaacSim prim type check - different result
        ]
        self.manager.register_schema_type_names({"PhysicsScene"})

        mock_stage = MagicMock()
        with patch.object(self.manager, '_gather_stage_schemas', return_value=({"PhysicsScene"}, set())):
            result = self.manager.check_stage_capabilities(mock_stage)

        self.assertEqual(len(result), 2)

        # Verify each simulation has results
        physx_result = next((r for r in result if r[0] == "PhysX"), None)
        isaac_result = next((r for r in result if r[0] == "IsaacSim"), None)
        self.assertIsNotNone(physx_result)
        self.assertIsNotNone(isaac_result)

    @patch('omni.physics.isaacsimready.scripts.capability_manager.check_simulation_capabilities')
    @patch('omni.physics.isaacsimready.scripts.capability_manager.get_active_simulations')
    def test_multiple_simulations_with_combined_schemas(self, mock_get_active_simulations, mock_check_capabilities):
        """When multiple simulations check both prim types and API schemas."""
        mock_get_active_simulations.return_value = [(1, "PhysX"), (2, "IsaacSim")]
        mock_check_capabilities.side_effect = [
            (True, [True]),   # PhysX prim type check
            (True, [True]),   # PhysX API schema check
            (True, [False]),  # IsaacSim prim type check
            (True, [True]),   # IsaacSim API schema check
        ]
        self.manager.register_schema_type_names({"PhysicsScene"})
        self.manager.register_api_schema_names({"PhysicsRigidBodyAPI"})

        mock_stage = MagicMock()
        with patch.object(self.manager, '_gather_stage_schemas', return_value=({"PhysicsScene"}, {"PhysicsRigidBodyAPI"})):
            result = self.manager.check_stage_capabilities(mock_stage)

        self.assertEqual(len(result), 2)

        # Both simulations should have 2 results each
        for sim_name, sim_results, capability_available in result:
            self.assertTrue(capability_available)
            self.assertEqual(len(sim_results), 2)

    # =========================================================================
    # Tests for capability check failure scenarios
    # =========================================================================

    @patch('omni.physics.isaacsimready.scripts.capability_manager.check_simulation_capabilities')
    @patch('omni.physics.isaacsimready.scripts.capability_manager.get_active_simulations')
    def test_prim_type_capability_check_fails(self, mock_get_active_simulations, mock_check_capabilities):
        """When prim type capability check fails (success=False)."""
        mock_get_active_simulations.return_value = [(1, "PhysX")]
        mock_check_capabilities.return_value = (False, [False])  # check failed
        self.manager.register_schema_type_names({"PhysicsScene"})

        mock_stage = MagicMock()
        with patch.object(self.manager, '_gather_stage_schemas', return_value=({"PhysicsScene"}, set())):
            result = self.manager.check_stage_capabilities(mock_stage)

        self.assertEqual(len(result), 1)
        sim_name, sim_results, capability_available = result[0]
        self.assertFalse(capability_available)
        self.assertEqual(sim_results, [])  # Results should be cleared

    @patch('omni.physics.isaacsimready.scripts.capability_manager.check_simulation_capabilities')
    @patch('omni.physics.isaacsimready.scripts.capability_manager.get_active_simulations')
    def test_api_schema_capability_check_fails(self, mock_get_active_simulations, mock_check_capabilities):
        """When API schema capability check fails (success=False)."""
        mock_get_active_simulations.return_value = [(1, "PhysX")]
        mock_check_capabilities.side_effect = [
            (True, [True]),   # prim type check succeeds
            (False, [False]), # api schema check fails
        ]
        self.manager.register_schema_type_names({"PhysicsScene"})
        self.manager.register_api_schema_names({"PhysicsRigidBodyAPI"})

        mock_stage = MagicMock()
        with patch.object(self.manager, '_gather_stage_schemas', return_value=({"PhysicsScene"}, {"PhysicsRigidBodyAPI"})):
            result = self.manager.check_stage_capabilities(mock_stage)

        self.assertEqual(len(result), 1)
        sim_name, sim_results, capability_available = result[0]
        self.assertFalse(capability_available)
        self.assertEqual(sim_results, [])  # Results should be cleared

    @patch('omni.physics.isaacsimready.scripts.capability_manager.check_simulation_capabilities')
    @patch('omni.physics.isaacsimready.scripts.capability_manager.get_active_simulations')
    def test_api_check_skipped_when_prim_type_check_fails(self, mock_get_active_simulations, mock_check_capabilities):
        """When prim type check fails, API schema check should be skipped."""
        mock_get_active_simulations.return_value = [(1, "PhysX")]
        mock_check_capabilities.return_value = (False, [False])  # prim type check fails
        self.manager.register_schema_type_names({"PhysicsScene"})
        self.manager.register_api_schema_names({"PhysicsRigidBodyAPI"})

        mock_stage = MagicMock()
        with patch.object(self.manager, '_gather_stage_schemas', return_value=({"PhysicsScene"}, {"PhysicsRigidBodyAPI"})):
            result = self.manager.check_stage_capabilities(mock_stage)

        # Should only call check_simulation_capabilities once (for prim types)
        # API check should be skipped since prim type check failed
        self.assertEqual(mock_check_capabilities.call_count, 1)

    # =========================================================================
    # Tests for edge cases
    # =========================================================================

    @patch('omni.physics.isaacsimready.scripts.capability_manager.get_active_simulations')
    def test_simulation_with_none_name_uses_unknown(self, mock_get_active_simulations):
        """When simulation name is None, should use 'Unknown' as default."""
        mock_get_active_simulations.return_value = [(1, None)]  # None simulation name
        self.manager.register_schema_type_names({"PhysicsScene"})

        mock_stage = MagicMock()
        with patch.object(self.manager, '_gather_stage_schemas', return_value=(set(), set())):
            result = self.manager.check_stage_capabilities(mock_stage)

        self.assertEqual(len(result), 1)
        sim_name, _, _ = result[0]
        self.assertEqual(sim_name, "Unknown")

    @patch('omni.physics.isaacsimready.scripts.capability_manager.get_active_simulations')
    def test_simulation_with_empty_string_name_uses_unknown(self, mock_get_active_simulations):
        """When simulation name is empty string, should use 'Unknown' as default."""
        mock_get_active_simulations.return_value = [(1, "")]  # Empty string simulation name
        self.manager.register_schema_type_names({"PhysicsScene"})

        mock_stage = MagicMock()
        with patch.object(self.manager, '_gather_stage_schemas', return_value=(set(), set())):
            result = self.manager.check_stage_capabilities(mock_stage)

        self.assertEqual(len(result), 1)
        sim_name, _, _ = result[0]
        self.assertEqual(sim_name, "Unknown")

    @patch('omni.physics.isaacsimready.scripts.capability_manager.check_simulation_capabilities')
    @patch('omni.physics.isaacsimready.scripts.capability_manager.get_active_simulations')
    def test_results_are_sorted_alphabetically(self, mock_get_active_simulations, mock_check_capabilities):
        """Schema names in results should be sorted alphabetically."""
        mock_get_active_simulations.return_value = [(1, "PhysX")]
        mock_check_capabilities.return_value = (True, [True, True, True])
        self.manager.register_schema_type_names({"Zebra", "Alpha", "Middle"})

        mock_stage = MagicMock()
        with patch.object(self.manager, '_gather_stage_schemas', return_value=({"Zebra", "Alpha", "Middle"}, set())):
            result = self.manager.check_stage_capabilities(mock_stage)

        self.assertEqual(len(result), 1)
        sim_name, sim_results, capability_available = result[0]

        # Extract schema names and verify sorted order
        schema_names = [r[0] for r in sim_results]
        self.assertEqual(schema_names, ["Alpha", "Middle", "Zebra"])

    @patch('omni.physics.isaacsimready.scripts.capability_manager.check_simulation_capabilities')
    @patch('omni.physics.isaacsimready.scripts.capability_manager.get_active_simulations')
    def test_only_registered_schemas_are_checked(self, mock_get_active_simulations, mock_check_capabilities):
        """Only schemas that are registered should be included in results."""
        mock_get_active_simulations.return_value = [(1, "PhysX")]
        mock_check_capabilities.return_value = (True, [True])
        # Only register one schema type
        self.manager.register_schema_type_names({"PhysicsScene"})

        mock_stage = MagicMock()
        # _gather_stage_schemas only returns schemas matching registered ones
        # so we only return the registered one
        with patch.object(self.manager, '_gather_stage_schemas', return_value=({"PhysicsScene"}, set())):
            result = self.manager.check_stage_capabilities(mock_stage)

        self.assertEqual(len(result), 1)
        sim_name, sim_results, capability_available = result[0]
        self.assertEqual(len(sim_results), 1)
        self.assertEqual(sim_results[0][0], "PhysicsScene")

    # =========================================================================
    # Tests for schema type categorization
    # =========================================================================

    @patch('omni.physics.isaacsimready.scripts.capability_manager.check_simulation_capabilities')
    @patch('omni.physics.isaacsimready.scripts.capability_manager.get_active_simulations')
    def test_prim_type_labeled_correctly(self, mock_get_active_simulations, mock_check_capabilities):
        """Prim types should be labeled as 'Prim Type' in results."""
        mock_get_active_simulations.return_value = [(1, "PhysX")]
        mock_check_capabilities.return_value = (True, [True])
        self.manager.register_schema_type_names({"PhysicsScene"})

        mock_stage = MagicMock()
        with patch.object(self.manager, '_gather_stage_schemas', return_value=({"PhysicsScene"}, set())):
            result = self.manager.check_stage_capabilities(mock_stage)

        sim_results = result[0][1]
        self.assertEqual(sim_results[0][2], "Prim Type")

    @patch('omni.physics.isaacsimready.scripts.capability_manager.check_simulation_capabilities')
    @patch('omni.physics.isaacsimready.scripts.capability_manager.get_active_simulations')
    def test_api_schema_labeled_correctly(self, mock_get_active_simulations, mock_check_capabilities):
        """API schemas should be labeled as 'API Schema' in results."""
        mock_get_active_simulations.return_value = [(1, "PhysX")]
        mock_check_capabilities.return_value = (True, [True])
        self.manager.register_api_schema_names({"PhysicsRigidBodyAPI"})

        mock_stage = MagicMock()
        with patch.object(self.manager, '_gather_stage_schemas', return_value=(set(), {"PhysicsRigidBodyAPI"})):
            result = self.manager.check_stage_capabilities(mock_stage)

        sim_results = result[0][1]
        self.assertEqual(sim_results[0][2], "API Schema")

    # =========================================================================
    # Tests for return type validation
    # =========================================================================

    @patch('omni.physics.isaacsimready.scripts.capability_manager.check_simulation_capabilities')
    @patch('omni.physics.isaacsimready.scripts.capability_manager.get_active_simulations')
    def test_return_type_is_list(self, mock_get_active_simulations, mock_check_capabilities):
        """Return value should always be a list."""
        mock_get_active_simulations.return_value = [(1, "PhysX")]
        mock_check_capabilities.return_value = (True, [True])
        self.manager.register_schema_type_names({"PhysicsScene"})

        mock_stage = MagicMock()
        with patch.object(self.manager, '_gather_stage_schemas', return_value=({"PhysicsScene"}, set())):
            result = self.manager.check_stage_capabilities(mock_stage)

        self.assertIsInstance(result, list)

    @patch('omni.physics.isaacsimready.scripts.capability_manager.check_simulation_capabilities')
    @patch('omni.physics.isaacsimready.scripts.capability_manager.get_active_simulations')
    def test_result_tuple_structure(self, mock_get_active_simulations, mock_check_capabilities):
        """Each result item should be a tuple of (str, list|None, bool)."""
        mock_get_active_simulations.return_value = [(1, "PhysX")]
        mock_check_capabilities.return_value = (True, [True])
        self.manager.register_schema_type_names({"PhysicsScene"})

        mock_stage = MagicMock()
        with patch.object(self.manager, '_gather_stage_schemas', return_value=({"PhysicsScene"}, set())):
            result = self.manager.check_stage_capabilities(mock_stage)

        self.assertEqual(len(result), 1)
        item = result[0]
        self.assertIsInstance(item, tuple)
        self.assertEqual(len(item), 3)

        sim_name, sim_results, capability_available = item
        self.assertIsInstance(sim_name, str)
        self.assertIsInstance(sim_results, list)
        self.assertIsInstance(capability_available, bool)

    @patch('omni.physics.isaacsimready.scripts.capability_manager.check_simulation_capabilities')
    @patch('omni.physics.isaacsimready.scripts.capability_manager.get_active_simulations')
    def test_result_item_structure(self, mock_get_active_simulations, mock_check_capabilities):
        """Each result item in sim_results should be a tuple of (str, bool, str)."""
        mock_get_active_simulations.return_value = [(1, "PhysX")]
        mock_check_capabilities.return_value = (True, [True])
        self.manager.register_schema_type_names({"PhysicsScene"})

        mock_stage = MagicMock()
        with patch.object(self.manager, '_gather_stage_schemas', return_value=({"PhysicsScene"}, set())):
            result = self.manager.check_stage_capabilities(mock_stage)

        sim_results = result[0][1]
        self.assertEqual(len(sim_results), 1)

        item = sim_results[0]
        self.assertIsInstance(item, tuple)
        self.assertEqual(len(item), 3)

        schema_name, is_supported, schema_type = item
        self.assertIsInstance(schema_name, str)
        self.assertIsInstance(is_supported, bool)
        self.assertIsInstance(schema_type, str)

    # =========================================================================
    # Tests for large number of schemas
    # =========================================================================

    @patch('omni.physics.isaacsimready.scripts.capability_manager.check_simulation_capabilities')
    @patch('omni.physics.isaacsimready.scripts.capability_manager.get_active_simulations')
    def test_many_prim_types(self, mock_get_active_simulations, mock_check_capabilities):
        """Should handle many prim types correctly."""
        mock_get_active_simulations.return_value = [(1, "PhysX")]
        num_schemas = 50
        mock_check_capabilities.return_value = (True, [True] * num_schemas)

        schema_names = {f"Schema{i}" for i in range(num_schemas)}
        self.manager.register_schema_type_names(schema_names)

        mock_stage = MagicMock()
        with patch.object(self.manager, '_gather_stage_schemas', return_value=(schema_names, set())):
            result = self.manager.check_stage_capabilities(mock_stage)

        self.assertEqual(len(result), 1)
        sim_results = result[0][1]
        self.assertEqual(len(sim_results), num_schemas)

    @patch('omni.physics.isaacsimready.scripts.capability_manager.check_simulation_capabilities')
    @patch('omni.physics.isaacsimready.scripts.capability_manager.get_active_simulations')
    def test_many_api_schemas(self, mock_get_active_simulations, mock_check_capabilities):
        """Should handle many API schemas correctly."""
        mock_get_active_simulations.return_value = [(1, "PhysX")]
        num_schemas = 50
        mock_check_capabilities.return_value = (True, [True] * num_schemas)

        schema_names = {f"API{i}" for i in range(num_schemas)}
        self.manager.register_api_schema_names(schema_names)

        mock_stage = MagicMock()
        with patch.object(self.manager, '_gather_stage_schemas', return_value=(set(), schema_names)):
            result = self.manager.check_stage_capabilities(mock_stage)

        self.assertEqual(len(result), 1)
        sim_results = result[0][1]
        self.assertEqual(len(sim_results), num_schemas)

    @patch('omni.physics.isaacsimready.scripts.capability_manager.check_simulation_capabilities')
    @patch('omni.physics.isaacsimready.scripts.capability_manager.get_active_simulations')
    def test_many_simulations(self, mock_get_active_simulations, mock_check_capabilities):
        """Should handle many active simulations correctly."""
        num_simulations = 10
        mock_get_active_simulations.return_value = [(i, f"Sim{i}") for i in range(num_simulations)]
        mock_check_capabilities.return_value = (True, [True])
        self.manager.register_schema_type_names({"PhysicsScene"})

        mock_stage = MagicMock()
        with patch.object(self.manager, '_gather_stage_schemas', return_value=({"PhysicsScene"}, set())):
            result = self.manager.check_stage_capabilities(mock_stage)

        self.assertEqual(len(result), num_simulations)
        for i, (sim_name, sim_results, capability_available) in enumerate(result):
            self.assertEqual(sim_name, f"Sim{i}")
            self.assertTrue(capability_available)
            self.assertEqual(len(sim_results), 1)
