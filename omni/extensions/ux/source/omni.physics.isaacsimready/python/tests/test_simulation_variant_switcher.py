# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

from unittest.mock import patch, MagicMock, PropertyMock
from omni.kit.test.async_unittest import AsyncTestCase

from omni.physics.isaacsimready.scripts.variant_switcher import VariantSwitcher


class VariantSwitcherMappingTests(AsyncTestCase):
    """Tests for VariantSwitcher simulator-to-variant mapping operations."""

    def setUp(self):
        """Set up test fixtures."""
        self.switcher = VariantSwitcher()

    def tearDown(self):
        """Clean up after tests."""
        self.switcher = None

    # =========================================================================
    # Tests for register_simulator_variant
    # =========================================================================

    def test_register_single_mapping(self):
        """Registering a single simulator-variant mapping should work."""
        self.switcher.register_simulator_variant("PhysX", "physx_variant")

        result = self.switcher.get_variant_for_simulator("PhysX")
        self.assertEqual(result, "physx_variant")

    def test_register_multiple_mappings(self):
        """Registering multiple simulator-variant mappings should work."""
        self.switcher.register_simulator_variant("PhysX", "physx_variant")
        self.switcher.register_simulator_variant("Newton", "newton_variant")
        self.switcher.register_simulator_variant("MuJoCo", "mujoco_variant")

        self.assertEqual(self.switcher.get_variant_for_simulator("PhysX"), "physx_variant")
        self.assertEqual(self.switcher.get_variant_for_simulator("Newton"), "newton_variant")
        self.assertEqual(self.switcher.get_variant_for_simulator("MuJoCo"), "mujoco_variant")

    def test_register_overwrites_existing_mapping(self):
        """Registering with same simulator name should overwrite existing mapping."""
        self.switcher.register_simulator_variant("PhysX", "old_variant")
        self.switcher.register_simulator_variant("PhysX", "new_variant")

        result = self.switcher.get_variant_for_simulator("PhysX")
        self.assertEqual(result, "new_variant")

    # =========================================================================
    # Tests for unregister_simulator_variant
    # =========================================================================

    def test_unregister_existing_mapping(self):
        """Unregistering an existing mapping should remove it."""
        self.switcher.register_simulator_variant("PhysX", "physx_variant")
        self.switcher.unregister_simulator_variant("PhysX")

        result = self.switcher.get_variant_for_simulator("PhysX")
        self.assertIsNone(result)

    def test_unregister_nonexistent_mapping_does_not_raise(self):
        """Unregistering a non-existent mapping should not raise an error."""
        # Should not raise
        self.switcher.unregister_simulator_variant("NonExistent")

    def test_unregister_does_not_affect_other_mappings(self):
        """Unregistering one mapping should not affect others."""
        self.switcher.register_simulator_variant("PhysX", "physx_variant")
        self.switcher.register_simulator_variant("Newton", "newton_variant")
        self.switcher.unregister_simulator_variant("PhysX")

        self.assertIsNone(self.switcher.get_variant_for_simulator("PhysX"))
        self.assertEqual(self.switcher.get_variant_for_simulator("Newton"), "newton_variant")

    # =========================================================================
    # Tests for get_variant_for_simulator
    # =========================================================================

    def test_get_variant_for_nonexistent_simulator_returns_none(self):
        """Getting variant for non-existent simulator should return None."""
        result = self.switcher.get_variant_for_simulator("NonExistent")
        self.assertIsNone(result)

    def test_get_variant_for_empty_string_simulator(self):
        """Getting variant for empty string simulator should return None if not registered."""
        result = self.switcher.get_variant_for_simulator("")
        self.assertIsNone(result)

    def test_get_variant_for_empty_string_simulator_when_registered(self):
        """Empty string can be used as a simulator name if registered."""
        self.switcher.register_simulator_variant("", "empty_variant")
        result = self.switcher.get_variant_for_simulator("")
        self.assertEqual(result, "empty_variant")

    # =========================================================================
    # Tests for get_all_mappings
    # =========================================================================

    def test_get_all_mappings_returns_empty_dict_initially(self):
        """get_all_mappings should return empty dict when no mappings registered."""
        result = self.switcher.get_all_mappings()
        self.assertEqual(result, {})

    def test_get_all_mappings_returns_copy(self):
        """get_all_mappings should return a copy, not the internal dict."""
        self.switcher.register_simulator_variant("PhysX", "physx_variant")
        mappings = self.switcher.get_all_mappings()

        # Modifying the returned dict should not affect internal state
        mappings["PhysX"] = "modified"
        self.assertEqual(self.switcher.get_variant_for_simulator("PhysX"), "physx_variant")

    def test_get_all_mappings_contains_all_registered(self):
        """get_all_mappings should contain all registered mappings."""
        self.switcher.register_simulator_variant("PhysX", "physx_variant")
        self.switcher.register_simulator_variant("Newton", "newton_variant")

        result = self.switcher.get_all_mappings()
        self.assertEqual(result, {"PhysX": "physx_variant", "Newton": "newton_variant"})

    # =========================================================================
    # Tests for clear_mappings
    # =========================================================================

    def test_clear_mappings_removes_all(self):
        """clear_mappings should remove all registered mappings."""
        self.switcher.register_simulator_variant("PhysX", "physx_variant")
        self.switcher.register_simulator_variant("Newton", "newton_variant")
        self.switcher.clear_mappings()

        self.assertEqual(self.switcher.get_all_mappings(), {})

    def test_clear_mappings_on_empty_does_not_raise(self):
        """clear_mappings on empty switcher should not raise."""
        # Should not raise
        self.switcher.clear_mappings()
        self.assertEqual(self.switcher.get_all_mappings(), {})


class SwitchVariantsForSimulationTests(AsyncTestCase):
    """Tests for VariantSwitcher.switch_variants_for_simulation method."""

    def setUp(self):
        """Set up test fixtures."""
        self.switcher = VariantSwitcher()
        # Create a mock variant manager
        self.mock_variant_manager = MagicMock()
        self.switcher._variant_manager = self.mock_variant_manager

    def tearDown(self):
        """Clean up after tests."""
        self.switcher = None

    # =========================================================================
    # Tests for null/empty scenarios
    # =========================================================================

    def test_returns_zero_when_stage_is_none(self):
        """Should return 0 when stage is None."""
        self.switcher.register_simulator_variant("PhysX", "physx_variant")

        count, paths = self.switcher.switch_variants_for_simulation(None, "PhysX", "Physics")
        self.assertEqual(count, 0)
        self.assertEqual(paths, [])

    def test_returns_zero_when_variant_manager_is_none(self):
        """Should return 0 when variant manager is None."""
        self.switcher._variant_manager = None
        self.switcher.register_simulator_variant("PhysX", "physx_variant")
        mock_stage = MagicMock()

        count, paths = self.switcher.switch_variants_for_simulation(mock_stage, "PhysX", "Physics")
        self.assertEqual(count, 0)
        self.assertEqual(paths, [])

    def test_returns_zero_when_no_variant_mapping_exists(self):
        """Should return 0 when no variant mapping exists for simulation."""
        mock_stage = MagicMock()

        count, paths = self.switcher.switch_variants_for_simulation(mock_stage, "UnknownSim", "Physics")
        self.assertEqual(count, 0)
        self.assertEqual(paths, [])

    def test_returns_zero_when_no_prims_with_variant(self):
        """Should return 0 when no prims have the specified variant set."""
        self.switcher.register_simulator_variant("PhysX", "physx_variant")
        self.mock_variant_manager.get_prims_with_variant.return_value = []
        mock_stage = MagicMock()

        count, paths = self.switcher.switch_variants_for_simulation(mock_stage, "PhysX", "Physics")
        self.assertEqual(count, 0)
        self.assertEqual(paths, [])

    # =========================================================================
    # Tests for variant switching
    # =========================================================================

    def test_switches_variant_on_single_prim(self):
        """Should switch variant on a single prim."""
        self.switcher.register_simulator_variant("PhysX", "physx_variant")
        self.mock_variant_manager.get_prims_with_variant.return_value = ["/World/Prim1"]

        # Create mock stage and prim
        mock_stage = MagicMock()
        mock_prim = MagicMock()
        mock_prim.IsValid.return_value = True
        mock_variant_sets = MagicMock()
        mock_variant_sets.HasVariantSet.return_value = True
        mock_variant_set = MagicMock()
        mock_variant_set.GetVariantNames.return_value = ["physx_variant", "newton_variant"]
        mock_variant_set.GetVariantSelection.return_value = "newton_variant"
        mock_variant_sets.GetVariantSet.return_value = mock_variant_set
        mock_prim.GetVariantSets.return_value = mock_variant_sets
        mock_stage.GetPrimAtPath.return_value = mock_prim

        count, paths = self.switcher.switch_variants_for_simulation(mock_stage, "PhysX", "Physics")

        self.assertEqual(count, 1)
        self.assertEqual(paths, ["/World/Prim1"])
        mock_variant_set.SetVariantSelection.assert_called_once_with("physx_variant")

    def test_does_not_switch_when_already_selected(self):
        """Should not switch if variant is already selected."""
        self.switcher.register_simulator_variant("PhysX", "physx_variant")
        self.mock_variant_manager.get_prims_with_variant.return_value = ["/World/Prim1"]

        mock_stage = MagicMock()
        mock_prim = MagicMock()
        mock_prim.IsValid.return_value = True
        mock_variant_sets = MagicMock()
        mock_variant_sets.HasVariantSet.return_value = True
        mock_variant_set = MagicMock()
        mock_variant_set.GetVariantNames.return_value = ["physx_variant", "newton_variant"]
        mock_variant_set.GetVariantSelection.return_value = "physx_variant"  # Already selected
        mock_variant_sets.GetVariantSet.return_value = mock_variant_set
        mock_prim.GetVariantSets.return_value = mock_variant_sets
        mock_stage.GetPrimAtPath.return_value = mock_prim

        count, paths = self.switcher.switch_variants_for_simulation(mock_stage, "PhysX", "Physics")

        self.assertEqual(count, 0)
        self.assertEqual(paths, [])
        mock_variant_set.SetVariantSelection.assert_not_called()

    def test_skips_invalid_prim(self):
        """Should skip prims that are not valid."""
        self.switcher.register_simulator_variant("PhysX", "physx_variant")
        self.mock_variant_manager.get_prims_with_variant.return_value = ["/World/InvalidPrim"]

        mock_stage = MagicMock()
        mock_prim = MagicMock()
        mock_prim.IsValid.return_value = False
        mock_stage.GetPrimAtPath.return_value = mock_prim

        count, paths = self.switcher.switch_variants_for_simulation(mock_stage, "PhysX", "Physics")
        self.assertEqual(count, 0)
        self.assertEqual(paths, [])

    def test_skips_prim_without_variant_set(self):
        """Should skip prims that don't have the specified variant set."""
        self.switcher.register_simulator_variant("PhysX", "physx_variant")
        self.mock_variant_manager.get_prims_with_variant.return_value = ["/World/Prim1"]

        mock_stage = MagicMock()
        mock_prim = MagicMock()
        mock_prim.IsValid.return_value = True
        mock_variant_sets = MagicMock()
        mock_variant_sets.HasVariantSet.return_value = False
        mock_prim.GetVariantSets.return_value = mock_variant_sets
        mock_stage.GetPrimAtPath.return_value = mock_prim

        count, paths = self.switcher.switch_variants_for_simulation(mock_stage, "PhysX", "Physics")
        self.assertEqual(count, 0)
        self.assertEqual(paths, [])

    def test_skips_prim_when_variant_not_available(self):
        """Should skip prim if the target variant is not available in the variant set."""
        self.switcher.register_simulator_variant("PhysX", "physx_variant")
        self.mock_variant_manager.get_prims_with_variant.return_value = ["/World/Prim1"]

        mock_stage = MagicMock()
        mock_prim = MagicMock()
        mock_prim.IsValid.return_value = True
        mock_variant_sets = MagicMock()
        mock_variant_sets.HasVariantSet.return_value = True
        mock_variant_set = MagicMock()
        mock_variant_set.GetVariantNames.return_value = ["other_variant"]  # physx_variant not available
        mock_variant_sets.GetVariantSet.return_value = mock_variant_set
        mock_prim.GetVariantSets.return_value = mock_variant_sets
        mock_stage.GetPrimAtPath.return_value = mock_prim

        count, paths = self.switcher.switch_variants_for_simulation(mock_stage, "PhysX", "Physics")
        self.assertEqual(count, 0)
        self.assertEqual(paths, [])

    def test_switches_multiple_prims(self):
        """Should switch variants on multiple prims."""
        self.switcher.register_simulator_variant("PhysX", "physx_variant")
        self.mock_variant_manager.get_prims_with_variant.return_value = [
            "/World/Prim1",
            "/World/Prim2",
            "/World/Prim3"
        ]

        mock_stage = MagicMock()

        def create_mock_prim():
            mock_prim = MagicMock()
            mock_prim.IsValid.return_value = True
            mock_variant_sets = MagicMock()
            mock_variant_sets.HasVariantSet.return_value = True
            mock_variant_set = MagicMock()
            mock_variant_set.GetVariantNames.return_value = ["physx_variant", "newton_variant"]
            mock_variant_set.GetVariantSelection.return_value = "newton_variant"
            mock_variant_sets.GetVariantSet.return_value = mock_variant_set
            mock_prim.GetVariantSets.return_value = mock_variant_sets
            return mock_prim

        mock_stage.GetPrimAtPath.side_effect = [create_mock_prim() for _ in range(3)]

        count, paths = self.switcher.switch_variants_for_simulation(mock_stage, "PhysX", "Physics")
        self.assertEqual(count, 3)
        self.assertEqual(paths, ["/World/Prim1", "/World/Prim2", "/World/Prim3"])


class SwitchVariantsForActiveSimulationTests(AsyncTestCase):
    """Tests for VariantSwitcher.switch_variants_for_active_simulation method."""

    def setUp(self):
        """Set up test fixtures."""
        self.switcher = VariantSwitcher()
        self.mock_variant_manager = MagicMock()
        self.switcher._variant_manager = self.mock_variant_manager

    def tearDown(self):
        """Clean up after tests."""
        self.switcher = None

    # =========================================================================
    # Tests for no active simulation scenarios
    # =========================================================================

    @patch.object(VariantSwitcher, 'get_active_simulation')
    def test_returns_zero_when_no_active_simulation(self, mock_get_active_sim):
        """Should return 0 when no active simulation is found."""
        mock_get_active_sim.return_value = None
        mock_stage = MagicMock()

        result = self.switcher.switch_variants_for_active_simulation(mock_stage)
        self.assertEqual(result, 0)

    @patch.object(VariantSwitcher, 'get_active_simulation')
    def test_returns_zero_when_no_variant_manager(self, mock_get_active_sim):
        """Should return 0 when variant manager is None."""
        mock_get_active_sim.return_value = (1, "PhysX")
        self.switcher._variant_manager = None
        mock_stage = MagicMock()

        result = self.switcher.switch_variants_for_active_simulation(mock_stage)
        self.assertEqual(result, 0)

    @patch.object(VariantSwitcher, 'get_active_simulation')
    def test_returns_zero_when_no_registered_variant_names(self, mock_get_active_sim):
        """Should return 0 when no variant names are registered."""
        mock_get_active_sim.return_value = (1, "PhysX")
        self.mock_variant_manager.get_registered_variant_names.return_value = []
        mock_stage = MagicMock()

        result = self.switcher.switch_variants_for_active_simulation(mock_stage)
        self.assertEqual(result, 0)

    # =========================================================================
    # Tests for successful variant switching
    # =========================================================================

    @patch('omni.physics.isaacsimready.scripts.variant_switcher.nm')
    @patch.object(VariantSwitcher, 'switch_variants_for_simulation')
    @patch.object(VariantSwitcher, 'get_active_simulation')
    def test_switches_variants_for_each_registered_variant_name(
        self, mock_get_active_sim, mock_switch_for_sim, mock_nm
    ):
        """Should call switch_variants_for_simulation for each registered variant name."""
        mock_get_active_sim.return_value = (1, "PhysX")
        self.mock_variant_manager.get_registered_variant_names.return_value = ["Physics", "Rendering"]
        mock_switch_for_sim.return_value = (1, ["/World/Prim1"])
        mock_stage = MagicMock()

        result = self.switcher.switch_variants_for_active_simulation(mock_stage)

        self.assertEqual(result, 2)
        self.assertEqual(mock_switch_for_sim.call_count, 2)
        mock_switch_for_sim.assert_any_call(mock_stage, "PhysX", "Physics")
        mock_switch_for_sim.assert_any_call(mock_stage, "PhysX", "Rendering")

    @patch('omni.physics.isaacsimready.scripts.variant_switcher.nm')
    @patch.object(VariantSwitcher, 'switch_variants_for_simulation')
    @patch.object(VariantSwitcher, 'get_active_simulation')
    def test_returns_total_switched_count(self, mock_get_active_sim, mock_switch_for_sim, mock_nm):
        """Should return the sum of all switched prims across variant names."""
        mock_get_active_sim.return_value = (1, "PhysX")
        self.mock_variant_manager.get_registered_variant_names.return_value = ["Physics", "Rendering"]
        mock_switch_for_sim.side_effect = [
            (3, ["/World/Prim1", "/World/Prim2", "/World/Prim3"]),  # 3 for Physics
            (2, ["/World/Prim4", "/World/Prim5"])  # 2 for Rendering
        ]
        mock_stage = MagicMock()

        result = self.switcher.switch_variants_for_active_simulation(mock_stage)
        self.assertEqual(result, 5)

    # =========================================================================
    # Tests for notification posting
    # =========================================================================

    @patch('omni.physics.isaacsimready.scripts.variant_switcher._HAS_NOTIFICATION_MANAGER', True)
    @patch('omni.physics.isaacsimready.scripts.variant_switcher.nm')
    @patch.object(VariantSwitcher, 'switch_variants_for_simulation')
    @patch.object(VariantSwitcher, 'get_active_simulation')
    def test_posts_notification_when_variants_switched(
        self, mock_get_active_sim, mock_switch_for_sim, mock_nm
    ):
        """Should post notification when variants are actually switched."""
        mock_get_active_sim.return_value = (1, "PhysX")
        self.mock_variant_manager.get_registered_variant_names.return_value = ["Physics"]
        mock_switch_for_sim.return_value = (2, ["/World/Prim1", "/World/Prim2"])
        self.switcher.register_simulator_variant("PhysX", "physx_variant")
        mock_stage = MagicMock()

        self.switcher.switch_variants_for_active_simulation(mock_stage)

        mock_nm.post_notification.assert_called_once()
        call_args = mock_nm.post_notification.call_args
        # Check that the notification message contains the simulation and variant names and prim names
        self.assertIn("PhysX", call_args[0][0])
        self.assertIn("physx_variant", call_args[0][0])
        self.assertIn("/World/Prim1", call_args[0][0])
        self.assertEqual(call_args[1]["duration"], 5)

    @patch('omni.physics.isaacsimready.scripts.variant_switcher._HAS_NOTIFICATION_MANAGER', True)
    @patch('omni.physics.isaacsimready.scripts.variant_switcher.nm')
    @patch.object(VariantSwitcher, 'switch_variants_for_simulation')
    @patch.object(VariantSwitcher, 'get_active_simulation')
    def test_does_not_post_notification_when_no_variants_switched(
        self, mock_get_active_sim, mock_switch_for_sim, mock_nm
    ):
        """Should not post notification when no variants were switched."""
        mock_get_active_sim.return_value = (1, "PhysX")
        self.mock_variant_manager.get_registered_variant_names.return_value = ["Physics"]
        mock_switch_for_sim.return_value = (0, [])  # No variants switched
        mock_stage = MagicMock()

        self.switcher.switch_variants_for_active_simulation(mock_stage)

        mock_nm.post_notification.assert_not_called()

    @patch('omni.physics.isaacsimready.scripts.variant_switcher._HAS_NOTIFICATION_MANAGER', True)
    @patch('omni.physics.isaacsimready.scripts.variant_switcher.nm')
    @patch.object(VariantSwitcher, 'switch_variants_for_simulation')
    @patch.object(VariantSwitcher, 'get_active_simulation')
    def test_notification_includes_correct_simulation_name(
        self, mock_get_active_sim, mock_switch_for_sim, mock_nm
    ):
        """Notification should include the correct simulation name."""
        mock_get_active_sim.return_value = (1, "Newton")
        self.mock_variant_manager.get_registered_variant_names.return_value = ["Physics"]
        mock_switch_for_sim.return_value = (1, ["/World/NewtonPrim"])
        self.switcher.register_simulator_variant("Newton", "newton_variant")
        mock_stage = MagicMock()

        self.switcher.switch_variants_for_active_simulation(mock_stage)

        call_args = mock_nm.post_notification.call_args
        self.assertIn("Newton", call_args[0][0])
        self.assertIn("newton_variant", call_args[0][0])
        self.assertIn("/World/NewtonPrim", call_args[0][0])

    @patch('omni.physics.isaacsimready.scripts.variant_switcher._HAS_NOTIFICATION_MANAGER', False)
    @patch.object(VariantSwitcher, 'switch_variants_for_simulation')
    @patch.object(VariantSwitcher, 'get_active_simulation')
    def test_does_not_post_notification_when_manager_unavailable(
        self, mock_get_active_sim, mock_switch_for_sim
    ):
        """Should not attempt to post notification when notification manager is unavailable."""
        mock_get_active_sim.return_value = (1, "PhysX")
        self.mock_variant_manager.get_registered_variant_names.return_value = ["Physics"]
        mock_switch_for_sim.return_value = (2, ["/World/Prim1", "/World/Prim2"])  # Variants switched
        self.switcher.register_simulator_variant("PhysX", "physx_variant")
        mock_stage = MagicMock()

        # Should not raise even though nm might be None
        result = self.switcher.switch_variants_for_active_simulation(mock_stage)
        self.assertEqual(result, 2)

    @patch('omni.physics.isaacsimready.scripts.variant_switcher._HAS_NOTIFICATION_MANAGER', True)
    @patch('omni.physics.isaacsimready.scripts.variant_switcher.nm')
    @patch.object(VariantSwitcher, 'switch_variants_for_simulation')
    @patch.object(VariantSwitcher, 'get_active_simulation')
    def test_notification_truncates_long_prim_list(
        self, mock_get_active_sim, mock_switch_for_sim, mock_nm
    ):
        """Notification should show first 3 prims and indicate 'N more' when there are more than 3."""
        mock_get_active_sim.return_value = (1, "PhysX")
        self.mock_variant_manager.get_registered_variant_names.return_value = ["Physics"]
        # Return 5 prims
        mock_switch_for_sim.return_value = (5, [
            "/World/Prim1", "/World/Prim2", "/World/Prim3",
            "/World/Prim4", "/World/Prim5"
        ])
        self.switcher.register_simulator_variant("PhysX", "physx_variant")
        mock_stage = MagicMock()

        self.switcher.switch_variants_for_active_simulation(mock_stage)

        mock_nm.post_notification.assert_called_once()
        call_args = mock_nm.post_notification.call_args
        message = call_args[0][0]
        # Check that first 3 prims are shown
        self.assertIn("/World/Prim1", message)
        self.assertIn("/World/Prim2", message)
        self.assertIn("/World/Prim3", message)
        # Check that "2 more" is indicated
        self.assertIn("2 more", message)
        # Check that the 4th and 5th prims are NOT in the message
        self.assertNotIn("/World/Prim4", message)
        self.assertNotIn("/World/Prim5", message)


class GetActiveSimulationTests(AsyncTestCase):
    """Tests for VariantSwitcher.get_active_simulation method."""

    def setUp(self):
        """Set up test fixtures."""
        self.switcher = VariantSwitcher()

    def tearDown(self):
        """Clean up after tests."""
        self.switcher = None

    @patch('omni.physics.isaacsimready.scripts.variant_switcher.omni.physics.core.get_physics_interface')
    def test_returns_none_when_no_simulations(self, mock_get_physics):
        """Should return None when no simulations exist."""
        mock_physics = MagicMock()
        mock_physics.get_simulation_ids.return_value = []
        mock_get_physics.return_value = mock_physics

        result = self.switcher.get_active_simulation()
        self.assertIsNone(result)

    @patch('omni.physics.isaacsimready.scripts.variant_switcher.omni.physics.core.k_invalid_simulation_id', -1)
    @patch('omni.physics.isaacsimready.scripts.variant_switcher.omni.physics.core.get_physics_interface')
    def test_skips_invalid_simulation_ids(self, mock_get_physics):
        """Should skip invalid simulation IDs."""
        mock_physics = MagicMock()
        mock_physics.get_simulation_ids.return_value = [-1]  # Invalid ID
        mock_get_physics.return_value = mock_physics

        result = self.switcher.get_active_simulation()
        self.assertIsNone(result)

    @patch('omni.physics.isaacsimready.scripts.variant_switcher.omni.physics.core.k_invalid_simulation_id', -1)
    @patch('omni.physics.isaacsimready.scripts.variant_switcher.omni.physics.core.get_physics_interface')
    def test_returns_first_active_simulation(self, mock_get_physics):
        """Should return the first active simulation."""
        mock_physics = MagicMock()
        mock_physics.get_simulation_ids.return_value = [1, 2, 3]
        mock_physics.is_simulation_active.side_effect = [False, True, True]
        mock_physics.get_simulation_name.return_value = "PhysX"
        mock_get_physics.return_value = mock_physics

        result = self.switcher.get_active_simulation()

        self.assertIsNotNone(result)
        sim_id, sim_name = result
        self.assertEqual(sim_id, 2)
        self.assertEqual(sim_name, "PhysX")

    @patch('omni.physics.isaacsimready.scripts.variant_switcher.omni.physics.core.k_invalid_simulation_id', -1)
    @patch('omni.physics.isaacsimready.scripts.variant_switcher.omni.physics.core.get_physics_interface')
    def test_returns_empty_string_for_none_simulation_name(self, mock_get_physics):
        """Should return empty string if simulation name is None."""
        mock_physics = MagicMock()
        mock_physics.get_simulation_ids.return_value = [1]
        mock_physics.is_simulation_active.return_value = True
        mock_physics.get_simulation_name.return_value = None
        mock_get_physics.return_value = mock_physics

        result = self.switcher.get_active_simulation()

        self.assertIsNotNone(result)
        sim_id, sim_name = result
        self.assertEqual(sim_id, 1)
        self.assertEqual(sim_name, "")

    @patch('omni.physics.isaacsimready.scripts.variant_switcher.omni.physics.core.k_invalid_simulation_id', -1)
    @patch('omni.physics.isaacsimready.scripts.variant_switcher.omni.physics.core.get_physics_interface')
    def test_returns_none_when_no_active_simulations(self, mock_get_physics):
        """Should return None when simulations exist but none are active."""
        mock_physics = MagicMock()
        mock_physics.get_simulation_ids.return_value = [1, 2, 3]
        mock_physics.is_simulation_active.return_value = False
        mock_get_physics.return_value = mock_physics

        result = self.switcher.get_active_simulation()
        self.assertIsNone(result)


class VariantManagerPropertyTests(AsyncTestCase):
    """Tests for VariantSwitcher.variant_manager property."""

    def setUp(self):
        """Set up test fixtures."""
        self.switcher = VariantSwitcher()

    def tearDown(self):
        """Clean up after tests."""
        self.switcher = None

    def test_variant_manager_returns_none_initially(self):
        """variant_manager property should return None before startup."""
        self.assertIsNone(self.switcher.variant_manager)

    def test_variant_manager_returns_instance_after_assignment(self):
        """variant_manager property should return the assigned instance."""
        mock_manager = MagicMock()
        self.switcher._variant_manager = mock_manager

        self.assertEqual(self.switcher.variant_manager, mock_manager)
