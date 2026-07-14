# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

from unittest.mock import patch, MagicMock, PropertyMock
from omni.kit.test.async_unittest import AsyncTestCase

from pxr import Sdf

from omni.physics.isaacsimready.scripts.variant_manager import VariantManager
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

    @patch("omni.physics.isaacsimready.scripts.variant_switcher.Sdf")
    def test_switches_variant_on_single_prim(self, mock_sdf):
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

        # Mock target layer without existing opinion
        mock_target_layer = MagicMock()
        mock_target_layer.GetPrimAtPath.return_value = None
        mock_edit_target = MagicMock()
        mock_edit_target.GetLayer.return_value = mock_target_layer
        mock_stage.GetEditTarget.return_value = mock_edit_target

        # Mock session layer
        mock_session_prim_spec = MagicMock()
        mock_session_prim_spec.variantSelections = {}
        mock_sdf.CreatePrimInLayer.return_value = mock_session_prim_spec

        count, paths = self.switcher.switch_variants_for_simulation(mock_stage, "PhysX", "Physics")

        self.assertEqual(count, 1)
        self.assertEqual(paths, ["/World/Prim1"])
        self.assertEqual(mock_session_prim_spec.variantSelections["Physics"], "physx_variant")

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

    @patch("omni.physics.isaacsimready.scripts.variant_switcher.Sdf")
    def test_switches_multiple_prims(self, mock_sdf):
        """Should switch variants on multiple prims."""
        self.switcher.register_simulator_variant("PhysX", "physx_variant")
        self.mock_variant_manager.get_prims_with_variant.return_value = [
            "/World/Prim1",
            "/World/Prim2",
            "/World/Prim3"
        ]

        mock_stage = MagicMock()

        # Mock target layer without existing opinion
        mock_target_layer = MagicMock()
        mock_target_layer.GetPrimAtPath.return_value = None
        mock_edit_target = MagicMock()
        mock_edit_target.GetLayer.return_value = mock_target_layer
        mock_stage.GetEditTarget.return_value = mock_edit_target

        # Mock session layer
        mock_session_prim_spec = MagicMock()
        mock_session_prim_spec.variantSelections = {}
        mock_sdf.CreatePrimInLayer.return_value = mock_session_prim_spec

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


class FindVariantBySubstringTests(AsyncTestCase):
    """Tests for VariantSwitcher._find_variant_by_substring method."""

    def setUp(self):
        """Set up test fixtures."""
        self.switcher = VariantSwitcher()

    def tearDown(self):
        """Clean up after tests."""
        self.switcher = None

    # =========================================================================
    # Tests for exact and substring matches
    # =========================================================================

    def test_exact_match_returns_variant(self):
        """Should return the variant when an exact match exists."""
        result = self.switcher._find_variant_by_substring(
            "physx", ["physx", "newton"], "/World/Prim1", "Physics"
        )
        self.assertEqual(result, "physx")

    def test_substring_match_returns_variant(self):
        """Should return the variant that contains the search string as a substring."""
        result = self.switcher._find_variant_by_substring(
            "physx", ["physx_mimic", "newton"], "/World/Prim1", "Physics"
        )
        self.assertEqual(result, "physx_mimic")

    def test_no_match_returns_none(self):
        """Should return None when no variant contains the search string."""
        result = self.switcher._find_variant_by_substring(
            "physx", ["newton", "mujoco"], "/World/Prim1", "Physics"
        )
        self.assertIsNone(result)

    def test_empty_available_variants_returns_none(self):
        """Should return None when available variants list is empty."""
        result = self.switcher._find_variant_by_substring(
            "physx", [], "/World/Prim1", "Physics"
        )
        self.assertIsNone(result)

    def test_multiple_matches_returns_first(self):
        """Should return the first match when multiple variants contain the substring."""
        result = self.switcher._find_variant_by_substring(
            "physx", ["physx_mimic", "physx_gpu", "newton"], "/World/Prim1", "Physics"
        )
        self.assertEqual(result, "physx_mimic")

    def test_multiple_matches_logs_info(self):
        """Should log an info message when multiple matches are found."""
        with patch("omni.physics.isaacsimready.scripts.variant_switcher.carb") as mock_carb:
            self.switcher._find_variant_by_substring(
                "physx", ["physx_mimic", "physx_gpu"], "/World/Prim1", "Physics"
            )
            mock_carb.log_info.assert_called_once()
            msg = mock_carb.log_info.call_args[0][0]
            self.assertIn("Multiple variants", msg)
            self.assertIn("using the first one", msg)
            self.assertIn("physx_mimic", msg)

    def test_single_match_does_not_log_multiple(self):
        """Should not log a 'multiple variants' message when only one match exists."""
        with patch("omni.physics.isaacsimready.scripts.variant_switcher.carb") as mock_carb:
            self.switcher._find_variant_by_substring(
                "physx", ["physx_mimic", "newton"], "/World/Prim1", "Physics"
            )
            # Should not have logged any info about multiple variants
            if mock_carb.log_info.called:
                msg = mock_carb.log_info.call_args[0][0]
                self.assertNotIn("Multiple variants", msg)

    def test_no_match_does_not_log_warning(self):
        """Should not log a warning when no match is found."""
        with patch("omni.physics.isaacsimready.scripts.variant_switcher.carb") as mock_carb:
            self.switcher._find_variant_by_substring(
                "physx", ["newton", "mujoco"], "/World/Prim1", "Physics"
            )
            mock_carb.log_warn.assert_not_called()

    def test_substring_at_end(self):
        """Should match when the search string appears at the end of a variant name."""
        result = self.switcher._find_variant_by_substring(
            "gpu", ["physx_gpu", "newton"], "/World/Prim1", "Physics"
        )
        self.assertEqual(result, "physx_gpu")

    def test_substring_in_middle(self):
        """Should match when the search string appears in the middle of a variant name."""
        result = self.switcher._find_variant_by_substring(
            "phy", ["my_physx_variant", "newton"], "/World/Prim1", "Physics"
        )
        self.assertEqual(result, "my_physx_variant")


class SubstringMatchInSwitchVariantsTests(AsyncTestCase):
    """Tests for substring matching behavior integrated into switch_variants_for_simulation."""

    def setUp(self):
        """Set up test fixtures."""
        self.switcher = VariantSwitcher()
        self.mock_variant_manager = MagicMock()
        self.switcher._variant_manager = self.mock_variant_manager

    def tearDown(self):
        """Clean up after tests."""
        self.switcher = None

    def _make_mock_stage(self, available_variants, current_variant="newton"):
        """Helper to create a mock stage with a single prim that has the given available variants."""
        mock_stage = MagicMock()
        mock_prim = MagicMock()
        mock_prim.IsValid.return_value = True
        mock_variant_sets = MagicMock()
        mock_variant_sets.HasVariantSet.return_value = True
        mock_variant_set = MagicMock()
        mock_variant_set.GetVariantNames.return_value = available_variants
        mock_variant_set.GetVariantSelection.return_value = current_variant
        mock_variant_sets.GetVariantSet.return_value = mock_variant_set
        mock_prim.GetVariantSets.return_value = mock_variant_sets
        mock_stage.GetPrimAtPath.return_value = mock_prim

        # Mock target layer without existing opinion
        mock_target_layer = MagicMock()
        mock_target_layer.GetPrimAtPath.return_value = None
        mock_edit_target = MagicMock()
        mock_edit_target.GetLayer.return_value = mock_target_layer
        mock_stage.GetEditTarget.return_value = mock_edit_target

        # Mock session layer
        mock_session_layer = MagicMock()
        mock_session_prim_spec = MagicMock()
        mock_session_prim_spec.variantSelections = {}
        mock_session_layer.GetPrimAtPath.return_value = mock_session_prim_spec
        mock_stage.GetSessionLayer.return_value = mock_session_layer

        return mock_stage, mock_session_prim_spec

    @patch("omni.physics.isaacsimready.scripts.variant_switcher.Sdf")
    def test_substring_match_switches_variant(self, mock_sdf):
        """Should switch to a variant found by substring match."""
        self.switcher.register_simulator_variant("PhysX", "physx")
        self.mock_variant_manager.get_prims_with_variant.return_value = ["/World/Prim1"]

        mock_stage, mock_session_prim_spec = self._make_mock_stage(
            ["physx_mimic", "newton"], current_variant="newton"
        )
        mock_sdf.CreatePrimInLayer.return_value = mock_session_prim_spec

        count, paths = self.switcher.switch_variants_for_simulation(mock_stage, "PhysX", "Physics")

        self.assertEqual(count, 1)
        self.assertEqual(paths, ["/World/Prim1"])
        # The session layer should get the full matched variant name, not the substring
        self.assertEqual(mock_session_prim_spec.variantSelections["Physics"], "physx_mimic")

    @patch("omni.physics.isaacsimready.scripts.variant_switcher.Sdf")
    def test_no_substring_match_skips_prim_without_warning(self, mock_sdf):
        """Should skip prim without warning when no variant matches the substring."""
        self.switcher.register_simulator_variant("PhysX", "physx")
        self.mock_variant_manager.get_prims_with_variant.return_value = ["/World/Prim1"]

        mock_stage, _ = self._make_mock_stage(["newton", "mujoco"], current_variant="newton")

        with patch("omni.physics.isaacsimready.scripts.variant_switcher.carb") as mock_carb:
            count, paths = self.switcher.switch_variants_for_simulation(mock_stage, "PhysX", "Physics")

        self.assertEqual(count, 0)
        self.assertEqual(paths, [])
        mock_carb.log_warn.assert_not_called()

    @patch("omni.physics.isaacsimready.scripts.variant_switcher.Sdf")
    def test_multiple_substring_matches_uses_first(self, mock_sdf):
        """Should use the first matching variant when multiple contain the substring."""
        self.switcher.register_simulator_variant("PhysX", "physx")
        self.mock_variant_manager.get_prims_with_variant.return_value = ["/World/Prim1"]

        mock_stage, mock_session_prim_spec = self._make_mock_stage(
            ["physx_mimic", "physx_gpu", "newton"], current_variant="newton"
        )
        mock_sdf.CreatePrimInLayer.return_value = mock_session_prim_spec

        count, paths = self.switcher.switch_variants_for_simulation(mock_stage, "PhysX", "Physics")

        self.assertEqual(count, 1)
        self.assertEqual(mock_session_prim_spec.variantSelections["Physics"], "physx_mimic")

    @patch("omni.physics.isaacsimready.scripts.variant_switcher.Sdf")
    def test_exact_match_still_works(self, mock_sdf):
        """Exact match should still work since it is a valid substring match."""
        self.switcher.register_simulator_variant("PhysX", "physx_variant")
        self.mock_variant_manager.get_prims_with_variant.return_value = ["/World/Prim1"]

        mock_stage, mock_session_prim_spec = self._make_mock_stage(
            ["physx_variant", "newton_variant"], current_variant="newton_variant"
        )
        mock_sdf.CreatePrimInLayer.return_value = mock_session_prim_spec

        count, paths = self.switcher.switch_variants_for_simulation(mock_stage, "PhysX", "Physics")

        self.assertEqual(count, 1)
        self.assertEqual(mock_session_prim_spec.variantSelections["Physics"], "physx_variant")

    @patch("omni.physics.isaacsimready.scripts.variant_switcher.Sdf")
    def test_no_switch_when_already_on_matched_variant(self, mock_sdf):
        """Should not switch when prim is already on the substring-matched variant."""
        self.switcher.register_simulator_variant("PhysX", "physx")
        self.mock_variant_manager.get_prims_with_variant.return_value = ["/World/Prim1"]

        mock_stage, _ = self._make_mock_stage(
            ["physx_mimic", "newton"], current_variant="physx_mimic"
        )

        count, paths = self.switcher.switch_variants_for_simulation(mock_stage, "PhysX", "Physics")

        self.assertEqual(count, 0)
        self.assertEqual(paths, [])

    # =========================================================================
    # Tests for session layer writes and target layer opinion skipping
    # =========================================================================

    @patch("omni.physics.isaacsimready.scripts.variant_switcher.Sdf")
    def test_variant_written_to_session_layer(self, mock_sdf):
        """Variant selection should be written via Sdf.CreatePrimInLayer on the session layer."""
        self.switcher.register_simulator_variant("PhysX", "physx")
        self.mock_variant_manager.get_prims_with_variant.return_value = ["/World/Prim1"]

        mock_stage, mock_session_prim_spec = self._make_mock_stage(
            ["physx_mimic", "newton"], current_variant="newton"
        )
        mock_session_layer = mock_stage.GetSessionLayer()
        mock_sdf.CreatePrimInLayer.return_value = mock_session_prim_spec

        self.switcher.switch_variants_for_simulation(mock_stage, "PhysX", "Physics")

        # Sdf.CreatePrimInLayer must be called with the session layer, not the target layer
        mock_sdf.CreatePrimInLayer.assert_called_once_with(mock_session_layer, "/World/Prim1")
        self.assertEqual(mock_session_prim_spec.variantSelections["Physics"], "physx_mimic")

    @patch("omni.physics.isaacsimready.scripts.variant_switcher.Sdf")
    def test_session_layer_opinion_tracked(self, mock_sdf):
        """Switched prim should be recorded in _session_layer_opinions."""
        self.switcher.register_simulator_variant("PhysX", "physx")
        self.mock_variant_manager.get_prims_with_variant.return_value = ["/World/Prim1"]

        mock_stage, mock_session_prim_spec = self._make_mock_stage(
            ["physx_mimic", "newton"], current_variant="newton"
        )
        mock_sdf.CreatePrimInLayer.return_value = mock_session_prim_spec

        self.switcher.switch_variants_for_simulation(mock_stage, "PhysX", "Physics")

        self.assertIn(("/World/Prim1", "Physics"), self.switcher._session_layer_opinions)

    @patch("omni.physics.isaacsimready.scripts.variant_switcher.Sdf")
    def test_skips_prim_when_target_layer_has_opinion(self, mock_sdf):
        """Should skip a prim when the target layer already has a variant opinion."""
        self.switcher.register_simulator_variant("PhysX", "physx")
        self.mock_variant_manager.get_prims_with_variant.return_value = ["/World/Prim1"]

        mock_stage, mock_session_prim_spec = self._make_mock_stage(
            ["physx_mimic", "newton"], current_variant="newton"
        )

        # Set up target layer to already have an opinion for this variant set
        mock_target_prim_spec = MagicMock()
        mock_target_prim_spec.variantSelections = {"Physics": "newton"}
        mock_target_layer = mock_stage.GetEditTarget().GetLayer()
        mock_target_layer.GetPrimAtPath.return_value = mock_target_prim_spec

        count, paths = self.switcher.switch_variants_for_simulation(mock_stage, "PhysX", "Physics")

        self.assertEqual(count, 0)
        self.assertEqual(paths, [])
        mock_sdf.CreatePrimInLayer.assert_not_called()

    @patch("omni.physics.isaacsimready.scripts.variant_switcher.Sdf")
    def test_target_layer_opinion_on_different_variant_set_does_not_block(self, mock_sdf):
        """A target layer opinion on a different variant set should not block switching."""
        self.switcher.register_simulator_variant("PhysX", "physx")
        self.mock_variant_manager.get_prims_with_variant.return_value = ["/World/Prim1"]

        mock_stage, mock_session_prim_spec = self._make_mock_stage(
            ["physx_mimic", "newton"], current_variant="newton"
        )

        # Target layer has opinion for "Rendering" but not "Physics"
        mock_target_prim_spec = MagicMock()
        mock_target_prim_spec.variantSelections = {"Rendering": "high"}
        mock_target_layer = mock_stage.GetEditTarget().GetLayer()
        mock_target_layer.GetPrimAtPath.return_value = mock_target_prim_spec

        mock_sdf.CreatePrimInLayer.return_value = mock_session_prim_spec

        count, paths = self.switcher.switch_variants_for_simulation(mock_stage, "PhysX", "Physics")

        self.assertEqual(count, 1)
        self.assertEqual(paths, ["/World/Prim1"])
        self.assertEqual(mock_session_prim_spec.variantSelections["Physics"], "physx_mimic")

    @patch("omni.physics.isaacsimready.scripts.variant_switcher.Sdf")
    def test_mixed_prims_opinion_and_no_opinion(self, mock_sdf):
        """Should switch only prims without a target layer opinion, skipping those that have one."""
        self.switcher.register_simulator_variant("PhysX", "physx")
        self.mock_variant_manager.get_prims_with_variant.return_value = [
            "/World/Prim1",
            "/World/Prim2",
        ]

        mock_stage = MagicMock()

        # Prim1: valid, has variant set, no target layer opinion -> should switch
        mock_prim1 = MagicMock()
        mock_prim1.IsValid.return_value = True
        mock_variant_sets1 = MagicMock()
        mock_variant_sets1.HasVariantSet.return_value = True
        mock_variant_set1 = MagicMock()
        mock_variant_set1.GetVariantNames.return_value = ["physx_mimic", "newton"]
        mock_variant_set1.GetVariantSelection.return_value = "newton"
        mock_variant_sets1.GetVariantSet.return_value = mock_variant_set1
        mock_prim1.GetVariantSets.return_value = mock_variant_sets1

        # Prim2: valid, has variant set, but target layer has opinion -> should skip
        mock_prim2 = MagicMock()
        mock_prim2.IsValid.return_value = True
        mock_variant_sets2 = MagicMock()
        mock_variant_sets2.HasVariantSet.return_value = True
        mock_variant_set2 = MagicMock()
        mock_variant_set2.GetVariantNames.return_value = ["physx_mimic", "newton"]
        mock_variant_set2.GetVariantSelection.return_value = "newton"
        mock_variant_sets2.GetVariantSet.return_value = mock_variant_set2
        mock_prim2.GetVariantSets.return_value = mock_variant_sets2

        mock_stage.GetPrimAtPath.side_effect = [mock_prim1, mock_prim2]

        # Target layer: no spec for Prim1, has opinion for Prim2
        mock_target_prim_spec2 = MagicMock()
        mock_target_prim_spec2.variantSelections = {"Physics": "newton"}
        mock_target_layer = MagicMock()
        mock_target_layer.GetPrimAtPath.side_effect = [None, mock_target_prim_spec2]
        mock_edit_target = MagicMock()
        mock_edit_target.GetLayer.return_value = mock_target_layer
        mock_stage.GetEditTarget.return_value = mock_edit_target

        # Session layer
        mock_session_layer = MagicMock()
        mock_session_prim_spec = MagicMock()
        mock_session_prim_spec.variantSelections = {}
        mock_stage.GetSessionLayer.return_value = mock_session_layer
        mock_sdf.CreatePrimInLayer.return_value = mock_session_prim_spec

        count, paths = self.switcher.switch_variants_for_simulation(mock_stage, "PhysX", "Physics")

        self.assertEqual(count, 1)
        self.assertEqual(paths, ["/World/Prim1"])
        # CreatePrimInLayer should only be called once (for Prim1)
        mock_sdf.CreatePrimInLayer.assert_called_once_with(mock_session_layer, "/World/Prim1")

    @patch("omni.physics.isaacsimready.scripts.variant_switcher.Sdf")
    def test_no_target_prim_spec_does_not_block(self, mock_sdf):
        """When target layer has no prim spec at all for the path, switching should proceed."""
        self.switcher.register_simulator_variant("PhysX", "physx")
        self.mock_variant_manager.get_prims_with_variant.return_value = ["/World/Prim1"]

        mock_stage, mock_session_prim_spec = self._make_mock_stage(
            ["physx_mimic", "newton"], current_variant="newton"
        )
        # Target layer returns None for the prim path (no spec exists)
        mock_target_layer = mock_stage.GetEditTarget().GetLayer()
        mock_target_layer.GetPrimAtPath.return_value = None

        mock_sdf.CreatePrimInLayer.return_value = mock_session_prim_spec

        count, paths = self.switcher.switch_variants_for_simulation(mock_stage, "PhysX", "Physics")

        self.assertEqual(count, 1)
        self.assertEqual(paths, ["/World/Prim1"])


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


class StageLifecycleRegressionTests(AsyncTestCase):
    """Regression tests for NvBugs 6187664.

    The deferred resync handler must never call stage.GetPrimAtPath against a
    stage that was swapped or closed between the time paths were queued and
    the time the deferred update fires. Doing so can crash inside USD's
    instance cache (Usd_InstanceCache::IsPathInPrototype) when the cache for
    the original stage has been torn down or rebuilt for a different stage.
    """

    def setUp(self):
        self.switcher = VariantSwitcher()
        self.switcher._variant_manager = VariantManager()
        self.switcher._variant_manager.startup()
        # Register at least one variant name so process_resynced_paths gets
        # past its early-exit on empty names_to_process.
        self.switcher._variant_manager.register_simulation_variant("Physics")

    def tearDown(self):
        if self.switcher._variant_manager is not None:
            self.switcher._variant_manager.shutdown()
        self.switcher = None

    @staticmethod
    def _make_stage(identifier):
        """Create a MagicMock stage whose root layer reports the given identifier."""
        stage = MagicMock()
        root_layer = MagicMock()
        root_layer.identifier = identifier
        stage.GetRootLayer.return_value = root_layer
        return stage

    # ----- VariantManager: queue tagging & invalidation -----

    def test_vm_add_with_new_stage_id_drops_old_queue(self):
        """Adding paths with a different stage_id must drop the old queue."""
        vm = self.switcher._variant_manager
        vm.add_pending_resync_paths([Sdf.Path("/World/A")], stage_id="stage_A")
        self.assertEqual(vm.get_pending_resync_stage_id(), "stage_A")
        self.assertEqual(len(vm._pending_resync_paths), 1)

        vm.add_pending_resync_paths([Sdf.Path("/World/B")], stage_id="stage_B")

        self.assertEqual(vm.get_pending_resync_stage_id(), "stage_B")
        self.assertEqual(vm._pending_resync_paths, {Sdf.Path("/World/B")})

    def test_vm_add_with_none_stage_id_preserves_existing(self):
        """In-place requeue (stage_id=None) must keep the originating stage id."""
        vm = self.switcher._variant_manager
        vm.add_pending_resync_paths([Sdf.Path("/World/A")], stage_id="stage_A")
        vm.add_pending_resync_paths([Sdf.Path("/World/B")])

        self.assertEqual(vm.get_pending_resync_stage_id(), "stage_A")
        self.assertEqual(
            vm._pending_resync_paths,
            {Sdf.Path("/World/A"), Sdf.Path("/World/B")},
        )

    def test_vm_clear_pending_clears_paths_and_stage_id(self):
        vm = self.switcher._variant_manager
        vm.add_pending_resync_paths([Sdf.Path("/World/A")], stage_id="stage_A")

        vm.clear_pending_resync_paths()

        self.assertFalse(vm.has_pending_resync_paths())
        self.assertIsNone(vm.get_pending_resync_stage_id())

    def test_vm_process_drops_queue_on_stage_mismatch(self):
        """process_resynced_paths must not touch GetPrimAtPath when the
        current stage's identifier differs from the queued one — that's the
        precise condition that crashes USD in NvBugs 6187664."""
        vm = self.switcher._variant_manager
        vm.add_pending_resync_paths([Sdf.Path("/World/A")], stage_id="stage_A")

        other_stage = self._make_stage("stage_B")
        vm.process_resynced_paths(other_stage)

        other_stage.GetPrimAtPath.assert_not_called()
        self.assertFalse(vm.has_pending_resync_paths())
        self.assertIsNone(vm.get_pending_resync_stage_id())

    def test_vm_process_drops_queue_when_stage_is_none(self):
        """process_resynced_paths with a None stage must clear and return
        without dereferencing anything."""
        vm = self.switcher._variant_manager
        vm.add_pending_resync_paths([Sdf.Path("/World/A")], stage_id="stage_A")

        vm.process_resynced_paths(None)

        self.assertFalse(vm.has_pending_resync_paths())
        self.assertIsNone(vm.get_pending_resync_stage_id())

    def test_vm_process_proceeds_on_stage_match(self):
        """When the stage identifier matches, GetPrimAtPath is called and the
        queue drains. Uses an invalid prim to avoid traversing the subtree."""
        vm = self.switcher._variant_manager
        vm.add_pending_resync_paths([Sdf.Path("/World/A")], stage_id="stage_A")

        stage = self._make_stage("stage_A")
        invalid_prim = MagicMock()
        invalid_prim.IsValid.return_value = False
        stage.GetPrimAtPath.return_value = invalid_prim

        vm.process_resynced_paths(stage)

        stage.GetPrimAtPath.assert_called_once_with(Sdf.Path("/World/A"))
        self.assertFalse(vm.has_pending_resync_paths())

    # ----- VariantSwitcher._on_update: stage-identity early return -----

    def test_on_update_returns_early_when_stage_swapped(self):
        """_on_update must drop the queue and return without calling
        GetPrimAtPath or any downstream variant-switch helper when the
        current stage doesn't match the queued one."""
        vm = self.switcher._variant_manager
        vm.add_pending_resync_paths([Sdf.Path("/World/A")], stage_id="stage_A")

        current_stage = self._make_stage("stage_B")
        mock_context = MagicMock()
        mock_context.get_stage.return_value = current_stage
        self.switcher._usd_context = mock_context

        # Spy on downstream code paths — none of them should be reached.
        self.switcher.switch_variants_for_active_simulation = MagicMock()
        self.switcher._switch_variants_for_paths = MagicMock()

        # Pretend a deferred update was scheduled.
        self.switcher._update_sub = MagicMock()

        self.switcher._on_update(None)

        current_stage.GetPrimAtPath.assert_not_called()
        self.switcher.switch_variants_for_active_simulation.assert_not_called()
        self.switcher._switch_variants_for_paths.assert_not_called()
        self.assertFalse(vm.has_pending_resync_paths())
        self.assertIsNone(self.switcher._update_sub)

    def test_on_update_returns_early_when_stage_is_none(self):
        """_on_update must drop the queue and return when no stage is open."""
        vm = self.switcher._variant_manager
        vm.add_pending_resync_paths([Sdf.Path("/World/A")], stage_id="stage_A")

        mock_context = MagicMock()
        mock_context.get_stage.return_value = None
        self.switcher._usd_context = mock_context

        self.switcher.switch_variants_for_active_simulation = MagicMock()
        self.switcher._switch_variants_for_paths = MagicMock()

        self.switcher._on_update(None)

        self.switcher.switch_variants_for_active_simulation.assert_not_called()
        self.switcher._switch_variants_for_paths.assert_not_called()
        self.assertFalse(vm.has_pending_resync_paths())

    # ----- VariantSwitcher: CLOSING / CLOSED unified handler -----

    def test_on_stage_closed_cancels_update_sub_and_clears_queue(self):
        """_on_stage_closed (used for both CLOSING and CLOSED events) cancels
        the deferred update and clears the queue while the stage is still
        alive — this is what closes the race window in NvBugs 6187664."""
        vm = self.switcher._variant_manager
        vm.add_pending_resync_paths([Sdf.Path("/World/A")], stage_id="stage_A")
        self.switcher._update_sub = MagicMock()
        self.switcher._session_layer_opinions.add(("/World/A", "Physics"))

        self.switcher._on_stage_closed(None)

        self.assertIsNone(self.switcher._update_sub)
        self.assertFalse(vm.has_pending_resync_paths())
        self.assertEqual(self.switcher._session_layer_opinions, set())

    def test_on_stage_closed_is_idempotent(self):
        """The same handler is registered for both CLOSING and CLOSED, so it
        must be safe to invoke twice in a row."""
        vm = self.switcher._variant_manager
        vm.add_pending_resync_paths([Sdf.Path("/World/A")], stage_id="stage_A")
        self.switcher._update_sub = MagicMock()

        self.switcher._on_stage_closed(None)
        self.switcher._on_stage_closed(None)  # second call: no-op

        self.assertIsNone(self.switcher._update_sub)
        self.assertFalse(vm.has_pending_resync_paths())
