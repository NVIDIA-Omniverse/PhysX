# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from omni.physx.scripts import physicsUtils
from omni.physx.scripts import utils
from omni.physxtestsvisual.utils import TestCase
from omni.physxtests.utils.physicsBase import TestCategory
from pxr import Gf
import carb.input
import carb.tokens
import omni.physx.bindings._physx as pxb
import omni.ui as ui
import omni.kit.ui_test as ui_test
import omni.kit.undo
import carb.settings


class PhysxMetadataTest(TestCase):
    category = TestCategory.Core

    async def select_and_focus_property_window(self, prims):
        # select prim and focus property window so the widgets are built
        paths = [prim.GetPath().pathString for prim in prims]
        omni.usd.get_context().get_selection().set_selected_prim_paths(paths, False)
        window = ui.Workspace.get_window("Property")
        window.focus()
        await self.wait(10)

    async def test_physics_visual_local_velocities_toggle(self):
        def _set_physics_settings(output_velocities_local_space):
            settings = carb.settings.get_settings()
            settings.set_bool(pxb.SETTING_OUTPUT_VELOCITIES_LOCAL_SPACE, output_velocities_local_space)
            value = settings.get_as_bool(pxb.SETTING_OUTPUT_VELOCITIES_LOCAL_SPACE)
            self.assertTrue(value == output_velocities_local_space)
            return value

        stage = await self.new_stage()

        # create dummy actors
        prims = [physicsUtils.add_rigid_cube(stage, f"/cubeActor{i}", Gf.Vec3f(100.0)) for i in range(3)]

        # define widgets we are interested in
        local_velocities_checkbox_id = "LocalSpaceVelocitiesCheckBox"
        local_velocities_reset_button_id = "LocalSpaceVelocitiesResetButton"
        local_velocities_mixed_overlay_id = "LocalSpaceVelocitiesMixedOverlay"
        my_widget_ids_set = {local_velocities_checkbox_id, local_velocities_reset_button_id, local_velocities_mixed_overlay_id}

        widgets_dict = {}  # {widget name: widget} dictionary
        states = []  # list of states as they were during the testing

        async def _select_prims_and_find_widgets(prims):
            # focus the Ui control
            await self.select_and_focus_property_window(prims)

            # find main Rigid Body frame
            rb_frame = ui_test.find("Property//Frame/**/CollapsableFrame[*].title=='Rigid Body'")
            self.assertTrue(rb_frame is not None)

            # find my widgets
            widgets_dict.clear()
            for t in rb_frame.find_all("**"):
                if t.widget.identifier in my_widget_ids_set:
                    widgets_dict[t.widget.identifier] = t.widget
            # make sure we have found all our widgets
            self.assertTrue(my_widget_ids_set <= widgets_dict.keys())

        def _get_state():
            return [utils.get_custom_metadata(prim, pxb.METADATA_ATTRIBUTE_NAME_LOCALSPACEVELOCITIES) for prim in prims]

        def _get_and_save_state():
            state = _get_state()
            states.append(state)
            return state

        def _check_undo_redo():
            if len(states) < 2:
                return
            current_state = states[-1]
            prev_state = states[-2]
            omni.kit.undo.undo()
            undo_state = _get_state()
            self.assertTrue(prev_state == undo_state)
            omni.kit.undo.redo()
            redo_state = _get_state()
            self.assertTrue(current_state == redo_state)

        def _test_widgets_states_and_undo_redo(reset_button_enabled=False, mixed_overlay_visible=False, checkbox_checked=False, check_undo_redo=True):
            self.assertTrue(widgets_dict[local_velocities_reset_button_id].enabled == reset_button_enabled)
            self.assertTrue(widgets_dict[local_velocities_mixed_overlay_id].visible == mixed_overlay_visible)
            self.assertTrue(widgets_dict[local_velocities_checkbox_id].model.get_value_as_bool() == checkbox_checked)
            if check_undo_redo:
                _check_undo_redo()

        # set physics settings to false
        _set_physics_settings(False)

        await _select_prims_and_find_widgets(prims)

        # get initial values = no metadata as of yet
        state = _get_and_save_state()
        self.assertTrue(all(s is None for s in state))  # all None
        _test_widgets_states_and_undo_redo(check_undo_redo=False)

        # set all metadata to true
        widgets_dict[local_velocities_checkbox_id].model.set_value(True)
        state = _get_and_save_state()
        self.assertTrue(all(s is True for s in state))  # all True
        _test_widgets_states_and_undo_redo(True, False, True)

        # set all metadata to false
        widgets_dict[local_velocities_checkbox_id].model.set_value(False)
        state = _get_and_save_state()
        self.assertTrue(all(s is False for s in state))  # all False
        _test_widgets_states_and_undo_redo(True)

        # reset all values = remove metadata
        widgets_dict[local_velocities_reset_button_id].call_clicked_fn()
        state = _get_and_save_state()
        self.assertTrue(all(s is None for s in state))  # all None
        _test_widgets_states_and_undo_redo()

        # set physics settings to true
        _set_physics_settings(True)

        # get updated initial values after the setting change = no metadata are present
        state = _get_state()
        self.assertTrue(all(s is None for s in state))  # all None
        _test_widgets_states_and_undo_redo(False, False, True, False)

        # individual changes
        await _select_prims_and_find_widgets([prims[0]])
        widgets_dict[local_velocities_checkbox_id].model.set_value(False)  # create metadata set to False
        state = _get_and_save_state()
        self.assertFalse(state[0])
        self.assertTrue(state[1] is None)
        self.assertTrue(state[2] is None)
        _test_widgets_states_and_undo_redo(True, False, False)

        await _select_prims_and_find_widgets([prims[1]])
        widgets_dict[local_velocities_checkbox_id].model.set_value(False)  # create metadata set to False
        state = _get_and_save_state()
        self.assertFalse(state[0])
        self.assertFalse(state[1])
        self.assertTrue(state[2] is None)
        _test_widgets_states_and_undo_redo(True, False, False)

        widgets_dict[local_velocities_checkbox_id].model.set_value(True)   # create metadata set to True
        await _select_prims_and_find_widgets(prims)
        state = _get_and_save_state()
        self.assertFalse(state[0])
        self.assertTrue(state[1])
        self.assertTrue(state[2] is None)
        _test_widgets_states_and_undo_redo(True, True, False)

        # set physics settings to false
        _set_physics_settings(False)

        # test that metadata stayed as they were
        state = _get_state()
        self.assertFalse(state[0])
        self.assertTrue(state[1])
        self.assertTrue(state[2] is None)
        _test_widgets_states_and_undo_redo(True, True, False, False)

        # set all values (metadata) to true after mixed state
        widgets_dict[local_velocities_checkbox_id].model.set_value(True)
        state = _get_and_save_state()
        self.assertTrue(all(s is True for s in state))  # all True
        _test_widgets_states_and_undo_redo(True, False, True)

        # set all values (metadata) to false. Values will correspond to physics settings but reset button should be enabled
        widgets_dict[local_velocities_checkbox_id].model.set_value(False)
        state = _get_and_save_state()
        self.assertTrue(all(s is False for s in state))  # all False
        _test_widgets_states_and_undo_redo(True, False, False)

        # reset to default - clear all metadata even if checkbox state corresponds to physics settings value
        widgets_dict[local_velocities_reset_button_id].call_clicked_fn()
        state = _get_and_save_state()
        self.assertTrue(all(s is None for s in state))  # all None
        _test_widgets_states_and_undo_redo()

        # set physics settings to true
        _set_physics_settings(True)

        # no new metadata should be created but values in UI should change to true
        state = _get_state()
        self.assertTrue(all(s is None for s in state))  # all None
        _test_widgets_states_and_undo_redo(False, False, True, False)

        # undo test - complete step-by-step rollback
        for i in range(len(states) - 2, -1, -1):
            s = states[i]
            omni.kit.undo.undo()
            state = _get_state()
            self.assertTrue(state == s)

        # redo test - complete step-by-step restore
        for s in states[1:]:
            omni.kit.undo.redo()
            state = _get_state()
            self.assertTrue(state == s)

        # set physics settings to false
        _set_physics_settings(False)
