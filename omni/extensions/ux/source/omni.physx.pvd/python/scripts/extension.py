# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.ext
import carb
import carb.profiler
from carb.eventdispatcher import get_eventdispatcher
from omni import ui
from omni.physxpvd.bindings import _physxPvd
import omni.usd
from pprint import pprint
from datetime import datetime
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Sdf, Gf, Tf, UsdLux, UsdUtils
import os, sys
import ctypes
from enum import IntEnum
from omni.timeline import get_timeline_interface
from omni.kit.widget.stage.stage_icons import StageIcons
import omni.kit.app
import asyncio
import time
import math

import omni.kit.tool.asset_importer as ai
from typing import List, Union, Dict

from omni.physxuicommon import windowmenuitem
from omni.physx.scripts.utils import safe_import_tests

from omni.kit.widget.settings import SettingsWidgetBuilder
from omni.kit.widget.settings.settings_widget import SettingType, create_setting_widget, create_setting_widget_combo
from omni.kit.viewport.utility import frame_viewport_selection, get_active_viewport
from omni.kit.window.filepicker import FilePickerDialog

from omni.physxpvd.scripts.property_widget.property_widget_omni_pvd import PropertyWidgetOmniPvd
from omni.physxpvd.scripts.property_widget.prim_handlers.default import prim_handler as default_prim_handler

from omni.physx import get_physx_interface
from omni.physx.bindings._physx import SimulationEvent

import omni.kit.viewport.utility as vp_utils

from omni.physxpvd.scripts.omniusd_to_physxusd.omniusd_to_physxusd import ConvertOmniPvdToPhysXUSD

from omni.physx.bindings._physx import (
    SETTING_OMNIPVD_OVD_RECORDING_DIRECTORY,
    SETTING_OMNIPVD_IS_OVD_STAGE,
    SETTING_OMNIPVD_IS_RECORDING,
    SETTING_OMNIPVD_ENABLED
)
from omni.physxpvd.bindings._physxPvd import (
    SETTING_OMNIPVD_IMPORTED_OVD,
    SETTING_OMNIPVD_OVD_FOR_BAKING,
    SETTING_OMNIPVD_USD_CACHE_DIRECTORY,
    SETTING_OMNIPVD_PHYSX_USD_DIRECTORY,
    SETTING_OMNIPVD_INVALIDATE_CACHE,
    SETTING_OMNIPVD_LAST_IMPORT_DIRECTORY,

    SETTING_OMNIPVD_GIZMO_CONTACT_VIZMODE,
    SETTING_OMNIPVD_GIZMO_CENTER_OF_MASS_VIZMODE,
    SETTING_OMNIPVD_GIZMO_JOINT_VIZMODE,
    SETTING_OMNIPVD_GIZMO_BOUNDING_BOX_VIZMODE,
    SETTING_OMNIPVD_GIZMO_COORDINATE_SYSTEM_VIZMODE,
    SETTING_OMNIPVD_GIZMO_VELOCITY_VIZMODE,
    SETTING_OMNIPVD_GIZMO_TRANSPARENCY_VIZMODE,
    SETTING_OMNIPVD_GIZMO_GLOBAL_SCALE,
    SETTING_OMNIPVD_GIZMO_CONTACT_SCALE,
    SETTING_OMNIPVD_GIZMO_CENTER_OF_MASS_SCALE,
    SETTING_OMNIPVD_GIZMO_JOINT_SCALE,
    SETTING_OMNIPVD_GIZMO_COORDINATE_SYSTEM_SCALE,
    SETTING_OMNIPVD_GIZMO_VELOCITY_SCALE,
    SETTING_OMNIPVD_GIZMO_TRANSPARENCY_SCALE,

    SETTING_OMNIPVD_TIMELINE_IS_LEGACY_OVD,
    SETTING_OMNIPVD_TIMELINE_FRAME_MODE,
    SETTING_OMNIPVD_TIMELINE_PLAYBACK_STATE,
    SETTING_OMNIPVD_TIMELINE_FRAME_DELTA_MS,
    SETTING_OMNIPVD_TIMELINE_FRAMES_PER_SIM_STEP,
    SETTING_OMNIPVD_TIMELINE_FRAME_ID,
    SETTING_OMNIPVD_TIMELINE_FRAME_ID_MIN,
    SETTING_OMNIPVD_TIMELINE_FRAME_ID_MAX,
    SETTING_OMNIPVD_TIMELINE_SIM_STEP_ID,
    SETTING_OMNIPVD_TIMELINE_SIM_STEP_ID_MIN,
    SETTING_OMNIPVD_TIMELINE_SIM_STEP_ID_MAX
)

from omni.kit.widget.settings.settings_widget import SettingType
from omni.kit.widget.settings.settings_model import SettingModel
from omni.ui import color as cl

ENABLED_STYLE = {"color": 0xffcccccc}
DISABLED_STYLE = {"color": 0xff888888}

class OvdTimelineFrameMode(IntEnum):
    ePostSim = 0
    ePreSim = 1
    ePreSimAndPostSim = 2

class OvdTimelinePlaybackState(IntEnum):
    ePlaying = 0
    ePaused = 1

################################################################################
# Mirrors this from the C++ code
################################################################################
#struct OmniPVDDebugGizmoType
#{
#    enum Enum
#    {
#        eBoundingBox,
#        eContact,
#        eCenterOfMass,
#        eVelocity,
#        eCoordinateSystem,
#        eJoint,
#        eTransparency,
#        eNbrEnums
#    };
#};
################################################################################

class OmniPVDGizmoType(IntEnum):
    eBoundingBox = 0
    eContact = 1
    eCenterOfMass = 2
    eVelocity = 3
    eCoordinateSystem = 4
    eJoint = 5
    eTransparency = 6

import datetime


def set_style(style):
    if style == "NvidiaLight":
        style_data = {
            "Button.Image::filter": {"image_url": StageIcons().get("filter"), "color": 0xFF535354},
            "Button.Image::options": {"image_url": StageIcons().get("options"), "color": 0xFF535354},
            "Button.Image::visibility": {"image_url": StageIcons().get("eye_on")},
            "Button.Image::visibility:checked": {"image_url": StageIcons().get("eye_off")},
            "Button.Image::visibility:disabled": {"color": 0x608A8777},
            "Button.Image::visibility:selected": {"color": 0xFF23211F},
            "Button::filter": {"background_color": 0x0, "margin": 0},
            "Button::options": {"background_color": 0x0, "margin": 0},
            "Button::visibility": {"background_color": 0x0, "margin": 0, "margin_width": 1},
            "Button::visibility:checked": {"background_color": 0x0},
            "Button::visibility:hovered": {"background_color": 0x0},
            "Button::visibility:pressed": {"background_color": 0x0},
            "Field": {"background_color": 0xFF535354, "color": 0xFFCCCCCC},
            "Label::search": {"color": 0xFFACACAC},
            "Menu.CheckBox": {"background_color": 0x0, "margin": 0},
            "Menu.CheckBox::drag": {
                "image_url": StageIcons().get("drag"),
                "color": 0xFF505050,
                "alignment": ui.Alignment.CENTER,
            },
            "Menu.CheckBox.Image": {"image_url": StageIcons().get("check_off"), "color": 0xFF8A8777},
            "Menu.CheckBox.Image:checked": {"image_url": StageIcons().get("check_on")},
            "ScrollingFrame": {"secondary_color": 0xFF444444},
            "TreeView": {
                "background_color": 0xFFE0E0E0,
                "background_selected_color": 0x109D905C,
                "secondary_color": 0xFFACACAC,
            },
            "TreeView.ScrollingFrame": {"background_color": 0xFFE0E0E0},
            "TreeView.Header": {"color": 0xFFCCCCCC},
            "TreeView.Header::background": {
                "background_color": 0xFF535354,
                "border_color": 0xFF707070,
                "border_width": 0.5,
            },
            "TreeView.Header::columnname": {"margin": 3},
            "TreeView.Header::visibility_header": {"image_url": StageIcons().get("eye_header")},
            "TreeView.Image::object_icon_grey": {"color": 0x80FFFFFF},
            "TreeView.Item": {"color": 0xFF535354, "font_size": 16},
            "TreeView.Item::object_name": {"margin": 3},
            "TreeView.Item::object_name_grey": {"color": 0xFFACACAC},
            "TreeView.Item::object_name_missing": {"color": 0xFF6F72FF},
            "TreeView.Item:selected": {"color": 0xFF2A2825},
            "TreeView:selected": {"background_color": 0x409D905C},
            "TreeView:drop": {"background_color": 0x409D905C},
        }
    else:
        style_data = {
            "Button.Image::filter": {"image_url": StageIcons().get("filter"), "color": 0xFF8A8777},
            "Button.Image::options": {"image_url": StageIcons().get("options"), "color": 0xFF8A8777},
            "Button.Image::visibility": {"image_url": StageIcons().get("eye_on"), "color": 0xFF8A8777},
            "Button.Image::visibility:checked": {"image_url": StageIcons().get("eye_off")},
            "Button.Image::visibility:disabled": {"color": 0x608A8777},
            "Button.Image::visibility:selected": {"color": 0xFF23211F},
            "Button::filter": {"background_color": 0x0, "margin": 0},
            "Button::options": {"background_color": 0x0, "margin": 0},
            "Button::visibility": {"background_color": 0x0, "margin": 0, "margin_width": 1},
            "Button::visibility:checked": {"background_color": 0x0},
            "Button::visibility:hovered": {"background_color": 0x0},
            "Button::visibility:pressed": {"background_color": 0x0},
            "Label::search": {"color": 0xFF808080, "margin_width": 4},
            "Menu.CheckBox": {"background_color": 0x0, "margin": 0},
            "Menu.CheckBox::drag": {
                "image_url": StageIcons().get("drag"),
                "color": 0xFF505050,
                "alignment": ui.Alignment.CENTER,
            },
            "Menu.CheckBox.Image": {"image_url": StageIcons().get("check_off"), "color": 0xFF8A8777},
            "Menu.CheckBox.Image:checked": {"image_url": StageIcons().get("check_on")},
            "TreeView": {
                "background_color": 0xFF23211F,
                "background_selected_color": 0x664F4D43,
                "secondary_color": 0xFF403B3B,
            },
            "TreeView.ScrollingFrame": {"background_color": 0xFF23211F},
            "TreeView.Header": {"background_color": 0xFF343432, "color": 0xFFCCCCCC, "font_size": 12},
            "TreeView.Header::visibility_header": {"image_url": StageIcons().get("eye_header")},
            "TreeView.Image::object_icon_grey": {"color": 0x80FFFFFF},
            "TreeView.Image:disabled": {"color": 0x60FFFFFF},
            "TreeView.Item": {"color": 0xFF8A8777},
            "TreeView.Item:disabled": {"color": 0x608A8777},
            "TreeView.Item::object_name_grey": {"color": 0xFF4D4B42},
            "TreeView.Item::object_name_missing": {"color": 0xFF6F72FF},
            "TreeView.Item:selected": {"color": 0xFF23211F},
            "TreeView:selected": {"background_color": 0xFF8A8777},
            "TreeView:drop": {"background_color": 0xFF8A8777},
        }

    return style_data

def hasRequiredOVDVersion(metaDataPrim, minMajorVersion, minMinorVersion):
    isCompatible = False
    majorVersion = None
    minorVersion = None
    if metaDataPrim:
        versionMajorPath = "omni:pvd:ovdIntegrationVersionMajor"
        versionMinorPath = "omni:pvd:ovdIntegrationVersionMinor"
        if metaDataPrim.HasAttribute(versionMajorPath):
            majorVersion = metaDataPrim.GetAttribute(versionMajorPath).Get( Usd.TimeCode.EarliestTime() )
        if metaDataPrim.HasAttribute(versionMinorPath):
            minorVersion = metaDataPrim.GetAttribute(versionMinorPath).Get( Usd.TimeCode.EarliestTime() )
        if (majorVersion != None) and (minorVersion != None):
            if (majorVersion > minMajorVersion):
                isCompatible = True
            elif (majorVersion == minMajorVersion):
                if (minorVersion >= minMinorVersion):
                    isCompatible = True
    return isCompatible

def cleanup_fp_dialog(fp_dialog):
    async def cleanup_async(fp_dialog):
        await omni.kit.app.get_app().next_update_async()
        fp_dialog.destroy()
    asyncio.ensure_future(cleanup_async(fp_dialog))

def set_usd_cache_dir():
    def on_choose(fp_dialog, filename, dirpath):
        fp_dialog.hide()
        if (dirpath):
            if not dirpath.endswith("/"):
                dirpath += "/"
            carb.settings.get_settings().set(SETTING_OMNIPVD_USD_CACHE_DIRECTORY, dirpath)
        cleanup_fp_dialog(fp_dialog)

    item_filter_options = ["OVD Files (*.ovd)"]
    fp_dialog = FilePickerDialog(
        "Select the USD Cache Directory",
        apply_button_label="Select Current",
        click_apply_handler=lambda filename, dirname: on_choose(fp_dialog, filename, dirname),
        item_filter_options=item_filter_options,
    )
    fp_dialog.show()


def getStageUpAxisAsInt():
    if not omni.usd.get_context():
        return False, -1
    stage = omni.usd.get_context().get_stage()
    if not stage:
        return False, -1
    up_axis_from_stage = UsdGeom.GetStageUpAxis(stage)
    if (up_axis_from_stage == "Y"):
        return True, 0
    if (up_axis_from_stage == "Z"):
        return True, 1
    return False, -1

def _build_checkbox(*args, **kwargs):
    with ui.VStack():
        ui.Spacer()
        cb = ui.CheckBox(*args, **kwargs, height=0)
        ui.Spacer()
    return cb

class Item(ui.AbstractItem):
    def __init__(self, prim):
        super().__init__()
        self._pvd_node = OmniPvdNode(prim, self)

class OmniPvdNode():
    def __init__(self, prim, item):
        prim_path = str(prim.GetPath())
        self._prim_name = prim_path.split("/")[-1]
        self._node_name = prim_path
        self._all_children = None
        self._found_in_search = False
        self._item = item
        self._highlighted =  False

    def get_time(self):
        timeline = get_timeline_interface()
        return timeline.get_current_time() * timeline.get_time_codes_per_seconds()

    def _build_all_children(self, stage):
        if not self._all_children:
            if stage:
                ancestorPrim = stage.GetPrimAtPath(self._node_name)
                if ancestorPrim:
                    self._all_children = []
                    display_predicate = Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)
                    children_iterator = ancestorPrim.GetFilteredChildren(display_predicate)
                    for child_prim in children_iterator:
                        if child_prim.HasAttribute("omni:pvdi:class"):
                            self._all_children.append( Item(child_prim) )

    def get_child_item(self, prim_name, stage):
        if not self._all_children:
            self._build_all_children(stage)
        for child_item in self._all_children:
            # the pvd_node contains the "payload data"
            pvd_node = child_item._pvd_node
            if pvd_node._prim_name == prim_name:
                return child_item
        return None

    def get_current_visibile_children(self, stage):
        if not self._all_children:
            self._build_all_children(stage)
        visible_children = []
        if stage:
            for childItem in self._all_children:
                prim = stage.GetPrimAtPath(childItem._pvd_node._node_name)
                if prim:
                    # if the prim is below /shared, don't check for visibility
                    if not childItem._pvd_node._node_name.startswith('/Shared'):
                        visibility_attribute = prim.GetAttribute('visibility')
                        if not visibility_attribute:
                            continue
                        if visibility_attribute.Get(self.get_time()) != 'inherited':
                            continue
                    visible_children.append(childItem)
        return visible_children

class Model(ui.AbstractItemModel):
    def __init__(self):
        super().__init__()
        self.reset()

    def reset(self):
        self._pvd_node = None
        self._found_pvd_nodes = []

    def create_item_chain_and_expand(self, split_prim_names, stage, tree, is_for_search = True):
        if not self._pvd_node:
            self.get_item_children(None)
        if not self._pvd_node:
            return
        ancestor_pvd_node = self._pvd_node
        last_prim_idx = len(split_prim_names) - 1
        for idx, prim_name in enumerate(split_prim_names):
            child_item = ancestor_pvd_node.get_child_item(prim_name, stage)
            # if this is the last prim, set the child leaf node to found_in_search to be True
            if child_item:
                if idx == last_prim_idx:
                    if is_for_search:
                        child_item._pvd_node._found_in_search = True
                        self._found_pvd_nodes.append(child_item._pvd_node)
                    else:
                        self._expanded_select_items.append(child_item)
                    self._item_changed(child_item)
                else:
                    tree.set_expanded(child_item, True, False)
                ancestor_pvd_node = child_item._pvd_node
            else:
                # something went wrong, exit the loop
                return

    def get_item_children(self, item):
        visible_children = []
        if omni.usd.get_context():
            stage = omni.usd.get_context().get_stage()
            if stage:
                if item is not None:
                    visible_children = item._pvd_node.get_current_visibile_children(stage)
                else:
                    if not self._pvd_node:
                        rootPrim = stage.GetPrimAtPath("/")
                        self._pvd_node = OmniPvdNode(rootPrim, None)
                    visible_children = self._pvd_node.get_current_visibile_children(stage)
        return visible_children

    def get_item_value_model_count(self, item):
        return 2

    def get_item_value_model(self, item, column_id):
        return item.name_model

class OmniPVDNodeDelegate(ui.AbstractItemDelegate):
    def __init__(self):
        super().__init__()
        self._actortype_map = dict([
            ('eRIGID_STATIC', 'PxRigidStatic'),
            ('eRIGID_DYNAMIC', 'PxRigidDynamic'),
            ('eARTICULATION_LINK', 'PxArticulationLink'),
            ('eSOFTBODY', 'PxSoftBody'),
            ('eDEFORMABLE_SURFACE', 'PxDeformableSurface'),
            ('ePBD_PARTICLESYSTEM', 'PxPBDParticleSystem')
            ])

        self._jointtype_map = dict([
            ('eSPHERICAL', 'PxSphericalJoint'),
            ('eREVOLUTE', 'PxRevoluteJoint'),
            ('ePRISMATIC', 'PxPrismaticJoint'),
            ('eFIXED', 'PxFixedJoint'),
            ('eDISTANCE', 'PxDistanceJoint'),
            ('eD6', 'PxD6Joint'),
            ('eGEAR', 'PxGearJoint'),
            ('eRACK_AND_PINION', 'PxRackAndPinionJoint')
            ])

        self.reset()

    def reset(self):
        self._ctx = None
        self._stage = None
        self._tree = None
        self._search_string = ""
        self._current_idx = -1
        # Initialize reference navigation state if not already present
        if not hasattr(self, '_reference_history'):
            self._reference_history = []  # Stack of previous selections
            self._reference_history_index = -1  # Current position in history
            self._max_history_size = 50  # Maximum history entries

    def _set_stage_vars(self):
        if not omni.usd.get_context():
            self.reset()
            return
        self._ctx = omni.usd.get_context()
        self._stage = self._ctx.get_stage()

    def get_time(self):
        timeline = get_timeline_interface()
        return timeline.get_current_time() * timeline.get_time_codes_per_seconds()

    ################################################################################
    # OVD Tree Functions START
    ################################################################################
    def build_branch(self, model, item, column_idx, level, expanded):
        if column_idx!=0:
            return
        if item is not None:
            padding = "    " * level
            if model.get_item_children(item):
                with ui.HStack():
                    if model.can_item_have_children(item):
                        sign = "-" if expanded else "+"
                        if item._pvd_node._found_in_search:
                            ui.Label(f"{padding}{sign}  ", style_type_name_override="TreeView.Item", style={"color": ui.color.white})
                        else:
                            ui.Label(f"{padding}{sign}  ", style_type_name_override="TreeView.Item")
            else:
                with ui.HStack():
                    if item._pvd_node._found_in_search:
                        ui.Label(f"{padding}  ", style_type_name_override="TreeView.Item", style={"color": ui.color.white})
                    else:
                        ui.Label(f"{padding}  ", style_type_name_override="TreeView.Item")

    def build_widget(self, model, item, column_idx, level, expanded):
        if item is None:
            return
        if not self._stage:
            self._set_stage_vars()
        if not self._stage:
            return
        prim = self._stage.GetPrimAtPath(item._pvd_node._node_name)
        if not prim:
            return
        if column_idx == 0:
            if prim.HasAttribute("omni:pvd:name"):
                object_name_attr = prim.GetAttribute("omni:pvd:name")
                object_name = object_name_attr.Get(self.get_time())
            else:
                if prim.HasAttribute("omni:pvdi:name"):
                    object_name = prim.GetAttribute("omni:pvdi:name").Get()
                else:
                    object_name = item._pvd_node._prim_name
            if prim.HasAttribute("omni:pvdi:uid"):
                uid = prim.GetAttribute("omni:pvdi:uid").Get()
                if uid>0:
                    object_name = "(" + str(uid) + ")" + object_name
            if item._pvd_node._found_in_search:
                ui.Label(f"{object_name}", style_type_name_override="TreeView.Item", style={"color": ui.color.white})
            else:
                ui.Label(f"{object_name}", style_type_name_override="TreeView.Item")
        else:
            omniPvdClass = ""
            if prim.HasAttribute("omni:pvdi:class"):
                omniPvdClass = prim.GetAttribute("omni:pvdi:class").Get()
                if omniPvdClass.startswith("Px"):
                    if prim.HasAttribute("omni:pvd:type"):
                        object_type = prim.GetAttribute("omni:pvd:type").Get(self.get_time())
                        if object_type:
                            if omniPvdClass == "PxActor":
                                omniPvdClass = self._actortype_map[object_type]
                            elif omniPvdClass == "PxJoint":
                                omniPvdClass = self._jointtype_map[object_type]
                else:
                    omniPvdClass = ""
            if item._pvd_node._found_in_search:
                ui.Label(f"{omniPvdClass}", alignment = omni.ui.Alignment.LEFT,  style_type_name_override="TreeView.Item", style={"color": ui.color.white})
            else:
                ui.Label(f"{omniPvdClass}", alignment = omni.ui.Alignment.LEFT,  style_type_name_override="TreeView.Item")

    # For all prims that correspond to the search criteria, create the items for each prim (and its ancestors)
    # and expand up until including the found prims ancestor item
    def iterate_search_on_prim(self, prim, search_string, display_predicate):
        if prim:
            if prim.HasAttribute("omni:pvdi:class"):
                visibility_attribute = prim.GetAttribute('visibility')
                if not visibility_attribute:
                    return
                if visibility_attribute.Get(self.get_time()) != 'inherited':
                    return

                # This should rather be in a pre-allocated Item or PVDNode
                # the whole strategy on traversing the USD stage makes less and less sense
                # as these "hidden" costs, such as re-generation of the search name, start to amass
                if prim.HasAttribute("omni:pvd:name"):
                    object_name_attr = prim.GetAttribute("omni:pvd:name")
                    object_name = object_name_attr.Get(self.get_time())
                else:
                    if prim.HasAttribute("omni:pvdi:name"):
                        object_name = prim.GetAttribute("omni:pvdi:name").Get()
                    else:
                        object_name = prim.GetName()
                if prim.HasAttribute("omni:pvdi:uid"):
                    uid = prim.GetAttribute("omni:pvdi:uid").Get()
                    if uid>0:
                        object_name = object_name + "(" + str(uid) + ")"

                # The test below should be on the OmniPVD object name, which can be any string
                if search_string in object_name:
                    path_str = prim.GetPath().pathString
                    split_prim_names = path_str.split("/")
                    # filter out the empty elements : typically the first one "/World/box" -> ["", "World", "box"]
                    split_prim_names = list(filter(None, split_prim_names))
                    self._model.create_item_chain_and_expand(split_prim_names, self._stage, self._tree)
                # Also check the children
                children_iterator = prim.GetFilteredChildren(display_predicate)
                for child_prim in children_iterator:
                    self.iterate_search_on_prim(child_prim, search_string, display_predicate)

    def reset_nodes(self, pvd_node):
        if not pvd_node:
            return
        if pvd_node._found_in_search:
            pvd_node._found_in_search = False
            pvd_node._highlighted = False
            self._model._item_changed(pvd_node._item)
        if pvd_node._all_children:
            for item in pvd_node._all_children:
                pvd_node_of_item = item._pvd_node
                self.reset_nodes(pvd_node_of_item)

    def on_search(self, model):
        self._search_string = f"{model.as_string}"
        self._set_stage_vars()
        self._current_idx = -1
        if not self._ctx:
            return
        self.reset_nodes(self._model._pvd_node)
        if not self._search_string.strip():
            self._model._found_pvd_nodes = []
            return
        if not self._stage:
            return
        self._model._found_pvd_nodes = []
        rootPrim = self._stage.GetPrimAtPath("/")
        display_predicate = Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)
        if rootPrim:
            children_iterator = rootPrim.GetFilteredChildren(display_predicate)
            for child_prim in children_iterator:
                self.iterate_search_on_prim(child_prim, self._search_string, display_predicate)
        if len(self._model._found_pvd_nodes)>0:
            self._current_idx = 0
            self.on_selection_of_item([self._model._found_pvd_nodes[self._current_idx]._item])
        else:
            self._current_idx = -1
        self._model._item_changed(None)

    ################################################################################
    # on_selection_of_item gets called when something is selected in either the USD Stage
    # view or the OVD Tree view
    ################################################################################
    def on_selection_of_item(self, items):
        if not self:
            return
        ctx = omni.usd.get_context()
        if not ctx:
            return
        stage = ctx.get_stage()
        if not stage:
            return
        if not carb.settings.get_settings().get_as_bool(SETTING_OMNIPVD_IS_OVD_STAGE):
            return
        selected_paths = []
        for item in items:
            selected_paths.append(item._pvd_node._node_name)
        if (len(selected_paths) > 0):
            self._set_stage_vars()
            if self._window.focused:
                self._ctx.get_selection().set_selected_prim_paths(selected_paths, True)
            self._tree.selection = items
        else:
            if hasattr(self, '_ctx'):
                if not self._ctx:
                    return
            #self._set_stage_vars()
            if self._window.focused:
                self._ctx.get_selection().set_selected_prim_paths(selected_paths, True)
            self._tree.selection = []

    def _iter_next_phase_1(self):
        if self._current_idx == -1:
            return False
        self._model._found_pvd_nodes[self._current_idx]._highlighted = False
        item = self._model._found_pvd_nodes[self._current_idx]._item
        self._model._item_changed(item)
        return True

    def _iter_next_phase_2(self):
        self._model._found_pvd_nodes[self._current_idx]._highlighted = True
        item = self._model._found_pvd_nodes[self._current_idx]._item
        self._model._item_changed(item)
        items_selected = [item]
        self.on_selection_of_item(items_selected)

    def _on_prev_button_pressed(self):
        if self._iter_next_phase_1():
            self._current_idx = self._current_idx - 1
            if self._current_idx < 0:
                self._current_idx = len(self._model._found_pvd_nodes)-1
            self._iter_next_phase_2()

    def _on_next_button_pressed(self):
        if self._iter_next_phase_1():
            self._current_idx = self._current_idx + 1
            if self._current_idx > (len(self._model._found_pvd_nodes)-1):
                self._current_idx = 0
            self._iter_next_phase_2()

    def _on_collapse_all_button_pressed(self):
        self._set_stage_vars()
        if not self._ctx:
            return
        self.reset_nodes(self._model._pvd_node)

        root_node = self._model._pvd_node
        if not root_node:
            return
        for item in root_node._all_children:
            self._tree.set_expanded(item, False, True)
    
    def _on_ref_prev_button_pressed(self):
        """Navigate to previous reference in history."""
        try:
            from omni.physxpvd.scripts.property_widget.usd_helpers import navigate_reference_history_prev
            navigate_reference_history_prev()
        except Exception:
            pass
    
    def _on_ref_next_button_pressed(self):
        """Navigate to next reference in history."""
        try:
            from omni.physxpvd.scripts.property_widget.usd_helpers import navigate_reference_history_next
            navigate_reference_history_next()
        except Exception:
            pass
    
    def _set_selection_without_history(self, prim_path):
        """Set selection without adding to history (for navigation)."""
        if self._ctx:
            self._ctx.get_selection().set_selected_prim_paths([prim_path], True)
    
    def _update_reference_navigation_buttons(self):
        """Update the enabled state of reference navigation buttons."""
        try:
            from omni.physxpvd.scripts.property_widget.usd_helpers import get_global_navigation_button_states
            prev_enabled, next_enabled = get_global_navigation_button_states()
            
            if hasattr(self, '_ref_prev_button'):
                self._ref_prev_button.enabled = prev_enabled
                
            if hasattr(self, '_ref_next_button'):
                self._ref_next_button.enabled = next_enabled
        except Exception:
            pass
    
    def add_to_reference_history(self, prim_path):
        """Add a prim path to reference navigation history."""
        # If we're not at the end of history, remove everything after current position
        if self._reference_history_index < len(self._reference_history) - 1:
            self._reference_history = self._reference_history[:self._reference_history_index + 1]
        
        # Add new path if it's different from the current one
        if not self._reference_history or self._reference_history[-1] != prim_path:
            self._reference_history.append(prim_path)
            self._reference_history_index = len(self._reference_history) - 1
            
            # Limit history size
            if len(self._reference_history) > self._max_history_size:
                self._reference_history.pop(0)
                self._reference_history_index = len(self._reference_history) - 1
        
        # Update button states
        self._update_reference_navigation_buttons()

    def build_header(self, column_idx):
        with ui.VStack():
            with ui.HStack():
                if column_idx == 0:
                    ui.Label("Name", tooltip="Header")
                else:
                    ui.Label("Type", tooltip="Header", alignment = omni.ui.Alignment.LEFT, height=0)
    ################################################################################
    # OVD Tree Functions STOP
    ################################################################################


################################################################################
# OVD Tree START
################################################################################
class OmniPVdObjectTreeWindow(ui.Window):
    def __del__(self):
        # Unregister delegate when window is destroyed
        try:
            from omni.physxpvd.scripts.property_widget.usd_helpers import unregister_reference_navigation_delegate
            unregister_reference_navigation_delegate()
        except Exception as e:
            print(f"[DEBUG] Failed to unregister navigation delegate: {e}")
    
    def get_time(self):
        timeline = get_timeline_interface()
        return timeline.get_current_time() * timeline.get_time_codes_per_seconds()

    def mybuild(self):
        with self.frame:
            if not hasattr(self, '_style'):
                self._style = carb.settings.get_settings().get_as_string("/persistent/app/window/uiStyle") or "NvidiaDark"
                self._style_data = set_style(self._style)
                self.frame.set_style(self._style_data)

            if not hasattr(self, '_model'):
                self._model = Model()
            else:
                self._model.reset()

            if not hasattr(self, '_delegate'):
                self._delegate = OmniPVDNodeDelegate()
            else:
                self._delegate.reset()

            ################################################################################
            # Window handle is needed to know if the OVD Tree window is in focuse from the
            # on_selection event handled by the delegate
            ################################################################################
            self._delegate._window = self
            
            # Register delegate for reference navigation
            try:
                from omni.physxpvd.scripts.property_widget.usd_helpers import register_reference_navigation_delegate
                register_reference_navigation_delegate(self._delegate)
            except Exception as e:
                print(f"[DEBUG] Failed to register navigation delegate: {e}")

            with ui.VStack():
                with ui.HStack(height=0):
                    ui.Label("Search", tooltip="Search", width=80)
                    sf_filter = ui.StringField()
                    sf_filter.model.add_end_edit_fn(self._delegate.on_search)
                    sf_filter.model.set_value(self._delegate._search_string)
                with ui.HStack(height=0):
                    self._delegate._collapse_all_button = ui.Button("Collapse All", width=20, height=0)
                    self._delegate._collapse_all_button.set_clicked_fn(self._delegate._on_collapse_all_button_pressed)

                    self._delegate._prev_button = ui.Button("Previous", width=20, height=0)
                    self._delegate._prev_button.set_clicked_fn(self._delegate._on_prev_button_pressed)

                    self._delegate._next_button = ui.Button("Next", width=20, height=0)
                    self._delegate._next_button.set_clicked_fn(self._delegate._on_next_button_pressed)
                    
                    # Reference navigation buttons
                    self._delegate._ref_prev_button = ui.Button("<", width=25, height=0, tooltip="Previous reference")
                    self._delegate._ref_prev_button.set_clicked_fn(self._delegate._on_ref_prev_button_pressed)
                    self._delegate._ref_prev_button.enabled = False
                    
                    self._delegate._ref_next_button = ui.Button(">", width=25, height=0, tooltip="Next reference")
                    self._delegate._ref_next_button.set_clicked_fn(self._delegate._on_ref_next_button_pressed)
                    self._delegate._ref_next_button.enabled = False
                with ui.ScrollingFrame(
                    horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF,
                    vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                    style_type_name_override="TreeView",
                ):
                    self._tree = ui.TreeView(self._model, delegate = self._delegate, root_visible = False, header_visible = True, style={"margin": 0.5},  columns_resizable = True, style_type_name_override="TreeView", column_widths=[ui.Fraction(3.5), ui.Fraction(1)])
                    self._model._tree = self._tree
                    self._delegate._model = self._model
                    self._delegate._tree = self._tree
                    self._tree.set_selection_changed_fn(self._delegate.on_selection_of_item)

    def _setVisibleChildren(self, prim):
        if not prim:
            return
        if prim.HasAttribute("omni:pvdi:viz"):
            if not prim.GetAttribute("omni:pvdi:viz").Get(self.cached_time):
                UsdGeom.Imageable(prim).MakeInvisible()
            else:
                UsdGeom.Imageable(prim).MakeVisible()
                children_iterator = prim.GetChildren()
                for child_prim in children_iterator:
                    self._setVisibleChildren(child_prim)
        else:
            children_iterator = prim.GetChildren()
            for child_prim in children_iterator:
                self._setVisibleChildren(child_prim)

    def _updatedFromEvent(self):
        if not hasattr(self, "_model"):
            return
        if not carb.settings.get_settings().get_as_bool(SETTING_OMNIPVD_IS_OVD_STAGE):
            return
        usd_context = omni.usd.get_context()
        if not usd_context:
            return
        stage = usd_context.get_stage()
        if stage:
            stage.SetInterpolationType(Usd.InterpolationTypeHeld)
            self.cached_time = self.get_time()
            with Sdf.ChangeBlock():
                self._setVisibleChildren(stage.GetPrimAtPath("/Scenes"))
                self._setVisibleChildren(stage.GetPrimAtPath("/Shared"))
            self._model._expanded_select_items = []
            selected_prim_paths = usd_context.get_selection().get_selected_prim_paths()
            for path in selected_prim_paths:
                split_prim_names = path.split("/")
                split_prim_names = list(filter(None, split_prim_names))
                self._model.create_item_chain_and_expand(split_prim_names, stage, self._tree, False)
            self._tree.selection = self._model._expanded_select_items
        self._model._item_changed(None)

    def _on_timeline_event(self, e):
        current_time = e.payload["currentTime"]
        if current_time != self._current_time:
            self._current_time = current_time
            self._updatedFromEvent()

    def __init__(self):
        super().__init__("OVD Tree", ui.DockPreference.RIGHT_TOP, width=640, height=480)
        self.deferred_dock_in("Stage", ui.DockPolicy.DO_NOTHING)
        self._usd_context = omni.usd.get_context()

        self.frame.set_build_fn(self.mybuild)
        self.frame.rebuild()

        @carb.profiler.profile
        def on_stage_selection_changed():
            if (not self._usd_context):
                return
            if (not self._usd_context.get_stage()):
                return
            if hasattr(self, '_model'):
                self._model._expanded_select_items = []
                if carb.settings.get_settings().get_as_bool(SETTING_OMNIPVD_IS_OVD_STAGE):
                    stage = self._usd_context.get_stage()
                    if stage:
                        selected_prim_paths = self._usd_context.get_selection().get_selected_prim_paths()
                        for path in selected_prim_paths:
                            split_prim_names = path.split("/")
                            split_prim_names = list(filter(None, split_prim_names))
                            self._model.create_item_chain_and_expand(split_prim_names, stage, self._tree, False)
                self._tree.selection = self._model._expanded_select_items

        @carb.profiler.profile
        def on_stage_opened():
            self.frame.rebuild()
            self._updatedFromEvent()

        self._stage_event_sub = [
            get_eventdispatcher().observe_event(
                observer_name="omni.physx.pvd:OmniPVdObjectTreeWindow",
                event_name=self._usd_context.stage_event_name(event),
                on_event=func
            )
            for event, func in (
                (omni.usd.StageEventType.OPENED, lambda _: on_stage_opened()),
                (omni.usd.StageEventType.SELECTION_CHANGED, lambda _: on_stage_selection_changed()),
            )
        ]

        timeline = omni.timeline.get_timeline_interface()
        self._current_time = timeline.get_current_time()
        stream = timeline.get_timeline_event_stream()
        self._timeline_subscription = stream.create_subscription_to_pop(self._on_timeline_event)


    def on_shutdown(self):
        self._stage_event_sub = None
        self._timeline_subscription = None

    def traverse_stage(self):
        return
################################################################################
# OVD Tree STOP
################################################################################

def build_section(name, build_func):
    with ui.CollapsableFrame(name, height=0):
        with ui.HStack():
            ui.Spacer(width=20)
            with ui.VStack():
                build_func()

class MessageItem(ui.AbstractItem):
    def __init__(self, message):
        super().__init__()
        self._message = message

class MessagesModel(ui.AbstractItemModel):
    def __init__(self, import_ovd_fn):
        super().__init__()
        self._import_ovd_fn = import_ovd_fn
        self._messageItems = []

    def reset(self):
        self._import_ovd_fn(True)

        physxPvdInterface = _physxPvd.acquire_physx_pvd_interface()
        messages = physxPvdInterface.get_messages()

        for message in messages:
            self._messageItems.append(MessageItem(message))

    def clear(self):
        self._messageItems.clear()

    def get_item_children(self, item):
        if item is not None:
            return item.children

        return self._messageItems

    def get_item_value_model_count(self, item):
        return 2

    def get_item_value_model(self, item, column_id):
        return item._message

class OmniPVDMessageDelegate(ui.AbstractItemDelegate):
    def __init__(self):
        super().__init__()
        self.reset()

    def reset(self):
        self._frameId = None
        self._detail_message_field = None
        self._detail_type_field = None
        self._detail_file_field = None
        self._detail_path_field = None
        self._detail_line_field = None
        self._detail_frame_button = None

    def build_branch(self, model, item, column_idx, level, expanded):
        return

    def build_widget(self, model, item, column_idx, level, expanded):
        if item is None:
            return

        text = ""

        if item is not None:
            if column_idx == 0:
                text = item._message["message"]
            elif column_idx == 1:
                text = item._message["typeName"]
                if text == "":
                    text = item._message["type"]

        ui.Label(f"{text}", alignment=omni.ui.Alignment.LEFT, style_type_name_override="TreeView.Item")

    def on_selection_of_item(self, items):
        if not self:
            return

        if len(items) > 0:
            item = items[0]
            type = item._message["typeName"]

            if type == "":
                type = item._message["type"]

            fileString = str(item._message["file"])
            index = fileString.rfind('\\')

            if index > 0:
                file = fileString[index + 1:]
                path = fileString[0:index - 1]
            else:
                file = fileString
                path = fileString

            self._frameId = item._message["frameId"]

            self._detail_message_field.model.set_value(item._message["message"])
            self._detail_type_field.model.set_value(type)
            self._detail_file_field.model.set_value(file)
            self._detail_path_field.model.set_value(path)
            self._detail_line_field.model.set_value(item._message["line"])
            self._detail_frame_button.text = str(self._frameId)

    def build_header(self, column_idx):
        with ui.VStack():
            with ui.HStack():
                if column_idx == 0:
                    ui.Label("Message", tooltip="The message text")
                else:
                    ui.Label("Type", tooltip="The type of message", alignment=omni.ui.Alignment.LEFT, height=0)

    def set_time(self):
        if self._frameId is not None:
            timeline = get_timeline_interface()
            timeline.set_current_time(self._frameId / timeline.get_time_codes_per_seconds())

################################################################################
# The OmniPVD Messages Window
################################################################################
class OmniPvdMessagesWindow(ui.Window):
    def set_import_fn(self, import_ovd_fn):
        self._import_ovd_fn = import_ovd_fn

    def mybuild(self):
        with self.frame:
            if not hasattr(self, '_style'):
                self._style = carb.settings.get_settings().get_as_string("/persistent/app/window/uiStyle") or "NvidiaDark"
                self._style_data = set_style(self._style)
                self.frame.set_style(self._style_data)

            if not hasattr(self, '_model'):
                self._model = MessagesModel(self._import_ovd_fn)
            else:
                self._model.reset()

            if not hasattr(self, '_delegate'):
                self._delegate = OmniPVDMessageDelegate()
            else:
                self._delegate.reset()

            ################################################################################
            # Window handle is needed to know if the OVD Tree window is in focus from the
            # on_selection event handled by the delegate
            ################################################################################
            self._delegate._window = self

            with ui.HStack():
                with ui.ScrollingFrame(
                    horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF,
                    vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                    style_type_name_override="TreeView",
                ):
                    self._tree = ui.TreeView(self._model, delegate=self._delegate, root_visible=False, header_visible=True, style={"margin": 0.5},  columns_resizable=True, style_type_name_override="TreeView", column_widths=[ui.Fraction(3.0), ui.Fraction(1.5)])
                    self._model._tree = self._tree
                    self._delegate._model = self._model
                    self._delegate._tree = self._tree
                    self._tree.set_selection_changed_fn(self._delegate.on_selection_of_item)
                with ui.ScrollingFrame(
                    horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF,
                    vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED):

                    with ui.VStack(height=20):
                        ui.Spacer(height=5)
                        with ui.HStack():
                            ui.Spacer(width=10)
                            ui.Label("Message", height=20, width=100)
                            self._delegate._detail_message_field = ui.StringField(height=40, multiline=True)
                            self._delegate._detail_message_field.model.set_value("")
                        ui.Spacer(height=5)
                        with ui.HStack():
                            ui.Spacer(width=10)
                            ui.Label("Type", height=20, width=100)
                            self._delegate._detail_type_field = ui.StringField(height=20)
                            self._delegate._detail_type_field.model.set_value("")
                        ui.Spacer(height=5)
                        with ui.HStack():
                            ui.Spacer(width=10)
                            ui.Label("File", height=20, width=100)
                            self._delegate._detail_file_field = ui.StringField(height=20)
                            self._delegate._detail_file_field.model.set_value("")
                        ui.Spacer(height=5)
                        with ui.HStack():
                            ui.Spacer(width=10)
                            ui.Label("Path", height=20, width=100)
                            self._delegate._detail_path_field = ui.StringField(height=20)
                            self._delegate._detail_path_field.model.set_value("")
                        ui.Spacer(height=5)
                        with ui.HStack():
                            ui.Spacer(width=10)
                            ui.Label("Line", height=20, width=100)
                            self._delegate._detail_line_field = ui.IntField(height=20)
                            self._delegate._detail_line_field.model.set_value(0)
                        ui.Spacer(height=5)
                        with ui.HStack():
                            ui.Spacer(width=10)
                            ui.Label("Frame", height=20, width=100)
                            self._delegate._detail_frame_button = ui.Button("Frame", width=100, height=20)
                            self._delegate._detail_frame_button.set_clicked_fn(self._delegate.set_time)

    def __init__(self):
        super().__init__("OVD Messages", ui.DockPreference.LEFT_BOTTOM, width=640, height=480)
        self.deferred_dock_in("Content", ui.DockPolicy.DO_NOTHING)
        self._usd_context = omni.usd.get_context()

        self.frame.set_build_fn(self.mybuild)
        self.frame.rebuild()

        @carb.profiler.profile
        def on_stage_event_closed():
            physxPvdInterface = _physxPvd.acquire_physx_pvd_interface()
            physxPvdInterface.clear_messages()
            if hasattr(self, '_model'):
                if self._model is not None:
                    self._model.clear()

        self._stage_event_sub = [
            get_eventdispatcher().observe_event(
                observer_name="omni.physx.pvd:OmniPvdMessagesWindow",
                event_name=self._usd_context.stage_event_name(event),
                on_event=func
            )
            for event, func in (
                (omni.usd.StageEventType.OPENED, lambda _: self.frame.rebuild()),
                (omni.usd.StageEventType.CLOSED, lambda _: on_stage_event_closed()),
            )
        ]

    def on_shutdown(self):
        self._stage_event_sub = None

    def traverse_stage(self):
        return

################################################################################
# OVD File->Import dialogue START
################################################################################
class OVDImporter(ai.AbstractImporterDelegate):
    def __init__(self, ovd_importer) -> None:
        super().__init__()
        self._name = "OVD OmniPVD Importer"
        self._filters = [".*\\.ovd$"]
        self._descriptions = ["OVD Files (*.ovd)"]
        self._default_color = [200, 200, 200]
        # used for icon paths
        self._ovdImporter = ovd_importer

    def destroy(self):
        self._ovdImporter = None
        return

    @property
    def name(self) -> str:
        return self._name

    @property
    def filter_regexes(self) -> List[str]:
        return self._filters

    @property
    def filter_descriptions(self) -> List[str]:
        return self._descriptions

    def build_options(self, paths: List[str]) -> None:
        return True

    async def convert_assets(self, paths: List[str]) -> Dict[str, Union[str, None]]:
        converted_assets = {}
        full_path = paths[0]
        file_name = os.path.basename(full_path)
        file_dir = os.path.dirname(full_path)
        file_dir += "/"
        self._ovdImporter._import_ovd_file_from_path(file_name, file_dir)
        return converted_assets

################################################################################
# OVD File->Import dialogue STOP
################################################################################

class PhysxPvdExtension(omni.ext.IExt):
    instance = None
    icons_folder = "data/icons"

    def __init__(self):
        self._propertyWidgetOmniPvd = None
        self._ovdLoaded = False
        self._timelineSub = None
        self._lastFrameId = 0
        self._sliderIsActive = False
        self._isLoadingOVD = False
        self._isTimelineFrameTypeOverridden = False # used to temporarily switch from Pre to Post frame types without changing the slider position
        self._ext_path = ""
        self._icons_path = ""
        self._frameModeWidget = None
        self._stage_event_sub_ovd = None
        self._settings_subs = []
        self._omniPvdEnabledCheckbox = None
        super().__init__()

    def modTime(self, fileName):
        concatPath = self._dirPath + fileName
        return os.path.getmtime(concatPath)

    def _getLatestFileByModificationTime(self, directory):
        def is_valid_file(filename):
            return filename != 'tmp.ovd' and filename.endswith('.ovd')

        self._dirPath = directory
        filePath = None
        if not os.path.exists(directory):
            return filePath
        try:
            my_list = next(os.walk(directory))[2]
            if my_list:
                my_list = list(filter(is_valid_file, my_list))
                if my_list:
                    filePath = max(my_list, key=self.modTime)
        except StopIteration:
            pass
        return filePath

    def _printDebugInfoInit(self):
        startFrameId = carb.settings.get_settings().get_as_int(SETTING_OMNIPVD_TIMELINE_FRAME_ID_MIN)
        stopFrameId = carb.settings.get_settings().get_as_int(SETTING_OMNIPVD_TIMELINE_FRAME_ID_MAX)
        startSimStepId = carb.settings.get_settings().get_as_int(SETTING_OMNIPVD_TIMELINE_SIM_STEP_ID_MIN)
        stopSimStepId = carb.settings.get_settings().get_as_int(SETTING_OMNIPVD_TIMELINE_SIM_STEP_ID_MAX)
        frameMode = carb.settings.get_settings().get_as_int(SETTING_OMNIPVD_TIMELINE_FRAME_MODE)

        stage = omni.usd.get_context().get_stage()
        print(f"startTimeCode{int(stage.GetStartTimeCode())}")
        print(f"stopTimeCode{int(stage.GetEndTimeCode())}")
        print(f"startFrameId{startFrameId}")
        print(f"stopFrameId{stopFrameId}")
        print(f"startSimStepId{startSimStepId}")
        print(f"stopSimStepId{stopSimStepId}")
        print(f"frameMode{frameMode}")

    def _build_omnipvd_ui(self):
        def define_recording_dir():
            def on_choose(fp_dialog, filename, dirpath):
                fp_dialog.hide()
                if (dirpath):
                    if not dirpath.endswith("/"):
                        dirpath += "/"
                    # Sets the OVD recording directory for Omni Physics simulations
                    carb.settings.get_settings().set(SETTING_OMNIPVD_OVD_RECORDING_DIRECTORY, dirpath)
                cleanup_fp_dialog(fp_dialog)

            item_filter_options = ["OVD Files (*.ovd)"]
            fp_dialog = FilePickerDialog(
                "Select the OVD Recording Directory",
                apply_button_label="Select Current",
                click_apply_handler=lambda filename, dirname: on_choose(fp_dialog, filename, dirname),
                item_filter_options=item_filter_options,
            )
            fp_dialog.show()

        # Set the default OVD recording directory lazily
        ovdRecordingDir = carb.settings.get_settings().get(SETTING_OMNIPVD_OVD_RECORDING_DIRECTORY)
        if not ovdRecordingDir:
            ovdRecordingDir = carb.tokens.get_tokens_interface().resolve("${omni_documents}") + "/omni_pvd/out/"
            if not os.path.exists(ovdRecordingDir):
                os.makedirs(ovdRecordingDir)
            carb.settings.get_settings().set(SETTING_OMNIPVD_OVD_RECORDING_DIRECTORY, ovdRecordingDir)

        omnipvd_settings = [
            [SettingType.BOOL, "Recording Enabled", SETTING_OMNIPVD_ENABLED]
        ]

        def build_inner():
            for setting in omnipvd_settings:
                with ui.HStack(height=23):
                    ui.Spacer(width=5)
                    ui.Label(setting[1], width=160)
                    if setting[0] == SettingType.BOOL:
                        cb = _build_checkbox(SettingModel(setting[2]))
                        if setting[2] == SETTING_OMNIPVD_ENABLED:
                            self._omniPvdEnabledCheckbox = cb
                    elif setting[0] == SettingType.STRING:
                        widget = ui.StringField(SettingModel(setting[2]), height=20)
            with ui.HStack(height=23):
                button = ui.Button("Set Recording Directory", height=30, width=160)
                button.set_clicked_fn(define_recording_dir)
                ui.Spacer(width=5)
                with ui.VStack(height=23):
                    ui.Spacer(height=4)
                    widget = ui.StringField(SettingModel(SETTING_OMNIPVD_OVD_RECORDING_DIRECTORY), height=22, enabled = False)
                    widget.enabled = False
                    widget.set_style(DISABLED_STYLE)
        build_inner()

        # Disallow switching omniPVD on and off while sim is running or the stage is an OmniPVD stage
        def on_simulation_event_omnipvd_enable(event):
            if event.type == int(SimulationEvent.RESUMED):
                self._omniPvdEnabledCheckbox.enabled = False
            elif event.type == int(SimulationEvent.STOPPED):
                if not carb.settings.get_settings().get_as_bool(SETTING_OMNIPVD_IS_OVD_STAGE):
                    self._omniPvdEnabledCheckbox.enabled = True

        def on_stage_event_omnipvd_enable():
            self._ovdLoaded = False

            # check the stage if OmniPVD stage?
            is_OVD_stage = False
            ctx = omni.usd.get_context()
            if ctx:
                stage = ctx.get_stage()
                if stage:
                    prim = stage.GetPrimAtPath("/scenes")
                    if not prim:
                        prim = stage.GetPrimAtPath("/shared")
                    if not prim:
                        prim = stage.GetPrimAtPath("/Scenes")
                    if not prim:
                        prim = stage.GetPrimAtPath("/Shared")
                    if prim:
                        if prim.HasAttribute("omni:pvdi:class"):
                            is_OVD_stage = True
            if not is_OVD_stage:
                self._omniPvdEnabledCheckbox.enabled = True
                carb.settings.get_settings().set(SETTING_OMNIPVD_IS_OVD_STAGE, False)
            else:
                self._isLoadingOVD = True

                self._omniPvdEnabledCheckbox.enabled = False
                carb.settings.get_settings().set(SETTING_OMNIPVD_IS_OVD_STAGE, True)

                carb.settings.get_settings().set_int(SETTING_OMNIPVD_TIMELINE_PLAYBACK_STATE, OvdTimelinePlaybackState.ePaused)

                if hasRequiredOVDVersion(stage.GetPrimAtPath("/Shared/PxOmniPvdMetaData/PxOmniPvdMetaData_1"), 1, 6):
                    framesPerSimStep = 2
                    carb.settings.get_settings().set_bool(SETTING_OMNIPVD_TIMELINE_IS_LEGACY_OVD, False)
                else:
                    framesPerSimStep = 1
                    carb.settings.get_settings().set_bool(SETTING_OMNIPVD_TIMELINE_IS_LEGACY_OVD, True)

                carb.settings.get_settings().set_int(SETTING_OMNIPVD_TIMELINE_FRAMES_PER_SIM_STEP, framesPerSimStep)

                # set the frame mode to post sim as default one
                carb.settings.get_settings().set_int(SETTING_OMNIPVD_TIMELINE_FRAME_MODE, OvdTimelineFrameMode.ePostSim)
                if framesPerSimStep == 1:
                    # In case of frame per sim step being one because legacy file, only allow the "legacy" frame mode
                    # also disable the dropdown
                    self._frameModeWidget.enabled = False
                    self._frameModeWidget.set_style(DISABLED_STYLE)
                else:
                    self._frameModeWidget.enabled = True
                    self._frameModeWidget.set_style(ENABLED_STYLE)

                startFrameId = int(stage.GetStartTimeCode())
                # make sure the first frameId is 1
                startFrameId = 1
                stopFrameId = int(stage.GetEndTimeCode())
                ################################################################################
                # If it's a "new" OVD also check that the last recorded frame is a postSim frame,
                # otherwise padd it to make sure the last frameId is even : stopping on a post sim frame
                ################################################################################
                if framesPerSimStep == 2:
                    if self._frameIsPreSim(stopFrameId):
                        stopFrameId += 1
                carb.settings.get_settings().set_int(SETTING_OMNIPVD_TIMELINE_FRAME_ID_MIN, startFrameId)
                carb.settings.get_settings().set_int(SETTING_OMNIPVD_TIMELINE_FRAME_ID_MAX, stopFrameId)

                startSimStepId = 1
                nbrSimSteps = stopFrameId - startSimStepId + 1
                stopSimStepId = int(nbrSimSteps / framesPerSimStep)
                carb.settings.get_settings().set_int(SETTING_OMNIPVD_TIMELINE_SIM_STEP_ID_MIN, startSimStepId)
                carb.settings.get_settings().set_int(SETTING_OMNIPVD_TIMELINE_SIM_STEP_ID_MAX, stopSimStepId)

                ################################################################################
                # Figure out if the frame has to be adjusted because of the frame mode
                # make into a function
                ################################################################################
                frameId = startFrameId
                if framesPerSimStep == 2:
                    frameId += 1 # make it post-sim as default is to be the first frame
                simStepId = self._ovd_timeline_frame_to_step(frameId)

                self._updateFrameId(frameId)
                self._ovd_timeline_update_slider()
                carb.settings.get_settings().set_int(SETTING_OMNIPVD_TIMELINE_SIM_STEP_ID, simStepId)
                self._isLoadingOVD = False
                #self._printDebugInfoInit()

        self._omniPvdEnabledCheckbox.enabled = not get_physx_interface().is_running()

        physxInterface = get_physx_interface()
        self._omniPvdEventSubcription = physxInterface.get_simulation_event_stream_v2().create_subscription_to_pop(
            on_simulation_event_omnipvd_enable
        )

        usd_context = omni.usd.get_context()
        self._stage_event_sub_ovd = get_eventdispatcher().observe_event(
            observer_name="omni.physx.pvd:PhysxPvdExtension",
            event_name=usd_context.stage_event_name(omni.usd.StageEventType.OPENED),
            on_event=lambda _: on_stage_event_omnipvd_enable()
        )

    def _getLatestDirByDirName(self, directory):
        maxInt = 0
        try:
            my_list = next(os.walk(directory))[1]
            if my_list:
                filtered_list = list(filter(lambda x:x.isnumeric(), my_list))
                if filtered_list:
                    filtered_ints = map(lambda x:int(x), filtered_list)
                    if filtered_ints:
                        maxInt = max(filtered_ints)
        except StopIteration:
            maxInt = 0
        return maxInt + 1

    # Scan the directory for all directories that consist of a number and
    # keep the largest number. Put the new directory to be created to be +1 of that largest
    # number.
    def _getNextIntFromDirectories(self, directory):
        maxInt = 0
        try:
            my_list = next(os.walk(directory))[1]
            if my_list:
                filtered_list = list(filter(lambda x:x.isnumeric(), my_list))
                if filtered_list:
                    filtered_ints = map(lambda x:int(x), filtered_list)
                    if filtered_ints:
                        maxInt = max(filtered_ints)
        except StopIteration:
            maxInt = 0
        return maxInt + 1

    def _open_latest_stage(self, outputBasePath):
        lastInt = self._getNextIntFromDirectories(outputBasePath) - 1
        if (lastInt>0):
            outputStagePath = outputBasePath + str(lastInt) + "/";
            # now check if the stage is usd or usda
            stageFile = outputStagePath + "stage.usda"
            if not os.path.exists(stageFile):
                stageFile = outputStagePath + "stage.usd"
            if os.path.exists(stageFile):
                omni.usd.get_context().open_stage(stageFile)

    def _button_stage_traversal(self):
        toPhysxUSDConverter = ConvertOmniPvdToPhysXUSD()
        outputBasePath = carb.settings.get_settings().get(SETTING_OMNIPVD_PHYSX_USD_DIRECTORY)
        nextInt = self._getNextIntFromDirectories(outputBasePath)
        outputStagePath = outputBasePath + str(nextInt);
        if (self._omniPvdUSDType == 0):
            outputStagePath += "/stage.usd"
        else:
            outputStagePath += "/stage.usda"
        toPhysxUSDConverter.convert(outputStagePath, False)
        toPhysxUSDConverter = None

    def _button_open_latest_usd_stage(self):
        outputBasePath = carb.settings.get_settings().get(SETTING_OMNIPVD_USD_CACHE_DIRECTORY)
        self._open_latest_stage(outputBasePath)

    def _button_open_latest_physx_stage(self):
        outputBasePath = carb.settings.get_settings().get(SETTING_OMNIPVD_PHYSX_USD_DIRECTORY)
        self._open_latest_stage(outputBasePath)

    def _getUSDCacheDirName(self, dirPath, fileName):
        mtime = os.path.getmtime(dirPath + fileName)
        mtimeString = f"{mtime}"
        mtimeString = mtimeString.replace(".","_")
        cacheDirName = mtimeString + "_" + fileName.replace(".","_")
        return cacheDirName

    def _open_cached_stage(self, stageFile):
        if os.path.exists(stageFile):
            omni.usd.get_context().open_stage(stageFile)
            stage = omni.usd.get_context().get_stage()
            if not stage:
                return False

            domeLight = UsdLux.DomeLight.Define(stage, Sdf.Path("/DomeLight"))
            domeLight.CreateIntensityAttr(500)
            domeLight.CreateSpecularAttr().Set(0.0)

            distantLight = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
            distantLight.CreateIntensityAttr(500)
            distantLight.CreateAngleAttr(0.53)
            distantLight.CreateSpecularAttr().Set(0.0)

            stage.SetEditTarget(stage.GetSessionLayer())

            cam = UsdGeom.Camera.Define(stage, "/OmniverseKit_Persp")
            if cam:
                # Fix for the camera clipping range using the physx tolerances scale
                clipRangeWasSet = false = False
                physxInstancePrim = stage.GetPrimAtPath("/Scenes/PxPhysics/PxPhysics_1")
                if physxInstancePrim:
                    if physxInstancePrim.HasAttribute("omni:pvd:tolerancesScale"):
                        tolerancesScale = physxInstancePrim.GetAttribute("omni:pvd:tolerancesScale").Get(Usd.TimeCode.EarliestTime())
                        if tolerancesScale:
                            metersPerStageUnit = 1 / tolerancesScale[0]
                            cam.CreateClippingRangeAttr(Gf.Vec2f(0.01/metersPerStageUnit, 1000000))
                            clipRangeWasSet = True
                            # Set the default global gizmo slider scale, which works like a multiplier
                            carb.settings.get_settings().set(SETTING_OMNIPVD_GIZMO_GLOBAL_SCALE, 1.0/metersPerStageUnit)
                if not clipRangeWasSet:
                    cam.CreateClippingRangeAttr(Gf.Vec2f(0.000001, 1000000))

            stage.SetEditTarget(stage.GetRootLayer())

            resolution = vp_utils.get_active_viewport().resolution
            viewport_api = get_active_viewport()
            cam_path = viewport_api.camera_path
            omni.kit.commands.execute(
                'FramePrimsCommand',
                prim_to_move= cam_path,
                prims_to_frame = ["/Scenes"],
                time_code=Usd.TimeCode.Default(),
                aspect_ratio=resolution[0]/resolution[1],
                zoom=0.25
                )
            return True
        return False

    def _import_ovd_file_from_path(self, fileName, dirPath, loadMessages=False):
        carb.settings.get_settings().set(SETTING_OMNIPVD_LAST_IMPORT_DIRECTORY, dirPath)
        ################################################################################
        # Convert the fileName into the cacheFileDir
        ################################################################################
        cacheFileDir = self._getUSDCacheDirName(dirPath, fileName)
        cacheDir = carb.settings.get_settings().get(SETTING_OMNIPVD_USD_CACHE_DIRECTORY)
        fullCacheDirPath = cacheDir + cacheFileDir + "/"
        cacheStagePath = fullCacheDirPath + "stage.usda"
        ovdFile = dirPath + fileName

        if loadMessages:
            if self._ovdLoaded:
                self._physxPvdInterface.load_ovd(ovdFile)
        else:
            omni.usd.get_context().close_stage()

            useCacheFile = not carb.settings.get_settings().get_as_bool(SETTING_OMNIPVD_INVALIDATE_CACHE)
            if useCacheFile:
                self._ovdLoaded = self._open_cached_stage(cacheStagePath)
                if self._ovdLoaded:
                    carb.settings.get_settings().set(SETTING_OMNIPVD_IMPORTED_OVD, ovdFile)
                    foundAxis, upAxis = getStageUpAxisAsInt()
                    if foundAxis:
                        self._axisComboBox.model.get_item_value_model().set_value(upAxis)
                    window = ui.Workspace.get_window("OVD Tree")
                    if window:
                        window.visible = True
                        window.focus()
                    return
            if not os.path.exists(fullCacheDirPath):
                os.makedirs(fullCacheDirPath)

            ################################################################################
            # if stage.usda does NOT exist in the directory
            #   convert ovd to usda into the cache dir (or not might do it in memory)
            ################################################################################
            self._physxPvdInterface.ovd_to_usd(ovdFile, fullCacheDirPath, self._omniPvdUpAxis, self._omniPvdUSDType)

            self._ovdLoaded = self._open_cached_stage(cacheStagePath)

    def _import_ovd_file(self, loadMessages=False):
        def on_choose(fp_dialog, fileName, dirPath):
            fp_dialog.hide()
            if dirPath and fileName:
                if not dirPath.endswith("/"):
                    dirPath += "/"
                fullPath = dirPath + fileName
                if os.path.exists(fullPath):
                    self._import_ovd_file_from_path(fileName, dirPath)
            cleanup_fp_dialog(fp_dialog)

        ################################################################################
        # Get the OVD recording directory
        ################################################################################
        kitOvdRecordingDir = carb.settings.get_settings().get(SETTING_OMNIPVD_LAST_IMPORT_DIRECTORY)
        if not kitOvdRecordingDir:
            kitOvdRecordingDir = carb.settings.get_settings().get(SETTING_OMNIPVD_OVD_RECORDING_DIRECTORY)
        defaultPath = ""
        if kitOvdRecordingDir:
            fileName = self._getLatestFileByModificationTime(kitOvdRecordingDir)
            if fileName:
                defaultPath = kitOvdRecordingDir + fileName
        if defaultPath:
            if not kitOvdRecordingDir.endswith("/"):
                kitOvdRecordingDir += "/"
            self._import_ovd_file_from_path(fileName, kitOvdRecordingDir, loadMessages)
        else:
            item_filter_options = ["OVD Files (*.ovd)", "All Files (*)"]
            fp_dialog = FilePickerDialog(
                "Import OVD File",
                apply_button_label="Import",
                click_apply_handler=lambda filename, dirname: on_choose(fp_dialog, filename, dirname),
                item_filter_options=item_filter_options,
            )
            fp_dialog.show(path = defaultPath)

    def get_ovd_for_baking(self):
        def modTime(fileName):
            concatPath = kitOvdRecordingDir + fileName
            return os.path.getmtime(concatPath)

        def _getLatestFileByModificationTime(directory):
            def is_valid_file(filename):
                return filename != 'tmp.ovd' and filename.endswith('.ovd')

            kitOvdRecordingDir = directory
            filePath = None
            if not os.path.exists(directory):
                return filePath
            try:
                my_list = next(os.walk(directory))[2]
                if my_list:
                    my_list = list(filter(is_valid_file, my_list))
                    if my_list:
                        filePath = max(my_list, key=modTime)
            except StopIteration:
                pass
            return filePath

        def on_choose(fp_dialog, filename, dirpath):
            fp_dialog.hide()
            if (dirpath and filename):
                if not dirpath.endswith("/"):
                    dirpath += "/"
                # Set the OmniPVD output dir here, needs new var, outputdir or similar
                carb.settings.get_settings().set(SETTING_OMNIPVD_OVD_FOR_BAKING, dirpath + filename)
                daString = carb.settings.get_settings().get(SETTING_OMNIPVD_OVD_FOR_BAKING)
                self._physxPvdInterface.ovd_to_usd_over(daString)

            cleanup_fp_dialog(fp_dialog)

        kitOvdRecordingDir = carb.settings.get_settings().get(SETTING_OMNIPVD_OVD_RECORDING_DIRECTORY)
        defaultPath = ""
        if kitOvdRecordingDir:
            fileName = _getLatestFileByModificationTime(kitOvdRecordingDir)
            if fileName:
                defaultPath = kitOvdRecordingDir + fileName

        item_filter_options = ["OVD Files (*.ovd)"]
        fp_dialog = FilePickerDialog(
            "Latest OVD Recording",
            apply_button_label="Select Current",
            click_apply_handler=lambda filename, dirname: on_choose(fp_dialog, filename, dirname),
            item_filter_options=item_filter_options,
        )
        fp_dialog.show(path = defaultPath)

    def _add_over_layer_to_stage(self):
        self.get_ovd_for_baking()

    def _up_axis_changed(self, model, item):
        self._omniPvdUpAxis = model.get_item_value_model().as_int
        if not omni.usd.get_context():
            return
        stage = omni.usd.get_context().get_stage()
        if not stage:
            return
        if (self._omniPvdUpAxis == 0):
            UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
        else:
            UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

    def _usd_type_changed(self, model, item):
        self._omniPvdUSDType = model.get_item_value_model().as_int

    def _input_file_changed(model):
        carb.settings.get_settings().set(SETTING_OMNIPVD_OVD_FOR_BAKING, model.as_string)

    ################################################################################
    # OVD timeline START
    ################################################################################
    def _get_frame_id(self):
        return carb.settings.get_settings().get_as_int(SETTING_OMNIPVD_TIMELINE_FRAME_ID)

    def _get_sim_step(self):
        return carb.settings.get_settings().get_as_int(SETTING_OMNIPVD_TIMELINE_SIM_STEP_ID)

    def _ovd_timeline_get_frames_per_step(self):
        return carb.settings.get_settings().get_as_int(SETTING_OMNIPVD_TIMELINE_FRAMES_PER_SIM_STEP)

    def _ovd_timeline_get_sim_step_id_min(self):
        return carb.settings.get_settings().get_as_int(SETTING_OMNIPVD_TIMELINE_SIM_STEP_ID_MIN)

    def _ovd_timeline_get_sim_step_id_max(self):
        return carb.settings.get_settings().get_as_int(SETTING_OMNIPVD_TIMELINE_SIM_STEP_ID_MAX)

    def _ovd_timeline_get_frame_id_min(self):
        return carb.settings.get_settings().get_as_int(SETTING_OMNIPVD_TIMELINE_FRAME_ID_MIN)

    def _ovd_timeline_get_frame_id_max(self):
        return carb.settings.get_settings().get_as_int(SETTING_OMNIPVD_TIMELINE_FRAME_ID_MAX)

    def _ovd_timeline_is_legacy_ovd(self):
        return carb.settings.get_settings().get_as_bool(SETTING_OMNIPVD_TIMELINE_IS_LEGACY_OVD)

    def _ovd_timeline_get_frame_mode(self):
        return carb.settings.get_settings().get_as_int(SETTING_OMNIPVD_TIMELINE_FRAME_MODE)

    def _ovd_timeline_frame_to_step(self, frameId):
        return math.ceil( float(frameId) / float(self._ovd_timeline_get_frames_per_step()) )

    def _ovd_timeline_step_to_min_frame(self, step):
        return ( (step -1) * self._ovd_timeline_get_frames_per_step() ) + 1

    def _ovd_timeline_clamp_min_max(self, val, valMin, valMax):
        if val < valMin:
            val = valMin
        if val > valMax:
            val = valMax
        return val

    def _ovd_timeline_cap_frame_id_min_max(self, frameId):
        return self._ovd_timeline_clamp_min_max( frameId, self._ovd_timeline_get_frame_id_min(), self._ovd_timeline_get_frame_id_max())

    def _ovd_timeline_cap_simstep_min_max(self, simStepId):
        return self._ovd_timeline_clamp_min_max( simStepId, self._ovd_timeline_get_sim_step_id_min(), self._ovd_timeline_get_sim_step_id_max())

    def _frameIsPreSim(self, frameId):
        if frameId % 2 == 0: # postSim frames are even : 2, 4, 6 etc
            return False
        return True

    # Depending on the replay mode and the number of actual frames per simulation step, we step 1 or 2 frames
    def _ovd_timeline_get_frame_steps_dynamic(self):
        frameMode =  self._ovd_timeline_get_frame_mode()
        if ( (frameMode == OvdTimelineFrameMode.ePreSimAndPostSim) or self._ovd_timeline_is_legacy_ovd() ):
            return 1
        return 2

    def _get_min_legal_frame_id(self):
        frameMin = self._ovd_timeline_get_frame_id_min()
        # is the frameMin a legal frameId?
        frameMode =  self._ovd_timeline_get_frame_mode()
        if ((frameMode == OvdTimelineFrameMode.ePreSimAndPostSim) or self._ovd_timeline_is_legacy_ovd() ):
            return frameMin
        elif frameMode == OvdTimelineFrameMode.ePostSim:
            return frameMin+1
        else: # frameType is preSim
            return frameMin

    def _get_max_legal_frame_id(self):
        frameMax = self._ovd_timeline_get_frame_id_max()
        # is the frameMax a legal frameId?
        frameMode =  self._ovd_timeline_get_frame_mode()
        if ((frameMode == OvdTimelineFrameMode.ePreSimAndPostSim) or self._ovd_timeline_is_legacy_ovd() ):
            return frameMax
        elif frameMode == OvdTimelineFrameMode.ePreSim:
            return frameMax-1
        else: # frameType is postSim
            return frameMax

    # Given the current frameMode, clamp the incoming frameId to be a legal frameId
    # Does not take into account the temporary override of the user when switching it manually
    def _clamp_legal_frame_id(self, frameId):
        if self._ovd_timeline_is_legacy_ovd(): # legacy
            return self._ovd_timeline_cap_frame_id_min_max(frameId) # just clamp min/max
        frameMode = self._ovd_timeline_get_frame_mode()
        if frameMode == OvdTimelineFrameMode.ePreSimAndPostSim:
            return self._ovd_timeline_cap_frame_id_min_max(frameId) # just clamp min/max
        elif frameMode == OvdTimelineFrameMode.ePreSim:
            frameId = self._ovd_timeline_cap_frame_id_min_max(frameId)
            if not self._frameIsPreSim(frameId): # frameId must be preSim
                frameId -=1 # frame is postSim -> switch it to preSim
            return frameId
        else: # post-sim mode
            frameId = self._ovd_timeline_cap_frame_id_min_max(frameId)
            if self._frameIsPreSim(frameId): # frameId must be postSim
                frameId +=1 # frame is preSim -> switch it to postSim
            return frameId

    def _frameIdToSliderVal(self, frameId):
        frameMode = self._ovd_timeline_get_frame_mode()
        if frameMode == OvdTimelineFrameMode.ePreSimAndPostSim:
            # frameId values go from frameMin to frameMax
            # but the slideVal goes from 0 to nbrFrames - 1 aka frameIdNormalized
            # frameIdNormalized = frameId - self._ovd_timeline_get_frame_id_min()
            return frameId - self._ovd_timeline_get_frame_id_min()
        elif frameMode == OvdTimelineFrameMode.ePreSim:
            # frameId values go from frameMin to frameMax
            # but the slideVal goes from 0 to nbrSimFrames - 1, aka simStepNormalized
            # the first frame might actually not be a preSim frame, as recording can start
            # at any frame, but that doesn't matter for
            # simStepNormalized = (frameId - frameMin) / nbrFramesPerSimStep
            return int(frameId - self._ovd_timeline_get_frame_id_min()) / self._ovd_timeline_get_frames_per_step()
        else: # post-sim mode
            if self._ovd_timeline_is_legacy_ovd(): # legacy
                # sliderVal values go from 0 to nbrFrames - 1, so we need to re-position the value
                # frameIdNormalized = frameId - self._ovd_timeline_get_frame_id_min()
                return frameId - self._ovd_timeline_get_frame_id_min()
            else:
                # First assume all sim steps are ok -> all sim steps have a post frame
                # the last sim frame might not have a postSim frame
                nbrFramesPerSimStep = self._ovd_timeline_get_frames_per_step()
                return (frameId - self._ovd_timeline_get_frame_id_min() - 1) / nbrFramesPerSimStep

    def _sliderValToFrameId(self, sliderVal):
        frameMode = self._ovd_timeline_get_frame_mode()
        if frameMode == OvdTimelineFrameMode.ePreSimAndPostSim:
            # sliderVal values go from 0 to nbrFrames - 1, so we need to re-position the value
            # frameId = self._ovd_timeline_get_frame_id_min() + sliderVal
            return self._ovd_timeline_get_frame_id_min() + sliderVal
        elif frameMode == OvdTimelineFrameMode.ePreSim:
            # sliderVal :: simStepNormalized (0 -> nbrSimSteps - 1)
            # frameId = self._ovd_timeline_get_frame_id_min() + sliderVal * nbrFramesPerSimStep
            return self._ovd_timeline_get_frame_id_min() + sliderVal * self._ovd_timeline_get_frames_per_step()
        else: # post-sim mode
            if self._ovd_timeline_is_legacy_ovd(): # legacy
                # sliderVal values go from 0 to nbrFrames - 1, so we need to re-position the value
                # frameId = self._ovd_timeline_get_frame_id_min() + sliderVal
                return self._ovd_timeline_get_frame_id_min() + sliderVal
            else:
                # First assume all sim steps are ok -> all sim steps have a post frame
                # sliderVal values go from 0 to nbrSimSteps - 1
                # frameId = self._ovd_timeline_get_frame_id_min() + sliderVal * nbrFramesPerSimStep
                # check if the last pos frame is valid? hmmm...
                nbrFramesPerSimStep = self._ovd_timeline_get_frames_per_step()
                return self._ovd_timeline_get_frame_id_min() + (sliderVal * nbrFramesPerSimStep) + 1

    def _ovd_timeline_set_frame_type_label(self, frameId):
        if self._ovd_timeline_is_legacy_ovd():
            self._frame_type_label.text = "Post"
            return
        if self._frameIsPreSim(frameId):
            self._frame_type_label.text = "Pre"
        else:
            self._frame_type_label.text = "Post"

    def _updateFrameId(self, frameId):
        carb.settings.get_settings().set_int(SETTING_OMNIPVD_TIMELINE_FRAME_ID, frameId)
        self._ovd_timeline_set_frame_type_label(frameId)
        timeline = omni.timeline.get_timeline_interface()
        timeline.set_current_time(frameId / timeline.get_time_codes_per_seconds())
        if (not self._sliderIsActive) and ((not self._isTimelineFrameTypeOverridden) or (self._ovd_timeline_get_frame_mode() ==  OvdTimelineFrameMode.ePreSimAndPostSim)):
            sliderVal = self._frameIdToSliderVal(frameId)
            self._timeline_slider_model.set_value(float(sliderVal))

    def _ovd_timeline_frame_advance(self, isNext):
        frameIdOrig = self._get_frame_id()
        frameId = self._clamp_legal_frame_id(frameIdOrig)
        if isNext:
            frameId += self._ovd_timeline_get_frame_steps_dynamic()
        else:
            frameId -= self._ovd_timeline_get_frame_steps_dynamic()
        frameId = self._clamp_legal_frame_id(frameId)
        if frameId != frameIdOrig:
            self._updateFrameId(frameId)
            simStepOrig = self._get_sim_step()
            simStep = self._ovd_timeline_frame_to_step(frameId)
            if (simStep != simStepOrig):
                carb.settings.get_settings().set_int(SETTING_OMNIPVD_TIMELINE_SIM_STEP_ID, simStep)

    def _ovd_timeline_update_slider(self):
        # depending on the frame playback mode allow the slider to be certain values
        frameMode = self._ovd_timeline_get_frame_mode()
        nbrSimSteps = self._ovd_timeline_get_sim_step_id_max() - self._ovd_timeline_get_sim_step_id_min() + 1
        nbrFrames = self._ovd_timeline_get_frame_id_max() - self._ovd_timeline_get_frame_id_min() + 1
        if frameMode == OvdTimelineFrameMode.ePreSimAndPostSim:
            self._timeline_slider.min = 0
            self._timeline_slider.max = nbrFrames - 1
        else:
            self._timeline_slider.min = 0
            self._timeline_slider.max = nbrSimSteps - 1
        # assumes that the frameId is properly capped already
        frameId = carb.settings.get_settings().get_as_int(SETTING_OMNIPVD_TIMELINE_FRAME_ID)
        sliderVal = self._frameIdToSliderVal(frameId)
        self._timeline_slider_model.set_value(float(sliderVal))
        self._timeline_slider.step = 1

    # Called with the "heartbeat" of Kit
    def _ovd_timeline_play_update(self, e: carb.events.IEvent):
        if self._isLoadingOVD or self._sliderIsActive:
            return
        playbackState = carb.settings.get_settings().get_as_int(SETTING_OMNIPVD_TIMELINE_PLAYBACK_STATE)
        if playbackState == OvdTimelinePlaybackState.ePaused:
            self._firstTimelineUpdate = True # Reset the value whenever we are not playing
            self._ovdTimelinePlayButton.image_url = f"{self._icons_path}/play.svg"
            return
        currTime = time.monotonic()
        if self._firstTimelineUpdate:
            self._lastTimelineUpdateTime = currTime
            self._firstTimelineUpdate = False
        self._isTimelineFrameTypeOverridden = False
        frameIdOrig = carb.settings.get_settings().get_as_int(SETTING_OMNIPVD_TIMELINE_FRAME_ID)
        frameId = self._clamp_legal_frame_id(frameIdOrig)
        deltaTimeSec = 0.001 * float(carb.settings.get_settings().get_as_int(SETTING_OMNIPVD_TIMELINE_FRAME_DELTA_MS))
        if ((currTime - self._lastTimelineUpdateTime) > deltaTimeSec):
            # the stepping is on purpose not fractional, so as to allow for very large scenes not to skip frames
            # if the replay is not able to show the frames fast enough, so that no frames are skipped
            frameId = frameId + self._ovd_timeline_get_frame_steps_dynamic()
            frameId = self._ovd_timeline_cap_frame_id_min_max(frameId)
            self._lastTimelineUpdateTime = currTime
        if frameId != frameIdOrig:
            self._updateFrameId(frameId)
            carb.settings.get_settings().set_int(SETTING_OMNIPVD_TIMELINE_SIM_STEP_ID, self._ovd_timeline_frame_to_step(frameId))
        if (frameId >= self._get_max_legal_frame_id()):
            self._ovdTimelinePlayButton.image_url = f"{self._icons_path}/play.svg"
            carb.settings.get_settings().set_int(SETTING_OMNIPVD_TIMELINE_PLAYBACK_STATE, OvdTimelinePlaybackState.ePaused)


    ################################################################################
    # User action callbacks
    ################################################################################

    # User pressed the "|<<" button
    def _ovd_timeline_goto_first_simstep(self):
        self._isTimelineFrameTypeOverridden = False
        carb.settings.get_settings().set_int(SETTING_OMNIPVD_TIMELINE_PLAYBACK_STATE, OvdTimelinePlaybackState.ePaused)
        frameId = self._clamp_legal_frame_id(self._ovd_timeline_get_frame_id_min())
        self._updateFrameId(frameId)
        carb.settings.get_settings().set_int(SETTING_OMNIPVD_TIMELINE_SIM_STEP_ID, self._ovd_timeline_frame_to_step(frameId))

    # User pressed the "<<" button
    def _ovd_timeline_prev_frame(self):
        self._isTimelineFrameTypeOverridden = False
        self._ovd_timeline_frame_advance(False)

    # User pressed the ">" / "||" button
    def _ovd_timeline_play_or_pause(self):
        self._isTimelineFrameTypeOverridden = False
        playbackState = carb.settings.get_settings().get_as_int(SETTING_OMNIPVD_TIMELINE_PLAYBACK_STATE)
        if playbackState == OvdTimelinePlaybackState.ePaused:
            # we are trying to switch from paused to playing
            frameId = self._clamp_legal_frame_id(self._get_frame_id()) # make sure the frametype is a valid one
            frameIdMaxlegal = self._get_max_legal_frame_id()
            if (frameId >= frameIdMaxlegal):
                # jump to beginning and play
                frameId = self._get_min_legal_frame_id()
            self._updateFrameId(frameId)
            self._ovdTimelinePlayButton.image_url = f"{self._icons_path}/pause.svg"
            carb.settings.get_settings().set_int(SETTING_OMNIPVD_TIMELINE_PLAYBACK_STATE, OvdTimelinePlaybackState.ePlaying)
        else:
            # we are switching from playing to paused
            carb.settings.get_settings().set_int(SETTING_OMNIPVD_TIMELINE_PLAYBACK_STATE, OvdTimelinePlaybackState.ePaused)
            self._ovdTimelinePlayButton.image_url = f"{self._icons_path}/play.svg"

    # User pressed the ">>" button
    def _ovd_timeline_next_frame(self):
        self._isTimelineFrameTypeOverridden = False
        self._ovd_timeline_frame_advance(True)

    # User pressed the ">>|" button
    def _ovd_timeline_goto_last_simstep(self):
        self._isTimelineFrameTypeOverridden = False
        carb.settings.get_settings().set_int(SETTING_OMNIPVD_TIMELINE_PLAYBACK_STATE, OvdTimelinePlaybackState.ePaused)
        frameId = self._clamp_legal_frame_id(self._ovd_timeline_get_frame_id_max())
        self._updateFrameId(frameId)
        carb.settings.get_settings().set_int(SETTING_OMNIPVD_TIMELINE_SIM_STEP_ID, self._ovd_timeline_frame_to_step(frameId))

    # User chose a frameType playback mode
    def _ovd_timeline_frame_mode_changed(self, item, event_type):
        self._isTimelineFrameTypeOverridden = False
        # if the mode is pre+pos OR post-legacy -> then no potential change of frame
        # if the mode is pre OR post -> then if the frame is the opposte -> switch it
        if event_type == carb.settings.ChangeEventType.CHANGED:
            frameMode = self._ovd_timeline_get_frame_mode()
            frameIdOrig = self._get_frame_id()
            frameId = self._clamp_legal_frame_id(frameIdOrig)
            if (frameId != frameIdOrig):
                self._updateFrameId(frameId)
            self._ovd_timeline_update_slider()
            # constrain the frameId and update the frameTypeField

    def _ovd_is_ovd_stage_setting_changed(self, item, event_type):
        if event_type == carb.settings.ChangeEventType.CHANGED:
            is_ovd_stage = carb.settings.get_settings().get_as_bool(SETTING_OMNIPVD_IS_OVD_STAGE)
            self._register_to_global_event_update(is_ovd_stage)

    # User input a sim step (or some other function did)
    def _ovd_timeline_sim_step_id_changed(self, item, event_type):
        if self._isLoadingOVD or self._sliderIsActive:
            return
        if event_type == carb.settings.ChangeEventType.CHANGED:
            self._isTimelineFrameTypeOverridden = False
            simStepIdOrig = carb.settings.get_settings().get_as_int(SETTING_OMNIPVD_TIMELINE_SIM_STEP_ID)
            simStepId = self._ovd_timeline_cap_simstep_min_max(simStepIdOrig)
            if simStepId != simStepIdOrig:
                carb.settings.get_settings().set_int(SETTING_OMNIPVD_TIMELINE_SIM_STEP_ID, simStepId)
            frameStepsPerSimStep = self._ovd_timeline_get_frames_per_step()
            frameIdMin = self._ovd_timeline_step_to_min_frame(simStepId)
            frameIdMax = frameIdMin + frameStepsPerSimStep -1
            frameId = self._get_frame_id()
            if not ((frameId >= frameIdMin) and (frameId <= frameIdMax)):
                frameId = frameIdMin
                frameId = self._clamp_legal_frame_id(frameId)
                self._updateFrameId(frameId)

    # User pressed the temporary frameType switched button : (preSim -> postSim) or (postSim -> preSim)
    def _ovd_timeline_frametype_clicked(self):
        self._isTimelineFrameTypeOverridden = True
        frameIdOrig = self._get_frame_id()
        frameId = frameIdOrig
        if self._frameIsPreSim(frameId):
            frameId +=1 # make it postSim
        else:
            frameId -=1 # this assumes the previous sim step has a preSim frame
        if frameId!=frameIdOrig:
            self._updateFrameId(frameId)

    def _build_ovd_timeline(self):
        frameMode = ["Post", "Pre", "Pre+Post"]
        uiWidth = 50
        uiHeight = 25
        uiHeight2 = 25
        with ui.HStack():
            self._ovdTimelineGotoStartButton = ui.Button(image_url = f"{self._icons_path}/start.svg", tooltip = "Go to first simulation step", width = uiWidth, height = uiHeight)
            self._ovdTimelineGotoStartButton.set_clicked_fn(self._ovd_timeline_goto_first_simstep)
            ui.Spacer(width=5)
            self._ovdTimelinePrevButton = ui.Button(image_url = f"{self._icons_path}/previous_frame.svg", tooltip = "Go to previous frame", width = uiWidth, height = uiHeight)
            self._ovdTimelinePrevButton.set_clicked_fn(self._ovd_timeline_prev_frame)
            ui.Spacer(width=5)
            self._ovdTimelinePlayButton = ui.Button(image_url = f"{self._icons_path}/play.svg", tooltip = "Play or Pause. Does not skip frames.", width = uiWidth, height = uiHeight)
            self._ovdTimelinePlayButton.set_clicked_fn(self._ovd_timeline_play_or_pause)
            ui.Spacer(width=5)
            self._ovdTimelineNextButton = ui.Button(image_url = f"{self._icons_path}/next_frame.svg", tooltip = "Go to next frame", width = uiWidth, height = uiHeight)
            self._ovdTimelineNextButton.set_clicked_fn(self._ovd_timeline_next_frame)
            ui.Spacer(width=5)
            self._ovdTimelineStopButton = ui.Button(image_url = f"{self._icons_path}/end.svg", tooltip = "Go to last simulation step", width = uiWidth, height = uiHeight)
            self._ovdTimelineStopButton.set_clicked_fn(self._ovd_timeline_goto_last_simstep)
            ui.Spacer(width=5)
            with ui.VStack(width = 0):
                ui.Spacer(height=3)
                create_setting_widget(SETTING_OMNIPVD_TIMELINE_SIM_STEP_ID, SettingType.INT, width = uiWidth, height = uiHeight2, tooltip = "Current simulation step, click and drag to change")
            ui.Spacer(width=2)
            self._frame_type_label = ui.Button("Post", tooltip = "Toggle between Pre-Simulation and Post-Simulation views", width = uiWidth, height = uiHeight)
            self._frame_type_label.set_clicked_fn(self._ovd_timeline_frametype_clicked)
            ui.Spacer(width=5)

            ################################################################################
            # Slider action callbacks
            ################################################################################
            def slider_begin_edit(model, s):
                self._isTimelineFrameTypeOverridden = False
                self._sliderIsActive = True

            def slider_end_edit(model, s):
                self._sliderIsActive = False

            def slider_value_changed(model, s):
                if self._isLoadingOVD:
                    return
                if not self._sliderIsActive:
                    return
                sliderVal = int(model.get_value_as_float())
                frameId = s._sliderValToFrameId(sliderVal)
                frameIdOrig = carb.settings.get_settings().get_as_int(SETTING_OMNIPVD_TIMELINE_FRAME_ID)
                if frameId != frameIdOrig:
                    carb.settings.get_settings().set_int( SETTING_OMNIPVD_TIMELINE_FRAME_ID, frameId )
                    carb.settings.get_settings().set_int( SETTING_OMNIPVD_TIMELINE_SIM_STEP_ID, self._ovd_timeline_frame_to_step(frameId) )
                    timeline = omni.timeline.get_timeline_interface()
                    timeline.set_current_time(frameId / timeline.get_time_codes_per_seconds())
                    self._ovd_timeline_set_frame_type_label(frameId)

            self._timeline_slider_model = ui.SimpleFloatModel(0.0)
            self._carb_subs = []
            self._carb_subs.append(self._timeline_slider_model.subscribe_begin_edit_fn(lambda m: slider_begin_edit(m, self)))
            self._carb_subs.append(self._timeline_slider_model.subscribe_value_changed_fn(lambda m: slider_value_changed(m, self)))
            self._carb_subs.append(self._timeline_slider_model.subscribe_end_edit_fn(lambda m: slider_end_edit(m, self)))
            with ui.VStack():
                ui.Spacer(height=2)
                self._timeline_slider = ui.FloatSlider(
                    name="timeline_slider",
                    model=self._timeline_slider_model,
                    style = {"color":cl.transparent},
                    min=0.0,
                    max=1.0,
                    step=1.0,
                    height = 20
                )
        with ui.HStack():
            ui.Label("Replay Mode", width = 20)
            ui.Spacer(width=5)
            with ui.Frame(width = 90):
                self._frameModeWidget, model = create_setting_widget_combo(SETTING_OMNIPVD_TIMELINE_FRAME_MODE, frameMode, tooltip = "Toggle between Pre-simulation, Post-simulation and Pre- and Post-simulation replay modes")
            ui.Spacer(width=5)
            ui.Label("Speed (ms)", width = 30)
            ui.Spacer(width=5)
            create_setting_widget(SETTING_OMNIPVD_TIMELINE_FRAME_DELTA_MS, SettingType.INT, width = 40, tooltip = "Set the millisecond wait time between simulation frames")

        ui.Spacer(height=5)

    ################################################################################
    # OVD timeline STOP
    ################################################################################

    ################################################################################
    # OVD Import START
    ################################################################################
    def _build_usdtform(self):
        def define_import_dir():
            def on_choose(fp_dialog, filename, dirpath):
                fp_dialog.hide()
                if (dirpath):
                    if not dirpath.endswith("/"):
                        dirpath += "/"
                    # Sets the OVD import directory
                    carb.settings.get_settings().set(SETTING_OMNIPVD_LAST_IMPORT_DIRECTORY, dirpath)
                cleanup_fp_dialog(fp_dialog)

            item_filter_options = ["OVD Files (*.ovd)"]
            fp_dialog = FilePickerDialog(
                "Select the OVD Import Directory",
                apply_button_label="Select Current",
                click_apply_handler=lambda filename, dirname: on_choose(fp_dialog, filename, dirname),
                item_filter_options=item_filter_options,
            )
            fp_dialog.show()

        with ui.HStack():
            self._transformToOmniPvdUSDButton = ui.Button("Load Latest", tooltip="Load the last modified OVD file in the import directory", width = 160, height = 20)
            self._transformToOmniPvdUSDButton.set_clicked_fn(self._import_ovd_file)
            ui.Spacer(width=5)
            with ui.VStack():
                ui.Spacer(height=3)
                widget, model = create_setting_widget(SETTING_OMNIPVD_IMPORTED_OVD, SettingType.STRING, height = 15, enabled = False)
                widget.enabled = False
                widget.set_style(DISABLED_STYLE)
        with ui.HStack(height=23):
            button = ui.Button("Set Import Directory", tooltip="Select the import directory from where Load Latest will scan for the last modified OVD file. If an OVD file is recorded, the import directory is automatically set to the recording directory.", height=30, width=160)
            button.set_clicked_fn(define_import_dir)
            ui.Spacer(width=5)
            with ui.VStack(height=23):
                ui.Spacer(height=4)
                widget = ui.StringField(SettingModel(SETTING_OMNIPVD_LAST_IMPORT_DIRECTORY), height=22, enabled = False)
                widget.enabled = False
                widget.set_style(DISABLED_STYLE)
            #ui.Spacer(height=5)
        with ui.HStack():
            ui.Spacer(width=3)
            with ui.VStack():
                ui.Spacer(height=3)
                with ui.HStack():
                    ################################################################################
                    # Up axis
                    ################################################################################
                    axisTypes = ["Y up axis", "Z up axis"]
                    axisFound, upAxis = getStageUpAxisAsInt()
                    if axisFound:
                        self._omniPvdUpAxis = upAxis
                    self._axisComboBox = omni.ui.ComboBox(self._omniPvdUpAxis, *axisTypes, tooltip="Change the up axis of the Stage", width = 155, height = 20)
                    self._axisComboBox.model.add_item_changed_fn(self._up_axis_changed)

                    ui.Spacer(width=10)
                    ui.Label("Invalidate Cache", width=100)
                    self._cacheBox = _build_checkbox(SettingModel(SETTING_OMNIPVD_INVALIDATE_CACHE))

        with ui.HStack():
            self._my_button = ui.Button("Set Cache Directory", tooltip="Select where the USD Stage of the imported OVD file will be saved", width = 160, height = 20 )
            self._my_button.set_clicked_fn(set_usd_cache_dir)
            ui.Spacer(width=5)
            widget, model = create_setting_widget(SETTING_OMNIPVD_USD_CACHE_DIRECTORY, SettingType.STRING, height = 15)
            widget.enabled = False
            widget.set_style(DISABLED_STYLE)
        ui.Spacer(height=5)


    ################################################################################
    # OVD Import STOP
    ################################################################################

    ################################################################################
    # OVD Gizmos START
    ################################################################################

    def _build_omnipvd_gizmos(self):
        ################################################################################
        # Set contacts/bounding boxes/center of mass (None/Selected/Unselected/Everything)
        ################################################################################
        vizModes = ["None", "Selected", "All"]
        vizModesHide = ["None", "Selected"]

        #ui.Spacer(height=5)
        with ui.HStack():
            ################################################################################
            # All
            ################################################################################
            ui.Spacer(width=5)
            ui.Label("All", width = 150, height = 20)
            ui.Spacer(width=5+100+20)
            ui.Label("Scale", width = 40, height = 20)
            ui.Spacer(width=5)
            create_setting_widget(SETTING_OMNIPVD_GIZMO_GLOBAL_SCALE, SettingType.FLOAT, 0.0, 1000.0, width = 80, height = 15)
        ui.Spacer(height=10)
        ui.Separator(height=10)
        with ui.HStack():
            ################################################################################
            # Velocities
            ################################################################################
            ui.Spacer(width=5)
            ui.Label("Hide", width = 150, height = 20)
            ui.Spacer(width=5)
            with ui.Frame(width = 100, height = 20):
                create_setting_widget_combo(SETTING_OMNIPVD_GIZMO_TRANSPARENCY_VIZMODE, vizModesHide)
        ui.Spacer(height=5)
        with ui.HStack():
            ################################################################################
            # Contacts
            ################################################################################
            ui.Spacer(width=5)
            ui.Label("Contacts", width = 150, height = 20)
            ui.Spacer(width=5)
            #workaround for create_setting_widget_combo not taking layout parameters
            with ui.Frame(width = 100, height = 20):
                create_setting_widget_combo(SETTING_OMNIPVD_GIZMO_CONTACT_VIZMODE, vizModes)
            ui.Spacer(width=20)
            ui.Label("Scale", width = 40, height = 20)
            ui.Spacer(width=5)
            create_setting_widget(SETTING_OMNIPVD_GIZMO_CONTACT_SCALE, SettingType.FLOAT, 0.0, 100.0, width = 80, height = 15)
        ui.Spacer(height=5)
        with ui.HStack():
            ################################################################################
            # Bounding Boxes
            ################################################################################
            ui.Spacer(width=5)
            ui.Label("Bounding Boxes", width = 150, height = 20)
            ui.Spacer(width=5)
            with ui.Frame(width = 100, height = 20):
                create_setting_widget_combo(SETTING_OMNIPVD_GIZMO_BOUNDING_BOX_VIZMODE, vizModes)
        ui.Spacer(height=5)
        with ui.HStack():
            ################################################################################
            # Coordinate System
            ################################################################################
            ui.Spacer(width=5)
            ui.Label("Center of Mass", width = 150, height = 20)
            ui.Spacer(width=5)
            with ui.Frame(width = 100, height = 20):
                create_setting_widget_combo(SETTING_OMNIPVD_GIZMO_CENTER_OF_MASS_VIZMODE, vizModes)
            ui.Spacer(width=20)
            ui.Label("Scale", width = 40, height = 20)
            ui.Spacer(width=5)
            create_setting_widget(SETTING_OMNIPVD_GIZMO_CENTER_OF_MASS_SCALE, SettingType.FLOAT, 0.0, 100.0, width = 80, height = 15)
        ui.Spacer(height=5)
        with ui.HStack():
            ################################################################################
            # Coordinate System
            ################################################################################
            ui.Spacer(width=5)
            ui.Label("Coordinate System", width = 150, height = 20)
            ui.Spacer(width=5)
            with ui.Frame(width = 100, height = 20):
                create_setting_widget_combo(SETTING_OMNIPVD_GIZMO_COORDINATE_SYSTEM_VIZMODE, vizModes)
            ui.Spacer(width=20)
            ui.Label("Scale", width = 40, height = 20)
            ui.Spacer(width=5)
            create_setting_widget(SETTING_OMNIPVD_GIZMO_COORDINATE_SYSTEM_SCALE, SettingType.FLOAT, 0.0, 100.0, width = 80, height = 15)
        ui.Spacer(height=5)
        with ui.HStack():
            ################################################################################
            # Velocities
            ################################################################################
            ui.Spacer(width=5)
            ui.Label("Velocities", width = 150, height = 20)
            ui.Spacer(width=5)
            with ui.Frame(width = 100, height = 20):
                create_setting_widget_combo(SETTING_OMNIPVD_GIZMO_VELOCITY_VIZMODE, vizModes)
            ui.Spacer(width=20)
            ui.Label("Scale", width = 40, height = 20)
            ui.Spacer(width=5)
            create_setting_widget(SETTING_OMNIPVD_GIZMO_VELOCITY_SCALE, SettingType.FLOAT, 0.0, 100.0, width = 80, height = 15)
        ui.Spacer(height=5)
        with ui.HStack():
            ################################################################################
            # Joints
            ################################################################################
            ui.Spacer(width=5)
            ui.Label("Joints", width = 150, height = 20)
            ui.Spacer(width=5)
            #workaround for create_setting_widget_combo not taking layout parameters
            with ui.Frame(width = 100, height = 20):
                create_setting_widget_combo(SETTING_OMNIPVD_GIZMO_JOINT_VIZMODE, vizModes)
            ui.Spacer(width=20)
            ui.Label("Scale", width = 40, height = 20)
            ui.Spacer(width=5)
            create_setting_widget(SETTING_OMNIPVD_GIZMO_JOINT_SCALE, SettingType.FLOAT, 0.0, 100.0, width = 80, height = 15)
        ui.Spacer(height=5)

    ################################################################################
    # OVD Gizmos STOP
    ################################################################################

    def _build_physxoverlay(self):
        ################################################################################
        # Add an over layer to the current Stage using and OmniPvd OVD file as input
        ################################################################################
        ui.Spacer(height=5)
        with ui.HStack():
            self._my_button = ui.Button("Bake PhysX Transforms into the Active Edit Layer", width = 160, height = 20 )
            self._my_button.set_clicked_fn(self._add_over_layer_to_stage)
        ui.Spacer(height=5)
        with ui.HStack():
            ui.Spacer(width=5)
            ui.Label("Overlay OVD", width = 150, height = 20)
            ui.Spacer(width=5)
            widget, model = create_setting_widget(SETTING_OMNIPVD_OVD_FOR_BAKING, SettingType.STRING, height = 15)
        ui.Spacer(height=5)

    def _make_my_window(self):
        self._omniPvdUpAxis = 0
        self._omniPvdUSDType = 0 # default to usdc sub layers - binary USD

        self._window_example = ui.Window("OmniPVD", width=100, dockPreference=omni.ui.DockPreference.RIGHT_BOTTOM)
        self._window_example.deferred_dock_in("Content", ui.DockPolicy.CURRENT_WINDOW_IS_ACTIVE)

        ################################################################################
        # Chose input file
        ################################################################################
        from omni.kit.widget.settings.settings_widget import SettingType, create_setting_widget

        outDirectory = carb.settings.get_settings().get(SETTING_OMNIPVD_USD_CACHE_DIRECTORY)
        if not outDirectory:
            outDirectory = carb.tokens.get_tokens_interface().resolve("${omni_documents}") + "/omni_pvd/out/omnivpd_usd/"
            if not os.path.exists(outDirectory):
                os.makedirs(outDirectory)
            carb.settings.get_settings().set(SETTING_OMNIPVD_USD_CACHE_DIRECTORY, outDirectory)
        outDirectory = carb.settings.get_settings().get(SETTING_OMNIPVD_PHYSX_USD_DIRECTORY)
        if not outDirectory:
            outDirectory = carb.tokens.get_tokens_interface().resolve("${omni_documents}") + "/omni_pvd/out/omnivpd_physx_usd/"
            if not os.path.exists(outDirectory):
                os.makedirs(outDirectory)
            carb.settings.get_settings().set(SETTING_OMNIPVD_PHYSX_USD_DIRECTORY, outDirectory)

        with self._window_example.frame:
            with ui.ScrollingFrame(
                                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF,
                                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED):
                with ui.VStack(height = 0):
                    build_section("Timeline", self._build_ovd_timeline)
                    ui.Spacer(height=5)
                    build_section("Load OVD", self._build_usdtform)
                    ui.Spacer(height=5)
                    build_section("Record OVD", self._build_omnipvd_ui)
                    ui.Spacer(height=5)
                    build_section("Gizmos", self._build_omnipvd_gizmos)
                    ui.Spacer(height=5)
                    build_section("Overlay OVD on an Active Edit Layer", self._build_physxoverlay)

        return self._window_example

    def _register_to_global_event_update(self, register: bool = True):
        if register:
            self._firstTimelineUpdate = True
            self._timelineSub = get_eventdispatcher().observe_event(
                event_name=omni.kit.app.GLOBAL_EVENT_UPDATE,
                    on_event=self._ovd_timeline_play_update,
                    observer_name="[OVD Timeline update loop",
                )
        else:
            self._timelineSub = None
            carb.settings.get_settings().set_int(SETTING_OMNIPVD_TIMELINE_PLAYBACK_STATE, OvdTimelinePlaybackState.ePaused)

    def on_startup(self, ext_id):
        self._ext_path = omni.kit.app.get_app().get_extension_manager().get_extension_path(ext_id)
        self._icons_path =  icon_folder = f"{os.path.join(self._ext_path, PhysxPvdExtension.icons_folder)}"
        print(f"icon_path{self._icons_path}")
        ################################################################################
        # Preload library plugin dependencies, to make them discoverable
        ################################################################################
        if sys.platform == "win32":
            ext_folder = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../.."))
            omnipvd_dll_path = os.path.join(ext_folder, "bin/PVDRuntime_64.dll")
            ctypes.WinDLL(omnipvd_dll_path)

        ################################################################################
        # Acquire the instance reference pointer (for the sake of testing)
        ################################################################################
        PhysxPvdExtension.instance = self
        self._physxPvdInterface = _physxPvd.acquire_physx_pvd_interface()

        ################################################################################
        # Create the OmniPVD import/conversion menu
        ################################################################################
        self._menu = windowmenuitem.WindowMenuItem("OmniPVD", lambda: self._make_my_window(), True)

        ################################################################################
        # Create the OVD Tree
        ################################################################################
        self._treeView = windowmenuitem.WindowMenuItem("OVD Tree", lambda: OmniPVdObjectTreeWindow(), True)

        ################################################################################
        # Create the OmniPVD Messages Tree
        ################################################################################
        self._ovdLoaded = False
        self._pvdMessagesWindow = OmniPvdMessagesWindow()
        self._pvdMessagesWindow.set_import_fn(self._import_ovd_file)

        self._messagesView = windowmenuitem.WindowMenuItem("OVD Messages", lambda: self._pvdMessagesWindow, True)

        ################################################################################
        # Create the property widget
        ################################################################################
        if not self._propertyWidgetOmniPvd:
            prim_handlers = [
                default_prim_handler,
            ]
            self._propertyWidgetOmniPvd = PropertyWidgetOmniPvd(prim_handlers)
            from omni.kit.property.physx import register_widget
            register_widget(self._propertyWidgetOmniPvd.name, self._propertyWidgetOmniPvd)
            self._propertyWidgetOmniPvd.on_startup()

        ################################################################################
        # Create the OVD file import menu and register it with Kit
        ################################################################################
        self._importer = OVDImporter(self)
        omni.kit.tool.asset_importer.register_importer(self._importer)

        self._settings_subs.append(omni.kit.app.SettingChangeSubscription(SETTING_OMNIPVD_TIMELINE_SIM_STEP_ID, self._ovd_timeline_sim_step_id_changed))
        self._settings_subs.append(omni.kit.app.SettingChangeSubscription(SETTING_OMNIPVD_TIMELINE_FRAME_MODE, self._ovd_timeline_frame_mode_changed))
        self._settings_subs.append(omni.kit.app.SettingChangeSubscription(SETTING_OMNIPVD_IS_OVD_STAGE, self._ovd_is_ovd_stage_setting_changed))

    def on_shutdown(self):
        ################################################################################
        # Unregister the messages window
        ################################################################################
        self._pvdMessagesWindow.on_shutdown()
        self._pvdMessagesWindow = None

        ################################################################################
        # Unregister the property widget
        ################################################################################
        if self._propertyWidgetOmniPvd:
            self._propertyWidgetOmniPvd.on_shutdown()
            from omni.kit.property.physx import unregister_widget
            unregister_widget(self._propertyWidgetOmniPvd.name)
            self._propertyWidgetOmniPvd = None

        ################################################################################
        # Release the pvd interface
        ################################################################################
        _physxPvd.release_physx_pvd_interface(self._physxPvdInterface)
        self._physxPvdInterface = None

        ################################################################################
        # Remove the OVD asset importer from the Kit file menu
        ################################################################################
        omni.kit.tool.asset_importer.remove_importer(self._importer)
        self._importer.destroy()
        self._importer = None

        ################################################################################
        # Shut down the conversion menu
        ################################################################################
        self._menu.on_shutdown()
        self._menu = None

        ################################################################################
        # Remove the reference to the OmniPVD extension
        ################################################################################
        PhysxPvdExtension.instance = None

        ################################################################################
        # Shut down the OVD Tree
        ################################################################################
        if (self._treeView):
            self._treeView.on_shutdown()
            self._treeView = None

        self._timelineSub = None
        self._settings_subs = []
        self._stage_event_sub_ovd = None

safe_import_tests("omni.physxpvd.scripts.tests")
