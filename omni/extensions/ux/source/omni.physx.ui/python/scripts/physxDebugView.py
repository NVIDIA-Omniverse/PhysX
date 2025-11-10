# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import sys
import os
from functools import partial
from pxr import UsdGeom, UsdPhysics, PhysxSchema, Usd, UsdUtils, Plug, Sdf, PhysicsSchemaTools
from omni import ui, usd
from omni.kit.window.popup_dialog import MessageDialog
import carb.settings
import carb.profiler
from carb.eventdispatcher import get_eventdispatcher
from omni.timeline import get_timeline_interface
from omni.physxuicommon.windowmenuitem import WindowMenuItem
from omni.physx import (
    get_physx_interface,
    get_physx_simulation_interface,
    get_physx_visualization_interface,
    get_physxunittests_interface,
    get_physx_property_query_interface
)
from omni.physx.bindings._physx import SimulationEvent
from omni.physx.bindings._physx import (
    SETTING_PVD_ENABLED,
    SETTING_PVD_IP_ADDRESS,
    SETTING_PVD_STREAM_TO_FILE,
    SETTING_PVD_OUTPUT_DIRECTORY,
    SETTING_PVD_PROFILE,
    SETTING_PVD_MEMORY,
    SETTING_PVD_DEBUG,
    SETTING_VISUALIZATION_GAP,
    SETTING_VISUALIZATION_COLLISION_MESH,
    SETTING_OMNIPVD_OVD_RECORDING_DIRECTORY,
    SETTING_OMNIPVD_ENABLED,
    SETTING_OMNIPVD_IS_OVD_STAGE,
    SETTING_OMNIPVD_IS_RECORDING,
    SETTING_LOG_ROBOTICS,
    SETTING_LOG_SCENEMULTIGPU,
    SETTING_DISABLE_SLEEPING,
    SETTING_ENABLE_SYNCHRONOUS_KERNEL_LAUNCHES    
)
from omni.physx.bindings._physx import PhysxPropertyQueryRigidBodyResponse, PhysxPropertyQueryColliderResponse, PhysxPropertyQueryResult
from omni.physx.scripts.physicsUtils import get_initial_collider_pairs

from omni.physxui import get_physxui_interface
from omni.kit.widget.settings.settings_widget import SettingType
from omni.kit.widget.settings.settings_model import SettingModel
import omni.physxui.scripts.utils as utils

import asyncio
from omni.kit.window.filepicker import FilePickerDialog
import omni.kit.app

LINE_HEIGHT = 23
LABEL_WIDTH = 200
SETTING_PARTICLE_SMOOTH = "/persistent/physics/visualizationDisplayParticlesParticlePositions"
SETTING_PARTICLE_RADIUS = "/persistent/physics/visualizationDisplayParticlesParticleRadius"
SETTING_PARTICLE_CLOTH_WIREFRAME = "/persistent/physics/visualizationDisplayParticlesClothMeshLines"


def _build_section(title, inner_fn, spacer=12):
    with ui.CollapsableFrame(title, height=0):
        with ui.HStack():
            ui.Spacer(width=spacer)
            inner_fn()


def _build_checkbox(*args, **kwargs):
    with ui.VStack():
        ui.Spacer()
        cb = ui.CheckBox(*args, **kwargs, height=0)
        ui.Spacer()
    return cb


def remove_schema(physx_schema_remove, physics_schema_remove, stage, paths):

    def removeAPINative(prim, api_name):
        schema_split = api_name.split(":")
        if len(schema_split) == 1:
            api = Usd.SchemaRegistry().GetTypeFromSchemaTypeName(api_name)
            prim.RemoveAPI(api)
        else:
            api_pre = schema_split[0]
            suffix = schema_split[1]
            api = Usd.SchemaRegistry().GetTypeFromSchemaTypeName(api_pre)
            prim.RemoveAPI(api, suffix)

    def removeAPI(prim, api_name):
        if not prim:
            return

        if prim.IsInstanceProxy() or prim.IsInPrototype():
            carb.log_info("Prim at " + prim.GetPath().GetText() + " is an instance proxy or is inside an instance master.")
            return

        # Get the listop at the current edit target.
        stage = prim.GetStage()
        editTarget = stage.GetEditTarget()
        primSpec = editTarget.GetPrimSpecForScenePath(prim.GetPath())

        # if not primSpec fallback to removeAPI
        if not primSpec:
            removeAPINative(prim, api_name)
        else:
            listOp = primSpec.GetInfo(Usd.Tokens.apiSchemas)

            # remove from the list
            existingApiSchemas = listOp.prependedItems

            if api_name not in existingApiSchemas:
                # if not in prepended its in explicit, that
                # we want to change to move it to prepended if thats the case
                existingApiSchemas = listOp.explicitItems
                if api_name not in existingApiSchemas:
                    removeAPINative(prim, api_name)
                    return

            existingApiSchemas.remove(api_name)
            listOp.prependedItems = existingApiSchemas

            result = listOp.ApplyOperations([])
            newListOp = Sdf.TokenListOp()
            newListOp.prependedItems = result
            primSpec.SetInfo(Usd.Tokens.apiSchemas, newListOp)

    # TODO figure out how to get this out from the schema itself
    ext_multi_api_prefixes = {
        "PhysicsLimitAPI": "limit",
        "PhysicsDriveAPI": "drive",
        "PhysxLimitAPI": "physxLimit",
        "PhysicsJointStateAPI": "state",
        "PhysxTendonAttachmentAPI": "physxTendon",
        "PhysxTendonAttachmentLeafAPI": "physxTendon",
        "PhysxTendonAttachmentRootAPI": "physxTendon",
        "PhysxTendonAxisRootAPI": "physxTendon",
        "PhysxTendonAxisAPI": "physxTendon",
        "PhysxCookedDataAPI": "physxCookedData",
        "PhysxVehicleBrakesAPI": "physxVehicleBrakes",
        "PhysxVehicleNonlinearCommandResponseAPI": "physxVehicleNCR",
    }

    for p in paths:
        selected_prim = stage.GetPrimAtPath(Sdf.Path(p))
        for prim in Usd.PrimRange(selected_prim):
            schemas = prim.GetAppliedSchemas()
            for schema in schemas:
                schema_split = schema.split(":")
                schemaToken = Usd.SchemaRegistry().GetTypeFromSchemaTypeName(schema_split[0])
                physics_plugin = Plug.Registry().GetPluginWithName("usdPhysics")
                physx_plugin = Plug.Registry().GetPluginWithName("physxSchema")
                if physx_plugin and physx_schema_remove and physx_plugin.DeclaresType(schemaToken):
                    carb.log_info("Removing PhysxSchema plugin defined schemaAPI: " + str(schema))
                    prop_list = (
                        Usd.SchemaRegistry().FindAppliedAPIPrimDefinition(schema_split[0]).GetPropertyNames()
                    )
                    if len(schema_split) == 1:
                        for prop in prop_list:
                            prim.RemoveProperty(prop)
                    else:
                        prefix = ext_multi_api_prefixes[schema_split[0]]
                        suffix = schema_split[1]
                        for prop in prop_list:
                            full_prop = prefix + ":" + suffix + ":" + prop
                            prim.RemoveProperty(full_prop)
                    removeAPI(prim, schema)

                if physics_plugin and physics_schema_remove and physics_plugin.DeclaresType(schemaToken):
                    carb.log_info("Removing UsdPhysics plugin defined schemaAPI: " + str(schema_split[0]))
                    prop_list = (
                        Usd.SchemaRegistry().FindAppliedAPIPrimDefinition(schema_split[0]).GetPropertyNames()
                    )
                    if len(schema_split) == 1:
                        for prop in prop_list:
                            prim.RemoveProperty(prop)
                    else:
                        prefix = ext_multi_api_prefixes[schema_split[0]]
                        suffix = schema_split[1]
                        for prop in prop_list:
                            full_prop = prefix + ":" + suffix + ":" + prop
                            prim.RemoveProperty(full_prop)
                    removeAPI(prim, schema)


class CollisionPairItem(ui.AbstractItem):
    def __init__(self, path0, path1):
        super().__init__()
        self.actor0_path_model = ui.SimpleStringModel(path0)
        self.actor1_path_model = ui.SimpleStringModel(path1)

    def __repr__(self):
        return f'"{self.actor0_path_model.as_string} {self.actor1_path_model.as_string}"'


class CollisionPairModel(ui.AbstractItemModel):
    def __init__(self):
        super().__init__()
        self._children = []

    def get_item_children(self, item):
        if item is not None:
            return []
        return self._children

    def get_item_value_model_count(self, item):
        return 2

    def get_item_value_model(self, item, column_id):
        if item is not None:
            return item.actor1_path_model if column_id == 1 else item.actor0_path_model

    def clear(self):
        self._children.clear()
        self._item_changed(None)

    def add_pairs(self, pairs):
        self._children = [CollisionPairItem(pair[0], pair[1]) for pair in pairs]
        self._item_changed(None)


class PhysxDebugWindow(ui.Window):
    def __init__(self):
        super().__init__("Physics Debug", ui.DockPreference.RIGHT_BOTTOM, width=640, height=480)
        self.deferred_dock_in("Property", ui.DockPolicy.CURRENT_WINDOW_IS_ACTIVE)
        self._debug_visualization_enabled = False
        self._streamToFileChb = None
        self._pvdIpAddressField = None
        self._pvdOutputFileField = None
        self._subs = []
        self._stage_event_sub = None

        self.setup()

        with self.frame:
            with ui.VStack():
                self._build_ui()

    def setup(self):
        self.vis_names = [
            ["WorldAxes", "World Axes", False],
            ["BodyAxes", "Body Axes", False],
            ["BodyMassAxes", "Body Mass Axes", False],
            ["BodyLinearVel", "Body Linear Velocity", False],
            ["BodyAngularVel", "Body Angular Velocity", False],
            ["ContactPoint", "Contact Point", False],
            ["ContactNormal", "Contact Normal", False],
            ["ContactError", "Contact Error", False],
            ["ContactImpulse", "Contact Impulse", False],
            ["FrictionPoint", "Friction Point", False],
            ["FrictionNormal", "Friction Normal", False],
            ["FrictionImpulse", "Friction Impulse", False],
            ["ActorAxes", "Actor Axes", False],
            ["CollisionAABBs", "Collision AABBs", False],
            ["CollisionShapes", "Collision Shapes", False],
            ["CollisionAxes", "Collision Axes", False],
            ["CollisionCompounds", "Collision Compounds", False],
            ["CollisionFaceNormals", "Collision Face Normals", False],
            ["CollisionEdges", "Collision Edges", False],
            ["CollisionStaticPruner", "Collision Static Pruner", False],
            ["CollisionDynamicPruner", "Collision Dynamic Pruner", False],
            ["JointLocalFrames", "Joint Local Frames", False],
            ["JointLimits", "Joint Limits", False],
            ["CullBox", "Cull Box", False],
            ["MBPRegions", "MBP Regions", False],
            ["SDFs", "SDFs", False],
        ]

    def on_shutdown(self):
        self.visible = False
        self._subs = []
        self._popup = None
        self.stage_update_button = None
        self._stage_event_sub = None

    def _build_ui(self):
        spacer_height = 5

        with ui.ScrollingFrame(
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
        ):
            with ui.VStack():
                self._build_simulation_debug_control_ui()
                ui.Spacer(height=spacer_height)
                self._build_simulation_overrides_ui()
                ui.Spacer(height=spacer_height)
                self._build_mass_information_ui()
                ui.Spacer(height=spacer_height)
                self._build_visualization_ui()
                ui.Spacer(height=spacer_height)
                self._build_deformable_visualization_property_ui()
                ui.Spacer(height=spacer_height)
                self._build_particles_visualization_property_ui()
                ui.Spacer(height=spacer_height)
                self._build_collision_mesh_visualization_property_ui()
                ui.Spacer(height=spacer_height)
                self._build_vehicle_visualization_property_ui()
                ui.Spacer(height=spacer_height)
                self._build_omnipvd_ui()
                ui.Spacer(height=1)
                self._build_pvd_ui()
                ui.Spacer(height=spacer_height)
                self._build_physics_clean_ui()
                ui.Spacer(height=spacer_height)
                self._build_logging_ui()
                ui.Spacer(height=spacer_height)
                self._build_collision_checker_ui()

    ###########################################################################
    def _build_logging_ui(self):
        settings = carb.settings.get_settings()

        def build_channel_line(channel_name, channel_setting):
            with ui.HStack(height=LINE_HEIGHT):
                ui.Label(channel_name, width=LABEL_WIDTH)
                rob_model = ui.SimpleBoolModel(settings.get(channel_setting))
                rob_model.add_value_changed_fn(lambda val: settings.set(channel_setting, val.as_bool))
                _build_checkbox(rob_model)

        def build_inner():
            with ui.VStack():
                build_channel_line("Robotics", SETTING_LOG_ROBOTICS)
                build_channel_line("Scene Multi-GPU", SETTING_LOG_SCENEMULTIGPU)

        _build_section("Logging Channels", build_inner)

    ###########################################################################
    def _build_simulation_debug_control_ui(self):
        # defaults
        self._physx_simulation_started = False
        self._physx_timestep = 1 / 60.0
        self._popup = None
        self._settings = carb.settings.get_settings()

        # simulation events callback
        def on_simulation_event(event):
            if event.type == int(SimulationEvent.RESUMED):
                if get_timeline_interface().is_playing():
                    self._play_button.text = "Pause"
                    self._physx_simulation_started = True

            if event.type == int(SimulationEvent.PAUSED):
                self._play_button.text = "Run"

            if event.type == int(SimulationEvent.STOPPED):
                self._play_button.text = "Run"
                self._physx_simulation_started = False

        events = get_physx_interface().get_simulation_event_stream_v2()
        self._subs.append(events.create_subscription_to_pop(on_simulation_event))

        # button callbacks
        def on_play_click():
            timeline = get_timeline_interface()
            if timeline.is_playing():
                timeline.pause()
            else:
                timeline.play()

        def on_stop_click():
            ti = get_timeline_interface()
            if not ti.is_stopped():
                ti.stop()
            else:
                ti.set_current_time(ti.get_start_time())
            get_physx_interface().reset_simulation()

        def on_disable_async_clicked(self, dialog: MessageDialog):
            # set all asynchronous scene updates to synchronous
            for prim in usd.get_context().get_stage().Traverse():
                if prim.HasAPI(PhysxSchema.PhysxSceneAPI):
                    scene = PhysxSchema.PhysxSceneAPI(prim)
                    sceneUpdateType = scene.GetUpdateTypeAttr().Get()
                    if sceneUpdateType == PhysxSchema.Tokens.Asynchronous:
                        scene.CreateUpdateTypeAttr().Set(PhysxSchema.Tokens.Synchronous)

            # refresh internals if needed, REGISTER_CHANGE takes care of this when timeline/simulation is running
            if get_timeline_interface().is_stopped():
                physx = get_physx_interface()
                physx.reset_simulation()
                physx.start_simulation()

            dialog.hide()

        def _create_async_dialog(self):
            resetmsg = " Simulation will also reset." if get_timeline_interface().is_stopped() else ""
            self._popup = None
            self._popup = MessageDialog(
                width=400,
                message=f"One of the PhysicsScene prims has 'asynchronous simulation' enabled. In order to step the simulation, you must disable it. Would you like to disable it now?{resetmsg}",
                ok_handler=lambda dialog: on_disable_async_clicked(self, dialog),
                ok_label="Yes",
                title="Async Warning",
                disable_cancel_button=False,
            )
            self._popup.show()
            return

        def on_step_click():
            timeline = get_timeline_interface()
            if timeline.is_playing():
                timeline.pause()

            physx = get_physx_interface()
            if not self._physx_simulation_started:
                self._physx_simulation_started = True

                physx.start_simulation()
                get_physxui_interface().enable_debug_visualization(True)
                for prim in usd.get_context().get_stage().Traverse():
                    if prim.IsA(UsdPhysics.Scene) and prim.HasAPI(PhysxSchema.PhysxSceneAPI):
                        api = PhysxSchema.PhysxSceneAPI(prim)
                        self._physx_timestep = 1 / api.GetTimeStepsPerSecondAttr().Get()
                        break

            if get_physx_interface().is_asyncsimrender_enabled():
                _create_async_dialog(self)
                return

            timeline.set_current_time(timeline.get_current_time() + self._physx_timestep)
            physx.update_simulation(self._physx_timestep, timeline.get_current_time())
            physx.update_transformations(False, True, True)

        # define ui
        def build_inner():
            with ui.HStack():
                self._play_button = ui.Button("Run", clicked_fn=on_play_click, height=25, width=70,
                        tooltip="Runs the simulation by executing the timeline play")
                self._stop_button = ui.Button("Stop", clicked_fn=on_stop_click, height=25, width=70,
                        tooltip="Stops the simulation by executing the timeline play")
                self._step_button = ui.Button("Step", clicked_fn=on_step_click, height=25, width=70,
                        tooltip="Steps the simulation only and sets new timeline time, the step size is equal to one over physics scene time steps per second")

        _build_section("Simulation Control", build_inner)

    ###########################################################################
    def _build_deformable_visualization_property_ui(self):
        def update_tet_mesh_gap_value_in_stage(gap_value: float):
            stage = usd.get_context().get_stage()
            if not stage:
                return

            # TODO: might use meta data again in the future
            # metadata = stage.GetMetadata('customLayerData')
            # if 'physicsSettings' in metadata:
            #    physicsSettings = metadata['physicsSettings']
            # else:
            #    physicsSettings = {}
            # metadata['physicsSettings'] = {"tetrahedralMeshGap": gap_value}
            # stage.SetMetadata('customLayerData', metadata)

            for prim in stage.Traverse():
                if prim.IsA(PhysxSchema.TetrahedralMesh):
                    visGapAttr = prim.GetAttribute("visualizationGap")
                    if visGapAttr:
                        visGapAttr.Set(gap_value)

        def vis_tet_mesh_gap_changed(value):
            carb.settings.get_settings().set(SETTING_VISUALIZATION_GAP, value.as_float)
            update_tet_mesh_gap_value_in_stage(value.as_float)

        # define ui
        def build_inner():
            with ui.VStack():
                with ui.HStack(height=LINE_HEIGHT):
                    ui.Label("Stage Tetrahedral Mesh Gap", width=LABEL_WIDTH)
                    fd_gap = ui.FloatDrag(width=200, min=0.0, max=1.0, step=0.01)
                    gap_value = carb.settings.get_settings().get(SETTING_VISUALIZATION_GAP)
                    fd_gap.model.set_value(gap_value)
                    fd_gap.model.add_value_changed_fn(vis_tet_mesh_gap_changed)
                    update_tet_mesh_gap_value_in_stage(gap_value)

        _build_section("Deformable Debug Visualization", build_inner)

    ###########################################################################
    def _build_particles_visualization_property_ui(self):

        pa_rad_keys = [
            "Contact Offset",
            "Rest Offset",
            "Particle Contact Offset",
            "Particle Rest Offset",
            "Anisotropy",
            "Render Geometry",
        ]

        pa_rad_default_sel = carb.settings.get_settings().get(SETTING_PARTICLE_RADIUS)

        def cb_pa_rad_item_changed(model, _):
            sel = model.get_item_value_model().as_int
            carb.settings.get_settings().set(SETTING_PARTICLE_RADIUS, sel)

        cl_mesh_default_sel = carb.settings.get_settings().get(SETTING_PARTICLE_CLOTH_WIREFRAME)

        def cb_cl_mesh_value_changed(value):
            sel = value.as_bool
            carb.settings.get_settings().set(SETTING_PARTICLE_CLOTH_WIREFRAME, sel)

        pa_smooth_keys = [
            "Sim Positions",
            "Smoothed Positions"
        ]

        pa_smoothed_default_sel = carb.settings.get_settings().get(SETTING_PARTICLE_SMOOTH)

        def cb_pa_smoothed_item_changed(model, _):
            sel = model.get_item_value_model().as_int
            carb.settings.get_settings().set(SETTING_PARTICLE_SMOOTH, sel)

        def build_inner():
            with ui.VStack():
                with ui.HStack(height=LINE_HEIGHT):
                    ui.Label("Particle Radius", width=LABEL_WIDTH)
                    ui.ComboBox(pa_rad_default_sel, *pa_rad_keys, width=200).model.add_item_changed_fn(cb_pa_rad_item_changed)
                with ui.HStack(height=LINE_HEIGHT):
                    ui.Label("Particle Positions", width=LABEL_WIDTH)
                    ui.ComboBox(pa_smoothed_default_sel, *pa_smooth_keys, width=200).model.add_item_changed_fn(cb_pa_smoothed_item_changed)
                with ui.HStack(height=LINE_HEIGHT):
                    ui.Label("Cloth Wireframe", width=LABEL_WIDTH)
                    cl_mesh_model = ui.SimpleBoolModel(cl_mesh_default_sel)
                    _build_checkbox(cl_mesh_model).model.add_value_changed_fn(cb_cl_mesh_value_changed)

        _build_section("Particles Debug Visualization", build_inner)

    ###########################################################################

    def _build_collision_mesh_visualization_property_ui(self):
        pa_rad_keys = [
            "Collision Only",
            "Graphics Only",
            "Both",
        ]
        pa_rad_values = [
            "collision_only",
            "graphics_only",
            "both",
        ]

        pa_rad_default_sel = 0
        get_physxui_interface().set_collision_mesh_type(pa_rad_values[pa_rad_default_sel])

        def cb_pa_rad_item_changed(model, _):
            sel = model.get_item_value_model().as_int
            get_physxui_interface().set_collision_mesh_type(pa_rad_values[sel])

        cl_mesh_default = carb.settings.get_settings().get(SETTING_VISUALIZATION_COLLISION_MESH)

        def cb_cl_mesh_value_changed(value):
            carb.settings.get_settings().set(SETTING_VISUALIZATION_COLLISION_MESH, value.as_bool)

        def vis_explode_view_changed(value):
            get_physxui_interface().explode_view_distance(value.as_float)

        # define ui
        def build_inner():
            with ui.VStack():
                with ui.HStack(height=LINE_HEIGHT):
                    ui.Label("Solid Collision Mesh Visualization", width=LABEL_WIDTH)
                    cl_mesh_model = ui.SimpleBoolModel(cl_mesh_default)
                    _build_checkbox(cl_mesh_model).model.add_value_changed_fn(cb_cl_mesh_value_changed)
                with ui.HStack(height=LINE_HEIGHT):
                    ui.Label("Collision Mesh Display Type", width=LABEL_WIDTH)
                    ui.ComboBox(pa_rad_default_sel, *pa_rad_keys, width=200).model.add_item_changed_fn(cb_pa_rad_item_changed)
                with ui.HStack(height=LINE_HEIGHT):
                    ui.Label("Explode View Distance", width=LABEL_WIDTH)
                    fd_gap = ui.FloatDrag(width=200, min=0.0, max=4.0, step=0.01)
                    gap_value = 0
                    fd_gap.model.set_value(gap_value)
                    fd_gap.model.add_value_changed_fn(vis_explode_view_changed)

        _build_section("Collision Mesh Debug Visualization", build_inner)

    ###########################################################################
    def _build_vehicle_visualization_property_ui(self):
        vehicleDebugIdSuspension = "Suspension"

        def cb_vh_suspension_value_changed(value):
            get_physxui_interface().set_vehicle_visualization(vehicleDebugIdSuspension, value.as_bool)

        # define ui
        def build_inner():
            with ui.VStack():
                ui.Spacer(height=10)
                with ui.HStack(height=0):
                    ui.Label("Suspension", width=LABEL_WIDTH)
                    chb = _build_checkbox()
                    enabled = get_physxui_interface().get_vehicle_visualization(vehicleDebugIdSuspension)
                    chb.model.set_value(enabled)
                    chb.model.add_value_changed_fn(cb_vh_suspension_value_changed)
                ui.Spacer(height=10)

        _build_section("Vehicle Debug Visualization", build_inner)

    ###########################################################################
    def _build_simulation_overrides_ui(self):
        # force/release buttons
        def on_force_click():
            get_physx_interface().force_load_physics_from_usd()

        def on_release_click():
            get_physx_interface().release_physics_objects()

        self.stage_update_button = None
        self.stage_update_attached = True

        def on_stage_node_click():
            if self.stage_update_attached:
                # detach from stage update
                get_physx_simulation_interface().attach_stage(0)
                self.stage_update_attached = False
                self.stage_update_button.text = "Attach OmniPhysX StageUpdateNode"
            else:
                # attach back
                get_physx_simulation_interface().detach_stage()
                self.stage_update_attached = True
                self.stage_update_button.text = "Detach OmniPhysX StageUpdateNode"

        # gpu/cpu setting
        gpu_keys = ["Schema Based", "Force GPU", "Force CPU"]
        gpu_values = [-1, 1, 0]
        gpu_default_index = gpu_values.index(get_physx_interface().get_overwrite_gpu_setting())

        def gpu_cb_item_changed(model, _):
            sel = model.get_item_value_model().as_int
            get_physx_interface().overwrite_gpu_setting(gpu_values[sel])

        # solver type
        solver_keys = ["Schema Based", "Force PGS", "Force TGS"]
        solver_values = [-1, 0, 1]

        def solver_cb_item_changed(model, _):
            sel = model.get_item_value_model().as_int
            get_physx_interface().overwrite_solver_type(solver_values[sel])

        # define ui
        def build_inner():
            with ui.VStack():
                ui.Button("Force Physics USD Load", clicked_fn=on_force_click, height=25, width=170)
                ui.Button("Release Physics Objects", clicked_fn=on_release_click, height=25, width=170)
                self.stage_update_button = ui.Button("Detach OmniPhysX StageUpdateNode", clicked_fn=on_stage_node_click, height=25, width=170)
                with ui.HStack(height=LINE_HEIGHT):
                    ui.Label("GPU/CPU Settings", width=LABEL_WIDTH)
                    ui.ComboBox(gpu_default_index, *gpu_keys, width=200).model.add_item_changed_fn(gpu_cb_item_changed)
                with ui.HStack(height=LINE_HEIGHT):
                    ui.Label("Solver Type", width=LABEL_WIDTH)
                    ui.ComboBox(-1, *solver_keys, width=200).model.add_item_changed_fn(solver_cb_item_changed)
                with ui.HStack(height=LINE_HEIGHT):
                    settings = carb.settings.get_settings()
                    ui.Label("Disable Sleeping", width=LABEL_WIDTH)
                    rob_model = ui.SimpleBoolModel(settings.get(SETTING_DISABLE_SLEEPING))
                    rob_model.add_value_changed_fn(lambda val: settings.set(SETTING_DISABLE_SLEEPING, val.as_bool))
                    _build_checkbox(rob_model)
                with ui.HStack(height=LINE_HEIGHT):
                    settings = carb.settings.get_settings()
                    ui.Label("Enable Synchronous CUDA Kernel Launches", width=LABEL_WIDTH * 1.3)
                    rob_model = ui.SimpleBoolModel(settings.get(SETTING_ENABLE_SYNCHRONOUS_KERNEL_LAUNCHES))
                    rob_model.add_value_changed_fn(lambda val: settings.set(SETTING_ENABLE_SYNCHRONOUS_KERNEL_LAUNCHES, val.as_bool))
                    _build_checkbox(rob_model)

        _build_section("Simulation Overrides", build_inner)

    ###########################################################################
    def _build_visualization_ui(self):
        def build_inner():
            with ui.VStack():
                self._build_visualization_base_settings_ui()
                self._build_visualization_toggles()

        _build_section("Simulation Debug Visualization", build_inner)

    def _build_mass_information_ui(self):
        def on_mass_info():
            selection = usd.get_context().get_selection()
            paths = list(selection.get_selected_prim_paths())
            print("Printing out mass information for prims:")
            for p in paths:
                selected_prim = usd.get_context().get_stage().GetPrimAtPath(p)
                for prim in Usd.PrimRange(selected_prim):
                    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                        mass = get_physxunittests_interface().get_mass_information(str(prim.GetPrimPath()))
                        if mass["mass"] > 0.0:
                            print("Prim: " + str(prim.GetPrimPath()) + " Mass Info: " + str(mass))
                        else:
                            print("Mass information not available, please make sure the simulation is running.")
                            return

        def on_query_rigid_body():
            selection = usd.get_context().get_selection()
            paths = list(selection.get_selected_prim_paths())
            def rigid_info_received(rigid_info : PhysxPropertyQueryRigidBodyResponse):
                print("[Found a rigid body - printing data]")
                if rigid_info.result == PhysxPropertyQueryResult.VALID:
                    rb_prim_path = PhysicsSchemaTools.intToSdfPath(rigid_info.path_id)
                    print("Prim: " + str(rb_prim_path) + "\nRigid Body Info:" +
                            "\n\tmass:" + str(rigid_info.mass) +
                            "\n\tinertia:" + str(rigid_info.inertia) +
                            "\n\tcenter of mass:" + str(rigid_info.center_of_mass) +
                            "\n\tprincipal axes:" + str(rigid_info.principal_axes))
                else:
                    print(f"Error while waiting for query to finish code={rigid_info.result}")
            def collider_info_received(collider_info : PhysxPropertyQueryColliderResponse):
                print("[Found a collider - printing data]")
                if collider_info.result == PhysxPropertyQueryResult.VALID:
                    collider_prim_path = PhysicsSchemaTools.intToSdfPath(collider_info.path_id)
                    print("Prim: " + str(collider_prim_path) + "\nCollider Info:" +
                            "\n\tAABB Min:" + str(collider_info.aabb_local_min) +
                            "\n\tAABB Max:" + str(collider_info.aabb_local_max) +
                            "\n\tVolume:"   + str(collider_info.volume) +
                            "\n\tLocal Position:"   + str(collider_info.local_pos) +
                            "\n\tLocal Rotation:"   + str(collider_info.local_rot)
                            )
                else:
                    print(f"Error while waiting for query to finish code={collider_info.result}")
            stage_id = UsdUtils.StageCache.Get().Insert(usd.get_context().get_stage()).ToLongInt()
            for p in paths:
                selected_prim = usd.get_context().get_stage().GetPrimAtPath(p)
                for prim in Usd.PrimRange(selected_prim):
                    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                        body_path = PhysicsSchemaTools.sdfPathToInt(prim.GetPrimPath())
                        get_physx_property_query_interface().query_prim(stage_id = stage_id,
                                                                        prim_id = body_path,
                                                                        rigid_body_fn = rigid_info_received,
                                                                        collider_fn = collider_info_received,
                                                                        timeout_ms = 2000) # Timeout after 2 sec


        def build_inner():
            with ui.VStack():
                button = ui.Button("Get mass information for selected prims and its sub-hierarchies", height=30, width=200)
                button.set_clicked_fn(on_mass_info)
                # For now we want to keep it under development mode only
                if carb.settings.get_settings().get("physics/developmentMode"):
                    button = ui.Button("Query rigid body", height=30, width=200)
                    button.set_clicked_fn(on_query_rigid_body)
                ui.Spacer(height=5)

        _build_section("Mass Information", build_inner)

    def _build_visualization_base_settings_ui(self):

        self.debug_visualization_scale_meters_per_unit = -1.0

        # widget callbacks
        def vis_toggle_changed(value):
            self._debug_visualization_enabled = value.as_bool
            get_physxui_interface().enable_debug_visualization(self._debug_visualization_enabled)
            get_physx_visualization_interface().enable_visualization(self._debug_visualization_enabled)
            self._build_visualization_toggles()

        def vis_distance_changed(value):
            get_physxui_interface().set_visualization_distance(value.as_float)

        def vis_scale_changed(value):
            get_physx_visualization_interface().set_visualization_scale(value.as_float)
            stage = usd.get_context().get_stage()
            if stage:
                # Store the meters per unit we have adjusted this scale to.
                self.debug_visualization_scale_meters_per_unit = UsdGeom.GetStageMetersPerUnit(stage)

        def get_default_scale():
            stage = usd.get_context().get_stage()
            if stage:
                meters_per_unit = 1.0 / UsdGeom.GetStageMetersPerUnit(stage)
                return 1.0 + ((meters_per_unit - 1) * 0.3)
            return 1.0

        # define ui with change callbacks
        def build_inner():
            # define ui
            with ui.VStack():
                with ui.HStack(height=LINE_HEIGHT):
                    ui.Label("Enabled", width=LABEL_WIDTH)
                    chb = _build_checkbox()
                    chb.model.set_value(self._debug_visualization_enabled)
                    chb.model.add_value_changed_fn(vis_toggle_changed)
                with ui.HStack(height=LINE_HEIGHT):
                    ui.Label("Visibility Maximum Distance", width=LABEL_WIDTH)
                    fd_dist = ui.FloatDrag(width=200, min=1, max=sys.float_info.max, step=0.1)
                    fd_dist.model.add_value_changed_fn(vis_distance_changed)
                    fd_dist.model.set_value(1000)
                with ui.HStack(height=LINE_HEIGHT):
                    ui.Label("Scale", width=LABEL_WIDTH)
                    fd_scale = ui.FloatDrag(width=200, min=1, max=sys.float_info.max, step=0.1)
                    fd_scale.model.add_value_changed_fn(vis_scale_changed)
                    fd_scale.model.set_value(get_default_scale())
                self._toggles_frame = ui.Frame()

            # scale callback to reset it on stage opened
            @carb.profiler.profile
            def on_stage_opened_event():
                stage = usd.get_context().get_stage()
                if stage and self.debug_visualization_scale_meters_per_unit != UsdGeom.GetStageMetersPerUnit(stage):
                    self.debug_visualization_scale_meters_per_unit = UsdGeom.GetStageMetersPerUnit(stage)
                    default = get_default_scale()
                    get_physxui_interface().set_visualization_distance(default)
                    fd_scale.model.set_value(default)

            usd_context = omni.usd.get_context()
            self._subs.append(
                get_eventdispatcher().observe_event(
                    observer_name="omni.physx.ui:PhysxDebugView",
                    event_name=usd_context.stage_event_name(omni.usd.StageEventType.OPENED),
                    on_event=lambda _: on_stage_opened_event()
                )
            )

            # resend settings on resume
            def on_simulation_event(event):
                if event.type == int(SimulationEvent.RESUMED):
                    get_physxui_interface().enable_debug_visualization(chb.model.as_bool)
                    get_physx_visualization_interface().enable_visualization(chb.model.as_bool)
                    get_physxui_interface().set_visualization_distance(fd_dist.model.as_float)
                    get_physx_visualization_interface().set_visualization_scale(fd_scale.model.as_float)

            events = get_physx_interface().get_simulation_event_stream_v2()
            self._subs.append(events.create_subscription_to_pop(on_simulation_event))

        build_inner()

    def _build_visualization_toggles(self):
        def on_toggle_changed(index, value):
            get_physx_visualization_interface().set_visualization_parameter(self.vis_names[index][0], value.as_bool)
            self.vis_names[index][2] = value.as_bool

        # define ui
        with self._toggles_frame:
            if self._debug_visualization_enabled:
                with ui.VStack():
                    ui.Label("Show:", height=LINE_HEIGHT)
                    ui.Spacer(height=5)
                    index = 0
                    for name in self.vis_names:
                        with ui.HStack(height=20):
                            chb = ui.CheckBox(width=30)
                            chb.model.add_value_changed_fn(partial(on_toggle_changed, index))
                            ui.Label(name[1])
                            chb.model.set_value(name[2])
                            get_physx_visualization_interface().set_visualization_parameter(name[0], name[2])
                        index = index + 1
            else:
                ui.VStack()

    ###########################################################################
    def _build_omnipvd_ui(self):
        def cleanup_fp_dialog(fp_dialog):
            async def cleanup_async(fp_dialog):
                await omni.kit.app.get_app().next_update_async()
                fp_dialog.destroy()
            asyncio.ensure_future(cleanup_async(fp_dialog))

        def define_output_dir():
            def on_choose(fp_dialog, filename, dirpath):
                fp_dialog.hide()
                if (dirpath):
                    if not dirpath.endswith("/"):
                        dirpath += "/"
                    # Set the OmniPVD output dir here, needs new var, outputdir or similar
                    carb.settings.get_settings().set(SETTING_OMNIPVD_OVD_RECORDING_DIRECTORY, dirpath)
                cleanup_fp_dialog(fp_dialog)

            item_filter_options = ["OVD Files (*.ovd)"]
            fp_dialog = FilePickerDialog(
                "Select the OVD Output Directory",
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
            with ui.VStack():
                for setting in omnipvd_settings:
                    with ui.HStack(height=LINE_HEIGHT):
                        ui.Label(setting[1], width=LABEL_WIDTH)
                        if setting[0] == SettingType.BOOL:
                            cb = _build_checkbox(SettingModel(setting[2]))
                            if setting[2] == SETTING_OMNIPVD_ENABLED:
                                self._omniPvdEnabledCheckbox = cb
                        elif setting[0] == SettingType.STRING:
                            ui.StringField(SettingModel(setting[2]), height=20)

                with ui.HStack(height=LINE_HEIGHT):
                    button = ui.Button("Set Recording Directory", height=30, width=200)
                    button.set_clicked_fn(define_output_dir)
                    with ui.VStack(height=LINE_HEIGHT):
                        ui.Spacer(height=4)
                        ui.StringField(SettingModel(SETTING_OMNIPVD_OVD_RECORDING_DIRECTORY), height=22, enabled = False)
                ui.Spacer(height=10)

        _build_section("OmniPVD", build_inner)

        self._omniPvdEnabledCheckbox.enabled = not get_physx_interface().is_running()

        # Disallow switching omniPVD on and off while sim is running or the stage is an OmniPVD stage
        def on_simulation_event_omnipvd_enable(event):
            if event.type == int(SimulationEvent.RESUMED):
                self._omniPvdEnabledCheckbox.enabled = False
            elif event.type == int(SimulationEvent.STOPPED):
                if not carb.settings.get_settings().get_as_bool(SETTING_OMNIPVD_IS_OVD_STAGE):
                    self._omniPvdEnabledCheckbox.enabled = True

        def on_stage_event_opened_omnipvd_enable():
            # check the stage if OmniPVD stage?
            is_OVD_stage = False
            ctx = omni.usd.get_context()
            if ctx:
                stage = ctx.get_stage()
                if stage:
                    prim = stage.GetPrimAtPath("/scenes")
                    if not prim:
                        prim = stage.GetPrimAtPath("/shared")
                    if prim:
                        if prim.HasAttribute("omni:pvdi:class"):
                            is_OVD_stage = True
            if not is_OVD_stage:
                self._omniPvdEnabledCheckbox.enabled = True
                carb.settings.get_settings().set(SETTING_OMNIPVD_IS_OVD_STAGE, False)
            else:
                self._omniPvdEnabledCheckbox.enabled = False
                carb.settings.get_settings().set(SETTING_OMNIPVD_IS_OVD_STAGE, True)

        carb.settings.get_settings().set(SETTING_OMNIPVD_IS_OVD_STAGE, False)
        physxInterface = get_physx_interface()
        self._omniPvdEventSubcription = physxInterface.get_simulation_event_stream_v2().create_subscription_to_pop(
            on_simulation_event_omnipvd_enable
        )

        usd_context = omni.usd.get_context()
        self._stage_event_sub = get_eventdispatcher().observe_event(
            observer_name="omni.physx.ui:PhysxDebugView",
            event_name=usd_context.stage_event_name(omni.usd.StageEventType.OPENED),
            on_event=lambda _: on_stage_event_opened_omnipvd_enable()
        )

    ###########################################################################
    def _build_pvd_ui(self):
        ENABLED_STYLE = {"color": 0xFFCCCCCC}
        DISABLED_STYLE = {"color": 0xFF888888}

        # widget callbacks
        def stream_to_file_toggle_changed(value):
            if value.as_bool:
                if self._pvdIpAddressField is not None:
                    self._pvdIpAddressField.enabled = False
                    self._pvdIpAddressField.set_style(DISABLED_STYLE)
                if self._pvdOutputFileField is not None:
                    self._pvdOutputFileField.enabled = True
                    self._pvdOutputFileField.set_style(ENABLED_STYLE)
            else:
                if self._pvdIpAddressField is not None:
                    self._pvdIpAddressField.enabled = True
                    self._pvdIpAddressField.set_style(ENABLED_STYLE)
                if self._pvdOutputFileField is not None:
                    self._pvdOutputFileField.enabled = False
                    self._pvdOutputFileField.set_style(DISABLED_STYLE)

        pvd_settings = [
            [SettingType.BOOL, "PVD Enabled", SETTING_PVD_ENABLED],
            [SettingType.BOOL, "Stream to File", SETTING_PVD_STREAM_TO_FILE],
            [SettingType.STRING, "Output Directory", SETTING_PVD_OUTPUT_DIRECTORY],
            [SettingType.STRING, "IP Address", SETTING_PVD_IP_ADDRESS],
            [SettingType.BOOL, "Profile Information", SETTING_PVD_PROFILE],
            [SettingType.BOOL, "Memory Information", SETTING_PVD_MEMORY],
            [SettingType.BOOL, "Debug Information (Object Data)", SETTING_PVD_DEBUG],
        ]

        def on_click():
            get_physx_interface().reconnect_pvd()

        def on_click_repX():
            utils.save_to_repX()

        def build_inner():

            with ui.VStack():
                for setting in pvd_settings:
                    with ui.HStack(height=LINE_HEIGHT):
                        ui.Label(setting[1], width=LABEL_WIDTH)
                        if setting[0] == SettingType.BOOL:
                            chb = _build_checkbox(SettingModel(setting[2]))
                            if setting[2] == SETTING_PVD_STREAM_TO_FILE:
                                chb.model.add_value_changed_fn(stream_to_file_toggle_changed)
                                self._streamToFileChb = chb
                            if setting[2] == SETTING_PVD_ENABLED:
                                self._enableLegacyPvdCheckbox = chb
                        elif setting[0] == SettingType.STRING:
                            strField = ui.StringField(SettingModel(setting[2]), height=20)
                            # Store the output file and ip address edit box references to allow
                            # toggling them off and on
                            if setting[2] == SETTING_PVD_OUTPUT_DIRECTORY:
                                self._pvdOutputFileField = strField
                                # Set default output directory dynamically if not already set
                                outPath = carb.settings.get_settings().get(SETTING_PVD_OUTPUT_DIRECTORY)
                                if not outPath:
                                    #print("outFile was NOT set, setting default and creating temp dir..")
                                    outDirectory = (
                                        carb.tokens.get_tokens_interface().resolve("${omni_documents}") + "/pvd/out"
                                    )
                                    if not os.path.exists(outDirectory):
                                        os.makedirs(outDirectory)
                                    outPath = outDirectory
                                    carb.settings.get_settings().set(SETTING_PVD_OUTPUT_DIRECTORY, outPath)
                                #else:
                                #    print("output directory was already set to " + outPath)
                            if setting[2] == SETTING_PVD_IP_ADDRESS:
                                self._pvdIpAddressField = strField
                button = ui.Button("Reconnect", height=30, width=200)
                button.set_clicked_fn(on_click)

                button = ui.Button("Save scene to repX", height=30, width=200)
                button.set_clicked_fn(on_click_repX)
                ui.Spacer(height=7)

        _build_section("Legacy PVD", build_inner)
        physxRunning = False

        self._enableLegacyPvdCheckbox.enabled = not get_physx_interface().is_running()

        stream_to_file_toggle_changed(self._streamToFileChb.model)

        # Disallow switching legacy pvd on and off while sim is running
        def on_simulation_event_legacy_pvd_enable(event):
            if event.type == int(SimulationEvent.RESUMED):
                self._enableLegacyPvdCheckbox.enabled = False
            elif event.type == int(SimulationEvent.STOPPED):
                self._enableLegacyPvdCheckbox.enabled = True

        physxInterface = get_physx_interface()
        self._legacyPvdEventSubcription = physxInterface.get_simulation_event_stream_v2().create_subscription_to_pop(
            on_simulation_event_legacy_pvd_enable
        )



    ###########################################################################
    def _build_physics_clean_ui(self):
        def on_physx_remove():
            remove_schema(True, False, usd.get_context().get_stage(), list(usd.get_context().get_selection().get_selected_prim_paths()))

        def on_physics_remove():
            remove_schema(True, True, usd.get_context().get_stage(), list(usd.get_context().get_selection().get_selected_prim_paths()))

        def build_inner():
            with ui.VStack():
                button = ui.Button("Remove PhysicsSchema and PhysxSchema on selection", height=30, width=200)
                button.set_clicked_fn(on_physics_remove)
                ui.Spacer(height=5)
                button = ui.Button("Remove PhysxSchema on selection", height=30, width=200)
                button.set_clicked_fn(on_physx_remove)

        _build_section("Physics schema removal", build_inner)

    def _build_collision_checker_ui(self):
        def on_check_collisions():
            stage = usd.get_context().get_stage()
            collider_pairs = get_initial_collider_pairs(stage)  

            if len(collider_pairs) > 0:
                # Update the tree view with the collision pairs
                self._collision_tree.model.clear()
                self._collision_tree.model.add_pairs(collider_pairs)
            else:
                print("No contacts found")
                # Clear the tree view if no contacts were found
                self._collision_tree.model.clear()
        
        def build_inner():
            with ui.VStack():
                button = ui.Button("Check Initial Collisions", height=30, width=200)
                button.set_clicked_fn(on_check_collisions)
                # Add a tree view to display collision pairs
                self._collision_model = CollisionPairModel()
                self._collision_tree = ui.TreeView(self._collision_model)

        _build_section("Collision Checker", build_inner)


class PhysXDebugView:
    def __init__(self, spawn_immediately):
        self._menu = WindowMenuItem("Debug", lambda: PhysxDebugWindow(), spawn_immediately)

    def on_shutdown(self):
        self._menu.on_shutdown()
        self._menu = None
