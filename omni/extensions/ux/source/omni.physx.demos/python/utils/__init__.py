# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import importlib
from omni import ui
import omni.kit.app
import omni.timeline
import carb
import carb.settings
from omni.physxui.scripts.utils import cleanup_fp_dialog
from omni.kit.window.filepicker import FilePickerDialog
from pxr import Gf, UsdGeom, UsdPhysics
from omni.physxdemos.utils.room_helper import RoomHelper, RoomInstancer
import types
import queue
from omni.kit.viewport.utility.camera_state import ViewportCameraState
import omni.physx.bindings._physx as pb
from omni.physx.scripts import assets_paths


class Categories:
    SNIPPETS = TUTORIALS = "Uncategorized"
    SAMPLES = DEMOS = "Uncategorized"
    PSCL = "PSCL"
    INTERNAL = INTERNAL_SAMPLES = INTERNAL_DEMOS = "Internal"
    PREVIEW = "Previews"
    ###
    NONE = "None"
    BASICS = "Basics"
    RIGID_BODIES = "Rigid Bodies"
    MATERIALS = "Materials"
    SIMULATION_PART = "Simulation Partitioning"
    CONTACTS = "Contacts"
    TRIGGERS = "Triggers"
    SCENE_QUERY = "Scene Query"
    JOINTS = "Joints"
    ARTICULATIONS = "Articulations"
    CCT = "Character Controller"
    VEHICLES = "Vehicles"
    DEFORMABLES = "Deformables"
    PARTICLES = "Particles"
    COMPLEX_SHOWCASES = "Complex Showcases"
    FORCE_FIELDS = "Force Fields"
    BENCHMARKS = "Benchmarks"

CATEGORY_ORDER = [
    Categories.BASICS,
    Categories.RIGID_BODIES,
    Categories.MATERIALS,
    Categories.SIMULATION_PART,
    Categories.CONTACTS,
    Categories.TRIGGERS,
    Categories.SCENE_QUERY,
    Categories.JOINTS,
    Categories.ARTICULATIONS,
    Categories.CCT,
    Categories.VEHICLES,
    Categories.DEFORMABLES,
    Categories.PARTICLES,
    Categories.FORCE_FIELDS,
    Categories.COMPLEX_SHOWCASES,
    Categories.BENCHMARKS
]


class Base:
    title = ""
    category = Categories.NONE
    short_description = ""
    description = ""
    params = {}
    tags = []

    """
    Configurable params are defined in a dictionary as e.g.
    params = {"length": IntParam(4, 0, 20, 1)}
    and then need to be added to the create method
    def create(self, stage, length)
    """

    kit_settings = {}

    """
    Dictionary of demo-specific kit settings that are applied through the carb.settings interface
    in the first stage update after the demo's create method is called.

    The kit settings are automatically reset after the demo's on_shutdown is called.

    Example:
    import omni.physx.bindings._physx as physx_settings_bindings
    class SomeDemo(demo.Base):
        title = "Some Demo"
        ...

        kit_settings = {
            "persistent/app/viewport/displayOptions": demo.get_viewport_minimal_display_options_int(),
            physx_settings_bindings.SETTING_UPDATE_VELOCITIES_TO_USD: False,
            "rtx/post/dlss/execMode": 0,  # set DLSS to performance
        }
    """

    demo_camera = ""


    """
    Invoke kit's autofocus at the first update to center the scene on whatever was created and with a custom zoom
    """
    autofocus = False
    autofocus_zoom = 0.45

    """
    Sdf.Path or path string to UsdGeom.Camera to set the viewport active camera to.

    The camera is set in the first stage update after the demo's create method is called.
    (Kit cannot handle the camera change in the create method.)
    """

    demo_base_usd_url = ""

    """
    Path / url to base USD layer to load instead of empty new stage. The base layer may provide scene-specific render/physics parameters in its metadata.
    """

    """
    Main create function called after a new stage is created.
"""
    def create(self, stage):
        pass

    """
    Update called with each Kit update.
    """
    def update(self, stage, dt, viewport, physxIFace):
        pass

    """
    Called before a new stage is created for the demo, and before create.
    """
    def on_startup(self):
        pass

    """
    Called on shutdown.
    """
    def on_shutdown(self):
        pass


class _Param():
    def __init__(self, val, name=None):
        self.val = val
        self.name = name


class _RangeParam(_Param):
    def __init__(self, val, min, max, step, name=None):
        _Param.__init__(self, val, name)
        self.min = min
        self.max = max
        self.step = step


class _ImplicitBoolModel(ui.SimpleBoolModel):
    def __init__(self, val):
        ui.SimpleBoolModel.__init__(self, val)

    def get_value(self):
        return self.get_value_as_bool()


class _ImplicitIntModel(ui.SimpleIntModel):
    def __init__(self, val):
        ui.SimpleIntModel.__init__(self, val)

    def get_value(self):
        return self.get_value_as_int()


class _ImplicitFloatModel(ui.SimpleFloatModel):
    def __init__(self, val):
        ui.SimpleFloatModel.__init__(self, val)

    def get_value(self):
        return self.get_value_as_float()


class _ImplicitStringModel(ui.SimpleStringModel):
    def __init__(self, val):
        ui.SimpleStringModel.__init__(self, val)

    def get_value(self):
        return self.get_value_as_string()


class FloatParam(_RangeParam):
    def __init__(self, val, min, max, step, name=None):
        _RangeParam.__init__(self, val, min, max, step, name)

    def build_control(self):
        model = _ImplicitFloatModel(self.val)
        model.set_min(self.min)
        model.set_max(self.max)
        ui.FloatDrag(model, min=self.min, max=self.max, step=self.step)
        return model


class IntParam(_RangeParam):
    def __init__(self, val, min, max, step, name=None):
        _RangeParam.__init__(self, val, min, max, step, name)

    def build_control(self):
        model = _ImplicitIntModel(self.val)
        model.set_min(self.min)
        model.set_max(self.max)
        ui.IntDrag(model, min=self.min, max=self.max, step=self.step)
        return model


class CheckboxParam(_Param):
    def build_control(self):
        model = _ImplicitBoolModel(self.val)
        with ui.HStack():
            ui.Spacer(width=ui.Fraction(1))
            ui.CheckBox(model, width=ui.Fraction(0.1))
            ui.Spacer(width=ui.Fraction(1))
        return model

class FileParam(_Param):
    def build_control(self):
        with ui.HStack(height=20):
            model = _ImplicitStringModel(self.val)
            ui.StringField(model)

            def on_open(dialog: FilePickerDialog, filename: str, dirname: str):
                dialog.hide()
                fullpath = f"{dirname}/{filename}" if dirname else filename
                model.set_value(fullpath)
                cleanup_fp_dialog(dialog)

            def on_choose():
                item_filter_options = ["USD Files (*.usd, *.usda, *.usdc, *.usdz)", "All Files (*)"]
                dialog = FilePickerDialog(
                    "Open File",
                    apply_button_label="Open",
                    click_apply_handler=lambda filename, dirname: on_open(dialog, filename, dirname),
                    item_filter_options=item_filter_options,
                )

            ui.Button("Choose", width=100, clicked_fn=on_choose)
            return model

class BoolParam(CheckboxParam):
    ...


class AsyncDemoBase(Base):
    """
    Base class for demos that use async sim and might need to run physics step callbacks that interface with the sim
    through the omni.physics.tensor API.

    The base class automatically subscribes to physics step, physics event, and timeline callbacks.

    The demo may override the public methods below, and may configure enabling the omni.physx.tensors and
    omni.physx.fabric extensions by passing flags to the constructor.

    Important notes:
    - Make sure not to forget to call this base class constructor in your derived demo using super().__init__(...)
    - In the demo on_shutdown method:
        1. specifically release any views that you may create in on_tensor_start with self._some_view = None
        2. call this base class shutdown to cleanup properly with super().on_shutdown() at the end of your method
    - Be careful not to write to USD from the async called on_physics_step. Use the class attribute queue to communicate
    between threads.
    """

    def on_physics_step(self, dt):
        """
        This method is called on each physics step callback, and the first callback is issued
        after the on_tensor_start method is called if the tensor API is enabled.
        """
        pass

    def on_tensor_start(self, tensorApi: types.ModuleType):
        """
        This method is called when
            1. the tensor API is enabled, and
            2. when the simulation data is ready for the user to setup views using the tensor API.
        """
        pass

    def on_timeline_event(self, e):
        """
        This method is called on timeline events. Note that in async sim, the STOP event is issued before the end
        of the simulation is awaited. Use on_simualtion_event and checking for

          e.type == int(omni.physx.bindings._physx.SimulationEvent.STOPPED)

        instead if you need to do operations that are illegal during simulation (e.g. actor removal).
        """
        pass

    def on_simulation_event(self, e):
        """
        This method is called on simulation events. See omni.physx.bindings._physx.SimulationEvent.
        """
        pass

    def on_shutdown(self):
        """
        Make sure to call this from your optional on_shutdown override with super().on_shutdown() for proper cleanup.
        """
        # unsubscribe
        self._timeline_sub = None
        self._physics_update_sub = None
        self._simulation_event_subscription = None

        # disable/enable tensor and fabric ext if needed
        manager = omni.kit.app.get_app().get_extension_manager()
        manager.set_extension_enabled_immediate("omni.physx.tensors", self._tensorapi_was_enabled)
        self._tensor_api = None
        manager.set_extension_enabled_immediate("omni.physx.fabric", self._fabric_was_enabled)

    def __init__(self, enable_tensor_api=False, enable_fabric=False, fabric_compatible=True):
        """
        Use the constructor arguments to configure what additional extensions are loaded
        and made available to the demo. If fabric_compatible=False then enable_fabric=True is ignored.
        If fabric_compatible=True and enable_fabric=False, the fabric extension is not unloaded.

        Call from your derived class constructor, for example like this:
            def __init__(self):
                super().__init__(enable_tensor_api=True, enable_fabric=False)
        """
        super().__init__()
        self._is_stopped = True
        self._tensor_started = False
        self._tensor_api = None
        self._fabric_was_enabled = False
        self._tensorapi_was_enabled = False
        self.queue = queue.Queue()

        # setup subscriptions:
        self._setup_callbacks()

        self._get_tensor_api_state()
        if enable_tensor_api:
            self._enable_tensor_api()

        if not fabric_compatible:
            enable_fabric = False

        self._enable_fabric(enable_fabric, fabric_compatible)

    @staticmethod
    def flush_fifo_queue(fifo_queue: queue.Queue):
        """
        Flush the demo Python FIFO queue.Queue (self.demo_queue) and return last element and number of elements that
        were in the queue. (A queue is suitable for passing data between the async called physics step callback and
        callbacks that are on the main thread)

        Returns:
            last_element, num_elements
        """
        last_element = None
        num_elements = 0
        try:
            while True:
                last_element = fifo_queue.get(block=False)
                num_elements += 1
        except queue.Empty:
            pass  # do nothing, exception is just signaling queue empty after getting all elements

        return last_element, num_elements

    @property
    def tensor_api(self):
        return self._tensor_api

    # "PRIVATE" METHODS #
    def _can_callback_physics_step(self) -> bool:
        if self._is_stopped:
            return False

        if self._tensor_started or self._tensor_api is None:
            return True

        self._tensor_started = True
        self.on_tensor_start(self._tensor_api)
        return True

    def _on_timeline_event(self, e):
        if e.type == int(omni.timeline.TimelineEventType.STOP):
            self._is_stopped = True
            self._tensor_started = False
        if e.type == int(omni.timeline.TimelineEventType.PLAY):
            self._is_stopped = False

        # call user implementation
        self.on_timeline_event(e)

    def _on_physics_step(self, dt):
        if not self._can_callback_physics_step():
            return

        # call user implementation
        self.on_physics_step(dt)

    def _setup_callbacks(self):
        stream = omni.timeline.get_timeline_interface().get_timeline_event_stream()
        self._timeline_sub = stream.create_subscription_to_pop(self._on_timeline_event)
        # subscribe to Physics updates:
        self._physics_update_sub = omni.physx.get_physx_interface().subscribe_physics_step_events(self._on_physics_step)
        events = omni.physx.get_physx_interface().get_simulation_event_stream_v2()
        self._simulation_event_subscription = events.create_subscription_to_pop(self.on_simulation_event)

    def _get_tensor_api_state(self):
        manager = omni.kit.app.get_app().get_extension_manager()
        self._tensorapi_was_enabled = manager.is_extension_enabled("omni.physx.tensors")

    def _enable_tensor_api(self):
        if not self._tensorapi_was_enabled:
            enable_extension("omni.physx.tensors")
        self._tensor_api = importlib.import_module("omni.physics.tensors")

    def _enable_fabric(self, enable_fabric, fabric_compatible):
        manager = omni.kit.app.get_app().get_extension_manager()
        self._fabric_was_enabled = manager.is_extension_enabled("omni.physx.fabric")
        if enable_fabric and not self._fabric_was_enabled:
            enable_extension("omni.physx.fabric")
        if not fabric_compatible and self._fabric_was_enabled:
            manager.set_extension_enabled_immediate("omni.physx.fabric", False)

def get_demo_asset_path(resource_folder: str, resource_relative_path: str) -> str:
    return assets_paths.get_asset_path(pb.SETTING_DEMO_ASSETS_PATH, resource_folder, resource_relative_path)

def get_viewport_minimal_display_options_int() -> int:
    """
    Helper that returns viewport options for max. FPS performance.
    Disables all viewport viz options except showing meshes, fps, and gpu mem
    """
           # meshes  | gpu mem   | fps
    return (1 << 10) | (1 << 13) | 1

def get_demo_room(demoInstance, stage, **kwargs):
    return RoomHelper(demoInstance, stage, **kwargs)

def get_room_instancer(stage):
    return RoomInstancer(stage)

def get_room_helper_class():
    return RoomHelper

def get_hit_color():
    return Gf.Vec3f(0.0, 1.0, 0.0)

def get_static_color(index = 0.0):
    colors = [
        Gf.Vec3f(165.0, 21.0, 21.0)/255.0,
        Gf.Vec3f(165.0, 80.0, 21.0)/255.0
    ]

    return colors[0]*(1.0-index) + colors[1]*index

def get_primary_color(index = 0.0):
    colors = [
        Gf.Vec3f(30.0, 60.0, 255.0)/255.0,
        Gf.Vec3f(200.0, 25.0, 255.0)/255.0
    ]

    return colors[0]*(1.0-index) + colors[1]*index

def setup_physics_scene(demoInstance, stage, primPathAsString = True, metersPerUnit = 0.01, gravityMod = 1.0, upAxis = UsdGeom.Tokens.z):
    try:
        currentDemoParams = demoInstance.demoParams
    except AttributeError:
        currentDemoParams = None

    defaultPrimPath = stage.GetDefaultPrim().GetPath()
    if currentDemoParams:
        defaultPrimPath = defaultPrimPath.AppendPath(currentDemoParams["rootPath"])

    UsdGeom.SetStageUpAxis(stage, upAxis)
    UsdGeom.SetStageMetersPerUnit(stage, metersPerUnit)

    gravityDir = Gf.Vec3f(0.0, 0.0, -1.0)
    if upAxis == UsdGeom.Tokens.y:
        gravityDir = Gf.Vec3f(0.0, -1.0, 0.0)
    if upAxis == UsdGeom.Tokens.x:
        gravityDir = Gf.Vec3f(-1.0, 0.0, 0.0)

    scene = UsdPhysics.Scene.Define(stage, defaultPrimPath.AppendPath("physicsScene"))
    scene.CreateGravityDirectionAttr().Set(gravityDir)
    scene.CreateGravityMagnitudeAttr().Set(9.81 * gravityMod / metersPerUnit)

    if primPathAsString:
        defaultPrimPath = str(defaultPrimPath)

    return (defaultPrimPath, scene)

def animate_attributes(demoInstance, stage, timeStep=0):
    if not getattr(demoInstance, "_animationData", None):
        return

    # if we want to animate the scene independent of the timeline, set manualAnimation "True"

    manualAnimation = demoInstance._animationData["manualAnimation"]

    if "maxStep" in demoInstance._animationData:
        maxStep = demoInstance._animationData["maxStep"]
    else:
        maxStep = 0

    if "timeCodesPerSecond" in demoInstance._animationData:
        timeCodesPerSecond = demoInstance._animationData["timeCodesPerSecond"]
    else:
        timeCodesPerSecond = 0

    keyframes = demoInstance._animationData["keyframes"]

    if timeCodesPerSecond > 0:
        timeStepScaled = (timeStep*timeCodesPerSecond/60.0)
    else:
        timeStepScaled = timeStep

    if maxStep > 0:
        curStep = timeStepScaled % (maxStep + 1)
    else:
        curStep = timeStepScaled

    if manualAnimation:
        for item in keyframes:
            op = item["op"]
            totalItems = len(item["times"])
            for i in range(totalItems):
                itemTime = item["times"][i]
                itemValue = item["values"][i]

                if i == (totalItems - 1):
                    nextIndex = i
                else:
                    nextIndex = i + 1

                nextItemTime = item["times"][nextIndex]
                nextItemValue = item["values"][nextIndex]

                if (curStep >= itemTime) and (curStep < nextItemTime):
                    lerpVal = (curStep - itemTime) / (nextItemTime - itemTime)
                    if type(itemValue) == type(True): # Booleans cannot be interpolated
                        newVal = itemValue if (lerpVal < 1.0) else nextItemValue
                    else:
                        newVal = (1.0-lerpVal)*itemValue + lerpVal*nextItemValue

                    op.Set(newVal)
    else:
        if timeCodesPerSecond > 0:
            stage.SetTimeCodesPerSecond(timeCodesPerSecond)
        for item in keyframes:
            op = item["op"]
            for i in range(len(item["times"])):
                itemTime = item["times"][i]
                itemValue = item["values"][i]
                op.Set(time=itemTime, value=itemValue)


def enable_extension_with_check(ext_id):
    manager = omni.kit.app.get_app().get_extension_manager()
    was_enabled = manager.is_extension_enabled(ext_id)
    if not was_enabled:
        enable_extension(ext_id)
    return was_enabled


def enable_extension(ext_id):
    manager = omni.kit.app.get_app().get_extension_manager()
    if not manager.set_extension_enabled_immediate(ext_id, True):
        raise Exception(f"Cannot enable {ext_id}! The extension is not available in your local installation and could not be downloaded from the online repository (see potential errors above).")


def disable_extension(ext_id):
    manager = omni.kit.app.get_app().get_extension_manager()
    manager.set_extension_enabled_immediate(ext_id, False)

"""
Set camera's postion Can write also to a specific camera type in USD.

Args:
    position: New camera position.
    target: New camera target.
    radius: New camera radius.
    name: Choose from "Perspective", "Front", "Top", "Right".
    stage: Pass USD stage when you want to write the data to "cameraSettings" in customLayerData.
"""
def reset_builtin_camera(position, target, radius, name = "Perspective", stage = None):
    camera_paths = {
        "Perspective": "/OmniverseKit_Persp",
        "Top": "/OmniverseKit_Top",
        "Front": "/OmniverseKit_Front",
        "Right": "/OmniverseKit_Right",
    }

    path = camera_paths[name]
    state = ViewportCameraState(path)

    state.set_position_world(position, False)
    state.set_target_world(target, True)

    if stage:
        cam_key = f"cameraSettings:{name}"
        stage.SetMetadataByDictKey("customLayerData", f"{cam_key}:position", Gf.Vec3d(*position))
        stage.SetMetadataByDictKey("customLayerData", f"{cam_key}:target", Gf.Vec3d(*target))
        stage.SetMetadataByDictKey("customLayerData", f"{cam_key}:radius", radius)
