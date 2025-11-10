# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from pathlib import Path
import carb
import carb.tokens
import carb.windowing
import sys
import traceback
import omni.appwindow
import omni.kit.test
import omni.ui as ui
import inspect
import pathlib
import carb.settings
import omni.usd
import asyncio
import time
from omni.timeline import get_timeline_interface
from omni.physx import get_physx_interface
from omni.physxtests.utils.physicsBase import PhysicsKitStageAsyncTestCase, PhysicsBaseAsyncTestCase
from omni.physx.bindings._physx import SETTING_MIN_FRAME_RATE_DEFAULT, SETTING_DISPLAY_JOINTS, SETTING_MASS_DISTRIBUTION_MANIPULATOR, SETTING_DISPLAY_MASS_PROPERTIES
from omni.physxui.scripts.physicsViewportMassEdit import SETTING_DISPLAY_MASS_PROPERTIES_NONE, SETTING_DISPLAY_MASS_PROPERTIES_SELECTED, SETTING_DISPLAY_MASS_PROPERTIES_ALL
from omni.physx.bindings._physx import SETTING_VISUALIZATION_COLLISION_MESH
import omni.kit.viewport.utility as viewport_utils


APP_ROOT = Path(carb.tokens.get_tokens_interface().resolve("${app}"))
OUTPUTS_DIR = APP_ROOT.parent.parent.parent.joinpath("outputs")
FILE_ROOT = Path(__file__).parent.parent.parent.parent
GOLDEN_DIR = FILE_ROOT.joinpath("data/")

settings = carb.settings.get_settings()


class CompareError(Exception):
    pass


class TestCase(PhysicsKitStageAsyncTestCase):
    def __init__(self, tests=()):
        super().__init__(tests)
        self._saved_width = None
        self._saved_height = None
        self._saved_maximized = None
        self._saved_fullscreen = None
        self._restore_window = None
        self._restore_position = None
        self._restore_dock_window = None
        self._restore_dock_position = None
        self._layout_dump = None
        self._image_paths = set()

        self._viewport_resolution_settings = {}
        self._viewport_data = None

        # Settings that should be applied for this test. By using dictionaries, derived tests can override these values.
        self._base_settings = {
            "/app/window/scaleToMonitor" : (False, True),
            "/app/window/dpiScaleOverride" : (1.0, -1.0),
            "/exts/omni.kit.notification_manager/disable_notifications" : (True, False),
            "/app/profilerBackend" : ("", ""),
            "/app/file/ignoreUnsavedOnExit" : (True, False),
            "/exts/omni.kit.viewport.window/startup/windowName" : ("Viewport", "Viewport"), # Required whenever we run tests in VP 2 only.
        }

        self._viewport_settings = {
            "/persistent/app/viewport/displayOptions" : ((1 << 7) | (1 << 10), (1 << 7) | (1 << 10)),
            "/persistent/exts/omni.kit.manipulator.transform/manipulator/intersectionThickness" :  (10, 10),
            "/persistent/exts/omni.kit.manipulator.transform/manipulator/scaleMultiplier" :  (1.4, 1.4),
            "/app/runLoops/main/rateLimitFrequency" : (60, 60),
            SETTING_MIN_FRAME_RATE_DEFAULT : (60, 60),
            SETTING_DISPLAY_JOINTS : (False, True),
            "/app/viewport/grid/enabled" : (False, True),
            "/app/viewport/snapEnabled" : (False, False),
            "/app/asyncRendering" : (False, True),
            "/app/captureFrame/setAlphaTo1" : (True, False),            
            "/persistent/app/viewport/gizmo/constantScaleCamera" : (False, False),
            "/persistent/app/viewport/gizmo/constantScaleEnabled" : (True, True),
            "/persistent/app/viewport/gizmo/constantScale" : (10.0, 10.0),
            "/persistent/app/viewport/gizmo/scale" : (1.0, 1.0),
            "/persistent/app/viewport/gizmo/lineWidth" : (10, 10),
            "/persistent/app/viewport/gizmo/minFadeOut" : (1, 1),
            "/persistent/app/viewport/gizmo/maxFadeOut" : (50, 50),
            "/app/transform/operation" : ("select", "select"),
            "/app/transform/moveMode" : ("global", "global"),
            "/app/transform/rotateMode" : ("global", "global"),
            "/metricsAssembler/showUnitsOverlay" : (False, True),
            SETTING_DISPLAY_MASS_PROPERTIES : (SETTING_DISPLAY_MASS_PROPERTIES_NONE, SETTING_DISPLAY_MASS_PROPERTIES_NONE),
            SETTING_MASS_DISTRIBUTION_MANIPULATOR : (False, False),
            "/renderer/enabled" : ("rtx", "rtx"),
            "/renderer/active" : ("rtx", "rtx"),
            "/rtx/materialDb/syncLoads" : (True, True),
            "/rtx/hydra/materialSyncLoads" : (True, False),
        }

        self._render_settings = {
            "/rtx-transient/resourcemanager/enableTextureStreaming": (False, True),
            "/rtx-transient/directLighting/sampledLighting/enforced": (False, True),
            "/rtx/ecoMode/enabled": (False, False),
            "/rtx/sceneDB/ambientLightIntensity": (0.0, 0.0),
            "/rtx/directLighting/sampledLighting/enabled": (False, False),
            "/rtx/directLighting/sampledLighting/autoEnable": (False, True),
            "/rtx/newDenoiser/enabled": (False, True), 
            "/rtx/indirectDiffuse/enabled": (False, True),
            "/rtx/post/aa/op" : (0, 0),
            "/rtx/shadows/enabled" : (False, True),
            "/rtx/reflections/enabled" : (False, True),
            "/rtx/ambientOcclusion/enabled" : (False, True),
            "/rtx/post/tonemap/op" : (1, 6),
            "/rtx-transient/post/aa/limitedOps" : (False, False),
            "/rtx/pathtracing/lightcache/cached/enabled" : (False, True),
            "/rtx/raytracing/lightcache/spatialCache/enabled" : (False, True),
            "/ngx/enabled": (False, True),
        }

        self._distant_light_setting = "/rtx/useViewLightingMode"

        self._settings_cache = {}

    def _setup_settings(self, settings_dict):
        for name, value in settings_dict.items():
            if settings.get(name) != value[0]:
                self._settings_cache[name] = settings.get(name)
                settings.set(name, value[0])

    async def _setup_base_settings(self):
        self._setup_settings(self._base_settings)
        await omni.kit.app.get_app().next_update_async()

    async def _setup_viewport_settings(self):
        self._layout_dump = ui.Workspace.dump_workspace()
        self._setup_settings(self._viewport_settings)
        await omni.kit.app.get_app().next_update_async()

    async def _setup_render_settings(self, use_distant_light):
        self._render_settings.update({self._distant_light_setting : (use_distant_light, False)})

        self._setup_settings(self._render_settings)
        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()

    async def _restore_settings(self):
        if len(self._settings_cache) == 0:
            return

        def restore(settings_dict):
            for name, value in settings_dict.items():
                try:
                    cache = self._settings_cache[name]
                    settings.set(name, cache if cache is not None else value[1])
                except KeyError:
                    pass

        restore(self._base_settings)
        restore(self._viewport_settings)
        restore(self._render_settings)

        if self._viewport_resolution_settings:
            restore(self._viewport_resolution_settings)

        if self._layout_dump is not None:
            ui.Workspace.restore_workspace(self._layout_dump)

        self._settings_cache.clear()
        await omni.kit.app.get_app().next_update_async()

    async def _setup_window(self, width, height):
        app_window = omni.appwindow.get_default_app_window()
        dpi_scale = ui.Workspace.get_dpi_scale()

        width_with_dpi = int(width * dpi_scale)
        height_with_dpi = int(height * dpi_scale)

        current_width = app_window.get_width()
        current_height = app_window.get_height()

        if width_with_dpi == current_width and height_with_dpi == current_height:
            self._saved_width = None
            self._saved_height = None
        else:
            self._saved_width = current_width
            self._saved_height = current_height
            if app_window.is_maximized():
                self._saved_maximized = True
                app_window.restore_window()
            elif app_window.is_fullscreen():
                self._saved_fullscreen = True
                app_window.set_fullscreen(False)
            app_window.resize(width_with_dpi, height_with_dpi)
            await app_window.get_window_resize_event_stream().next_event()

        # Move the cursor away to avoid hovering on element and trigger tooltips that break the tests
        windowing = carb.windowing.acquire_windowing_interface()
        os_window = app_window.get_window()
        windowing.set_cursor_position(os_window, (0, 0))

    def show_all_vp_menus(self, show):
        def _show(names, show):
            settings = carb.settings.get_settings()
            for name in names:
                settings.set(f"/persistent/exts/omni.kit.viewport.menubar.{name}/visible", show)

        _show(["settings", "camera", "display", "render"], show)

    def force_resize_window(self, window_name, width, height):
        window = ui.Workspace.get_window(window_name)
        window.width = width
        window.height = height

        app_window = omni.appwindow.get_default_app_window()
        dpi_scale = ui.Workspace.get_dpi_scale()
        width_with_dpi = int(width * dpi_scale)
        height_with_dpi = int(height * dpi_scale)
        app_window.resize(width_with_dpi, height_with_dpi)

    async def _setup_docked(self, window, restore_dock_window=None, width=800, height=600, restore_dock_position=ui.DockPosition.SAME, no_mouse=True):
        window.undock()
        await omni.kit.app.get_app().next_update_async()
        window.focus()

        self._restore_flags = window.flags
        self._restore_window = window
        self._restore_position = window.width, window.height, window.position_x, window.position_y
        self._restore_dock_window = restore_dock_window
        self._restore_dock_position = restore_dock_position

        window.flags = window.flags | ui.WINDOW_FLAGS_NO_SCROLLBAR | ui.WINDOW_FLAGS_NO_TITLE_BAR | ui.WINDOW_FLAGS_NO_RESIZE
        if no_mouse:
            window.flags |= ui.WINDOW_FLAGS_NO_MOUSE_INPUTS
        window.width = width
        window.height = height
        window.position_x = 0
        window.position_y = 0

        await self._setup_window(width, height)        

    async def _restore_windows(self):
        if self._saved_width is not None and self._saved_height is not None:
            app_window = omni.appwindow.get_default_app_window()
            if self._saved_maximized:
                self._saved_maximized = False
                app_window.maximize_window()
            elif self._saved_fullscreen:
                self._saved_fullscreen = False
                app_window.set_fullscreen(True)
            else:
                app_window.resize(self._saved_width, self._saved_height)
            await app_window.get_window_resize_event_stream().next_event()
            self._saved_width = None
            self._saved_height = None

        if self._restore_window:
            self._restore_window.flags = self._restore_flags
            self._restore_window.width = self._restore_position[0]
            self._restore_window.height = self._restore_position[1]
            self._restore_window.position_x = self._restore_position[2]
            self._restore_window.position_y = self._restore_position[3]

            if self._restore_dock_window:
                self._restore_window.dock_in(self._restore_dock_window, self._restore_dock_position)

            self._restore_window = None
            self._restore_dock_window = None

    async def restore(self):
        await self._restore_windows()
        await self._restore_settings()

    async def setup_viewport_test(self, width=800, height=600, window_name=None):
        await self._setup_base_settings()
        await self._setup_viewport_settings()
        await self._setup_window(width, height)
        self._restore_window = None
        self._restore_dock_window = None

        self._viewport_resolution_settings = {
            "/app/renderer/resolution/width" : (width, width),
            "/app/renderer/resolution/height" : (height, height),
            "/app/renderer/resolution/multiplier" : (1, 1),
        }
        self._viewport_data = (width, height, window_name)
        self._setup_settings(self._viewport_resolution_settings)

    async def setup_docked_test(self, window_name, window_restore=None, width=800, height=600):
        window = ui.Workspace.get_window(window_name)
        restore_window = ui.Workspace.get_window(window_restore) if window_restore else None
        await self._setup_base_settings()
        await self._setup_docked(window, restore_window, width, height)

    async def step(self, num_steps=1, dt=1.0 / 60.0, precise=False, stop_timeline_after=False):
        timeline_iface = get_timeline_interface()

        if precise:
            physx_iface = get_physx_interface()
            stepping_time = 0.0
            physx_iface.start_simulation()
            await omni.kit.app.get_app().next_update_async()
            for _ in range(num_steps):
                stepping_time = stepping_time + dt
                timeline_iface.set_current_time(stepping_time)
                physx_iface.update_simulation(dt, stepping_time)
                physx_iface.update_transformations(False, True, True)
                await omni.kit.app.get_app().next_update_async()
        else:
            timeline_iface.play()
            await omni.kit.app.get_app().next_update_async()
            for _ in range(num_steps):
                await omni.kit.app.get_app().next_update_async()

        if stop_timeline_after:
            timeline_iface.stop()
        else:
            timeline_iface.pause()

    def get_img_name(self, img_name, img_suffix):
        return f"{img_name if img_name is not None else inspect.stack()[2][3]}{img_suffix}.png"

    async def do_visual_test(
        self,
        threshold=1e-4,
        inverse=False,
        img_name=None,
        img_suffix="",
        skip_assert=False,
        use_distant_light=True,
        use_renderer_capture=False,
        setup_and_restore=True
    ):
        def compare_ge(a, b):
            return a >= b

        def compare_le(a, b):
            return a <= b

        compare_op = compare_le if inverse else compare_ge

        if setup_and_restore:
            await self._setup_render_settings(use_distant_light)

        img_name = self.get_img_name(img_name, img_suffix)
        diff = await self.capture_and_compare(img_name, threshold, compare_op, self._viewport_data, use_renderer_capture)
        op_str = ">" if inverse else "<"
        message = f"{img_name}: difference ratio {diff:.6f} {op_str} {threshold} threshold"
        carb.log_warn(message) if compare_op(diff, threshold) else carb.log_info(message)

        if setup_and_restore:
            await self.restore()

        res = diff is not None and not compare_op(diff, threshold)
        if not skip_assert:
            self.assertTrue(res, f"The image doesn't match the golden one: {img_name}")
        return res

    async def do_capture(self, img_name=None, img_suffix="", img_artifact_path=""):
        image_path = self.get_img_name(img_name, img_suffix)
        await self.capture(image_path)
        await self.restore()

    def compare(self, image1, image2, image_diffmap, threshold, compare_op):
        if not image1.exists():
            raise CompareError(f"File image1 {image1} does not exist")
        if not image2.exists():
            raise CompareError(f"File image2 {image2} does not exist")

        if "PIL" not in sys.modules.keys():
            try:
                from PIL import Image
            except ImportError:
                import omni.kit.pipapi
                omni.kit.pipapi.install("Pillow", module="PIL")

        from PIL import Image, ImageChops, ImageStat

        original = Image.open(str(image1))
        contrast = Image.open(str(image2))

        if original.size != contrast.size:
            raise CompareError(
                f"[omni.ui.test] Can't compare different resolutions\n\n"
                f"{image1} {original.size[0]}x{original.size[1]}\n"
                f"{image2} {contrast.size[0]}x{contrast.size[1]}\n\n"
                f"It's possible that your monitor DPI is not 100%.\n\n"
            )

        difference = ImageChops.difference(original, contrast).convert("RGB")
        stat = ImageStat.Stat(difference)
        diff_ratio = sum(stat.mean) / (len(stat.mean) * 255)

        if compare_op(diff_ratio, threshold):
            difference = difference.point(lambda i: min(i * 255, 255))
            difference.save(str(image_diffmap))
            carb.log_warn(f"Saved diffmap to {image_diffmap}")
        return diff_ratio

    async def capture(self, image_res_path, viewport_data, use_renderer_capture=False, skip_duplicate_check=False):
        import omni

        if image_res_path in self._image_paths and not skip_duplicate_check:
            carb.log_error(f"Capture target {image_res_path} was already used for this test!")
        else:
            self._image_paths.add(image_res_path)

        image_path = Path(image_res_path).resolve().absolute()

        async def check_if_file_exists():
            if image_path.exists():
                carb.log_info(f"{image_path} exists immediately")
            else:
                exists = False
                for i in range(0, exists_timeout):
                    if image_path.exists():
                        exists = True
                        break
                    await omni.kit.app.get_app().next_update_async()
                if exists:
                    carb.log_info(f"{image_path} appeared in {i} frames")
                else:
                    carb.log_warn(f"{image_path} didn't appear in {i} frames")

        if viewport_data and not use_renderer_capture:
            viewport_api = viewport_utils.get_viewport_from_window_name(viewport_data[2])
            viewport_api.resolution = (viewport_data[0], viewport_data[1])
            viewport_api.resolution_scale = 1
            await viewport_utils.next_viewport_frame_async(viewport_api)

            # TODO: rewrite this to capture_viewport_to_buffer so we control when the file has finished writing to disk
            cap_obj = viewport_utils.capture_viewport_to_file(viewport_api, file_path=str(image_res_path))

            # waiting with future wait for early exit, cap_obj.wait_for_result waits in a fixed manner
            capture_timeout = 1   # seconds
            exists_timeout = 256  # frames
            try:
                start_time = time.time()
                await asyncio.wait_for(cap_obj._Capture__future, capture_timeout)
                await check_if_file_exists()
                await cap_obj.wait_for_result(completion_frames=5)
                end_time = time.time()
                elapsed_time = end_time - start_time
                carb.log_info(f"{image_path} captured in {elapsed_time}")
            except asyncio.exceptions.TimeoutError:
                carb.log_warn(f"{image_res_path} not captured in {capture_timeout}s!")
        else:
            import omni.renderer_capture
            omni.renderer_capture.acquire_renderer_capture_interface().capture_next_frame_swapchain(str(image_res_path))
            await omni.kit.app.get_app().next_update_async()
            omni.renderer_capture.acquire_renderer_capture_interface().wait_async_capture()
            await omni.kit.app.get_app().next_update_async()
            await check_if_file_exists()

    async def capture_and_compare(self, image_name, threshold, compare_op, viewport_data, use_renderer_capture = False):
        image_artifact_path = ""
        image_res_name = image_name
        image_res_path = OUTPUTS_DIR.joinpath(image_res_name)

        image_gld_name = image_name
        image_gld_path = GOLDEN_DIR.joinpath(image_gld_name)

        image_diffmap_name = f"{Path(image_res_name).stem}.diffmap.png"
        image_diffmap_path = OUTPUTS_DIR.joinpath(image_diffmap_name)

        attempts = 5
        for i in range(attempts):
            await self.capture(image_res_path, viewport_data, use_renderer_capture, i > 0)
            carb.log_info(f"Capturing {image_res_path} and comparing with {image_gld_path}. Attempt {i+1} of {attempts}.")

            try:
                diff_ratio = self.compare(image_res_path, image_gld_path, image_diffmap_path, threshold, compare_op)
                return diff_ratio
            except CompareError as e:
                carb.log_error(f"Failed to compare images for {image_name}. Error: {e}")
                return 0
            except OSError as e:
                carb.log_warn(f"OSError: {str(e)}")
            except SyntaxError as e:
                carb.log_warn(f"SyntaxError: {str(e)}")

        carb.log_error(f"Failed to compare images for {image_name}. Quitting after {attempts} attempts.")
        return 0
