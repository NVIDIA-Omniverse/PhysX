# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.ui
from omni.ui import scene as sc

# This class allows routing immediate mode drawing from omni.ui scene to a callback
# It's currently used as an adapter for the legacy C++ drawing code, without the need
# to link to omni.ui scene. It will be removed once all drawing code will be ported to omni.ui.scene
class ImmediateModeViewportOverlaysManipulator(sc.Manipulator):
    def __init__(self, immediate_mode_draw_cb):
        super().__init__()
        self._drawing = False
        self._scene_view = None
        self._clip_vstack = None
        self._immediate_mode_draw_cb = immediate_mode_draw_cb

    def set_scene_view(self, scene_view):
        self._scene_view = scene_view

    def set_clip_vstack(self, clip_vstack):
        self._clip_vstack = clip_vstack

    def _enable_pick(self, enable):
        if self._clip_vstack:
            self._clip_vstack.content_clipping = not enable

    def _camera_util_conformed_window(self, policy, res_aspect, proj_in):
        if policy == sc.AspectRatioPolicy.STRETCH:
            return proj_in
        def _safe_div(a,b):
            return a / b if b != 0.0 else a
        def _sign(a):
            return -1.0 if a < 0.0 else +1.0
        def _resolve_conform_window_policy(policy, fov_aspect, res_aspect):
            if policy == sc.AspectRatioPolicy.PRESERVE_ASPECT_VERTICAL or policy == sc.AspectRatioPolicy.PRESERVE_ASPECT_HORIZONTAL:
                return policy

            if (policy == sc.AspectRatioPolicy.PRESERVE_ASPECT_FIT) ^ (fov_aspect > res_aspect):
                return sc.AspectRatioPolicy.PRESERVER_ASPECT_VERTICAL
            
            return sc.AspectRatioPolicy.PRESERVE_ASPECT_HORIZONTAL

        result = sc.Matrix44(proj_in)

        proj_matrix_0_0 = proj_in[0*4 + 0]
        proj_matrix_1_1 = proj_in[1*4 + 1]

        window = [abs(proj_matrix_1_1), abs(proj_matrix_0_0)]
        fov_aspect = _safe_div(window[0], window[1])

        resolved_policy = _resolve_conform_window_policy(policy, fov_aspect, res_aspect)

        if resolved_policy == sc.AspectRatioPolicy.PRESERVE_ASPECT_HORIZONTAL:
            result[1*4+1] = _sign(proj_matrix_1_1) * window[1] * res_aspect

            scale_factor = _safe_div(result[1*4 + 1], proj_matrix_1_1)
            result[2*4 + 1] *= scale_factor
        else:
            result[0*4 + 0] = _sign(proj_matrix_0_0) * _safe_div(window[0], res_aspect)
            scale_factor = _safe_div(result[0*4 + 0], proj_matrix_0_0)
            result[2*4 + 0] *= scale_factor
            result[3*4 + 0] *= scale_factor

        return result

    # This is python port of SceneView::getAmendedProjection() const
    def _get_amended_projection(self, projection, view, width, height, aspect_ratio_policy, screen_aspect):
        need_to_fit_screen = True
        if screen_aspect <= 0:
            if height > 0:
                screen_aspect = width / height
            else:
                screen_aspect = 1
            need_to_fit_screen = False

        projection = self._camera_util_conformed_window(aspect_ratio_policy, screen_aspect, projection) 
        if need_to_fit_screen:
            widget_aspect = height / width
            projection_aspect = projection[0*4 + 0] / projection[1*4 + 1]
            projection_widget_ratio = projection_aspect / widget_aspect

            if projection_widget_ratio > 1.0:
                projection[0*4 + 0] = projection[1*4 + 1] * widget_aspect
            else:
                projection[1*4 + 1] = projection[0*4 + 0] / widget_aspect
        return projection

    def on_build(self):
        if self._scene_view:
            amended_projection = self._get_amended_projection(  self._scene_view.projection,
                                                                self._scene_view.view,
                                                                self._scene_view.computed_content_width,
                                                                self._scene_view.computed_content_height,
                                                                self._scene_view.aspect_ratio_policy,
                                                                self._scene_view.screen_aspect_ratio)
            self._immediate_mode_draw_cb(self._scene_view, self._enable_pick, amended_projection)
        if self._drawing:
            self.invalidate()

    def destroy(self):
        self._drawing = False
        self._scene_view = None
        self._clip_vstack = None
        self._immediate_mode_draw_cb = None

    def start_drawing(self):
        self._drawing = True
        self.invalidate()

class ImmediateModeViewportOverlays():
    def __init__(self, frame_name, immediate_mode_draw_cb):
        self._viewport = None
        self._scene_view = None
        self._clip_vstack = None
        self._frame_name = frame_name
        self._immediate_mode_draw_cb = immediate_mode_draw_cb
        self._manipulator = None
    
    def install_on_viewport(self, viewport):
        self._viewport = viewport
        if not self._viewport:
            return

        frame = self._viewport.get_frame(self._frame_name)
        with frame:
            self._clip_vstack = omni.ui.VStack(content_clipping=False)
            with self._clip_vstack:
                self._scene_view = omni.ui_scene.scene.SceneView()
                with self._scene_view.scene:
                    self._manipulator = ImmediateModeViewportOverlaysManipulator(self._immediate_mode_draw_cb)
        self._viewport.viewport_api.add_scene_view(self._scene_view)
        self._manipulator.set_clip_vstack(self._clip_vstack)
        self._manipulator.set_scene_view(self._scene_view)
        self._manipulator.start_drawing()

    def destroy(self):
        if self._viewport:
            if self._scene_view:
                self._viewport.viewport_api.remove_scene_view(self._scene_view)
        if self._manipulator:
            self._manipulator.destroy()
        self._manipulator = None
        self._viewport = None
        self._scene_view = None
        self._clip_vstack = None
        self._frame_name = None
        self._immediate_mode_draw_cb = None
