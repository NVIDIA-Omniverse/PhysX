# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import carb
import carb.settings
from pxr import Tf
import re, os
from omni.physx import get_physx_interface
from pxr import Usd, UsdUtils, Gf, UsdPhysics
from omni import ui
import asyncio
import omni.client
from omni.kit.window.popup_dialog import MessageDialog
from omni.kit.window.filepicker import FilePickerDialog
from omni.kit.commands import execute
from omni.physx.scripts import utils
import omni.kit.undo

dialog = None


def cleanup_fp_dialog(fp_dialog):
    async def cleanup_async(fp_dialog):
        await omni.kit.app.get_app().next_update_async()
        fp_dialog.destroy()
    asyncio.ensure_future(cleanup_async(fp_dialog))


def run_backward_compat_on_folder(stage_url=None):
    def on_choose(fp_dialog, filename, dirpath):
        fp_dialog.hide()
        asyncio.ensure_future(run_backward_compat_on_folder_async(dirpath))
        cleanup_fp_dialog(fp_dialog)

    item_filter_options = ["USD Files (*.usd, *.usda, *.usdc, *.usdz)", "All Files (*)"]
    fp_dialog = FilePickerDialog(
        "Choose Folder",
        apply_button_label="Choose",
        click_apply_handler=lambda filename, dirname: on_choose(fp_dialog, filename, dirname),
        item_filter_options=item_filter_options,
    )

    fp_dialog.show(stage_url)


def save_to_repX():
    def on_choose(fp_dialog, filename, dirpath):
        fp_dialog.hide()
        repx_path = os.path.join(dirpath, filename + ".repx")            
        print("Writing to repX: " + repx_path)
        if not get_physx_interface().save_scene_to_repx(repx_path):
            print("Write to repX failed.")
        else:
            print("Write to repX succeeded.")
        cleanup_fp_dialog(fp_dialog)

    item_filter_options = ["RepX Files (*.repX)", "All Files (*)"]
    fp_dialog = FilePickerDialog(
        "Choose File",
        apply_button_label="Save",
        click_apply_handler=lambda filename, dirname: on_choose(fp_dialog, filename, dirname),
        item_filter_options=item_filter_options,
    )

    fp_dialog.show()


async def run_backward_compat_on_folder_async(dirpath):
    carb.log_warn("Running backward compatibility.")
    if not dirpath:
        carb.log_error("Folder not defined!")
        return

    cancelled = False

    def on_cancel():
        nonlocal cancelled
        cancelled = True

    prompt = BackwardCompatProgress(on_cancel)

    files = await list_files(dirpath)
    fixed_num = 0
    total_num = 0
    last_pct = 0
    ok_files = []
    ro_files = []
    for i, (path, entry) in enumerate(files):
        if cancelled:
            carb.log_warn("Cancelled!")
            break

        curr_pct = (i + 1) / len(files)
        if (curr_pct - last_pct) > 0.01:
            last_pct = curr_pct
            prompt.progress.set_value(curr_pct)
            await omni.kit.app.get_app().next_update_async()

        ext = path.split(".")[-1]
        if ext not in ["usd", "usda", "usdc", "usdz"]:
            continue

        carb.log_warn(f"Checking {path}")

        try:
            stage = Usd.Stage.Open(path)
        except Tf.ErrorException:
            carb.log_error("Can't open file. Not a valid format?")
            continue

        cache = UsdUtils.StageCache.Get()
        cache.Insert(stage)
        stage_id = cache.GetId(stage).ToLongInt()
        res = get_physx_interface().check_backwards_compatibility(stage_id)
        if res:
            total_num += 1
            check_log = get_physx_interface().get_backwards_compatibility_check_log()
            carb.log_warn(check_log)
            if (entry.flags & omni.client.ItemFlags.WRITEABLE_FILE) > 0:
                carb.log_warn("Deprecated schema found.")
                get_physx_interface().run_backwards_compatibility(stage_id)
                try:
                    stage.Save()
                    fixed_num += 1
                    ok_files.append(path)
                    carb.log_warn("Fixed and resaved.")
                except Tf.ErrorException as err:
                    err_files = re.findall("Unable to open file '(.*)' for writing'", str(err))
                    if err_files:
                        for err_file in err_files:
                            carb.log_error(f"Unable to open '{err_file}' for writing!")
                            ro_files.append(err_file)
                    else:
                        carb.log_error(str(err))
            else:
                carb.log_error(f"Unable to open '{path}' for writing!")
                ro_files.append(path)
        cache.Erase(stage)
        stage = None
    if not cancelled:
        carb.log_warn("Done!")

    if ok_files:
        msg = "\n".join([file for file in ok_files])
        carb.log_warn(f"\nFixed files:\n{msg}")

    if ro_files:
        msg = "\n".join([file for file in ro_files])
        carb.log_warn(f"\nNon-writeable files:\n{msg}")

    info_msg = ""
    if total_num > 0:
        info_msg = f"Fixed {fixed_num} out of {total_num} files with a deprecated schema."
        carb.log_warn(info_msg)
        if fixed_num != total_num:
            err_msg = "Some files were not writeable! Please check console log, fix manually and rerun."
            carb.log_error(err_msg)
            info_msg += "\n" + err_msg
    else:
        info_msg = "No file with a deprecated schema found."
        carb.log_warn(info_msg)

    prompt.progress.set_value(curr_pct)
    await omni.kit.app.get_app().next_update_async()
    prompt.hide()

    if not cancelled:
        failed = fixed_num != total_num
        if not failed:
            has_deprecated = get_physx_interface().check_backwards_compatibility()
            if has_deprecated:
                log = get_physx_interface().get_backwards_compatibility_check_log()
                carb.log_warn(f"Check log:\n{log}")
                info_msg += "\nSome files still have a deprecated schema! Please check console log and fix manually."
                failed = True
        global dialog
        ending = "unsuccessful" if failed else "finished"
        dialog = MessageDialog(
            title="Error" if failed else "Info",
            width=600,
            parent=prompt._window.frame,
            message=f"Backward compatibility check on folder {ending}. {info_msg}",
            ok_label="Ok",
            ok_handler=lambda _: dialog.hide(),
            disable_cancel_button=True,
        )
        dialog.show(offset_x=-1, offset_y=-24)


async def list_files(dirpath):
    paths = [dirpath]
    files = []

    while paths:
        tasks = [omni.client.list_async(path) for path in paths]
        results = await asyncio.gather(*tasks, return_exceptions=True)

        next_paths = []
        for i, (result, entries) in enumerate(results):
            parent_path = paths[i]
            if result == omni.client.Result.OK:
                for entry in entries:
                    full_path = f"{parent_path}/{entry.relative_path}"
                    is_folder = (entry.flags & omni.client.ItemFlags.CAN_HAVE_CHILDREN) > 0
                    if is_folder:
                        next_paths.append(full_path)
                    else:
                        files.append((full_path, entry))
        paths = next_paths

    return files


class ProgressModel(ui.AbstractValueModel):
    def __init__(self):
        super().__init__()
        self._value = 0.0

    def set_value(self, value):
        value = float(value)
        if value != self._value:
            self._value = value
            self._value_changed()

    def get_value_as_float(self):
        return self._value

    def get_value_as_string(self):
        return str(int(self._value * 100)) + "%"


class BackwardCompatProgress:
    def __init__(self, on_cancel_fn):
        self._window = ui.Window("Physics Backward Compatibility Check", visible=True, width=320, height=0, dockPreference=ui.DockPreference.DISABLED)
        self._window.flags = (ui.WINDOW_FLAGS_NO_COLLAPSE | ui.WINDOW_FLAGS_NO_SCROLLBAR | ui.WINDOW_FLAGS_NO_RESIZE | ui.WINDOW_FLAGS_NO_MOVE | ui.WINDOW_FLAGS_MODAL)
        self._skip_visibility_check = False

        with self._window.frame:
            with ui.VStack(height=0):
                with ui.ZStack(width=300):
                    ui.Spacer()
                    self.progress = ProgressModel()
                    ui.ProgressBar(self.progress, width=300, style={"color": 0xFFFF9E3D})
                    ui.Spacer()
                with ui.HStack():
                    ui.Spacer()
                    self.cancel_button = ui.Button("Cancel", width=100, clicked_fn=on_cancel_fn)
                    ui.Spacer()

        def visibility_changed(visible):
            if not visible and not self._skip_visibility_check:
                on_cancel_fn()

        self._window.set_visibility_changed_fn(visibility_changed)

    def hide(self):
        self._skip_visibility_check = True
        self._window.visible = False


class BackwardCompatReadOnlyPrompt:
    def __init__(self):
        self._window = ui.Window("Physics Backward Compatibility Check", visible=False, width=320, height=0, dockPreference=ui.DockPreference.DISABLED)
        self._window.flags = (ui.WINDOW_FLAGS_NO_COLLAPSE | ui.WINDOW_FLAGS_NO_SCROLLBAR | ui.WINDOW_FLAGS_NO_RESIZE | ui.WINDOW_FLAGS_NO_MOVE | ui.WINDOW_FLAGS_MODAL)
        self.result = -1

    def build_ui(self, filename):
        self.result = -1

        def on_no():
            self.result = 0
            self.hide()

        def on_yes():
            self.result = 1
            self.hide()

        def on_yes_all():
            self.result = 2
            self.hide()

        with self._window.frame:
            with ui.VStack(height=0):
                ui.Spacer(width=0, height=10)
                with ui.HStack(height=0):
                    ui.Spacer()
                    text = f"Can't open file {filename} for writing. Remove write protection?"
                    ui.Label(text, word_wrap=True, width=self._window.width - 80, height=0)
                    ui.Spacer()
                with ui.HStack(height=0):
                    ui.Spacer(height=0)
                    ui.Button("Yes", width=60, height=0).set_clicked_fn(on_yes)
                    ui.Button("Yes to All", width=60, height=0).set_clicked_fn(on_yes_all)
                    ui.Button("No", width=60, height=0).set_clicked_fn(on_no)
                    ui.Spacer(height=0)
                ui.Spacer(width=0, height=10)

        self._window.visible = True

    async def show(self, filename):
        finished = asyncio.Future()
        self.build_ui(filename)

        def visibility_changed(visible):
            if not visible:
                nonlocal finished
                finished.set_result(True)

        self._window.set_visibility_changed_fn(visibility_changed)
        await asyncio.wait_for(finished, None)

    def hide(self):
        self._window.visible = False


FOLDER_NOTE = "Preferences/Physics/Run Backwards Compatibility On Folder."


class BackwardCompatPrompt:
    def __init__(self, text, check_log, folder=False, title="Physics Backward Compatibility Check"):
        self._window = ui.Window(title, visible=False, width=640, height=0, dockPreference=omni.ui.DockPreference.DISABLED)
        self._window.flags = (ui.WINDOW_FLAGS_NO_COLLAPSE | ui.WINDOW_FLAGS_NO_SCROLLBAR | ui.WINDOW_FLAGS_NO_RESIZE
                              | ui.WINDOW_FLAGS_NO_MOVE | ui.WINDOW_FLAGS_MODAL)

        def on_no():
            self.hide()

        def on_yes():
            self.hide()
            get_physx_interface().run_backwards_compatibility()
            if get_physx_interface().check_backwards_compatibility():
                text = f"Backward compatibility processing unsuccessful! We are at this moment unable to e.g. automatically process external references, please try our folder batch processing tool by Run on Folder button or through {FOLDER_NOTE}"
                title = "Physics Backward Compatibility Processing"
                prompt = BackwardCompatPrompt(text, get_physx_interface().get_backwards_compatibility_check_log(), True, title)
                prompt._window.position_x = self._window.position_x
                prompt._window.position_y = self._window.position_y
                prompt.show()

        def on_folder():
            self.hide()
            run_backward_compat_on_folder(omni.usd.get_context().get_stage_url())

        with self._window.frame:
            with ui.VStack(height=0):
                ui.Spacer(width=0, height=10)
                with ui.HStack(height=0):
                    ui.Spacer()
                    ui.Label(text, word_wrap=True, width=self._window.width - 80, height=0)
                    ui.Spacer()
                ui.Spacer(width=0, height=10)
                if not folder:
                    with ui.ScrollingFrame(height=100):
                        with ui.HStack(height=0):
                            ui.Label(check_log, word_wrap=True, width=self._window.width - 80, alignment=ui.Alignment.LEFT_TOP)
                with ui.HStack(height=0):
                    ui.Spacer(height=0)
                    if folder:
                        ui.Button("Run on Folder", width=60, height=0).set_clicked_fn(on_folder)
                    else:
                        ui.Button("Run", width=60, height=0).set_clicked_fn(on_yes)
                    ui.Button("Cancel", width=60, height=0).set_clicked_fn(on_no)
                    ui.Spacer(height=0)
                ui.Spacer(width=0, height=10)

    def show(self):
        self._window.visible = True

    def hide(self):
        self._window.visible = False


def set_opposite_body_transform(stage, cache, prim, body0base, fixpos, fixrot):
    joint = UsdPhysics.Joint(prim)
    rel_tm = utils.get_aligned_body_transform(stage, cache, joint, body0base)

    axis = 255
    if prim.IsA(UsdPhysics.PrismaticJoint):
        axis_attrib = prim.GetAttribute("physics:axis")
        axis = utils.AXES_INDICES[axis_attrib.Get()] if axis_attrib.IsValid() else 255

    omni.kit.undo.begin_group()

    if fixpos:
        pos = rel_tm.GetTranslation()
        if body0base:
            if axis < 3:
                pos[axis] = joint.GetLocalPos1Attr().Get()[axis]
            execute("ChangePropertyCommand", prop_path=joint.GetLocalPos1Attr().GetPath(), value=pos, prev=None)
        else:
            if axis < 3:
                pos[axis] = joint.GetLocalPos0Attr().Get()[axis]
            execute("ChangePropertyCommand", prop_path=joint.GetLocalPos0Attr().GetPath(), value=pos, prev=None)

    if fixrot:
        rot = Gf.Quatf(rel_tm.GetRotation().GetQuat())
        if body0base:
            execute("ChangePropertyCommand", prop_path=joint.GetLocalRot1Attr().GetPath(), value=rot, prev=None)
        else:
            execute("ChangePropertyCommand", prop_path=joint.GetLocalRot0Attr().GetPath(), value=rot, prev=None)

    omni.kit.undo.end_group()


def register_stage_update_node(display_name, priority=8, **kwargs): # priority defaults to 8 = pre-physics
    stage_update = omni.stageupdate.get_stage_update_interface()
    stage_update_node = stage_update.create_stage_update_node(display_name, **kwargs)
    nodes = stage_update.get_stage_update_nodes()
    stage_update.set_stage_update_node_order(len(nodes) - 1, priority)
    return stage_update_node
