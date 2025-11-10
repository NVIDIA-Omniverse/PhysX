# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import sys
import carb
import omni.ext
import omni.kit.app
import omni.kit.test
from omni.physxuicommon import windowmenuitem
from omni.physx.scripts.pythonUtils import ScopeGuard
from omni.physx.scripts import assets_paths
import omni.kit.ui
import omni.ui as ui
from omni.ui_scene import scene as sc
from omni.ui import color as cl
from .utils.loaders import import_tests_to_module, import_tests_to_globals, import_tests_auto
from fnmatch import fnmatch
import carb.settings
import unittest
import omni.kit.test
from omni.kit.test import AsyncTestSuite
from omni.kit.test.test_reporters import TestRunStatus
from omni.kit.test.async_unittest import AsyncTextTestRunner, TeamcityTestResult
import asyncio
import subprocess
import math
import os

from omni.physx.bindings._physx import (
    SETTING_TEST_RUNNER_FILTER,
    SETTING_TEST_RUNNER_SELECTION,
    SETTING_TEST_RUNNER_STATUS,
    SETTING_TEST_RUNNER_REPEATS,
)

import_tests_auto("omni.physxtests.tests")

GLYPHS = {
    TestRunStatus.UNKNOWN: (omni.kit.ui.get_custom_glyph_code("${glyphs}/question.svg"), 0xffffffff),
    TestRunStatus.RUNNING: (omni.kit.ui.get_custom_glyph_code("${glyphs}/spinner.svg"), 0xffff7d7d),
    TestRunStatus.PASSED: (omni.kit.ui.get_custom_glyph_code("${glyphs}/check_solid.svg"), 0xff00ff00),
    TestRunStatus.FAILED: (omni.kit.ui.get_custom_glyph_code("${glyphs}/exclamation.svg"), 0xff0000ff)
}

GREY_STYLE = {"color": 0xff888888}
WHITE_STYLE = {"color": 0xffffffff}

carb_settings = carb.settings.get_settings()
setting_selection = carb_settings.get(SETTING_TEST_RUNNER_SELECTION)
setting_selection = set(setting_selection) if setting_selection is not None else set(["placeholder"])


class TreeDelegate(ui.AbstractItemDelegate):
    def __init__(self):
        super().__init__()
        self._style = carb.settings.get_settings().get_as_string("/persistent/app/window/uiStyle")

    def build_widget(self, model, item, column_id, level, expanded):
        if item is None:
            return

        if column_id == 0:
            with ui.HStack(width=0):
                with ui.VStack(width=0):
                    ui.Spacer(height=5)
                    ui.CheckBox(model=item.enabled_model)
                ui.Label(item.column_0, style_type_name_override="TreeView.Item")
        else:
            if isinstance(item, TestItem):
                if column_id == 1:
                    ui.Label(item.column_1, style_type_name_override="TreeView.Item", width=0)
                elif column_id == 2:
                    item.status_label = ui.Label(item.glyph[0], style={"color": item.glyph[1]}, width=0)
            else:
                if column_id == 1:
                    item.selected_label = ui.Label(item.column_1, style_type_name_override="TreeView.Item", width=0)

    def build_branch(self, model, item, column_id, level, expanded):
        if column_id == 0:
            with ui.HStack(width=16*level, height=0):
                ui.Spacer(width=16*level)
                item.branch_label = ui.Label(item.get_branch_text(expanded), style_type_name_override="TreeView.Item", width=15)

    def build_header(self, column_id):
        pass


def update_selection_setting(name, value):
    if value:
        setting_selection.add(name)
    else:
        setting_selection.discard(name)


class TestItem(ui.AbstractItem):
        def __init__(self, test):
            super().__init__()
            self.status = TestRunStatus.UNKNOWN
            self.glyph = GLYPHS[self.status]
            self.status_label = None
            self.branch_label = None
            self.test = test
            self.enabled_model = ui.SimpleBoolModel(False)
            self.enabled_model.add_value_changed_fn(self.on_checkbox)
            self.filtered = True
            self.test_id = test.id()

            parts = self.test_id.split(".")
            test_method = parts[-1].replace("test_", "")
            test_class = parts[-2]
            self.column_0 = test_method
            self.column_1 = test_class            

        def get_children(self):
            return []

        def set_status(self, status):
            self.status = status
            self.glyph = GLYPHS[status]
            if self.status_label is not None:
                self.status_label.text = self.glyph[0]
                self.status_label.set_style({"color": self.glyph[1]})

        def update_filter(self, filters):
            res = [not fnmatch(self.test_id, f[1])^f[0] for f in filters]
            self.filtered = all(res)

        def get_branch_text(self, expanded):
            return ""

        def on_checkbox(self, model):
            update_selection_setting(self.test_id, model.as_bool)


class CategoryItem(ui.AbstractItem):
    def __init__(self, name, tests):
        super().__init__()
        self.column_0 = name
        self.children = [TestItem(t) for t in tests]
        self.enabled_model = ui.SimpleBoolModel(False)
        self.enabled_model.add_value_changed_fn(self.on_checkbox)
        self.column_1 = ""
        self.selected_label = None
        self.branch_label = None
        for c in self.children:
            c.enabled_model.add_value_changed_fn(self.on_child_checkbox)
        self.update_selected()

        self._enabled_guard = ScopeGuard()

    def get_children(self):
        return [c for c in self.children if c.filtered]

    def update_filter(self, filters):
        for c in self.children:
            c.update_filter(filters)
        self.update_selected()

    def set_selection(self, enabled):
        for c in self.children:
            c.enabled_model.set_value(enabled)
        self.update_selected()

    def on_checkbox(self, model):
        if not self._enabled_guard.is_guarded():
            self.set_selection(model.as_bool)
        update_selection_setting(self.column_0, model.as_bool)

    def on_child_checkbox(self, model):
        self.update_selected()
        if not model.as_bool:
            with self._enabled_guard:
                self.enabled_model.set_value(False)

    def update_selected(self):
        filtered = 0
        selected = 0
        for test_item in self.children:
            if test_item.filtered:
                filtered += 1
                if test_item.enabled_model.as_bool:
                    selected += 1
        self.column_1 = f"Selected {selected}/{filtered}"
        if self.selected_label is not None:
            self.selected_label.text = self.column_1

    def get_branch_text(self, expanded):
        return ("-" if expanded else "+") if len(self.get_children()) != 0 else " "


class RootItem(ui.AbstractItem):
    def __init__(self, test_modules):
        super().__init__()
        self.children = [CategoryItem(name, tests) for name, tests in test_modules.items()]

    def update_filter(self, text):
        for c in self.children:
            c.update_filter(text)

    def get_children(self):
        return [c for c in self.children if len(c.children) != 0]

class TreeModel(ui.AbstractItemModel):
    def __init__(self, test_modules):
        super().__init__()
        self._root = RootItem(test_modules)
        self._filter_text = ""

    def get_item_children(self, item):
        curr = self._root if item is None else item
        return curr.get_children()

    def get_item_value_model_count(self, item):
        return 3

    def get_item_value_model(self, item, column_id):
        if item is None:
            return ui.SimpleStringModel("root")
        if column_id == 0:
            return item.name_model
        return None

    def update_filter(self, text):
        filters = []

        for f in text.split(" "):
            if len(f)==0:
                continue
            include = True
            if f[0] == "-":
                include = False
                f = f[1:]
            f = f"*{f}*"
            filters.append((include, f)) 

        self._root.update_filter(filters)

        for c in self._root.children:
            self._item_changed(c)


class RotatingIcon(sc.Manipulator):
    def __init__(self):
        super().__init__()   
        point_count = 10
        self._points = []
        self._sizes = []
        self._colors = []
        for i in range(point_count):
            weight = i / point_count
            angle = 2.0 * math.pi * weight
            self._points.append([math.cos(angle), math.sin(angle), 0])
            self._colors.append(0xffff7d7d)
            self._sizes.append(6 * (weight + 1.0 / point_count))
        self._angle = 120

    def on_build(self):
        with sc.Transform(transform=sc.Matrix44.get_rotation_matrix(0, 0, self._angle, True)):
            sc.Points(self._points, colors=self._colors, sizes=self._sizes)
        self._angle += 5
        self.invalidate()


class TestResult(unittest.TextTestResult):
    def __init__(self, stream, descriptions, verbosity):
        super().__init__(stream, descriptions, verbosity)
        self.on_status_report_fn = None

    def _report_status(self, *args, **kwargs):
        if self.on_status_report_fn:
            self.on_status_report_fn(*args, **kwargs)

    def addError(self, test, err, *k):
        super().addError(test, err)
        self.report_fail(test, "Error", err)

    def addFailure(self, test, err, *k):
        super().addFailure(test, err)
        self.report_fail(test, "Failure", err)

    def report_fail(self, test, fail_type, err):
        self._report_status(test.id(), TestRunStatus.FAILED, fail_message=err)

    def startTest(self, test):
        # fail on carb error, deprecated in kit sdk in favor of subprocess tests
        omni.kit.app.get_app().get_log_event_stream().pump()
        test._log_error_checker = omni.kit.test.async_unittest.LogErrorChecker()

        super().startTest(test)
        self.stream.write("\n")
        self._report_status(test.id(), TestRunStatus.RUNNING)

    def stopTest(self, test):
        super().stopTest(test)
        passed = test._outcome.success if test._outcome else True
        skipped = test._outcome is None  # test._outcome is None when test is skipped
        if passed or skipped:
            self._report_status(test.id(), TestRunStatus.PASSED)


class Window(ui.Window):
    class Logger():
        prefix = "[py stdout]:"
        source = "omni.kit.app.impl"
        prefix_len = len(prefix)

        def __init__(self, window):
            self._window = window
            self._logging = carb.logging.acquire_logging()

        def open(self):
            self._logger_handle = self._logging.add_logger(self._on_log)

        def close(self):
            self._logging.remove_logger(self._logger_handle)

        def _on_log(self, source, level, filename, lineNumber, message):
            def removeprefix(text):
                return text[Window.Logger.prefix_len:] if text.startswith(Window.Logger.prefix) else text

            if source == (Window.Logger.source):
                self._window._log_frame.scroll_y = self._window._log_label.computed_height + 20
                self._window._log_label.text += removeprefix(message)

    def __init__(self):
        super().__init__("Physics Test Runner", ui.DockPreference.LEFT_BOTTOM, width=960, height=600)
        self.deferred_dock_in("Content", ui.DockPolicy.CURRENT_WINDOW_IS_ACTIVE)

        self._test_modules = dict()
        self._test_info = dict()
        self._tree_view = None
        self._delegate = TreeDelegate()
        self._interrupt = False
        self._running_task = None
        self._repeats = 1
        self._iteration = 0
        ext_man = omni.kit.app.get_app().get_extension_manager()

        def is_phys_module(ext_id):
            config = omni.kit.app.get_app().get_extension_manager().get_extension_dict(ext_id)
            package = config.get("package", None)
            if package is None:
                return None

            if package.get("repository", None) == "//sw/physx/omniverse/physics/trunk/":
                return package["name"]

            if config["path"].lower().find("extsphysics") != -1:
                return package["name"]

            return None
            

        # all physx tests should be by definition enabled after, since they would depend on this ext
        def on_ext_enabled(ext_id, *_):
            name = is_phys_module(ext_id)
            if name is None:
                return

            self._test_modules[name] = omni.kit.test.get_tests_from_modules([m for m in self._generate_modules(ext_id)])
            self._build_ui()

        self._hook_enable = ext_man.get_hooks().create_extension_state_change_hook(
            on_ext_enabled, omni.ext.ExtensionStateChangeType.AFTER_EXTENSION_ENABLE,
            ext_dict_path="python", hook_name="python.physics_test_runner enable")

        def on_ext_disabled(ext_id, *_):
            name = is_phys_module(ext_id)
            if name is None:
                return

            ret = self._test_modules.pop(name, None)
            if ret is not None:
                self._build_ui()

        self._hook_disable = ext_man.get_hooks().create_extension_state_change_hook(
            on_ext_disabled, omni.ext.ExtensionStateChangeType.BEFORE_EXTENSION_DISABLE,
            ext_dict_path="python", hook_name="python.physics_test_runner disable")

        self._build_ui()

    def on_shutdown(self):
        self._hook_disable = None
        self._hook_enable = None
        self._tree_view = None

        carb_settings.set(SETTING_TEST_RUNNER_SELECTION, list(setting_selection))

    def _running_tests_on(self, enable_interrupt=True):
        self._button_run_selected.set_style(GREY_STYLE)
        self._button_run_failed.set_style(GREY_STYLE)
        self._button_run_cpp.set_style(GREY_STYLE)
        self._button_run_selected.enabled = False
        self._button_run_failed.enabled = False
        self._verbose_cpp_output.enabled = False
        self._button_run_cpp.enabled = False

        self._interrupt = False
        if enable_interrupt:
            self._button_interrupt.enabled = True
            self._button_interrupt.set_style(WHITE_STYLE)

        self._log_label.text = ""
        self._log_status.visible = True

    def _running_tests_off(self):
        self._button_run_selected.set_style(WHITE_STYLE)
        self._button_run_failed.set_style(WHITE_STYLE)
        self._button_run_cpp.set_style(WHITE_STYLE)
        self._button_run_selected.enabled = True
        self._button_run_failed.enabled = True
        self._verbose_cpp_output.enabled = True
        self._button_run_cpp.enabled = True

        self._button_interrupt.enabled = False
        self._button_interrupt.set_style(GREY_STYLE)
        self._log_status.visible = False

    def _run_tests(self, test_items):
        self._running_tests_on()
        logger = Window.Logger(self)
        logger.open()
        finished = False

        self._iteration = 1

        def on_finish():
            # interrupt on the last test can call this twice
            nonlocal finished
            if finished:
                return

            def has_failure():
                for test_item in test_items.values():
                    if test_item.status == TestRunStatus.FAILED:
                        return True
                return False

            if self._interrupt or self._iteration >= self._repeats or has_failure():
                setting_status = [f"{test_id}:{test_item.status.name}" for test_id, test_item in test_items.items() if test_item.status != TestRunStatus.UNKNOWN]
                carb_settings.set(SETTING_TEST_RUNNER_STATUS, setting_status)
                self._running_tests_off()
                logger.close()
                finished = True
            else:
                self._iteration = self._iteration + 1
                run()

        def on_status_report(test_id, status, **kwargs):
            test_items[test_id].set_status(status)
            
            if self._interrupt:
                self._running_task.cancel()
                on_finish()

        def run():
            tests = [item.test for item in test_items.values()]
            loader = unittest.TestLoader()
            loader.suiteClass = AsyncTestSuite
            suite = AsyncTestSuite()
            suite.addTests(tests)

            AsyncTextTestRunner.resultclass = TestResult
            runner = AsyncTextTestRunner(verbosity=2, stream=sys.stdout)

            async def single_run():
                await runner.run(suite, on_status_report)
                on_finish()

            iter_str = f"Iteration {self._iteration}, " if self._repeats > 1 else ""
            print("=======================================")
            print(f"{iter_str}Running Tests (count: {len(tests)}):")
            print("=======================================")

            self._running_task = asyncio.ensure_future(single_run())

        run()

    def _gen_test_items(self):
        for cat_item in self._list_model._root.children:
            for test_item in cat_item.children:
                yield test_item

    def _run_selected(self):
        test_items = {ti.test_id: ti for ti in self._gen_test_items() if ti.filtered and ti.enabled_model.as_bool}
        self._run_tests(test_items)

    def _run_failed(self):
        test_items = {ti.test_id: ti for ti in self._gen_test_items() if ti.status == TestRunStatus.FAILED}
        self._run_tests(test_items)

    def _on_interrupt(self):
        self._interrupt = True

    def _set_selection(self, enabled):
        for cat_item in self._list_model._root.children:
            cat_item.enabled_model.set_value(enabled)
            if not enabled:
                for test_item in cat_item.children:
                    test_item.enabled_model.set_value(False)

    def _get_unit_test_path(self):
        app_folder = carb.tokens.get_tokens_interface().resolve("${app}")
        shell_ext = carb.tokens.get_tokens_interface().resolve("${shell_ext}")
        script_path = os.path.join(os.path.split(app_folder)[0], f'test.unit.physics{shell_ext}').replace("\\", "/")
        return script_path

    def _run_cpp_tests(self):
        self._running_tests_on(enable_interrupt=False)
        self._log_label.text = "Running C++ Tests...\n"
        
        script_path = self._get_unit_test_path()
        p = subprocess.Popen(
            [script_path], universal_newlines=True, stdout=subprocess.PIPE, stdin=subprocess.PIPE, stderr=subprocess.STDOUT
        )

        out = p.communicate()[0].split("\n")
        out = [l for l in out if l.startswith("[d")]
        self._log_label.text = "\n".join(out)
        self._running_tests_off()

    def _run_cpp_tests_async(self):
        from threading  import Thread
        from queue import Queue, Empty

        script_path = self._get_unit_test_path()
        p = subprocess.Popen(
            [script_path], universal_newlines=True, stdout=subprocess.PIPE, stdin=subprocess.PIPE, stderr=subprocess.STDOUT
        )

        # This functions filters down all the output from the script
        # Update the rules here to include/exclude info
        def include_line(line, verbose_output):
            # always include doctest messages, the results are reported that way
            if line.startswith("[d"):
                return True
            
            # always include errors so failures can be triaged
            elif "[Error]" in line:
                return True

            # only include warnings if verbose mode is selected
            elif verbose_output and "[Warning]" in line:
                return True
            
            return False

        def enqueue_output(out, queue, verbose_output):
            try:
                for line in iter(out.readline, b''):
                    if p.poll() is None and line != '' and include_line(line, verbose_output):
                        queue.put(line.rstrip("\n"))
            except ValueError:
                pass
            out.close()

        q = Queue()
        verbose_output = self._verbose_cpp_output.model.get_value_as_bool()
        t = Thread(target=enqueue_output, args=(p.stdout, q, verbose_output))
        t.start()
        logger = Window.Logger(self)
        logger.open()
        self._running_tests_on(enable_interrupt=False)

        self._log_label.text = "Running C++ Tests...\n"

        async def wait():
            def _add_to_log(line):
                print(line)
                # append it to the log
                self._log_label.text += f"\n{line}"
                # and scroll to the bottom of the frame
                self._log_frame.scroll_y = self._log_frame.scroll_y_max + 100

            while True:
                await omni.kit.app.get_app().next_update_async()
                try:  line = q.get_nowait()
                except Empty: pass
                else:
                    _add_to_log(line)
                if p.poll() is not None:
                    p.stdout.close()
                    while not q.empty():
                        _add_to_log(q.get())
                    logger.close()
                    self._running_tests_off()
                    break

        asyncio.ensure_future(wait())

    def _build_ui(self):
        self._test_info.clear()

        def on_double_click(x, y, b, m):
            if len(self._tree_view.selection) <= 0:
                return
            item = self._tree_view.selection[0]
            if isinstance(item, CategoryItem):
                self._tree_view.set_expanded(item, not self._tree_view.is_expanded(item), False)           
            if isinstance(item, TestItem):
                item.enabled_model.set_value(not item.enabled_model.get_value_as_bool())

        def on_search(model):
            self._tree_view.model.update_filter(model.as_string)
            carb_settings.set(SETTING_TEST_RUNNER_FILTER, model.as_string)

        def on_repeats(model):
            self._repeats = model.as_int
            carb_settings.set(SETTING_TEST_RUNNER_REPEATS, model.as_int)

        with self.frame:
            with ui.VStack():
                with ui.HStack(height=25):
                    self._button_run_selected = ui.Button("Run Selected", width=100, clicked_fn=self._run_selected)
                    self._button_run_failed = ui.Button("Run Failed", width=100, clicked_fn=self._run_failed)
                    self._button_interrupt = ui.Button("Interrupt", width=100, clicked_fn=self._on_interrupt, style=GREY_STYLE, enabled=False)
                    ui.Spacer(width=5)
                    ui.Label("Filter: ", width=0)
                    sf_filter = ui.StringField(width=200)
                    sf_filter.model.add_value_changed_fn(on_search)
                    ui.Button("Select All", width=100, clicked_fn=lambda: self._set_selection(True))
                    ui.Button("Deselect All", width=100, clicked_fn=lambda: self._set_selection(False))
                    ui.Spacer(width=5)
                    ui.Label("Repeats: ", width=0)
                    sf_repeats = ui.StringField(width=40)
                    sf_repeats.model.add_value_changed_fn(on_repeats)
                    ui.Spacer()
                    with ui.VStack(width=25):
                        ui.Spacer(height=6, width=0)
                        self._verbose_cpp_output = ui.CheckBox(ui.SimpleBoolModel(False), name="verbose", width=0)
                    verbose_tooltip = (
                        "When checked, warnings are included in the C++ test output below."
                        "\nUnchecked, only errors will be displayed"
                    )
                    ui.Label("Verbose Output", width=125, tooltip=verbose_tooltip)
                    self._button_run_cpp = ui.Button("Run C++ Tests", width=100, clicked_fn=lambda: self._run_cpp_tests_async())
                ui.Spacer(height=5)
                with ui.HStack():
                    with ui.ScrollingFrame(
                        horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF,
                        vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                        style_type_name_override="TreeView", width=ui.Fraction(0.50)
                    ):
                        self._list_model = TreeModel(self._test_modules)
                        self._tree_view = ui.TreeView(
                            self._list_model,
                            delegate=self._delegate,
                            root_visible=False,
                            style={"TreeView.Item": {"margin": 4}},
                            column_widths=[300])
                        self._tree_view.set_mouse_double_clicked_fn(on_double_click)
                    ui.Spacer(width=5)
                    with ui.ZStack(width=ui.Fraction(0.50)):
                        self._log_frame = ui.ScrollingFrame(
                            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF,
                            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                            style_type_name_override="TreeView", width=ui.Fraction(0.50)
                        )
                        with self._log_frame:
                            self._log_label = ui.Label("", word_wrap=True, height=0)
                        with ui.VStack():
                            ui.Spacer(height=15)
                            with ui.HStack():
                                ui.Spacer(width=ui.Fraction(1))
                                scene_view = sc.SceneView(aspect_ratio_policy=sc.AspectRatioPolicy.PRESERVE_ASPECT_FIT, height=30, width=30)
                                with scene_view.scene:
                                    self._log_status = RotatingIcon()
                                ui.Spacer(width=15)
                                self._log_status.visible = False

        # set state from settings
        test_items = {}
        for cat_item in self._list_model._root.children:
            test_items[cat_item.column_0] = cat_item
            for test_item in cat_item.children:
                test_items[test_item.test_id] = test_item
                

        setting_filter = carb_settings.get(SETTING_TEST_RUNNER_FILTER)
        sf_filter.model.set_value(setting_filter)

        setting_repeats = carb_settings.get(SETTING_TEST_RUNNER_REPEATS)
        sf_repeats.model.set_value(setting_repeats)
        
        setting_status = carb_settings.get(SETTING_TEST_RUNNER_STATUS)
        if setting_status:
            for status in setting_status:
                split = status.split(":")
                test_item = test_items.get(split[0])
                if test_item is not None:
                    test_item.set_status(TestRunStatus[split[1]])

        selection_tmp = set(setting_selection)
        for selection in selection_tmp:
            test_item = test_items.get(selection)
            if test_item is not None:
                test_item.enabled_model.set_value(True)

    def _get_extension_modules(self):
        manager = omni.kit.app.get_app().get_extension_manager()
        module_names = manager.get_enabled_extension_module_names()
        sys_modules = set()
        for name in module_names:
            if name in sys.modules:
                sys_modules.add(sys.modules[name])
            if not name.endswith(".tests"):
                test_module = omni.kit.test._try_import(f"{name}.tests")
                if test_module:
                    sys_modules.add(test_module)
        return sys_modules

    def _generate_modules(self, ext_id):
        ext_dict = omni.kit.app.get_app().get_extension_manager().get_extension_dict(ext_id)
        if ext_dict["package"]["name"].startswith("omni.physx") or (ext_dict["dependencies"] is not None and ext_dict["dependencies"].get("omni.physx.tests") is not None):
            if ext_dict.get("python", None):
                for ext_module in ext_dict["python"].get("modules", ext_dict["python"].get("module", {})):
                    module = sys.modules.get(ext_module["name"])
                    if module is not None:
                        yield module
                    module = sys.modules.get(ext_module["name"] + ".tests")
                    if module is not None:
                        yield module


class Extension(omni.ext.IExt): 
    def on_startup(self):
        # setup assets path only when empty to allow explicit overrides
        assets_path = carb.settings.get_settings().get("physics/testsAssetsPath")
        if assets_path == "":
            assets_path = assets_paths.get_server_path(force_ov_path=True)
            carb.settings.get_settings().set("physics/testsAssetsPath", assets_path)

        carb.log_info(assets_path)
        self._window = windowmenuitem.WindowMenuItem("Test Runner", lambda: Window(), carb.settings.get_settings().get("physics/developmentMode"))

    def on_shutdown(self):
        self._window.on_shutdown()
        self._window = None
