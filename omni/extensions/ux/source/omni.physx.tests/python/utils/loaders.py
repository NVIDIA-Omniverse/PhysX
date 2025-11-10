# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import carb
import omni.kit.test
from . import physicsBase
from os.path import dirname, basename, isfile, join, splitext
from glob import glob
import importlib
import inspect
import sys

"""
Helpers to scan and import all classes from submodules of a module inherited from AsyncTestCase. For the imports to have
the expected effect you have to provide a suitable target for the tests to reside in: a name of a module (or its globals)
that is being autoimported through [[python.module]] in a toml file, since the omni.kit.test loader
searches through the attributes of autoimported modules for test cases.

Filtering: all tests are loaded when physics development mode is on. When off only tests without a "category" attribute or with
"category" explicitly set to physicsBase.TestCategory.Kit will get loaded.

The helpers are intended to be used in an __init__ file of an autoimported module.

Usage:

Import tests from a module. Gets the globals dict from its caller's scope.
example: import_tests_auto("omni.physxtests.mytests")

Imports test from a module to a provided globals dictionary.
example: import_tests_to_globals("omni.physxtests.mytests", globals())

Import tests from a module to a module.
example: import_tests_to_module("omni.physxtests.mytests", "omni.physxtests")

All functions support an additional submodules_list param to explicitly state which submodules should be searched for tests.
"""

local_mode = carb.settings.get_settings().get("/exts/omni.physx.tests/localMode")


def import_tests_to_globals(source_module_name, target_globals, submodules_list=None):
    module = importlib.import_module(source_module_name)

    dirpath = module.__path__._path[0] if module.__file__ is None else dirname(module.__file__)
    for submodule_file in [basename(f) for f in glob(join(dirpath, "*.py")) if isfile(f)]:
        if not submodule_file.startswith("_"):
            submodule_name = splitext(submodule_file)[0]
            if submodules_list and submodule_name not in submodules_list:
                continue

            import_submodule_name = f"{source_module_name}.{submodule_name}"
            submodule = importlib.import_module(import_submodule_name)

            for n in submodule.__dict__:
                if n.startswith("_"):
                    continue

                test_class = getattr(submodule, n)

                if not inspect.isclass(test_class):
                    continue

                if test_class in [
                    physicsBase.PhysicsMemoryStageBaseAsyncTestCase,
                    physicsBase.PhysicsKitStageAsyncTestCase,
                    physicsBase.PhysicsBaseAsyncTestCase
                ]:
                    continue

                def category_check(test_class):
                    if local_mode:
                        return True

                    if not hasattr(test_class, "category"):
                        return True

                    if test_class.__dict__.get("category", physicsBase.TestCategory.Kit) == physicsBase.TestCategory.Kit:
                        return True

                    if test_class.__dict__.get("category", physicsBase.TestCategory.Core) == physicsBase.TestCategory.Core:
                        return True

                if issubclass(test_class, omni.kit.test.AsyncTestCase) and category_check(test_class):
                    target_globals[n] = test_class


def import_tests_to_module(source_module_name, target_module_name, submodules_list=None):
    import_tests_to_globals(source_module_name, sys.modules[target_module_name].__dict__, submodules_list)


def import_tests_auto(source_module_name, submodules_list=None, offset=1):
    import_tests_to_globals(source_module_name, inspect.stack()[offset][0].f_globals, submodules_list)
