# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import pkgutil
import os
from inspect import getfullargspec
from functools import wraps


def autoassign(func):
    """decorator that assigns all parameters of a method to member variables with
    a "_" prefix."""
    @wraps(func)
    def wrapper(self, *args, **kwargs):
        spec = getfullargspec(func)

        for name, arg in list(zip(spec.args[1:], args)) + list(kwargs.items()):
            setattr(self, "_" + name, arg)

        if spec.defaults is not None:
            for name, default in zip(reversed(spec.args), reversed(spec.defaults)):
                newName = "_" + name
                if not hasattr(self, newName):
                    setattr(self, newName, default)

        func(self, *args, **kwargs)

    return wrapper


def get_all_submodules(file, exclude=[]):
    return [module for _, module, _ in pkgutil.iter_modules([os.path.dirname(file)]) if module not in exclude]


class ScopeGuard():
    def __init__(self):
        self._guard = False

    def __enter__(self):
        self._guard = True

    def __exit__(self, type, value, traceback):
        self._guard = False

    def is_guarded(self):
        return self._guard
