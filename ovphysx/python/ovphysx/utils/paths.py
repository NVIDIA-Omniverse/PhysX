# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PYTHON-UTILS-001
# @covers AC-1 AC-2 AC-5

"""Stage path uniquification.

Authoring a prim at a path another prim already holds silently redefines that
prim, so every helper that defines geometry resolves its path through here
first. Both helpers only compute a name -- neither touches the stage beyond
asking what is already at a path.
"""

import logging
import re
import typing

from pxr import Sdf, Usd

logger = logging.getLogger(__name__)

__all__ = [
    "get_stage_next_free_path",
]


def get_stage_next_free_path(stage: Usd.Stage, path: typing.Union[str, Sdf.Path], prepend_default_prim: bool) -> str:
    """
    Gets valid path in stage, if the path already exists it will append number.

    Args:
        stage:      The Usd.Stage to add path.
        path:       The desired path to create.
        prepend_default_prim: Whether prepend default prim path name.
    """
    if isinstance(path, str) and not Sdf.Path.IsValidPathString(path):
        raise ValueError(f"{path} is not a valid path")

    path = Sdf.Path(path)
    # A relative path passes IsValidPathString but can crash other USD APIs.
    corrected_path = path.MakeAbsolutePath(Sdf.Path.absoluteRootPath)
    if path != corrected_path:
        logger.warning(f"Path {path} is auto-corrected to {corrected_path}. Please verify your path format.")
        path = corrected_path

    if prepend_default_prim and stage.HasDefaultPrim():
        defaultPrim = stage.GetDefaultPrim()
        if defaultPrim and not (path.HasPrefix(defaultPrim.GetPath()) and path != defaultPrim.GetPath()):
            path = path.ReplacePrefix(Sdf.Path.absoluteRootPath, defaultPrim.GetPath())

    def increment_path(path):
        match = re.search(r"_(\d+)$", path)
        if match:
            new_num = int(match.group(1)) + 1
            ret = re.sub(r"_(\d+)$", str.format("_{:02d}", new_num), path)
        else:
            ret = path + "_01"
        return ret

    path_string = path.pathString
    while stage.GetPrimAtPath(path_string):
        path_string = increment_path(path_string)

    return path_string


def _create_unused_path(stage: Usd.Stage, base_path: str, path: str) -> str:
    """
    Returns a leaf name under base_path that is not yet used, appending a counter if needed.

    Private, and its one caller is ``joints.create_joint``. It uniquifies with a
    bare ``0`` / ``1`` / ``2`` suffix over concatenated strings, where the public
    ``get_stage_next_free_path`` uses ``_01`` over an ``Sdf.Path``.

    Args:
        stage:      The Usd.Stage to check.
        base_path:  The parent path.
        path:       The desired leaf name.
    """
    if stage.GetPrimAtPath(base_path + "/" + path).IsValid():
        uniquifier = 0
        while stage.GetPrimAtPath(base_path + "/" + path + str(uniquifier)).IsValid():
            uniquifier += 1
        path = path + str(uniquifier)
    return path
