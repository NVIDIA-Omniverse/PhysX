# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.kit.commands
import omni.kit.undo
import omni.physxdemos as demo
import omni.usd
import os
import os.path

def get_usd_asset_path(filename):
    schema_folder = os.path.abspath(os.path.normpath(os.path.join(__file__, "../../../../../data/usd/assets")))
    schema_folder = schema_folder.replace("\\", "/") + "/"
    full_filename = schema_folder + filename
    # print(full_filename)
    exists = os.path.exists(full_filename)
    assert(exists)
    return full_filename

# helper to load a usd file
def _open_test_usd_file(stage, filename):
    full_filename = get_usd_asset_path(filename)
    context = omni.usd.get_context()
    context.open_stage(full_filename)
