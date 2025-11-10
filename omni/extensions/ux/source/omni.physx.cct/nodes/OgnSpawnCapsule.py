# SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

from pxr import Gf
import omni.usd
import omni.graph.core as og
from omni.physxcct.scripts import utils as cct_utils


class OgnSpawnCapsule:
    @staticmethod
    def compute(db) -> bool:
        if db.inputs.spawn:
            stage = omni.usd.get_context().get_stage()
            node_path = db.node.get_prim_path()
            cct_path = node_path + "/capsule"
            if stage.GetPrimAtPath(cct_path).IsValid():
                stage.RemovePrim(cct_path)

            pos = db.inputs.capsulePos

            cct_utils.spawn_capsule(
                stage,
                cct_path,
                Gf.Vec3f(pos[0].item(), pos[1].item(), pos[2].item()),
                db.inputs.capsuleHeight,
                db.inputs.capsuleRadius
            )

            db.outputs.path = cct_path
            db.outputs.done = og.ExecutionAttributeState.ENABLED

            return True

        return False
