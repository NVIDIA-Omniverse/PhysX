# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

from pxr import UsdGeom, Usd
import omni.kit.notification_manager as nm
from .schemas import material_apis


def has_any_material_api(p):
    return any([p.HasAPI(api) for api in material_apis])


def check_surface_deformable_application(prim: Usd.Prim) -> bool:
    error_msg = None
    mesh = UsdGeom.Mesh(prim)
    if mesh:
        face_vertex_counts = mesh.GetFaceVertexCountsAttr().Get()
        if not face_vertex_counts or len(face_vertex_counts) == 0:
            error_msg = "Surface Deformable Body: Mesh is empty or has non-triangular faces."
        elif not all(c == 3 for c in face_vertex_counts):
            error_msg = (
                "Surface Deformable Body: Mesh has non-triangular faces. "
                "Nest mesh under Xform and create surface deformable on Xform."
            )
    if error_msg is not None:
        nm.post_notification(error_msg, duration=8, status=nm.NotificationStatus.WARNING)
        return False
    return True
