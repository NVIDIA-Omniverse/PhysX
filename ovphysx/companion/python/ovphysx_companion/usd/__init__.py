# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-PYTHON-001
# @covers AC-1 AC-2 AC-3 AC-4

"""Customer-USD utilities. Importing this module does not import pxr or numpy.

All operations take an explicit EditSession and layer. Register codeless schemas
before opening stages when using attachment authoring. No simulation is loaded.
"""

from .session import EditSession, EditPatch, DependencyError, StaleResultError, RecoveryError
from .meshes import MeshSnapshot, TetMeshSnapshot, capture_mesh, capture_tet_mesh
from .attachments import (VertexAttachment, ElementAttachment, ElementCollisionFilter,
                          apply_vertex_attachment, apply_element_attachment,
                          apply_collision_filter, remove_owned_attachment, match_vertex_attachment)
from .deformables import DeformableMesh, DeformableBody, DeformableMaterial, Embedding, apply_deformable_body
from .writeback import MeshFrame, write_simulation_output

__all__ = [
    "EditSession", "EditPatch", "DependencyError", "StaleResultError", "RecoveryError",
    "MeshSnapshot", "capture_mesh", "VertexAttachment", "apply_vertex_attachment", "match_vertex_attachment",
    "MeshFrame", "write_simulation_output", "TetMeshSnapshot", "capture_tet_mesh",
    "ElementAttachment", "ElementCollisionFilter", "apply_element_attachment",
    "apply_collision_filter", "remove_owned_attachment", "DeformableMesh", "DeformableBody",
    "DeformableMaterial", "Embedding", "apply_deformable_body",
]
