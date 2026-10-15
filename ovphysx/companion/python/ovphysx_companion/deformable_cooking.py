# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-DEFORM-COOK-001
# @covers AC-1 AC-2 AC-3 AC-4
"""Stopped volume construction from copied surface data; no USD dependency."""
from dataclasses import dataclass
from .cooking import Kind
from .mesh import embed_points, _points, _topology


@dataclass(frozen=True)
class VolumeCook:
    """Independent simulation/collision topologies and visual correspondence.

    Each Cooked cache key is that mesh's topology/rest revision. Both meshes use
    the same declared frame/units; their vertex counts may differ. Visual mapping
    stores its own indices into the simulation mesh, never the collision mesh.
    Arrays and mapping remain valid after all native services have closed.
    """
    simulation: object
    collision: object
    visual_points: tuple
    visual_triangles: tuple
    visual_mapping: object

    @property
    def complete(self):
        return self.visual_mapping.complete and all(d == 0 for d in self.visual_mapping.distances)

    def require_complete(self):
        self.visual_mapping.require_complete()
        outside = tuple(i for i, distance in enumerate(self.visual_mapping.distances) if distance != 0)
        if outside:
            raise ValueError(f'Visual points outside the simulation volume: {outside}')
        return self


def cook_volume(service, points, triangles, *, voxel_resolution=16, frame='body-local', units='stage',
                meters_per_unit=1, right_handed=True, mapping_distance=0):
    """Cook a conforming collider, voxel simulation mesh, and visual embedding.

    Blocking orchestration intended for an authoring worker. Individual native
    jobs obey service cancellation/drain. No stage is attached and no USD is
    imported. Uncontained visual vertices remain explicit in `complete` and
    `visual_mapping`; callers must validate before applying a complete binding.
    """
    points, triangles = _points(points), _topology(triangles, 3)
    conventions = dict(frame=frame, units=units, meters_per_unit=meters_per_unit)
    collision = service.cook(Kind.CONFORMING_TETS, points, triangles,
                             right_handed=right_handed, **conventions).result()
    simulation = service.cook(Kind.VOXEL_TETS, collision.points, collision.indices,
                              voxel_resolution=voxel_resolution, **conventions).result()
    workload = len(points)*len(simulation.indices)
    if workload > 10_000_000:
        raise ValueError(f'Visual embedding requires {workload} point/tetrahedron pairs; '
                         'the native limit is 10000000. Reduce voxel_resolution or visual mesh complexity.')
    mapping = embed_points(service._library, simulation.points, simulation.indices, points,
                            element_width=4, topology_version=simulation.cache_key,
                            frame=frame, units=units, max_distance=mapping_distance)
    return VolumeCook(simulation, collision, points, triangles, mapping)
