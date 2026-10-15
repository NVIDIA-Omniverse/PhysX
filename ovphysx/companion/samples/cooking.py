# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-COOK-001
# @covers AC-1 AC-2 AC-3 AC-4
"""Collider overlay tool: cook polygons, retain a result, invalidate, export data."""
from ovphysx import PhysX
from ovphysx_companion.cooking import CookingService, Kind

POINTS = ((-1,-1,-1),(1,-1,-1),(1,1,-1),(-1,1,-1),
          (-1,-1,1),(1,-1,1),(1,1,1),(-1,1,1))
FACES = ((0,3,2,1),(4,5,6,7),(0,1,5,4),(3,7,6,2),(0,4,7,3),(1,2,6,5))
TRIANGLES = tuple((f[0],f[1],f[2]) for f in FACES) + tuple((f[0],f[2],f[3]) for f in FACES)


def run():
    PhysX.set_cpu_mode(True)
    physics = PhysX()
    try:
        with CookingService(physics) as service:
            job = service.cook(Kind.TRIANGLE, POINTS, FACES)
            overlay = job.result()
            assert len(overlay.indices) == 12
            assert service.cook(Kind.TRIANGLE, POINTS, FACES).result() is overlay
            service.invalidate(overlay.cache_key)
            replacement = service.cook(Kind.TRIANGLE, POINTS, FACES).result()
            assert replacement is not overlay
            # A viewport consumes these arrays on its own drawing thread.
            return dict(points=overlay.points.tolist(), triangles=overlay.indices.tolist(),
                        polygon_per_triangle=overlay.face_map.tolist(), phase=job.progress)
    finally:
        physics.destroy()


if __name__ == '__main__':
    import json
    print(json.dumps(run()))
