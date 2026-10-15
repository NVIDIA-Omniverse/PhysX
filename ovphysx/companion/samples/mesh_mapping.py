# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-ATTACH-COMPUTE-001
# @covers AC-1 AC-2 AC-3 AC-4
# @implements REQ-COMP-DEFORM-COOK-001
# @covers AC-2 AC-3 AC-4
"""Stopped attachment/visual binding computation; no USD or live runtime."""
import json
from ovphysx_companion.jobs import AuthoringService
from ovphysx_companion.mesh import interpolate_elements


def run():
    from ovphysx import _bindings
    vertices = ((0,0,0), (1,0,0), (0,1,0), (0,0,1))
    with AuthoringService(_bindings._lib) as authoring:
        mapping = authoring.embed_points(vertices, ((0,1,2,3),), ((.25,.25,.25),),
                                          element_width=4, topology_version='simulation-v1').result(10)
        mapping.require_complete()
        # Stand-in completed simulation positions. Interpolation uses the same
        # native mapping path an extension calls after copying solver output.
        moved = tuple((x,y,z+1) for x,y,z in vertices)
        visual = interpolate_elements(_bindings._lib, moved, mapping, topology_version='simulation-v1')
        assert visual == ((.25,.25,1.25),)
        return dict(elements=mapping.elements, weights=mapping.weights, visual_positions=visual)


if __name__ == '__main__':
    print(json.dumps(run()))
