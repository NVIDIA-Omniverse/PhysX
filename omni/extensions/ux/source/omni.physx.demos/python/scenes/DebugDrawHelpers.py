# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import carb

COLOR_RED = 0xffff0000
COLOR_GREEN = 0xff00ff00
COLOR_BLUE = 0xff0000ff
COLOR_YELLOW = 0xffffff00

def draw_box(debugDraw, origin, half_extent, rot = carb.Float4(0.0, 0.0, 0.0, 1.0), color=4283782485):  
    extent = carb.Float3(half_extent.x * 2.0, half_extent.y * 2.0, half_extent.z * 2.0)
    debugDraw.draw_box(origin, rot, extent, color)

def draw_sphere(debugDraw, origin, radius, color=4283782485):
    debugDraw.draw_sphere(origin, radius, color)

def draw_frame(debugDraw, pos, scale=1.0):
    frameXAxis = carb.Float3(pos[0] + scale, pos[1], pos[2])
    frameYAxis = carb.Float3(pos[0], pos[1] + scale, pos[2])
    frameZAxis = carb.Float3(pos[0], pos[1], pos[2] + scale)
    debugDraw.draw_line(pos, COLOR_RED, frameXAxis, COLOR_RED)
    debugDraw.draw_line(pos, COLOR_GREEN, frameYAxis, COLOR_GREEN)
    debugDraw.draw_line(pos, COLOR_BLUE, frameZAxis, COLOR_BLUE)
