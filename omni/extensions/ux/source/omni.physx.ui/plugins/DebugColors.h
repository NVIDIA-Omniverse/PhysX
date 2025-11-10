// SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <stdint.h>

enum class DebugColor : uint32_t
{
    COLOR01,
    COLOR02,
    COLOR03,
    COLOR04,
    COLOR05,
    COLOR06,
    COLOR07,
    COLOR08,
    COLOR09,
    COLOR10,
    COLOR11,
    COLOR12,
    COLOR13,
    COLOR14,
    COLOR15,
    COLOR16,
    COLOR17,
    COLOR18,
    COLOR19,
    COLOR20,
    COLOR21,
    COLOR22,
    COLOR23,
    COLOR24,
    COLOR25,
    COLOR26,
    COLOR27,
    COLOR28,
    COLOR29,
    COLOR30,
    NUM_COLORS
};

uint32_t getDebugColor(DebugColor c); // return the debug color as a 32 bit integer in the format of ARGB
void getDebugColor(DebugColor c, float value[3]); // return the debug color as a float value RGB each 0-1

float getFloatColor(uint32_t v);
uint8_t getIntColor(float c);
uint32_t convertColor(uint8_t r, uint8_t g, uint8_t b, uint8_t a);
uint8_t getAlpha(uint32_t color);

// Convert rgb to hsv floats ([0-1],[0-1],[0-1])
void ColorConvertRGBtoHSV(uint32_t rgb, float& out_h, float& out_s, float& out_v);
// Convert hsv floats ([0-1],[0-1],[0-1],[0.255]) to rgba
uint32_t ColorConvertHSVtoRGB(float h, float s, float v, uint8_t alpha);
