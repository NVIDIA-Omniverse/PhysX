// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#include "UsdPCH.h"
#include "DebugColors.h"


static uint32_t gColors[]
{
0x00188f,
0x009e49,
0x00b294,
0x00bcf2,
0x17cea3,
0x34926e,
0x376495,
0x4398f2,
0x44ad57,
0x4ffccb,
0x68217a,
0x7db87d,
0x7fd51a,
0x88c674,
0x991b82,
0xa079cb,
0xb7a171,
0xbad80a,
0xc6e072,
0xc8745f,
0xd11c12,
0xd1f74d,
0xd2be4c,
0xe4b375,
0xe81123,
0xec008c,
0xf55f0d,
0xf7f219,
0xff8c00,
0xfff100
};


uint32_t getDebugColor(DebugColor c) // return the debug color as a 32 bit integer in the format of ARGB
{
    return gColors[uint32_t(c)];
}

float getFloatColor(uint32_t v)
{
    return float(v)/255.0f;
}

uint8_t getIntColor(float c)
{
    return uint8_t(c * 255.0f);
}

uint32_t convertColor(uint8_t r, uint8_t g, uint8_t b, uint8_t a)
{
    uint32_t outColor{ uint32_t(r) << 0 | uint32_t(g) << 8 | uint32_t(b) << 16 | uint32_t(a) << 24 };
    return outColor;
}

uint8_t getAlpha(uint32_t color)
{
    return uint8_t(color >> 24);
}

void getDebugColor(DebugColor c, float value[3]) // return the debug color as a float value RGB each 0-1
{
    uint32_t v = getDebugColor(c);
    value[0] = getFloatColor((v>>16)&0xFF);
    value[1] = getFloatColor((v>>8)&0xFF);
    value[2] = getFloatColor(v&0xFF);
}

// Convert rgb to hsv floats ([0-1],[0-1],[0-1])
void ColorConvertRGBtoHSV(uint32_t rgb, float& out_h, float& out_s, float& out_v)
{
    auto Swap = [](float& a, float& b) {
        float tmp = a;
        a = b;
        b = tmp;
    };

    float r = getFloatColor((rgb >> 16) & 0xFF);
    float g = getFloatColor((rgb >> 8) & 0xFF);
    float b = getFloatColor(rgb & 0xFF);

    float K = 0.f;
    if (g < b)
    {
        Swap(g, b);
        K = -1.f;
    }
    if (r < g)
    {
        Swap(r, g);
        K = -2.f / 6.f - K;
    }

    const float chroma = r - (g < b ? g : b);
    out_h = fabsf(K + (g - b) / (6.f * chroma + 1e-20f));
    out_s = chroma / (r + 1e-20f);
    out_v = r;
}

// Convert hsv floats ([0-1],[0-1],[0-1]) to rgb
uint32_t ColorConvertHSVtoRGB(float h, float s, float v, uint8_t alpha)
{
    float out_r, out_g, out_b;

    if (s == 0.0f)
    {
        // gray
        out_r = out_g = out_b = v;
        return convertColor(getIntColor(out_r), getIntColor(out_g), getIntColor(out_b), alpha);
    }

    h = fmodf(h, 1.0f) / (60.0f / 360.0f);
    int   i = (int)h;
    float f = h - (float)i;
    float p = v * (1.0f - s);
    float q = v * (1.0f - s * f);
    float t = v * (1.0f - s * (1.0f - f));

    switch (i)
    {
    case 0: out_r = v; out_g = t; out_b = p; break;
    case 1: out_r = q; out_g = v; out_b = p; break;
    case 2: out_r = p; out_g = v; out_b = t; break;
    case 3: out_r = p; out_g = q; out_b = v; break;
    case 4: out_r = t; out_g = p; out_b = v; break;
    case 5: default: out_r = v; out_g = p; out_b = q; break;
    }

    return convertColor(getIntColor(out_r), getIntColor(out_g), getIntColor(out_b), alpha);
}
