// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2022-2024 NVIDIA Corporation. All rights reserved.


#ifndef NVBLASTEXTAUTHORINGPERLINNOISE_H
#define NVBLASTEXTAUTHORINGPERLINNOISE_H

#include <NvBlastExtAuthoringFractureTool.h>

#include "NvVec4.h"
#include "NvVec3.h"

#define PERLIN_NOISE_SAMPLE_TABLE 512
using nvidia::NvVec3;
namespace Nv
{
namespace Blast
{

/***********
    Noise generation routines, copied from Apex.    
*/


NV_INLINE float at3(const float& rx, const float& ry, const float& rz, const NvVec3 q)
{
    return rx * q[0] + ry * q[1] + rz * q[2];
}

NV_INLINE float fade(float t) { return t * t * t * (t * (t  * 6.0f - 15.0f) + 10.0f); }

NV_INLINE float lerp(float t, float a, float b) { return a + t * (b - a); }

NV_INLINE void setup(int i, NvVec3 point, float& t, int& b0, int& b1, float& r0, float& r1)
{
    t = point[i] + (0x1000);
    b0 = ((int)t) & (PERLIN_NOISE_SAMPLE_TABLE - 1);
    b1 = (b0 + 1)   & (PERLIN_NOISE_SAMPLE_TABLE - 1);
    r0 = t - (int)t;
    r1 = r0 - 1.0f;
}


NV_INLINE float noiseSample(NvVec3 point, int* p, NvVec3* g)
{
    int bx0, bx1, by0, by1, bz0, bz1, b00, b10, b01, b11;
    float rx0, rx1, ry0, ry1, rz0, rz1, sy, sz, a, b, c, d, t, u, v;
    NvVec3 q;
    int i, j;

    setup(0, point, t, bx0, bx1, rx0, rx1);
    setup(1, point, t, by0, by1, ry0, ry1);
    setup(2, point, t, bz0, bz1, rz0, rz1);

    i = p[bx0];
    j = p[bx1];

    b00 = p[i + by0];
    b10 = p[j + by0];
    b01 = p[i + by1];
    b11 = p[j + by1];

    t = fade(rx0);
    sy = fade(ry0);
    sz = fade(rz0);

    q = g[b00 + bz0]; u = at3(rx0, ry0, rz0, q);
    q = g[b10 + bz0]; v = at3(rx1, ry0, rz0, q);
    a = lerp(t, u, v);

    q = g[b01 + bz0]; u = at3(rx0, ry1, rz0, q);
    q = g[b11 + bz0]; v = at3(rx1, ry1, rz0, q);
    b = lerp(t, u, v);

    c = lerp(sy, a, b);

    q = g[b00 + bz1]; u = at3(rx0, ry0, rz1, q);
    q = g[b10 + bz1]; v = at3(rx1, ry0, rz1, q);
    a = lerp(t, u, v);

    q = g[b01 + bz1]; u = at3(rx0, ry1, rz1, q);
    q = g[b11 + bz1]; v = at3(rx1, ry1, rz1, q);
    b = lerp(t, u, v);

    d = lerp(sy, a, b);

    return lerp(sz, c, d);
}

/**
    Perlin Noise generation tool
*/
class PerlinNoise
{
public:
    /**
        \param[in] rnd Random value generator
        \param[in] octaves Number of noise octaves
        \param[in] frequency Frequency of noise
        \param[in] amplitude Amplitude of noise
    */
    PerlinNoise(Nv::Blast::RandomGeneratorBase* rnd, int octaves = 1, float frequency = 1., float amplitude = 1.)
        : mRnd(rnd),
        mOctaves(octaves),
        mFrequency(frequency),
        mAmplitude(amplitude),
        mbInit(false)
    {

    }

    /*
        Reset state of noise generator
        \param[in] octaves Number of noise octaves
        \param[in] frequency Frequency of noise
        \param[in] amplitude Amplitude of noise
    */
    void reset(int octaves = 1, float frequency = 1.f, float amplitude = 1.f)
    {
        mOctaves = octaves;
        mFrequency = frequency;
        mAmplitude = amplitude;
        init();
    }

    /**
        Get Perlin Noise value at given point
    */
    float sample(const nvidia::NvVec3& point)
    {
        return perlinNoise(point);
    }

private:
    PerlinNoise& operator=(const PerlinNoise&);

    float perlinNoise(nvidia::NvVec3 point)
    {
        if (!mbInit)
            init();

        const int octaves = mOctaves;
        const float frequency = mFrequency;
        float amplitude = mAmplitude;
        float result = 0.0f;

        point *= frequency;

        for (int i = 0; i < octaves; ++i)
        {
            NvVec3 lpnt;
            lpnt[0] = point.x;
            lpnt[1] = point.y;
            lpnt[2] = point.z;
            result += (noiseSample(lpnt, p, g)) * amplitude;
            point *= 2.0f;
            amplitude *= 0.5f;
        }
        return result;
    }

    void init(void)
    {
        mbInit = true;

        unsigned  i, j;
        int k;

        for (i = 0; i < (unsigned)PERLIN_NOISE_SAMPLE_TABLE; i++)
        {
            p[i] = (int)i;
            for (j = 0; j < 3; ++j)
                g[i][j] = mRnd->getRandomValue();
            g[i].normalize();
        }

        while (--i)
        {
            k = p[i];
            j = static_cast<uint32_t>(mRnd->getRandomValue() * PERLIN_NOISE_SAMPLE_TABLE);
            p[i] = p[j];
            p[j] = k;
        }

        for (i = 0; i < PERLIN_NOISE_SAMPLE_TABLE + 2; ++i)
        {
            p[(unsigned)PERLIN_NOISE_SAMPLE_TABLE + i] = p[i];
            for (j = 0; j < 3; ++j)
                g[(unsigned)PERLIN_NOISE_SAMPLE_TABLE + i][j] = g[i][j];
        }

    }

    Nv::Blast::RandomGeneratorBase* mRnd;
    int                             mOctaves;
    float                           mFrequency;
    float                           mAmplitude;

    // Permutation vector
    int                             p[(unsigned)(PERLIN_NOISE_SAMPLE_TABLE + PERLIN_NOISE_SAMPLE_TABLE + 2)];
    // Gradient vector
    NvVec3                          g[(unsigned)(PERLIN_NOISE_SAMPLE_TABLE + PERLIN_NOISE_SAMPLE_TABLE + 2)];

    bool                            mbInit;
};


/**
    Simplex noise generation tool
*/
class SimplexNoise
{

    int32_t             mOctaves;
    float               mAmplitude;
    float               mFrequency;
    int32_t             mSeed;

    static const int    X_NOISE_GEN = 1619;
    static const int    Y_NOISE_GEN = 31337;
    static const int    Z_NOISE_GEN = 6971;
    static const int    W_NOISE_GEN = 1999;
    static const int    SEED_NOISE_GEN = 1013;
    static const int    SHIFT_NOISE_GEN = 8;

    NV_INLINE int fastfloor(float x)
    {
        return (x >= 0) ? (int)x : (int)(x - 1);
    }

    SimplexNoise& operator=(const SimplexNoise&)
    {
        return *this;
    }

public:
    /**
        \param[in] ampl Amplitude of noise
        \param[in] freq Frequency of noise
        \param[in] octaves Number of noise octaves
        \param[in] seed Random seed value
    */
    SimplexNoise(float ampl, float freq, int32_t octaves, int32_t seed) : mOctaves(octaves), mAmplitude(ampl), mFrequency(freq), mSeed(seed) {};
    // 4D simplex noise
    // returns: (x,y,z) = noise grad, w = noise value

    /**
        Evaluate noise at given 4d-point
        \param[in] x x coordinate of point
        \param[in] y y coordinate of point
        \param[in] z z coordinate of point
        \param[in] w w coordinate of point
        \param[in] seed Random seed value
        \return Noise valued vector (x,y,z) and scalar (w)
    */
    nvidia::NvVec4 eval4D(float x, float y, float z, float w, int seed)
    {
        // The skewing and unskewing factors are hairy again for the 4D case
        const float F4 = (nvidia::NvSqrt(5.0f) - 1.0f) / 4.0f;
        const float G4 = (5.0f - nvidia::NvSqrt(5.0f)) / 20.0f;
        // Skew the (x,y,z,w) space to determine which cell of 24 simplices we're in
        float s = (x + y + z + w) * F4; // Factor for 4D skewing
        int ix = fastfloor(x + s);
        int iy = fastfloor(y + s);
        int iz = fastfloor(z + s);
        int iw = fastfloor(w + s);
        float tu = (ix + iy + iz + iw) * G4; // Factor for 4D unskewing
                                            // Unskew the cell origin back to (x,y,z,w) space
        float x0 = x - (ix - tu); // The x,y,z,w distances from the cell origin
        float y0 = y - (iy - tu);
        float z0 = z - (iz - tu);
        float w0 = w - (iw - tu);

        int c = (x0 > y0) ? (1 << 0) : (1 << 2);
        c += (x0 > z0) ? (1 << 0) : (1 << 4);
        c += (x0 > w0) ? (1 << 0) : (1 << 6);
        c += (y0 > z0) ? (1 << 2) : (1 << 4);
        c += (y0 > w0) ? (1 << 2) : (1 << 6);
        c += (z0 > w0) ? (1 << 4) : (1 << 6);

        nvidia::NvVec4 res;
        res.setZero();

        // Calculate the contribution from the five corners
        for (int p = 4; p >= 0; --p)
        {
            int ixp = ((c >> 0) & 3) >= p ? 1 : 0;
            int iyp = ((c >> 2) & 3) >= p ? 1 : 0;
            int izp = ((c >> 4) & 3) >= p ? 1 : 0;
            int iwp = ((c >> 6) & 3) >= p ? 1 : 0;

            float xp = x0 - ixp + (4 - p) * G4;
            float yp = y0 - iyp + (4 - p) * G4;
            float zp = z0 - izp + (4 - p) * G4;
            float wp = w0 - iwp + (4 - p) * G4;

            float t = 0.6f - xp * xp - yp * yp - zp * zp - wp * wp;
            if (t > 0)
            {
                //get index
                int gradIndex = int((
                    X_NOISE_GEN    * (ix + ixp)
                    + Y_NOISE_GEN    * (iy + iyp)
                    + Z_NOISE_GEN    * (iz + izp)
                    + W_NOISE_GEN    * (iw + iwp)
                    + SEED_NOISE_GEN * seed)
                    & 0xffffffff);
                gradIndex ^= (gradIndex >> SHIFT_NOISE_GEN);
                gradIndex &= 31;

                nvidia::NvVec4 g;
                {
                    const int h = gradIndex;
                    const int hs = 2 - (h >> 4);
                    const int h1 = (h >> 3);
                    g.x = (h1 == 0) ? 0.0f : ((h & 4) ? -1.0f : 1.0f);
                    g.y = (h1 == 1) ? 0.0f : ((h & (hs << 1)) ? -1.0f : 1.0f);
                    g.z = (h1 == 2) ? 0.0f : ((h & hs) ? -1.0f : 1.0f);
                    g.w = (h1 == 3) ? 0.0f : ((h & 1) ? -1.0f : 1.0f);
                }
                float gdot = (g.x * xp + g.y * yp + g.z * zp + g.w * wp);

                float t2 = t * t;
                float t3 = t2 * t;
                float t4 = t3 * t;

                float dt4gdot = 8 * t3 * gdot;

                res.x += t4 * g.x - dt4gdot * xp;
                res.y += t4 * g.y - dt4gdot * yp;
                res.z += t4 * g.z - dt4gdot * zp;
                res.w += t4 * gdot;
            }
        }
        // scale the result to cover the range [-1,1]
        res *= 27;
        return res;
    }

    /**
        Evaluate noise at given 3d-point
        \param[in] p Point in which noise will be evaluated
        \return Noise value at given point
    */
    float sample(nvidia::NvVec3 p)
    {
        p *= mFrequency;
        float result = 0.0f;
        float alpha = 1;
        for (int32_t i = 1; i <= mOctaves; ++i)
        {
            result += eval4D(p.x * i, p.y * i, p.z * i, i * 5.0f, mSeed).w * alpha;
            alpha *= 0.45f;
        }
        return result * mAmplitude;
    }

};


    } // Blast namespace
} // Nv namespace



#endif