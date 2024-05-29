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
// Copyright (c) 2016-2024 NVIDIA Corporation. All rights reserved.


#ifndef NVBLASTAUTHORINGFCUTOUTIMPL_H
#define NVBLASTAUTHORINGFCUTOUTIMPL_H

#include "NvBlastExtAuthoringCutout.h"
#include <vector>
#include "NvVec2.h"
#include "NvVec3.h"
#include "NvMat44.h"

namespace Nv
{
namespace Blast
{

struct PolyVert
{
    uint16_t index;
    uint16_t flags;
};

struct ConvexLoop
{
    std::vector<PolyVert> polyVerts;
};

struct Cutout
{
    std::vector<nvidia::NvVec3> vertices;
    //std::vector<ConvexLoop> convexLoops;
    std::vector<nvidia::NvVec3> smoothingGroups;
};

struct POINT2D
{
    POINT2D() {}
    POINT2D(int32_t _x, int32_t _y) : x(_x), y(_y) {}

    int32_t x;
    int32_t y;

    bool operator==(const POINT2D& other) const
    {
        return x == other.x && y == other.y;
    }
    bool operator<(const POINT2D& other) const
    {
        if (x == other.x) return y < other.y;
        return x < other.x;
    }
};

struct CutoutSetImpl : public CutoutSet
{
    CutoutSetImpl() : periodic(false), dimensions(0.0f)
    {
    }

    uint32_t            getCutoutCount() const
    {
        return (uint32_t)cutouts.size() - 1;
    }

    uint32_t            getCutoutVertexCount(uint32_t cutoutIndex, uint32_t loopIndex) const
    {
        return (uint32_t)cutoutLoops[cutouts[cutoutIndex] + loopIndex].vertices.size();
    }
    uint32_t            getCutoutLoopCount(uint32_t cutoutIndex) const
    {
        return (uint32_t)cutouts[cutoutIndex + 1] - cutouts[cutoutIndex];
    }

    const NvcVec3& getCutoutVertex(uint32_t cutoutIndex, uint32_t loopIndex, uint32_t vertexIndex) const;

    bool                isCutoutVertexToggleSmoothingGroup(uint32_t cutoutIndex, uint32_t loopIndex, uint32_t vertexIndex) const
    {
        auto& vRef = cutoutLoops[cutouts[cutoutIndex] + loopIndex].vertices[vertexIndex];
        for (auto& v : cutoutLoops[cutouts[cutoutIndex] + loopIndex].smoothingGroups)
        {
            if ((vRef - v).magnitudeSquared() < 1e-5)
            {
                return true;
            }
        }
        return false;
    }

    bool                    isPeriodic() const
    {
        return periodic;
    }
    const NvcVec2& getDimensions() const;

    //void                  serialize(nvidia::NvFileBuf& stream) const;
    //void                  deserialize(nvidia::NvFileBuf& stream);

    void                    release()
    {
        delete this;
    }

    std::vector<Cutout>     cutoutLoops;
    std::vector<uint32_t>   cutouts;
    bool                    periodic;
    nvidia::NvVec2           dimensions;
};

void createCutoutSet(Nv::Blast::CutoutSetImpl& cutoutSet, const uint8_t* pixelBuffer, uint32_t bufferWidth, uint32_t bufferHeight,
    float segmentationErrorThreshold, float snapThreshold, bool periodic, bool expandGaps);


} // namespace Blast
} // namespace Nv

#endif // ifndef NVBLASTAUTHORINGFCUTOUTIMPL_H
