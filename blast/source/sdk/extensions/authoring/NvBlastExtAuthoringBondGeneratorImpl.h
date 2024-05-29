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


#ifndef NVBLASTEXTAUTHORINGBONDGENERATORIMPL_H
#define NVBLASTEXTAUTHORINGBONDGENERATORIMPL_H

#include "NvBlastExtAuthoringBondGenerator.h"
#include "NvBlastExtAuthoringFractureTool.h"
#include "NvPlane.h"
#include <NvBlastExtAuthoringConvexMeshBuilder.h>
#include <vector>
#include <set>

namespace Nv
{
namespace Blast
{

/**
    Tool for gathering bond information from provided mesh geometry
*/

class BlastBondGeneratorImpl : public BlastBondGenerator
{
public: 
                
    BlastBondGeneratorImpl(ConvexMeshBuilder* builder) 
        : mConvexMeshBuilder(builder) {};

    virtual void release() override;

    virtual int32_t buildDescFromInternalFracture(FractureTool* tool, const bool* chunkIsSupport,
        NvBlastBondDesc*& resultBondDescs, NvBlastChunkDesc*& resultChunkDescriptors)  override;

    virtual int32_t createBondBetweenMeshes(uint32_t meshACount, const Triangle* meshA, uint32_t meshBCount, const Triangle* meshB,
        NvBlastBond& resultBond, BondGenerationConfig conf) override;

    virtual int32_t createBondBetweenMeshes(uint32_t meshCount, const uint32_t* geometryOffset, const Triangle* geometry,
        uint32_t overlapsCount, const uint32_t* overlapsA, const uint32_t* overlapsB,
        NvBlastBondDesc*& resultBond, BondGenerationConfig cfg) override;

    virtual int32_t bondsFromPrefractured(uint32_t meshCount, const uint32_t* geometryOffset, const Triangle* geometry,
        const bool* chunkIsSupport, NvBlastBondDesc*& resultBondDescs,
        BondGenerationConfig conf) override;

    virtual int32_t bondsFromPrefractured(uint32_t meshCount, const uint32_t* convexHullOffset, const CollisionHull** chunkHulls,
        const bool* chunkIsSupport, const uint32_t* meshGroups, NvBlastBondDesc*& resultBondDescs, float maxSeparation) override;


                
private:
    float   processWithMidplanes(TriangleProcessor* trProcessor, const Triangle* mA, uint32_t mavc, const Triangle* mB, uint32_t mbvc, const CollisionHull* hull1, const CollisionHull* hull2,
                             const std::vector<nvidia::NvVec3>& hull1p, const std::vector<nvidia::NvVec3>& hull2p,
                             nvidia::NvVec3& normal, nvidia::NvVec3& centroid, float maxRelSeparation);

    int32_t createFullBondListAveraged( uint32_t meshCount, const uint32_t* geometryOffset, const Triangle* geometry, const CollisionHull** chunkHulls,
                                        const bool* supportFlags, const uint32_t* meshGroups, NvBlastBondDesc*& resultBondDescs, BondGenerationConfig conf, std::set<std::pair<uint32_t, uint32_t> >* pairNotToTest = nullptr);
    int32_t createFullBondListExact(    uint32_t meshCount, const uint32_t* geometryOffset, const Triangle* geometry,
                                        const bool* supportFlags, NvBlastBondDesc*& resultBondDescs, BondGenerationConfig conf);
    int32_t createFullBondListExactInternal(uint32_t meshCount, const uint32_t* geometryOffset, const Triangle* geometry,
                                            std::vector<PlaneChunkIndexer>& planeTriangleMapping , NvBlastBondDesc*& resultBondDescs);
    int32_t createBondForcedInternal(   const std::vector<nvidia::NvVec3>& hull0, const std::vector<nvidia::NvVec3>& hull1,const CollisionHull& cHull0, 
                                        const CollisionHull& cHull1, nvidia::NvBounds3 bound0, nvidia::NvBounds3 bound1, NvBlastBond& resultBond, float overlapping);

    void    buildGeometryCache(uint32_t meshCount, const uint32_t* geometryOffset, const Triangle* geometry);
    void    resetGeometryCache();

    ConvexMeshBuilder*                          mConvexMeshBuilder;

    std::vector<std::vector<Triangle> >         mGeometryCache;

    std::vector<PlaneChunkIndexer>              mPlaneCache;
    std::vector<CollisionHull*>                 mCHullCache;
    std::vector<std::vector<nvidia::NvVec3> >    mHullsPointsCache;
    std::vector<nvidia::NvBounds3 >              mBoundsCache;


};

}   // namespace Blast
}   // namespace Nv

#endif // NVBLASTEXTAUTHORINGBONDGENERATORIMPL_H