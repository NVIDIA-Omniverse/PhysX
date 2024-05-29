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

#include "NvBlastExtAuthoring.h"
#include "NvBlastTypes.h"
#include "NvBlastIndexFns.h"
#include "NvBlast.h"
#include "NvBlastAssert.h"
#include "NvBlastGlobals.h"
#include "NvBlastExtAssetUtils.h"
#include "NvBlastExtAuthoringPatternGeneratorImpl.h"
#include "NvBlastExtAuthoringBooleanToolImpl.h"
#include "NvBlastExtAuthoringAcceleratorImpl.h"
#include "NvBlastExtAuthoringMeshImpl.h"
#include "NvBlastExtAuthoringMeshCleanerImpl.h"
#include "NvBlastExtAuthoringFractureToolImpl.h"
#include "NvBlastExtAuthoringBondGeneratorImpl.h"
#include "NvBlastExtAuthoringCollisionBuilderImpl.h"
#include "NvBlastExtAuthoringCutoutImpl.h"
#include "NvBlastExtAuthoringInternalCommon.h"
#include "NvBlastNvSharedHelpers.h"

#include <algorithm>
#include <memory>

using namespace Nv::Blast;
using namespace nvidia;

#define SAFE_ARRAY_NEW(T, x) ((x) > 0) ? reinterpret_cast<T*>(NVBLAST_ALLOC(sizeof(T) * (x))) : nullptr;
#define SAFE_ARRAY_DELETE(x) if (x != nullptr) {NVBLAST_FREE(x); x = nullptr;}

Mesh* NvBlastExtAuthoringCreateMesh(const NvcVec3* position, const NvcVec3* normals, const NvcVec2* uv, uint32_t verticesCount, const uint32_t* indices, uint32_t indicesCount)
{
    return new MeshImpl(position, normals, uv, verticesCount, indices, indicesCount);
}

Mesh* NvBlastExtAuthoringCreateMeshOnlyTriangles(const void* Vertices, uint32_t vcount, uint32_t* indices, uint32_t indexCount, void* materials, uint32_t materialStride)
{
    return new MeshImpl((Vertex*)Vertices, vcount, indices, indexCount, materials, materialStride);
}

Mesh* NvBlastExtAuthoringCreateMeshFromFacets(const void* vertices, const void* edges, const void* facets, uint32_t verticesCount, uint32_t edgesCount, uint32_t facetsCount)
{
    return new MeshImpl((Vertex*)vertices, (Edge*)edges, (Facet*)facets, verticesCount, edgesCount, facetsCount);
}

MeshCleaner* NvBlastExtAuthoringCreateMeshCleaner()
{
    return new MeshCleanerImpl;
}

VoronoiSitesGenerator* NvBlastExtAuthoringCreateVoronoiSitesGenerator(Mesh* mesh, RandomGeneratorBase* rng)
{
    return new VoronoiSitesGeneratorImpl(mesh, rng);
}

CutoutSet* NvBlastExtAuthoringCreateCutoutSet()
{
    return new CutoutSetImpl();
}

void NvBlastExtAuthoringBuildCutoutSet(CutoutSet& cutoutSet, const uint8_t* pixelBuffer, uint32_t bufferWidth, uint32_t bufferHeight, 
    float segmentationErrorThreshold, float snapThreshold, bool periodic, bool expandGaps)
{
    ::createCutoutSet(*(CutoutSetImpl*)&cutoutSet, pixelBuffer, bufferWidth, bufferHeight, segmentationErrorThreshold, snapThreshold, periodic, expandGaps);
}

FractureTool* NvBlastExtAuthoringCreateFractureTool()
{
    return new FractureToolImpl;
}

BlastBondGenerator* NvBlastExtAuthoringCreateBondGenerator(Nv::Blast::ConvexMeshBuilder* builder)
{
    return new BlastBondGeneratorImpl(builder);
}

int32_t NvBlastExtAuthoringBuildMeshConvexDecomposition(ConvexMeshBuilder* cmb, const Nv::Blast::Triangle* mesh,
                                                                    uint32_t triangleCount,
                                                                    const ConvexDecompositionParams& params,
                                                                    CollisionHull**& convexes)
{
    NVBLAST_ASSERT(cmb != nullptr);
    return buildMeshConvexDecomposition(*cmb, mesh, triangleCount, params, convexes);
}


void NvBlastExtAuthoringTrimCollisionGeometry(ConvexMeshBuilder* cmb, uint32_t chunksCount,
                                              Nv::Blast::CollisionHull** in, const uint32_t* chunkDepth)
{
    return trimCollisionGeometry(*cmb, chunksCount, in, chunkDepth);
}

void NvBlastExtAuthoringTransformCollisionHullInPlace(CollisionHull* hull, const NvcVec3* scaling, const NvcQuat* rotation, const NvcVec3* translation)
{
    // Local copies of scaling (S), rotation (R), and translation (T)
    nvidia::NvVec3 S = { 1, 1, 1 };
    nvidia::NvQuat R = { 0, 0, 0, 1 };
    nvidia::NvVec3 T = { 0, 0, 0 };
    nvidia::NvVec3 cofS = { 1, 1, 1 };
    float sgnDetS = 1;

    {
        if (rotation)
        {
            R = *toNvShared(rotation);
        }

        if (scaling)
        {
            S       = *toNvShared(scaling);
            cofS.x = S.y * S.z;
            cofS.y = S.z * S.x;
            cofS.z = S.x * S.y;
            sgnDetS = (S.x * S.y * S.z < 0) ? -1 : 1;
        }

        if (translation)
        {
            T = *toNvShared(translation);
        }
    }

    const uint32_t pointCount = hull->pointsCount;
    for (uint32_t pi = 0; pi < pointCount; pi++)
    {
        nvidia::NvVec3& p = toNvShared(hull->points[pi]);
        p = (R.rotate(p.multiply(S)) + T);
    }
    
    const uint32_t planeCount = hull->polygonDataCount;
    for (uint32_t pi = 0; pi < planeCount; pi++)
    {
        float* plane = hull->polygonData[pi].plane;
        nvidia::NvPlane nvPlane(plane[0], plane[1], plane[2], plane[3]);
        NvVec3 transformedNormal = sgnDetS*R.rotate(nvPlane.n.multiply(cofS)).getNormalized();
        NvVec3 transformedPt = R.rotate(nvPlane.pointInPlane().multiply(S)) + T;
        
        nvidia::NvPlane transformedPlane(transformedPt, transformedNormal);
        plane[0] = transformedPlane.n[0];
        plane[1] = transformedPlane.n[1];
        plane[2] = transformedPlane.n[2];
        plane[3] = transformedPlane.d;
    }
}


CollisionHull* NvBlastExtAuthoringTransformCollisionHull(const CollisionHull* hull, const NvcVec3* scaling, const NvcQuat* rotation, const NvcVec3* translation)
{
    CollisionHull* ret = new CollisionHull(*hull);
    ret->points             = SAFE_ARRAY_NEW(NvcVec3, ret->pointsCount);
    ret->indices            = SAFE_ARRAY_NEW(uint32_t, ret->indicesCount);
    ret->polygonData        = SAFE_ARRAY_NEW(HullPolygon, ret->polygonDataCount);
    memcpy(ret->points, hull->points, sizeof(ret->points[0]) * ret->pointsCount);
    memcpy(ret->indices, hull->indices, sizeof(ret->indices[0]) * ret->indicesCount);
    memcpy(ret->polygonData, hull->polygonData, sizeof(ret->polygonData[0]) * ret->polygonDataCount);
    NvBlastExtAuthoringTransformCollisionHullInPlace(ret, scaling, rotation, translation);
    return ret;
}

void buildPhysicsChunks(ConvexMeshBuilder& collisionBuilder, AuthoringResult& result, const ConvexDecompositionParams& params, uint32_t chunksToProcessCount = 0, uint32_t* chunksToProcess = nullptr)
{
    uint32_t chunkCount = (uint32_t)result.chunkCount;
    if (params.maximumNumberOfHulls == 1)
    {
        result.collisionHullOffset = SAFE_ARRAY_NEW(uint32_t, chunkCount + 1);
        result.collisionHullOffset[0] = 0;
        result.collisionHull = SAFE_ARRAY_NEW(CollisionHull*, chunkCount);
        for (uint32_t i = 0; i < chunkCount; ++i)
        {
            std::vector<NvcVec3> vertices;
            for (uint32_t p = result.geometryOffset[i]; p < result.geometryOffset[i + 1]; ++p)
            {
                Nv::Blast::Triangle& tri = result.geometry[p];
                vertices.push_back(tri.a.p);
                vertices.push_back(tri.b.p);
                vertices.push_back(tri.c.p);
            }
            result.collisionHullOffset[i + 1] = result.collisionHullOffset[i] + 1;
            result.collisionHull[i] = collisionBuilder.buildCollisionGeometry((uint32_t)vertices.size(), vertices.data());
        }
    }
    else
    {
        std::set<int32_t> chunkSet;
        for (uint32_t c = 0; c < chunksToProcessCount; c++)
        {
            chunkSet.insert(chunksToProcess[c]);
        }
        std::vector<std::vector<CollisionHull*> > hulls(chunkCount);
        int32_t totalHulls = 0;
        for (uint32_t i = 0; i < chunkCount; ++i)
        {
            if (chunkSet.size() > 0 && chunkSet.find(i) == chunkSet.end())
            {
                int32_t newHulls = result.collisionHullOffset[i + 1] - result.collisionHullOffset[i];
                int32_t off = result.collisionHullOffset[i];
                for (int32_t subhull = 0; subhull < newHulls; ++subhull)
                {
                    hulls[i].push_back(result.collisionHull[off + subhull]);
                }
                totalHulls += newHulls;
                continue;
            }

            CollisionHull** tempHull;

            int32_t newHulls =
                buildMeshConvexDecomposition(collisionBuilder, result.geometry + result.geometryOffset[i], 
                                                result.geometryOffset[i + 1] - result.geometryOffset[i], params, tempHull);
            totalHulls += newHulls;         
            for (int32_t h = 0; h < newHulls; ++h)
            {
                hulls[i].push_back(tempHull[h]);
            }
            SAFE_ARRAY_DELETE(tempHull);
        }

        result.collisionHullOffset = SAFE_ARRAY_NEW(uint32_t, chunkCount + 1);
        result.collisionHullOffset[0] = 0;
        result.collisionHull = SAFE_ARRAY_NEW(CollisionHull*, totalHulls);

        for (uint32_t i = 0; i < chunkCount; ++i)
        {
            result.collisionHullOffset[i + 1] = result.collisionHullOffset[i] + hulls[i].size();
            int32_t off = result.collisionHullOffset[i];
            for (uint32_t subhull = 0; subhull < hulls[i].size(); ++subhull)
            {
                result.collisionHull[off + subhull] = hulls[i][subhull];
            }           
        }
    }
}


void NvBlastExtAuthoringReleaseAuthoringResultCollision(Nv::Blast::ConvexMeshBuilder& collisionBuilder, Nv::Blast::AuthoringResult* ar)
{
    if (ar->collisionHull != nullptr)
    {
        for (uint32_t ch = 0; ch < ar->collisionHullOffset[ar->chunkCount]; ch++)
        {
            collisionBuilder.releaseCollisionHull(ar->collisionHull[ch]);
        }
        SAFE_ARRAY_DELETE(ar->collisionHullOffset);
        SAFE_ARRAY_DELETE(ar->collisionHull);
    }
}

void NvBlastExtAuthoringReleaseAuthoringResult(Nv::Blast::ConvexMeshBuilder& collisionBuilder, Nv::Blast::AuthoringResult* ar)
{
    NvBlastExtAuthoringReleaseAuthoringResultCollision(collisionBuilder, ar);
    if (ar->asset)
    {
        NVBLAST_FREE(ar->asset);
        ar->asset = nullptr;
    }
    SAFE_ARRAY_DELETE(ar->assetToFractureChunkIdMap);
    SAFE_ARRAY_DELETE(ar->geometryOffset);
    SAFE_ARRAY_DELETE(ar->geometry);
    SAFE_ARRAY_DELETE(ar->chunkDescs);
    SAFE_ARRAY_DELETE(ar->bondDescs);
    delete ar;
}

static float getGeometryVolumeAndCentroid(NvcVec3& centroid, const Nv::Blast::Triangle* tris, size_t triCount)
{
    class GeometryQuery
    {
    public:
        GeometryQuery(const Nv::Blast::Triangle* tris, size_t triCount) : m_tris(tris), m_triCount(triCount) {}

        size_t faceCount() const { return m_triCount; }

        size_t vertexCount(size_t faceIndex) const { NV_UNUSED(faceIndex); return 3; }

        NvcVec3 vertex(size_t faceIndex, size_t vertexIndex) const
        {
            const Nv::Blast::Triangle& tri = m_tris[faceIndex];
            switch (vertexIndex)
            {
            case 0: return tri.a.p;
            case 1: return tri.b.p;
            case 2: return tri.c.p;
            }
            return NvcVec3({0.0f, 0.0f, 0.0f});
        }

        const Nv::Blast::Triangle* m_tris;  
        size_t m_triCount;
    };

    return calculateMeshVolumeAndCentroid<GeometryQuery>(centroid, {tris, triCount});
}

AuthoringResult* NvBlastExtAuthoringProcessFracture(FractureTool& fTool, BlastBondGenerator& bondGenerator, ConvexMeshBuilder& collisionBuilder, const ConvexDecompositionParams& collisionParam, int32_t defaultSupportDepth)
{
    fTool.finalizeFracturing();
    const uint32_t chunkCount = fTool.getChunkCount();
    if (chunkCount == 0)
    {
        return nullptr;
    }
    AuthoringResult* ret = new AuthoringResult;
    if (ret == nullptr)
    {
        return nullptr;
    }
    AuthoringResult& aResult = *ret;
    aResult.chunkCount = chunkCount;

    std::shared_ptr<bool> isSupport(new bool[chunkCount], [](bool* b) {delete[] b; });
    memset(isSupport.get(), 0, sizeof(bool) * chunkCount);
    for (uint32_t i = 0; i < fTool.getChunkCount(); ++i)
    {
        if (defaultSupportDepth < 0 || fTool.getChunkDepth(fTool.getChunkId(i)) < defaultSupportDepth)
        {
            isSupport.get()[i] = fTool.getChunkInfo(i).isLeaf;
        }
        else if (fTool.getChunkDepth(fTool.getChunkId(i)) == defaultSupportDepth)
        {
            isSupport.get()[i] = true;
        }
    }



    const uint32_t bondCount = bondGenerator.buildDescFromInternalFracture(&fTool, isSupport.get(), aResult.bondDescs, aResult.chunkDescs);
    aResult.bondCount = bondCount;
    if (bondCount == 0)
    {
        aResult.bondDescs = nullptr;
    }

    // order chunks, build map
    std::vector<uint32_t> chunkReorderInvMap;
    {
        std::vector<uint32_t> chunkReorderMap(chunkCount);
        std::vector<char> scratch(chunkCount * sizeof(NvBlastChunkDesc));
        NvBlastEnsureAssetExactSupportCoverage(aResult.chunkDescs, chunkCount, scratch.data(), logLL);
        NvBlastBuildAssetDescChunkReorderMap(chunkReorderMap.data(), aResult.chunkDescs, chunkCount, scratch.data(), logLL);
        NvBlastApplyAssetDescChunkReorderMapInPlace(aResult.chunkDescs, chunkCount, aResult.bondDescs, bondCount, chunkReorderMap.data(), true, scratch.data(), logLL);
        chunkReorderInvMap.resize(chunkReorderMap.size());
        Nv::Blast::invertMap(chunkReorderInvMap.data(), chunkReorderMap.data(), static_cast<unsigned int>(chunkReorderMap.size()));
    }

    // get result geometry
    aResult.geometryOffset = SAFE_ARRAY_NEW(uint32_t, chunkCount + 1);
    aResult.assetToFractureChunkIdMap = SAFE_ARRAY_NEW(uint32_t, chunkCount + 1);
    aResult.geometryOffset[0] = 0;
    std::vector<Nv::Blast::Triangle*> chunkGeometry(chunkCount);
    for (uint32_t i = 0; i < chunkCount; ++i)
    {
        uint32_t chunkInfoIndex = chunkReorderInvMap[i];
        aResult.geometryOffset[i+1] = aResult.geometryOffset[i] + fTool.getBaseMesh(chunkInfoIndex, chunkGeometry[i]);
        aResult.assetToFractureChunkIdMap[i] = fTool.getChunkId(chunkInfoIndex);
    }
    aResult.geometry = SAFE_ARRAY_NEW(Triangle, aResult.geometryOffset[chunkCount]);
    for (uint32_t i = 0; i < chunkCount; ++i)
    {
        uint32_t trianglesCount = aResult.geometryOffset[i + 1] - aResult.geometryOffset[i];
        memcpy(aResult.geometry + aResult.geometryOffset[i], chunkGeometry[i], trianglesCount * sizeof(Nv::Blast::Triangle));
        delete chunkGeometry[i];
        chunkGeometry[i] = nullptr;
    }

    float maxX = FLT_MAX;
    float maxY = FLT_MAX;
    float maxZ = FLT_MAX;

    float minX = -FLT_MAX;
    float minY = -FLT_MAX;
    float minZ = -FLT_MAX;

    for (uint32_t i = 0; i < bondCount; i++)
    {
        NvBlastBondDesc& bondDesc = aResult.bondDescs[i];

        minX = std::min(minX, bondDesc.bond.centroid[0]);
        maxX = std::max(maxX, bondDesc.bond.centroid[0]);

        minY = std::min(minY, bondDesc.bond.centroid[1]);
        maxY = std::max(maxY, bondDesc.bond.centroid[1]);

        minZ = std::min(minZ, bondDesc.bond.centroid[2]);
        maxZ = std::max(maxZ, bondDesc.bond.centroid[2]);
    }

    // prepare physics data (convexes)
    buildPhysicsChunks(collisionBuilder, aResult, collisionParam);

    // set NvBlastChunk volume and centroid from CollisionHull
    for (uint32_t i = 0; i < chunkCount; i++)
    {
        float totalVolume = 0.f;
        NvcVec3 totalCentroid = {0.0f, 0.0f, 0.0f};
        for (uint32_t k = aResult.collisionHullOffset[i]; k < aResult.collisionHullOffset[i+1]; k++)
        {
            const CollisionHull* hull = aResult.collisionHull[k];
            if (hull)
            {
                NvcVec3 centroid;
                const float volume = calculateCollisionHullVolumeAndCentroid(centroid, *hull);
                totalVolume += volume;
                totalCentroid = totalCentroid + volume*centroid;
            }
            else
            {
                totalVolume = 0.0f; // Found a null hull, signal this with zero volume
                break;
            }
        }
        if (totalVolume > 0.0f)
        {
            totalCentroid = totalCentroid / totalVolume;
            aResult.chunkDescs[i].volume = totalVolume;
            aResult.chunkDescs[i].centroid[0] = totalCentroid.x;
            aResult.chunkDescs[i].centroid[1] = totalCentroid.y;
            aResult.chunkDescs[i].centroid[2] = totalCentroid.z;
        }
        else
        {
            // Fallback to using mesh
            size_t triCount = aResult.geometryOffset[i+1] - aResult.geometryOffset[i];
            const Nv::Blast::Triangle* tris = aResult.geometry + aResult.geometryOffset[i];
            NvcVec3 centroid;
            aResult.chunkDescs[i].volume = getGeometryVolumeAndCentroid(centroid, tris, triCount);
            aResult.chunkDescs[i].centroid[0] = centroid.x;
            aResult.chunkDescs[i].centroid[1] = centroid.y;
            aResult.chunkDescs[i].centroid[2] = centroid.z;
        }
        
    }

    // build and serialize ExtPhysicsAsset
    NvBlastAssetDesc    descriptor;
    descriptor.bondCount = bondCount;
    descriptor.bondDescs = aResult.bondDescs;
    descriptor.chunkCount = chunkCount;
    descriptor.chunkDescs = aResult.chunkDescs;

    std::vector<uint8_t> scratch(static_cast<unsigned int>(NvBlastGetRequiredScratchForCreateAsset(&descriptor, logLL)));
    void* mem = NVBLAST_ALLOC(NvBlastGetAssetMemorySize(&descriptor, logLL));
    aResult.asset = NvBlastCreateAsset(mem, &descriptor, scratch.data(), logLL);

    //aResult.asset = std::shared_ptr<NvBlastAsset>(asset, [=](NvBlastAsset* asset)
    //{
    //  NVBLAST_FREE(asset);
    //});

    //std::cout << "Done" << std::endl;
    ret->materialCount = 0;
    ret->materialNames = nullptr;
    return ret;
}

uint32_t NvBlastExtAuthoringFindAssetConnectingBonds
(
    const NvBlastAsset** components,
    const NvcVec3* scales,
    const NvcQuat* rotations,
    const NvcVec3* translations,
    const uint32_t** convexHullOffsets,
    const CollisionHull*** chunkHulls,
    uint32_t componentCount,
    NvBlastExtAssetUtilsBondDesc*& newBondDescs,
    float maxSeparation
)
{
    //We don't need to use any of the cooking related parts of this
    BlastBondGeneratorImpl bondGenerator(nullptr);

    std::vector<uint32_t> componentChunkOffsets;
    componentChunkOffsets.reserve(componentCount + 1);
    componentChunkOffsets.push_back(0);
    
    std::vector<uint32_t> combinedConvexHullOffsets;
    std::vector<const CollisionHull*> combinedConvexHulls;
    std::vector<CollisionHull*> hullsToRelease;
    combinedConvexHullOffsets.push_back(0);

    std::vector<uint32_t> originalComponentIndex;

    const nvidia::NvVec3 identityScale(1);

    //Combine our hull lists into a single combined list for bondsFromPrefractured
    for (uint32_t c = 0; c < componentCount; c++)
    {
        const uint32_t chunkCount = NvBlastAssetGetChunkCount(components[c], &logLL);
        const NvcVec3* scale = scales ? scales + c : nullptr;
        const NvcQuat* rotation          = rotations ? rotations + c : nullptr;
        const NvcVec3* translation = translations ? translations + c : nullptr;

        componentChunkOffsets.push_back(chunkCount + componentChunkOffsets.back());
        for (uint32_t chunk = 0; chunk < chunkCount; chunk++)
        {
            const uint32_t hullsStart = convexHullOffsets[c][chunk];
            const uint32_t hullsEnd = convexHullOffsets[c][chunk + 1];
            for (uint32_t hull = hullsStart; hull < hullsEnd; hull++)
            {
                if ((scale != nullptr && *toNvShared(scale) != identityScale) ||
                    (rotation != nullptr && !toNvShared(rotation)->isIdentity()) ||
                    (translation != nullptr && !toNvShared(translation)->isZero()))
                {
                    hullsToRelease.emplace_back(NvBlastExtAuthoringTransformCollisionHull(chunkHulls[c][hull], scale, rotation, translation));
                    combinedConvexHulls.emplace_back(hullsToRelease.back());
                }
                else
                {
                    //No need to transform
                    combinedConvexHulls.emplace_back(chunkHulls[c][hull]);
                }
            }
            combinedConvexHullOffsets.push_back((hullsEnd - hullsStart) + combinedConvexHullOffsets.back());
            originalComponentIndex.push_back(c);
        }
    }
    const uint32_t totalChunkCount = componentChunkOffsets.back();
    //Can't use std::vector<bool> since we need a bool* later
    std::unique_ptr<bool[]> isSupportChunk(new bool[totalChunkCount]);
    for (uint32_t c = 0; c < componentCount; c++)
    {
        const uint32_t chunkCount = componentChunkOffsets[c + 1] - componentChunkOffsets[c];
        NvBlastSupportGraph supportGraph = NvBlastAssetGetSupportGraph(components[c], &logLL);
        for (uint32_t chunk = 0; chunk < chunkCount; chunk++)
        {
            auto chunkIndiciesEnd = supportGraph.chunkIndices + supportGraph.nodeCount;
            isSupportChunk[chunk + componentChunkOffsets[c]] = (std::find(supportGraph.chunkIndices, chunkIndiciesEnd, chunk) != chunkIndiciesEnd);
        }
    }

    //Find the bonds
    NvBlastBondDesc* newBonds = nullptr;
    const int32_t newBoundCount = bondGenerator.bondsFromPrefractured(totalChunkCount, combinedConvexHullOffsets.data(), combinedConvexHulls.data(), isSupportChunk.get(), originalComponentIndex.data(), newBonds, maxSeparation);

    //Convert the bonds back to per-component chunks
    newBondDescs = SAFE_ARRAY_NEW(NvBlastExtAssetUtilsBondDesc, newBoundCount);
    for (int32_t nb = 0; nb < newBoundCount; ++nb)
    {
        newBondDescs[nb].bond = newBonds[nb].bond;
        for (uint32_t ci = 0; ci < 2; ++ci)
        {
            uint32_t absChunkIdx = newBonds[nb].chunkIndices[ci];
            uint32_t componentIdx = originalComponentIndex[absChunkIdx];
            newBondDescs[nb].componentIndices[ci] = componentIdx;
            newBondDescs[nb].chunkIndices[ci] = absChunkIdx - componentChunkOffsets[componentIdx];
        }
    }
    //Don't need this anymore
    NVBLAST_FREE(newBonds);

    // These hulls were generated by NvBlastExtAuthoringTransformCollisionHull, which uses SAFE_ARRAY_NEW
    // to allocate the arrays referenced in each hull.  Be sure to delete the array pointers here before
    // deleting the CollisionHull structs. 
    for (CollisionHull* hull : hullsToRelease)
    {
        SAFE_ARRAY_DELETE(hull->indices);
        SAFE_ARRAY_DELETE(hull->points);
        SAFE_ARRAY_DELETE(hull->polygonData);
        delete hull;
    }

    return newBoundCount;
}


void NvBlastExtAuthoringUpdateGraphicsMesh(Nv::Blast::FractureTool& fTool, Nv::Blast::AuthoringResult& aResult)
{
    uint32_t chunkCount = fTool.getChunkCount();
    for (uint32_t i = 0; i < chunkCount; ++i)
    {
        fTool.updateBaseMesh(fTool.getChunkInfoIndex(aResult.assetToFractureChunkIdMap[i]), aResult.geometry + aResult.geometryOffset[i]);
    }
}

void NvBlastExtAuthoringBuildCollisionMeshes(Nv::Blast::AuthoringResult& ares, Nv::Blast::ConvexMeshBuilder& collisionBuilder,
    const Nv::Blast::ConvexDecompositionParams& collisionParam, uint32_t chunksToProcessCount, uint32_t* chunksToProcess)
{
    buildPhysicsChunks(collisionBuilder, ares, collisionParam, chunksToProcessCount, chunksToProcess);
}

PatternGenerator* NvBlastExtAuthoringCreatePatternGenerator()
{
    return NVBLAST_NEW(PatternGeneratorImpl);
}

SpatialGrid* NvBlastExtAuthoringCreateSpatialGrid(uint32_t resolution, const Mesh* m)
{
    Grid* g = NVBLAST_NEW(Grid)(resolution);
    g->setMesh(m);
    return g;
}

SpatialAccelerator* NvBlastExtAuthoringCreateGridAccelerator(SpatialGrid* parentGrid)
{
    return NVBLAST_NEW(GridAccelerator)((Grid*)parentGrid);
}

SpatialAccelerator* NvBlastExtAuthoringCreateSweepingAccelerator(const Mesh* m)
{
    return NVBLAST_NEW(SweepingAccelerator)(m);
}

SpatialAccelerator* NvBlastExtAuthoringCreateBBoxBasedAccelerator(uint32_t resolution, const Mesh* m)
{
    return NVBLAST_NEW(BBoxBasedAccelerator)(m, resolution);
}

BooleanTool* NvBlastExtAuthoringCreateBooleanTool()
{
    return new BooleanToolImpl;
}
