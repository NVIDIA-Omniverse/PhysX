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
// Copyright (c) 2016-2023 NVIDIA Corporation. All rights reserved.

#include <NvBlastGlobals.h>
#include "NvBlastExtAuthoringCollisionBuilderImpl.h"
#include <NvBlastExtApexSharedParts.h>
#include <NvBlastExtAuthoringInternalCommon.h>

#include <NvBlastExtAuthoringBooleanToolImpl.h>
#include <NvBlastExtAuthoringMeshImpl.h>
#include <NvBlastExtAuthoringMeshUtils.h>
#include <NvBlastNvSharedHelpers.h>
#include <VHACD.h>
#include <vector>

using namespace nvidia;

namespace Nv
{
namespace Blast
{

#define SAFE_ARRAY_NEW(T, x) ((x) > 0) ? reinterpret_cast<T*>(NVBLAST_ALLOC(sizeof(T) * (x))) : nullptr;
#define SAFE_ARRAY_DELETE(x) if (x != nullptr) {NVBLAST_FREE(x); x = nullptr;}

void trimCollisionGeometry(ConvexMeshBuilder& cmb, uint32_t chunksCount, CollisionHull** in, const uint32_t* chunkDepth)
{
    std::vector<std::vector<NvPlane> > chunkMidplanes(chunksCount);
    std::vector<NvVec3> centers(chunksCount);
    std::vector<NvBounds3> hullsBounds(chunksCount);
    for (uint32_t i = 0; i < chunksCount; ++i)
    {
        hullsBounds[i].setEmpty();
        centers[i] = NvVec3(0, 0, 0);
        for (uint32_t p = 0; p < in[i]->pointsCount; ++p)
        {
            centers[i] += toNvShared(in[i]->points[p]);
            hullsBounds[i].include(toNvShared(in[i]->points[p]));
        }
        centers[i] = hullsBounds[i].getCenter();
    }

    Separation params;
    for (uint32_t hull = 0; hull < chunksCount; ++hull)
    {
        for (uint32_t hull2 = hull + 1; hull2 < chunksCount; ++hull2)
        {
            if (chunkDepth[hull] != chunkDepth[hull2])
            {
                continue;
            }
            if (importerHullsInProximityApexFree(in[hull]->pointsCount, toNvShared(in[hull]->points), hullsBounds[hull],
                                                 NvTransform(NvIdentity), NvVec3(1, 1, 1), in[hull2]->pointsCount,
                                                 toNvShared(in[hull2]->points), hullsBounds[hull2], NvTransform(NvIdentity),
                                                 NvVec3(1, 1, 1), 0.0, &params) == false)
            {
                continue;
            }
            NvVec3 c1 = centers[hull];
            NvVec3 c2 = centers[hull2];
            float d = FLT_MAX;
            NvVec3 n1;
            NvVec3 n2;
            for (uint32_t p = 0; p < in[hull]->pointsCount; ++p)
            {
                float ld = (toNvShared(in[hull]->points[p]) - c2).magnitude();
                if (ld < d)
                {
                    n1 = toNvShared(in[hull]->points[p]);
                    d = ld;
                }
            }
            d = FLT_MAX;
            for (uint32_t p = 0; p < in[hull2]->pointsCount; ++p)
            {
                float ld = (toNvShared(in[hull2]->points[p]) - c1).magnitude();
                if (ld < d)
                {
                    n2 = toNvShared(in[hull2]->points[p]);
                    d = ld;
                }
            }

            NvVec3 dir = c2 - c1;

            NvPlane pl = NvPlane((n1 + n2) * 0.5, dir.getNormalized());
            chunkMidplanes[hull].push_back(pl);
            NvPlane pl2 = NvPlane((n1 + n2) * 0.5, -dir.getNormalized());
            chunkMidplanes[hull2].push_back(pl2);       
        }
    }
    std::vector<NvVec3> hPoints;
    for (uint32_t i = 0; i < chunksCount; ++i)
    {
        std::vector<Facet> facets;
        std::vector<Vertex> vertices;
        std::vector<Edge> edges;
        for (uint32_t fc = 0; fc < in[i]->polygonDataCount; ++fc)
        {
            Facet nFc;
            nFc.firstEdgeNumber = edges.size();
            auto& pd = in[i]->polygonData[fc];
            uint32_t n = pd.vertexCount;
            for (uint32_t ed = 0; ed < n; ++ed)
            {
                uint32_t vr1 = in[i]->indices[(ed) + pd.indexBase];
                uint32_t vr2 = in[i]->indices[(ed + 1) % n + pd.indexBase];
                edges.push_back({vr1, vr2});
            }
            nFc.edgesCount = n;
            facets.push_back(nFc);
        }
        vertices.resize(in[i]->pointsCount);
        for (uint32_t vr = 0; vr < in[i]->pointsCount; ++vr)
        {
            vertices[vr].p = in[i]->points[vr];
        }
        Mesh* hullMesh = new MeshImpl(vertices.data(), edges.data(), facets.data(), vertices.size(), edges.size(), facets.size());
        BooleanEvaluator evl;
        //I think the material ID is unused for collision meshes so harcoding MATERIAL_INTERIOR is ok
        Mesh* cuttingMesh = getCuttingBox(NvVec3(0, 0, 0), NvVec3(0, 0, 1), 40, 0, kMaterialInteriorId);
        for (uint32_t p = 0; p < chunkMidplanes[i].size(); ++p)
        {
            NvPlane& pl = chunkMidplanes[i][p];
            setCuttingBox(pl.pointInPlane(), pl.n.getNormalized(), cuttingMesh, 60, 0);
            evl.performFastCutting(hullMesh, cuttingMesh, BooleanConfigurations::BOOLEAN_DIFFERENCE());
            Mesh* result = evl.createNewMesh();
            if (result == nullptr)
            {
                break;
            }
            delete hullMesh;
            hullMesh = result;
        }
        delete cuttingMesh;
        if (hullMesh == nullptr)
        {
            continue;
        }
        hPoints.clear();
        hPoints.resize(hullMesh->getVerticesCount());
        for (uint32_t v = 0; v < hullMesh->getVerticesCount(); ++v)
        {
            hPoints[v] = toNvShared(hullMesh->getVertices()[v].p);
        }
        delete hullMesh;
        if (in[i] != nullptr)
        {
            delete in[i];
        }
        in[i] = cmb.buildCollisionGeometry(hPoints.size(), fromNvShared(hPoints.data()));
    }
}

int32_t buildMeshConvexDecomposition(ConvexMeshBuilder& cmb, const Triangle* mesh, uint32_t triangleCount,
                                     const ConvexDecompositionParams& iparams, CollisionHull**& convexes)
{
    std::vector<float> coords(triangleCount * 9);
    std::vector<uint32_t> indices(triangleCount * 3);

    uint32_t indx = 0;
    uint32_t indxCoord = 0;

    NvBounds3 chunkBound = NvBounds3::empty();
    for (uint32_t i = 0; i < triangleCount; ++i)
    {
        for (auto& t : { mesh[i].a.p , mesh[i].b.p , mesh[i].c.p })
        {

            chunkBound.include(toNvShared(t));
            coords[indxCoord] = t.x;
            coords[indxCoord + 1] = t.y;
            coords[indxCoord + 2] = t.z;
            indxCoord += 3;
        }
        indices[indx] = indx;
        indices[indx + 1] = indx + 1;
        indices[indx + 2] = indx + 2;
        indx += 3;
    }

    NvVec3 rsc = chunkBound.getDimensions();

    for (uint32_t i = 0; i < coords.size(); i += 3)
    {
        coords[i] = (coords[i] - chunkBound.minimum.x) / rsc.x;
        coords[i + 1] = (coords[i + 1] - chunkBound.minimum.y) / rsc.y;
        coords[i + 2] = (coords[i + 2] - chunkBound.minimum.z) / rsc.z;
    }
    
    VHACD::IVHACD* decomposer = VHACD::CreateVHACD();

    VHACD::IVHACD::Parameters vhacdParam;
    vhacdParam.m_maxConvexHulls = iparams.maximumNumberOfHulls;
    vhacdParam.m_resolution = iparams.voxelGridResolution;
    vhacdParam.m_concavity = iparams.concavity;
    vhacdParam.m_oclAcceleration = false;
    //TODO vhacdParam.m_callback
    vhacdParam.m_minVolumePerCH = 0.003f; // 1.f / (3 * vhacdParam.m_resolution ^ (1 / 3));

    decomposer->Compute(coords.data(), triangleCount * 3, indices.data(), triangleCount, vhacdParam);

    const uint32_t nConvexHulls = decomposer->GetNConvexHulls();
    convexes = SAFE_ARRAY_NEW(CollisionHull*, nConvexHulls);

    for (uint32_t i = 0; i < nConvexHulls; ++i)
    {
        VHACD::IVHACD::ConvexHull hl;
        decomposer->GetConvexHull(i, hl);
        std::vector<NvVec3> vertices;
        for (uint32_t v = 0; v < hl.m_nPoints; ++v)
        {
            vertices.push_back(NvVec3(hl.m_points[v * 3], hl.m_points[v * 3 + 1], hl.m_points[v * 3 + 2]));
            vertices.back().x = vertices.back().x * rsc.x + chunkBound.minimum.x;
            vertices.back().y = vertices.back().y * rsc.y + chunkBound.minimum.y;
            vertices.back().z = vertices.back().z * rsc.z + chunkBound.minimum.z;

        }
        convexes[i] = cmb.buildCollisionGeometry(vertices.size(), fromNvShared(vertices.data()));
    }
    //VHACD::~VHACD called from release does nothign and does not call Clean()
    decomposer->Clean();
    decomposer->Release();

    return nConvexHulls;
}



} // namespace Blast
} // namespace Nv
