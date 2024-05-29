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


#include "NvBlastExtDamageShaders.h"
#include "NvBlastExtDamageAcceleratorInternal.h"
#include "NvBlastIndexFns.h"
#include "NvBlastMath.h"
#include "NvBlastGeometry.h"
#include "NvBlastAssert.h"
#include "NvBlastFixedQueue.h"
#include "NvBlastFixedBitmap.h"
#include "NvBlast.h"
#include <cmath> // for abs() on linux
#include <new>


using namespace Nv::Blast;
using namespace Nv::Blast::VecMath;
using namespace nvidia;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                  Profiles
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

typedef float(*ProfileFunction)(float, float, float, float);

float falloffProfile(float min, float max, float x, float f = 1.0f)
{
    if (x > max) return 0.0f;
    if (x < min) return f;

    float y = 1.0f - (x - min) / (max - min);
    return y * f;
}

float cutterProfile(float min, float max, float x, float f = 1.0f)
{
    if (x > max || x < min) return 0.0f;

    return f;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                  Damage Functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

typedef float(*DamageFunction)(const float pos[3], const void* damageDescBuffer);

template <ProfileFunction profileFn, typename DescT = NvBlastExtRadialDamageDesc>
float pointDistanceDamage(const float pos[3], const void* damageDescBuffer)
{
    const DescT& desc = *static_cast<const DescT*>(damageDescBuffer);

    float relativePosition[3];
    sub(desc.position, pos, relativePosition);
    const float distance = sqrtf(dot(relativePosition, relativePosition));
    const float damage = profileFn(desc.minRadius, desc.maxRadius, distance, desc.damage);
    return damage;
}


// Distance from point 'p' to line segment '(a, b)'
float distanceToSegment(const float p[3], const float a[3], const float b[3])
{
    float v[3];
    sub(b, a, v);

    float w[3];
    sub(p, a, w);

    const float c1 = dot(v, w);
    if (c1 <= 0)
        return length(w);

    const float c2 = dot(v, v);
    if (c2 < c1)
        return dist(p, b);

    const float t = c1 / c2;
    mul(v, t);
    return dist(v, w);
}

template <ProfileFunction profileFn>
float capsuleDistanceDamage(const float pos[3], const void* damageDesc)
{
    const NvBlastExtCapsuleRadialDamageDesc& desc = *static_cast<const NvBlastExtCapsuleRadialDamageDesc*>(damageDesc);

    const float distance = distanceToSegment(pos, desc.position0, desc.position1);
    const float damage = profileFn(desc.minRadius, desc.maxRadius, distance, desc.damage);
    return damage;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                  AABB Functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

typedef NvBounds3(*BoundFunction)(const void* damageDesc);

NvBounds3 sphereBounds(const void* damageDesc)
{
    const NvBlastExtRadialDamageDesc& desc = *static_cast<const NvBlastExtRadialDamageDesc*>(damageDesc);
    const nvidia::NvVec3& p = (reinterpret_cast<const nvidia::NvVec3&>(desc.position));
    return nvidia::NvBounds3::centerExtents(p, nvidia::NvVec3(desc.maxRadius, desc.maxRadius, desc.maxRadius));
}

NvBounds3 capsuleBounds(const void* damageDesc)
{
    const NvBlastExtCapsuleRadialDamageDesc& desc = *static_cast<const NvBlastExtCapsuleRadialDamageDesc*>(damageDesc);
    const nvidia::NvVec3& p0 = (reinterpret_cast<const nvidia::NvVec3&>(desc.position0));
    const nvidia::NvVec3& p1 = (reinterpret_cast<const nvidia::NvVec3&>(desc.position1));
    NvBounds3 b = NvBounds3::empty();
    b.include(p0);
    b.include(p1);
    b.fattenFast(desc.maxRadius);
    return b;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                              Radial Graph Shader Template
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template <DamageFunction damageFn, BoundFunction boundsFn>
void RadialProfileGraphShader(NvBlastFractureBuffers* commandBuffers, const NvBlastGraphShaderActor* actor, const void* params)
{
    const uint32_t* graphNodeIndexLinks = actor->graphNodeIndexLinks;
    const uint32_t firstGraphNodeIndex = actor->firstGraphNodeIndex;
    const uint32_t* adjacencyPartition = actor->adjacencyPartition;
    const uint32_t* adjacentNodeIndices = actor->adjacentNodeIndices;
    const uint32_t* adjacentBondIndices = actor->adjacentBondIndices;
    const NvBlastBond* assetBonds = actor->assetBonds;
    const float* familyBondHealths = actor->familyBondHealths;
    const NvBlastExtProgramParams* programParams = static_cast<const NvBlastExtProgramParams*>(params);

    uint32_t outCount = 0;

    auto processBondFn = [&](uint32_t bondIndex, uint32_t node0, uint32_t node1)
    {
        // skip bonds that are already broken or were visited already
        // TODO: investigate why testing against health > -1.0f seems slower
        // could reuse the island edge bitmap instead
        if (canTakeDamage(familyBondHealths[bondIndex]))
        {
            const NvBlastBond& bond = assetBonds[bondIndex];

            const float totalBondDamage = damageFn(bond.centroid, programParams->damageDesc);
            if (totalBondDamage > 0.0f)
            {
                NvBlastBondFractureData& outCommand = commandBuffers->bondFractures[outCount++];
                outCommand.nodeIndex0 = node0;
                outCommand.nodeIndex1 = node1;
                outCommand.health = totalBondDamage;
            }
        }
    };

    const ExtDamageAcceleratorInternal* damageAccelerator = programParams->accelerator ? static_cast<const ExtDamageAcceleratorInternal*>(programParams->accelerator) : nullptr;
    const uint32_t ACTOR_MINIMUM_NODE_COUNT_TO_ACCELERATE = actor->assetNodeCount / 3;
    if (damageAccelerator && actor->graphNodeCount > ACTOR_MINIMUM_NODE_COUNT_TO_ACCELERATE)
    {
        nvidia::NvBounds3 bounds = boundsFn(programParams->damageDesc);

        const uint32_t CALLBACK_BUFFER_SIZE = 1000;

        class AcceleratorCallback : public ExtDamageAcceleratorInternal::ResultCallback
        {
        public:
            AcceleratorCallback(NvBlastFractureBuffers* commandBuffers, uint32_t& outCount, const NvBlastGraphShaderActor* actor, const NvBlastExtProgramParams* programParams) :
                ExtDamageAcceleratorInternal::ResultCallback(m_buffer, CALLBACK_BUFFER_SIZE),
                m_actor(actor),
                m_commandBuffers(commandBuffers),
                m_outCount(outCount),
                m_programParams(programParams)
            {
            }

            virtual void processResults(const ExtDamageAcceleratorInternal::QueryBondData* bondBuffer, uint32_t count) override
            {
                for (uint32_t i = 0; i < count; i++)
                {
                    const ExtDamageAcceleratorInternal::QueryBondData& bondData = bondBuffer[i];
                    if (m_actor->nodeActorIndices[bondData.node0] == m_actor->actorIndex)
                    {
                        if (canTakeDamage(m_actor->familyBondHealths[bondData.bond]))
                        {
                            const NvBlastBond& bond = m_actor->assetBonds[bondData.bond];

                            const float totalBondDamage = damageFn(bond.centroid, m_programParams->damageDesc);
                            if (totalBondDamage > 0.0f)
                            {
                                NvBlastBondFractureData& outCommand = m_commandBuffers->bondFractures[m_outCount++];
                                outCommand.nodeIndex0 = bondData.node0;
                                outCommand.nodeIndex1 = bondData.node1;
                                outCommand.health = totalBondDamage;
                            }
                        }
                    }
                }
            }

        private:
            const NvBlastGraphShaderActor* m_actor;
            NvBlastFractureBuffers* m_commandBuffers;
            uint32_t& m_outCount;
            const NvBlastExtProgramParams* m_programParams;

            ExtDamageAcceleratorInternal::QueryBondData m_buffer[CALLBACK_BUFFER_SIZE];
        };

        AcceleratorCallback cb(commandBuffers, outCount, actor, programParams);

        damageAccelerator->findBondCentroidsInBounds(bounds, cb);
    }
    else
    {
        uint32_t currentNodeIndex = firstGraphNodeIndex;
        while (!Nv::Blast::isInvalidIndex(currentNodeIndex))
        {
            for (uint32_t adj = adjacencyPartition[currentNodeIndex]; adj < adjacencyPartition[currentNodeIndex + 1]; adj++)
            {
                uint32_t adjacentNodeIndex = adjacentNodeIndices[adj];
                if (currentNodeIndex < adjacentNodeIndex)
                {
                    uint32_t bondIndex = adjacentBondIndices[adj];
                    processBondFn(bondIndex, currentNodeIndex, adjacentNodeIndex);
                }
            }
            currentNodeIndex = graphNodeIndexLinks[currentNodeIndex];
        }
    }

    commandBuffers->bondFractureCount = outCount;
    commandBuffers->chunkFractureCount = 0;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                          Radial Single Shader Template
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template <DamageFunction damageFn>
void RadialProfileSubgraphShader(NvBlastFractureBuffers* commandBuffers, const NvBlastSubgraphShaderActor* actor, const void* params)
{
    uint32_t chunkFractureCount = 0;
    uint32_t chunkFractureCountMax = commandBuffers->chunkFractureCount;
    const uint32_t chunkIndex = actor->chunkIndex;
    const NvBlastChunk* assetChunks = actor->assetChunks;
    const NvBlastChunk& chunk = assetChunks[chunkIndex];
    const NvBlastExtProgramParams* programParams = static_cast<const NvBlastExtProgramParams*>(params);

    const float totalDamage = damageFn(chunk.centroid, programParams->damageDesc);
    if (totalDamage > 0.0f && chunkFractureCount < chunkFractureCountMax)
    {
        NvBlastChunkFractureData& frac = commandBuffers->chunkFractures[chunkFractureCount++];
        frac.chunkIndex = chunkIndex;
        frac.health = totalDamage;
    }

    commandBuffers->bondFractureCount = 0;
    commandBuffers->chunkFractureCount = chunkFractureCount;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                              Radial Shaders Instantiation
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void NvBlastExtFalloffGraphShader(NvBlastFractureBuffers* commandBuffers, const NvBlastGraphShaderActor* actor, const void* params)
{
    RadialProfileGraphShader<pointDistanceDamage<falloffProfile>, sphereBounds>(commandBuffers, actor, params);
}

void NvBlastExtFalloffSubgraphShader(NvBlastFractureBuffers* commandBuffers, const NvBlastSubgraphShaderActor* actor, const void* params)
{
    RadialProfileSubgraphShader<pointDistanceDamage<falloffProfile>>(commandBuffers, actor, params);
}

void NvBlastExtCutterGraphShader(NvBlastFractureBuffers* commandBuffers, const NvBlastGraphShaderActor* actor, const void* params)
{
    RadialProfileGraphShader<pointDistanceDamage<cutterProfile>, sphereBounds>(commandBuffers, actor, params);
}

void NvBlastExtCutterSubgraphShader(NvBlastFractureBuffers* commandBuffers, const NvBlastSubgraphShaderActor* actor, const void* params)
{
    RadialProfileSubgraphShader<pointDistanceDamage<cutterProfile>>(commandBuffers, actor, params);
}

void NvBlastExtCapsuleFalloffGraphShader(NvBlastFractureBuffers* commandBuffers, const NvBlastGraphShaderActor* actor, const void* params)
{
    RadialProfileGraphShader<capsuleDistanceDamage<falloffProfile>, capsuleBounds>(commandBuffers, actor, params);
}

void NvBlastExtCapsuleFalloffSubgraphShader(NvBlastFractureBuffers* commandBuffers, const NvBlastSubgraphShaderActor* actor, const void* params)
{
    RadialProfileSubgraphShader<capsuleDistanceDamage<falloffProfile>>(commandBuffers, actor, params);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                  Shear Shader
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void NvBlastExtShearGraphShader(NvBlastFractureBuffers* commandBuffers, const NvBlastGraphShaderActor* actor, const void* params)
{
    uint32_t chunkFractureCount = 0;
    uint32_t chunkFractureCountMax = commandBuffers->chunkFractureCount;
    uint32_t bondFractureCount = 0;
    uint32_t bondFractureCountMax = commandBuffers->bondFractureCount;
    const NvBlastExtProgramParams* programParams = static_cast<const NvBlastExtProgramParams*>(params);
    const NvBlastExtShearDamageDesc& desc = *static_cast<const NvBlastExtShearDamageDesc*>(programParams->damageDesc);
    const uint32_t* graphNodeIndexLinks = actor->graphNodeIndexLinks;
    const uint32_t firstGraphNodeIndex = actor->firstGraphNodeIndex;
    const uint32_t* chunkIndices = actor->chunkIndices;
    const uint32_t* adjacencyPartition = actor->adjacencyPartition;
    const uint32_t* adjacentNodeIndices = actor->adjacentNodeIndices;
    const uint32_t* adjacentBondIndices = actor->adjacentBondIndices;
    const NvBlastBond* assetBonds = actor->assetBonds;
    const NvBlastChunk* assetChunks = actor->assetChunks;
    const float* familyBondHealths = actor->familyBondHealths;
    const float* supportChunkHealths = actor->supportChunkHealths;

    uint32_t closestNode = findClosestNode(desc.position
        , firstGraphNodeIndex, graphNodeIndexLinks
        , adjacencyPartition, adjacentNodeIndices, adjacentBondIndices
        , assetBonds, familyBondHealths
        , assetChunks, supportChunkHealths, chunkIndices);

    if (!isInvalidIndex(chunkIndices[closestNode]))
    {
        uint32_t nodeIndex = closestNode;
        float maxDist = 0.0f;
        uint32_t nextNode = invalidIndex<uint32_t>();

        if (chunkFractureCount < chunkFractureCountMax)
        {
            const uint32_t chunkIndex = chunkIndices[nodeIndex];
            const NvBlastChunk& chunk = assetChunks[chunkIndex];
            NvBlastChunkFractureData& frac = commandBuffers->chunkFractures[chunkFractureCount++];
            frac.chunkIndex = chunkIndex;
            frac.health = pointDistanceDamage<falloffProfile, NvBlastExtShearDamageDesc>(chunk.centroid, programParams->damageDesc);
        }

        do {
            const uint32_t startIndex = adjacencyPartition[nodeIndex];
            const uint32_t stopIndex = adjacencyPartition[nodeIndex + 1];


            for (uint32_t adjacentNodeIndex = startIndex; adjacentNodeIndex < stopIndex; adjacentNodeIndex++)
            {
                const uint32_t neighbourIndex = adjacentNodeIndices[adjacentNodeIndex];
                const uint32_t bondIndex = adjacentBondIndices[adjacentNodeIndex];
                const NvBlastBond& bond = assetBonds[bondIndex];

                if (!canTakeDamage(familyBondHealths[bondIndex]))
                    continue;

                float shear = 1 * std::abs(1 - std::abs(VecMath::dot(desc.normal, bond.normal)));

                float d[3]; VecMath::sub(bond.centroid, desc.position, d);
                float ahead = VecMath::dot(d, desc.normal);
                if (ahead > maxDist)
                {
                    maxDist = ahead;
                    nextNode = neighbourIndex;
                }

                const float damage = pointDistanceDamage<falloffProfile, NvBlastExtShearDamageDesc>(bond.centroid, programParams->damageDesc);
                if (damage > 0.0f && bondFractureCount < bondFractureCountMax)
                {
                    NvBlastBondFractureData& frac = commandBuffers->bondFractures[bondFractureCount++];
                    frac.userdata = bond.userData;
                    frac.nodeIndex0 = nodeIndex;
                    frac.nodeIndex1 = neighbourIndex;
                    frac.health = shear * damage;
                }
            }

            if (nodeIndex == nextNode)
                break;

            nodeIndex = nextNode;
        } while (!isInvalidIndex(nextNode));
    }

    commandBuffers->bondFractureCount = bondFractureCount;
    commandBuffers->chunkFractureCount = chunkFractureCount;
}

void NvBlastExtShearSubgraphShader(NvBlastFractureBuffers* commandBuffers, const NvBlastSubgraphShaderActor* actor, const void* params)
{
    RadialProfileSubgraphShader<pointDistanceDamage<falloffProfile, NvBlastExtShearDamageDesc>>(commandBuffers, actor, params);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                              Triangle Intersection Damage
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define SMALL_NUMBER    (1.e-4f)

bool intersectSegmentTriangle(const NvVec3& p, const NvVec3& q, const NvVec3& a, const NvVec3& b, const NvVec3& c, const NvPlane& trianglePlane)
{
    const NvVec3 N = trianglePlane.n;
    const float D = trianglePlane.d;

    NvVec3 intersectPoint;
    float t = (-D - (p.dot(N))) / ((q - p).dot(N));
    // If the parameter value is not between 0 and 1, there is no intersection
    if (t > -SMALL_NUMBER && t < 1.f + SMALL_NUMBER)
    {
        intersectPoint = p + t * (q - p);
    }
    else
    {
        return false;
    }

    // Compute the normal of the triangle
    const NvVec3 TriNorm = (b - a).cross(c - a);

    // Compute twice area of triangle ABC
    const float AreaABCInv = 1.0f / (N.dot(TriNorm));

    // Compute v contribution
    const float AreaPBC = N.dot((b - intersectPoint).cross(c - intersectPoint));
    const float v = AreaPBC * AreaABCInv;
    if (v <= 0.f)
        return false;

    // Compute w contribution
    const float AreaPCA = N.dot((c - intersectPoint).cross(a - intersectPoint));
    const float w = AreaPCA * AreaABCInv;
    if (w <= 0.f)
        return false;

    const float u = 1.0f - v - w;
    return u > 0.f;
}

void NvBlastExtTriangleIntersectionGraphShader(NvBlastFractureBuffers* commandBuffers, const NvBlastGraphShaderActor* actor, const void* params)
{
    const uint32_t* graphNodeIndexLinks = actor->graphNodeIndexLinks;
    const uint32_t firstGraphNodeIndex = actor->firstGraphNodeIndex;
    const uint32_t* adjacencyPartition = actor->adjacencyPartition;
    const uint32_t* adjacentNodeIndices = actor->adjacentNodeIndices;
    const uint32_t* adjacentBondIndices = actor->adjacentBondIndices;
    const NvBlastBond* assetBonds = actor->assetBonds;
    const NvBlastChunk* assetChunks = actor->assetChunks;
    const uint32_t* chunkIndices = actor->chunkIndices;
    const float* familyBondHealths = actor->familyBondHealths;
    const NvBlastExtProgramParams* programParams = static_cast<const NvBlastExtProgramParams*>(params);
    const NvBlastExtTriangleIntersectionDamageDesc& desc = *static_cast<const NvBlastExtTriangleIntersectionDamageDesc*>(programParams->damageDesc);
    const nvidia::NvVec3& t0 = (reinterpret_cast<const nvidia::NvVec3&>(desc.position0));
    const nvidia::NvVec3& t1 = (reinterpret_cast<const nvidia::NvVec3&>(desc.position1));
    const nvidia::NvVec3& t2 = (reinterpret_cast<const nvidia::NvVec3&>(desc.position2));
    const NvPlane trianglePlane(t0, t1, t2);

    uint32_t outCount = 0;

    const ExtDamageAcceleratorInternal* damageAccelerator = programParams->accelerator ? static_cast<const ExtDamageAcceleratorInternal*>(programParams->accelerator) : nullptr;
    const uint32_t ACTOR_MINIMUM_NODE_COUNT_TO_ACCELERATE = actor->assetNodeCount / 3;
    if (damageAccelerator && actor->graphNodeCount > ACTOR_MINIMUM_NODE_COUNT_TO_ACCELERATE)
    {
        const uint32_t CALLBACK_BUFFER_SIZE = 1000;

        class AcceleratorCallback : public ExtDamageAcceleratorInternal::ResultCallback
        {
        public:
            AcceleratorCallback(NvBlastFractureBuffers* commandBuffers, uint32_t& outCount, const NvBlastGraphShaderActor* actor, const NvBlastExtTriangleIntersectionDamageDesc& desc) :
                ExtDamageAcceleratorInternal::ResultCallback(m_buffer, CALLBACK_BUFFER_SIZE),
                m_actor(actor),
                m_commandBuffers(commandBuffers),
                m_outCount(outCount),
                m_desc(desc)
            {
            }

            virtual void processResults(const ExtDamageAcceleratorInternal::QueryBondData* bondBuffer, uint32_t count) override
            {
                const nvidia::NvVec3& t0 = (reinterpret_cast<const nvidia::NvVec3&>(m_desc.position0));
                const nvidia::NvVec3& t1 = (reinterpret_cast<const nvidia::NvVec3&>(m_desc.position1));
                const nvidia::NvVec3& t2 = (reinterpret_cast<const nvidia::NvVec3&>(m_desc.position2));
                const NvPlane trianglePlane(t0, t1, t2);

                for (uint32_t i = 0; i < count; i++)
                {
                    const ExtDamageAcceleratorInternal::QueryBondData& bondData = bondBuffer[i];
                    if (m_actor->nodeActorIndices[bondData.node0] == m_actor->actorIndex)
                    {
                        if (canTakeDamage(m_actor->familyBondHealths[bondData.bond]))
                        {
                            const NvBlastBond& bond = m_actor->assetBonds[bondData.bond];
                            const uint32_t chunkIndex0 = m_actor->chunkIndices[bondData.node0];
                            const uint32_t chunkIndex1 = m_actor->chunkIndices[bondData.node1];
                            const nvidia::NvVec3& c0 = (reinterpret_cast<const nvidia::NvVec3&>(m_actor->assetChunks[chunkIndex0].centroid));
                            const NvVec3& normal = (reinterpret_cast<const NvVec3&>(bond.normal));
                            const NvVec3& bondCentroid = (reinterpret_cast<const NvVec3&>(bond.centroid));
                            const nvidia::NvVec3& c1 = isInvalidIndex(chunkIndex1) ? (c0 + normal * (bondCentroid - c0).dot(normal)) :
                                (reinterpret_cast<const nvidia::NvVec3&>(m_actor->assetChunks[chunkIndex1].centroid));

                            if(intersectSegmentTriangle(c0, c1, t0, t1, t2, trianglePlane))
                            {
                                NvBlastBondFractureData& outCommand = m_commandBuffers->bondFractures[m_outCount++];
                                outCommand.nodeIndex0 = bondData.node0;
                                outCommand.nodeIndex1 = bondData.node1;
                                outCommand.health = m_desc.damage;
                            }
                        }
                    }
                }
            }

        private:
            const NvBlastGraphShaderActor* m_actor;
            NvBlastFractureBuffers* m_commandBuffers;
            uint32_t& m_outCount;
            const NvBlastExtTriangleIntersectionDamageDesc& m_desc;

            ExtDamageAcceleratorInternal::QueryBondData m_buffer[CALLBACK_BUFFER_SIZE];
        };

        AcceleratorCallback cb(commandBuffers, outCount, actor, desc);

        damageAccelerator->findBondSegmentsPlaneIntersected(trianglePlane, cb);
    }
    else 
    {
        uint32_t currentNodeIndex = firstGraphNodeIndex;
        while (!Nv::Blast::isInvalidIndex(currentNodeIndex))
        {
            for (uint32_t adj = adjacencyPartition[currentNodeIndex]; adj < adjacencyPartition[currentNodeIndex + 1]; adj++)
            {
                uint32_t adjacentNodeIndex = adjacentNodeIndices[adj];
                if (currentNodeIndex < adjacentNodeIndex)
                {
                    uint32_t bondIndex = adjacentBondIndices[adj];
                    // skip bonds that are already broken or were visited already
                    // TODO: investigate why testing against health > -1.0f seems slower
                    // could reuse the island edge bitmap instead
                    if (canTakeDamage(familyBondHealths[bondIndex]))
                    {
                        const NvBlastBond& bond = assetBonds[bondIndex];
                        const uint32_t chunkIndex0 = chunkIndices[currentNodeIndex];
                        const uint32_t chunkIndex1 = chunkIndices[adjacentNodeIndex];
                        const nvidia::NvVec3& c0 = (reinterpret_cast<const nvidia::NvVec3&>(assetChunks[chunkIndex0].centroid));
                        const NvVec3& normal = (reinterpret_cast<const NvVec3&>(bond.normal));
                        const NvVec3& bondCentroid = (reinterpret_cast<const NvVec3&>(bond.centroid));
                        const nvidia::NvVec3& c1 = isInvalidIndex(chunkIndex1) ? (c0 + normal * (bondCentroid - c0).dot(normal)) : 
                            (reinterpret_cast<const nvidia::NvVec3&>(assetChunks[chunkIndex1].centroid));

                        if (intersectSegmentTriangle(c0, c1, t0, t1, t2, trianglePlane))
                        {
                            NvBlastBondFractureData& outCommand = commandBuffers->bondFractures[outCount++];
                            outCommand.nodeIndex0 = currentNodeIndex;
                            outCommand.nodeIndex1 = adjacentNodeIndex;
                            outCommand.health = desc.damage;
                        }
                    }
                }
            }
            currentNodeIndex = graphNodeIndexLinks[currentNodeIndex];
        }
    }

    commandBuffers->bondFractureCount = outCount;
    commandBuffers->chunkFractureCount = 0;
}

void NvBlastExtTriangleIntersectionSubgraphShader(NvBlastFractureBuffers* commandBuffers, const NvBlastSubgraphShaderActor* actor, const void* params)
{
    uint32_t chunkFractureCount = 0;
    uint32_t chunkFractureCountMax = commandBuffers->chunkFractureCount;
    const uint32_t chunkIndex = actor->chunkIndex;
    const NvBlastChunk* assetChunks = actor->assetChunks;
    const NvBlastChunk& chunk = assetChunks[chunkIndex];
    const NvBlastExtProgramParams* programParams = static_cast<const NvBlastExtProgramParams*>(params);
    const NvBlastExtTriangleIntersectionDamageDesc& desc = *static_cast<const NvBlastExtTriangleIntersectionDamageDesc*>(programParams->damageDesc);
    const nvidia::NvVec3& t0 = (reinterpret_cast<const nvidia::NvVec3&>(desc.position0));
    const nvidia::NvVec3& t1 = (reinterpret_cast<const nvidia::NvVec3&>(desc.position1));
    const nvidia::NvVec3& t2 = (reinterpret_cast<const nvidia::NvVec3&>(desc.position2));
    const NvPlane trianglePlane(t0, t1, t2);

    for (uint32_t subChunkIndex = chunk.firstChildIndex; subChunkIndex < chunk.childIndexStop; subChunkIndex++)
    {
        const nvidia::NvVec3& c0 = (reinterpret_cast<const nvidia::NvVec3&>(assetChunks[subChunkIndex].centroid));
        const nvidia::NvVec3& c1 = (reinterpret_cast<const nvidia::NvVec3&>(assetChunks[subChunkIndex + 1].centroid));
        if (chunkFractureCount < chunkFractureCountMax && intersectSegmentTriangle(c0, c1, t0, t1, t2, trianglePlane))
        {
            NvBlastChunkFractureData& frac = commandBuffers->chunkFractures[chunkFractureCount++];
            frac.chunkIndex = chunkIndex;
            frac.health = desc.damage;
            break;
        }
    }

    commandBuffers->bondFractureCount = 0;
    commandBuffers->chunkFractureCount = chunkFractureCount;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                               Impact Spread Shader
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void NvBlastExtImpactSpreadGraphShader(NvBlastFractureBuffers* commandBuffers, const NvBlastGraphShaderActor* actor, const void* params)
{
    uint32_t bondFractureCount = 0;
    uint32_t bondFractureCountMax = commandBuffers->bondFractureCount;
    const NvBlastExtProgramParams* programParams = static_cast<const NvBlastExtProgramParams*>(params);
    const NvBlastExtImpactSpreadDamageDesc& desc = *static_cast<const NvBlastExtImpactSpreadDamageDesc*>(programParams->damageDesc);
    const uint32_t* graphNodeIndexLinks = actor->graphNodeIndexLinks;
    const uint32_t firstGraphNodeIndex = actor->firstGraphNodeIndex;
    const uint32_t* chunkIndices = actor->chunkIndices;
    const uint32_t* adjacencyPartition = actor->adjacencyPartition;
    const uint32_t* adjacentNodeIndices = actor->adjacentNodeIndices;
    const uint32_t* adjacentBondIndices = actor->adjacentBondIndices;
    const NvBlastBond* assetBonds = actor->assetBonds;
    const NvBlastChunk* assetChunks = actor->assetChunks;
    const float* familyBondHealths = actor->familyBondHealths;
    const float* supportChunkHealths = actor->supportChunkHealths;

    // Find nearest chunk. 
    uint32_t closestNode = findClosestNode(desc.position
        , firstGraphNodeIndex, graphNodeIndexLinks
        , adjacencyPartition, adjacentNodeIndices, adjacentBondIndices
        , assetBonds, familyBondHealths
        , assetChunks, supportChunkHealths, chunkIndices);

    // Breadth-first support graph traversal. For radial falloff metric distance is measured along the edges of the graph
    ExtDamageAcceleratorInternal* damageAccelerator = programParams->accelerator ? static_cast<ExtDamageAcceleratorInternal*>(programParams->accelerator) : nullptr;
    NVBLAST_ASSERT_WITH_MESSAGE(damageAccelerator, "This shader requires damage accelerator passed");
    if (!isInvalidIndex(chunkIndices[closestNode]) && damageAccelerator)
    {
        struct NodeData
        {
            uint32_t index;
            float distance;
        };

        // Calculating scratch size and requesting it from the accelerator
        const uint32_t bondCount = actor->adjacencyPartition[actor->assetNodeCount];
        const size_t nodeQueueSize = align16(FixedQueue<NodeData>::requiredMemorySize(actor->graphNodeCount));
        const size_t visitedBitmapSize = align16(FixedBitmap::requiredMemorySize(bondCount));
        const size_t scratchSize = 16 + nodeQueueSize + visitedBitmapSize;

        void* scratch = damageAccelerator->getImmediateScratch(scratchSize);

        // prepare intermediate data on scratch
        scratch = (void*)align16((size_t)scratch); // Bump to 16-byte alignment
        FixedQueue<NodeData>* nodeQueue = new (scratch)FixedQueue<NodeData>(actor->graphNodeCount);
        scratch = pointerOffset(scratch, align16(nodeQueueSize));
        FixedBitmap* visitedBitmap = new (scratch)FixedBitmap(bondCount);
        scratch = pointerOffset(scratch, align16(FixedBitmap::requiredMemorySize(bondCount)));

        // initalize traversal
        nodeQueue->pushBack({ closestNode, 0.f });
        visitedBitmap->clear();

        while (!nodeQueue->empty())
        {
            NodeData currentNode = nodeQueue->popFront();
            const uint32_t startIndex = adjacencyPartition[currentNode.index];
            const uint32_t stopIndex = adjacencyPartition[currentNode.index + 1];

            for (uint32_t adjacentNodeIndex = startIndex; adjacentNodeIndex < stopIndex; adjacentNodeIndex++)
            {
                const uint32_t neighbourIndex = adjacentNodeIndices[adjacentNodeIndex];
                const uint32_t bondIndex = adjacentBondIndices[adjacentNodeIndex];
                const NvBlastBond& bond = assetBonds[bondIndex];

                const NvVec3& bondCentroid = (reinterpret_cast<const NvVec3&>(bond.centroid));

                if (!canTakeDamage(familyBondHealths[bondIndex]))
                    continue;
                
                if (visitedBitmap->test(bondIndex))
                    continue;
                visitedBitmap->set(bondIndex);

                const uint32_t chunkIndex0 = chunkIndices[currentNode.index];
                const uint32_t chunkIndex1 = chunkIndices[neighbourIndex];
                const nvidia::NvVec3& c0 = reinterpret_cast<const nvidia::NvVec3&>(assetChunks[chunkIndex0].centroid);
                bool isNeighbourWorldChunk = isInvalidIndex(chunkIndex1);
                const nvidia::NvVec3& c1 = isNeighbourWorldChunk ? bondCentroid : (reinterpret_cast<const nvidia::NvVec3&>(assetChunks[chunkIndex1].centroid));

                const float distance = (c1 - c0).magnitude() * (isNeighbourWorldChunk ? 2.f : 1.f);
                float totalDistance = currentNode.distance + distance;
                float totalDamage = desc.damage * falloffProfile(desc.minRadius, desc.maxRadius, totalDistance);
                if (totalDamage > 0.0f && bondFractureCount < bondFractureCountMax)
                {
                    NvBlastBondFractureData& frac = commandBuffers->bondFractures[bondFractureCount++];
                    frac.userdata = bond.userData;
                    frac.nodeIndex0 = currentNode.index;
                    frac.nodeIndex1 = neighbourIndex;
                    frac.health = totalDamage;
                    if (!isNeighbourWorldChunk)
                    {
                        nodeQueue->pushBack({ neighbourIndex, totalDistance });
                    }
                }
            }
        }
    }

    commandBuffers->bondFractureCount = bondFractureCount;
    commandBuffers->chunkFractureCount = 0;
}

void NvBlastExtImpactSpreadSubgraphShader(NvBlastFractureBuffers* commandBuffers, const NvBlastSubgraphShaderActor* actor, const void* params)
{
    uint32_t chunkFractureCount = 0;
    uint32_t chunkFractureCountMax = commandBuffers->chunkFractureCount;
    const uint32_t chunkIndex = actor->chunkIndex;
    const NvBlastExtProgramParams* programParams = static_cast<const NvBlastExtProgramParams*>(params);
    const NvBlastExtImpactSpreadDamageDesc& desc = *static_cast<const NvBlastExtImpactSpreadDamageDesc*>(programParams->damageDesc);

    if (chunkFractureCount < chunkFractureCountMax)
    {
        NvBlastChunkFractureData& frac = commandBuffers->chunkFractures[chunkFractureCount++];
        frac.chunkIndex = chunkIndex;
        frac.health = desc.damage;
    }

    commandBuffers->bondFractureCount = 0;
    commandBuffers->chunkFractureCount = chunkFractureCount;
}
