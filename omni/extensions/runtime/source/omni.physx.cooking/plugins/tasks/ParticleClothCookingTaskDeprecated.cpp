// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "../service/CookingTask.h"
#include "../service/CookingComputeService.h"

#include <common/foundation/Algorithms.h>
#include <carb/logging/Log.h>

#include "extensions/PxParticleClothCooker.h"
#include "extensions/PxTetMakerExt.h"


using namespace ::physx;
using namespace omni::physx;
using namespace ::physx::ExtGpu;
using namespace pxr;

namespace cookingtask
{

static void writeParticleClothMeshDataDeprecated(PxDefaultMemoryOutputStream& data,
    const carb::Int2* springIndices, const uint32_t springIndicesSize,
    const float* springStiffnesses, const float* springDampings, const float* springRestLengths,
    const uint32_t* weldedTriangleIndices, const uint32_t weldedTriangleIndicesSize,
    const uint32_t* verticesRemapToWeld, const uint32_t verticesRemapToWeldSize,
    const uint32_t* verticesRemapToOrig, const uint32_t verticesRemapToOrigSize,
    const float inflatableVolume)
{
    data.write(&springIndicesSize, sizeof(uint32_t));
    if (springIndicesSize > 0) //don't overwrite springs if nothing to write
    {
        data.write(springIndices, sizeof(carb::Int2)*springIndicesSize);
        data.write(springStiffnesses, sizeof(float)*springIndicesSize);
        data.write(springDampings, sizeof(float)*springIndicesSize);
        data.write(springRestLengths, sizeof(float)*springIndicesSize);
    }

    data.write(&weldedTriangleIndicesSize, sizeof(uint32_t));
    if (weldedTriangleIndicesSize > 0)
        data.write(weldedTriangleIndices, sizeof(uint32_t)*weldedTriangleIndicesSize);

    data.write(&verticesRemapToWeldSize, sizeof(uint32_t));
    if (verticesRemapToWeldSize > 0)
        data.write(verticesRemapToWeld, sizeof(uint32_t)*verticesRemapToWeldSize);

    data.write(&verticesRemapToOrigSize, sizeof(uint32_t));
    if (verticesRemapToOrigSize > 0)
        data.write(verticesRemapToOrig, sizeof(uint32_t)*verticesRemapToOrigSize);

    data.write(&inflatableVolume, sizeof(float));
}

void readParticleClothMeshDataDeprecated(PhysxCookingParticleClothMeshDataDeprecated& out, const PhysxCookedDataSpan& cookedData)
{
    const uint8_t* dataPtr = reinterpret_cast<const uint8_t*>(cookedData.data);

    out.springIndicesSize = *reinterpret_cast<const uint32_t*>(dataPtr);
    dataPtr += sizeof(uint32_t);

    out.springIndices = (out.springIndicesSize > 0) ? reinterpret_cast<const carb::Int2*>(dataPtr) : nullptr;
    dataPtr += out.springIndicesSize * sizeof(carb::Int2);

    out.springStiffnesses = (out.springIndicesSize > 0) ? reinterpret_cast<const float*>(dataPtr) : nullptr;
    dataPtr += out.springIndicesSize * sizeof(float);

    out.springDampings = (out.springIndicesSize > 0) ? reinterpret_cast<const float*>(dataPtr) : nullptr;
    dataPtr += out.springIndicesSize * sizeof(float);

    out.springRestLengths = (out.springIndicesSize > 0) ? reinterpret_cast<const float*>(dataPtr) : nullptr;
    dataPtr += out.springIndicesSize * sizeof(float);

    out.weldedTriangleIndicesSize = *reinterpret_cast<const uint32_t*>(dataPtr);
    dataPtr += sizeof(uint32_t);

    out.weldedTriangleIndices = (out.weldedTriangleIndicesSize > 0) ? reinterpret_cast<const uint32_t*>(dataPtr) : nullptr;
    dataPtr += out.weldedTriangleIndicesSize * sizeof(uint32_t);

    out.verticesRemapToWeldSize = *reinterpret_cast<const uint32_t*>(dataPtr);
    dataPtr += sizeof(uint32_t);

    out.verticesRemapToWeld = (out.verticesRemapToWeldSize > 0) ? reinterpret_cast<const uint32_t*>(dataPtr) : nullptr;
    dataPtr += out.verticesRemapToWeldSize * sizeof(uint32_t);

    out.verticesRemapToOrigSize = *reinterpret_cast<const uint32_t*>(dataPtr);
    dataPtr += sizeof(uint32_t);

    out.verticesRemapToOrig = (out.verticesRemapToOrigSize > 0) ? reinterpret_cast<const uint32_t*>(dataPtr) : nullptr;
    dataPtr += out.verticesRemapToOrigSize * sizeof(uint32_t);

    out.inflatableVolume = *reinterpret_cast<const float*>(dataPtr);
    dataPtr += sizeof(float);

    CARB_ASSERT(dataPtr == reinterpret_cast<const uint8_t*>(cookedData.data) + cookedData.sizeInBytes);
}

class MeshCleaner
{
public:
    MeshCleaner(PxU32 nbVerts, const PxVec3* verts, PxU32 nbTris, const PxU32* indices, PxF32 meshWeldTolerance);
    ~MeshCleaner();

    PxU32	 mNbVerts;
    PxU32	 mNbTris;
    PxVec3* mVerts;
    PxU32*	 mIndices;
    PxU32*	 mRemapTriangles;
    PxU32*  mRemapVertsToWeld;
    PxU32*  mRemapVertsToOrig;
};

struct Indices
{
    PxU32 mRef[3];

    PX_FORCE_INLINE bool operator!=(const Indices&v) const { return mRef[0] != v.mRef[0] || mRef[1] != v.mRef[1] || mRef[2] != v.mRef[2]; }
};

static PX_FORCE_INLINE PxU32 getHashValue(const PxVec3& v)
{
    const PxU32* h = reinterpret_cast<const PxU32*>(&v.x);
    const PxU32 f = (h[0] + h[1] * 11 - (h[2] * 17)) & 0x7fffffff;	// avoid problems with +-0
    return (f >> 22) ^ (f >> 12) ^ (f);
}

static PX_FORCE_INLINE PxU32 getHashValue(const Indices& v)
{
    //	const PxU32* h = v.mRef;
    //	const PxU32 f = (h[0]+h[1]*11-(h[2]*17)) & 0x7fffffff;	// avoid problems with +-0
    //	return (f>>22)^(f>>12)^(f);

    PxU32 a = v.mRef[0];
    PxU32 b = v.mRef[1];
    PxU32 c = v.mRef[2];
    a = a - b;  a = a - c;  a = a ^ (c >> 13);
    b = b - c;  b = b - a;  b = b ^ (a << 8);
    c = c - a;  c = c - b;  c = c ^ (b >> 13);
    a = a - b;  a = a - c;  a = a ^ (c >> 12);
    b = b - c;  b = b - a;  b = b ^ (a << 16);
    c = c - a;  c = c - b;  c = c ^ (b >> 5);
    a = a - b;  a = a - c;  a = a ^ (c >> 3);
    b = b - c;  b = b - a;  b = b ^ (a << 10);
    c = c - a;  c = c - b;  c = c ^ (b >> 15);
    return c;
}

MeshCleaner::MeshCleaner(PxU32 nbVerts, const PxVec3* srcVerts, PxU32 nbTris, const PxU32* srcIndices, PxF32 meshWeldTolerance)
{
    PxVec3* cleanVerts = PX_ALLOCATE(PxVec3, nbVerts, "MeshCleaner");
    PX_ASSERT(cleanVerts);

    PxU32* indices = PX_ALLOCATE(PxU32, (nbTris * 3), "MeshCleaner");

    PxU32* remapTriangles = PX_ALLOCATE(PxU32, nbTris, "MeshCleaner");

    PxU32* vertexIndices = NULL;
    if (meshWeldTolerance != 0.0f)
    {
        vertexIndices = PX_ALLOCATE(PxU32, nbVerts, "MeshCleaner");
        const PxF32 weldTolerance = 1.0f / meshWeldTolerance;
        // snap to grid
        for (PxU32 i = 0; i < nbVerts; i++)
        {
            vertexIndices[i] = i;
            cleanVerts[i] = PxVec3(PxFloor(srcVerts[i].x*weldTolerance + 0.5f),
                PxFloor(srcVerts[i].y*weldTolerance + 0.5f),
                PxFloor(srcVerts[i].z*weldTolerance + 0.5f));
        }
    }
    else
    {
        PxMemCopy(cleanVerts, srcVerts, nbVerts * sizeof(PxVec3));
    }

    const PxU32 maxNbElems = PxMax(nbTris, nbVerts);
    const PxU32 hashSize = PxNextPowerOfTwo(maxNbElems);
    const PxU32 hashMask = hashSize - 1;
    PxU32* hashTable = PX_ALLOCATE(PxU32, (hashSize + maxNbElems), "MeshCleaner");
    PX_ASSERT(hashTable);
    PxMemSet(hashTable, 0xff, hashSize * sizeof(PxU32));
    PxU32* const next = hashTable + hashSize;

    mRemapVertsToWeld = PX_ALLOCATE(PxU32, nbVerts, "MeshCleaner");
    PxMemSet(mRemapVertsToWeld, 0xff, nbVerts * sizeof(PxU32));

    for (PxU32 i = 0; i < nbTris * 3; i++)
    {
        const PxU32 vref = srcIndices[i];
        if (vref < nbVerts)
            mRemapVertsToWeld[vref] = 0;
    }

    PxU32 nbCleanedVerts = 0;
    for (PxU32 i = 0; i < nbVerts; i++)
    {
        if (mRemapVertsToWeld[i] == 0xffffffff)
            continue;

        const PxVec3& v = cleanVerts[i];
        const PxU32 hashValue = getHashValue(v) & hashMask;
        PxU32 offset = hashTable[hashValue];

        while (offset != 0xffffffff && cleanVerts[offset] != v)
            offset = next[offset];

        if (offset == 0xffffffff)
        {
            mRemapVertsToWeld[i] = nbCleanedVerts;
            cleanVerts[nbCleanedVerts] = v;
            if (vertexIndices)
                vertexIndices[nbCleanedVerts] = i;
            next[nbCleanedVerts] = hashTable[hashValue];
            hashTable[hashValue] = nbCleanedVerts++;
        }
        else mRemapVertsToWeld[i] = offset;
    }

    PxU32 nbCleanedTris = 0;
    for (PxU32 i = 0; i < nbTris; i++)
    {
        PxU32 vref0 = *srcIndices++;
        PxU32 vref1 = *srcIndices++;
        PxU32 vref2 = *srcIndices++;
        if (vref0 >= nbVerts || vref1 >= nbVerts || vref2 >= nbVerts)
            continue;

        // PT: you can still get zero-area faces when the 3 vertices are perfectly aligned
        const PxVec3& p0 = srcVerts[vref0];
        const PxVec3& p1 = srcVerts[vref1];
        const PxVec3& p2 = srcVerts[vref2];
        const float area2 = ((p0 - p1).cross(p0 - p2)).magnitudeSquared();
        if (area2 == 0.0f)
            continue;

        PxU32 new_vref0 = mRemapVertsToWeld[vref0];
        PxU32 new_vref1 = mRemapVertsToWeld[vref1];
        PxU32 new_vref2 = mRemapVertsToWeld[vref2];
        if (new_vref0 == new_vref1 || new_vref1 == new_vref2 || new_vref2 == new_vref0)
            continue;

        indices[nbCleanedTris * 3 + 0] = new_vref0;
        indices[nbCleanedTris * 3 + 1] = new_vref1;
        indices[nbCleanedTris * 3 + 2] = new_vref2;
        remapTriangles[nbCleanedTris] = i;
        nbCleanedTris++;
    }

    PxU32 nbToGo = nbCleanedTris;
    nbCleanedTris = 0;
    PxMemSet(hashTable, 0xff, hashSize * sizeof(PxU32));

    Indices* const I = reinterpret_cast<Indices*>(indices);
    bool idtRemap = true;
    for (PxU32 i = 0; i < nbToGo; i++)
    {
        const Indices& v = I[i];
        const PxU32 hashValue = getHashValue(v) & hashMask;
        PxU32 offset = hashTable[hashValue];

        while (offset != 0xffffffff && I[offset] != v)
            offset = next[offset];

        if (offset == 0xffffffff)
        {
            const PxU32 originalIndex = remapTriangles[i];
            PX_ASSERT(nbCleanedTris <= i);
            remapTriangles[nbCleanedTris] = originalIndex;
            if (originalIndex != nbCleanedTris)
                idtRemap = false;
            I[nbCleanedTris] = v;
            next[nbCleanedTris] = hashTable[hashValue];
            hashTable[hashValue] = nbCleanedTris++;
        }
    }
    PX_FREE(hashTable);

    if (vertexIndices)
    {
        for (PxU32 i = 0; i < nbCleanedVerts; i++)
            cleanVerts[i] = srcVerts[vertexIndices[i]];
        PX_FREE(vertexIndices);
    }
    mNbVerts = nbCleanedVerts;
    mNbTris = nbCleanedTris;
    mVerts = cleanVerts;
    mIndices = indices;
    if (idtRemap)
    {
        PX_FREE(remapTriangles);
        mRemapTriangles = NULL;
    }
    else
    {
        mRemapTriangles = remapTriangles;
    }

    mRemapVertsToOrig = PX_ALLOCATE(PxU32, nbCleanedVerts, "MeshCleaner");
    PxMemSet(mRemapVertsToOrig, 0xff, nbCleanedVerts * sizeof(PxU32));
    std::vector<PxF32> minDistToWeldedPoint(nbCleanedVerts, FLT_MAX);
    for (uint32_t origIndex = 0; origIndex < nbVerts; ++origIndex)
    {
        const uint32_t weldedIndex = mRemapVertsToWeld[origIndex];
        if (weldedIndex == 0xffffffff)
        {
            //vertices not referenced by triangles don't have a valid mRemapVertsToWeld entry
            continue;
        }
        const float dist = (srcVerts[origIndex] - mVerts[weldedIndex]).magnitude();
        if (dist < minDistToWeldedPoint[weldedIndex])
        {
            mRemapVertsToOrig[weldedIndex] = origIndex;
            minDistToWeldedPoint[weldedIndex] = dist;
        }
    }
}

MeshCleaner::~MeshCleaner()
{
    PX_FREE(mRemapTriangles);
    PX_FREE(mIndices);
    PX_FREE(mVerts);
    PX_FREE(mRemapVertsToWeld);
    PX_FREE(mRemapVertsToOrig);
}

// This class handles cooking a single particle cloth in a background task.
class ParticleClothMeshCookingTask : public cookingtask::CookingTask
{
public:
    ParticleClothMeshCookingTask(
        const ParticleClothMeshCookingParamsDeprecated& params,
        omni::physx::PhysxCookingComputeResult& result)
    : CookingTask(result)
    , m_needsSprings(params.needsSprings)
    , m_isInflatable(params.isInflatable)
    , m_enableWelding(params.enableWelding)
    , m_springStretchStiffness(params.springStretchStiffness)
    , m_springBendStiffness(params.springBendStiffness)
    , m_springShearStiffness(params.springShearStiffness)
    , m_springDamping(params.springDamping)
    , m_inflatableVolume(0.0f)
    {
        m_restPoints.resize(params.restPoints.size());
        memcpy(m_restPoints.data(), params.restPoints.data(), sizeof(carb::Float3)*m_restPoints.size());
    }

    virtual ~ParticleClothMeshCookingTask(void)
    {
        // Wait until thread is finished
        CookingTask::futureWait();
        // Finalize the results; store them in the USD prim and mesh caches
        if(!CookingTask::isFinalized())
        {
            finalize();
        }
    }

    bool isValid(void) const
    {
        return true;
    }

//repeat the definition from the SDK because we don't have a particle system instance. Ideally this would go into PxPhysics
#define MAX_PARTICLE_TRIANGLES 16u

    // This operation happens in a background thread and, therefore, must be
    // completely thread safe.
    virtual void performTask(void) final
    {
        if (!CookingTask::isCanceled())
        {
            CookingTask::setSucceeded(true);
            CookingTask::performTriangulation();

            uint32_t oldTriangleCount;
            uint32_t oldRestPointsSize = uint32_t(m_restPoints.size());
            carb::Float3* oldRestPoints = &m_restPoints[0];

            uint32_t restPointsSize = oldRestPointsSize;
            PxVec3* restPoints = (PxVec3*)oldRestPoints;

            const uint32_t* oldTriangleIndices = CookingTask::getIndices(oldTriangleCount);
            m_weldedTriangleIndices.insert(m_weldedTriangleIndices.begin(), oldTriangleIndices, oldTriangleIndices + oldTriangleCount * 3);

            PxU32* triangleIndices = m_weldedTriangleIndices.data();
            uint32_t triangleCount = oldTriangleCount;
            uint32_t triangleIndicesSize = triangleCount * 3u;

            // check for invalid meshes
            {
                PxSimpleTriangleMesh triMesh;
                triMesh.points.count = oldRestPointsSize;
                triMesh.points.data = oldRestPoints;
                triMesh.triangles.count = oldTriangleCount;
                triMesh.triangles.data = oldTriangleIndices;
                triMesh.flags = PxMeshFlags(0u); // disable both flags
                PxTriangleMeshAnalysisResults skinMeshAnalysis = PxTetMaker::validateTriangleMesh(triMesh);
                if (skinMeshAnalysis & PxTriangleMeshAnalysisResult::eEDGE_SHARED_BY_MORE_THAN_TWO_TRIANGLES)
                {
                    CARB_LOG_ERROR("Cooking mesh for particle cloth simulation failed: %s. The input mesh is non-manifold. ",
                        CookingTask::getPrimPathText().c_str());

                    CookingTask::setSucceeded(false);
                    CookingTask::setFinished(true);
                    return;
                }
            }

            if (m_enableWelding)
            {
                //consider doing "metersPerUnit" query before task creation
                UsdStageWeakPtr stage = CookingTask::getStage();
                if (stage)
                {
                    PxF32 meshWeldTolerance = 0.01f / 100.0f; // set default to 0.01 cm
                    float metersPerUnit = (float)UsdGeomGetStageMetersPerUnit(stage);
                    meshWeldTolerance /= metersPerUnit;

                    cookingtask::MeshCleaner cleaner(oldRestPointsSize, (PxVec3*)oldRestPoints, oldTriangleCount, (PxU32*)triangleIndices, meshWeldTolerance);

                    // Skip if the welded mesh and original mesh are the same
                    if (!(cleaner.mNbVerts == oldRestPointsSize && cleaner.mNbTris == oldTriangleCount))
                    {
                        restPointsSize = cleaner.mNbVerts;
                        triangleIndicesSize = cleaner.mNbTris * 3u;

                        m_weldedRestPoints.resize(restPointsSize);
                        memcpy(m_weldedRestPoints.data(), cleaner.mVerts, sizeof(PxVec3) * restPointsSize);
                        restPoints = m_weldedRestPoints.data();

                        m_weldedTriangleIndices.resize(triangleIndicesSize);
                        memcpy(m_weldedTriangleIndices.data(), cleaner.mIndices, sizeof(uint32_t) * triangleIndicesSize);
                        triangleIndices = m_weldedTriangleIndices.data();

                        m_verticesRemapToWeld.resize(oldRestPointsSize);
                        memcpy(m_verticesRemapToWeld.data(), cleaner.mRemapVertsToWeld, sizeof(PxU32) * oldRestPointsSize);

                        m_verticesRemapToOrig.resize(restPointsSize);
                        memcpy(m_verticesRemapToOrig.data(), cleaner.mRemapVertsToOrig, sizeof(PxU32) * restPointsSize);

                        // check for welded mesh
                        {
                            PxSimpleTriangleMesh triMesh;
                            triMesh.points.count = restPointsSize;
                            triMesh.points.data = restPoints;
                            triMesh.triangles.count = cleaner.mNbTris;
                            triMesh.triangles.data = cleaner.mIndices;
                            triMesh.flags = PxMeshFlags(0u); // disable both flags
                            PxTriangleMeshAnalysisResults skinMeshAnalysis = PxTetMaker::validateTriangleMesh(triMesh);
                            if (skinMeshAnalysis & PxTriangleMeshAnalysisResult::eEDGE_SHARED_BY_MORE_THAN_TWO_TRIANGLES)
                            {
                                CARB_LOG_ERROR("Cooking mesh for particle cloth simulation failed: %s. The input mesh becomes non-manifold after welding. "
                                    "Consider disabling welding.", CookingTask::getPrimPathText().c_str());

                                CookingTask::setSucceeded(false);
                                CookingTask::setFinished(true);
                                return;
                            }
                        }
                    }
                }
            }

            //TODO: fix cooker interface
            std::vector<PxVec4> restPoints4(restPointsSize);
            for (size_t i = 0; i < restPoints4.size(); ++i)
            {
                const PxVec3& src = restPoints[i];
                restPoints4[i] = PxVec4(src, 0.0f);
            }

            PxU32 constraintFlags =
                PxParticleClothConstraint::eTYPE_HORIZONTAL_CONSTRAINT |
                PxParticleClothConstraint::eTYPE_VERTICAL_CONSTRAINT |
                PxParticleClothConstraint::eTYPE_DIAGONAL_CONSTRAINT |
                PxParticleClothConstraint::eTYPE_BENDING_CONSTRAINT |
                PxParticleClothConstraint::eTYPE_DIAGONAL_BENDING_CONSTRAINT;

            PxParticleClothCooker* cooker = PxCreateParticleClothCooker(restPointsSize, restPoints4.data(),
                triangleIndicesSize, triangleIndices, constraintFlags);

            if (m_needsSprings)
            {
                //compute springs from auto parameters
                cooker->cookConstraints();

                int constraintCount = cooker->getConstraintCount();
                PxParticleClothConstraint* constraintBuffer = cooker->getConstraints();

                m_springIndices.reserve(constraintCount);
                m_springStiffnesses.reserve(constraintCount);
                m_springDampings.reserve(constraintCount);
                m_springRestLengths.reserve(constraintCount);

                for (int i = 0; i < constraintCount; i++)
                {
                    const PxParticleClothConstraint& c = constraintBuffer[i];
                    float stiffness{ 0 };
                    switch (c.constraintType)
                    {
                    case PxParticleClothConstraint::eTYPE_INVALID_CONSTRAINT:
                        continue;
                    case PxParticleClothConstraint::eTYPE_HORIZONTAL_CONSTRAINT:
                    case PxParticleClothConstraint::eTYPE_VERTICAL_CONSTRAINT:
                        stiffness = m_springStretchStiffness;
                        break;
                    case PxParticleClothConstraint::eTYPE_DIAGONAL_CONSTRAINT:
                        stiffness = m_springShearStiffness;
                        break;
                    case PxParticleClothConstraint::eTYPE_BENDING_CONSTRAINT:
                        stiffness = m_springBendStiffness;
                        break;
                    default:
                        CARB_ASSERT("Invalid cloth constraint generated by PxClothCooker");
                    }

                    if (stiffness > 0.0f)
                    {
                        CARB_ASSERT(c.particleIndexA != c.particleIndexB);
                        int32_t a = c.particleIndexA;
                        int32_t b = c.particleIndexB;
                        m_springIndices.push_back(carb::Int2{a, b});
                        m_springStiffnesses.push_back(stiffness);
                        m_springDampings.push_back(m_springDamping);
                        m_springRestLengths.push_back(c.length);
                    }
                }
            }
            else
            {
                //trick the cooker into thinking that we provided spring constraints.
                //it will then early out and just limit the triangle indices to MAX_PARTICLE_TRIANGLES per particle.
                PxParticleClothConstraint dummyConstraint;
                cooker->cookConstraints(&dummyConstraint, 0);
            }

            if (m_isInflatable)
            {
                //TODO move equvalent functionality back to SDK
                //cooker->calculateMeshVolume();
                //float inflatableVolume = cooker->getMeshVolume();
                //float inflatableConstraintScale = cooker->getConstraintScale();
                float sum = 0.0f;

                std::vector<PxVec3> gradients;
                gradients.resize(restPointsSize, PxVec3(0.0f));

                PxVec3 centroid(0.f);
                for (uint32_t i = 0; i < restPointsSize; ++i)
                {
                    centroid += *(const PxVec3*)&restPoints[i];
                }

                if (restPointsSize > 1)
                {
                    centroid *= (1.f/restPointsSize);
                }

                //Loop over all triangles
                for(uint32_t i = 0; i < triangleIndicesSize; i += 3)
                {
                    //Get vertices
                    PxVec3 vA = *(const PxVec3*)&restPoints[triangleIndices[i + 0]];
                    PxVec3 vB = *(const PxVec3*)&restPoints[triangleIndices[i + 1]];
                    PxVec3 vC = *(const PxVec3*)&restPoints[triangleIndices[i + 2]];

                    //Calculate 6*volume of tetrahedron with apex at origin
                    PxVec3 n = (vB - vA).cross(vC - vA);
                    sum += (vA - centroid).dot(n);

                    gradients[triangleIndices[i + 0]] += n;
                    gradients[triangleIndices[i + 1]] += n;
                    gradients[triangleIndices[i + 2]] += n;
                }

                PxVec3 gradientComponentSquares(0.0f);
                for (uint32_t i = 0; i < restPointsSize; ++i)
                {
                    gradientComponentSquares += gradients[i].multiply(gradients[i]);
                }

                m_inflatableVolume = sum; //should be / 6.0 but the physx api takes volume*6 now too.
            }

            cooker->release();
        }
        CookingTask::setFinished(true);
    }

    // Once the task is complete, we now need to store the results into USD.
    // This must all be done from the main thread since USD is not thread safe
    virtual void finalize(void) final
    {
        // If the task was canceled or already finalized, skip the saving step
        if (CookingTask::isCanceled() || CookingTask::isFinalized())
        {
            CookingTask::fireFinishedCallback(omni::physx::PhysxCookingResult::eERROR_CANCELED);
        }
        else if(!CookingTask::isSucceeded())
        {
            CookingTask::setFinalized(true); // Set the finalized semaphore to true
            CookingTask::fireFinishedCallback(omni::physx::PhysxCookingResult::eERROR_COOKING_FAILED);
        }
        else
        {
            CookingTask::setFinalized(true);

            // Save the triangulation to the cache in finalize, since saving it out in performTask
            // seemed to exhibit some thread safety issues
            CookingTask::saveTriangulation(getTriangulationOutputStream());

            // Write output to stream, for both cache and direct consumption of the result
            if (CookingTask::getCookedDataOutputStreams().empty())
            {
                CookingTask::getCookedDataOutputStreams().push_back(std::make_unique<PxDefaultMemoryOutputStream>());
            }
            PxDefaultMemoryOutputStream& outputStream = *CookingTask::getCookedDataOutputStreams()[0].get();
            writeParticleClothMeshDataDeprecated(outputStream,
                m_springIndices.data(), (uint32_t)m_springIndices.size(),
                m_springStiffnesses.data(), m_springDampings.data(), m_springRestLengths.data(),
                m_weldedTriangleIndices.data(), (uint32_t)m_weldedTriangleIndices.size(),
                m_verticesRemapToWeld.data(), (uint32_t)m_verticesRemapToWeld.size(),
                m_verticesRemapToOrig.data(), (uint32_t)m_verticesRemapToOrig.size(),
                m_inflatableVolume);

            omni::physx::PhysxCookingComputeResult& result = CookingTask::getResultObject();
            PxDefaultMemoryOutputStream& cookDataStream = outputStream;
            PhysxCookedDataSpan cookedDataSpan;
            result.cookedData = &cookedDataSpan;
            result.cookedDataNumElements = 1;
            cookedDataSpan.data = cookDataStream.getData();
            cookedDataSpan.sizeInBytes = cookDataStream.getSize();

            CookingTask::fireFinishedCallback(PhysxCookingResult::eVALID);
        }
    }

    //in
    std::vector<carb::Float3> m_restPoints;
    bool m_needsSprings;
    bool m_isInflatable;
    bool m_enableWelding;
    float m_springStretchStiffness;
    float m_springBendStiffness;
    float m_springShearStiffness;
    float m_springDamping;

    //out
    std::vector<carb::Int2> m_springIndices;
    std::vector<float> m_springStiffnesses;
    std::vector<float> m_springDampings;
    std::vector<float> m_springRestLengths;
    std::vector<PxVec3> m_weldedRestPoints;
    std::vector<uint32_t> m_weldedTriangleIndices;
    std::vector<uint32_t> m_verticesRemapToWeld;
    std::vector<uint32_t> m_verticesRemapToOrig;
    float m_inflatableVolume;
};

CookingTask* createParticleClothMeshCookingTaskDeprecated(
    const ParticleClothMeshCookingParamsDeprecated& params,
    omni::physx::PhysxCookingComputeResult& result
)
{
    return new ParticleClothMeshCookingTask(params,
                                            result);
}

}
