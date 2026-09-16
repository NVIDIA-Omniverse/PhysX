// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// This include must come first
// clang-format off
// clang-format on

#include <carb/logging/Log.h>

#include <PxPhysicsAPI.h>
#include <common/foundation/Allocator.h>
#include "extensions/PxSamplingExt.h"

#include "../service/CookingTask.h"
#include "../service/CookingComputeService.h"


using namespace ::physx;
using namespace omni::physx;
namespace cookingtask
{

static void writeParticlePoissonSamplingData(PxDefaultMemoryOutputStream& outData,
                                             const carb::Float3* positions,
                                             const uint32_t positionsSize)
{
    outData.write(&positionsSize, sizeof(positionsSize));
    outData.write(positions, sizeof(carb::Float3) * positionsSize);
}

void readParticlePoissonSamplingData(PhysxCookingParticlePoissonSamplingData& out,
                                     const PhysxCookedDataSpan& cookedData)
{
    const uint8_t* dataPtr = reinterpret_cast<const uint8_t*>(cookedData.data);

    out.positionsSize = *reinterpret_cast<const uint32_t*>(dataPtr);
    dataPtr += sizeof(out.positionsSize);

    out.positions = reinterpret_cast<const carb::Float3*>(dataPtr);
    dataPtr += out.positionsSize * sizeof(carb::Float3);

    CARB_ASSERT(dataPtr == reinterpret_cast<const uint8_t*>(cookedData.data) + cookedData.sizeInBytes);
}

class PoissonSamplingCookingTask : public CookingTask
{
public:
    PoissonSamplingCookingTask(
        const ParticlePoissonSamplingCookingParams& params,
        omni::physx::PhysxCookingComputeResult& result)
        : CookingTask(result)
        , m_samplingDistance(params.samplingDistance)
        , m_sampleVolume(params.sampleVolume)
        , m_maxSamples(params.maxSamples)
    {
        // params.shearScale is carb::Double3[3] holding the producer's GfMatrix3d
        // row by row (CookingDataAsync memcpy's GfMatrix3d::data() into it). PxMat33d
        // holds the same nine doubles column by column, and Gf row i is PhysX column
        // i, so this is an element copy with no transpose -- see the mapping table in
        // common/foundation/MatrixTools.h.
        static_assert(sizeof(params.shearScale) == sizeof(::physx::PxMat33d));
        const carb::Double3* ss = params.shearScale;
        m_shearScale = PxMat33d(PxVec3d(ss[0].x, ss[0].y, ss[0].z), PxVec3d(ss[1].x, ss[1].y, ss[1].z),
                                PxVec3d(ss[2].x, ss[2].y, ss[2].z));
    }

    virtual ~PoissonSamplingCookingTask(void)
    {
        // Wait until thread is finished
        futureWait();
        // Finalize the results; store them in the USD prim and mesh caches
        if(!CookingTask::isFinalized())
        {
            finalize();
        }
    }

    /* must be threadSafe!! */
    virtual void performTask(void) final
    {
        if (!isCanceled())
        {
            performTriangulation();
            uint32_t vertexCount;
            const float* vertices = getVertices(vertexCount);
            std::vector<float> scaledVertices;
            scaledVertices.insert(scaledVertices.begin(), vertices, vertices + vertexCount * 3);

            for (uint32_t i = 0; i < vertexCount * 3; i += 3)
            {
                // Gf `v * M` (row-vector) is PhysX `M * v` -- the operands swap.
                const PxVec3d point =
                    m_shearScale.transform(PxVec3d(double(vertices[i]), double(vertices[i + 1]), double(vertices[i + 2])));
                scaledVertices[i] = float(point.x);
                scaledVertices[i + 1] = float(point.y);
                scaledVertices[i + 2] = float(point.z);
            }
            vertices = scaledVertices.data();

            uint32_t triCount;
            const uint32_t* triIndices = getIndices(triCount);

            float volumeSamplingDistance = m_sampleVolume ? m_samplingDistance : 0.0f;

            m_positions.resize(0);

            PxSimpleTriangleMesh mesh;
            mesh.points.data = &vertices[0];
            mesh.points.count = PxU32(vertexCount);
            mesh.triangles.data = &triIndices[0];
            mesh.triangles.count = PxU32(triCount);

            bool success = PxSamplingExt::poissonSample(
                mesh,
                m_samplingDistance,
                m_positions,
                volumeSamplingDistance,
                NULL, NULL, NULL, NULL,
                m_maxSamples);

            setSucceeded(success);
            if ((m_positions.size() >= m_maxSamples) && (m_maxSamples > 0))
            {
                CARB_LOG_WARN("Mesh sampling aborted because maxSamples was reached. Consider increasing maxSamples or sampling distance.");
                if ((m_positions.size() > m_maxSamples) && (m_maxSamples > 0))
                {
                    m_positions.removeRange(m_maxSamples, m_positions.size() - m_maxSamples);
                }
            }
        }
        setFinished(true);
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
        else if (!CookingTask::isSucceeded())
        {
            CookingTask::setFinalized(true);
            CookingTask::fireFinishedCallback(omni::physx::PhysxCookingResult::eERROR_COOKING_FAILED);
        }
        else
        {
            // Marks this cooked data as fully processed
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
            writeParticlePoissonSamplingData(outputStream, (const carb::Float3*)m_positions.begin(), m_positions.size());

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

    //input
    PxMat33d m_shearScale;
    float m_samplingDistance;
    bool m_sampleVolume;
    uint32_t m_maxSamples;

    //output
    PxArray<PxVec3> m_positions;
};

CookingTask* createPoissonSamplingCookingTask(
    const ParticlePoissonSamplingCookingParams& params,
    omni::physx::PhysxCookingComputeResult& result
)
{
    return new PoissonSamplingCookingTask(params,
                                          result);
}

}
