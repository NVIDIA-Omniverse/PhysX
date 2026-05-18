// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include <carb/logging/Log.h>

#include <PxPhysicsAPI.h>
#include <common/foundation/Allocator.h>
#include "extensions/PxSamplingExt.h"

#include "../service/CookingTask.h"
#include "../service/CookingComputeService.h"


using namespace ::physx;
using namespace omni::physx;
using namespace pxr;
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
        static_assert(sizeof(params.shearScale) == sizeof(GfMatrix3d));
        m_shearScale = *reinterpret_cast<const GfMatrix3d*>(params.shearScale);
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
                GfVec3d point = GfVec3d(&vertices[i]) * m_shearScale;
                scaledVertices[i] = point[0];
                scaledVertices[i + 1] = point[1];
                scaledVertices[i + 2] = point[2];
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
            CookingTask::setFinalized(true); // Set the finalized semaphore to true
            CookingTask::fireFinishedCallback(omni::physx::PhysxCookingResult::eERROR_COOKING_FAILED);
        }
        else
        {
            // Set the finalized flag to true, indicating that we have fully
            // completed processing this cooked data
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
    GfMatrix3d m_shearScale;
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
