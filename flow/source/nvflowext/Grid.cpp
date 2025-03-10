// SPDX-FileCopyrightText: Copyright (c) 2014-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
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

#include "NvFlowExt.h"

#include "NvFlowUploadBuffer.h"
#include "NvFlowBufferVariable.h"
#include "NvFlowTextureVariable.h"

#include "NvFlowMath.h"

#include <stdlib.h>
#include <string.h>

#include "NvFlowTimer.h"

namespace NvFlowGridSimple
{
    static const NvFlowUint densityLevelIdx = 0u;
    static const NvFlowUint velocityLevelIdx = 1u;

    struct TimeStepper
    {
        float deltaTime;
        float timeError;
        float fixedDt;
        int maxSteps;
        int numSteps;
        float maxTimeError;
    };

    static void TimeStepper_init(TimeStepper* ptr)
    {
        ptr->deltaTime = 0.f;
        ptr->timeError = 0.f;
        ptr->fixedDt = (1.f / 60.f);
        ptr->maxSteps = 1;
        ptr->numSteps = 0;
        ptr->maxTimeError = (4.f / 60.f);
    }

    static void TimeStepper_configure(TimeStepper* ptr, float stepsPerSecond, NvFlowUint maxStepsPerSimulate)
    {
        ptr->fixedDt = 1.f / stepsPerSecond;
        ptr->maxTimeError = 4.f / stepsPerSecond;
        ptr->maxSteps = maxStepsPerSimulate;
    }

    static int TimeStepper_update(TimeStepper* ptr, float dt)
    {
        ptr->deltaTime = dt;

        ptr->timeError += ptr->deltaTime;

        ptr->numSteps = int(floorf((ptr->timeError / ptr->fixedDt)));

        if (ptr->numSteps < 0)
            ptr->numSteps = 0;

        if (ptr->numSteps > ptr->maxSteps)
            ptr->numSteps = ptr->maxSteps;

        ptr->timeError -= ptr->fixedDt * float(ptr->numSteps);

        if (ptr->timeError > ptr->maxTimeError)
        {
            ptr->timeError = ptr->maxTimeError;
        }

        return ptr->numSteps;
    }

    struct GridNanoVdbReadback
    {
        NvFlowSparseNanoVdbExportReadback data = {};
        NvFlowArray<NvFlowUint> temperatureNanoVdbReadback;
        NvFlowArray<NvFlowUint> fuelNanoVdbReadback;
        NvFlowArray<NvFlowUint> burnNanoVdbReadback;
        NvFlowArray<NvFlowUint> smokeNanoVdbReadback;
        NvFlowArray<NvFlowUint> velocityNanoVdbReadback;
        NvFlowArray<NvFlowUint> divergenceNanoVdbReadback;
        NvFlowArray<NvFlowUint> rgbaNanoVdbReadback;
        NvFlowArray<NvFlowUint> rgbNanoVdbReadback;
    };

    struct GridReadbackTypeSnapshot
    {
        NvFlowArray<NvFlowUint8*> instanceDatas;
        NvFlowArrayPointer<GridNanoVdbReadback*> gridNanoVdbReadbacks;
        NvFlowArrayPointer<NvFlowSparseNanoVdbExportInterop*> gridNanoVdbInterops;
        NvFlowArrayPointer<NvFlowGridReadbackData*> gridReadbackData;
    };

    void GridReadbackTypeSnapshot_reset(GridReadbackTypeSnapshot* ptr)
    {
        ptr->instanceDatas.size = 0u;
        ptr->gridNanoVdbReadbacks.size = 0u;
        ptr->gridNanoVdbInterops.size = 0u;
        ptr->gridReadbackData.size = 0u;
    }

    struct GridReadbackSnapshot
    {
        NvFlowGridParamsDescSnapshot snapshot = {};
        NvFlowArray<NvFlowDatabaseTypeSnapshot> typeSnapshots;
        NvFlowArrayPointer<GridReadbackTypeSnapshot*> gridTypeSnapshots;
    };

    struct GridGridParamsReadback
    {
        NvFlowArray<char> name;
        NvFlowGridParamsNamed* gridParamsNamed;

        NvFlowArrayPointer<GridReadbackSnapshot*> gridReadbackSnapshots;
    };

    struct Grid
    {
        NvFlowGridDesc desc = {};

        NvFlowContextInterface contextInterface = {};

        NvFlowSparseInterface sparseInterface = {};
        NvFlowSparse* sparse = nullptr;
        NvFlowSparse* sparseIsosurface = nullptr;

        NvFlowSparseNanoVdbExport mSparseNanoVdbExport = {};
        NvFlowAdvectionCombustionDensity mAdvectionCombustionDensity = {};
        NvFlowAdvectionCombustionVelocity mAdvectionCombustionVelocity = {};
        NvFlowEmitterSphere mEmitterSphere = {};
        NvFlowEmitterSphereAllocate mEmitterSphereAllocate = {};
        NvFlowEmitterBox mEmitterBox = {};
        NvFlowEmitterBoxAllocate mEmitterBoxAllocate = {};
        NvFlowEmitterPoint mEmitterPoint = {};
        NvFlowEmitterPointAllocate mEmitterPointAllocate = {};
        NvFlowEmitterMesh mEmitterMesh = {};
        NvFlowEmitterMeshAllocate mEmitterMeshAllocate = {};
        NvFlowEmitterTexture mEmitterTexture = {};
        NvFlowEmitterTextureAllocate mEmitterTextureAllocate = {};
        NvFlowEmitterNanoVdb mEmitterNanoVdb = {};
        NvFlowEmitterNanoVdbAllocate mEmitterNanoVdbAllocate = {};
        NvFlowEllipsoidRaster mEllipsoidRaster = {};
        NvFlowEllipsoidRasterAllocate mEllipsoidRasterAllocate = {};
        NvFlowPressure mPressure = {};
        NvFlowVorticity mVorticity = {};
        NvFlowSummary mSummary = {};
        NvFlowSummaryAllocate mSummaryAllocate = {};
        NvFlowShadow mShadow = {};
        NvFlowDebugVolume mDebugVolume = {};
        NvFlowRayMarch mRayMarch = {};
        NvFlowRayMarchUpdateColormap mRayMarchUpdateColormap = {};
        NvFlowRayMarchIsosurface mRayMarchIsosurface = {};
        NvFlowRayMarchCopyTexture mRayMarchCopyTexture = {};

        NvFlowTextureVariable velocityVariable = {};
        NvFlowTextureVariable densityVariable = {};
        NvFlowTextureVariable densityInterpVariable = {};
        NvFlowTextureVariable coarseDensityVariable = {};
        NvFlowTextureVariable voxelWeightFineVariable = {};
        NvFlowTextureVariable voxelWeightCoarseVariable = {};
        NvFlowTextureVariable shadowVariable = {};
        NvFlowTextureVariable colormapVariable = {};

        NvFlowBufferVariable temperatureNanoVdbVariable = {};
        NvFlowBufferVariable fuelNanoVdbVariable = {};
        NvFlowBufferVariable burnNanoVdbVariable = {};
        NvFlowBufferVariable smokeNanoVdbVariable = {};
        NvFlowBufferVariable velocityNanoVdbVariable = {};
        NvFlowBufferVariable divergenceNanoVdbVariable = {};

        NvFlowGridRenderDataNanoVdb gridRenderDataNanoVdb = {};
        NvFlowArray<NvFlowGridRenderDataNanoVdbReadback> gridRenderDataNanoVdbReadbacks;

        NvFlowTextureVariable splatDensityVariable = {};

        NvFlowUint activeBlockCount = 0u;
        NvFlowUint activeBlockCountIsosurface = 0u;

        NvFlowGridSimulateLayerParams simulateDefault = {};
        NvFlowGridOffscreenLayerParams offscreenDefault = {};
        NvFlowGridRenderLayerParams renderDefault = {};

        NvFlowArray<const NvFlowGridSimulateLayerParams*> simulateParams;
        NvFlowArrayPointer<NvFlowGridSimulateLayerParams*> simulateParamsLevels;
        NvFlowArray<int> simulateParamsRootLevel;
        NvFlowArray<const NvFlowGridOffscreenLayerParams*> offscreenParams;
        NvFlowArray<const NvFlowGridRenderLayerParams*> renderParams;

        NvFlowArray<NvFlowSparseUpdateLayerParams> updateLayerParams;
        NvFlowArray<const NvFlowSummaryAllocateParams*> summaryAllocateParams;
        NvFlowArray<const NvFlowSparseNanoVdbExportParams*> nanoVdbExportParams;
        NvFlowArray<const NvFlowAdvectionCombustionParams*> advectionCombustionParams;
        NvFlowArray<const NvFlowVorticityParams*> vorticityParams;
        NvFlowArray<const NvFlowPressureParams*> pressureParams;
        NvFlowArray<const NvFlowRayMarchColormapParams*> rayMarchColormapParams;
        NvFlowArray<const NvFlowShadowParams*> shadowParams;
        NvFlowArray<const NvFlowDebugVolumeParams*> debugVolumeParams;
        NvFlowArray<const NvFlowRayMarchParams*> rayMarchParams;

        NvFlowArray<const NvFlowGridIsosurfaceLayerParams*> isosurfaceParams;
        NvFlowArray<NvFlowSparseUpdateLayerParams> updateIsosurfaceLayerParams;
        NvFlowArray<const NvFlowEllipsoidRasterParams*> ellipsoidRasterParams;
        NvFlowArray<const NvFlowRayMarchIsosurfaceParams*> rayMarchIsosurfaceParams;

        NvFlowArray<int> physicsCollisionLayers;

        NvFlowArray<NvFlowInt4> locations;
        NvFlowArray<NvFlowInt4> locationsIsosurface;

        NvFlowUint64 updateId = 1llu;
        NvFlowUint64 updateIdIsosurface = 1llu;

        NvFlowArray<int> autoRescaleOldLayerAndLevels;
        NvFlowArray<float> autoRescaleOldDensityCellSizes;
        NvFlowUint autoRescaleMaxLocations = 0u;
        NvFlowUint autoRescaleLevel = 0u;
        NvFlowUint autoRescaleStableFrames = 0u;
        NvFlowArray<NvFlowUint2> autoRescaleHistoryLevelCounts;

        NvFlowUint64 streamingVersion = 0llu;
        NvFlowUint64 streamingFinishedVersion = 0llu;
        NvFlowUint64 streamingPointsVersion = 0llu;
        NvFlowUint64 streamingNanoVdbVersion = 0llu;

        TimeStepper timeStepper;
        NvFlowArray<float> layerDeltaTimes;

        // readback
        NvFlowArrayPointer<GridGridParamsReadback*> gridParamsReadbacks;
    };

    NV_FLOW_CAST_PAIR(NvFlowGrid, Grid)

    NvFlowGrid* createGrid(NvFlowContextInterface* contextInterface, NvFlowContext* context, NvFlowOpList* opListIn, NvFlowExtOpList* extOpListIn, const NvFlowGridDesc* desc)
    {
        auto ptr = new Grid();

        TimeStepper_init(&ptr->timeStepper);

        ptr->simulateDefault = NvFlowGridSimulateLayerParams_default;
        ptr->offscreenDefault = NvFlowGridOffscreenLayerParams_default;
        ptr->renderDefault = NvFlowGridRenderLayerParams_default;

        NvFlowOpList opList = {};
        NvFlowExtOpList extOpList = {};
        NvFlowOpList_duplicate(&opList, opListIn);
        NvFlowExtOpList_duplicate(&extOpList, extOpListIn);

        ptr->desc = *desc;

        NvFlowContextInterface_duplicate(&ptr->contextInterface, contextInterface);

        NvFlowSparseInterface_duplicate(&ptr->sparseInterface, opList.getSparseInterface());
        ptr->sparse = ptr->sparseInterface.create(&ptr->contextInterface, context, ptr->desc.maxLocations);
        ptr->sparseIsosurface = ptr->sparseInterface.create(&ptr->contextInterface, context, ptr->desc.maxLocationsIsosurface);

#if 0
        // describe op graph
        {
            static const NvFlowUint opInterfaceCount = 18u;
            NvFlowReflectContext* opReflectContexts[opInterfaceCount] = { };
            NvFlowOpInterface* opInterfaces[opInterfaceCount] = {
                opList.pAdvectionCombustionDensity(),
                opList.pAdvectionCombustionVelocity(),
                extOpList.pEmitterSphereVelocity(),
                extOpList.pEmitterSphereVelocityAllocate(),
                extOpList.pEmitterSphereDensity(),
                extOpList.pEmitterSphereDensityAllocate(),
                extOpList.pEmitterDensityFieldVelocity(),
                extOpList.pEmitterDensityFieldDensity(),
                extOpList.pEllipsoidRaster(),
                extOpList.pEllipsoidRasterAllocate(),
                opList.pPressure(),
                opList.pVorticity(),
                opList.pSummary(),
                opList.pSummaryAllocate(),
                extOpList.pShadow(),
                extOpList.pRayMarch(),
                extOpList.pRayMarchUpdateColormap(),
                extOpList.pRayMarchIsosurface()
            };
            ptr->opGraphInterface.addOpInterfaces(ptr->simulationOpGraph, opReflectContexts, opInterfaces, opInterfaceCount);

            ptr->opGraphInterface.addSnippet(
                ptr->simulationOpGraph,
                ""
                , ptr->contextInterface.getLogPrint(context)
            );
        }
#endif

#define NV_FLOW_OP_CREATE(name) \
        NvFlow##name##PinsIn name##_pinsIn = {}; \
        name##_pinsIn.contextInterface = &ptr->contextInterface; \
        name##_pinsIn.context = context; \
        NvFlow##name##PinsOut name##_pinsOut = {}; \
        NvFlow##name##_init(&ptr->m##name, opList.p##name(), &name##_pinsIn, &name##_pinsOut);

#define NV_FLOW_OP_CREATE_EXT(name) \
        NvFlow##name##PinsIn name##_pinsIn = {}; \
        name##_pinsIn.contextInterface = &ptr->contextInterface; \
        name##_pinsIn.context = context; \
        NvFlow##name##PinsOut name##_pinsOut = {}; \
        NvFlow##name##_init(&ptr->m##name, extOpList.p##name(), &name##_pinsIn, &name##_pinsOut);

        NV_FLOW_OP_CREATE(SparseNanoVdbExport);
        NV_FLOW_OP_CREATE(AdvectionCombustionDensity);
        NV_FLOW_OP_CREATE(AdvectionCombustionVelocity);
        NV_FLOW_OP_CREATE_EXT(EmitterSphere);
        NV_FLOW_OP_CREATE_EXT(EmitterSphereAllocate);
        NV_FLOW_OP_CREATE_EXT(EmitterBox);
        NV_FLOW_OP_CREATE_EXT(EmitterBoxAllocate);
        NV_FLOW_OP_CREATE_EXT(EmitterPoint);
        NV_FLOW_OP_CREATE_EXT(EmitterPointAllocate);
        NV_FLOW_OP_CREATE_EXT(EmitterMesh);
        NV_FLOW_OP_CREATE_EXT(EmitterMeshAllocate);
        NV_FLOW_OP_CREATE_EXT(EmitterTexture);
        NV_FLOW_OP_CREATE_EXT(EmitterTextureAllocate);
        NV_FLOW_OP_CREATE_EXT(EmitterNanoVdb);
        NV_FLOW_OP_CREATE_EXT(EmitterNanoVdbAllocate);
        NV_FLOW_OP_CREATE_EXT(EllipsoidRaster);
        NV_FLOW_OP_CREATE_EXT(EllipsoidRasterAllocate);
        NV_FLOW_OP_CREATE(Pressure);
        NV_FLOW_OP_CREATE(Vorticity);
        NV_FLOW_OP_CREATE(Summary);
        NV_FLOW_OP_CREATE(SummaryAllocate);
        NV_FLOW_OP_CREATE_EXT(Shadow);
        NV_FLOW_OP_CREATE_EXT(DebugVolume);
        NV_FLOW_OP_CREATE_EXT(RayMarch);
        NV_FLOW_OP_CREATE_EXT(RayMarchUpdateColormap);
        NV_FLOW_OP_CREATE_EXT(RayMarchIsosurface);
        NV_FLOW_OP_CREATE_EXT(RayMarchCopyTexture);

#undef NV_FLOW_OP_CREATE

        NvFlowTextureVariable_init(&ptr->contextInterface, &ptr->velocityVariable);
        NvFlowTextureVariable_init(&ptr->contextInterface, &ptr->densityVariable);
        NvFlowTextureVariable_init(&ptr->contextInterface, &ptr->densityInterpVariable);
        NvFlowTextureVariable_init(&ptr->contextInterface, &ptr->coarseDensityVariable);
        NvFlowTextureVariable_init(&ptr->contextInterface, &ptr->voxelWeightFineVariable);
        NvFlowTextureVariable_init(&ptr->contextInterface, &ptr->voxelWeightCoarseVariable);
        NvFlowTextureVariable_init(&ptr->contextInterface, &ptr->shadowVariable);
        NvFlowTextureVariable_init(&ptr->contextInterface, &ptr->colormapVariable);

        NvFlowBufferVariable_init(&ptr->contextInterface, &ptr->temperatureNanoVdbVariable);
        NvFlowBufferVariable_init(&ptr->contextInterface, &ptr->fuelNanoVdbVariable);
        NvFlowBufferVariable_init(&ptr->contextInterface, &ptr->burnNanoVdbVariable);
        NvFlowBufferVariable_init(&ptr->contextInterface, &ptr->smokeNanoVdbVariable);
        NvFlowBufferVariable_init(&ptr->contextInterface, &ptr->velocityNanoVdbVariable);
        NvFlowBufferVariable_init(&ptr->contextInterface, &ptr->divergenceNanoVdbVariable);

        NvFlowTextureVariable_init(&ptr->contextInterface, &ptr->splatDensityVariable);

        return cast(ptr);
    }

    void destroyGrid(NvFlowContext* context, NvFlowGrid* grid)
    {
        auto ptr = cast(grid);

        for (NvFlowUint64 idx = 0u; idx < ptr->gridParamsReadbacks.size; idx++)
        {
            if (ptr->gridParamsReadbacks[idx]->gridParamsNamed)
            {
                // need to explicitly reset params to make sure client isn't currently mapped
                NvFlowGridParams* gridParams = NvFlowGetGridParamsInterface()->mapGridParamsNamed(
                    ptr->gridParamsReadbacks[idx]->gridParamsNamed);
                NvFlowGetGridParamsInterface()->resetParams(gridParams);

                NvFlowGetGridParamsInterface()->destroyGridParamsNamed(ptr->gridParamsReadbacks[idx]->gridParamsNamed);
                ptr->gridParamsReadbacks[idx]->gridParamsNamed = nullptr;
            }
        }

        NvFlowTextureVariable_destroy(context, &ptr->velocityVariable);
        NvFlowTextureVariable_destroy(context, &ptr->densityVariable);
        NvFlowTextureVariable_destroy(context, &ptr->densityInterpVariable);
        NvFlowTextureVariable_destroy(context, &ptr->coarseDensityVariable);
        NvFlowTextureVariable_destroy(context, &ptr->voxelWeightFineVariable);
        NvFlowTextureVariable_destroy(context, &ptr->voxelWeightCoarseVariable);
        NvFlowTextureVariable_destroy(context, &ptr->shadowVariable);
        NvFlowTextureVariable_destroy(context, &ptr->colormapVariable);

        NvFlowBufferVariable_destroy(context, &ptr->temperatureNanoVdbVariable);
        NvFlowBufferVariable_destroy(context, &ptr->fuelNanoVdbVariable);
        NvFlowBufferVariable_destroy(context, &ptr->burnNanoVdbVariable);
        NvFlowBufferVariable_destroy(context, &ptr->smokeNanoVdbVariable);
        NvFlowBufferVariable_destroy(context, &ptr->velocityNanoVdbVariable);
        NvFlowBufferVariable_destroy(context, &ptr->divergenceNanoVdbVariable);

        NvFlowTextureVariable_destroy(context, &ptr->splatDensityVariable);

#define NV_FLOW_OP_DESTROY(name) \
        NvFlow##name##PinsIn name##_pinsIn = {}; \
        name##_pinsIn.contextInterface = &ptr->contextInterface; \
        name##_pinsIn.context = context; \
        NvFlow##name##PinsOut name##_pinsOut = {}; \
        NvFlow##name##_destroy(&ptr->m##name, &name##_pinsIn, &name##_pinsOut);

        NV_FLOW_OP_DESTROY(SparseNanoVdbExport);
        NV_FLOW_OP_DESTROY(AdvectionCombustionDensity);
        NV_FLOW_OP_DESTROY(AdvectionCombustionVelocity);
        NV_FLOW_OP_DESTROY(EmitterSphere);
        NV_FLOW_OP_DESTROY(EmitterSphereAllocate);
        NV_FLOW_OP_DESTROY(EmitterBox);
        NV_FLOW_OP_DESTROY(EmitterBoxAllocate);
        NV_FLOW_OP_DESTROY(EmitterPoint);
        NV_FLOW_OP_DESTROY(EmitterPointAllocate);
        NV_FLOW_OP_DESTROY(EmitterMesh);
        NV_FLOW_OP_DESTROY(EmitterMeshAllocate);
        NV_FLOW_OP_DESTROY(EmitterTexture);
        NV_FLOW_OP_DESTROY(EmitterTextureAllocate);
        NV_FLOW_OP_DESTROY(EmitterNanoVdb);
        NV_FLOW_OP_DESTROY(EmitterNanoVdbAllocate);
        NV_FLOW_OP_DESTROY(EllipsoidRaster);
        NV_FLOW_OP_DESTROY(EllipsoidRasterAllocate);
        NV_FLOW_OP_DESTROY(Pressure);
        NV_FLOW_OP_DESTROY(Vorticity);
        NV_FLOW_OP_DESTROY(Summary);
        NV_FLOW_OP_DESTROY(SummaryAllocate);
        NV_FLOW_OP_DESTROY(Shadow);
        NV_FLOW_OP_DESTROY(DebugVolume);
        NV_FLOW_OP_DESTROY(RayMarch);
        NV_FLOW_OP_DESTROY(RayMarchUpdateColormap);
        NV_FLOW_OP_DESTROY(RayMarchIsosurface);
        NV_FLOW_OP_DESTROY(RayMarchCopyTexture);

        ptr->sparseInterface.destroy(context, ptr->sparse);
        ptr->sparseInterface.destroy(context, ptr->sparseIsosurface);

        delete ptr;
    }

    void resetGrid(
        NvFlowContext* context,
        NvFlowGrid* grid,
        const NvFlowGridDesc* desc
    )
    {
        auto ptr = cast(grid);

        ptr->sparseInterface.reset(context, ptr->sparse, desc->maxLocations);
        ptr->sparseInterface.reset(context, ptr->sparseIsosurface, desc->maxLocationsIsosurface);
    }

    struct SimulateStepResources
    {
        NvFlowSparseTexture velocityFront;
        NvFlowSparseTexture densityFront;
        NvFlowSparseTexture coarseDensityFront;
        NvFlowSparseTexture voxelWeightFineFront;
        NvFlowSparseTexture voxelWeightCoarseFront;
    };

    void simulateStep(
        NvFlowContext* context,
        Grid* ptr,
        const NvFlowGridParamsDesc* paramsIn,
        float deltaTime,
        NvFlowSparseParams sparseParams,
        NvFlowBufferTransient* sparseBuffer,
        SimulateStepResources* resources,
        NvFlowEmitterPointFeedback emitterPointFeedback,
        NvFlowEmitterMeshFeedback emitterMeshFeedback,
        NvFlowEmitterNanoVdbFeedback emitterNanoVdbFeedback
    )
    {
        NV_FLOW_PROFILE_BEGIN(60, 15)
        NV_FLOW_PROFILE_TIMESTAMP("SimulateStepBegin")

        // For now, fetching only the latest
        NvFlowDatabaseSnapshot snapshot = {};
        if (paramsIn->snapshotCount > 0u)
        {
            snapshot = paramsIn->snapshots[paramsIn->snapshotCount - 1u].snapshot;
        }
        NV_FLOW_DATABASE_SNAPSHOT_FIND_TYPE_ARRAY(&snapshot, NvFlowGridEmitterSphereParams)
        NV_FLOW_DATABASE_SNAPSHOT_FIND_TYPE_ARRAY(&snapshot, NvFlowGridEmitterBoxParams)
        NV_FLOW_DATABASE_SNAPSHOT_FIND_TYPE_ARRAY(&snapshot, NvFlowGridEmitterPointParams)
        NV_FLOW_DATABASE_SNAPSHOT_FIND_TYPE_ARRAY(&snapshot, NvFlowGridEmitterMeshParams)
        NV_FLOW_DATABASE_SNAPSHOT_FIND_TYPE_ARRAY(&snapshot, NvFlowGridEmitterTextureParams)
        NV_FLOW_DATABASE_SNAPSHOT_FIND_TYPE_ARRAY(&snapshot, NvFlowGridEmitterNanoVdbParams)

        NV_FLOW_DATABASE_SNAPSHOT_FIND_TYPE_ARRAY(&snapshot, NvFlowPointsParams)

        NvFlowSparseTexture* advectionDensityOut = nullptr;
        NvFlowSparseTexture* voxelWeightFineOut = nullptr;

        NvFlowSparseTexture densityTemp = {};
        NvFlowSparseTexture_duplicate(&ptr->contextInterface, context, &densityTemp, &resources->densityFront);

        NvFlowAdvectionCombustionDensityPinsIn advectDensityPinsIn = {};
        NvFlowAdvectionCombustionDensityPinsOut advectDensityPinsOut = {};
        advectDensityPinsIn.contextInterface = &ptr->contextInterface;
        advectDensityPinsIn.context = context;
        advectDensityPinsIn.deltaTime = deltaTime;
        advectDensityPinsIn.params = ptr->advectionCombustionParams.data;
        advectDensityPinsIn.paramCount = ptr->advectionCombustionParams.size;
        advectDensityPinsIn.velocity = resources->velocityFront;
        advectDensityPinsIn.density = resources->densityFront;
        advectDensityPinsIn.voxelWeight = resources->voxelWeightFineFront;
        advectDensityPinsIn.densityTemp = densityTemp;
        //advectDensityPins.densityOut = &advectionDensityOut;
        advectionDensityOut = &advectDensityPinsOut.density;
        voxelWeightFineOut = &advectDensityPinsOut.voxelWeight;

        NvFlowAdvectionCombustionDensity_execute(&ptr->mAdvectionCombustionDensity, &advectDensityPinsIn, &advectDensityPinsOut);

        NV_FLOW_PROFILE_TIMESTAMP("AdvectionCombustionDensity")

        // sphere emit density
        {
            NvFlowEmitterSpherePinsIn pinsIn = {};
            NvFlowEmitterSpherePinsOut pinsOut = {};
            pinsIn.contextInterface = &ptr->contextInterface;
            pinsIn.context = context;
            pinsIn.deltaTime = deltaTime;
            pinsIn.densityParams = NvFlowGridEmitterSphereParams_elements;
            pinsIn.densityParamCount = NvFlowGridEmitterSphereParams_elementCount;
            pinsIn.value = *advectionDensityOut;
            pinsIn.valueTemp = densityTemp;
            pinsIn.velocity = resources->velocityFront;
            pinsOut.value = *advectionDensityOut;

            NvFlowEmitterSphere_execute(&ptr->mEmitterSphere, &pinsIn, &pinsOut);

            NV_FLOW_PROFILE_TIMESTAMP("EmitterSphereDensity")
        }

        // box emit density
        {
            NvFlowEmitterBoxPinsIn pinsIn = {};
            NvFlowEmitterBoxPinsOut pinsOut = {};
            pinsIn.contextInterface = &ptr->contextInterface;
            pinsIn.context = context;
            pinsIn.deltaTime = deltaTime;
            pinsIn.densityParams = NvFlowGridEmitterBoxParams_elements;
            pinsIn.densityParamCount = NvFlowGridEmitterBoxParams_elementCount;
            pinsIn.physicsCollisionLayers = ptr->physicsCollisionLayers.data;
            pinsIn.physicsCollisionLayerCount = ptr->physicsCollisionLayers.size;
            pinsIn.value = *advectionDensityOut;
            pinsIn.valueTemp = densityTemp;
            pinsOut.value = *advectionDensityOut;

            NvFlowEmitterBox_execute(&ptr->mEmitterBox, &pinsIn, &pinsOut);

            NV_FLOW_PROFILE_TIMESTAMP("EmitterBoxDensity")
        }

        // point emit density
        {
            NvFlowEmitterPointPinsIn pinsIn = {};
            NvFlowEmitterPointPinsOut pinsOut = {};
            pinsIn.contextInterface = &ptr->contextInterface;
            pinsIn.context = context;
            pinsIn.deltaTime = deltaTime;
            pinsIn.densityParams = NvFlowGridEmitterPointParams_elements;
            pinsIn.densityParamCount = NvFlowGridEmitterPointParams_elementCount;
            pinsIn.pointsParams = NvFlowPointsParams_elements;
            pinsIn.pointsParamCount = NvFlowPointsParams_elementCount;
            pinsIn.value = *advectionDensityOut;
            pinsIn.voxelWeight = *voxelWeightFineOut;
            pinsIn.coarseDensity = resources->coarseDensityFront;
            pinsIn.feedback = emitterPointFeedback;
            pinsOut.value = *advectionDensityOut;
            pinsOut.value = *voxelWeightFineOut;

            pinsIn.streamingVersion = ptr->streamingVersion;
            pinsIn.streamingFinishedVersion = ptr->streamingFinishedVersion;
            pinsIn.streamingPointsVersion = ptr->streamingPointsVersion;
            pinsIn.streamingNanoVdbVersion = ptr->streamingNanoVdbVersion;

            NvFlowEmitterPoint_execute(&ptr->mEmitterPoint, &pinsIn, &pinsOut);

            ptr->streamingVersion = pinsOut.streamingVersion;
            ptr->streamingFinishedVersion = pinsOut.streamingFinishedVersion;
            ptr->streamingPointsVersion = pinsOut.streamingPointsVersion;

            NV_FLOW_PROFILE_TIMESTAMP("EmitterPointDensity")
        }

        // mesh emit density
        {
            NvFlowEmitterMeshPinsIn pinsIn = {};
            NvFlowEmitterMeshPinsOut pinsOut = {};
            pinsIn.contextInterface = &ptr->contextInterface;
            pinsIn.context = context;
            pinsIn.deltaTime = deltaTime;
            pinsIn.densityParams = NvFlowGridEmitterMeshParams_elements;
            pinsIn.densityParamCount = NvFlowGridEmitterMeshParams_elementCount;
            pinsIn.physicsCollisionLayers = ptr->physicsCollisionLayers.data;
            pinsIn.physicsCollisionLayerCount = ptr->physicsCollisionLayers.size;
            pinsIn.value = *advectionDensityOut;
            pinsIn.valueTemp = densityTemp;
            pinsIn.feedback = emitterMeshFeedback;
            pinsOut.value = *advectionDensityOut;

            NvFlowEmitterMesh_execute(&ptr->mEmitterMesh, &pinsIn, &pinsOut);

            NV_FLOW_PROFILE_TIMESTAMP("EmitterMeshDensity")
        }

        // texture emit density
        {
            NvFlowEmitterTexturePinsIn pinsIn = {};
            NvFlowEmitterTexturePinsOut pinsOut = {};
            pinsIn.contextInterface = &ptr->contextInterface;
            pinsIn.context = context;
            pinsIn.deltaTime = deltaTime;
            pinsIn.densityParams = NvFlowGridEmitterTextureParams_elements;
            pinsIn.densityParamCount = NvFlowGridEmitterTextureParams_elementCount;
            pinsIn.value = *advectionDensityOut;
            pinsIn.valueTemp = densityTemp;
            pinsOut.value = *advectionDensityOut;

            NvFlowEmitterTexture_execute(&ptr->mEmitterTexture, &pinsIn, &pinsOut);

            NV_FLOW_PROFILE_TIMESTAMP("EmitterTextureDensity")
        }

        // nanoVdb emit density
        {
            NvFlowEmitterNanoVdbPinsIn pinsIn = {};
            NvFlowEmitterNanoVdbPinsOut pinsOut = {};
            pinsIn.contextInterface = &ptr->contextInterface;
            pinsIn.context = context;
            pinsIn.deltaTime = deltaTime;
            pinsIn.densityParams = NvFlowGridEmitterNanoVdbParams_elements;
            pinsIn.densityParamCount = NvFlowGridEmitterNanoVdbParams_elementCount;
            pinsIn.value = *advectionDensityOut;
            pinsIn.voxelWeight = *voxelWeightFineOut;
            pinsIn.coarseDensity = resources->coarseDensityFront;
            pinsIn.feedback = emitterNanoVdbFeedback;
            pinsOut.value = *advectionDensityOut;
            pinsOut.value = *voxelWeightFineOut;

            pinsIn.streamingVersion = ptr->streamingVersion;
            pinsIn.streamingFinishedVersion = ptr->streamingFinishedVersion;
            pinsIn.streamingPointsVersion = ptr->streamingPointsVersion;
            pinsIn.streamingNanoVdbVersion = ptr->streamingNanoVdbVersion;

            NvFlowEmitterNanoVdb_execute(&ptr->mEmitterNanoVdb, &pinsIn, &pinsOut);

            ptr->streamingVersion = pinsOut.streamingVersion;
            ptr->streamingFinishedVersion = pinsOut.streamingFinishedVersion;
            ptr->streamingNanoVdbVersion = pinsOut.streamingNanoVdbVersion;

            NV_FLOW_PROFILE_TIMESTAMP("EmitterNanoVdbDensity")
        }

        NvFlowSparseTexture* advectionVelocityOut = nullptr;
        NvFlowSparseTexture* advectionDensityCoarseOut = nullptr;
        NvFlowSparseTexture* pressureOut = nullptr;
        NvFlowSparseTexture* voxelWeightCoarseOut = nullptr;

        // TODO: support subSteps per layer
        NvFlowUint maxVelocitySubSteps = 1u;
        for (NvFlowUint layerParamIdx = 0u; layerParamIdx < ptr->simulateParams.size; layerParamIdx++)
        {
            if (sparseParams.layers[layerParamIdx].numLocations > 0u)
            {
                if (ptr->simulateParams[layerParamIdx]->velocitySubSteps > maxVelocitySubSteps)
                {
                    maxVelocitySubSteps = ptr->simulateParams[layerParamIdx]->velocitySubSteps;
                }
            }
        }

        NvFlowSparseTexture velocityTemp = {};
        NvFlowSparseTexture_duplicate(&ptr->contextInterface, context, &velocityTemp, &resources->velocityFront);

        for (NvFlowUint velocitySubStepIdx = 0u; velocitySubStepIdx < maxVelocitySubSteps; velocitySubStepIdx++)
        {
            float subStepDeltaTime = deltaTime / float(maxVelocitySubSteps);

            NvFlowAdvectionCombustionVelocityPinsIn advectVelocityPinsIn = {};
            NvFlowAdvectionCombustionVelocityPinsOut advectVelocityPinsOut = {};
            advectVelocityPinsIn.contextInterface = &ptr->contextInterface;
            advectVelocityPinsIn.context = context;
            advectVelocityPinsIn.deltaTime = subStepDeltaTime;
            advectVelocityPinsIn.params = ptr->advectionCombustionParams.data;
            advectVelocityPinsIn.paramCount = ptr->advectionCombustionParams.size;
            advectVelocityPinsIn.velocity = resources->velocityFront;
            advectVelocityPinsIn.voxelWeight = resources->voxelWeightCoarseFront;
            advectVelocityPinsIn.velocityTemp = velocityTemp;
            advectVelocityPinsIn.density = *advectionDensityOut;
            advectVelocityPinsIn.densityCoarse = resources->coarseDensityFront;
            //advectVelocityPins.velocityOut = &advectionVelocityOut;
            //advectVelocityPins.densityCoarseOut = &advectionDensityCoarseOut;
            advectionVelocityOut = &advectVelocityPinsOut.velocity;
            voxelWeightCoarseOut = &advectVelocityPinsOut.voxelWeight;
            advectionDensityCoarseOut = &advectVelocityPinsOut.densityCoarse;

            NvFlowAdvectionCombustionVelocity_execute(&ptr->mAdvectionCombustionVelocity, &advectVelocityPinsIn, &advectVelocityPinsOut);

            NV_FLOW_PROFILE_TIMESTAMP("AdvectionCombustionVelocity")

            // sphere emit velocity
            {
                NvFlowEmitterSpherePinsIn pinsIn = {};
                NvFlowEmitterSpherePinsOut pinsOut = {};
                pinsIn.contextInterface = &ptr->contextInterface;
                pinsIn.context = context;
                pinsIn.deltaTime = subStepDeltaTime;
                pinsIn.velocityParams = NvFlowGridEmitterSphereParams_elements;
                pinsIn.velocityParamCount = NvFlowGridEmitterSphereParams_elementCount;
                pinsIn.value = *advectionVelocityOut;
                pinsIn.valueTemp = velocityTemp;
                pinsIn.velocity = resources->velocityFront;
                pinsIn.isPostPressure = NV_FLOW_FALSE;
                pinsOut.value = *advectionVelocityOut;

                NvFlowEmitterSphere_execute(&ptr->mEmitterSphere, &pinsIn, &pinsOut);

                NV_FLOW_PROFILE_TIMESTAMP("EmitterSphereVelocity")
            }

            // box emit velocity
            {
                NvFlowEmitterBoxPinsIn pinsIn = {};
                NvFlowEmitterBoxPinsOut pinsOut = {};
                pinsIn.contextInterface = &ptr->contextInterface;
                pinsIn.context = context;
                pinsIn.deltaTime = subStepDeltaTime;
                pinsIn.velocityParams = NvFlowGridEmitterBoxParams_elements;
                pinsIn.velocityParamCount = NvFlowGridEmitterBoxParams_elementCount;
                pinsIn.physicsCollisionLayers = ptr->physicsCollisionLayers.data;
                pinsIn.physicsCollisionLayerCount = ptr->physicsCollisionLayers.size;
                pinsIn.value = *advectionVelocityOut;
                pinsIn.valueTemp = velocityTemp;
                pinsIn.isPostPressure = NV_FLOW_FALSE;
                pinsOut.value = *advectionVelocityOut;

                NvFlowEmitterBox_execute(&ptr->mEmitterBox, &pinsIn, &pinsOut);

                NV_FLOW_PROFILE_TIMESTAMP("EmitterBoxVelocity")
            }

            // point emit velocity
            {
                NvFlowEmitterPointPinsIn pinsIn = {};
                NvFlowEmitterPointPinsOut pinsOut = {};
                pinsIn.contextInterface = &ptr->contextInterface;
                pinsIn.context = context;
                pinsIn.deltaTime = subStepDeltaTime;
                pinsIn.velocityParams = NvFlowGridEmitterPointParams_elements;
                pinsIn.velocityParamCount = NvFlowGridEmitterPointParams_elementCount;
                pinsIn.pointsParams = NvFlowPointsParams_elements;
                pinsIn.pointsParamCount = NvFlowPointsParams_elementCount;
                pinsIn.value = *advectionVelocityOut;
                pinsIn.voxelWeight = *voxelWeightCoarseOut;
                pinsIn.isPostPressure = NV_FLOW_FALSE;
                pinsIn.feedback = emitterPointFeedback;
                pinsOut.value = *advectionVelocityOut;
                pinsOut.voxelWeight = *voxelWeightCoarseOut;

                pinsIn.streamingVersion = ptr->streamingVersion;
                pinsIn.streamingFinishedVersion = ptr->streamingFinishedVersion;
                pinsIn.streamingPointsVersion = ptr->streamingPointsVersion;
                pinsIn.streamingNanoVdbVersion = ptr->streamingNanoVdbVersion;

                NvFlowEmitterPoint_execute(&ptr->mEmitterPoint, &pinsIn, &pinsOut);

                ptr->streamingVersion = pinsOut.streamingVersion;
                ptr->streamingFinishedVersion = pinsOut.streamingFinishedVersion;
                ptr->streamingPointsVersion = pinsOut.streamingPointsVersion;

                NV_FLOW_PROFILE_TIMESTAMP("EmitterPointVelocity")
            }

            // mesh emit velocity
            {
                NvFlowEmitterMeshPinsIn pinsIn = {};
                NvFlowEmitterMeshPinsOut pinsOut = {};
                pinsIn.contextInterface = &ptr->contextInterface;
                pinsIn.context = context;
                pinsIn.deltaTime = subStepDeltaTime;
                pinsIn.velocityParams = NvFlowGridEmitterMeshParams_elements;
                pinsIn.velocityParamCount = NvFlowGridEmitterMeshParams_elementCount;
                pinsIn.physicsCollisionLayers = ptr->physicsCollisionLayers.data;
                pinsIn.physicsCollisionLayerCount = ptr->physicsCollisionLayers.size;
                pinsIn.value = *advectionVelocityOut;
                pinsIn.valueTemp = velocityTemp;
                pinsIn.isPostPressure = NV_FLOW_FALSE;
                pinsIn.feedback = emitterMeshFeedback;
                pinsOut.value = *advectionVelocityOut;

                NvFlowEmitterMesh_execute(&ptr->mEmitterMesh, &pinsIn, &pinsOut);

                NV_FLOW_PROFILE_TIMESTAMP("EmitterMeshVelocity")
            }

            // texture emit velocity
            {
                NvFlowEmitterTexturePinsIn pinsIn = {};
                NvFlowEmitterTexturePinsOut pinsOut = {};
                pinsIn.contextInterface = &ptr->contextInterface;
                pinsIn.context = context;
                pinsIn.deltaTime = subStepDeltaTime;
                pinsIn.velocityParams = NvFlowGridEmitterTextureParams_elements;
                pinsIn.velocityParamCount = NvFlowGridEmitterTextureParams_elementCount;
                pinsIn.value = *advectionVelocityOut;
                pinsIn.valueTemp = velocityTemp;
                pinsIn.isPostPressure = NV_FLOW_FALSE;
                pinsOut.value = *advectionVelocityOut;

                NvFlowEmitterTexture_execute(&ptr->mEmitterTexture, &pinsIn, &pinsOut);

                NV_FLOW_PROFILE_TIMESTAMP("EmitterTextureVelocity")
            }

            // nanoVdb emit velocity
            {
                NvFlowEmitterNanoVdbPinsIn pinsIn = {};
                NvFlowEmitterNanoVdbPinsOut pinsOut = {};
                pinsIn.contextInterface = &ptr->contextInterface;
                pinsIn.context = context;
                pinsIn.deltaTime = subStepDeltaTime;
                pinsIn.velocityParams = NvFlowGridEmitterNanoVdbParams_elements;
                pinsIn.velocityParamCount = NvFlowGridEmitterNanoVdbParams_elementCount;
                pinsIn.value = *advectionVelocityOut;
                pinsIn.voxelWeight = *voxelWeightCoarseOut;
                pinsIn.isPostPressure = NV_FLOW_FALSE;
                pinsIn.feedback = emitterNanoVdbFeedback;
                pinsOut.value = *advectionVelocityOut;
                pinsOut.voxelWeight = *voxelWeightCoarseOut;

                pinsIn.streamingVersion = ptr->streamingVersion;
                pinsIn.streamingFinishedVersion = ptr->streamingFinishedVersion;
                pinsIn.streamingPointsVersion = ptr->streamingPointsVersion;
                pinsIn.streamingNanoVdbVersion = ptr->streamingNanoVdbVersion;

                NvFlowEmitterNanoVdb_execute(&ptr->mEmitterNanoVdb, &pinsIn, &pinsOut);

                ptr->streamingVersion = pinsOut.streamingVersion;
                ptr->streamingFinishedVersion = pinsOut.streamingFinishedVersion;
                ptr->streamingNanoVdbVersion = pinsOut.streamingNanoVdbVersion;

                NV_FLOW_PROFILE_TIMESTAMP("EmitterNanoVdbVelocity")
            }

            NvFlowSparseTexture* vorticityOut = nullptr;

            NvFlowVorticityPinsIn vorticityPinsIn = {};
            NvFlowVorticityPinsOut vorticityPinsOut = {};
            vorticityPinsIn.contextInterface = &ptr->contextInterface;
            vorticityPinsIn.context = context;
            vorticityPinsIn.deltaTime = subStepDeltaTime;
            vorticityPinsIn.params = ptr->vorticityParams.data;
            vorticityPinsIn.paramCount = ptr->vorticityParams.size;
            vorticityPinsIn.velocity = *advectionVelocityOut;
            vorticityPinsIn.coarseDensity = *advectionDensityCoarseOut;
            //vorticityPins.velocityOut = &vorticityOut;
            vorticityOut = &vorticityPinsOut.velocity;

            NvFlowVorticity_execute(&ptr->mVorticity, &vorticityPinsIn, &vorticityPinsOut);

            NV_FLOW_PROFILE_TIMESTAMP("Vorticity")

            NvFlowPressurePinsIn pressurePinsIn = {};
            NvFlowPressurePinsOut pressurePinsOut = {};
            pressurePinsIn.contextInterface = &ptr->contextInterface;
            pressurePinsIn.context = context;
            pressurePinsIn.params = ptr->pressureParams.data;
            pressurePinsIn.paramCount = ptr->pressureParams.size;
            pressurePinsIn.velocity = *vorticityOut;
            //pressurePins.velocityOut = &pressureOut;
            pressureOut = &pressurePinsOut.velocity;

            NvFlowPressure_execute(&ptr->mPressure, &pressurePinsIn, &pressurePinsOut);

            NV_FLOW_PROFILE_TIMESTAMP("Pressure")

            // sphere emit velocity
            {
                NvFlowEmitterSpherePinsIn pinsIn = {};
                NvFlowEmitterSpherePinsOut pinsOut = {};
                pinsIn.contextInterface = &ptr->contextInterface;
                pinsIn.context = context;
                pinsIn.deltaTime = subStepDeltaTime;
                pinsIn.velocityParams = NvFlowGridEmitterSphereParams_elements;
                pinsIn.velocityParamCount = NvFlowGridEmitterSphereParams_elementCount;
                pinsIn.value = *pressureOut;
                pinsIn.velocity = resources->velocityFront;
                pinsIn.isPostPressure = NV_FLOW_TRUE;
                pinsOut.value = *pressureOut;

                NvFlowEmitterSphere_execute(&ptr->mEmitterSphere, &pinsIn, &pinsOut);

                NV_FLOW_PROFILE_TIMESTAMP("EmitterSpherePostPressure")
            }

            // box emit velocity
            {
                NvFlowEmitterBoxPinsIn pinsIn = {};
                NvFlowEmitterBoxPinsOut pinsOut = {};
                pinsIn.contextInterface = &ptr->contextInterface;
                pinsIn.context = context;
                pinsIn.deltaTime = subStepDeltaTime;
                pinsIn.velocityParams = NvFlowGridEmitterBoxParams_elements;
                pinsIn.velocityParamCount = NvFlowGridEmitterBoxParams_elementCount;
                pinsIn.physicsCollisionLayers = ptr->physicsCollisionLayers.data;
                pinsIn.physicsCollisionLayerCount = ptr->physicsCollisionLayers.size;
                pinsIn.value = *pressureOut;
                pinsIn.isPostPressure = NV_FLOW_TRUE;
                pinsOut.value = *pressureOut;

                NvFlowEmitterBox_execute(&ptr->mEmitterBox, &pinsIn, &pinsOut);

                NV_FLOW_PROFILE_TIMESTAMP("EmitterBoxPostPressure")
            }

            // point emit velocity
            {
                NvFlowEmitterPointPinsIn pinsIn = {};
                NvFlowEmitterPointPinsOut pinsOut = {};
                pinsIn.contextInterface = &ptr->contextInterface;
                pinsIn.context = context;
                pinsIn.deltaTime = subStepDeltaTime;
                pinsIn.velocityParams = NvFlowGridEmitterPointParams_elements;
                pinsIn.velocityParamCount = NvFlowGridEmitterPointParams_elementCount;
                pinsIn.pointsParams = NvFlowPointsParams_elements;
                pinsIn.pointsParamCount = NvFlowPointsParams_elementCount;
                pinsIn.value = *pressureOut;
                pinsIn.voxelWeight = *voxelWeightCoarseOut;
                pinsIn.isPostPressure = NV_FLOW_TRUE;
                pinsIn.feedback = emitterPointFeedback;
                pinsOut.value = *pressureOut;
                pinsOut.voxelWeight = *voxelWeightCoarseOut;

                pinsIn.streamingVersion = ptr->streamingVersion;
                pinsIn.streamingFinishedVersion = ptr->streamingFinishedVersion;
                pinsIn.streamingPointsVersion = ptr->streamingPointsVersion;
                pinsIn.streamingNanoVdbVersion = ptr->streamingNanoVdbVersion;

                NvFlowEmitterPoint_execute(&ptr->mEmitterPoint, &pinsIn, &pinsOut);

                ptr->streamingVersion = pinsOut.streamingVersion;
                ptr->streamingFinishedVersion = pinsOut.streamingFinishedVersion;
                ptr->streamingPointsVersion = pinsOut.streamingPointsVersion;

                NV_FLOW_PROFILE_TIMESTAMP("EmitterPointPostPressure")
            }

            // mesh emit velocity
            {
                NvFlowEmitterMeshPinsIn pinsIn = {};
                NvFlowEmitterMeshPinsOut pinsOut = {};
                pinsIn.contextInterface = &ptr->contextInterface;
                pinsIn.context = context;
                pinsIn.deltaTime = subStepDeltaTime;
                pinsIn.velocityParams = NvFlowGridEmitterMeshParams_elements;
                pinsIn.velocityParamCount = NvFlowGridEmitterMeshParams_elementCount;
                pinsIn.physicsCollisionLayers = ptr->physicsCollisionLayers.data;
                pinsIn.physicsCollisionLayerCount = ptr->physicsCollisionLayers.size;
                pinsIn.value = *pressureOut;
                pinsIn.isPostPressure = NV_FLOW_TRUE;
                pinsIn.feedback = emitterMeshFeedback;
                pinsOut.value = *pressureOut;

                NvFlowEmitterMesh_execute(&ptr->mEmitterMesh, &pinsIn, &pinsOut);

                NV_FLOW_PROFILE_TIMESTAMP("EmitterMeshPostPressure")
            }

            // texture emit velocity
            {
                NvFlowEmitterTexturePinsIn pinsIn = {};
                NvFlowEmitterTexturePinsOut pinsOut = {};
                pinsIn.contextInterface = &ptr->contextInterface;
                pinsIn.context = context;
                pinsIn.deltaTime = subStepDeltaTime;
                pinsIn.velocityParams = NvFlowGridEmitterTextureParams_elements;
                pinsIn.velocityParamCount = NvFlowGridEmitterTextureParams_elementCount;
                pinsIn.value = *pressureOut;
                pinsIn.isPostPressure = NV_FLOW_TRUE;
                pinsOut.value = *pressureOut;

                NvFlowEmitterTexture_execute(&ptr->mEmitterTexture, &pinsIn, &pinsOut);

                NV_FLOW_PROFILE_TIMESTAMP("EmitterTexturePostPressure")
            }

            // nanoVdb emit velocity
            {
                NvFlowEmitterNanoVdbPinsIn pinsIn = {};
                NvFlowEmitterNanoVdbPinsOut pinsOut = {};
                pinsIn.contextInterface = &ptr->contextInterface;
                pinsIn.context = context;
                pinsIn.deltaTime = subStepDeltaTime;
                pinsIn.velocityParams = NvFlowGridEmitterNanoVdbParams_elements;
                pinsIn.velocityParamCount = NvFlowGridEmitterNanoVdbParams_elementCount;
                pinsIn.value = *pressureOut;
                pinsIn.voxelWeight = *voxelWeightCoarseOut;
                pinsIn.isPostPressure = NV_FLOW_TRUE;
                pinsIn.feedback = emitterNanoVdbFeedback;
                pinsOut.value = *pressureOut;
                pinsOut.voxelWeight = *voxelWeightCoarseOut;

                pinsIn.streamingVersion = ptr->streamingVersion;
                pinsIn.streamingFinishedVersion = ptr->streamingFinishedVersion;
                pinsIn.streamingPointsVersion = ptr->streamingPointsVersion;
                pinsIn.streamingNanoVdbVersion = ptr->streamingNanoVdbVersion;

                NvFlowEmitterNanoVdb_execute(&ptr->mEmitterNanoVdb, &pinsIn, &pinsOut);

                ptr->streamingVersion = pinsOut.streamingVersion;
                ptr->streamingNanoVdbVersion = pinsOut.streamingNanoVdbVersion;

                NV_FLOW_PROFILE_TIMESTAMP("EmitterNanoVdbPostPressure")
            }

            if (maxVelocitySubSteps >= 2u)
            {
                resources->velocityFront = *pressureOut;
            }
        }

        resources->velocityFront = *pressureOut;
        resources->densityFront = *advectionDensityOut;
        resources->coarseDensityFront = *advectionDensityCoarseOut;
        resources->voxelWeightFineFront = *voxelWeightFineOut;
        resources->voxelWeightCoarseFront = *voxelWeightCoarseOut;

        NV_FLOW_PROFILE_TIMESTAMP("SimulateStepEnd")
        NV_FLOW_PROFILE_FLUSH("SimulateStep", ptr->contextInterface.getLogPrint(context))
    }

    void simulate(
        NvFlowContext* context,
        NvFlowGrid* grid,
        const NvFlowGridParamsDesc* paramsIn,
        NvFlowBool32 globalForceClearIn
    )
    {
        auto ptr = cast(grid);

        NV_FLOW_PROFILE_BEGIN(60, 0)
        NV_FLOW_PROFILE_TIMESTAMP("SimulateBegin")

        // For now, fetching only the latest
        NvFlowDatabaseSnapshot snapshot = {};
        if (paramsIn->snapshotCount > 0u)
        {
            snapshot = paramsIn->snapshots[paramsIn->snapshotCount - 1u].snapshot;
        }
        NV_FLOW_DATABASE_SNAPSHOT_FIND_TYPE_ARRAY(&snapshot, NvFlowGridSimulateLayerParams)
        NV_FLOW_DATABASE_SNAPSHOT_FIND_TYPE_ARRAY(&snapshot, NvFlowGridOffscreenLayerParams)
        NV_FLOW_DATABASE_SNAPSHOT_FIND_TYPE_ARRAY(&snapshot, NvFlowGridRenderLayerParams)
        NV_FLOW_DATABASE_SNAPSHOT_FIND_TYPE_ARRAY(&snapshot, NvFlowGridEmitterSphereParams)
        NV_FLOW_DATABASE_SNAPSHOT_FIND_TYPE_ARRAY(&snapshot, NvFlowGridEmitterBoxParams)
        NV_FLOW_DATABASE_SNAPSHOT_FIND_TYPE_ARRAY(&snapshot, NvFlowGridEmitterPointParams)
        NV_FLOW_DATABASE_SNAPSHOT_FIND_TYPE_ARRAY(&snapshot, NvFlowGridEmitterMeshParams)
        NV_FLOW_DATABASE_SNAPSHOT_FIND_TYPE_ARRAY(&snapshot, NvFlowGridEmitterTextureParams)
        NV_FLOW_DATABASE_SNAPSHOT_FIND_TYPE_ARRAY(&snapshot, NvFlowGridEmitterNanoVdbParams)

        NV_FLOW_DATABASE_SNAPSHOT_FIND_TYPE_ARRAY(&snapshot, NvFlowPointsParams)
        NV_FLOW_DATABASE_SNAPSHOT_FIND_TYPE_ARRAY(&snapshot, NvFlowVolumeParams)
        NV_FLOW_DATABASE_SNAPSHOT_FIND_TYPE_ARRAY(&snapshot, NvFlowNanoVdbAssetParams)

        NV_FLOW_DATABASE_SNAPSHOT_FIND_TYPE_ARRAY(&snapshot, NvFlowGridReadbackParams)

        ptr->physicsCollisionLayers.size = 0u;
        for (NvFlowUint64 idx = 0u; idx < NvFlowGridSimulateLayerParams_elementCount; idx++)
        {
            if (NvFlowGridSimulateLayerParams_elements[idx]->physicsCollisionEnabled)
            {
                ptr->physicsCollisionLayers.pushBack(NvFlowGridSimulateLayerParams_elements[idx]->layer);
            }
        }

        // Accumulate time in snapshots
        float deltaTime = 0.f;
        NvFlowBool32 globalForceClear = globalForceClearIn;
        for (NvFlowUint64 snapshotIdx = 0u; snapshotIdx < paramsIn->snapshotCount; snapshotIdx++)
        {
            deltaTime += paramsIn->snapshots[snapshotIdx].deltaTime;
            if (paramsIn->snapshots[snapshotIdx].globalForceClear)
            {
                globalForceClear = NV_FLOW_TRUE;
            }
        }

        double absoluteSimTime = 0.0;
        if (paramsIn->snapshotCount > 0u)
        {
            absoluteSimTime = paramsIn->snapshots[paramsIn->snapshotCount - 1u].absoluteSimTime;
        }

        // vote on simulation if paused
        NvFlowBool32 canSkipSimulation = (deltaTime == 0.f);
        if (globalForceClear)
        {
            canSkipSimulation = NV_FLOW_FALSE;
        }
        for (NvFlowUint readIdx = 0u; readIdx < NvFlowGridSimulateLayerParams_elementCount; readIdx++)
        {
            if (NvFlowGridSimulateLayerParams_elements[readIdx]->simulateWhenPaused || NvFlowGridSimulateLayerParams_elements[readIdx]->forceClear)
            {
                canSkipSimulation = NV_FLOW_FALSE;
                break;
            }
        }
        if (canSkipSimulation && NvFlowGridSimulateLayerParams_elementCount == 0u)
        {
            NvFlowSparseParams sparseParams = {};
            ptr->sparseInterface.getParams(ptr->sparse, &sparseParams);
            if (sparseParams.layerCount > 0u)
            {
                canSkipSimulation = NV_FLOW_FALSE;
            }
        }
        if (canSkipSimulation)
        {
            return;
        }

        // free shadow, since it will need to be regenerated anyways
        NvFlowTextureVariable_set(context, &ptr->shadowVariable, nullptr, eNvFlowFormat_unknown);

        // generate simulate params array that avoids duplicates
        ptr->simulateParams.size = 0u;
        ptr->simulateParamsLevels.size = 0u;
        ptr->simulateParamsRootLevel.size = 0u;
        for (NvFlowUint readIdx = 0u; readIdx < NvFlowGridSimulateLayerParams_elementCount; readIdx++)
        {
            // level support
            int levelCount = 1;
            if (NvFlowGridSimulateLayerParams_elements[readIdx]->levelCount > 0)
            {
                levelCount = NvFlowGridSimulateLayerParams_elements[readIdx]->levelCount;
            }
            for (int levelIdx = 0; levelIdx < levelCount; levelIdx++)
            {
                NvFlowUint writeIdx = 0u;
                for (; writeIdx < ptr->simulateParams.size; writeIdx++)
                {
                    int ptr_sim_layerAndLevel = NvFlow_packLayerAndLevel(
                        ptr->simulateParams[writeIdx]->layer,
                        ptr->simulateParams[writeIdx]->level);
                    int sim_layerAndLevel = NvFlow_packLayerAndLevel(
                        NvFlowGridSimulateLayerParams_elements[readIdx]->layer,
                        NvFlowGridSimulateLayerParams_elements[readIdx]->level + levelIdx);
                    if (ptr_sim_layerAndLevel == sim_layerAndLevel)
                    {
                        if (NvFlowGridSimulateLayerParams_elements[readIdx]->level >= ptr->simulateParamsRootLevel[writeIdx])
                        {
                            ptr->simulateParamsRootLevel[writeIdx] = NvFlowGridSimulateLayerParams_elements[readIdx]->level;
                            ptr->simulateParams[writeIdx] = NvFlowGridSimulateLayerParams_elements[readIdx];
                        }
                        break;
                    }
                }
                if (writeIdx == ptr->simulateParams.size)
                {
                    const NvFlowGridSimulateLayerParams* simParams = NvFlowGridSimulateLayerParams_elements[readIdx];
                    if (levelIdx > 0u)
                    {
                        NvFlowGridSimulateLayerParams* simParamsW = ptr->simulateParamsLevels.allocateBackPointer();
                        *simParamsW = *simParams;
                        simParamsW->level += levelIdx;
                        simParamsW->levelCount -= levelIdx;
                        simParamsW->densityCellSize *= powf(simParams->levelCellSizeMultiplier, float(levelIdx));
                        simParams = simParamsW;
                    }
                    ptr->simulateParams.pushBack(simParams);
                    ptr->simulateParamsRootLevel.pushBack(NvFlowGridSimulateLayerParams_elements[readIdx]->level);
                }
            }
        }

        // generate layerParamIdx aligned lists for operators that need them
        ptr->summaryAllocateParams.reserve(ptr->simulateParams.size);
        ptr->summaryAllocateParams.size = ptr->simulateParams.size;
        for (NvFlowUint layerParamIdx = 0u; layerParamIdx < ptr->simulateParams.size; layerParamIdx++)
        {
            ptr->summaryAllocateParams[layerParamIdx] = &ptr->simulateParams[layerParamIdx]->summaryAllocate;
        }

        ptr->nanoVdbExportParams.reserve(ptr->simulateParams.size);
        ptr->nanoVdbExportParams.size = ptr->simulateParams.size;
        for (NvFlowUint layerParamIdx = 0u; layerParamIdx < ptr->simulateParams.size; layerParamIdx++)
        {
            ptr->nanoVdbExportParams[layerParamIdx] = &ptr->simulateParams[layerParamIdx]->nanoVdbExport;
        }

        ptr->advectionCombustionParams.reserve(ptr->simulateParams.size);
        ptr->advectionCombustionParams.size = ptr->simulateParams.size;
        for (NvFlowUint layerParamIdx = 0u; layerParamIdx < ptr->simulateParams.size; layerParamIdx++)
        {
            ptr->advectionCombustionParams[layerParamIdx] = &ptr->simulateParams[layerParamIdx]->advection;
        }

        ptr->vorticityParams.reserve(ptr->simulateParams.size);
        ptr->vorticityParams.size = ptr->simulateParams.size;
        for (NvFlowUint layerParamIdx = 0u; layerParamIdx < ptr->simulateParams.size; layerParamIdx++)
        {
            ptr->vorticityParams[layerParamIdx] = &ptr->simulateParams[layerParamIdx]->vorticity;
        }

        ptr->pressureParams.reserve(ptr->simulateParams.size);
        ptr->pressureParams.size = ptr->simulateParams.size;
        for (NvFlowUint layerParamIdx = 0u; layerParamIdx < ptr->simulateParams.size; layerParamIdx++)
        {
            ptr->pressureParams[layerParamIdx] = &ptr->simulateParams[layerParamIdx]->pressure;
        }

        // use small blocks if anyone requests them
        NvFlowBool32 useSmallBlocks = NV_FLOW_FALSE;
        for (NvFlowUint layerParamIdx = 0u; layerParamIdx < ptr->simulateParams.size; layerParamIdx++)
        {
            if (ptr->simulateParams[layerParamIdx]->enableSmallBlocks)
            {
                useSmallBlocks = NV_FLOW_TRUE;
                break;
            }
        }

        // Standard block size
        NvFlowUint3 baseBlockDimBits = { 5u, 4u, 4u };
        if (useSmallBlocks)
        {
            baseBlockDimBits = NvFlowUint3{ 4u, 3u, 3u };
        }
        NvFlowUint3 baseBlockDim = {
            1u << baseBlockDimBits.x,
            1u << baseBlockDimBits.y,
            1u << baseBlockDimBits.z
        };

        // update layers
        NvFlowBool32 allForceClear = NV_FLOW_TRUE;
        ptr->updateLayerParams.reserve(ptr->simulateParams.size);
        ptr->updateLayerParams.size = ptr->simulateParams.size;
        for (NvFlowUint layerParamIdx = 0u; layerParamIdx < ptr->simulateParams.size; layerParamIdx++)
        {
            NvFlowFloat3 blockSizeWorld = NvFlowFloat3{
                float(baseBlockDim.x) * fmaxf(0.f, ptr->simulateParams[layerParamIdx]->densityCellSize),
                float(baseBlockDim.y) * fmaxf(0.f, ptr->simulateParams[layerParamIdx]->densityCellSize),
                float(baseBlockDim.z) * fmaxf(0.f, ptr->simulateParams[layerParamIdx]->densityCellSize)
            };
            NvFlowBool32 isZero = ptr->simulateParams[layerParamIdx]->densityCellSize <= 0.f;

            ptr->updateLayerParams[layerParamIdx].blockSizeWorld = blockSizeWorld;
            ptr->updateLayerParams[layerParamIdx].layer = ptr->simulateParams[layerParamIdx]->layer;
            ptr->updateLayerParams[layerParamIdx].level = ptr->simulateParams[layerParamIdx]->level;
            ptr->updateLayerParams[layerParamIdx].forceClear = (ptr->simulateParams[layerParamIdx]->forceClear || globalForceClear || isZero);
            ptr->updateLayerParams[layerParamIdx].forceDisableEmitters = ptr->simulateParams[layerParamIdx]->forceDisableEmitters;
            ptr->updateLayerParams[layerParamIdx].forceDisableCoreSimulation = ptr->simulateParams[layerParamIdx]->forceDisableCoreSimulation;
            ptr->updateLayerParams[layerParamIdx].clearOnRescale = ptr->simulateParams[layerParamIdx]->clearOnRescale;
            ptr->updateLayerParams[layerParamIdx].densityCellSizeNonAuto = isZero ? 0.f : ptr->simulateParams[layerParamIdx]->densityCellSize;

            if (!(ptr->simulateParams[layerParamIdx]->forceClear || globalForceClear))
            {
                allForceClear = NV_FLOW_FALSE;
            }
        }

        // allows multiple attempts to updateLayers/updateLocations to be tracked properly
        ptr->updateId++;

        NvFlowUint maxMinBlockLifetime = 0u;
        for (NvFlowUint layerParamIdx = 0u; layerParamIdx < ptr->simulateParams.size; layerParamIdx++)
        {
            NvFlowUint blockMinLifetime = ptr->simulateParams[layerParamIdx]->blockMinLifetime;
            if (ptr->updateLayerParams[layerParamIdx].forceClear || allForceClear)
            {
                blockMinLifetime = 0u;
            }
            if (blockMinLifetime > maxMinBlockLifetime)
            {
                maxMinBlockLifetime = blockMinLifetime;
            }
        }

        bool autoScaleSearchRefine = false;
        NvFlowUint autoScaleSearchWidth = 0u;
        NvFlowBool32 anyAutoCellSizeEnabled = NV_FLOW_FALSE;
        for (NvFlowUint64 layerParamIdx = 0u; layerParamIdx < ptr->simulateParams.size; layerParamIdx++)
        {
            if (ptr->simulateParams[layerParamIdx]->autoCellSize)
            {
                anyAutoCellSizeEnabled = NV_FLOW_TRUE;
                break;
            }
        }
        if (!anyAutoCellSizeEnabled)
        {
            ptr->autoRescaleLevel = 0u;
        }

        NvFlowBool32 cellSizeChanged = NV_FLOW_FALSE;
        for (NvFlowUint64 layerParamIdx = 0u; layerParamIdx < ptr->simulateParams.size; layerParamIdx++)
        {
            int newLayerAndLevel = NvFlow_packLayerAndLevel(ptr->simulateParams[layerParamIdx]->layer, ptr->simulateParams[layerParamIdx]->level);
            float newCellSize = ptr->simulateParams[layerParamIdx]->densityCellSize;
            float oldCellSize = 0.f;
            for (NvFlowUint64 historyIdx = 0u; historyIdx < ptr->autoRescaleOldLayerAndLevels.size; historyIdx++)
            {
                NvFlowUint64 lookupIdx = (historyIdx + layerParamIdx) % ptr->autoRescaleOldLayerAndLevels.size;
                if (ptr->autoRescaleOldLayerAndLevels[lookupIdx] == newLayerAndLevel)
                {
                    oldCellSize = ptr->autoRescaleOldDensityCellSizes[lookupIdx];
                    break;
                }
            }
            if (newCellSize != oldCellSize)
            {
                cellSizeChanged = NV_FLOW_TRUE;
                break;
            }
        }
        // reset auto cell size when authored cell size changes
        if (cellSizeChanged)
        {
            ptr->autoRescaleLevel = 0u;
        }

        NvFlowSparseSimParams sparseSimParams = {};
        NvFlowSparseParams sparseParams = {};
        NvFlowEmitterPointFeedback emitterPointFeedback = {};
        NvFlowEmitterMeshFeedback emitterMeshFeedback = {};
        NvFlowEmitterNanoVdbFeedback emitterNanoVdbFeedback = {};
        NvFlowSummaryAllocatePinsOut summaryAllocatePinsOut = {};
        for (NvFlowUint attemptIdx = 0u; attemptIdx < 64u; attemptIdx++)
        {
            float rescaleFactor = powf(2.f, float(ptr->autoRescaleLevel) / 3.f);
            for (NvFlowUint layerParamIdx = 0u; layerParamIdx < ptr->simulateParams.size; layerParamIdx++)
            {
                NvFlowFloat3 blockSizeWorld = NvFlowFloat3{
                    float(baseBlockDim.x) * fmaxf(0.f, ptr->simulateParams[layerParamIdx]->densityCellSize),
                    float(baseBlockDim.y) * fmaxf(0.f, ptr->simulateParams[layerParamIdx]->densityCellSize),
                    float(baseBlockDim.z) * fmaxf(0.f, ptr->simulateParams[layerParamIdx]->densityCellSize)
                };
                ptr->updateLayerParams[layerParamIdx].blockSizeWorld.x = blockSizeWorld.x * rescaleFactor;
                ptr->updateLayerParams[layerParamIdx].blockSizeWorld.y = blockSizeWorld.y * rescaleFactor;
                ptr->updateLayerParams[layerParamIdx].blockSizeWorld.z = blockSizeWorld.z * rescaleFactor;
            }

            ptr->sparseInterface.updateLayers(ptr->sparse, ptr->updateId, ptr->updateLayerParams.data, (NvFlowUint)ptr->updateLayerParams.size);

            ptr->sparseInterface.getSimParams(ptr->sparse, &sparseSimParams);
            sparseParams = sparseSimParams.sparseParams;

            if (sparseParams.levels && ptr->autoRescaleMaxLocations != sparseParams.levels[0u].maxLocations)
            {
                ptr->autoRescaleMaxLocations = sparseParams.levels[0u].maxLocations;
                ptr->autoRescaleLevel = 0u;
                continue;
            }

            // temporary manual allocation
            ptr->locations.size = 0u;

            // auto allocate
            {
                NV_FLOW_PROFILE_TIMESTAMP("PreSummaryAllocate")

                NvFlowSummaryAllocatePinsIn summaryAllocatePinsIn = {};
                summaryAllocatePinsIn.contextInterface = &ptr->contextInterface;
                summaryAllocatePinsIn.context = context;
                summaryAllocatePinsIn.sparseSimParams = sparseSimParams;
                summaryAllocatePinsIn.params = ptr->summaryAllocateParams.data;
                summaryAllocatePinsIn.paramCount = ptr->summaryAllocateParams.size;

                NvFlowSummaryAllocate_execute(&ptr->mSummaryAllocate, &summaryAllocatePinsIn, &summaryAllocatePinsOut);

                for (NvFlowUint idx = 0u; idx < summaryAllocatePinsOut.locationCount; idx++)
                {
                    ptr->locations.pushBack(summaryAllocatePinsOut.locations[idx]);
                }

                NV_FLOW_PROFILE_TIMESTAMP("SummaryAllocate")
            }

            // emitter sphere allocation
            {
                NvFlowEmitterSphereAllocatePinsIn pinsIn = {};
                NvFlowEmitterSphereAllocatePinsOut pinsOut = {};
                pinsIn.contextInterface = &ptr->contextInterface;
                pinsIn.context = context;
                pinsIn.sparseParams = sparseParams;
                pinsIn.deltaTime = deltaTime;
                pinsIn.params = NvFlowGridEmitterSphereParams_elements;
                pinsIn.paramCount = NvFlowGridEmitterSphereParams_elementCount;

                NvFlowEmitterSphereAllocate_execute(&ptr->mEmitterSphereAllocate, &pinsIn, &pinsOut);

                for (NvFlowUint idx = 0u; idx < pinsOut.locationCount; idx++)
                {
                    ptr->locations.pushBack(pinsOut.locations[idx]);
                }

                NV_FLOW_PROFILE_TIMESTAMP("EmitterSphereAllocate")
            }

            // emitter box allocation
            {
                NvFlowEmitterBoxAllocatePinsIn pinsIn = {};
                NvFlowEmitterBoxAllocatePinsOut pinsOut = {};
                pinsIn.contextInterface = &ptr->contextInterface;
                pinsIn.context = context;
                pinsIn.sparseParams = sparseParams;
                pinsIn.deltaTime = deltaTime;
                pinsIn.params = NvFlowGridEmitterBoxParams_elements;
                pinsIn.paramCount = NvFlowGridEmitterBoxParams_elementCount;
                pinsIn.physicsCollisionLayers = ptr->physicsCollisionLayers.data;
                pinsIn.physicsCollisionLayerCount = ptr->physicsCollisionLayers.size;

                NvFlowEmitterBoxAllocate_execute(&ptr->mEmitterBoxAllocate, &pinsIn, &pinsOut);

                for (NvFlowUint idx = 0u; idx < pinsOut.locationCount; idx++)
                {
                    ptr->locations.pushBack(pinsOut.locations[idx]);
                }

                NV_FLOW_PROFILE_TIMESTAMP("EmitterBoxAllocate")
            }

            // emitter point allocation
            {
                NvFlowEmitterPointAllocatePinsIn pinsIn = {};
                NvFlowEmitterPointAllocatePinsOut pinsOut = {};
                pinsIn.contextInterface = &ptr->contextInterface;
                pinsIn.context = context;
                pinsIn.sparseSimParams = sparseSimParams;
                pinsIn.deltaTime = deltaTime;
                pinsIn.params = NvFlowGridEmitterPointParams_elements;
                pinsIn.paramCount = NvFlowGridEmitterPointParams_elementCount;
                pinsIn.pointsParams = NvFlowPointsParams_elements;
                pinsIn.pointsParamCount = NvFlowPointsParams_elementCount;
                pinsIn.baseBlockDimBits = baseBlockDimBits;

                NvFlowEmitterPointAllocate_execute(&ptr->mEmitterPointAllocate, &pinsIn, &pinsOut);

                for (NvFlowUint idx = 0u; idx < pinsOut.locationCount; idx++)
                {
                    ptr->locations.pushBack(pinsOut.locations[idx]);
                }

                emitterPointFeedback = pinsOut.feedback;

                NV_FLOW_PROFILE_TIMESTAMP("EmitterPointAllocate")
            }

            // emitter mesh allocation
            {
                NvFlowEmitterMeshAllocatePinsIn pinsIn = {};
                NvFlowEmitterMeshAllocatePinsOut pinsOut = {};
                pinsIn.contextInterface = &ptr->contextInterface;
                pinsIn.context = context;
                pinsIn.sparseParams = sparseParams;
                pinsIn.deltaTime = deltaTime;
                pinsIn.params = NvFlowGridEmitterMeshParams_elements;
                pinsIn.paramCount = NvFlowGridEmitterMeshParams_elementCount;
                pinsIn.physicsCollisionLayers = ptr->physicsCollisionLayers.data;
                pinsIn.physicsCollisionLayerCount = ptr->physicsCollisionLayers.size;
                pinsIn.baseBlockDimBits = baseBlockDimBits;

                NvFlowEmitterMeshAllocate_execute(&ptr->mEmitterMeshAllocate, &pinsIn, &pinsOut);

                for (NvFlowUint idx = 0u; idx < pinsOut.locationCount; idx++)
                {
                    ptr->locations.pushBack(pinsOut.locations[idx]);
                }

                emitterMeshFeedback = pinsOut.feedback;

                NV_FLOW_PROFILE_TIMESTAMP("EmitterMeshAllocate")
            }

            // emitter texture allocation
            {
                NvFlowEmitterTextureAllocatePinsIn pinsIn = {};
                NvFlowEmitterTextureAllocatePinsOut pinsOut = {};
                pinsIn.contextInterface = &ptr->contextInterface;
                pinsIn.context = context;
                pinsIn.sparseParams = sparseParams;
                pinsIn.deltaTime = deltaTime;
                pinsIn.params = NvFlowGridEmitterTextureParams_elements;
                pinsIn.paramCount = NvFlowGridEmitterTextureParams_elementCount;

                NvFlowEmitterTextureAllocate_execute(&ptr->mEmitterTextureAllocate, &pinsIn, &pinsOut);

                for (NvFlowUint idx = 0u; idx < pinsOut.locationCount; idx++)
                {
                    ptr->locations.pushBack(pinsOut.locations[idx]);
                }

                NV_FLOW_PROFILE_TIMESTAMP("EmitterTextureAllocate")
            }

            // emitter NanoVdb allocation
            {
                NvFlowEmitterNanoVdbAllocatePinsIn pinsIn = {};
                NvFlowEmitterNanoVdbAllocatePinsOut pinsOut = {};
                pinsIn.contextInterface = &ptr->contextInterface;
                pinsIn.context = context;
                pinsIn.sparseSimParams = sparseSimParams;
                pinsIn.deltaTime = deltaTime;
                pinsIn.params = NvFlowGridEmitterNanoVdbParams_elements;
                pinsIn.paramCount = NvFlowGridEmitterNanoVdbParams_elementCount;
                pinsIn.volumeParams = NvFlowVolumeParams_elements;
                pinsIn.volumeParamCount = NvFlowVolumeParams_elementCount;
                pinsIn.nanoVdbAssetParams = NvFlowNanoVdbAssetParams_elements;
                pinsIn.nanoVdbAssetParamCount = NvFlowNanoVdbAssetParams_elementCount;

                NvFlowEmitterNanoVdbAllocate_execute(&ptr->mEmitterNanoVdbAllocate, &pinsIn, &pinsOut);

                for (NvFlowUint idx = 0u; idx < pinsOut.locationCount; idx++)
                {
                    ptr->locations.pushBack(pinsOut.locations[idx]);
                }

                emitterNanoVdbFeedback = pinsOut.feedback;

                NV_FLOW_PROFILE_TIMESTAMP("EmitterNanoVdbAllocate")
            }

            NV_FLOW_PROFILE_TIMESTAMP("PreUpdateLocations")

            NvFlowUint newLocationCount = 0u;
            NvFlowUint maxLocationCount = 0u;
            ptr->sparseInterface.testUpdateLocations(
                context,
                ptr->sparse,
                ptr->updateId,
                ptr->locations.data,
                (NvFlowUint)ptr->locations.size,
                baseBlockDimBits,
                maxMinBlockLifetime,
                &newLocationCount,
                &maxLocationCount
            );
            // push result into history
            NvFlowBool32 historyMatchFound = NV_FLOW_FALSE;
            NvFlowBool32 historyShouldReset = NV_FLOW_FALSE;
            for (NvFlowUint64 historyIdx = 0u; historyIdx < ptr->autoRescaleHistoryLevelCounts.size; historyIdx++)
            {
                NvFlowUint2 historyLevelCount = ptr->autoRescaleHistoryLevelCounts[historyIdx];
                if (historyLevelCount.x == ptr->autoRescaleLevel)
                {
                    if (historyLevelCount.y == newLocationCount)
                    {
                        historyMatchFound = NV_FLOW_TRUE;
                    }
                    else
                    {
                        historyShouldReset = NV_FLOW_TRUE;
                    }
                    break;
                }
            }
            if (historyShouldReset)
            {
                ptr->autoRescaleHistoryLevelCounts.size = 0u;
            }
            if (!historyMatchFound)
            {
                ptr->autoRescaleHistoryLevelCounts.pushBack(NvFlowUint2{ ptr->autoRescaleLevel, newLocationCount });
            }

            bool didFit = newLocationCount < maxLocationCount;

            // always accept result if auto cell size not enabled
            if (!anyAutoCellSizeEnabled)
            {
                break;
            }
            // if full res fits, always accept
            if (didFit && ptr->autoRescaleLevel == 0)
            {
                break;
            }
            // avoid trying cell size shrink too often
            if (didFit && attemptIdx == 0u)
            {
                ptr->autoRescaleStableFrames++;
                if (ptr->autoRescaleStableFrames > 120u)
                {
                    ptr->autoRescaleStableFrames = 0u;
                    bool couldBeFiner = (2u * newLocationCount < maxLocationCount) && (ptr->autoRescaleLevel > 0u);
                    bool reallyCouldBeFiner = (4u * newLocationCount < maxLocationCount) && (ptr->autoRescaleLevel > 0u);
                    if ((couldBeFiner || reallyCouldBeFiner) && ptr->autoRescaleLevel > 0u)
                    {
                        // check history to see if this has been tried and failed before
                        NvFlowUint decAutoRescaleLevel = ptr->autoRescaleLevel - 1u;
                        bool shouldTryDec = couldBeFiner;
                        bool shouldTryZero = reallyCouldBeFiner;
                        for (NvFlowUint64 historyIdx = 0u; historyIdx < ptr->autoRescaleHistoryLevelCounts.size; historyIdx++)
                        {
                            NvFlowUint2 historyLevelCount = ptr->autoRescaleHistoryLevelCounts[historyIdx];
                            if (historyLevelCount.x == decAutoRescaleLevel &&
                                historyLevelCount.y >= maxLocationCount)
                            {
                                shouldTryDec = false;
                            }
                            if (historyLevelCount.x == 0u &&
                                historyLevelCount.y >= maxLocationCount)
                            {
                                shouldTryZero = false;
                            }
                        }
                        if (shouldTryZero)
                        {
                            ptr->autoRescaleLevel = 0u;
                            continue;
                        }
                        else if (shouldTryDec)
                        {
                            ptr->autoRescaleLevel--;
                            continue;
                        }
                    }
                }
                break;
            }

            bool refineComplete = autoScaleSearchRefine && autoScaleSearchWidth == 1;
            bool postFirstUpscale = !autoScaleSearchRefine && autoScaleSearchWidth == 1;
            if (didFit && (refineComplete || postFirstUpscale))
            {
                break;
            }
            else
            {
                if (autoScaleSearchRefine)
                {
                    autoScaleSearchWidth /= 2;
                    if (autoScaleSearchWidth == 0u)
                    {
                        autoScaleSearchWidth = 1u;
                    }
                }
                else
                {
                    if (autoScaleSearchWidth == 0)
                    {
                        autoScaleSearchWidth = 1u;
                    }
                    else if (autoScaleSearchWidth < 0x20000000) // need to avoid overflow
                    {
                        autoScaleSearchWidth *= 2;
                    }
                }
                if (didFit)
                {
                    if (!autoScaleSearchRefine)
                    {
                        autoScaleSearchRefine = true;
                        autoScaleSearchWidth = (autoScaleSearchWidth) / 4;
                        if (autoScaleSearchWidth == 0u)
                        {
                            autoScaleSearchWidth = 1u;
                        }
                    }
                    if (ptr->autoRescaleLevel > autoScaleSearchWidth)
                    {
                        ptr->autoRescaleLevel -= autoScaleSearchWidth;
                    }
                    else
                    {
                        ptr->autoRescaleLevel = 0u;
                    }
                }
                else
                {
                    // need to avoid overflow
                    if (ptr->autoRescaleLevel + autoScaleSearchWidth < 0x20000000)
                    {
                        ptr->autoRescaleLevel += autoScaleSearchWidth;
                    }
                    else
                    {
                        ptr->autoRescaleLevel = 0x20000000;
                    }
                }
            }
        }

        // update cell size history
        ptr->autoRescaleOldLayerAndLevels.reserve(ptr->simulateParams.size);
        ptr->autoRescaleOldDensityCellSizes.reserve(ptr->simulateParams.size);
        ptr->autoRescaleOldLayerAndLevels.size = ptr->simulateParams.size;
        ptr->autoRescaleOldDensityCellSizes.size = ptr->simulateParams.size;
        for (NvFlowUint layerParamIdx = 0u; layerParamIdx < ptr->simulateParams.size; layerParamIdx++)
        {
            ptr->autoRescaleOldLayerAndLevels[layerParamIdx] = NvFlow_packLayerAndLevel(ptr->simulateParams[layerParamIdx]->layer, ptr->simulateParams[layerParamIdx]->level);
            ptr->autoRescaleOldDensityCellSizes[layerParamIdx] = ptr->simulateParams[layerParamIdx]->densityCellSize;
        }

        NV_FLOW_PROFILE_TIMESTAMP("TestUpdateLocations")

        ptr->sparseInterface.updateLocations(context, ptr->sparse, ptr->updateId, ptr->locations.data, (NvFlowUint)ptr->locations.size, baseBlockDimBits, maxMinBlockLifetime);

        NV_FLOW_PROFILE_TIMESTAMP("UpdateLocations")

        NvFlowBufferTransient* sparseBuffer = nullptr;
        ptr->sparseInterface.addPasses(context, ptr->sparse, &sparseBuffer);

        NV_FLOW_PROFILE_TIMESTAMP("UpdateLocationsAddPasses")

        ptr->sparseInterface.getSimParams(ptr->sparse, &sparseSimParams);
        sparseParams = sparseSimParams.sparseParams;

        if (sparseParams.levelCount > 0u)
        {
            ptr->activeBlockCount = sparseParams.levels[0u].numLocations;
        }

        SimulateStepResources stepResources = {};

        // clear new
        {
            // vote on precision
            NvFlowBool32 enableLowPrecisionVelocity = NV_FLOW_FALSE;
            NvFlowBool32 enableLowPrecisionDensity = NV_FLOW_FALSE;
            NvFlowBool32 enableLowPrecisionRescale = NV_FLOW_FALSE;
            NvFlowBool32 enableHighPrecisionVelocity = NV_FLOW_FALSE;
            NvFlowBool32 enableHighPrecisionDensity = NV_FLOW_FALSE;
            for (NvFlowUint layerParamIdx = 0u; layerParamIdx < ptr->simulateParams.size; layerParamIdx++)
            {
                if (ptr->simulateParams[layerParamIdx]->enableLowPrecisionVelocity)
                {
                    enableLowPrecisionVelocity = NV_FLOW_TRUE;
                }
                if (ptr->simulateParams[layerParamIdx]->enableLowPrecisionDensity)
                {
                    enableLowPrecisionDensity = NV_FLOW_TRUE;
                }
                if (ptr->simulateParams[layerParamIdx]->enableLowPrecisionRescale)
                {
                    enableLowPrecisionRescale = NV_FLOW_TRUE;
                }
                if (ptr->simulateParams[layerParamIdx]->enableHighPrecisionVelocity)
                {
                    enableHighPrecisionVelocity = NV_FLOW_TRUE;
                }
                if (ptr->simulateParams[layerParamIdx]->enableHighPrecisionDensity)
                {
                    enableHighPrecisionDensity = NV_FLOW_TRUE;
                }
            }

            NvFlowFormat velocity_format =
                enableHighPrecisionVelocity ? eNvFlowFormat_r32g32b32a32_float : (
                enableLowPrecisionVelocity ? eNvFlowFormat_r8g8b8a8_unorm : eNvFlowFormat_r16g16b16a16_float);
            NvFlowFormat density_format =
                enableHighPrecisionDensity ? eNvFlowFormat_r32g32b32a32_float : (
                enableLowPrecisionDensity ? eNvFlowFormat_r8g8b8a8_unorm : eNvFlowFormat_r16g16b16a16_float);

            // migrate existing variables to new mapping
            ptr->sparseInterface.addPassesMigrate(
                context,
                ptr->sparse,
                enableLowPrecisionRescale,
                NvFlowTextureVariable_get(context, &ptr->velocityVariable, nullptr),
                &ptr->velocityVariable.hintTexDesc,
                velocity_format,
                velocityLevelIdx,
                &stepResources.velocityFront,
                &ptr->velocityVariable.hintTexDesc
            );
            ptr->sparseInterface.addPassesMigrate(
                context,
                ptr->sparse,
                enableLowPrecisionRescale,
                NvFlowTextureVariable_get(context, &ptr->densityVariable, nullptr),
                &ptr->densityVariable.hintTexDesc,
                density_format,
                densityLevelIdx,
                &stepResources.densityFront,
                &ptr->densityVariable.hintTexDesc
            );
            ptr->sparseInterface.addPassesMigrate(
                context,
                ptr->sparse,
                enableLowPrecisionRescale,
                NvFlowTextureVariable_get(context, &ptr->coarseDensityVariable, nullptr),
                &ptr->coarseDensityVariable.hintTexDesc,
                density_format,
                velocityLevelIdx,
                &stepResources.coarseDensityFront,
                &ptr->coarseDensityVariable.hintTexDesc
            );
            ptr->sparseInterface.addPassesMigrate(
                context,
                ptr->sparse,
                enableLowPrecisionRescale,
                NvFlowTextureVariable_get(context, &ptr->voxelWeightFineVariable, nullptr),
                &ptr->voxelWeightFineVariable.hintTexDesc,
                eNvFlowFormat_r8_unorm,
                densityLevelIdx,
                &stepResources.voxelWeightFineFront,
                &ptr->voxelWeightFineVariable.hintTexDesc
            );
            ptr->sparseInterface.addPassesMigrate(
                context,
                ptr->sparse,
                enableLowPrecisionRescale,
                NvFlowTextureVariable_get(context, &ptr->voxelWeightCoarseVariable, nullptr),
                &ptr->voxelWeightCoarseVariable.hintTexDesc,
                eNvFlowFormat_r8_unorm,
                velocityLevelIdx,
                &stepResources.voxelWeightCoarseFront,
                &ptr->voxelWeightCoarseVariable.hintTexDesc
            );

            // free variables to save memory during simulation
            NvFlowTextureVariable_set(context, &ptr->velocityVariable, nullptr, eNvFlowFormat_unknown);
            NvFlowTextureVariable_set(context, &ptr->densityVariable, nullptr, eNvFlowFormat_unknown);
            NvFlowTextureVariable_set(context, &ptr->coarseDensityVariable, nullptr, eNvFlowFormat_unknown);
            NvFlowTextureVariable_set(context, &ptr->voxelWeightFineVariable, nullptr, eNvFlowFormat_unknown);
            NvFlowTextureVariable_set(context, &ptr->voxelWeightCoarseVariable, nullptr, eNvFlowFormat_unknown);
        }

        // TODO: make per layer proper, for now, must vote
        float stepsPerSecond = 1.f;
        NvFlowUint maxStepsPerSimulate = 1u;
        NvFlowBool32 enableVariableTimeStep = NV_FLOW_FALSE;
        NvFlowBool32 interpolateTimeSteps = NV_FLOW_FALSE;
        float minTimeScale = 1000.f;    // 1000.f is arbitrary, but reasonable
        for (NvFlowUint layerParamIdx = 0u; layerParamIdx < ptr->simulateParams.size; layerParamIdx++)
        {
            if (sparseParams.layers[layerParamIdx].numLocations > 0u)
            {
                if (ptr->simulateParams[layerParamIdx]->stepsPerSecond > stepsPerSecond)
                {
                    stepsPerSecond = ptr->simulateParams[layerParamIdx]->stepsPerSecond;
                }
                if (ptr->simulateParams[layerParamIdx]->maxStepsPerSimulate > maxStepsPerSimulate)
                {
                    maxStepsPerSimulate = ptr->simulateParams[layerParamIdx]->maxStepsPerSimulate;
                }
                if (ptr->simulateParams[layerParamIdx]->enableVariableTimeStep)
                {
                    enableVariableTimeStep = NV_FLOW_TRUE;
                }
                if (ptr->simulateParams[layerParamIdx]->timeScale < minTimeScale)
                {
                    minTimeScale = ptr->simulateParams[layerParamIdx]->timeScale;
                }
                if (ptr->simulateParams[layerParamIdx]->interpolateTimeSteps)
                {
                    interpolateTimeSteps = NV_FLOW_TRUE;
                }
            }
        }

        if (allForceClear)
        {
            TimeStepper_init(&ptr->timeStepper);
        }

        NvFlowUint numSteps = maxStepsPerSimulate;
        float stepDeltaTime = 0.f;
        if (enableVariableTimeStep)
        {
            stepDeltaTime = minTimeScale * deltaTime / float(numSteps);
        }
        else
        {
            TimeStepper_configure(&ptr->timeStepper, stepsPerSecond, maxStepsPerSimulate);
            TimeStepper_update(&ptr->timeStepper, deltaTime);
            numSteps = ptr->timeStepper.numSteps;
            stepDeltaTime = minTimeScale * ptr->timeStepper.fixedDt;
        }

        ptr->layerDeltaTimes.reserve(sparseParams.layerCount);
        ptr->layerDeltaTimes.size = sparseParams.layerCount;
        for (NvFlowUint layerParamIdx = 0u; layerParamIdx < sparseParams.layerCount; layerParamIdx++)
        {
            ptr->layerDeltaTimes[layerParamIdx] = float(numSteps) * stepDeltaTime;
        }
        ptr->sparseInterface.updateLayerDeltaTimes(ptr->sparse, ptr->layerDeltaTimes.data, ptr->layerDeltaTimes.size);

        for (NvFlowUint stepIdx = 0u; stepIdx < numSteps; stepIdx++)
        {
            NV_FLOW_PROFILE_TIMESTAMP("PreSimulateStep")

            simulateStep(context, ptr, paramsIn, stepDeltaTime, sparseParams, sparseBuffer, &stepResources, emitterPointFeedback, emitterMeshFeedback, emitterNanoVdbFeedback);

            NV_FLOW_PROFILE_TIMESTAMP("SimulateStep")
        }

        // update interpolated density if enabled
        NvFlowTextureTransient* densityInterpTransient = nullptr;
        if (interpolateTimeSteps)
        {
            NvFlowSparseTexture densityTemp = {};
            NvFlowSparseTexture_duplicate(&ptr->contextInterface, context, &densityTemp, &stepResources.densityFront);

            float interpDeltaTime = 0.f;
            if (enableVariableTimeStep)
            {
                interpDeltaTime = 0.f;
            }
            else
            {
                interpDeltaTime = minTimeScale * (ptr->timeStepper.timeError > 0.f ? ptr->timeStepper.timeError : 0.f);
            }

            NvFlowAdvectionCombustionDensityPinsIn advectDensityPinsIn = {};
            NvFlowAdvectionCombustionDensityPinsOut advectDensityPinsOut = {};
            advectDensityPinsIn.contextInterface = &ptr->contextInterface;
            advectDensityPinsIn.context = context;
            advectDensityPinsIn.deltaTime = interpDeltaTime;
            advectDensityPinsIn.params = ptr->advectionCombustionParams.data;
            advectDensityPinsIn.paramCount = ptr->advectionCombustionParams.size;
            advectDensityPinsIn.velocity = stepResources.velocityFront;
            advectDensityPinsIn.density = stepResources.densityFront;
            advectDensityPinsIn.densityTemp = densityTemp;

            NvFlowAdvectionCombustionDensity_execute(&ptr->mAdvectionCombustionDensity, &advectDensityPinsIn, &advectDensityPinsOut);

            densityInterpTransient = advectDensityPinsOut.density.textureTransient;

            NV_FLOW_PROFILE_TIMESTAMP("Interpolate")
        }

        // summary
        {
            NvFlowSummaryPinsIn summaryPinsIn = {};
            NvFlowSummaryPinsOut summaryPinsOut = {};
            summaryPinsIn.contextInterface = &ptr->contextInterface;
            summaryPinsIn.context = context;
            summaryPinsIn.feedback = summaryAllocatePinsOut.feedback;
            summaryPinsIn.velocity = stepResources.velocityFront;
            summaryPinsIn.densityCoarse = stepResources.coarseDensityFront;
            summaryPinsIn.params = ptr->summaryAllocateParams.data;
            summaryPinsIn.paramCount = ptr->summaryAllocateParams.size;

            NvFlowSummary_execute(&ptr->mSummary, &summaryPinsIn, &summaryPinsOut);

            NV_FLOW_PROFILE_TIMESTAMP("Summary")
        }

        // NanoVdb export
        {
            NvFlowUint maxReadbackRingBufferCount = 0u;
            NvFlowUint maxInteropRingBufferCount = 0u;
            for (NvFlowUint64 idx = 0u; idx < NvFlowGridReadbackParams_elementCount; idx++)
            {
                if (NvFlowGridReadbackParams_elements[idx]->readbackRingBufferCount > maxReadbackRingBufferCount)
                {
                    maxReadbackRingBufferCount = NvFlowGridReadbackParams_elements[idx]->readbackRingBufferCount;
                }
                if (NvFlowGridReadbackParams_elements[idx]->interopRingBufferCount > maxInteropRingBufferCount)
                {
                    maxInteropRingBufferCount = NvFlowGridReadbackParams_elements[idx]->interopRingBufferCount;
                }
            }

            NvFlowSparseNanoVdbExportPinsIn nanoVdbPinsIn = {};
            NvFlowSparseNanoVdbExportPinsOut nanoVdbPinsOut = {};
            nanoVdbPinsIn.contextInterface = &ptr->contextInterface;
            nanoVdbPinsIn.context = context;
            nanoVdbPinsIn.sparseInterface = &ptr->sparseInterface;
            nanoVdbPinsIn.sparse = ptr->sparse;
            nanoVdbPinsIn.params = ptr->nanoVdbExportParams.data;
            nanoVdbPinsIn.paramCount = ptr->nanoVdbExportParams.size;
            nanoVdbPinsIn.absoluteSimTime = absoluteSimTime;
            nanoVdbPinsIn.readbackRingBufferCount = maxReadbackRingBufferCount;
            nanoVdbPinsIn.interopRingBufferCount = maxInteropRingBufferCount;
            nanoVdbPinsIn.velocity = stepResources.velocityFront;
            nanoVdbPinsIn.density = stepResources.densityFront;

            NvFlowSparseNanoVdbExport_execute(&ptr->mSparseNanoVdbExport, &nanoVdbPinsIn, &nanoVdbPinsOut);

            NvFlowBufferVariable_set(context, &ptr->temperatureNanoVdbVariable, nanoVdbPinsOut.temperatureNanoVdb);
            NvFlowBufferVariable_set(context, &ptr->fuelNanoVdbVariable, nanoVdbPinsOut.fuelNanoVdb);
            NvFlowBufferVariable_set(context, &ptr->burnNanoVdbVariable, nanoVdbPinsOut.burnNanoVdb);
            NvFlowBufferVariable_set(context, &ptr->smokeNanoVdbVariable, nanoVdbPinsOut.smokeNanoVdb);
            NvFlowBufferVariable_set(context, &ptr->velocityNanoVdbVariable, nanoVdbPinsOut.velocityNanoVdb);
            NvFlowBufferVariable_set(context, &ptr->divergenceNanoVdbVariable, nanoVdbPinsOut.divergenceNanoVdb);

            // invalidate
            NvFlowGridRenderDataNanoVdb gridRenderDataNanoVdbNull = {};
            ptr->gridRenderDataNanoVdb = gridRenderDataNanoVdbNull;

            // only readback copied here
            NvFlowBool32 anyEnableGridRenderDataNanoVdb = NvFlowGridReadbackParams_elementCount == 0u ? NV_FLOW_TRUE : NV_FLOW_FALSE;
            NvFlowBool32 allOnlyLatest = NV_FLOW_TRUE;
            for (NvFlowUint64 idx = 0u; idx < NvFlowGridReadbackParams_elementCount; idx++)
            {
                if (NvFlowGridReadbackParams_elements[idx]->enableGridRenderDataNanoVdb)
                {
                    anyEnableGridRenderDataNanoVdb = NV_FLOW_TRUE;
                }
                if (!NvFlowGridReadbackParams_elements[idx]->onlyLatest)
                {
                    allOnlyLatest = NV_FLOW_FALSE;
                }
            }
            if (anyEnableGridRenderDataNanoVdb)
            {
                ptr->gridRenderDataNanoVdbReadbacks.size = 0u;
                ptr->gridRenderDataNanoVdbReadbacks.reserve(nanoVdbPinsOut.readbackCount);
                ptr->gridRenderDataNanoVdbReadbacks.size = nanoVdbPinsOut.readbackCount;
                for (NvFlowUint64 idx = 0u; idx < ptr->gridRenderDataNanoVdbReadbacks.size; idx++)
                {
                    NvFlowGridRenderDataNanoVdbReadback* dst = &ptr->gridRenderDataNanoVdbReadbacks[idx];
                    NvFlowSparseNanoVdbExportReadback* src = &nanoVdbPinsOut.readbacks[idx];
                    dst->globalFrameCompleted = src->globalFrameCompleted;
                    dst->temperatureNanoVdbReadback = (NvFlowUint8*)src->temperatureNanoVdbReadback;
                    dst->temperatureNanoVdbReadbackSize = sizeof(NvFlowUint) * src->temperatureNanoVdbReadbackCount;
                    dst->fuelNanoVdbReadback = (NvFlowUint8*)src->fuelNanoVdbReadback;
                    dst->fuelNanoVdbReadbackSize = sizeof(NvFlowUint) * src->fuelNanoVdbReadbackCount;
                    dst->burnNanoVdbReadback = (NvFlowUint8*)src->burnNanoVdbReadback;
                    dst->burnNanoVdbReadbackSize = sizeof(NvFlowUint) * src->burnNanoVdbReadbackCount;
                    dst->smokeNanoVdbReadback = (NvFlowUint8*)src->smokeNanoVdbReadback;
                    dst->smokeNanoVdbReadbackSize = sizeof(NvFlowUint) * src->smokeNanoVdbReadbackCount;
                    dst->velocityNanoVdbReadback = (NvFlowUint8*)src->velocityNanoVdbReadback;
                    dst->velocityNanoVdbReadbackSize = sizeof(NvFlowUint) * src->velocityNanoVdbReadbackCount;
                    dst->divergenceNanoVdbReadback = (NvFlowUint8*)src->divergenceNanoVdbReadback;
                    dst->divergenceNanoVdbReadbackSize = sizeof(NvFlowUint) * src->divergenceNanoVdbReadbackCount;
                }

                ptr->gridRenderDataNanoVdb.readbacks = ptr->gridRenderDataNanoVdbReadbacks.data;
                ptr->gridRenderDataNanoVdb.readbackCount = ptr->gridRenderDataNanoVdbReadbacks.size;
            }

            // preserve readbacks as needed to handle gridParams delay
            for (NvFlowUint64 gridReadbackIdx = 0u; gridReadbackIdx < NvFlowGridReadbackParams_elementCount; gridReadbackIdx++)
            {
                // get gridParams as needed
                GridGridParamsReadback* gridParamsReadback = nullptr;
                for (NvFlowUint64 cacheIdx = 0u; cacheIdx < ptr->gridParamsReadbacks.size; cacheIdx++)
                {
                    if (strcmp(ptr->gridParamsReadbacks[cacheIdx]->name.data, NvFlowGridReadbackParams_elements[gridReadbackIdx]->gridParamsName) == 0)
                    {
                        gridParamsReadback = ptr->gridParamsReadbacks[cacheIdx];
                        break;
                    }
                }
                if (!gridParamsReadback)
                {
                    gridParamsReadback = ptr->gridParamsReadbacks.allocateBackPointer();
                    gridParamsReadback->name.pushBackN(
                        NvFlowGridReadbackParams_elements[gridReadbackIdx]->gridParamsName,
                        strlen(NvFlowGridReadbackParams_elements[gridReadbackIdx]->gridParamsName)
                    );
                    gridParamsReadback->name.pushBack('\0');
                    gridParamsReadback->name.size--; // do not include null in string size

                    gridParamsReadback->gridParamsNamed = NvFlowGetGridParamsInterface()->createGridParamsNamed(gridParamsReadback->name.data);
                }

                NvFlowGridParams* gridParams = NvFlowGetGridParamsInterface()->mapGridParamsNamed(gridParamsReadback->gridParamsNamed);

                NvFlowUint64 stagingVersion = ~0llu;
                NvFlowUint64 minActiveVersion = ~0llu;
                NvFlowGetGridParamsInterface()->getVersion(gridParams, &stagingVersion, &minActiveVersion);

                // free old snapshots
                NvFlowUint64 writeSnapshotIdx = 0u;
                for (NvFlowUint64 snapshotIdx = 0u; snapshotIdx < gridParamsReadback->gridReadbackSnapshots.size; snapshotIdx++)
                {
                    if (gridParamsReadback->gridReadbackSnapshots[snapshotIdx]->snapshot.snapshot.version >= minActiveVersion)
                    {
                        gridParamsReadback->gridReadbackSnapshots.swapPointers(writeSnapshotIdx, snapshotIdx);
                        writeSnapshotIdx++;
                    }
                }
                gridParamsReadback->gridReadbackSnapshots.size = writeSnapshotIdx;

                // build snapshot
                GridReadbackSnapshot* snapshot = gridParamsReadback->gridReadbackSnapshots.allocateBackPointer();
                snapshot->typeSnapshots.size = 0u;
                snapshot->gridTypeSnapshots.size = 0u;

                // global readback data
                GridReadbackTypeSnapshot* snapshotReadbackData = snapshot->gridTypeSnapshots.allocateBackPointer();
                GridReadbackTypeSnapshot_reset(snapshotReadbackData);

                NvFlowGridReadbackData* gridReadbackData = snapshotReadbackData->gridReadbackData.allocateBackPointer();
                snapshotReadbackData->instanceDatas.pushBack((NvFlowUint8*)gridReadbackData);

                gridReadbackData->numLocations = sparseSimParams.sparseParams.levels[0u].numLocations;
                gridReadbackData->maxLocations = sparseSimParams.sparseParams.levels[0u].maxLocations;
                gridReadbackData->lastGlobalFrameCompleted = ptr->contextInterface.getLastGlobalFrameCompleted(context);
                gridReadbackData->lastAbsoluteSimTimeCompleted = nanoVdbPinsOut.lastAbsoluteSimTimeCompleted;

                // generate readback instances
                GridReadbackTypeSnapshot* snapshotNanoVdb = snapshot->gridTypeSnapshots.allocateBackPointer();
                GridReadbackTypeSnapshot_reset(snapshotNanoVdb);

                NvFlowUint64 begin_idx = 0u;
                if (allOnlyLatest && nanoVdbPinsOut.readbackCount > 0u)
                {
                    begin_idx = nanoVdbPinsOut.readbackCount - 1u;
                }
                for (NvFlowUint64 idx = begin_idx; idx < nanoVdbPinsOut.readbackCount; idx++)
                {
                    NvFlowSparseNanoVdbExportReadback* src = &nanoVdbPinsOut.readbacks[idx];

                    GridNanoVdbReadback* dst = snapshotNanoVdb->gridNanoVdbReadbacks.allocateBackPointer();

                    NvFlowSparseNanoVdbExportReadback* dstData = &dst->data;
                    snapshotNanoVdb->instanceDatas.pushBack((NvFlowUint8*)dstData);

                    *dstData = *src;

                    // copying array is only safe if GPU is completed
                    if (src->globalFrameCompleted <= gridReadbackData->lastGlobalFrameCompleted)
                    {
                        auto copyArray = [](NvFlowArray<NvFlowUint>& dst, const NvFlowUint* srcData, NvFlowUint64 srcCount)
                        {
                            dst.size = 0u;
                            dst.reserve(srcCount);
                            dst.size = srcCount;
                            memcpy(dst.data, srcData, srcCount * sizeof(NvFlowUint));
                            return dst.data;
                        };

                        // Need to make array copies to preserve longer than original lifetime
                        dstData->temperatureNanoVdbReadback = copyArray(dst->temperatureNanoVdbReadback, src->temperatureNanoVdbReadback, src->temperatureNanoVdbReadbackCount);
                        dstData->fuelNanoVdbReadback = copyArray(dst->fuelNanoVdbReadback, src->fuelNanoVdbReadback, src->fuelNanoVdbReadbackCount);
                        dstData->burnNanoVdbReadback = copyArray(dst->burnNanoVdbReadback, src->burnNanoVdbReadback, src->burnNanoVdbReadbackCount);
                        dstData->smokeNanoVdbReadback = copyArray(dst->smokeNanoVdbReadback, src->smokeNanoVdbReadback, src->smokeNanoVdbReadbackCount);
                        dstData->velocityNanoVdbReadback = copyArray(dst->velocityNanoVdbReadback, src->velocityNanoVdbReadback, src->velocityNanoVdbReadbackCount);
                        dstData->divergenceNanoVdbReadback = copyArray(dst->divergenceNanoVdbReadback, src->divergenceNanoVdbReadback, src->divergenceNanoVdbReadbackCount);
                        dstData->rgbaNanoVdbReadback = copyArray(dst->rgbaNanoVdbReadback, src->rgbaNanoVdbReadback, src->rgbaNanoVdbReadbackCount);
                        dstData->rgbNanoVdbReadback = copyArray(dst->rgbNanoVdbReadback, src->rgbNanoVdbReadback, src->rgbNanoVdbReadbackCount);
                    }
                }

                // generate interop instances
                GridReadbackTypeSnapshot* snapshotNanoVdbInterop = snapshot->gridTypeSnapshots.allocateBackPointer();
                GridReadbackTypeSnapshot_reset(snapshotNanoVdbInterop);

                /*NvFlowUint64*/ begin_idx = 0u;
                if (allOnlyLatest && nanoVdbPinsOut.interopCount > 0u)
                {
                    begin_idx = nanoVdbPinsOut.interopCount - 1u;
                }
                for (NvFlowUint64 idx = begin_idx; idx < nanoVdbPinsOut.interopCount; idx++)
                {
                    NvFlowSparseNanoVdbExportInterop* src = &nanoVdbPinsOut.interops[idx];
                    NvFlowSparseNanoVdbExportInterop* dst = snapshotNanoVdbInterop->gridNanoVdbInterops.allocateBackPointer();

                    snapshotNanoVdbInterop->instanceDatas.pushBack((NvFlowUint8*)dst);
                    *dst = *src;
                }

                // generate global snapshot
                NvFlowGridParamsDescSnapshot nullSnapshot = {};
                snapshot->snapshot = nullSnapshot;

                // global readback data snapshot
                NvFlowDatabaseTypeSnapshot typeSnapshot = {};
                typeSnapshot.instanceDatas = snapshotReadbackData->instanceDatas.data;
                typeSnapshot.instanceCount = snapshotReadbackData->instanceDatas.size;
                typeSnapshot.dataType = &NvFlowGridReadbackData_NvFlowReflectDataType;
                typeSnapshot.version = stagingVersion;
                snapshot->typeSnapshots.pushBack(typeSnapshot);

                // NanoVdb snapshot
                typeSnapshot.instanceDatas = snapshotNanoVdb->instanceDatas.data;
                typeSnapshot.instanceCount = snapshotNanoVdb->instanceDatas.size;
                typeSnapshot.dataType = &NvFlowSparseNanoVdbExportReadback_NvFlowReflectDataType;
                typeSnapshot.version = stagingVersion;
                snapshot->typeSnapshots.pushBack(typeSnapshot);

                // NanoVdbInterop snapshot
                typeSnapshot.instanceDatas = snapshotNanoVdbInterop->instanceDatas.data;
                typeSnapshot.instanceCount = snapshotNanoVdbInterop->instanceDatas.size;
                typeSnapshot.dataType = &NvFlowSparseNanoVdbExportInterop_NvFlowReflectDataType;
                typeSnapshot.version = stagingVersion;
                snapshot->typeSnapshots.pushBack(typeSnapshot);

                snapshot->snapshot.snapshot.version = stagingVersion;
                snapshot->snapshot.absoluteSimTime = absoluteSimTime;
                snapshot->snapshot.deltaTime = deltaTime;
                snapshot->snapshot.snapshot.typeSnapshots = snapshot->typeSnapshots.data;
                snapshot->snapshot.snapshot.typeSnapshotCount = snapshot->typeSnapshots.size;

                NvFlowGetGridParamsInterface()->commitParams(gridParams, &snapshot->snapshot);
            }

            NV_FLOW_PROFILE_TIMESTAMP("NanoVdbExport")
        }

        NvFlowTextureVariable_set(context, &ptr->velocityVariable, stepResources.velocityFront.textureTransient, stepResources.velocityFront.format);
        NvFlowTextureVariable_set(context, &ptr->densityVariable, stepResources.densityFront.textureTransient, stepResources.densityFront.format);
        NvFlowTextureVariable_set(context, &ptr->densityInterpVariable, densityInterpTransient, stepResources.densityFront.format);
        NvFlowTextureVariable_set(context, &ptr->coarseDensityVariable, stepResources.coarseDensityFront.textureTransient, stepResources.coarseDensityFront.format);
        NvFlowTextureVariable_set(context, &ptr->voxelWeightFineVariable, stepResources.voxelWeightFineFront.textureTransient, stepResources.voxelWeightFineFront.format);
        NvFlowTextureVariable_set(context, &ptr->voxelWeightCoarseVariable, stepResources.voxelWeightCoarseFront.textureTransient, stepResources.voxelWeightCoarseFront.format);

        NV_FLOW_PROFILE_TIMESTAMP("SimulateEnd")
        NV_FLOW_PROFILE_FLUSH("Simulate", ptr->contextInterface.getLogPrint(context))
    }

    void offscreen(
        NvFlowContext* context,
        NvFlowGrid* grid,
        const NvFlowGridParamsDesc* paramsIn
    )
    {
        auto ptr = cast(grid);

        // For now, fetching only the latest
        NvFlowDatabaseSnapshot snapshot = {};
        if (paramsIn->snapshotCount > 0u)
        {
            snapshot = paramsIn->snapshots[paramsIn->snapshotCount - 1u].snapshot;
        }
        NV_FLOW_DATABASE_SNAPSHOT_FIND_TYPE_ARRAY(&snapshot, NvFlowGridOffscreenLayerParams)

        // generate offscreen params array that aligns with simulateParams layers
        ptr->offscreenParams.size = 0u;
        ptr->offscreenParams.reserve(ptr->simulateParams.size);
        ptr->offscreenParams.size = ptr->simulateParams.size;
        for (NvFlowUint writeIdx = 0u; writeIdx < ptr->offscreenParams.size; writeIdx++)
        {
            ptr->offscreenParams[writeIdx] = nullptr;
            int writeLayer = ptr->simulateParams[writeIdx]->layer;
            int writeLevel = ptr->simulateParams[writeIdx]->level;
            int rootLevel = ptr->simulateParamsRootLevel[writeIdx];
            NvFlowUint closestMatch = ~0u;
            for (NvFlowUint readIdx = 0u; readIdx < NvFlowGridOffscreenLayerParams_elementCount; readIdx++)
            {
                int offLayer = NvFlowGridOffscreenLayerParams_elements[readIdx]->layer;
                int offLevel = NvFlowGridOffscreenLayerParams_elements[readIdx]->level;
                if (writeLayer == offLayer && offLevel >= rootLevel && offLevel <= writeLevel)
                {
                    NvFlowUint offDiff = (NvFlowUint)(writeLevel - offLevel);
                    if (offDiff <= closestMatch)
                    {
                        closestMatch = offDiff;
                        ptr->offscreenParams[writeIdx] = NvFlowGridOffscreenLayerParams_elements[readIdx];
                    }
                }
            }
            if (!ptr->offscreenParams[writeIdx])
            {
                ptr->offscreenParams[writeIdx] = &ptr->offscreenDefault;
            }
        }

        // generate layerParamIdx aligned lists for operators that need them
        ptr->rayMarchColormapParams.reserve(ptr->offscreenParams.size);
        ptr->rayMarchColormapParams.size = ptr->offscreenParams.size;
        for (NvFlowUint layerParamIdx = 0u; layerParamIdx < ptr->offscreenParams.size; layerParamIdx++)
        {
            ptr->rayMarchColormapParams[layerParamIdx] = &ptr->offscreenParams[layerParamIdx]->colormap;
        }

        ptr->shadowParams.reserve(ptr->offscreenParams.size);
        ptr->shadowParams.size = ptr->offscreenParams.size;
        for (NvFlowUint layerParamIdx = 0u; layerParamIdx < ptr->offscreenParams.size; layerParamIdx++)
        {
            ptr->shadowParams[layerParamIdx] = &ptr->offscreenParams[layerParamIdx]->shadow;
        }

        ptr->debugVolumeParams.reserve(ptr->offscreenParams.size);
        ptr->debugVolumeParams.size = ptr->offscreenParams.size;
        for (NvFlowUint layerParamIdx = 0u; layerParamIdx < ptr->offscreenParams.size; layerParamIdx++)
        {
            ptr->debugVolumeParams[layerParamIdx] = &ptr->offscreenParams[layerParamIdx]->debugVolume;
        }

        float deltaTime = 0.f;

        NvFlowBufferTransient* sparseBuffer = ptr->sparseInterface.getSparseBuffer(context, ptr->sparse);
        // default to interpolated density, fallback to raw density
        NvFlowFormat densityOutput_format = eNvFlowFormat_unknown;
        NvFlowTextureTransient* densityOutput_tex = NvFlowTextureVariable_get(context, &ptr->densityInterpVariable, &densityOutput_format);
        if (!densityOutput_tex)
        {
            densityOutput_tex = NvFlowTextureVariable_get(context, &ptr->densityVariable, &densityOutput_format);
        }
        NvFlowFormat coarseDensityOutput_format = eNvFlowFormat_unknown;
        NvFlowTextureTransient* coarseDensityOutput_tex = NvFlowTextureVariable_get(context, &ptr->coarseDensityVariable, &coarseDensityOutput_format);
        NvFlowFormat velocityOutput_format = eNvFlowFormat_unknown;
        NvFlowTextureTransient* velocityOutput_tex = NvFlowTextureVariable_get(context, &ptr->velocityVariable, &velocityOutput_format);

        NvFlowSparseParams sparseParams = {};
        ptr->sparseInterface.getParams(ptr->sparse, &sparseParams);

        NvFlowSparseTexture densityOutput = { densityOutput_tex, sparseBuffer, sparseParams, densityLevelIdx, densityOutput_format };
        NvFlowSparseTexture coarseDensityOutput = { coarseDensityOutput_tex, sparseBuffer, sparseParams, velocityLevelIdx, coarseDensityOutput_format };
        NvFlowSparseTexture velocityOutput = { velocityOutput_tex, sparseBuffer, sparseParams, velocityLevelIdx, velocityOutput_format };

        // color map
        NvFlowRayMarchUpdateColormapPinsIn colormapPinsIn = {};
        NvFlowRayMarchUpdateColormapPinsOut colormapPinsOut = {};
        colormapPinsIn.contextInterface = &ptr->contextInterface;
        colormapPinsIn.context = context;
        colormapPinsIn.params = ptr->rayMarchColormapParams.data;
        colormapPinsIn.paramCount = ptr->rayMarchColormapParams.size;
        //colormapPinsOut.colormap = colormap;
        NvFlowTextureTransient** colormap = &colormapPinsOut.colormap;

        NvFlowRayMarchUpdateColormap_execute(&ptr->mRayMarchUpdateColormap, &colormapPinsIn, &colormapPinsOut);

        NvFlowSparseTexture* shadowOutput = nullptr;
        NvFlowSparseTexture* coarseShadowOutput = nullptr;

        NvFlowDebugVolumePinsIn debugVolumePinsIn = {};
        NvFlowDebugVolumePinsOut debugVolumePinsOut = {};
        debugVolumePinsIn.contextInterface = &ptr->contextInterface;
        debugVolumePinsIn.context = context;
        debugVolumePinsIn.params = ptr->debugVolumeParams.data;
        debugVolumePinsIn.paramCount = ptr->debugVolumeParams.size;
        debugVolumePinsIn.velocity = velocityOutput;
        debugVolumePinsIn.densityShadow = densityOutput;
        debugVolumePinsIn.coarseDensityShadow = coarseDensityOutput;
        debugVolumePinsIn.isPreShadow = NV_FLOW_TRUE;
        shadowOutput = &debugVolumePinsOut.densityShadow;
        coarseShadowOutput = &debugVolumePinsOut.coarseDensityShadow;

        NvFlowDebugVolume_execute(&ptr->mDebugVolume, &debugVolumePinsIn, &debugVolumePinsOut);

        NvFlowShadowPinsIn shadowPinsIn = {};
        NvFlowShadowPinsOut shadowPinsOut = {};
        shadowPinsIn.contextInterface = &ptr->contextInterface;
        shadowPinsIn.context = context;
        shadowPinsIn.params = ptr->shadowParams.data;
        shadowPinsIn.paramCount = ptr->shadowParams.size;
        shadowPinsIn.colormap = *colormap;
        shadowPinsIn.density = *shadowOutput;
        shadowPinsIn.coarseDensity = *coarseShadowOutput;
        //shadowPins.densityShadowOut = &shadowOutput;
        shadowOutput = &shadowPinsOut.densityShadow;

        NvFlowShadow_execute(&ptr->mShadow, &shadowPinsIn, &shadowPinsOut);

        //NvFlowDebugVolumePinsIn debugVolumePinsIn = {};
        //NvFlowDebugVolumePinsOut debugVolumePinsOut = {};
        debugVolumePinsIn.contextInterface = &ptr->contextInterface;
        debugVolumePinsIn.context = context;
        debugVolumePinsIn.params = ptr->debugVolumeParams.data;
        debugVolumePinsIn.paramCount = ptr->debugVolumeParams.size;
        debugVolumePinsIn.velocity = velocityOutput;
        debugVolumePinsIn.densityShadow = *shadowOutput;
        debugVolumePinsIn.coarseDensityShadow = *coarseShadowOutput;
        debugVolumePinsIn.isPreShadow = NV_FLOW_FALSE;
        shadowOutput = &debugVolumePinsOut.densityShadow;
        coarseShadowOutput = &debugVolumePinsOut.coarseDensityShadow;

        NvFlowDebugVolume_execute(&ptr->mDebugVolume, &debugVolumePinsIn, &debugVolumePinsOut);

        // if offscreen pass ends up passthrough, assign variable as null to avoid double ownership
        if (shadowOutput->textureTransient == densityOutput.textureTransient)
        {
            NvFlowTextureVariable_set(context, &ptr->shadowVariable, nullptr, eNvFlowFormat_unknown);
        }
        else
        {
            NvFlowTextureVariable_set(context, &ptr->shadowVariable, shadowOutput->textureTransient, shadowOutput->format);
        }
        NvFlowTextureVariable_set(context, &ptr->colormapVariable, *colormap, eNvFlowFormat_r16g16b16a16_float);
    }

    void getRenderData(
        NvFlowContext* context,
        NvFlowGrid* grid,
        NvFlowGridRenderData* renderData
        )
    {
        auto ptr = cast(grid);

        NvFlowBufferTransient* sparseBuffer = ptr->sparseInterface.getSparseBuffer(context, ptr->sparse);
        NvFlowTextureTransient* shadowOutput_tex = NvFlowTextureVariable_get(context, &ptr->shadowVariable, nullptr);
        NvFlowTextureTransient* velocity_tex = NvFlowTextureVariable_get(context, &ptr->velocityVariable, nullptr);
        NvFlowTextureTransient* colormap = NvFlowTextureVariable_get(context, &ptr->colormapVariable, nullptr);

        NvFlowTextureTransient* densityOutput_tex = NvFlowTextureVariable_get(context, &ptr->densityInterpVariable, nullptr);
        if (!densityOutput_tex)
        {
            densityOutput_tex = NvFlowTextureVariable_get(context, &ptr->densityVariable, nullptr);
        }

        NvFlowSparseParams sparseParams = {};
        ptr->sparseInterface.getParams(ptr->sparse, &sparseParams);

        renderData->sparseBuffer = sparseBuffer;
        renderData->densityTexture = shadowOutput_tex ? shadowOutput_tex : densityOutput_tex;
        renderData->velocityTexture = velocity_tex;
        renderData->colormap = colormap;
        renderData->sparseParams = sparseParams;

        if (renderData->densityTexture && ptr->densityVariable.hintTexDesc.format == eNvFlowFormat_r8g8b8a8_unorm)
        {
            if (ptr->contextInterface.isFeatureSupported(context, eNvFlowContextFeature_aliasResourceFormats))
            {
                renderData->densityTexture = ptr->contextInterface.aliasTextureTransient(context, renderData->densityTexture, eNvFlowFormat_r8g8b8a8_unorm_srgb);
            }
        }

        renderData->nanoVdb = ptr->gridRenderDataNanoVdb;

        renderData->nanoVdb.temperatureNanoVdb = NvFlowBufferVariable_get(context, &ptr->temperatureNanoVdbVariable);
        renderData->nanoVdb.fuelNanoVdb = NvFlowBufferVariable_get(context, &ptr->fuelNanoVdbVariable);
        renderData->nanoVdb.burnNanoVdb = NvFlowBufferVariable_get(context, &ptr->burnNanoVdbVariable);
        renderData->nanoVdb.smokeNanoVdb = NvFlowBufferVariable_get(context, &ptr->smokeNanoVdbVariable);
        renderData->nanoVdb.velocityNanoVdb = NvFlowBufferVariable_get(context, &ptr->velocityNanoVdbVariable);
        renderData->nanoVdb.divergenceNanoVdb = NvFlowBufferVariable_get(context, &ptr->divergenceNanoVdbVariable);
    }

    void render(
        NvFlowContext* context,
        NvFlowGrid* grid,
        const NvFlowGridParamsDesc* paramsIn,
        const NvFlowFloat4x4* view,
        const NvFlowFloat4x4* projection,
        const NvFlowFloat4x4* projectionJittered,
        NvFlowUint width,
        NvFlowUint height,
        NvFlowUint sceneDepthWidth,
        NvFlowUint sceneDepthHeight,
        float compositeColorScale,
        NvFlowTextureTransient* sceneDepthIn,
        NvFlowFormat sceneColorFormat,
        NvFlowTextureTransient* sceneColorIn,
        NvFlowTextureTransient** pSceneColorOut
    )
    {
        auto ptr = cast(grid);

        NvFlowGridRenderData renderData = {};
        getRenderData(context, grid, &renderData);
        const NvFlowSparseParams* sparseParams = &renderData.sparseParams;

        // For now, fetching only the latest
        NvFlowDatabaseSnapshot snapshot = {};
        if (paramsIn->snapshotCount > 0u)
        {
            snapshot = paramsIn->snapshots[paramsIn->snapshotCount - 1u].snapshot;
        }
        NV_FLOW_DATABASE_SNAPSHOT_FIND_TYPE_ARRAY(&snapshot, NvFlowGridRenderLayerParams)

        // generate render params array that aligns with sparseParams layers
        ptr->renderParams.size = 0u;
        ptr->renderParams.reserve(sparseParams->layerCount);
        ptr->renderParams.size = sparseParams->layerCount;
        for (NvFlowUint layerParamIdx = 0u; layerParamIdx < sparseParams->layerCount; layerParamIdx++)
        {
            const NvFlowSparseLayerParams* layerParams = &sparseParams->layers[layerParamIdx];
            ptr->renderParams[layerParamIdx] = nullptr;
            for (NvFlowUint readIdx = 0u; readIdx < NvFlowGridRenderLayerParams_elementCount; readIdx++)
            {
                int renderLayerAndLevel = NvFlow_packLayerAndLevel(NvFlowGridRenderLayerParams_elements[readIdx]->layer,
                    NvFlowGridRenderLayerParams_elements[readIdx]->level);
                if (renderLayerAndLevel == layerParams->layerAndLevel)
                {
                    ptr->renderParams[layerParamIdx] = NvFlowGridRenderLayerParams_elements[readIdx];
                    break;
                }
            }
            if (!ptr->renderParams[layerParamIdx])
            {
                ptr->renderParams[layerParamIdx] = &ptr->renderDefault;
            }
        }

        // generate layerParamIdx aligned lists for operators that need them
        ptr->rayMarchParams.reserve(ptr->renderParams.size);
        ptr->rayMarchParams.size = ptr->renderParams.size;
        for (NvFlowUint layerParamIdx = 0u; layerParamIdx < ptr->renderParams.size; layerParamIdx++)
        {
            ptr->rayMarchParams[layerParamIdx] = &ptr->renderParams[layerParamIdx]->rayMarch;
        }

        NvFlowRayMarchTargetTexture target = {};
        target.view = view;
        target.projection = projection;
        target.projectionJittered = projectionJittered;
        target.textureWidth = width;
        target.textureHeight = height;
        target.sceneDepthWidth = sceneDepthWidth;
        target.sceneDepthHeight = sceneDepthHeight;
        target.sceneDepthIn = sceneDepthIn;
        target.sceneColorFormat = sceneColorFormat;
        target.sceneColorIn = sceneColorIn;
        target.pSceneColorOut = pSceneColorOut;

        {

            NvFlowSparseTexture velocityOutput = { renderData.velocityTexture, renderData.sparseBuffer, renderData.sparseParams, velocityLevelIdx, eNvFlowFormat_r16g16b16a16_float };
            NvFlowSparseTexture shadowOutput = { renderData.densityTexture, renderData.sparseBuffer, renderData.sparseParams, densityLevelIdx, eNvFlowFormat_r16g16b16a16_float };

            NvFlowRayMarchPinsIn pinsIn = {};
            NvFlowRayMarchPinsOut pinsOut = {};
            pinsIn.contextInterface = &ptr->contextInterface;
            pinsIn.context = context;
            pinsIn.params = ptr->rayMarchParams.data;
            pinsIn.paramCount = ptr->rayMarchParams.size;
            pinsIn.velocity = velocityOutput;
            pinsIn.density = shadowOutput;
            pinsIn.colormap = renderData.colormap;
            pinsIn.target = &target;
            pinsIn.compositeColorScale = compositeColorScale;

            // draw
            NvFlowRayMarch_execute(&ptr->mRayMarch, &pinsIn, &pinsOut);
        }
    }

    void updateIsosurface(
        NvFlowContext* context,
        NvFlowGrid* grid,
        const NvFlowGridParamsDesc* paramsIn
        )
    {
        auto ptr = cast(grid);

        NV_FLOW_PROFILE_BEGIN(60, 45)
        NV_FLOW_PROFILE_TIMESTAMP("UpdateIsosurfaceBegin")

        // For now, fetching only the latest
        NvFlowDatabaseSnapshot snapshot = {};
        if (paramsIn->snapshotCount > 0u)
        {
            snapshot = paramsIn->snapshots[paramsIn->snapshotCount - 1u].snapshot;
        }
        NV_FLOW_DATABASE_SNAPSHOT_FIND_TYPE_ARRAY(&snapshot, NvFlowGridIsosurfaceLayerParams)

        // generate isosurface params array that avoids duplicates
        ptr->isosurfaceParams.size = 0u;
        for (NvFlowUint readIdx = 0u; readIdx < NvFlowGridIsosurfaceLayerParams_elementCount; readIdx++)
        {
            NvFlowUint writeIdx = 0u;
            for (; writeIdx < ptr->isosurfaceParams.size; writeIdx++)
            {
                if (ptr->isosurfaceParams[writeIdx]->layer == NvFlowGridIsosurfaceLayerParams_elements[readIdx]->layer)
                {
                    ptr->isosurfaceParams[writeIdx] = NvFlowGridIsosurfaceLayerParams_elements[readIdx];
                    break;
                }
            }
            if (writeIdx == ptr->isosurfaceParams.size)
            {
                ptr->isosurfaceParams.pushBack(NvFlowGridIsosurfaceLayerParams_elements[readIdx]);
            }
        }

        // generate layerParamIdx aligned lists for operators that need them
        ptr->ellipsoidRasterParams.reserve(ptr->isosurfaceParams.size);
        ptr->ellipsoidRasterParams.size = ptr->isosurfaceParams.size;
        for (NvFlowUint layerParamIdx = 0u; layerParamIdx < ptr->isosurfaceParams.size; layerParamIdx++)
        {
            ptr->ellipsoidRasterParams[layerParamIdx] = &ptr->isosurfaceParams[layerParamIdx]->ellipsoidRaster;
        }

        // Standard block size
        NvFlowUint3 baseBlockDimBits = { 5u, 4u, 4u };
        NvFlowUint3 baseBlockDim = {
            1u << baseBlockDimBits.x,
            1u << baseBlockDimBits.y,
            1u << baseBlockDimBits.z
        };

        // allows multiple attempts to updateLayers/updateLocations to be tracked properly
        ptr->updateIdIsosurface++;

        // update layers
        ptr->updateIsosurfaceLayerParams.reserve(ptr->isosurfaceParams.size);
        ptr->updateIsosurfaceLayerParams.size = ptr->isosurfaceParams.size;
        for (NvFlowUint layerParamIdx = 0u; layerParamIdx < ptr->isosurfaceParams.size; layerParamIdx++)
        {
            NvFlowFloat3 blockSizeWorld = NvFlowFloat3{
                float(baseBlockDim.x) * fmaxf(0.f, ptr->isosurfaceParams[layerParamIdx]->densityCellSize),
                float(baseBlockDim.y) * fmaxf(0.f, ptr->isosurfaceParams[layerParamIdx]->densityCellSize),
                float(baseBlockDim.z) * fmaxf(0.f, ptr->isosurfaceParams[layerParamIdx]->densityCellSize)
            };

            ptr->updateIsosurfaceLayerParams[layerParamIdx].blockSizeWorld = blockSizeWorld;
            ptr->updateIsosurfaceLayerParams[layerParamIdx].layer = ptr->isosurfaceParams[layerParamIdx]->layer;
        }
        ptr->sparseInterface.updateLayers(ptr->sparseIsosurface, ptr->updateIdIsosurface, ptr->updateIsosurfaceLayerParams.data, (NvFlowUint)ptr->updateIsosurfaceLayerParams.size);

        NvFlowSparseParams sparseParams = {};
        ptr->sparseInterface.getParams(ptr->sparseIsosurface, &sparseParams);

        ptr->locationsIsosurface.size = 0u;

        // allocate
        NvFlowEllipsoidRasterAllocatePinsIn splatPinsIn = {};
        NvFlowEllipsoidRasterAllocatePinsOut splatPinsOut = {};
        {
            splatPinsIn.contextInterface = &ptr->contextInterface;
            splatPinsIn.context = context;
            splatPinsIn.sparseParams = sparseParams;
            splatPinsIn.params = ptr->ellipsoidRasterParams.data;
            splatPinsIn.paramCount = ptr->ellipsoidRasterParams.size;

            NvFlowEllipsoidRasterAllocate_execute(&ptr->mEllipsoidRasterAllocate, &splatPinsIn, &splatPinsOut);

            for (NvFlowUint idx = 0u; idx < splatPinsOut.locationCount; idx++)
            {
                ptr->locationsIsosurface.pushBack(splatPinsOut.locations[idx]);
            }
        }

        NV_FLOW_PROFILE_TIMESTAMP("EllipsoidRasterAllocate")

        ptr->sparseInterface.updateLocations(context, ptr->sparseIsosurface, ptr->updateIdIsosurface, ptr->locationsIsosurface.data, (NvFlowUint)ptr->locationsIsosurface.size, baseBlockDimBits, 0u);

        NV_FLOW_PROFILE_TIMESTAMP("UpdateLocations")

        NvFlowBufferTransient* sparseBuffer = nullptr;
        ptr->sparseInterface.addPasses(context, ptr->sparseIsosurface, &sparseBuffer);

        ptr->sparseInterface.getParams(ptr->sparseIsosurface, &sparseParams);

        NV_FLOW_PROFILE_TIMESTAMP("SparseBuffer")

        if (sparseParams.levelCount > 0u)
        {
            ptr->activeBlockCountIsosurface = sparseParams.levels[0u].numLocations;
        }

        // splat and smooth
        {
            NvFlowSparseTexture* splatDensity = nullptr;

            NvFlowSparseTexture splatTemplate = { nullptr, sparseBuffer, sparseParams, densityLevelIdx, eNvFlowFormat_r16_float };

            NvFlowEllipsoidRasterPinsIn ellipsoidRasterPinsIn = {};
            NvFlowEllipsoidRasterPinsOut ellipsoidRasterPinsOut = {};
            ellipsoidRasterPinsIn.contextInterface = &ptr->contextInterface;
            ellipsoidRasterPinsIn.context = context;
            ellipsoidRasterPinsIn.feedback = splatPinsOut.feedback;
            ellipsoidRasterPinsIn.params = ptr->ellipsoidRasterParams.data;
            ellipsoidRasterPinsIn.paramCount = ptr->ellipsoidRasterParams.size;
            ellipsoidRasterPinsIn.layout = splatTemplate;                    // Note: only used as template
            //ellipsoidRasterPins.valueOut = &splatDensityRaw;
            splatDensity = &ellipsoidRasterPinsOut.value;

            NvFlowEllipsoidRaster_execute(&ptr->mEllipsoidRaster, &ellipsoidRasterPinsIn, &ellipsoidRasterPinsOut);

            NvFlowTextureVariable_set(context, &ptr->splatDensityVariable, splatDensity->textureTransient, splatDensity->format);
        }

        NV_FLOW_PROFILE_TIMESTAMP("EllipsoidRaster")

        NV_FLOW_PROFILE_TIMESTAMP("UpdateIsosurfaceEnd")
        NV_FLOW_PROFILE_FLUSH("UpdateIsosurface", ptr->contextInterface.getLogPrint(context))
    }

    void getIsosurfaceData(
        NvFlowContext* context,
        NvFlowGrid* grid,
        NvFlowGridIsosurfaceData* isosurfaceData
        )
    {
        auto ptr = cast(grid);

        NvFlowBufferTransient* sparseBuffer = ptr->sparseInterface.getSparseBuffer(context, ptr->sparseIsosurface);
        NvFlowTextureTransient* splatDensity = NvFlowTextureVariable_get(context, &ptr->splatDensityVariable, nullptr);

        NvFlowSparseParams sparseParams = {};
        ptr->sparseInterface.getParams(ptr->sparseIsosurface, &sparseParams);

        isosurfaceData->sparseBuffer = sparseBuffer;
        isosurfaceData->densityTexture = splatDensity;
        isosurfaceData->sparseParams = sparseParams;
    }

    void renderIsosurface(
        NvFlowContext* context,
        NvFlowGrid* grid,
        const NvFlowGridParamsDesc* paramsIn,
        const NvFlowFloat4x4* view,
        const NvFlowFloat4x4* projection,
        const NvFlowFloat4x4* projectionJittered,
        NvFlowUint width,
        NvFlowUint height,
        NvFlowUint sceneDepthWidth,
        NvFlowUint sceneDepthHeight,
        float compositeColorScale,
        NvFlowTextureTransient* sceneDepthIn,
        NvFlowFormat sceneColorFormat,
        NvFlowTextureTransient* sceneColorIn,
        NvFlowTextureTransient** pSceneColorOut
        )
    {
        auto ptr = cast(grid);

        // For now, fetching only the latest
        NvFlowDatabaseSnapshot snapshot = {};
        if (paramsIn->snapshotCount > 0u)
        {
            snapshot = paramsIn->snapshots[paramsIn->snapshotCount - 1u].snapshot;
        }
        NV_FLOW_DATABASE_SNAPSHOT_FIND_TYPE_ARRAY(&snapshot, NvFlowGridIsosurfaceLayerParams)

        NvFlowRayMarchTargetTexture target = {};
        target.view = view;
        target.projection = projection;
        target.projectionJittered = projectionJittered;
        target.textureWidth = width;
        target.textureHeight = height;
        target.sceneDepthWidth = sceneDepthWidth;
        target.sceneDepthHeight = sceneDepthHeight;
        target.sceneDepthIn = sceneDepthIn;
        target.sceneColorFormat = sceneColorFormat;
        target.sceneColorIn = sceneColorIn;
        target.pSceneColorOut = pSceneColorOut;

        // update any changes to isosurface render params for active layers
        for (NvFlowUint readIdx = 0u; readIdx < NvFlowGridIsosurfaceLayerParams_elementCount; readIdx++)
        {
            for (NvFlowUint writeIdx = 0u; writeIdx < ptr->isosurfaceParams.size; writeIdx++)
            {
                if (ptr->isosurfaceParams[writeIdx]->layer == NvFlowGridIsosurfaceLayerParams_elements[readIdx]->layer)
                {
                    ptr->isosurfaceParams[writeIdx] = NvFlowGridIsosurfaceLayerParams_elements[readIdx];
                    break;
                }
            }
        }

        // generate layerParamIdx aligned lists for operators that need them
        ptr->rayMarchIsosurfaceParams.reserve(ptr->isosurfaceParams.size);
        ptr->rayMarchIsosurfaceParams.size = ptr->isosurfaceParams.size;
        for (NvFlowUint layerParamIdx = 0u; layerParamIdx < ptr->isosurfaceParams.size; layerParamIdx++)
        {
            ptr->rayMarchIsosurfaceParams[layerParamIdx] = &ptr->isosurfaceParams[layerParamIdx]->rayMarchIsosurface;
        }

        NvFlowTextureTransient* splatDensity_tex = NvFlowTextureVariable_get(context, &ptr->splatDensityVariable, nullptr);
        if (splatDensity_tex)
        {
            NvFlowBufferTransient* sparseBuffer = ptr->sparseInterface.getSparseBuffer(context, ptr->sparseIsosurface);
            NvFlowTextureTransient* colormap = NvFlowTextureVariable_get(context, &ptr->colormapVariable, nullptr);

            NvFlowSparseParams sparseParams = {};
            ptr->sparseInterface.getParams(ptr->sparseIsosurface, &sparseParams);

            NvFlowSparseTexture splatDensity = { splatDensity_tex, sparseBuffer, sparseParams, densityLevelIdx, eNvFlowFormat_r16_float };

            NvFlowRayMarchIsosurfacePinsIn pinsIn = {};
            NvFlowRayMarchIsosurfacePinsOut pinsOut = {};
            pinsIn.contextInterface = &ptr->contextInterface;
            pinsIn.context = context;
            pinsIn.params = ptr->rayMarchIsosurfaceParams.data;
            pinsIn.paramCount = ptr->rayMarchIsosurfaceParams.size;
            pinsIn.density = splatDensity;
            pinsIn.target = &target;
            pinsIn.compositeColorScale = compositeColorScale;

            NvFlowRayMarchIsosurface_execute(&ptr->mRayMarchIsosurface, &pinsIn, &pinsOut);
        }
        else
        {
            // passthrough
            *pSceneColorOut = sceneColorIn;
        }
    }

    void copyTexture(
        NvFlowContext* context,
        NvFlowGrid* grid,
        NvFlowUint width,
        NvFlowUint height,
        NvFlowFormat sceneColorFormat,
        NvFlowTextureTransient* sceneColorIn,
        NvFlowTextureTransient** pSceneColorOut
        )
    {
        auto ptr = cast(grid);

        NvFlowRayMarchCopyTexturePinsIn pinsIn = {};
        NvFlowRayMarchCopyTexturePinsOut pinsOut = {};
        pinsIn.contextInterface = &ptr->contextInterface;
        pinsIn.context = context;
        pinsIn.texture = sceneColorIn;
        pinsIn.width = width;
        pinsIn.height = height;
        pinsIn.format = sceneColorFormat;

        NvFlowRayMarchCopyTexture_execute(&ptr->mRayMarchCopyTexture, &pinsIn, &pinsOut);

        if (pSceneColorOut)
        {
            *pSceneColorOut = pinsOut.texture;
        }
    }

    NvFlowUint getActiveBlockCount(NvFlowGrid* grid)
    {
        auto ptr = cast(grid);

        return ptr->activeBlockCount;
    }

    NvFlowUint getActiveBlockCountIsosurface(NvFlowGrid* grid)
    {
        auto ptr = cast(grid);

        return ptr->activeBlockCountIsosurface;
    }

    void setResourceMinLifetime(NvFlowContext* context, NvFlowGrid* grid, NvFlowUint64 minLifetime)
    {
        // NOP
    }
}

NvFlowGridInterface* NvFlowGetGridInterfaceNoOpt()
{
    using namespace NvFlowGridSimple;
    static NvFlowGridInterface iface = { NV_FLOW_REFLECT_INTERFACE_INIT(NvFlowGridInterface) };
    iface.createGrid = createGrid;
    iface.destroyGrid = destroyGrid;
    iface.resetGrid = resetGrid;
    iface.simulate = simulate;
    iface.offscreen = offscreen;
    iface.getRenderData = getRenderData;
    iface.render = render;
    iface.updateIsosurface = updateIsosurface;
    iface.getIsosurfaceData = getIsosurfaceData;
    iface.renderIsosurface = renderIsosurface;
    iface.copyTexture = copyTexture;
    iface.getActiveBlockCount = getActiveBlockCount;
    iface.getActiveBlockCountIsosurface = getActiveBlockCountIsosurface;
    iface.setResourceMinLifetime = setResourceMinLifetime;
    return &iface;
}
