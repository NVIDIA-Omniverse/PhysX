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

#include "NvFlowTimer.h"

namespace NvFlowGridOpt
{
    struct Grid
    {
        NvFlowContextOptInterface contextOptInterface = {};
        NvFlowContextOpt* contextOpt = nullptr;
        NvFlowContextInterface contextInterface = {};
        NvFlowContext* context = nullptr;

        NvFlowGridInterface backendGridInterface = {};
        NvFlowGrid* backendGrid = nullptr;
    };

    NV_FLOW_CAST_PAIR(NvFlowGrid, Grid)

    NvFlowGrid* createGrid(NvFlowContextInterface* backendContextInterface, NvFlowContext* backendContext, NvFlowOpList* opListIn, NvFlowExtOpList* extOpListIn, const NvFlowGridDesc* desc)
    {
        auto ptr = new Grid();

        NvFlowContextOptInterface_duplicate(&ptr->contextOptInterface, NvFlowGetContextOptInterface());

        ptr->contextOpt = ptr->contextOptInterface.create(backendContextInterface, backendContext);

        NvFlowContextInterface* pContextInterface = nullptr;
        ptr->contextOptInterface.getContext(ptr->contextOpt, &pContextInterface, &ptr->context);
        NvFlowContextInterface_duplicate(&ptr->contextInterface, pContextInterface);

        NvFlowGridInterface_duplicate(&ptr->backendGridInterface, NvFlowGetGridInterfaceNoOpt());

        ptr->backendGrid = ptr->backendGridInterface.createGrid(&ptr->contextInterface, ptr->context, opListIn, extOpListIn, desc);

        ptr->contextOptInterface.flush(ptr->contextOpt);

        return cast(ptr);
    }

    void destroyGrid(NvFlowContext* backendContext, NvFlowGrid* grid)
    {
        auto ptr = cast(grid);

        ptr->backendGridInterface.destroyGrid(ptr->context, ptr->backendGrid);

        ptr->contextOptInterface.flush(ptr->contextOpt);

        ptr->contextOptInterface.destroy(ptr->contextOpt);

        delete ptr;
    }

    void resetGrid(
        NvFlowContext* context,
        NvFlowGrid* grid,
        const NvFlowGridDesc* desc
    )
    {
        auto ptr = cast(grid);

        ptr->backendGridInterface.resetGrid(ptr->context, ptr->backendGrid, desc);

        ptr->contextOptInterface.flush(ptr->contextOpt);
    }

    void simulate(
        NvFlowContext* backendContext,
        NvFlowGrid* grid,
        const NvFlowGridParamsDesc* paramsIn,
        NvFlowBool32 globalForceClear
    )
    {
        auto ptr = cast(grid);

        NV_FLOW_PROFILE_BEGIN(60, 30)
        NV_FLOW_PROFILE_TIMESTAMP("OptSimulateBegin")

        // a pre flush can reduce latency
        ptr->contextOptInterface.flush(ptr->contextOpt);

        NV_FLOW_PROFILE_TIMESTAMP("OptFlush")

        ptr->backendGridInterface.simulate(ptr->context, ptr->backendGrid, paramsIn, globalForceClear);

        NV_FLOW_PROFILE_TIMESTAMP("OptSimulate")

        ptr->contextOptInterface.flush(ptr->contextOpt);

        NV_FLOW_PROFILE_TIMESTAMP("OptSimulateFlush")
        NV_FLOW_PROFILE_FLUSH("OptSimulate", ptr->contextInterface.getLogPrint(ptr->context))
    }

    void offscreen(
        NvFlowContext* backendContext,
        NvFlowGrid* grid,
        const NvFlowGridParamsDesc* paramsIn
    )
    {
        auto ptr = cast(grid);

        ptr->backendGridInterface.offscreen(ptr->context, ptr->backendGrid, paramsIn);

        ptr->contextOptInterface.flush(ptr->contextOpt);
    }

    void getRenderData(
        NvFlowContext* backendContext,
        NvFlowGrid* grid,
        NvFlowGridRenderData* renderData_extern
    )
    {
        auto ptr = cast(grid);

        NvFlowGridRenderData renderData = {};
        ptr->backendGridInterface.getRenderData(ptr->context, ptr->backendGrid, &renderData);

        if (renderData_extern)
        {
            renderData_extern->sparseParams = renderData.sparseParams;

            ptr->contextOptInterface.exportBufferTransient(ptr->contextOpt, renderData.sparseBuffer, &renderData_extern->sparseBuffer);
            ptr->contextOptInterface.exportTextureTransient(ptr->contextOpt, renderData.densityTexture, &renderData_extern->densityTexture);
            ptr->contextOptInterface.exportTextureTransient(ptr->contextOpt, renderData.velocityTexture, &renderData_extern->velocityTexture);
            ptr->contextOptInterface.exportTextureTransient(ptr->contextOpt, renderData.colormap, &renderData_extern->colormap);

            renderData_extern->nanoVdb = renderData.nanoVdb;

            renderData_extern->nanoVdb.temperatureNanoVdb = nullptr;
            renderData_extern->nanoVdb.fuelNanoVdb = nullptr;
            renderData_extern->nanoVdb.burnNanoVdb = nullptr;
            renderData_extern->nanoVdb.smokeNanoVdb = nullptr;
            renderData_extern->nanoVdb.velocityNanoVdb = nullptr;
            renderData_extern->nanoVdb.divergenceNanoVdb = nullptr;

            ptr->contextOptInterface.exportBufferTransient(ptr->contextOpt, renderData.nanoVdb.temperatureNanoVdb, &renderData_extern->nanoVdb.temperatureNanoVdb);
            ptr->contextOptInterface.exportBufferTransient(ptr->contextOpt, renderData.nanoVdb.fuelNanoVdb, &renderData_extern->nanoVdb.fuelNanoVdb);
            ptr->contextOptInterface.exportBufferTransient(ptr->contextOpt, renderData.nanoVdb.burnNanoVdb, &renderData_extern->nanoVdb.burnNanoVdb);
            ptr->contextOptInterface.exportBufferTransient(ptr->contextOpt, renderData.nanoVdb.smokeNanoVdb, &renderData_extern->nanoVdb.smokeNanoVdb);
            ptr->contextOptInterface.exportBufferTransient(ptr->contextOpt, renderData.nanoVdb.velocityNanoVdb, &renderData_extern->nanoVdb.velocityNanoVdb);
            ptr->contextOptInterface.exportBufferTransient(ptr->contextOpt, renderData.nanoVdb.divergenceNanoVdb, &renderData_extern->nanoVdb.divergenceNanoVdb);
        }

        ptr->contextOptInterface.flush(ptr->contextOpt);
    }

    void render(
        NvFlowContext* backendContext,
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
        NvFlowTextureTransient* sceneDepthIn_extern,
        NvFlowFormat sceneColorFormat,
        NvFlowTextureTransient* sceneColorIn_extern,
        NvFlowTextureTransient** pSceneColorOut_extern
    )
    {
        auto ptr = cast(grid);

        NvFlowTextureTransient* sceneDepthIn = ptr->contextOptInterface.importBackendTextureTransient(ptr->contextOpt, sceneDepthIn_extern);
        NvFlowTextureTransient* sceneColorIn = ptr->contextOptInterface.importBackendTextureTransient(ptr->contextOpt, sceneColorIn_extern);
        NvFlowTextureTransient* sceneColorOut = nullptr;

        ptr->backendGridInterface.render(
            ptr->context,
            ptr->backendGrid,
            paramsIn,
            view,
            projection,
            projectionJittered,
            width,
            height,
            sceneDepthWidth,
            sceneDepthHeight,
            compositeColorScale,
            sceneDepthIn,
            sceneColorFormat,
            sceneColorIn,
            &sceneColorOut
        );

        ptr->contextOptInterface.exportTextureTransient(ptr->contextOpt, sceneColorOut, pSceneColorOut_extern);

        ptr->contextOptInterface.flush(ptr->contextOpt);
    }

    void updateIsosurface(
        NvFlowContext* backendContext,
        NvFlowGrid* grid,
        const NvFlowGridParamsDesc* paramsIn
    )
    {
        auto ptr = cast(grid);

        ptr->backendGridInterface.updateIsosurface(
            ptr->context,
            ptr->backendGrid,
            paramsIn
        );

        ptr->contextOptInterface.flush(ptr->contextOpt);
    }

    void getIsosurfaceData(
        NvFlowContext* backendContext,
        NvFlowGrid* grid,
        NvFlowGridIsosurfaceData* isosurfaceData_extern
    )
    {
        auto ptr = cast(grid);

        NvFlowGridIsosurfaceData isosurfaceData = {};
        ptr->backendGridInterface.getIsosurfaceData(ptr->context, ptr->backendGrid, &isosurfaceData);

        if (isosurfaceData_extern)
        {
            isosurfaceData_extern->sparseParams = isosurfaceData.sparseParams;

            ptr->contextOptInterface.exportBufferTransient(ptr->contextOpt, isosurfaceData.sparseBuffer, &isosurfaceData_extern->sparseBuffer);
            ptr->contextOptInterface.exportTextureTransient(ptr->contextOpt, isosurfaceData.densityTexture, &isosurfaceData_extern->densityTexture);
        }

        ptr->contextOptInterface.flush(ptr->contextOpt);
    }

    void renderIsosurface(
        NvFlowContext* backendContext,
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
        NvFlowTextureTransient* sceneDepthIn_extern,
        NvFlowFormat sceneColorFormat,
        NvFlowTextureTransient* sceneColorIn_extern,
        NvFlowTextureTransient** pSceneColorOut_extern
    )
    {
        auto ptr = cast(grid);

        NvFlowTextureTransient* sceneDepthIn = ptr->contextOptInterface.importBackendTextureTransient(ptr->contextOpt, sceneDepthIn_extern);
        NvFlowTextureTransient* sceneColorIn = ptr->contextOptInterface.importBackendTextureTransient(ptr->contextOpt, sceneColorIn_extern);
        NvFlowTextureTransient* sceneColorOut = nullptr;

        ptr->backendGridInterface.renderIsosurface(
            ptr->context,
            ptr->backendGrid,
            paramsIn,
            view,
            projection,
            projectionJittered,
            width,
            height,
            sceneDepthWidth,
            sceneDepthHeight,
            compositeColorScale,
            sceneDepthIn,
            sceneColorFormat,
            sceneColorIn,
            &sceneColorOut
        );

        ptr->contextOptInterface.exportTextureTransient(ptr->contextOpt, sceneColorOut, pSceneColorOut_extern);

        ptr->contextOptInterface.flush(ptr->contextOpt);
    }

    void copyTexture(
        NvFlowContext* backendContext,
        NvFlowGrid* grid,
        NvFlowUint width,
        NvFlowUint height,
        NvFlowFormat sceneColorFormat,
        NvFlowTextureTransient* sceneColorIn_extern,
        NvFlowTextureTransient** pSceneColorOut_extern
    )
    {
        auto ptr = cast(grid);

        NvFlowTextureTransient* sceneColorIn = ptr->contextOptInterface.importBackendTextureTransient(ptr->contextOpt, sceneColorIn_extern);
        NvFlowTextureTransient* sceneColorOut = nullptr;

        ptr->backendGridInterface.copyTexture(
            ptr->context,
            ptr->backendGrid,
            width,
            height,
            sceneColorFormat,
            sceneColorIn,
            &sceneColorOut
        );

        ptr->contextOptInterface.exportTextureTransient(ptr->contextOpt, sceneColorOut, pSceneColorOut_extern);

        ptr->contextOptInterface.flush(ptr->contextOpt);
    }

    NvFlowUint getActiveBlockCount(NvFlowGrid* grid)
    {
        auto ptr = cast(grid);

        return ptr->backendGridInterface.getActiveBlockCount(ptr->backendGrid);
    }

    NvFlowUint getActiveBlockCountIsosurface(NvFlowGrid* grid)
    {
        auto ptr = cast(grid);

        return ptr->backendGridInterface.getActiveBlockCountIsosurface(ptr->backendGrid);
    }

    void setResourceMinLifetime(NvFlowContext* context, NvFlowGrid* grid, NvFlowUint64 minLifetime)
    {
        auto ptr = cast(grid);
        ptr->contextOptInterface.setResourceMinLifetime(ptr->contextOpt, minLifetime);
    }
}

NvFlowGridInterface* NvFlowGetGridInterface()
{
    using namespace NvFlowGridOpt;
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
