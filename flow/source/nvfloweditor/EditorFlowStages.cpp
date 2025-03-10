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

#include "EditorCommon.h"

void editorFlowStage_applyOverrides(EditorFlow* ptr, float cellsizeOverride, NvFlowBool32 smallBlocksOverride)
{
    // by default, override colorScale
    editorFlow_setAttributeFloat(ptr, "offscreen/colormap", "colorScale", 1.f);

    if (cellsizeOverride > 0.f)
    {
        editorFlow_setAttributeFloat(ptr, "simulate", "densityCellSize", cellsizeOverride);
    }
    if (smallBlocksOverride)
    {
        editorFlow_setAttributeBool(ptr, "simulate", "enableSmallBlocks", NV_FLOW_TRUE);
    }
}

void* editorFlowStage_sphere_init(EditorFlow* ptr)
{
    editorFlow_definePrim(ptr, "FlowSimulate", "simulate", "simulate");
    editorFlow_definePrim(ptr, "FlowOffscreen", "offscreen", "offscreen");
    editorFlow_definePrim(ptr, "FlowRender", "render", "render");
    editorFlow_definePrim(ptr, "FlowEmitterSphere", "emitter", "emitter");
    return nullptr;
}
static const EditorFlowStage editorFlowStage_sphere = { "sphere", editorFlowStage_sphere_init, nullptr, nullptr };

void* editorFlowStage_box_init(EditorFlow* ptr)
{
    editorFlow_definePrim(ptr, "FlowSimulate", "simulate", "simulate");
    editorFlow_definePrim(ptr, "FlowOffscreen", "offscreen", "offscreen");
    editorFlow_definePrim(ptr, "FlowRender", "render", "render");
    editorFlow_definePrim(ptr, "FlowEmitterBox", "emitter", "emitter");
    return nullptr;
}
static const EditorFlowStage editorFlowStage_box = { "box", editorFlowStage_box_init, nullptr, nullptr };

void* editorFlowStage_point_init(EditorFlow* ptr)
{
    editorFlow_definePrim(ptr, "FlowSimulate", "simulate", "simulate");
    editorFlow_definePrim(ptr, "FlowOffscreen", "offscreen", "offscreen");
    editorFlow_definePrim(ptr, "FlowRender", "render", "render");
    editorFlow_definePrim(ptr, "FlowEmitterPoint", "emitter", "emitter");

    editorFlow_setAttributeFloat(ptr, "simulate/advection/velocity", "secondOrderBlendFactor", 0.5f);
    editorFlow_setAttributeFloat(ptr, "simulate/advection/temperature", "secondOrderBlendFactor", 0.5f);
    editorFlow_setAttributeFloat(ptr, "simulate/advection/fuel", "secondOrderBlendFactor", 0.5f);
    editorFlow_setAttributeFloat(ptr, "simulate/advection/smoke", "secondOrderBlendFactor", 0.5f);

    editorFlow_setAttributeBool(ptr, "offscreen/debugVolume", "enableSpeedAsTemperature", NV_FLOW_TRUE);

    editorFlow_setAttributeFloat(ptr, "render/rayMarch", "attenuation", 0.5f);
    editorFlow_setAttributeBool(ptr, "render/rayMarch", "enableRawMode", NV_FLOW_TRUE);
    return nullptr;
}
void editorFlowStage_point_update(EditorFlow* ptr, void* userdata, double time, float deltaTime)
{
    float timef = (float)time;
    static NvFlowFloat3 positions[64u];
    for (int j = 0; j < 4; j++)
    {
        for (int i = 0; i < 16; i++)
        {
            positions[j * 16 + i].x = 20.f * (float(j) + 1.f) * cosf(6.28f * (float(i) + 0.33f * float(j) + 0.5f + timef) / 16.f);
            positions[j * 16 + i].y = 20.f * (float(j) + 1.f) * sinf(6.28f * (float(i) + 0.33f * float(j) + 0.5f + timef) / 16.f);
            positions[j * 16 + i].z = 0.f;
        }
    }
    editorFlow_setAttributeFloat3Array(ptr, "emitter", "pointPositions", positions, 64u);
}
static const EditorFlowStage editorFlowStage_point = { "point", editorFlowStage_point_init, editorFlowStage_point_update, nullptr };

void* editorFlowStage_mesh_init(EditorFlow* ptr)
{
    editorFlow_definePrim(ptr, "FlowSimulate", "simulate", "simulate");
    editorFlow_definePrim(ptr, "FlowOffscreen", "offscreen", "offscreen");
    editorFlow_definePrim(ptr, "FlowRender", "render", "render");
    editorFlow_definePrim(ptr, "FlowEmitterMesh", "emitter", "emitter");

    static const NvFlowUint instanceCount = 1u; // 768u;

    static const int faceCountCount = 6;
    static const int faceIndexCount = 24;
    static const int positionCount = 8;
    /*static*/ int faceCounts[instanceCount * faceCountCount] = { 4, 4, 4, 4, 4, 4 };
    /*static */int faceIndices[instanceCount * faceIndexCount] = { 0, 1, 3, 2, 0, 4, 5, 1, 1, 5, 6, 3, 2, 3, 6, 7, 0, 2, 7, 4, 4, 7, 6, 5 };
    /*static */const NvFlowFloat3 positions[positionCount] = {
        {-5.f, -5.f, -5.f}, {5.f, -5.f, -5.f}, {-5.f, -5.f, 5.f}, {5.f, -5.f, 5.f}, {-5.f, 5.f, -5.f}, {5.f, 5.f, -5.f}, {5.f, 5.f, 5.f}, {-5.f, 5.f, 5.f}
    };

    for (int instance = 1; instance < instanceCount; instance++)
    {
        for (int faceCountIdx = 0u; faceCountIdx < faceCountCount; faceCountIdx++)
        {
            faceCounts[faceCountCount * instance + faceCountIdx] = faceCounts[faceCountIdx];
        }
        for (int faceIndicesIdx = 0u; faceIndicesIdx < faceIndexCount; faceIndicesIdx++)
        {
            faceIndices[faceIndexCount * instance + faceIndicesIdx] = faceIndices[faceIndicesIdx];
        }
    }

    editorFlow_setAttributeIntArray(ptr, "emitter", "meshFaceVertexCounts", faceCounts, instanceCount * faceCountCount);
    editorFlow_setAttributeIntArray(ptr, "emitter", "meshFaceVertexIndices", faceIndices, instanceCount * faceIndexCount);
    editorFlow_setAttributeFloat3Array(ptr, "emitter", "meshPositions", positions, positionCount);
    return nullptr;
}
static const EditorFlowStage editorFlowStage_mesh = { "mesh", editorFlowStage_mesh_init, nullptr, nullptr };

void* editorFlowStage_texture_init(EditorFlow* ptr)
{
    editorFlow_definePrim(ptr, "FlowSimulate", "simulate", "simulate");
    editorFlow_definePrim(ptr, "FlowOffscreen", "offscreen", "offscreen");
    editorFlow_definePrim(ptr, "FlowRender", "render", "render");

    static const NvFlowUint textureDim = 256u;
    static float velocities[textureDim][textureDim][textureDim][3];
    for (NvFlowUint k = 0; k < textureDim; k++)
    {
        for (NvFlowUint j = 0; j < textureDim; j++)
        {
            for (NvFlowUint i = 0; i < textureDim; i++)
            {
                float u = (float(i) + 0.5f) / float(textureDim);
                float v = (float(j) + 0.5f) / float(textureDim);
                float vx = 200.f * v - 100.f;
                float vy = -200.f * u + 100.f;
                velocities[k][j][i][0] = vx;
                velocities[k][j][i][1] = vy;
                velocities[k][j][i][2] = 20.f;
            }
        }
    }
    NvFlowFloat3* texVel = (NvFlowFloat3*)&velocities[0][0][0][0];
    NvFlowUint64 texVelCount = textureDim * textureDim * textureDim;

    editorFlow_definePrim(ptr, "FlowEmitterTexture", "emitter", "emitter");
    editorFlow_setAttributeFloat3(ptr, "emitter", "halfSize", NvFlowFloat3{ 50.f, 50.f, 50.f });
    editorFlow_setAttributeFloat(ptr, "emitter", "coupleRateVelocity", 50.f);
    editorFlow_setAttributeBool(ptr, "emitter", "applyPostPressure", NV_FLOW_TRUE);
    editorFlow_setAttributeFloat(ptr, "emitter", "coupleRateTemperature", 0.f);
    editorFlow_setAttributeFloat(ptr, "emitter", "coupleRateFuel", 0.f);
    editorFlow_setAttributeFloat(ptr, "emitter", "coupleRateBurn", 0.f);
    editorFlow_setAttributeFloat(ptr, "emitter", "coupleRateSmoke", 0.f);
    editorFlow_setAttributeFloat3(ptr, "emitter", "velocity", NvFlowFloat3{ 0.f, 0.f, 0.f });
    editorFlow_setAttributeUint(ptr, "emitter", "textureWidth", textureDim);
    editorFlow_setAttributeUint(ptr, "emitter", "textureHeight", textureDim);
    editorFlow_setAttributeUint(ptr, "emitter", "textureDepth", textureDim);
    editorFlow_setAttributeFloat3Array(ptr, "emitter", "textureVelocities", texVel, texVelCount);

    editorFlow_definePrim(ptr, "FlowEmitterSphere", "emitterSphere", "emitterSphere");
    editorFlow_setAttributeFloat3(ptr, "emitterSphere", "position", NvFlowFloat3{ 10.f, 0.f, -40.f });
    editorFlow_setAttributeFloat(ptr, "emitterSphere", "radius", 5.f);
    editorFlow_setAttributeFloat(ptr, "emitterSphere", "coupleRateTemperature", 10.f);
    editorFlow_setAttributeFloat(ptr, "emitterSphere", "coupleRateFuel", 10.f);
    return nullptr;
}
static const EditorFlowStage editorFlowStage_texture = { "texture", editorFlowStage_texture_init, nullptr, nullptr };

void* editorFlowStage_nanovdb_init(EditorFlow* ptr)
{
    editorFlow_definePrim(ptr, "FlowSimulate", "simulate", "simulate");
    editorFlow_definePrim(ptr, "FlowOffscreen", "offscreen", "offscreen");
    editorFlow_definePrim(ptr, "FlowRender", "render", "render");
    editorFlow_definePrim(ptr, "FlowEmitterNanoVdb", "emitter", "emitter");
    return nullptr;
}
static const EditorFlowStage editorFlowStage_nanovdb = { "nanovdb", editorFlowStage_nanovdb_init, nullptr, nullptr };

struct EditorFlowStageIsosurface
{
    NvFlowArray<NvFlowFloat3> positionFloat3;
    NvFlowArray<NvFlowFloat4> anisotropyE1;
    NvFlowArray<NvFlowFloat4> anisotropyE2;
    NvFlowArray<NvFlowFloat4> anisotropyE3;
};
void* editorFlowStage_isosurface_init(EditorFlow* ptr)
{
    auto* state = new EditorFlowStageIsosurface();

    editorFlow_definePrim(ptr, "FlowIsosurface", "isosurface", "isosurface");

    for (NvFlowUint idx = 0u; idx < 128u; idx++)
    {
        state->positionFloat3.pushBack(NvFlowFloat3{ 0.f, 0.f, 0.f });
        state->anisotropyE1.pushBack(NvFlowFloat4{ 1.f, 0.f, 0.f, 4.f });
        state->anisotropyE2.pushBack(NvFlowFloat4{ 0.f, 1.f, 0.f, 4.f });
        state->anisotropyE3.pushBack(NvFlowFloat4{ 0.f, 0.f, 1.f, 4.f });
    }

    editorFlow_setAttributeFloat3Array(ptr, "isosurface/ellipsoidRaster", "positionFloat3s", state->positionFloat3.data, state->positionFloat3.size);
    editorFlow_setAttributeFloat4Array(ptr, "isosurface/ellipsoidRaster", "anisotropyE1s", state->anisotropyE1.data, state->anisotropyE1.size);
    editorFlow_setAttributeFloat4Array(ptr, "isosurface/ellipsoidRaster", "anisotropyE2s", state->anisotropyE2.data, state->anisotropyE2.size);
    editorFlow_setAttributeFloat4Array(ptr, "isosurface/ellipsoidRaster", "anisotropyE3s", state->anisotropyE3.data, state->anisotropyE3.size);

    return state;
}
void editorFlowStage_isosurface_update(EditorFlow* ptr, void* userdata, double time, float deltaTime)
{
    auto* state = (EditorFlowStageIsosurface*)userdata;

    // loop at 4 seconds
    double timeScaled = 0.25f * time;
    float timeLooped = 4.f * (float)(timeScaled - floor(timeScaled));

    float animT = 4.f * timeLooped;
    for (NvFlowUint idx = 0u; idx < state->positionFloat3.size; idx++)
    {
        float theta = 6.28f * float(idx) / float(state->positionFloat3.size);
        state->positionFloat3[idx].x = 4.f * cosf(theta + 0.1f * animT) * animT;
        state->positionFloat3[idx].y = 4.f * sinf(theta + 0.1f * animT) * animT;
        state->positionFloat3[idx].z = -32.f * (1.f / 64.f) * (animT - 8.f) * (animT - 8.f) + 32.f;
    }

    editorFlow_setAttributeFloat3Array(ptr, "isosurface/ellipsoidRaster", "positionFloat3s", state->positionFloat3.data, state->positionFloat3.size);
}
void editorFlowStage_isosurface_destroy(EditorFlow* ptr, void* userdata)
{
    auto* state = (EditorFlowStageIsosurface*)userdata;

    delete state;
}
static const EditorFlowStage editorFlowStage_isosurface = { "isosurface", editorFlowStage_isosurface_init, editorFlowStage_isosurface_update, editorFlowStage_isosurface_destroy };

void* editorFlowStage_init_manySpheres(EditorFlow* ptr)
{
    editorFlow_definePrim(ptr, "FlowSimulate", "simulate", "simulate");
    editorFlow_definePrim(ptr, "FlowOffscreen", "offscreen", "offscreen");
    editorFlow_definePrim(ptr, "FlowRender", "render", "render");
    for (int j = 0; j < 4; j++)
    {
        for (int i = 0; i < 16; i++)
        {
            char name[80u];
            snprintf(name, 80u, "emitter_%d_%d", j, i);

            NvFlowFloat3 pos = {};
            pos.x = 20.f * (float(j) + 1.f) * cosf(6.28f * (float(i) + 0.33f * float(j) + 0.5f) / 16.f);
            pos.y = 20.f * (float(j) + 1.f) * sinf(6.28f * (float(i) + 0.33f * float(j) + 0.5f) / 16.f);
            pos.z = 0.f;

            editorFlow_definePrim(ptr, "FlowEmitterSphere", name, name);
            editorFlow_setAttributeFloat3(ptr, name, "position", pos);
        }
    }
    return nullptr;
}
static const EditorFlowStage editorFlowStage_manySpheres = { "manySpheres", editorFlowStage_init_manySpheres, nullptr, nullptr };

static const EditorFlowStage* gEditorFlowStage_builtinStages[] = {
    &editorFlowStage_sphere,
    &editorFlowStage_box,
    &editorFlowStage_point,
    &editorFlowStage_mesh,
    &editorFlowStage_texture,
    &editorFlowStage_nanovdb,
    //&editorFlowStage_isosurface,
    &editorFlowStage_manySpheres
};

void editorFlowStage_getBuiltinStages(const EditorFlowStage*** pStages, NvFlowUint64* pStageCount)
{
    *pStages = gEditorFlowStage_builtinStages;
    *pStageCount = sizeof(gEditorFlowStage_builtinStages) / sizeof(EditorFlowStage*);
}
