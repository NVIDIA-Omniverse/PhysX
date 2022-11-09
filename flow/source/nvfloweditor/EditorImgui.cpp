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
// Copyright (c) 2014-2022 NVIDIA Corporation. All rights reserved.

#include "EditorCommon.h"

void setStyle_NvidiaDark()
{
    ImGui::StyleColorsDark();
    ImGuiStyle& s = ImGui::GetStyle();

    s.FrameRounding = 4.0f;

    // Settings
    s.WindowPadding = ImVec2(8.0f, 8.0f);
    s.PopupRounding = 4.0f;
    s.FramePadding = ImVec2(8.0f, 4.0f);
    s.ItemSpacing = ImVec2(6.0f, 6.0f);
    s.ItemInnerSpacing = ImVec2(4.0f, 4.0f);
    s.TouchExtraPadding = ImVec2(0.0f, 0.0f);
    s.IndentSpacing = 21.0f;
    s.ScrollbarSize = 16.0f;
    s.GrabMinSize = 8.0f;

    // BorderSize
    s.WindowBorderSize = 1.0f;
    s.ChildBorderSize = 1.0f;
    s.PopupBorderSize = 1.0f;
    s.FrameBorderSize = 0.0f;
    s.TabBorderSize = 0.0f;

    // Rounding
    s.WindowRounding = 4.0f;
    s.ChildRounding = 4.0f;
    s.FrameRounding = 4.0f;
    s.ScrollbarRounding = 4.0f;
    s.GrabRounding = 4.0f;
    s.TabRounding = 4.0f;

    // Alignment
    s.WindowTitleAlign = ImVec2(0.5f, 0.5f);
    s.ButtonTextAlign = ImVec2(0.48f, 0.5f);

    s.DisplaySafeAreaPadding = ImVec2(3.0f, 3.0f);

    // Colors
    s.Colors[::ImGuiCol_Text] = ImVec4(0.89f, 0.89f, 0.89f, 1.00f);
    s.Colors[::ImGuiCol_Text] = ImVec4(0.89f, 0.89f, 0.89f, 1.00f);
    s.Colors[::ImGuiCol_TextDisabled] = ImVec4(0.43f, 0.43f, 0.43f, 1.00f);
    s.Colors[::ImGuiCol_WindowBg] = ImVec4(0.26f, 0.26f, 0.26f, 1.00f);
    s.Colors[::ImGuiCol_ChildBg] = ImVec4(0.25f, 0.25f, 0.25f, 1.00f);
    s.Colors[::ImGuiCol_PopupBg] = ImVec4(0.25f, 0.25f, 0.25f, 1.00f);
    s.Colors[::ImGuiCol_Border] = ImVec4(0.29f, 0.29f, 0.29f, 1.00f);
    s.Colors[::ImGuiCol_BorderShadow] = ImVec4(0.00f, 0.00f, 0.00f, 1.00f);
    s.Colors[::ImGuiCol_FrameBg] = ImVec4(0.14f, 0.14f, 0.14f, 1.00f);
    s.Colors[::ImGuiCol_FrameBgHovered] = ImVec4(0.29f, 0.29f, 0.29f, 1.00f);
    s.Colors[::ImGuiCol_FrameBgActive] = ImVec4(0.16f, 0.16f, 0.16f, 1.00f);
    s.Colors[::ImGuiCol_TitleBg] = ImVec4(0.14f, 0.14f, 0.14f, 1.00f);
    s.Colors[::ImGuiCol_TitleBgActive] = ImVec4(0.20f, 0.20f, 0.20f, 1.00f);
    s.Colors[::ImGuiCol_TitleBgCollapsed] = ImVec4(0.20f, 0.20f, 0.20f, 1.00f);
    s.Colors[::ImGuiCol_MenuBarBg] = ImVec4(0.20f, 0.20f, 0.20f, 1.00f);
    s.Colors[::ImGuiCol_ScrollbarBg] = ImVec4(0.16f, 0.16f, 0.16f, 1.00f);
    s.Colors[::ImGuiCol_ScrollbarGrab] = ImVec4(0.51f, 0.50f, 0.50f, 1.00f);
    s.Colors[::ImGuiCol_ScrollbarGrabHovered] = ImVec4(1.00f, 0.99f, 0.99f, 0.58f);
    s.Colors[::ImGuiCol_ScrollbarGrabActive] = ImVec4(0.47f, 0.53f, 0.54f, 0.76f);
    s.Colors[::ImGuiCol_CheckMark] = ImVec4(0.89f, 0.89f, 0.89f, 1.00f);
    s.Colors[::ImGuiCol_SliderGrab] = ImVec4(0.59f, 0.59f, 0.59f, 1.00f);
    s.Colors[::ImGuiCol_SliderGrabActive] = ImVec4(0.47f, 0.53f, 0.54f, 0.76f);
    s.Colors[::ImGuiCol_Button] = ImVec4(0.16f, 0.16f, 0.16f, 1.00f);
    s.Colors[::ImGuiCol_ButtonHovered] = ImVec4(0.59f, 0.59f, 0.59f, 1.00f);
    s.Colors[::ImGuiCol_ButtonActive] = ImVec4(0.47f, 0.53f, 0.54f, 0.76f);
    s.Colors[::ImGuiCol_Header] = ImVec4(0.16f, 0.16f, 0.16f, 1.00f);
    s.Colors[::ImGuiCol_HeaderHovered] = ImVec4(0.22f, 0.22f, 0.22f, 1.00f);
    s.Colors[::ImGuiCol_HeaderActive] = ImVec4(0.30f, 0.30f, 0.30f, 1.00f);
    s.Colors[::ImGuiCol_Separator] = ImVec4(0.16f, 0.16f, 0.16f, 1.00f);
    s.Colors[::ImGuiCol_SeparatorHovered] = ImVec4(0.23f, 0.44f, 0.69f, 1.00f);
    s.Colors[::ImGuiCol_SeparatorActive] = ImVec4(0.26f, 0.59f, 0.98f, 1.00f);
    s.Colors[::ImGuiCol_ResizeGrip] = ImVec4(0.16f, 0.16f, 0.16f, 1.00f);
    s.Colors[::ImGuiCol_ResizeGripHovered] = ImVec4(0.23f, 0.44f, 0.69f, 1.00f);
    s.Colors[::ImGuiCol_ResizeGripActive] = ImVec4(0.26f, 0.59f, 0.98f, 1.00f);
    s.Colors[::ImGuiCol_Tab] = ImVec4(0.16f, 0.16f, 0.16f, 1.00f);
    s.Colors[::ImGuiCol_TabHovered] = ImVec4(0.6f, 0.6f, 0.6f, 0.58f);
    s.Colors[::ImGuiCol_TabActive] = ImVec4(0.35f, 0.35f, 0.35f, 1.00f);
    s.Colors[::ImGuiCol_TabUnfocused] = ImVec4(0.16f, 0.16f, 0.16f, 1.00f);
    s.Colors[::ImGuiCol_TabUnfocusedActive] = ImVec4(0.25f, 0.25f, 0.25f, 1.00f);
    //s.Colors[::ImGuiCol_DockingPreview] = ImVec4(0.26f, 0.59f, 0.98f, 0.70f);
    //s.Colors[::ImGuiCol_DockingEmptyBg] = ImVec4(0.25f, 0.25f, 0.25f, 1.00f);
    s.Colors[::ImGuiCol_PlotLines] = ImVec4(0.61f, 0.61f, 0.61f, 1.00f);
    s.Colors[::ImGuiCol_PlotLinesHovered] = ImVec4(1.00f, 0.43f, 0.35f, 1.00f);
    s.Colors[::ImGuiCol_PlotHistogram] = ImVec4(0.90f, 0.70f, 0.00f, 1.00f);
    s.Colors[::ImGuiCol_PlotHistogramHovered] = ImVec4(1.00f, 0.60f, 0.00f, 1.00f);
    s.Colors[::ImGuiCol_TextSelectedBg] = ImVec4(0.97f, 0.97f, 0.97f, 0.19f);
    s.Colors[::ImGuiCol_DragDropTarget] = ImVec4(0.38f, 0.62f, 0.80f, 1.0f);
    s.Colors[::ImGuiCol_NavHighlight] = ImVec4(0.26f, 0.59f, 0.98f, 1.00f);
    s.Colors[::ImGuiCol_NavWindowingHighlight] = ImVec4(1.00f, 1.00f, 1.00f, 0.70f);
    s.Colors[::ImGuiCol_NavWindowingDimBg] = ImVec4(1.00f, 1.00f, 1.00f, 0.70f);
    s.Colors[::ImGuiCol_ModalWindowDimBg] = ImVec4(0.80f, 0.80f, 0.80f, 0.35f);
    //s.ScaleAllSizes(1.2f);
}

void editorImgui_init(EditorImgui* ptr, NvFlowContextInterface* contextInterface, NvFlowContext* context)
{
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();

    setStyle_NvidiaDark();

    io.KeyMap[ImGuiKey_Tab] = GLFW_KEY_TAB;
    io.KeyMap[ImGuiKey_LeftArrow] = GLFW_KEY_LEFT;
    io.KeyMap[ImGuiKey_RightArrow] = GLFW_KEY_RIGHT;
    io.KeyMap[ImGuiKey_UpArrow] = GLFW_KEY_UP;
    io.KeyMap[ImGuiKey_DownArrow] = GLFW_KEY_DOWN;
    io.KeyMap[ImGuiKey_PageUp] = GLFW_KEY_PAGE_UP;
    io.KeyMap[ImGuiKey_PageDown] = GLFW_KEY_PAGE_DOWN;
    io.KeyMap[ImGuiKey_Home] = GLFW_KEY_HOME;
    io.KeyMap[ImGuiKey_End] = GLFW_KEY_END;
    io.KeyMap[ImGuiKey_Insert] = GLFW_KEY_INSERT;
    io.KeyMap[ImGuiKey_Delete] = GLFW_KEY_DELETE;
    io.KeyMap[ImGuiKey_Backspace] = GLFW_KEY_BACKSPACE;
    io.KeyMap[ImGuiKey_Space] = GLFW_KEY_SPACE;
    io.KeyMap[ImGuiKey_Enter] = GLFW_KEY_ENTER;
    io.KeyMap[ImGuiKey_Escape] = GLFW_KEY_ESCAPE;
    io.KeyMap[ImGuiKey_KeyPadEnter] = GLFW_KEY_KP_ENTER;
    io.KeyMap[ImGuiKey_A] = GLFW_KEY_A;
    io.KeyMap[ImGuiKey_C] = GLFW_KEY_C;
    io.KeyMap[ImGuiKey_V] = GLFW_KEY_V;
    io.KeyMap[ImGuiKey_X] = GLFW_KEY_X;
    io.KeyMap[ImGuiKey_Y] = GLFW_KEY_Y;
    io.KeyMap[ImGuiKey_Z] = GLFW_KEY_Z;

    unsigned char* pixels = nullptr;
    int texWidth = 0;
    int texHeight = 0;
    io.Fonts->GetTexDataAsRGBA32(&pixels, &texWidth, &texHeight);

    NvFlowImguiRendererInterface_duplicate(&ptr->imguiRendererInterface, NvFlowGetImguiRendererInterface());

    ptr->imguiRenderer = ptr->imguiRendererInterface.create(contextInterface, context, pixels, texWidth, texHeight);

    editorCompute_logPrint(eNvFlowLogLevel_info, "Initialized Imgui Renderer");
}

void editorValue(NvFlowUint8* data, const NvFlowReflectData reflectData, NvFlowReflectProcess_t processReflect, void* userdata)
{
    /*if (reflectData.reflectHints & eNvFlowReflectHint_noEdit)
    {
        // NOP
    }
    else */if (reflectData.dataType->dataType == eNvFlowType_struct)
    {
        char buf[64];
        buf[63] = '\0';
        snprintf(buf, 64, "%s::%s", NvFlowReflectTrimPrefix(reflectData.dataType->structTypename), reflectData.name);

        bool isVisible = ImGui::TreeNode(buf);
        if (isVisible)
        {
            if (reflectData.dataType->childReflectDatas)
            {
                processReflect(data + reflectData.dataOffset, reflectData.dataType, userdata);
            }

            ImGui::TreePop();
        }
    }
    else if (reflectData.dataType->dataType == eNvFlowType_int)
    {
        ImGui::SliderInt(reflectData.name, (int*)(data + reflectData.dataOffset), -10, 10);
    }
    else if (reflectData.dataType->dataType == eNvFlowType_int2)
    {
        ImGui::SliderInt2(reflectData.name, (int*)(data + reflectData.dataOffset), -10, 10);
    }
    else if (reflectData.dataType->dataType == eNvFlowType_int3)
    {
        ImGui::SliderInt3(reflectData.name, (int*)(data + reflectData.dataOffset), -10, 10);
    }
    else if (reflectData.dataType->dataType == eNvFlowType_int4)
    {
        ImGui::SliderInt4(reflectData.name, (int*)(data + reflectData.dataOffset), -10, 10);
    }
    else if (reflectData.dataType->dataType == eNvFlowType_uint)
    {
        ImGui::SliderInt(reflectData.name, (int*)(data + reflectData.dataOffset), 0, 20);
    }
    else if (reflectData.dataType->dataType == eNvFlowType_uint64)
    {
        ImGui::SliderInt(reflectData.name, (int*)(data + reflectData.dataOffset), 0, 20);
    }
    else if (reflectData.dataType->dataType == eNvFlowType_uint2)
    {
        ImGui::SliderInt2(reflectData.name, (int*)(data + reflectData.dataOffset), 0, 20);
    }
    else if (reflectData.dataType->dataType == eNvFlowType_uint3)
    {
        ImGui::SliderInt3(reflectData.name, (int*)(data + reflectData.dataOffset), 0, 20);
    }
    else if (reflectData.dataType->dataType == eNvFlowType_uint4)
    {
        ImGui::SliderInt4(reflectData.name, (int*)(data + reflectData.dataOffset), 0, 20);
    }
    else if (reflectData.dataType->dataType == eNvFlowType_float)
    {
        ImGui::SliderFloat(reflectData.name, (float*)(data + reflectData.dataOffset), -10.f, 10.f);
    }
    else if (reflectData.dataType->dataType == eNvFlowType_float2)
    {
        ImGui::SliderFloat2(reflectData.name, (float*)(data + reflectData.dataOffset), -10.f, 10.f);
    }
    else if (reflectData.dataType->dataType == eNvFlowType_float3)
    {
        ImGui::SliderFloat3(reflectData.name, (float*)(data + reflectData.dataOffset), -10.f, 10.f);
    }
    else if (reflectData.dataType->dataType == eNvFlowType_float4)
    {
        ImGui::SliderFloat4(reflectData.name, (float*)(data + reflectData.dataOffset), -10.f, 10.f);
    }
    else if (reflectData.dataType->dataType == eNvFlowType_float4x4)
    {
        bool isVisible = ImGui::TreeNode(reflectData.name);
        if (isVisible)
        {
            NvFlowFloat4x4* pMat = (NvFlowFloat4x4*)(data + reflectData.dataOffset);
            ImGui::SliderFloat4("x", &pMat->x.x, -10.f, 10.f);
            ImGui::SliderFloat4("y", &pMat->y.x, -10.f, 10.f);
            ImGui::SliderFloat4("z", &pMat->z.x, -10.f, 10.f);
            ImGui::SliderFloat4("w", &pMat->w.x, -10.f, 10.f);

            ImGui::TreePop();
        }
    }
    else if (reflectData.dataType->dataType == eNvFlowType_bool32)
    {
        ImGui::Checkbox(reflectData.name, (bool*)(data + reflectData.dataOffset));
    }
    else
    {
        ImGui::Text("%s", reflectData.name);
    }
}

void editorProcess(NvFlowUint8* data, const NvFlowReflectDataType* dataType, void* userdata)
{
    for (NvFlowUint idx = 0; idx < dataType->childReflectDataCount; idx++)
    {
        const NvFlowReflectData reflectData = dataType->childReflectDatas[idx];
        if (reflectData.reflectMode & eNvFlowReflectMode_array)
        {
            bool isVisible = ImGui::TreeNode(reflectData.name);
            if (isVisible)
            {
                NvFlowUint64 numElements = *(NvFlowUint64*)(data + reflectData.arraySizeOffset);
                unsigned char* arrayData = *(unsigned char**)(data + reflectData.dataOffset);

                char buf[16];
                buf[15] = '\0';

                for (NvFlowUint arrayIdx = 0u; arrayIdx < numElements; arrayIdx++)
                {
                    snprintf(buf, 16, "%d", arrayIdx);

                    NvFlowReflectData reflectDataAtIndex = reflectData;
                    reflectDataAtIndex.name = buf;
                    reflectDataAtIndex.dataOffset = arrayIdx * reflectData.dataType->elementSize;

                    editorValue(arrayData, reflectDataAtIndex, editorProcess, userdata);
                }

                ImGui::TreePop();
            }
        }
        else
        {
            editorValue(data, reflectData, editorProcess, userdata);
        }
    }
};

void editorImgui_update(
    EditorImgui* ptr,
    App* app,
    EditorCompute* compute,
    EditorFlow* flow
)
{
    if (app->overlayEnabled)
    {
        //static bool show_demo_window = true;
        //ImGui::ShowDemoWindow(&show_demo_window);

        float overlayHeight = 512.f;
        float overlayWidth = 384.f;

        ImGui::SetNextWindowPos(ImVec2(16, 16), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(overlayWidth, overlayHeight), ImGuiCond_Always);

        ImGui::Begin("Overlay");
        ImGui::Text("Active Blocks: %d\n", flow->activeBlockCount);
        ImGui::Text("Active Cells: %d\n", flow->activeBlockCount * flow->activeBlockDim.x * flow->activeBlockDim.y * flow->activeBlockDim.z);
        if (flow->activeBlockCountIsosurface > 0u)
        {
            ImGui::Text("Active Isosurface Blocks: %d\n", flow->activeBlockCountIsosurface);
            ImGui::Text("Active Isosurface Cells: %d\n", flow->activeBlockCountIsosurface * flow->activeBlockDimIsosurface.x * flow->activeBlockDimIsosurface.y * flow->activeBlockDimIsosurface.z);
        }
        for (NvFlowUint statIdx = 0u; statIdx < app->compute.statOut_label.size; statIdx++)
        {
            ImGui::Text("%s cpu(%03f) gpu(%03f)\n", app->compute.statOut_label[statIdx], app->compute.statOut_cpu[statIdx], app->compute.statOut_gpu[statIdx]);
        }
        ImGui::End();
    }
    if (app->editorEnabled)
    {
        float editorHeight = 512.f;
        float editorWidth = 384.f;
        float border = 16.f;

        ImGui::SetNextWindowPos(ImVec2(((float)app->windowWidth) - editorWidth - border, border), ImGuiCond_Once);
        ImGui::SetNextWindowSize(ImVec2(editorWidth, editorHeight), ImGuiCond_Once);

        ImGui::Begin("Editor");

        if (ImGui::TreeNode("Settings"))
        {
            if (ImGui::Button(app->isPaused ? "Play" : "Pause"))
            {
                app->isPaused ^= NV_FLOW_TRUE;
            }
            ImGui::DragInt("maxLocations", (int*)&flow->targetMaxLocations, 1.f, 0, 300000);
            ImGui::Checkbox("Profile Window", (bool*)(&app->overlayEnabled));
            ImGui::Checkbox("Capture Enabled", (bool*)(&app->captureEnabled));
            ImGui::Checkbox("Sphere Enabled", (bool*)(&app->sphereEnabled));

            ImGui::TreePop();
        }

        if (ImGui::TreeNode("Stage Selection"))
        {
            for (NvFlowUint64 idx = 0u; idx < flow->stages.size; idx++)
            {
                bool isActive = flow->currentStage == flow->stages[idx];
                if (ImGui::RadioButton(flow->stages[idx]->stageName, isActive))
                {
                    flow->targetStageIdx = idx;
                }
            }

            ImGui::TreePop();
        }

        if (ImGui::TreeNode("Stage Properties"))
        {
            flow->abstractParamsList.size = 0u;
            NvFlowUint64 abstractParamsListCount = 0u;
            flow->gridParamsSet.enumerateActiveInstances(nullptr, &abstractParamsListCount);

            flow->abstractParamsList.reserve(abstractParamsListCount);
            flow->abstractParamsList.size = abstractParamsListCount;
            flow->gridParamsSet.enumerateActiveInstances(flow->abstractParamsList.data, &abstractParamsListCount);
            flow->abstractParamsList.size = abstractParamsListCount;

            NvFlowUint64 stagingVersion = 0llu;
            NvFlowUint64 minActiveVersion = 0llu;
            compute->loader.gridParamsInterface.getVersion(flow->gridParams, &stagingVersion, &minActiveVersion);

            for (NvFlowUint idx = 0u; idx < flow->abstractParamsList.size; idx++)
            {
                char buf[64];
                buf[63] = '\0';
                snprintf(buf, 64, "%s::%s", flow->abstractParamsList[idx]->displayTypename.get(), flow->abstractParamsList[idx]->name.get());

                bool isVisible = ImGui::TreeNode(buf);
                if (isVisible)
                {
                    flow->abstractParamsList[idx]->process(stagingVersion, editorProcess, ptr);

                    ImGui::TreePop();
                }
            }
        }

        ImGui::End();
    }
}

void editorImgui_render(
    EditorImgui* ptr,
    NvFlowContext* context,
    NvFlowTextureTransient* colorIn,
    NvFlowTextureTransient* colorOut,
    NvFlowUint windowWidth,
    NvFlowUint windowHeight
)
{
    ImGui::Render();

    auto drawData = ImGui::GetDrawData();

    ptr->imguiRendererInterface.render(context, ptr->imguiRenderer, drawData, windowWidth, windowHeight, colorIn, colorOut);
}

void editorImgui_destroy(EditorImgui* ptr, NvFlowContext* context)
{
    ptr->imguiRendererInterface.destroy(context, ptr->imguiRenderer);

    ImGui::DestroyContext();

    editorCompute_logPrint(eNvFlowLogLevel_info, "Destroyed Imgui Renderer");
}
