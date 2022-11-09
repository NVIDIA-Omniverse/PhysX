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

GlfwLoader glfwLoader{};

void windowSizeCallback(GLFWwindow* win, int width, int height);
void keyboardCallback(GLFWwindow* win, int key, int scanCode, int action, int modifiers);
void charInputCallback(GLFWwindow* win, uint32_t input);
void mouseMoveCallback(GLFWwindow* win, double mouseX, double mouseY);
void mouseButtonCallback(GLFWwindow* win, int button, int action, int modifiers);
void mouseWheelCallback(GLFWwindow* win, double scrollX, double scrollY);

int editorGlfw_init(App* ptr)
{
    GlfwLoader_init(&glfwLoader);

    if (!glfwLoader.p_glfwInit)
    {
        fprintf(stderr, "GLFW binary is missing!\n");
        return 1;
    }

    if (!glfwLoader.p_glfwInit())
    {
        fprintf(stderr, "Failed to init GLFW\n");
        return 1;
    }

    glfwLoader.p_glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);

    const char* windowName = "NVIDIA Flow 2 Editor";

    ptr->window = glfwLoader.p_glfwCreateWindow((int)ptr->windowWidth, (int)ptr->windowHeight, windowName, nullptr, nullptr);
    if (!ptr->window)
    {
        fprintf(stderr, "Failed to create GLFW window\n");
        return 1;
    }

    GLFWmonitor* monitor = glfwLoader.p_glfwGetPrimaryMonitor();
    const GLFWvidmode* mode = glfwLoader.p_glfwGetVideoMode(monitor);

    glfwLoader.p_glfwSetWindowUserPointer(ptr->window, ptr);

    glfwLoader.p_glfwSetWindowPos(ptr->window, mode->width / 2 - ((int)ptr->windowWidth) / 2, mode->height / 2 - ((int)ptr->windowHeight) / 2);

    glfwLoader.p_glfwSetWindowSizeCallback(ptr->window, windowSizeCallback);
    glfwLoader.p_glfwSetKeyCallback(ptr->window, keyboardCallback);
    glfwLoader.p_glfwSetCharCallback(ptr->window, charInputCallback);
    glfwLoader.p_glfwSetMouseButtonCallback(ptr->window, mouseButtonCallback);
    glfwLoader.p_glfwSetCursorPosCallback(ptr->window, mouseMoveCallback);
    glfwLoader.p_glfwSetScrollCallback(ptr->window, mouseWheelCallback);

    return 0;
}

void editorGlfw_getSwapchainDesc(App* ptr, NvFlowSwapchainDesc* outDesc)
{
    NvFlowSwapchainDesc swapchainDesc = {};
    swapchainDesc.format = eNvFlowFormat_b8g8r8a8_unorm;
#if defined(_WIN32)
    swapchainDesc.hwnd = glfwLoader.p_glfwGetWin32Window(ptr->window);
    swapchainDesc.hinstance = (HINSTANCE)GetWindowLongPtr(swapchainDesc.hwnd, GWLP_HINSTANCE);
#else
    swapchainDesc.dpy = glfwLoader.p_glfwGetX11Display();
    swapchainDesc.window = glfwLoader.p_glfwGetX11Window(ptr->window);
#endif
    *outDesc = swapchainDesc;
}

int editorGlfw_processEvents(App* ptr)
{
    glfwLoader.p_glfwPollEvents();

    if (glfwLoader.p_glfwWindowShouldClose(ptr->window))
    {
        editorCompute_logPrint(eNvFlowLogLevel_info, "GLFW Close Window.");
        return 1;
    }
    return 0;
}

void editorGlfw_destroy(App* ptr)
{
    glfwLoader.p_glfwDestroyWindow(ptr->window);

    glfwLoader.p_glfwTerminate();

    GlfwLoader_destroy(&glfwLoader);
}

void editorGlfw_newFrame(App* ptr, float deltaTime)
{
    ImGuiIO& io = ImGui::GetIO();

    io.DisplaySize = ImVec2(float(ptr->windowWidth), float(ptr->windowHeight));

    io.DeltaTime = deltaTime;

    for (int i = 0; i < IM_ARRAYSIZE(io.MouseDown); i++)
    {
        io.MouseDown[i] = ptr->mouseJustPressed[i] != 0u || glfwLoader.p_glfwGetMouseButton(ptr->window, i) != 0;
        ptr->mouseJustPressed[i] = NV_FLOW_FALSE;
    }

    io.MousePos.x = (float)ptr->mouseX;
    io.MousePos.y = (float)ptr->mouseY;

    ImGui::NewFrame();
}

void windowSizeCallback(GLFWwindow* win, int width, int height)
{
    auto ptr = (App*)glfwLoader.p_glfwGetWindowUserPointer(win);

    // resize
    ptr->compute.loader.deviceInterface.resizeSwapchain(ptr->compute.swapchain, (NvFlowUint)width, (NvFlowUint)height);

    if (width == 0 || height == 0)
    {
        return;
    }

    ptr->windowWidth = width;
    ptr->windowHeight = height;
}

void keyboardCallback(GLFWwindow* win, int key, int scanCode, int action, int modifiers)
{
    auto ptr = (App*)glfwLoader.p_glfwGetWindowUserPointer(win);

    ImGuiIO& io = ImGui::GetIO();

    if (!io.WantCaptureKeyboard)
    {
        if (action == GLFW_PRESS)
        {
            if (key == GLFW_KEY_ESCAPE)
            {
                ptr->shouldRun = false;
            }
            else if (key == GLFW_KEY_H)
            {
                NvFlowCameraConfig config = {};
                NvFlowCameraGetConfig(ptr->camera, &config);

                config.isProjectionRH = !config.isProjectionRH;

                NvFlowCameraSetConfig(ptr->camera, &config);
            }
            else if (key == GLFW_KEY_O)
            {
                NvFlowCameraConfig config = {};
                NvFlowCameraGetConfig(ptr->camera, &config);

                config.isOrthographic = !config.isOrthographic;
                if (config.isOrthographic)
                {
                    config.farPlane = 10000.f;
                }
                if (!config.isOrthographic && config.isReverseZ)
                {
                    config.farPlane = INFINITY;
                }

                NvFlowCameraSetConfig(ptr->camera, &config);
            }
            else if (key == GLFW_KEY_J)
            {
                NvFlowCameraConfig config = {};
                NvFlowCameraGetConfig(ptr->camera, &config);
                config.isReverseZ = !config.isReverseZ;
                if (config.isReverseZ)
                {
                    config.farPlane = INFINITY;
                }
                else
                {
                    config.farPlane = 10000.f;
                }
                if (config.isOrthographic)
                {
                    config.farPlane = 10000.f;
                }
                NvFlowCameraSetConfig(ptr->camera, &config);
            }
            else if (key == GLFW_KEY_K)
            {
                NvFlowCameraState state = {};
                NvFlowCameraGetState(ptr->camera, &state);
                bool isZup = state.eyeUp.z > 0.5f;
                NvFlowCameraGetDefaultState(&state, isZup);
                NvFlowCameraSetState(ptr->camera, &state);
            }
            else if (key == GLFW_KEY_V)
            {
                ptr->compute.vsync ^= NV_FLOW_TRUE;
            }
            else if (key == GLFW_KEY_P)
            {
                ptr->isPaused ^= NV_FLOW_TRUE;
            }
            else if (key == GLFW_KEY_G)
            {
                ptr->overlayEnabled ^= NV_FLOW_TRUE;
            }
            else if (key == GLFW_KEY_E)
            {
                ptr->editorEnabled ^= NV_FLOW_TRUE;
            }
            else if (key == GLFW_KEY_C)
            {
                ptr->captureEnabled ^= NV_FLOW_TRUE;
            }
            else if (key == GLFW_KEY_F11)
            {
                if (ptr->fullscreenState == 0)
                {
                    GLFWmonitor* monitor = glfwLoader.p_glfwGetPrimaryMonitor();
                    const GLFWvidmode* mode = glfwLoader.p_glfwGetVideoMode(monitor);

                    ptr->windowWidthOld = ptr->windowWidth;
                    ptr->windowHeightOld = ptr->windowHeight;

                    glfwLoader.p_glfwSetWindowMonitor(ptr->window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);

                    ptr->fullscreenState = 1;
                }
                else if (ptr->fullscreenState == 2)
                {
                    glfwLoader.p_glfwSetWindowMonitor(ptr->window, nullptr,
                        (int)(ptr->windowWidth / 2 - ptr->windowWidthOld / 2),
                        (int)(ptr->windowHeight / 2 - ptr->windowHeightOld / 2),
                        (int)(ptr->windowWidthOld), (int)(ptr->windowHeightOld),
                        GLFW_DONT_CARE
                    );
                    ptr->fullscreenState = 3;
                }
            }
        }
        else if (action == GLFW_RELEASE)
        {
            if (key == GLFW_KEY_F11)
            {
                if (ptr->fullscreenState == 1)
                {
                    ptr->fullscreenState = 2;
                }
                else if (ptr->fullscreenState == 3)
                {
                    ptr->fullscreenState = 0;
                }
            }
        }

        if (!ImGui::GetIO().WantCaptureMouse)
        {
            NvFlowCameraAction nvfAction = eNvFlowCameraAction_unknown;
            if (action == GLFW_PRESS)
            {
                nvfAction = eNvFlowCameraAction_down;
            }
            else if (action == GLFW_RELEASE)
            {
                nvfAction = eNvFlowCameraAction_up;
            }

            NvFlowCameraKey flowKey = eNvFlowCameraKey_unknown;
            if (key == GLFW_KEY_UP)
            {
                flowKey = eNvFlowCameraKey_up;
            }
            else if (key == GLFW_KEY_DOWN)
            {
                flowKey = eNvFlowCameraKey_down;
            }
            else if (key == GLFW_KEY_LEFT)
            {
                flowKey = eNvFlowCameraKey_left;
            }
            else if (key == GLFW_KEY_RIGHT)
            {
                flowKey = eNvFlowCameraKey_right;
            }
            NvFlowCameraKeyUpdate(ptr->camera, flowKey, nvfAction);
        }
    }
    // imgui always captures
    {
        if (action == GLFW_PRESS)
        {
            io.KeysDown[key] = true;
        }
        else if (action == GLFW_RELEASE)
        {
            io.KeysDown[key] = false;
        }
        io.KeyCtrl = io.KeysDown[GLFW_KEY_LEFT_CONTROL] || io.KeysDown[GLFW_KEY_RIGHT_CONTROL];
        io.KeyShift = io.KeysDown[GLFW_KEY_LEFT_SHIFT] || io.KeysDown[GLFW_KEY_RIGHT_SHIFT];
        io.KeyAlt = io.KeysDown[GLFW_KEY_LEFT_ALT] || io.KeysDown[GLFW_KEY_RIGHT_ALT];
        io.KeySuper = io.KeysDown[GLFW_KEY_LEFT_SUPER] || io.KeysDown[GLFW_KEY_RIGHT_SUPER];
    }
}

void charInputCallback(GLFWwindow* win, uint32_t input)
{
    auto ptr = (App*)glfwLoader.p_glfwGetWindowUserPointer(win);

    ImGuiIO& io = ImGui::GetIO();
    // imgui always captures
    {
        io.AddInputCharacter(input);
    }
}

void mouseMoveCallback(GLFWwindow* win, double mouseX, double mouseY)
{
    auto ptr = (App*)glfwLoader.p_glfwGetWindowUserPointer(win);

    int x = int(mouseX);
    int y = int(mouseY);

    ptr->mouseX = x;
    ptr->mouseY = y;
    ptr->mouseYInv = ptr->windowHeight - 1 - y;

    if (!ImGui::GetIO().WantCaptureMouse)
    {
        NvFlowCameraMouseUpdate(ptr->camera, eNvFlowCameraMouseButton_unknown, eNvFlowCameraAction_unknown, ptr->mouseX, ptr->mouseY, (int)(ptr->windowWidth), (int)(ptr->windowHeight));
    }
}

void mouseButtonCallback(GLFWwindow* win, int button, int action, int modifiers)
{
    auto ptr = (App*)glfwLoader.p_glfwGetWindowUserPointer(win);

    if (!ImGui::GetIO().WantCaptureMouse)
    {
        NvFlowCameraAction nvfAction = eNvFlowCameraAction_unknown;
        if (action == GLFW_PRESS)
        {
            nvfAction = eNvFlowCameraAction_down;
        }
        else if (action == GLFW_RELEASE)
        {
            nvfAction = eNvFlowCameraAction_up;
        }

        NvFlowCameraMouseButton nvfMouse = eNvFlowCameraMouseButton_unknown;
        if (button == GLFW_MOUSE_BUTTON_LEFT)
        {
            nvfMouse = eNvFlowCameraMouseButton_left;
        }
        else if (button == GLFW_MOUSE_BUTTON_MIDDLE)
        {
            nvfMouse = eNvFlowCameraMouseButton_middle;
        }
        else if (button == GLFW_MOUSE_BUTTON_RIGHT)
        {
            nvfMouse = eNvFlowCameraMouseButton_right;
        }
        NvFlowCameraMouseUpdate(ptr->camera, nvfMouse, nvfAction, ptr->mouseX, ptr->mouseY, (int)ptr->windowWidth, (int)ptr->windowHeight);
    }

    // imgui
    if (action == GLFW_PRESS && button >= 0 && button < 5)
    {
        ptr->mouseJustPressed[button] = NV_FLOW_TRUE;
    }
}

void mouseWheelCallback(GLFWwindow* win, double scrollX, double scrollY)
{
    auto ptr = (App*)glfwLoader.p_glfwGetWindowUserPointer(win);

    // imgui
    ImGuiIO& io = ImGui::GetIO();
    io.MouseWheelH += (float)scrollX;
    io.MouseWheel += (float)scrollY;
}
