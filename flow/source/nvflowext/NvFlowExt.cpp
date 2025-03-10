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

NvFlowOpInterface* NvFlowOp_EmitterSphere_getOpInterface();
NvFlowOpInterface* NvFlowOp_EmitterSphereAllocate_getOpInterface();
NvFlowOpInterface* NvFlowOp_EmitterBox_getOpInterface();
NvFlowOpInterface* NvFlowOp_EmitterBoxAllocate_getOpInterface();
NvFlowOpInterface* NvFlowOp_EmitterPoint_getOpInterface();
NvFlowOpInterface* NvFlowOp_EmitterPointAllocate_getOpInterface();
NvFlowOpInterface* NvFlowOp_EmitterMesh_getOpInterface();
NvFlowOpInterface* NvFlowOp_EmitterMeshAllocate_getOpInterface();
NvFlowOpInterface* NvFlowOp_EmitterTexture_getOpInterface();
NvFlowOpInterface* NvFlowOp_EmitterTextureAllocate_getOpInterface();
NvFlowOpInterface* NvFlowOp_EmitterNanoVdb_getOpInterface();
NvFlowOpInterface* NvFlowOp_EmitterNanoVdbAllocate_getOpInterface();
NvFlowOpInterface* NvFlowOp_EllipsoidRaster_getOpInterface();
NvFlowOpInterface* NvFlowOp_EllipsoidRasterAllocate_getOpInterface();
NvFlowOpInterface* NvFlowOp_Shadow_getOpInterface();
NvFlowOpInterface* NvFlowOp_DebugVolume_getOpInterface();
NvFlowOpInterface* NvFlowOp_RayMarch_getOpInterface();
NvFlowOpInterface* NvFlowOp_RayMarchUpdateColormap_getOpInterface();
NvFlowOpInterface* NvFlowOp_RayMarchIsosurface_getOpInterface();
NvFlowOpInterface* NvFlowOp_RayMarchCopyTexture_getOpInterface();

NV_FLOW_API NvFlowExtOpList* NvFlowGetExtOpList()
{
    static NvFlowExtOpList iface = { NV_FLOW_REFLECT_INTERFACE_INIT(NvFlowExtOpList) };
    iface.pEmitterSphere = NvFlowOp_EmitterSphere_getOpInterface;
    iface.pEmitterSphereAllocate = NvFlowOp_EmitterSphereAllocate_getOpInterface;
    iface.pEmitterBox = NvFlowOp_EmitterBox_getOpInterface;
    iface.pEmitterBoxAllocate = NvFlowOp_EmitterBoxAllocate_getOpInterface;
    iface.pEmitterPoint = NvFlowOp_EmitterPoint_getOpInterface;
    iface.pEmitterPointAllocate = NvFlowOp_EmitterPointAllocate_getOpInterface;
    iface.pEmitterMesh = NvFlowOp_EmitterMesh_getOpInterface;
    iface.pEmitterMeshAllocate = NvFlowOp_EmitterMeshAllocate_getOpInterface;
    iface.pEmitterTexture = NvFlowOp_EmitterTexture_getOpInterface;
    iface.pEmitterTextureAllocate = NvFlowOp_EmitterTextureAllocate_getOpInterface;
    iface.pEmitterNanoVdb = NvFlowOp_EmitterNanoVdb_getOpInterface;
    iface.pEmitterNanoVdbAllocate = NvFlowOp_EmitterNanoVdbAllocate_getOpInterface;
    iface.pEllipsoidRaster = NvFlowOp_EllipsoidRaster_getOpInterface;
    iface.pEllipsoidRasterAllocate = NvFlowOp_EllipsoidRasterAllocate_getOpInterface;
    iface.pShadow = NvFlowOp_Shadow_getOpInterface;
    iface.pDebugVolume = NvFlowOp_DebugVolume_getOpInterface;
    iface.pRayMarch = NvFlowOp_RayMarch_getOpInterface;
    iface.pRayMarchUpdateColormap = NvFlowOp_RayMarchUpdateColormap_getOpInterface;
    iface.pRayMarchIsosurface = NvFlowOp_RayMarchIsosurface_getOpInterface;
    iface.pRayMarchCopyTexture = NvFlowOp_RayMarchCopyTexture_getOpInterface;
    return &iface;
}

NvFlowDeviceInterface* NvFlowGetDeviceInterface_vulkan();
NvFlowDeviceInterface* NvFlowGetDeviceInterface_cpu();

NvFlowDeviceInterface* NvFlowGetDeviceInterface(NvFlowContextApi api)
{
    if (api == eNvFlowContextApi_vulkan)
    {
        return NvFlowGetDeviceInterface_vulkan();
    }
    if (api == eNvFlowContextApi_cpu)
    {
        return NvFlowGetDeviceInterface_cpu();
    }
    return nullptr;
}

#ifdef NV_FLOW_DEBUG_ALLOC
#include <stdio.h>
#include <atomic>
std::atomic_int32_t allocCount = 0u;
void* operator new(std::size_t sz)
{
    if (sz == 0u) sz = 1u;
    allocCount++;
    return std::malloc(sz);
}
void operator delete(void* ptr)
{
    std::free(ptr);
    int32_t count = allocCount.fetch_sub(1) - 1;
    printf("NvFlowExt.cpp free() refCount = %d\n", count);
}
void* operator new[](std::size_t sz)
{
    if (sz == 0u) sz = 1u;
    allocCount++;
    return std::malloc(sz);
}
void operator delete[](void* ptr)
{
    std::free(ptr);
    int32_t count = allocCount.fetch_sub(1) - 1;
    printf("NvFlowExt.cpp free() refCount = %d\n", count);
}
#endif
