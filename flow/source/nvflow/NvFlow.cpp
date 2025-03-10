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

#include <stdio.h>

#include "NvFlow.h"

NvFlowOpGraphInterface* NvFlowGetOpGraphInterface();
NvFlowOpRuntimeInterface* NvFlowGetOpRuntimeInterface();
NvFlowSparseInterface* NvFlowGetSparseInterface();
NvFlowOpInterface* NvFlowOp_SparseNanoVdbExport_getOpInterface();
NvFlowOpInterface* NvFlowOp_AdvectionSimple_getOpInterface();
NvFlowOpInterface* NvFlowOp_AdvectionCombustionDensity_getOpInterface();
NvFlowOpInterface* NvFlowOp_AdvectionCombustionVelocity_getOpInterface();
NvFlowOpInterface* NvFlowOp_Pressure_getOpInterface();
NvFlowOpInterface* NvFlowOp_Vorticity_getOpInterface();
NvFlowOpInterface* NvFlowOp_Summary_getOpInterface();
NvFlowOpInterface* NvFlowOp_SummaryAllocate_getOpInterface();

NV_FLOW_API NvFlowOpList* NvFlowGetOpList()
{
    static NvFlowOpList iface = { NV_FLOW_REFLECT_INTERFACE_INIT(NvFlowOpList) };
    iface.getOpGraphInterface = NvFlowGetOpGraphInterface;
    iface.getOpRuntimeInterface = NvFlowGetOpRuntimeInterface;
    iface.getSparseInterface = NvFlowGetSparseInterface;
    iface.pSparseNanoVdbExport = NvFlowOp_SparseNanoVdbExport_getOpInterface;
    iface.pAdvectionSimple = NvFlowOp_AdvectionSimple_getOpInterface;
    iface.pAdvectionCombustionDensity = NvFlowOp_AdvectionCombustionDensity_getOpInterface;
    iface.pAdvectionCombustionVelocity = NvFlowOp_AdvectionCombustionVelocity_getOpInterface;
    iface.pPressure = NvFlowOp_Pressure_getOpInterface;
    iface.pVorticity = NvFlowOp_Vorticity_getOpInterface;
    iface.pSummary = NvFlowOp_Summary_getOpInterface;
    iface.pSummaryAllocate = NvFlowOp_SummaryAllocate_getOpInterface;
    return &iface;
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
    printf("NvFlow.cpp free() refCount = %d\n", count);
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
    printf("NvFlow.cpp free() refCount = %d\n", count);
}
#endif
