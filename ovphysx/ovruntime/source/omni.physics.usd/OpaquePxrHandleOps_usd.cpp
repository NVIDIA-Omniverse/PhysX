// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-USDLIB-LOADABLE-001
 * @covers AC-2
 *
 * @implements REQ-BUILD-BRIDGE-001
 * @covers AC-3
 */

#include "OpaquePxrHandleOpsUsd.h"

#include <pxr/usd/sdf/layer.h>
#include <pxr/usd/usd/stage.h>

#include <new>

namespace omni::physics::usd
{
namespace
{

// --- Attached UsdStage handle (weak) ---
using StageWeak = PXR_NS::UsdStageWeakPtr;
static_assert(sizeof(StageWeak) == 2 * sizeof(void*) && alignof(StageWeak) == alignof(void*),
              "UsdStageWeakPtr storage assumption (2 pointers) broken; opaque storage size is wrong");

void sCtor(void* storage) noexcept
{
    new (storage) StageWeak();
}
void sCopy(void* dst, const void* src) noexcept
{
    new (dst) StageWeak(*static_cast<const StageWeak*>(src));
}
void sAssign(void* dst, const void* src) noexcept
{
    *static_cast<StageWeak*>(dst) = *static_cast<const StageWeak*>(src);
}
void sDtor(void* storage) noexcept
{
    static_cast<StageWeak*>(storage)->~StageWeak();
}
bool sBool(const void* storage) noexcept
{
    return bool(*static_cast<const StageWeak*>(storage));
}

constexpr omni::physics::parse::OpaquePxrHandleOps kStageOps{ sCtor, sCopy, sAssign, sDtor, sBool, nullptr, nullptr };

// --- Simulation SdfLayer handle (ref) ---
using SimLayerRef = PXR_NS::SdfLayerRefPtr;
static_assert(sizeof(SimLayerRef) == sizeof(void*) && alignof(SimLayerRef) == alignof(void*),
              "SdfLayerRefPtr storage assumption (1 pointer) broken; opaque storage size is wrong");

void lCtor(void* storage) noexcept
{
    new (storage) SimLayerRef();
}
void lCopy(void* dst, const void* src) noexcept
{
    new (dst) SimLayerRef(*static_cast<const SimLayerRef*>(src));
}
void lAssign(void* dst, const void* src) noexcept
{
    *static_cast<SimLayerRef*>(dst) = *static_cast<const SimLayerRef*>(src);
}
void lDtor(void* storage) noexcept
{
    static_cast<SimLayerRef*>(storage)->~SimLayerRef();
}
bool lBool(const void* storage) noexcept
{
    return bool(*static_cast<const SimLayerRef*>(storage));
}
void* lRaw(const void* storage) noexcept
{
    const SimLayerRef& r = *static_cast<const SimLayerRef*>(storage);
    return r ? r.operator->() : nullptr;
}
void lAdopt(void* storage, void* rawObject) noexcept
{
    *static_cast<SimLayerRef*>(storage) = SimLayerRef(static_cast<PXR_NS::SdfLayer*>(rawObject));
}

constexpr omni::physics::parse::OpaquePxrHandleOps kLayerOps{ lCtor, lCopy, lAssign, lDtor, lBool, lRaw, lAdopt };

} // namespace

const omni::physics::parse::OpaquePxrHandleOps* attachedStageUsdHandleOps()
{
    return &kStageOps;
}

const omni::physics::parse::OpaquePxrHandleOps* simulationLayerHandleOps()
{
    return &kLayerOps;
}

} // namespace omni::physics::usd
