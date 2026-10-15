// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-DEBUG-SNAPSHOT-001
// @covers AC-1 AC-2 AC-3
#include <omni/physx/NativeDebug.h>
#include <omni/physx/PhysXRuntime.h>
#include "RuntimeExecution.h"
#include "PhysXDebugVisualization.h"
#include "PhysXScene.h"
#include "usdLoad/LoadUsd.h"
#include "usdLoad/AttachedStage.h"
#include <cmath>
#include <utility>
namespace omni::physx::native
{
uint64_t debugParameterMask() noexcept
{
    return ((uint64_t(1) << ePhysXNumValues) - 1) & ~uint64_t(1) & ~(uint64_t(1) << eCullBox);
}
namespace
{
bool cpu(const usdparser::AttachedStage& owner)
{
    for (const std::pair<const uint64_t, omni::physx::PhysXScene *> & item : owner.getPhysXScenes())
    {
        ::physx::PxScene * scene = item.second->getScene();
        if (scene && (scene->getFlags().isSet(::physx::PxSceneFlag::eENABLE_GPU_DYNAMICS) ||
                      scene->getFlags().isSet(::physx::PxSceneFlag::eENABLE_DIRECT_GPU_API))) return false;
    }
    return true;
}
DebugVertex vertex(const carb::Float3& p, uint32_t color) { return {{p.x,p.y,p.z},color}; }
}
DebugResult configureDebug(uint64_t attachment, const DebugConfig& config) noexcept
try
{
    if ((config.parameters & ~debugParameterMask()) || !std::isfinite(config.scale) || config.scale < 0)
        return DebugResult::Invalid;
    if (config.culling)
        for (unsigned i=0;i<3;++i)
            if (!std::isfinite(config.minimum[i]) || !std::isfinite(config.maximum[i]) || config.minimum[i]>config.maximum[i])
                return DebugResult::Invalid;
    if (!runtime::tryGetPhysxInterface()) return DebugResult::Stale;
    std::shared_ptr<omni::physx::usdparser::AttachedStage> owner = usdparser::UsdLoad::getUsdLoad()->acquireAttachment(attachment);
    if (!owner) return DebugResult::Stale;
    RuntimeExecutionScope admission(attachment);
    if (!admission) return DebugResult::Busy;
    if (!cpu(*owner)) return DebugResult::Unavailable;
    for (unsigned i=1;i<ePhysXNumValues;++i)
    {
        // This table owns the magnitudes: one unit times the declared scale.
        setVisualizationParameterValue(*owner, PhysXVisualizationParameter(i), 1.0f);
        setVisualizationParameter(*owner, PhysXVisualizationParameter(i), (config.parameters & (uint64_t(1)<<i)) != 0);
    }
    setVisualizationParameter(*owner, eCullBox, config.culling);
    if (config.culling)
        setVisualizationCullingBox(*owner, {config.minimum[0],config.minimum[1],config.minimum[2]},
                                           {config.maximum[0],config.maximum[1],config.maximum[2]});
    setVisualizationScale(*owner, config.scale);
    enableVisualization(*owner, config.enabled);
    return DebugResult::Success;
}
catch (...) { return DebugResult::Error; }
DebugResult captureDebug(uint64_t attachment, uint64_t maxVertices, DebugFrame& output) noexcept
try
{
    if (!maxVertices || maxVertices > 10000000) return DebugResult::Invalid;
    if (!runtime::tryGetPhysxInterface()) return DebugResult::Stale;
    std::shared_ptr<omni::physx::usdparser::AttachedStage> owner = usdparser::UsdLoad::getUsdLoad()->acquireAttachment(attachment);
    if (!owner) return DebugResult::Stale;
    RuntimeExecutionScope admission(attachment);
    if (!admission) return DebugResult::Busy;
    if (!cpu(*owner)) return DebugResult::Unavailable;
    DebugFrame result;
    result.attachment = attachment;
    result.step = owner->simulationTimestamp();
    if (owner->isDebugVisualizationEnabled())
    {
        const uint64_t np=getNbPoints(*owner), nl=getNbLines(*owner), nt=getNbTriangles(*owner);
        if (np+2*nl+3*nt > maxVertices) return DebugResult::Limit;
        result.points.reserve(np); result.lines.reserve(2*nl); result.triangles.reserve(3*nt);
        const omni::physx::DebugPoint * points=getPoints(*owner);
        const omni::physx::DebugLine * lines=getLines(*owner);
        const omni::physx::DebugTriangle * triangles=getTriangles(*owner);
        if ((np && !points) || (nl && !lines) || (nt && !triangles)) return DebugResult::Error;
        for (uint64_t i=0;i<np;++i) result.points.push_back(vertex(points[i].mPos,points[i].mColor));
        for (uint64_t i=0;i<nl;++i)
        {
            result.lines.push_back(vertex(lines[i].mPos0,lines[i].mColor0));
            result.lines.push_back(vertex(lines[i].mPos1,lines[i].mColor1));
        }
        for (uint64_t i=0;i<nt;++i)
        {
            result.triangles.push_back(vertex(triangles[i].mPos0,triangles[i].mColor0));
            result.triangles.push_back(vertex(triangles[i].mPos1,triangles[i].mColor1));
            result.triangles.push_back(vertex(triangles[i].mPos2,triangles[i].mColor2));
        }
    }
    output=std::move(result);
    return DebugResult::Success;
}
catch (...) { return DebugResult::Error; }
}
