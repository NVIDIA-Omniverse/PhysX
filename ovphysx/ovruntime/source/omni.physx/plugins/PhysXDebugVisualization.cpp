// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-ATTACH-IO-001
// @covers AC-2 AC-3
// @implements REQ-ATTACH-EXECUTION-001
// @covers AC-1 AC-3 AC-4

/**
 * @implements REQ-PUBLICAPI-001
 * @covers AC-14
 *
 * @implements REQ-PUBLICAPI-003
 * @covers AC-6
 */

#include <omni/physics/parse/KnownTokens.h>

#include "PhysXDebugVisualization.h"
#include "RuntimeExecution.h"

#include "PhysXScene.h"
#include "PhysXTools.h"
#include "Setup.h"
#include "PhysXUpdate.h"
#include "CookingDataAsync.h"
#include "ConeCylinderConvexMesh.h"
#include "internal/InternalPhysXDatabase.h"
#include "usdLoad/LoadUsd.h"

#include <private/omni/physx/PhysxUsd.h>

#include <PxPhysicsAPI.h>
#include <common/PxRenderOutput.h>

#include <extensions/PxTriangleMeshExt.h>
#include <PhysXSettings.h>

#include <cmath>
#include <cstring>
#include <unordered_set>

#include <OvstageOutput.h> // buildPathList (checks for the source dictionary); private to omni.physics.ovstage
#include <OvstageSource.h> // OvstageSource::canonicalPath -- see vizScopeContains




using namespace ::physx;
using namespace carb;
using namespace omni::physx;
using namespace omni::physx::usdparser;
using namespace cookingdataasync;

PX_COMPILE_TIME_ASSERT(sizeof(PxDebugPoint) == sizeof(DebugPoint));
PX_COMPILE_TIME_ASSERT(sizeof(PxDebugLine) == sizeof(DebugLine));
PX_COMPILE_TIME_ASSERT(sizeof(PxDebugTriangle) == sizeof(DebugTriangle));
PX_COMPILE_TIME_ASSERT(PxVisualizationParameter::eWORLD_AXES == eWorldAxes);
PX_COMPILE_TIME_ASSERT(PxVisualizationParameter::eNUM_VALUES == ePhysXNumValues);
PX_COMPILE_TIME_ASSERT(PxVisualizationParameter::eNUM_VALUES <= 64);



// DebugVis Shape colors
static constexpr uint32_t gStaticShapeColor = PxU32(PxDebugColor::eARGB_MAGENTA);
static constexpr uint32_t gDynamicShapeColor = PxU32(PxDebugColor::eARGB_GREEN);
static constexpr uint32_t gKinematicShapeColor = PxU32(PxDebugColor::eARGB_YELLOW);
static constexpr uint32_t gDynamicTriMeshShapeColor = PxU32(PxDebugColor::eARGB_DARKRED);

uint32_t omni::physx::getDebugDrawCollShapeColor(usdparser::AttachedStage& owner, omni::physics::parse::ObjectKey key)
{
    uint32_t collColor = gStaticShapeColor;

    // All schema/attribute/structural reads (schema gates, approximation read, ancestor
    // walk) route through IPhysicsSource by ObjectKey, so no UsdPrim is materialized here.
    // Without an attached stage there is no source to consult.
    const usdparser::AttachedStage* attachedStage = &owner;
    const omni::physics::parse::IPhysicsSource* src = attachedStage ? attachedStage->getSource() : nullptr;
    if (!src)
        return collColor;

    omni::physics::parse::KnownTokens tok;
    tok.intern(*src);

    // Source-routed attribute read; getValue resolves the schema fallback, so
    // e.g. physicsRigidBodyEnabled correctly defaults to true when unauthored.
    auto readBool = [&](omni::physics::parse::ObjectKey k, omni::physics::parse::TokenId attr)
    {
        bool v = false;
        internal::getValue(*attachedStage, k, attr, omni::physics::parse::ReadTime::defaultTime(), v);
        return v;
    };

    bool isTrimesh = false;
    if (src->isA(key, tok.meshType))
    {
        if (src->hasSchema(key, tok.usdPhysicsMeshCollisionAPI))
        {
            omni::physics::parse::TokenId approx{};
            src->getAttribute(key, tok.physicsApproximation, approx);
            if (approx == tok.approximationNone)
            {
                isTrimesh = true;
            }
        }
        else
        {
            isTrimesh = true;
        }
    }

    // Walk ancestors through the source until a rigid body is found or the root
    // is reached (getParent returns the synthetic root for top-level objects).
    const omni::physics::parse::ObjectKey root = src->getRootKey();
    omni::physics::parse::ObjectKey parent = key;
    while (parent.valid() && parent != root)
    {
        if (src->hasSchema(parent, tok.physicsRigidBodyAPI))
        {
            if (readBool(parent, tok.physicsRigidBodyEnabled))
            {
                collColor = isTrimesh ? gDynamicTriMeshShapeColor : gDynamicShapeColor;

                if (readBool(parent, tok.physicsKinematicEnabled))
                {
                    collColor = gKinematicShapeColor;
                }
            }

            break; // rigid body found, end traversal
        }

        parent = src->getParent(parent);
    }

    return collColor;
}

// Visualization scope: exact OVStage prim-path membership limiting which objects
// get eVISUALIZATION flags (empty = every object).


// Returns the active source only when it is OVStage-backed. A non-OVStage
// ObjectKey is not comparable to dictionary-specific ovx_primpath_t handles.
static const omni::physics::parse::IPhysicsSource* vizOvstageSource(const usdparser::AttachedStage& owner)
{
    const usdparser::AttachedStage* stage = &owner;
    const omni::physics::parse::IPhysicsSource* src = stage ? stage->getSource() : nullptr;
    if (!src)
        return nullptr;
    ovx_path_dictionary_t* dict = nullptr;
    omni::physics::ovstage::buildPathList(*src, nullptr, 0, &dict);
    return dict ? src : nullptr;
}

static omni::physics::parse::ObjectKey vizActorKey(const usdparser::AttachedStage& owner, const PxActor* actor)
{
    if (!actor)
        return {};
    const std::vector<omni::physx::internal::InternalDatabase::Record>& records =
        owner.getInternalPhysXDatabase().getRecords();
    const size_t recordIndex = reinterpret_cast<size_t>(actor->userData);
    if (recordIndex >= records.size())
        return {};
    const omni::physx::internal::InternalDatabase::Record& record = records[recordIndex];
    if (record.mType == ePTActor || record.mType == ePTLink)
    {
        const omni::physx::internal::InternalActor* internalActor =
            static_cast<const omni::physx::internal::InternalActor*>(record.mInternalPtr);
        bool matchesActor = internalActor && internalActor->mActor == actor;
        if (internalActor && !matchesActor)
        {
            for (const omni::physx::internal::MirrorActor& mirror : internalActor->mMirrors)
            {
                if (mirror.actor == actor)
                {
                    matchesActor = true;
                    break;
                }
            }
        }
        if ((!internalActor && record.mPtr != actor) || (internalActor && !matchesActor))
            return {};
    }
    else if (record.mType == ePTCct)
    {
        const omni::physx::internal::InternalActor* internalActor =
            static_cast<const omni::physx::internal::InternalActor*>(record.mInternalPtr);
        if (!internalActor || internalActor->mActor != actor)
            return {};
    }
    else
    {
        return {};
    }
    return record.mKey;
}

static omni::physics::parse::ObjectKey vizShapeKey(const usdparser::AttachedStage& owner, const PxShape* shape)
{
    if (!shape)
        return {};
    const std::vector<omni::physx::internal::InternalDatabase::Record>& records =
        owner.getInternalPhysXDatabase().getRecords();
    const size_t recordIndex = reinterpret_cast<size_t>(shape->userData);
    if (recordIndex >= records.size())
        return {};
    const omni::physx::internal::InternalDatabase::Record& record = records[recordIndex];
    if (record.mType != ePTShape && record.mType != ePTCompoundShape)
        return {};
    return record.mKey;
}

static bool vizScopeContains(const usdparser::AttachedStage& owner, const omni::physics::parse::IPhysicsSource* source,
                             omni::physics::parse::ObjectKey primKey)
{
    if (owner.getDebugVisualizationCache().mScopePaths.empty())
        return true;
    if (!source || !primKey.valid())
        return false;
    // ADR-0021: ObjectKey.handle is no longer the raw ovx_primpath_t on this
    // backend (OvstageSource now packs a generation into it, mirroring
    // UsdSource's own private index). canonicalPath() is the one public,
    // still-raw-returning translation point -- use it explicitly instead of
    // reading canonicalKey(...).handle as if it were still the dictionary
    // handle. vizOvstageSource(owner) already gates this path to an OVStage-backed
    // source, but guard defensively with a dynamic_cast rather than assuming.
    const omni::physics::ovstage::OvstageSource* ovstageSource = dynamic_cast<const omni::physics::ovstage::OvstageSource*>(source);
    if (!ovstageSource)
        return false;
    const ovx_primpath_t canonical = ovstageSource->canonicalPath(primKey);
    return canonical != OVX_INVALID_PRIMPATH && (owner.getDebugVisualizationCache().mScopePaths.count(canonical) != 0);
}

static bool vizActorInScope(const usdparser::AttachedStage& owner, const omni::physics::parse::IPhysicsSource* source,
                            const ::physx::PxActor* actor)
{
    return vizScopeContains(owner, source, vizActorKey(owner, actor));
}

static bool vizShapeInScope(const usdparser::AttachedStage& owner, const omni::physics::parse::IPhysicsSource* source,
                            const ::physx::PxShape* shape)
{
    return vizScopeContains(owner, source, vizShapeKey(owner, shape));
}

static void enableActorVisualization(const usdparser::AttachedStage& owner, PxActor* actor,
                                     bool enableVis,
                                     const omni::physics::parse::IPhysicsSource* source)
{
    const bool actorInScope = enableVis && vizActorInScope(owner, source, actor);
    bool shapeInScope = false;
    const PxRigidActor* rigidActor = actor->is<PxRigidActor>();
    if (rigidActor)
    {
        const PxU32 nbShapes = rigidActor->getNbShapes();
        for (PxU32 j = 0; j < nbShapes; j++)
        {
            PxShape* shape = nullptr;
            rigidActor->getShapes(&shape, 1, j);
            if (shape)
            {
                const bool enabled = enableVis && vizShapeInScope(owner, source, shape);
                shapeInScope = shapeInScope || enabled;
                if (shape->isExclusive())
                    shape->setFlag(PxShapeFlag::eVISUALIZATION, enabled);
            }
        }
    }
    actor->setActorFlag(PxActorFlag::eVISUALIZATION, actorInScope || shapeInScope);
}

void omni::physx::enableVisualization(usdparser::AttachedStage& owner, bool enableVis)
{
    owner.setDebugVisualizationEnabled(enableVis);
    const omni::physics::parse::IPhysicsSource* scopeSource =
        owner.getDebugVisualizationCache().mScopePaths.empty() ? nullptr : vizOvstageSource(owner);

    for (AttachedStage::SceneMap::const_reference ref : owner.getPhysXScenes())
    {        
        PxScene* scene = ref.second->getScene();
        if (!scene)
            continue;
        
        // shapes/actors
        const PxU32 nbActors = scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC);
        PxActor* actor = nullptr;
        for (PxU32 i = 0; i < nbActors; i++)
        {
            scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC, &actor, 1, i);
            if (!actor)
                break;
            enableActorVisualization(owner, actor, enableVis, scopeSource);
        }

        // articulations
        const PxU32 nbArticulations = scene->getNbArticulations();
        PxArticulationReducedCoordinate* articulation = nullptr;
        for (PxU32 i = 0; i < nbArticulations; i++)
        {
            scene->getArticulations(&articulation, 1, i);
            if (!articulation)
                break;
            const PxU32 nbLinks = articulation->getNbLinks();
            PxArticulationLink* link = nullptr;
            for (PxU32 j = 0; j < nbLinks; j++)
            {
                articulation->getLinks(&link, 1, j);
                if (!link)
                    break;
                enableActorVisualization(owner, link, enableVis, scopeSource);
            }
        }

        // joints
        const PxU32 nbJoints = scene->getNbConstraints();
        PxConstraint* constraint = nullptr;
        for (PxU32 i = 0; i < nbJoints; i++)
        {
            scene->getConstraints(&constraint, 1, i);
            if (!constraint)
                break;
            // A joint is in scope when EITHER attached actor is: selecting a
            // body shows the joints hanging off it. When disabling, the flag
            // is cleared regardless, so skip the per-joint scope match.
            bool jointInScope = !enableVis || owner.getDebugVisualizationCache().mScopePaths.empty();
            if (!jointInScope)
            {
                PxRigidActor* a0 = nullptr;
                PxRigidActor* a1 = nullptr;
                constraint->getActors(a0, a1);
                jointInScope =
                    (a0 && vizActorInScope(owner, scopeSource, a0)) ||
                    (a1 && vizActorInScope(owner, scopeSource, a1));
            }
            constraint->setFlag(PxConstraintFlag::eVISUALIZATION,
                                enableVis && jointInScope);
        }

        if (enableVis)
        {
            const float gizmoScale = owner.getCachedSettings().viewportGizmoScale;
            scene->setVisualizationParameter(
                PxVisualizationParameter::eSCALE, gizmoScale * owner.getVisualizationScale());

            const uint64_t visMask = owner.getVisualizationBitMask();
            for (uint32_t i = 0; i < ePhysXNumValues; i++)
            {
                if (visMask & (1ull << i))
                {
                    scene->setVisualizationParameter(PxVisualizationParameter::Enum(i), owner.getDebugVisualizationCache().mParamValues[i]);
                }
            }
        }
        else
        {
            scene->setVisualizationParameter(PxVisualizationParameter::eSCALE, 0.0f);
        }
    }
}

void omni::physx::enableNormalsVisualization(usdparser::AttachedStage& owner, bool enableNormalsVis)
{
    owner.setNormalsVisualizationEnabled(enableNormalsVis);
}

void omni::physx::setVisualizationScale(usdparser::AttachedStage& owner, float scale)
{

    owner.setVisualizationScale(scale);
    const float gizmoScale = owner.getCachedSettings().viewportGizmoScale;

    if (owner.isDebugVisualizationEnabled())
    {

        for (AttachedStage::SceneMap::const_reference ref : owner.getPhysXScenes())
        {
            PxScene* scene = ref.second->getScene();
            if (scene)
                scene->setVisualizationParameter(PxVisualizationParameter::eSCALE, gizmoScale * scale);
        }
    }
}

void omni::physx::setVisualizationCullingBox(usdparser::AttachedStage& owner, const carb::Float3& min, const carb::Float3& max)
{

    PxBounds3 bounds;
    bounds.minimum = (const PxVec3&)min;
    bounds.maximum = (const PxVec3&)max;

    if (bounds.isValid())
    {

        for (AttachedStage::SceneMap::const_reference ref : owner.getPhysXScenes())
        {
            PxScene* scene = ref.second->getScene();
            if (scene)
                scene->setVisualizationCullingBox(bounds);
        }
    }
}

/**
 * @implements REQ-PUBLICAPI-004
 * @covers AC-1 AC-2 AC-3
 */
void omni::physx::applyDebugRenderForAttach(uint64_t attachHandle,
                                           bool enabled,
                                           float scale,
                                           const float* params,
                                           uint32_t paramCount,
                                           bool hasCull,
                                           const float cullMin[3],
                                           const float cullMax[3],
                                           const ovx_primpath_t* scopeTokens,
                                           uint32_t scopeCount)
{
    if (attachHandle == 0)
        return;
    // Lease first. getAttachedStageByHandle borrows the owner and requires it.
    RuntimeExecutionScope execution(static_cast<omni::physics::AttachHandle>(attachHandle));
    if (!execution)
        return;
    usdparser::AttachedStage* owner = UsdLoad::getUsdLoad()->getAttachedStageByHandle(attachHandle);
    if (!owner)
        return;
    applyDebugRenderToOwner(*owner, enabled, scale, params, paramCount, hasCull, cullMin, cullMax, scopeTokens, scopeCount);
}

void omni::physx::applyDebugRenderToOwner(usdparser::AttachedStage& owner,
                                         bool enabled,
                                         float scale,
                                         const float* params,
                                         uint32_t paramCount,
                                         bool hasCull,
                                         const float cullMin[3],
                                         const float cullMax[3],
                                         const ovx_primpath_t* scopeTokens,
                                         uint32_t scopeCount)
{
    const uint32_t limit = paramCount < static_cast<uint32_t>(ePhysXNumValues)
                               ? paramCount
                               : static_cast<uint32_t>(ePhysXNumValues);
    for (uint32_t i = 1; i < limit; ++i)
    {
        const float value = params && params[i] > 0.0f ? params[i] : 0.0f;
        const PhysXVisualizationParameter parameter = static_cast<PhysXVisualizationParameter>(i);
        setVisualizationParameterValue(owner, parameter, value);
        setVisualizationParameter(owner, parameter, value > 0.0f);
    }

    setVisualizationScale(owner, scale);
    (void)setVisualizationScopeTokens(owner, scopeCount > 0 ? scopeTokens : nullptr, scopeCount);
    if (hasCull && cullMin && cullMax)
    {
        setVisualizationCullingBox(owner,
                                   carb::Float3{ cullMin[0], cullMin[1], cullMin[2] },
                                   carb::Float3{ cullMax[0], cullMax[1], cullMax[2] });
    }
    else
    {
        const PxBounds3 empty = PxBounds3::empty();
        for (AttachedStage::SceneMap::const_reference ref : owner.getPhysXScenes())
        {
            PxScene* scene = ref.second ? ref.second->getScene() : nullptr;
            if (scene)
                scene->setVisualizationCullingBox(empty);
        }
    }

    // Authors flags from this attach's scope and records the enable flag new
    // actors on this attach read at creation.
    enableVisualization(owner, enabled);
}

void omni::physx::copyDebugPrimitivesFromOwner(usdparser::AttachedStage& owner,
                                              std::vector<DebugPoint>& points,
                                              std::vector<DebugLine>& lines,
                                              std::vector<DebugTriangle>& triangles)
{
    points.clear();
    lines.clear();
    triangles.clear();
    for (AttachedStage::SceneMap::const_reference ref : owner.getPhysXScenes())
    {
        PhysXScene* physxScene = ref.second;
        if (!physxScene || !physxScene->getScene())
            continue;
        PxScene* scene = physxScene->getScene();
        const PxRenderBuffer& buffer = scene->getRenderBuffer();
        const PxU32 nPoints = buffer.getNbPoints();
        const PxU32 nLines = buffer.getNbLines();
        const PxU32 nTris = buffer.getNbTriangles();
        if (nPoints && buffer.getPoints())
        {
            const auto* src = reinterpret_cast<const DebugPoint*>(buffer.getPoints());
            points.insert(points.end(), src, src + nPoints);
        }
        if (nLines && buffer.getLines())
        {
            const auto* src = reinterpret_cast<const DebugLine*>(buffer.getLines());
            lines.insert(lines.end(), src, src + nLines);
        }
        if (nTris && buffer.getTriangles())
        {
            const auto* src = reinterpret_cast<const DebugTriangle*>(buffer.getTriangles());
            triangles.insert(triangles.end(), src, src + nTris);
        }
    }

    // Spline debug lines are stored on this attach's database.
    const PxRenderBuffer& extra = owner.getInternalPhysXDatabase().getDebugRenderBuffer();
    const PxU32 nLines = extra.getNbLines();
    if (nLines && extra.getLines())
    {
        const auto* src = reinterpret_cast<const DebugLine*>(extra.getLines());
        lines.insert(lines.end(), src, src + nLines);
    }
}

/**
 * @implements REQ-PUBLICAPI-004
 * @covers AC-2 AC-3 AC-4
 */
bool omni::physx::copyDebugPrimitivesForAttach(uint64_t attachHandle,
                                              std::vector<DebugPoint>& points,
                                              std::vector<DebugLine>& lines,
                                              std::vector<DebugTriangle>& triangles)
{
    if (attachHandle == 0)
    {
        points.clear();
        lines.clear();
        triangles.clear();
        return true;
    }
    RuntimeExecutionScope execution(static_cast<omni::physics::AttachHandle>(attachHandle));
    if (!execution)
        return false;
    usdparser::AttachedStage* owner = UsdLoad::getUsdLoad()->getAttachedStageByHandle(attachHandle);
    if (!owner)
        return false;
    copyDebugPrimitivesFromOwner(*owner, points, lines, triangles);
    return true;
}

void omni::physx::applyDebugRenderForAttachRequest(const DebugRenderAttachRequest* request)
{
    if (!request)
        return;
    applyDebugRenderForAttach(request->attachHandle,
                              request->enabled,
                              request->scale,
                              request->params,
                              request->paramCount,
                              request->hasCullingBox,
                              request->hasCullingBox ? request->cullMin : nullptr,
                              request->hasCullingBox ? request->cullMax : nullptr,
                              request->scopeTokens,
                              request->scopeCount);
}

void omni::physx::copyDebugPrimitivesForAttachBuffers(uint64_t attachHandle,
                                                     DebugPoint* points,
                                                     uint32_t* pointCount,
                                                     DebugLine* lines,
                                                     uint32_t* lineCount,
                                                     DebugTriangle* triangles,
                                                     uint32_t* triangleCount)
{
    std::vector<DebugPoint> copiedPoints;
    std::vector<DebugLine> copiedLines;
    std::vector<DebugTriangle> copiedTriangles;
    if (!copyDebugPrimitivesForAttach(attachHandle, copiedPoints, copiedLines, copiedTriangles))
        return;

    const auto publish = [](auto* destination, uint32_t* count, const auto& source) {
        const uint32_t full = static_cast<uint32_t>(source.size());
        if (destination && count && *count > 0 && full > 0)
        {
            const uint32_t n = *count < full ? *count : full;
            std::memcpy(destination, source.data(), sizeof(*destination) * n);
        }
        if (count)
            *count = full;
    };
    publish(points, pointCount, copiedPoints);
    publish(lines, lineCount, copiedLines);
    publish(triangles, triangleCount, copiedTriangles);
}

void omni::physx::setVisualizationParameter(usdparser::AttachedStage& owner, PhysXVisualizationParameter par, bool val)
{
    if (par <= eNone || par >= eNumValues)
        return;

    uint64_t visMask = owner.getVisualizationBitMask();

    if (val)
        visMask |= (1ull << (uint64_t)par);
    else
        visMask &= ~(1ull << (uint64_t)par);

    owner.setVisualizationBitMask(visMask);


    if (par != eNone && par < ePhysXNumValues)
    {
        for (AttachedStage::SceneMap::const_reference ref : owner.getPhysXScenes())
        {
            PxScene* scene = ref.second->getScene();
            if (scene)
                scene->setVisualizationParameter(PxVisualizationParameter::Enum(par), val ? owner.getDebugVisualizationCache().mParamValues[par] : 0.0f);
        }
    }
    else
    {
        owner.getInternalPhysXDatabase().setVisualizationParameter(par, val);
    }
}

void omni::physx::setVisualizationParameterValue(usdparser::AttachedStage& owner, PhysXVisualizationParameter par, float value)
{
    // Records the magnitude this parameter applies when enabled and re-applies it live
    // if currently on. Does not change the enabled state. Values must be finite and >= 0.
    if (par == eNone || par >= ePhysXNumValues)
        return;
    if (!(value >= 0.0f) || !std::isfinite(value))
        return;

    owner.getDebugVisualizationCache().mParamValues[par] = value;

    const uint64_t visMask = owner.getVisualizationBitMask();
    if ((visMask & (1ull << (uint64_t)par)) == 0)
        return; // parameter is off: the value applies on the next enable


    for (AttachedStage::SceneMap::const_reference ref : owner.getPhysXScenes())
    {
        PxScene* scene = ref.second->getScene();
        if (scene)
            scene->setVisualizationParameter(PxVisualizationParameter::Enum(par), value);
    }
}

bool omni::physx::setVisualizationScopeTokens(usdparser::AttachedStage& owner, const ovx_primpath_t* tokens, uint32_t count)
{
    // Restrict eVISUALIZATION flag authoring to a set of interned prim paths, exact
    // membership (empty = every object). The caller expanded any hierarchy into its
    // object set and interned it against this instance's source dictionary; matching an
    // object canonicalizes its source key through that same dictionary.
    if (count > 0u && !tokens)
        return false;
    std::unordered_set<ovx_primpath_t> pending;
    for (uint32_t i = 0; i < count; ++i)
    {
        if (tokens[i] == OVX_INVALID_PRIMPATH)
            return false;
        pending.insert(tokens[i]);
    }
    if (count > 0u && vizOvstageSource(owner) == nullptr)
        return false;

    owner.getDebugVisualizationCache().mScopePaths.swap(pending);
    // Re-author the flags live so a scope change applies without a disable/enable
    // cycle, drawn on the next step. enableVisualization waits for completion itself.
    if (owner.isDebugVisualizationEnabled())
        omni::physx::enableVisualization(owner, true);
    return true;
}

uint32_t omni::physx::getNbPoints(usdparser::AttachedStage& owner)
{
    uint32_t numPoints = 0;


    for (AttachedStage::SceneMap::const_reference ref : owner.getPhysXScenes())
    {
        PxScene* scene = ref.second->getScene();
        if (scene)
            numPoints += scene->getRenderBuffer().getNbPoints();
    }    
    return numPoints;
}

const DebugPoint* omni::physx::getPoints(usdparser::AttachedStage& owner)
{

    if (owner.getPhysXScenes().empty())
        return nullptr;


    if (owner.getPhysXScenes().size() == 1)
        return (const DebugPoint*)owner.getPhysXScenes().begin()->second->getScene()->getRenderBuffer().getPoints();
    else
    {
        owner.getDebugVisualizationCache().mPointsBuffer.resize(getNbPoints(owner));
        uint32_t offset = 0;
        for (AttachedStage::SceneMap::const_reference ref : owner.getPhysXScenes())
        {
            PxScene* scene = ref.second->getScene();
            const uint32_t numPoints = scene->getRenderBuffer().getNbPoints();
            memcpy(owner.getDebugVisualizationCache().mPointsBuffer.data() + offset, scene->getRenderBuffer().getPoints(), sizeof(DebugPoint)*numPoints);
            offset += numPoints;
        }
    }

    return owner.getDebugVisualizationCache().mPointsBuffer.data();
}

uint32_t omni::physx::getNbLines(usdparser::AttachedStage& owner)
{
    uint32_t numLines = 0;


    for (AttachedStage::SceneMap::const_reference ref : owner.getPhysXScenes())
    {
        PxScene* scene = ref.second->getScene();
        if (scene)
            numLines += scene->getRenderBuffer().getNbLines();
    }
    numLines += owner.getInternalPhysXDatabase().getDebugRenderBuffer().getNbLines();
    return numLines;
}

const DebugLine* omni::physx::getLines(usdparser::AttachedStage& owner)
{

    if (owner.getPhysXScenes().empty())
        return nullptr;


    if (owner.getPhysXScenes().size() == 1 &&
        owner.getInternalPhysXDatabase().getDebugRenderBuffer().getNbLines() == 0)
        return (const DebugLine*)owner.getPhysXScenes().begin()->second->getScene()->getRenderBuffer().getLines();
    else
    {
        owner.getDebugVisualizationCache().mLinesBuffer.resize(getNbLines(owner));
        uint32_t offset = 0;
        for (AttachedStage::SceneMap::const_reference ref : owner.getPhysXScenes())
        {
            PxScene* scene = ref.second->getScene();
            const uint32_t numLines = scene->getRenderBuffer().getNbLines();
            memcpy(owner.getDebugVisualizationCache().mLinesBuffer.data() + offset, scene->getRenderBuffer().getLines(), sizeof(DebugLine) * numLines);
            offset += numLines;
        }

        if (owner.getInternalPhysXDatabase().getDebugRenderBuffer().getNbLines() > 0)
        {
            const uint32_t numLines = owner.getInternalPhysXDatabase().getDebugRenderBuffer().getNbLines();
            memcpy(owner.getDebugVisualizationCache().mLinesBuffer.data() + offset,
                   owner.getInternalPhysXDatabase().getDebugRenderBuffer().getLines(),
                   sizeof(PxDebugLine) * numLines);
        }
    }

    return owner.getDebugVisualizationCache().mLinesBuffer.data();
}

uint32_t omni::physx::getNbTriangles(usdparser::AttachedStage& owner)
{    
    uint32_t numTris = 0;


    for (AttachedStage::SceneMap::const_reference ref : owner.getPhysXScenes())
    {
        PxScene* scene = ref.second->getScene();
        if (scene)
            numTris += scene->getRenderBuffer().getNbTriangles();
    }
    return numTris;
}

const DebugTriangle* omni::physx::getTriangles(usdparser::AttachedStage& owner)
{    

    if (owner.getPhysXScenes().empty())
        return nullptr;


    if (owner.getPhysXScenes().size() == 1)
        return (const DebugTriangle*)owner.getPhysXScenes().begin()->second->getScene()->getRenderBuffer().getTriangles();
    else
    {
        owner.getDebugVisualizationCache().mTriangleBuffer.resize(getNbTriangles(owner));
        uint32_t offset = 0;
        for (AttachedStage::SceneMap::const_reference ref : owner.getPhysXScenes())
        {
            PxScene* scene = ref.second->getScene();
            const uint32_t numTris = scene->getRenderBuffer().getNbTriangles();
            memcpy(owner.getDebugVisualizationCache().mTriangleBuffer.data() + offset, scene->getRenderBuffer().getTriangles(), sizeof(DebugTriangle) * numTris);
            offset += numTris;
        }
    }

    return owner.getDebugVisualizationCache().mTriangleBuffer.data();
}

namespace debugrender
{
class RenderBuffer
{
public:
    RenderBuffer(uint32_t reserve)
    {
        mLines.reserve(reserve);
    };

    template <typename T>
    void append(std::vector<T>& dst, const T* src, PxU32 count)
    {
        dst.reserve(dst.size() + count);
        for (const T* end = src + count; src < end; ++src)
            dst.push_back(*src);
    }

    size_t getNbLines() const
    {
        return mLines.size();
    }
    const PxDebugLine* getLines() const
    {
        return mLines.data();
    }

    void append(const PxRenderBuffer& other)
    {
        append(mLines, other.getLines(), other.getNbLines());
    }

    void clear()
    {
        mLines.clear();
    }

    bool empty() const
    {
        return mLines.empty();
    }

    void shift(const PxVec3& delta)
    {
        for (PxU32 i = 0; i < mLines.size(); i++)
        {
            mLines[i].pos0 += delta;
            mLines[i].pos1 += delta;
        }
    }

    std::vector<PxDebugLine> mLines;
};

class RenderOutput
{
public:
    enum Primitive
    {
        LINES,
        LINESTRIP
    };

    RenderOutput(RenderBuffer& buffer)
        : mPrim(LINES), mColor(0), mVertex0(0.0f), mVertex1(0.0f), mVertexCount(0), mTransform(PxIdentity), mBuffer(buffer)
    {
    }

    RenderOutput& operator<<(Primitive prim);
    RenderOutput& operator<<(PxU32 color); // 0xbbggrr

    RenderOutput& operator<<(PxVec3 vertex);

    RenderOutput& operator<<(const PxTransform& transform);

private:
    RenderOutput& operator=(const RenderOutput&);

    Primitive mPrim;
    PxU32 mColor;
    PxVec3 mVertex0, mVertex1;
    PxU32 mVertexCount;
    PxTransform mTransform;
    RenderBuffer& mBuffer;
};

RenderOutput& RenderOutput::operator<<(Primitive prim)
{
    mPrim = prim;
    mVertexCount = 0;
    return *this;
}

RenderOutput& RenderOutput::operator<<(PxU32 color)
{
    mColor = color;
    return *this;
}

RenderOutput& RenderOutput::operator<<(const PxTransform& t)
{
    mTransform = t;
    return *this;
}

RenderOutput& RenderOutput::operator<<(PxVec3 vertex)
{
    vertex = mTransform.transform(vertex);
    ++mVertexCount;

    switch (mPrim)
    {
    case LINES:
        if (mVertexCount == 2)
        {
            mBuffer.mLines.push_back(PxDebugLine(mVertex0, vertex, mColor));
            mVertexCount = 0;
        }
        break;
    case LINESTRIP:
        if (mVertexCount >= 2)
            mBuffer.mLines.push_back(PxDebugLine(mVertex0, vertex, mColor));
        break;
    }

    // cache the last 2 vertices (for strips)
    if (1 < mVertexCount)
    {
        mVertex1 = mVertex0;
        mVertex0 = vertex;
    }
    else
    {
        mVertex0 = vertex;
    }
    return *this;
}

struct DebugCircle
{
    DebugCircle(PxU32 s, PxReal r) : nSegments(s), radius(r)
    {
    }
    PxU32 nSegments;
    PxReal radius;
};
RenderOutput& operator<<(RenderOutput& out, const DebugCircle& circle)
{
    const PxF32 step = PxTwoPi / circle.nSegments;
    PxF32 angle = 0;
    out << RenderOutput::LINESTRIP;
    for (PxU32 i = 0; i < circle.nSegments; i++, angle += step)
        out << PxVec3(circle.radius * PxSin(angle), circle.radius * PxCos(angle), 0);
    out << PxVec3(0, circle.radius, 0);
    return out;
}

struct DebugArc
{
    DebugArc(PxU32 s, PxReal r, PxReal minAng, PxReal maxAng)
        : nSegments(s), radius(r), minAngle(minAng), maxAngle(maxAng)
    {
    }
    PxU32 nSegments;
    PxReal radius;
    PxReal minAngle, maxAngle;
};
RenderOutput& operator<<(RenderOutput& out, const DebugArc& arc)
{
    const PxF32 step = (arc.maxAngle - arc.minAngle) / arc.nSegments;
    PxF32 angle = arc.minAngle;
    out << RenderOutput::LINESTRIP;
    for (PxU32 i = 0; i < arc.nSegments; i++, angle += step)
        out << PxVec3(arc.radius * PxSin(angle), arc.radius * PxCos(angle), 0);
    out << PxVec3(arc.radius * PxSin(arc.maxAngle), arc.radius * PxCos(arc.maxAngle), 0);
    return out;
}
} // namespace debugrender

void drawConvexMesh(debugrender::RenderOutput& out, const PxConvexMesh* convexMesh, const PxVec3& scale)
{
    if ( !convexMesh ) return;
    const PxVec3* vertices = convexMesh->getVertices();
    const PxU8* indexBuffer = convexMesh->getIndexBuffer();
    const PxU32 nbPolygons = convexMesh->getNbPolygons();

    for (PxU32 i = 0; i < nbPolygons; i++)
    {
        PxHullPolygon polygon;
        convexMesh->getPolygonData(i, polygon);
        const PxU32 pnbVertices = polygon.mNbVerts;

        PxVec3 begin = vertices[indexBuffer[0]].multiply(scale);
        for (PxU32 j = 1; j < pnbVertices; j++)
        {
            const PxVec3 end = (vertices[indexBuffer[j]]).multiply(scale);
            out << begin;
            out << end;
            begin = end;
        }

        out << begin;
        out << vertices[indexBuffer[0]].multiply(scale);

        indexBuffer += pnbVertices;
    }
}

void drawTriMesh(const usdparser::AttachedStage& owner, debugrender::RenderOutput& out, const PxTriangleMesh* triMesh, const PxVec3& scale)
{
    if ( !triMesh )
        return;

    const usdparser::AttachedStage* attachedStage = &owner;
    if (!attachedStage)
    {
        return;
    }

    const bool renderNormals = owner.isNormalsVisualizationEnabled();
    const float maxSize = 0.05f / attachedStage->getSourceUnits().metersPerUnit;
    const float size = 0.05f;

    // JIRA OM-96841: Change SDF debug visualization back to just rendering the triangle mesh
    // PxU32 dimX, dimY, dimZ;
    // triMesh->getSDFDimensions(dimX, dimY, dimZ);
    // if (triMesh->getSDF() != NULL && dimX > 0 && dimY > 0 && dimZ > 0)
    if (0)
    {
        PxArray<PxVec3> vertices;
        PxArray<PxU32> triangles;
        PxExtractIsosurfaceFromSDF(*triMesh, vertices, triangles);
        const PxU32 numTriangles = triangles.size() / 3;

        for (PxU32 i = 0; i < numTriangles; i++)
        {
            const PxVec3 p0 = vertices[triangles[i * 3]].multiply(scale);
            const PxVec3 p1 = vertices[triangles[i * 3 + 1]].multiply(scale);
            const PxVec3 p2 = vertices[triangles[i * 3 + 2]].multiply(scale);

            out << p0;
            out << p1;
            out << p1;
            out << p2;
            out << p2;
            out << p0;

            if (renderNormals)
            {
                PxVec3 normal = (p1 - p0).cross(p2 - p0);
                normal.normalize();
                const float triSize = fmin(normal.normalize() * size, maxSize);

                // normals
                out << p0;
                out << p0 + normal * triSize;
                out << p1;
                out << p1 + normal * triSize;
                out << p2;
                out << p2 + normal * triSize;
            }
        }
    }
    else
    {
        const bool use16bits = triMesh->getTriangleMeshFlags() & PxTriangleMeshFlag::e16_BIT_INDICES;        

        const PxVec3* vertices = triMesh->getVertices();
        const PxU32 numTriangles = triMesh->getNbTriangles();

        if (use16bits)
        {
            const PxU16* triangles = reinterpret_cast<const PxU16*>(triMesh->getTriangles());

            for (PxU32 i = 0; i < numTriangles; i++)
            {
                const PxVec3 p0 = vertices[triangles[i * 3]].multiply(scale);
                const PxVec3 p1 = vertices[triangles[i * 3 + 1]].multiply(scale);
                const PxVec3 p2 = vertices[triangles[i * 3 + 2]].multiply(scale);

                out << p0;
                out << p1;
                out << p1;
                out << p2;
                out << p2;
                out << p0;

                if (renderNormals)
                {
                    const float sign = (scale.x * scale.y * scale.z < 0.0f) ? -1.0f : 1.0f;
                    PxVec3 normal = (p1 - p0).cross(p2 - p0) * sign;
                    const float triSize = fmin(normal.normalize() * size, maxSize);

                    // normals
                    out << p0;
                    out << p0 + normal * triSize;
                    out << p1;
                    out << p1 + normal * triSize;
                    out << p2;
                    out << p2 + normal * triSize;
                }
            }
        }
        else
        {
            const PxU32* triangles = reinterpret_cast<const PxU32*>(triMesh->getTriangles());

            for (PxU32 i = 0; i < numTriangles; i++)
            {
                const PxVec3 p0 = vertices[triangles[i * 3]].multiply(scale);
                const PxVec3 p1 = vertices[triangles[i * 3 + 1]].multiply(scale);
                const PxVec3 p2 = vertices[triangles[i * 3 + 2]].multiply(scale);

                out << p0;
                out << p1;
                out << p1;
                out << p2;
                out << p2;
                out << p0;

                if (renderNormals)
                {
                    PxVec3 normal = (p1 - p0).cross(p2 - p0);
                    normal.normalize();
                    const float triSize = fmin(normal.normalize() * size, maxSize);

                    // normals
                    out << p0;
                    out << p0 + normal * triSize;
                    out << p1;
                    out << p1 + normal * triSize;
                    out << p2;
                    out << p2 + normal * triSize;
                }
            }
        }
    }
}




void DebugVisualizationCache::release()
{
    for (LineMap::const_reference& it : mLineMap)
    {
        delete[] it.second.mDebugLines;
    }
    mLineMap.clear();

    for (EdgeMap::const_reference& it : mEdgeMap)
    {
        delete[] it.second.mDebugEdges;
    }
    mEdgeMap.clear();

    mPointsBuffer.clear();
    mLinesBuffer.clear();
    mTriangleBuffer.clear();
    mShapeBuffer.clear();
}

void DebugVisualizationCache::releaseKey(omni::physics::parse::ObjectKey key)
{
    {
        LineMap::iterator it = mLineMap.find(key);
        if (it != mLineMap.end())
        {
            delete[] it->second.mDebugLines;
            mLineMap.erase(it);
        }
    }

    {
        EdgeMap::iterator it = mEdgeMap.find(key);
        if (it != mEdgeMap.end())
        {
            delete[] it->second.mDebugEdges;
            mEdgeMap.erase(it);
        }
    }
}

const DebugLine* DebugVisualizationCache::getLines(omni::physics::parse::ObjectKey key,
                                                   const PxTransform& transform,
                                                   uint32_t& numLines)
{
    LineMap::iterator it = mLineMap.find(key);
    if (it != mLineMap.end())
    {
        CachedLines& lines = it->second;
        if (lines.mTransform == transform)
        {
            numLines = lines.mNumLines;
            return lines.mDebugLines;
        }
        else
        {
            delete[] lines.mDebugLines; // TODO FIXME so we are now leaving entry in the map with no lines?
            return nullptr;
        }
    }

    return nullptr;
}

void DebugVisualizationCache::addLines(omni::physics::parse::ObjectKey key,
                                       const PxTransform& transform,
                                       DebugLine* debugLines,
                                       uint32_t numLines)
{
    // TODO FIXME: release in case key exists (not an issue right now).
    mLineMap[key] = { debugLines, transform, numLines };
}

const DebugEdge* DebugVisualizationCache::getEdges(omni::physics::parse::ObjectKey key, uint32_t& numEdges)
{
    EdgeMap::iterator it = mEdgeMap.find(key);
    if (it != mEdgeMap.end())
    {
        CachedEdges& edges = it->second;
        numEdges = edges.mNumEdges;
        return edges.mDebugEdges;
    }
    return nullptr;
}

void DebugVisualizationCache::addEdges(omni::physics::parse::ObjectKey key, DebugEdge* debugEdges, uint32_t numEdges)
{
    mEdgeMap[key] = { debugEdges, numEdges };
}

namespace
{
    struct OmniDebugRenderBuffer : PxRenderBuffer
    {
        debugrender::RenderBuffer& buff;
        OmniDebugRenderBuffer(debugrender::RenderBuffer& b) : buff(b) {}
        virtual PxU32 getNbPoints() const { return 0; }
        virtual const PxDebugPoint* getPoints() const { return nullptr; }
        virtual void addPoint(const PxDebugPoint&) {}
        virtual PxU32 getNbLines() const { return (PxU32)buff.getNbLines(); }
        virtual const PxDebugLine* getLines() const { return buff.getLines(); }
        virtual void addLine(const PxDebugLine& line) { buff.mLines.push_back(line); }
        virtual PxDebugLine* reserveLines(const PxU32 nbLines) { return nullptr; }
        virtual PxDebugPoint* reservePoints(const PxU32 nbLines) { return nullptr; }
        virtual PxU32 getNbTriangles() const { return 0; }
        virtual const PxDebugTriangle* getTriangles() const { return nullptr; }
        virtual void addTriangle(const PxDebugTriangle& triangle) {}
        virtual void append(const PxRenderBuffer& other) {}
        virtual void clear() { buff.clear(); }
        virtual void shift(const PxVec3& delta) { buff.shift(delta); }
        virtual bool empty() const { return buff.empty(); }
    };
}

const DebugLine* omni::physx::getShapeDebugDraw(usdparser::AttachedStage& owner, omni::physics::parse::ObjectKey key,
                                                const usdparser::PhysxShapeDesc* desc,
                                                uint32_t& numLines)
{
    debugrender::RenderBuffer gRenderBuffer(64);
    const DebugLine* outLines = nullptr;
    numLines = 0;

    CookingDataAsync* cookingDataAsync = OmniPhysX::getInstance().getPhysXSetup().getCookingDataAsync();
    if (!cookingDataAsync || !desc)
    {
        return outLines;
    }

    usdparser::AttachedStage* attachedStage = &owner;
    if (!attachedStage)
    {
        return outLines;
    }

    uint32_t collShapeColor = getDebugDrawCollShapeColor(owner, key);
    PxTransform transform = toPhysX(desc->localPos, desc->localRot);

    switch (desc->type)
    {
    case eSpherePointsShape:
    {
        debugrender::RenderOutput out(gRenderBuffer);
        const uint32_t numSegments = 100;
        const SpherePointsPhysxShapeDesc* sphereDesc = (SpherePointsPhysxShapeDesc*)desc;
        if ( sphereDesc->spheres.empty() )
        {
            const SpherePointsPhysxShapeDesc *sp = cookingDataAsync->getSpherePoints(*sphereDesc, key, *attachedStage, true);
            if ( sp )
            {
                sphereDesc = sp;
            }
        }
        for (auto &i:sphereDesc->spheres)
        {
            PxTransform stransform = transform;
            PxVec3 pos = *(const PxVec3 *)&i.position;
            PxVec3 rpos = transform.rotate(pos);
            stransform.p+=rpos;

            float radius = i.radius;

            out << collShapeColor;
            out << stransform;
            out << debugrender::DebugCircle(numSegments, radius);

            const float hRt2 = sqrt(2.0f) / 2.0f;
            const PxQuat rot = stransform.q;
            stransform.q = rot * PxQuat(-hRt2, 0.0f, 0.0f, hRt2);
            out << stransform;
            out << debugrender::DebugCircle(numSegments, radius);

            stransform.q = rot * PxQuat(0.0f, -hRt2, 0.0f, hRt2);
            out << stransform;
            out << debugrender::DebugCircle(numSegments, radius);

        }
    }
        break;
    case eSphereShape:
    {
        SpherePhysxShapeDesc* sphereDesc = (SpherePhysxShapeDesc*)desc;
        debugrender::RenderOutput out(gRenderBuffer);

        const uint32_t numSegments = 100;

        out << collShapeColor;
        out << transform;
        out << debugrender::DebugCircle(numSegments, sphereDesc->radius);

        const float hRt2 = sqrt(2.0f) / 2.0f;
        const PxQuat rot = transform.q;
        transform.q = rot * PxQuat(-hRt2, 0.0f, 0.0f, hRt2);
        out << transform;
        out << debugrender::DebugCircle(numSegments, sphereDesc->radius);

        transform.q = rot * PxQuat(0.0f, -hRt2, 0.0f, hRt2);
        out << transform;
        out << debugrender::DebugCircle(numSegments, sphereDesc->radius);
    }
    break;
    case eBoxShape:
    {
        BoxPhysxShapeDesc* boxDesc = (BoxPhysxShapeDesc*)desc;

        const PxVec3 minimum(-boxDesc->halfExtents.x, -boxDesc->halfExtents.y, -boxDesc->halfExtents.z);
        const PxVec3 maximum(boxDesc->halfExtents.x, boxDesc->halfExtents.y, boxDesc->halfExtents.z);

        debugrender::RenderOutput out(gRenderBuffer);

        out << collShapeColor;
        out << transform;
        out << debugrender::RenderOutput::LINESTRIP;
        out << PxVec3(minimum.x, minimum.y, minimum.z);
        out << PxVec3(maximum.x, minimum.y, minimum.z);
        out << PxVec3(maximum.x, maximum.y, minimum.z);
        out << PxVec3(minimum.x, maximum.y, minimum.z);
        out << PxVec3(minimum.x, minimum.y, minimum.z);
        out << PxVec3(minimum.x, minimum.y, maximum.z);
        out << PxVec3(maximum.x, minimum.y, maximum.z);
        out << PxVec3(maximum.x, maximum.y, maximum.z);
        out << PxVec3(minimum.x, maximum.y, maximum.z);
        out << PxVec3(minimum.x, minimum.y, maximum.z);
        out << debugrender::RenderOutput::LINES;
        out << PxVec3(maximum.x, minimum.y, minimum.z);
        out << PxVec3(maximum.x, minimum.y, maximum.z);
        out << PxVec3(maximum.x, maximum.y, minimum.z);
        out << PxVec3(maximum.x, maximum.y, maximum.z);
        out << PxVec3(minimum.x, maximum.y, minimum.z);
        out << PxVec3(minimum.x, maximum.y, maximum.z);
    }
    break;
    case eBoundingSphereShape:
    {
        BoundingSpherePhysxShapeDesc* sphereDesc = (BoundingSpherePhysxShapeDesc*)desc;
        debugrender::RenderOutput out(gRenderBuffer);

        const uint32_t numSegments = 100;

        const PxVec3 offsetPos = toPhysX(sphereDesc->positionOffset);
        transform.p = transform.p + transform.q.rotate(offsetPos);

        out << collShapeColor;
        out << transform;
        out << debugrender::DebugCircle(numSegments, sphereDesc->radius);

        const float hRt2 = sqrt(2.0f) / 2.0f;
        const PxQuat rot = transform.q;
        transform.q = rot * PxQuat(-hRt2, 0.0f, 0.0f, hRt2);
        out << transform;
        out << debugrender::DebugCircle(numSegments, sphereDesc->radius);

        transform.q = rot * PxQuat(0.0f, -hRt2, 0.0f, hRt2);
        out << transform;
        out << debugrender::DebugCircle(numSegments, sphereDesc->radius);
    }
    break;
    case eBoundingBoxShape:
    {
        BoundingBoxPhysxShapeDesc* boxDesc = (BoundingBoxPhysxShapeDesc*)desc;
        debugrender::RenderOutput out(gRenderBuffer);

        const PxTransform offsetTransform(toPhysX(boxDesc->positionOffset), toPhysXQuat(boxDesc->rotationOffset));
        transform = transform * offsetTransform;

        const PxVec3 minimum(-boxDesc->halfExtents.x, -boxDesc->halfExtents.y, -boxDesc->halfExtents.z);
        const PxVec3 maximum(boxDesc->halfExtents.x, boxDesc->halfExtents.y, boxDesc->halfExtents.z);

        out << collShapeColor;
        out << transform;

        out << debugrender::RenderOutput::LINESTRIP;
        out << PxVec3(minimum.x, minimum.y, minimum.z);
        out << PxVec3(maximum.x, minimum.y, minimum.z);
        out << PxVec3(maximum.x, maximum.y, minimum.z);
        out << PxVec3(minimum.x, maximum.y, minimum.z);
        out << PxVec3(minimum.x, minimum.y, minimum.z);
        out << PxVec3(minimum.x, minimum.y, maximum.z);
        out << PxVec3(maximum.x, minimum.y, maximum.z);
        out << PxVec3(maximum.x, maximum.y, maximum.z);
        out << PxVec3(minimum.x, maximum.y, maximum.z);
        out << PxVec3(minimum.x, minimum.y, maximum.z);
        out << debugrender::RenderOutput::LINES;
        out << PxVec3(maximum.x, minimum.y, minimum.z);
        out << PxVec3(maximum.x, minimum.y, maximum.z);
        out << PxVec3(maximum.x, maximum.y, minimum.z);
        out << PxVec3(maximum.x, maximum.y, maximum.z);
        out << PxVec3(minimum.x, maximum.y, minimum.z);
        out << PxVec3(minimum.x, maximum.y, maximum.z);
    }
    break;
    case eCapsuleShape:
    {
        CapsulePhysxShapeDesc* capsuleDesc = (CapsulePhysxShapeDesc*)desc;

        const float hRt2 = sqrt(2.0f) / 2.0f;
        PxQuat fixupQ(0.0f, hRt2, hRt2, 0.0f);
        int axis = 0;
        if (capsuleDesc->axis == eY)
        {
            axis = 1;
            fixupQ = PxQuat(hRt2, -hRt2, 0.0f, 0.0f);
        }
        else if (capsuleDesc->axis == eZ)
        {
            axis = 2;
            fixupQ = PxQuat(hRt2, 0.0f, -hRt2, 0.0f);
        }

        debugrender::RenderOutput out(gRenderBuffer);
        out << collShapeColor;

        const PxVec3 vleft2 = axis == 0 ? PxVec3(-capsuleDesc->halfHeight, 0.0f, 0.0f) :
                                          axis == 1 ? PxVec3(0.0f, -capsuleDesc->halfHeight, 0.0f) :
                                                      PxVec3(0.0f, 0.0f, -capsuleDesc->halfHeight);
        PxTransform left2 = transform.transform(PxTransform(vleft2));
        left2.q = left2.q * fixupQ;
        out << left2 << debugrender::DebugArc(100, capsuleDesc->radius, 0.0f, PxPi);

        PxTransform rotPose = left2;
        PxQuat rot = left2.q;
        rotPose.q = rot * PxQuat(-hRt2, 0.0f, 0.0f, hRt2);
        out << rotPose << debugrender::DebugArc(100, capsuleDesc->radius, 0.0f, PxPi);

        rotPose.q = rot * PxQuat(0.0f, -hRt2, 0.0f, hRt2);
        out << rotPose << debugrender::DebugCircle(100, capsuleDesc->radius);

        const PxVec3 vright2 = axis == 0 ? PxVec3(capsuleDesc->halfHeight, 0.0f, 0.0f) :
                                           axis == 1 ? PxVec3(0.0f, capsuleDesc->halfHeight, 0.0f) :
                                                       PxVec3(0.0f, 0.0f, capsuleDesc->halfHeight);
        PxTransform right2 = transform.transform(PxTransform(vright2));
        right2.q = right2.q * fixupQ;
        out << right2 << debugrender::DebugArc(100, capsuleDesc->radius, PxPi, PxTwoPi);

        rotPose = right2;
        rot = left2.q;
        rotPose.q = rot * PxQuat(-hRt2, 0.0f, 0.0f, hRt2);
        out << rotPose << debugrender::DebugArc(100, capsuleDesc->radius, PxPi, PxTwoPi);

        rotPose.q = rot * PxQuat(0.0f, -hRt2, 0.0f, hRt2);
        out << rotPose << debugrender::DebugCircle(100, capsuleDesc->radius);

        // rotPose = transform;
        // rotPose.q = transform.q * fixupQ;
        out << transform;
        out << debugrender::RenderOutput::LINES;
        if (capsuleDesc->axis == eX)
        {
            out << PxVec3(-capsuleDesc->halfHeight, capsuleDesc->radius, 0);
            out << PxVec3(capsuleDesc->halfHeight, capsuleDesc->radius, 0);
            out << PxVec3(-capsuleDesc->halfHeight, -capsuleDesc->radius, 0);
            out << PxVec3(capsuleDesc->halfHeight, -capsuleDesc->radius, 0);
            out << PxVec3(-capsuleDesc->halfHeight, 0, capsuleDesc->radius);
            out << PxVec3(capsuleDesc->halfHeight, 0, capsuleDesc->radius);
            out << PxVec3(-capsuleDesc->halfHeight, 0, -capsuleDesc->radius);
            out << PxVec3(capsuleDesc->halfHeight, 0, -capsuleDesc->radius);
        }
        else if (capsuleDesc->axis == eY)
        {
            out << PxVec3(0, -capsuleDesc->halfHeight, capsuleDesc->radius);
            out << PxVec3(0, capsuleDesc->halfHeight, capsuleDesc->radius);
            out << PxVec3(0, -capsuleDesc->halfHeight, -capsuleDesc->radius);
            out << PxVec3(0, capsuleDesc->halfHeight, -capsuleDesc->radius);
            out << PxVec3(capsuleDesc->radius, -capsuleDesc->halfHeight, 0);
            out << PxVec3(capsuleDesc->radius, capsuleDesc->halfHeight, 0);
            out << PxVec3(-capsuleDesc->radius, -capsuleDesc->halfHeight, 0);
            out << PxVec3(-capsuleDesc->radius, capsuleDesc->halfHeight, 0);
        }
        else if (capsuleDesc->axis == eZ)
        {
            out << PxVec3(capsuleDesc->radius, 0, -capsuleDesc->halfHeight);
            out << PxVec3(capsuleDesc->radius, 0, capsuleDesc->halfHeight);
            out << PxVec3(-capsuleDesc->radius, 0, -capsuleDesc->halfHeight);
            out << PxVec3(-capsuleDesc->radius, 0, capsuleDesc->halfHeight);
            out << PxVec3(0, capsuleDesc->radius, -capsuleDesc->halfHeight);
            out << PxVec3(0, capsuleDesc->radius, capsuleDesc->halfHeight);
            out << PxVec3(0, -capsuleDesc->radius, -capsuleDesc->halfHeight);
            out << PxVec3(0, -capsuleDesc->radius, capsuleDesc->halfHeight);
        }
    }
    break;
    case eCylinderShape:
    {
        CylinderPhysxShapeDesc* cylinderDesc = (CylinderPhysxShapeDesc*)desc;

        if (attachedStage->getCachedSettings().approximateCylinders)
        {
            const PxVec3 scale = getConeOrCylinderScale(cylinderDesc->halfHeight, cylinderDesc->radius,
                cylinderDesc->axis);

            debugrender::RenderOutput out(gRenderBuffer);
            out << collShapeColor;

            out << transform;
            out << debugrender::RenderOutput::LINES;

            const PxConvexMesh* convexMesh = OmniPhysX::getInstance().getPhysXSetup().getCylinderConvexMesh(cylinderDesc->axis);

            drawConvexMesh(out, convexMesh, scale);
        }
        else
        {
            PxConvexCoreGeometry geom(PxConvexCore::Cylinder(cylinderDesc->halfHeight * 2.0f, cylinderDesc->radius), cylinderDesc->margin);
            PxTransform pose = cylinderDesc->axis == eZ ? transform.transform(PxTransform(PxQuat(PxPiDivTwo, PxVec3(0, -1, 0)))) :
                                cylinderDesc->axis == eY ? transform.transform(PxTransform(PxQuat(PxPiDivTwo, PxVec3(0, 0, 1)))) :
                                transform;
            OmniDebugRenderBuffer omniRenderBuffer(gRenderBuffer);
            PxRenderOutput output(omniRenderBuffer);
            PxConvexCoreExt::visualize(geom, pose, false, PxBounds3(), output);
        }
    }
    break;
    case eConeShape:
    {
        ConePhysxShapeDesc* coneDesc = (ConePhysxShapeDesc*)desc;

        if (attachedStage->getCachedSettings().approximateCones)
        {
            const PxVec3 scale = getConeOrCylinderScale(coneDesc->halfHeight, coneDesc->radius,
                coneDesc->axis);

            debugrender::RenderOutput out(gRenderBuffer);
            out << collShapeColor;

            out << transform;
            out << debugrender::RenderOutput::LINES;

            const PxConvexMesh* convexMesh = OmniPhysX::getInstance().getPhysXSetup().getConeConvexMesh(coneDesc->axis);

            drawConvexMesh(out, convexMesh, scale);
        }
        else
        {
            PxConvexCoreGeometry geom(PxConvexCore::Cone(coneDesc->halfHeight * 2.0f, coneDesc->radius), coneDesc->margin);
            PxTransform pose = coneDesc->axis == eZ ? transform.transform(PxTransform(PxQuat(PxPiDivTwo, PxVec3(0, -1, 0)))) :
                               coneDesc->axis == eY ? transform.transform(PxTransform(PxQuat(PxPiDivTwo, PxVec3(0, 0, 1)))) :
                               transform;
            OmniDebugRenderBuffer omniRenderBuffer(gRenderBuffer);
            PxRenderOutput output(omniRenderBuffer);
            PxConvexCoreExt::visualize(geom, pose, false, PxBounds3(), output);
        }
    }
    break;
    case eConvexMeshShape:
    {
        ConvexMeshPhysxShapeDesc* convexDesc = (ConvexMeshPhysxShapeDesc*)desc;

        debugrender::RenderOutput out(gRenderBuffer);
        out << collShapeColor;

        out << transform;
        out << debugrender::RenderOutput::LINES;

        PxConvexMesh* convexMesh = cookingDataAsync->getConvexMesh(*convexDesc, key, *attachedStage, true);
        if ( convexMesh )
        {
            drawConvexMesh(
                out, convexMesh,
                PxVec3(
                    fabsf(convexDesc->meshScale.x) * convexDesc->convexCookingParams.signScale.x, 
                    fabsf(convexDesc->meshScale.y) * convexDesc->convexCookingParams.signScale.y, 
                    fabsf(convexDesc->meshScale.z) * convexDesc->convexCookingParams.signScale.z));
        }
    }
    break;
    case eConvexMeshDecompositionShape:
    {
        ConvexMeshDecompositionPhysxShapeDesc* convexDecompositionDesc = (ConvexMeshDecompositionPhysxShapeDesc*)desc;

        debugrender::RenderOutput out(gRenderBuffer);
        out << collShapeColor;

        out << transform;
        out << debugrender::RenderOutput::LINES;

        std::vector<PxConvexMesh*> convexMeshes = cookingDataAsync->getConvexMeshDecomposition(*convexDecompositionDesc, key, *attachedStage, true);
        if ( !convexMeshes.empty() )
        {
            const PxVec3 scale( fabsf(convexDecompositionDesc->meshScale.x) * convexDecompositionDesc->convexDecompositionCookingParams.signScale.x,
                                fabsf(convexDecompositionDesc->meshScale.y) * convexDecompositionDesc->convexDecompositionCookingParams.signScale.y,
                                fabsf(convexDecompositionDesc->meshScale.z) * convexDecompositionDesc->convexDecompositionCookingParams.signScale.z);

            for (size_t i = 0; i < convexMeshes.size(); i++)
            {
                drawConvexMesh(out, convexMeshes[i], scale);
            }
        }
    }
    break;
    case eTriangleMeshShape:
    {
        outLines = owner.getDebugVisualizationCache().getLines(key, transform, numLines);
        if (!outLines)
        {
            TriangleMeshPhysxShapeDesc* meshDesc = (TriangleMeshPhysxShapeDesc*)desc;

            debugrender::RenderOutput out(gRenderBuffer);
            out << collShapeColor;

            out << transform;
            out << debugrender::RenderOutput::LINES;

            PxTriangleMesh* triMesh = cookingDataAsync->getTriangleMesh(*meshDesc, key, *attachedStage, true);
            if ( triMesh )
            {
                drawTriMesh(owner, out, triMesh, toPhysX(meshDesc->meshScale));
                if (!gRenderBuffer.empty())
                {
                    const size_t nLines = gRenderBuffer.getNbLines();
                    DebugLine* cachedLines = new DebugLine[nLines];
                    memcpy(cachedLines, gRenderBuffer.getLines(), sizeof(DebugLine) * nLines);
                    owner.getDebugVisualizationCache().addLines(key, transform, cachedLines, (uint32_t)nLines);
                }
            }
        }
    }
    break;
    case ePlaneShape:
    {
        PlanePhysxShapeDesc* planeDesc = (PlanePhysxShapeDesc*)desc;

        const float hRt2 = sqrt(2.0f) / 2.0f;
        PxQuat fixupQ(PxIdentity);
        const float normalLength = 0.5f / owner.getSourceUnits().metersPerUnit;
        PxVec3 normal(0.0f, 0.0f, normalLength);
        if (planeDesc->axis == eY)
        {
            fixupQ = PxQuat(hRt2, 0.0f, 0.0f, hRt2);
            normal = PxVec3(0.0f, normalLength, 0.0f);
        }
        else if (planeDesc->axis == eX)
        {
            fixupQ = PxQuat(0.0f, hRt2, 0.0f, hRt2);
            normal = PxVec3(normalLength, 0.0f, 0.0f);
        }

        debugrender::RenderOutput out(gRenderBuffer);
        out << collShapeColor;

        PxTransform rotPose = transform;
        rotPose.q = transform.q * fixupQ;
        out << rotPose;

        for (PxReal radius = 2.0f; radius < 20.0f; radius += 2.0f)
            out << debugrender::DebugCircle(100, radius * radius);

        out << debugrender::RenderOutput::LINES;
        out << transform;
        out << PxVec3(0.0f, 0.0f, 0.0f);
        out << normal;
    }
    break;
    default:
    break;
    }

    if (!gRenderBuffer.empty())
    {
        numLines = (uint32_t)gRenderBuffer.getNbLines();
        std::vector<DebugLine>& output = owner.getDebugVisualizationCache().mShapeBuffer;
        output.resize(numLines);
        memcpy(output.data(), gRenderBuffer.getLines(), sizeof(DebugLine) * numLines);
        outLines = output.data();
    }

    return outLines;
}

void omni::physx::clearDebugVisualizationData(usdparser::AttachedStage& owner)
{
    owner.getDebugVisualizationCache().release();
}

const omni::physx::CollisionRepresentation* omni::physx::getCollisionRepresentation(usdparser::AttachedStage& owner, omni::physics::parse::ObjectKey key,const usdparser::PhysxShapeDesc* desc)
{
    const CollisionRepresentation *ret = nullptr;

    CookingDataAsync* cookingDataAsync = OmniPhysX::getInstance().getPhysXSetup().getCookingDataAsync();
    if (cookingDataAsync && desc)
    {
        ret = cookingDataAsync->getCollisionRepresentation(key,*desc);
    }

    return ret;
}
void omni::physx::releaseCollisionRepresentation(const CollisionRepresentation* cr)
{
    // A representation is an independent heap copy. Its release neither resolves
    // physics nor requires the SDK/cooking service to remain alive.
    if (!cr)
        return;
    for (uint32_t i = 0; i < cr->meshCount; ++i)
    {
        delete[] cr->meshes[i].vertices;
        delete[] cr->meshes[i].indices;
    }
    delete[] cr->meshes;
    delete cr;
}

bool omni::physx::getMeshKey(const omni::physx::usdparser::PhysxShapeDesc &desc,omni::physx::usdparser::MeshKey &meshKey)
{
    RuntimeExecutionScope execution(kNoAttach);
    if (!execution)
        return false;
    bool ret = false;

    CookingDataAsync* cookingDataAsync = OmniPhysX::getInstance().getPhysXSetup().getCookingDataAsync();
    if (cookingDataAsync)
    {
        ret = cookingDataAsync->getMeshkey(desc,meshKey);
    }
    return ret;
}

typedef std::unordered_set<Pair<uint32_t>, PairHash> EdgeSet;

template<typename T>
void addTriangleMesh(EdgeSet& edgeSet, const T* triangles, uint32_t nbTriangles)
{
    for (PxU32 t = 0; t < nbTriangles * 3; t += 3)
    {
        uint32_t v0 = triangles[t];
        uint32_t v1 = triangles[t + 1];
        uint32_t v2 = triangles[t + 2];

        edgeSet.insert(Pair<uint32_t>(v0, v1));
        edgeSet.insert(Pair<uint32_t>(v1, v2));
        edgeSet.insert(Pair<uint32_t>(v2, v0));
    }
}

// Public admission boundaries. Private overloads above never acquire nested admission.
void omni::physx::enableVisualization(AttachHandle attachHandle, bool enableVis)
{
    RuntimeExecutionScope execution(attachHandle);
    if (!execution)
        return;
    usdparser::AttachedStage* owner = UsdLoad::getUsdLoad()->getAttachedStageByHandle(attachHandle);
    if (!owner)
        return;
    enableVisualization(*owner, enableVis);
}

void omni::physx::enableNormalsVisualization(AttachHandle attachHandle, bool enableNormalsVis)
{
    RuntimeExecutionScope execution(attachHandle);
    if (!execution)
        return;
    usdparser::AttachedStage* owner = UsdLoad::getUsdLoad()->getAttachedStageByHandle(attachHandle);
    if (!owner)
        return;
    enableNormalsVisualization(*owner, enableNormalsVis);
}

void omni::physx::setVisualizationScale(AttachHandle attachHandle, float scale)
{
    RuntimeExecutionScope execution(attachHandle);
    if (!execution)
        return;
    usdparser::AttachedStage* owner = UsdLoad::getUsdLoad()->getAttachedStageByHandle(attachHandle);
    if (!owner)
        return;
    setVisualizationScale(*owner, scale);
}

void omni::physx::setVisualizationCullingBox(AttachHandle attachHandle, const carb::Float3& min, const carb::Float3& max)
{
    RuntimeExecutionScope execution(attachHandle);
    if (!execution)
        return;
    usdparser::AttachedStage* owner = UsdLoad::getUsdLoad()->getAttachedStageByHandle(attachHandle);
    if (!owner)
        return;
    setVisualizationCullingBox(*owner, min, max);
}

void omni::physx::setVisualizationParameter(AttachHandle attachHandle, PhysXVisualizationParameter par, bool val)
{
    RuntimeExecutionScope execution(attachHandle);
    if (!execution)
        return;
    usdparser::AttachedStage* owner = UsdLoad::getUsdLoad()->getAttachedStageByHandle(attachHandle);
    if (!owner)
        return;
    setVisualizationParameter(*owner, par, val);
}

void omni::physx::setVisualizationParameterValue(AttachHandle attachHandle, PhysXVisualizationParameter par, float value)
{
    RuntimeExecutionScope execution(attachHandle);
    if (!execution)
        return;
    usdparser::AttachedStage* owner = UsdLoad::getUsdLoad()->getAttachedStageByHandle(attachHandle);
    if (!owner)
        return;
    setVisualizationParameterValue(*owner, par, value);
}

bool omni::physx::setVisualizationScopeTokens(AttachHandle attachHandle, const ovx_primpath_t* tokens, uint32_t count)
{
    RuntimeExecutionScope execution(attachHandle);
    if (!execution)
        return false;
    usdparser::AttachedStage* owner = UsdLoad::getUsdLoad()->getAttachedStageByHandle(attachHandle);
    if (!owner)
        return false;
    return setVisualizationScopeTokens(*owner, tokens, count);
}

uint32_t omni::physx::getNbPoints(AttachHandle attachHandle)
{
    RuntimeExecutionScope execution(attachHandle);
    if (!execution)
        return 0;
    usdparser::AttachedStage* owner = UsdLoad::getUsdLoad()->getAttachedStageByHandle(attachHandle);
    if (!owner)
        return 0;
    return getNbPoints(*owner);
}

const DebugPoint* omni::physx::getPoints(AttachHandle attachHandle)
{
    RuntimeExecutionScope execution(attachHandle);
    if (!execution)
        return nullptr;
    usdparser::AttachedStage* owner = UsdLoad::getUsdLoad()->getAttachedStageByHandle(attachHandle);
    if (!owner)
        return nullptr;
    return getPoints(*owner);
}

uint32_t omni::physx::getNbLines(AttachHandle attachHandle)
{
    RuntimeExecutionScope execution(attachHandle);
    if (!execution)
        return 0;
    usdparser::AttachedStage* owner = UsdLoad::getUsdLoad()->getAttachedStageByHandle(attachHandle);
    if (!owner)
        return 0;
    return getNbLines(*owner);
}

const DebugLine* omni::physx::getLines(AttachHandle attachHandle)
{
    RuntimeExecutionScope execution(attachHandle);
    if (!execution)
        return nullptr;
    usdparser::AttachedStage* owner = UsdLoad::getUsdLoad()->getAttachedStageByHandle(attachHandle);
    if (!owner)
        return nullptr;
    return getLines(*owner);
}

uint32_t omni::physx::getNbTriangles(AttachHandle attachHandle)
{
    RuntimeExecutionScope execution(attachHandle);
    if (!execution)
        return 0;
    usdparser::AttachedStage* owner = UsdLoad::getUsdLoad()->getAttachedStageByHandle(attachHandle);
    if (!owner)
        return 0;
    return getNbTriangles(*owner);
}

const DebugTriangle* omni::physx::getTriangles(AttachHandle attachHandle)
{
    RuntimeExecutionScope execution(attachHandle);
    if (!execution)
        return nullptr;
    usdparser::AttachedStage* owner = UsdLoad::getUsdLoad()->getAttachedStageByHandle(attachHandle);
    if (!owner)
        return nullptr;
    return getTriangles(*owner);
}

const DebugLine* omni::physx::getShapeDebugDraw(AttachHandle attachHandle, omni::physics::parse::ObjectKey key,
                                                const usdparser::PhysxShapeDesc* desc,
                                                uint32_t& numLines)
{
    RuntimeExecutionScope execution(attachHandle);
    if (!execution)
        return nullptr;
    usdparser::AttachedStage* owner = UsdLoad::getUsdLoad()->getAttachedStageByHandle(attachHandle);
    if (!owner)
        return nullptr;
    return getShapeDebugDraw(*owner, key, desc, numLines);
}

void omni::physx::clearDebugVisualizationData(AttachHandle attachHandle)
{
    RuntimeExecutionScope execution(attachHandle);
    if (!execution)
        return;
    usdparser::AttachedStage* owner = UsdLoad::getUsdLoad()->getAttachedStageByHandle(attachHandle);
    if (!owner)
        return;
    clearDebugVisualizationData(*owner);
}

uint32_t omni::physx::getDebugDrawCollShapeColor(AttachHandle attachHandle, omni::physics::parse::ObjectKey key)
{
    RuntimeExecutionScope execution(attachHandle);
    if (!execution)
        return 0;
    usdparser::AttachedStage* owner = UsdLoad::getUsdLoad()->getAttachedStageByHandle(attachHandle);
    if (!owner)
        return 0;
    return getDebugDrawCollShapeColor(*owner, key);
}

const omni::physx::CollisionRepresentation* omni::physx::getCollisionRepresentation(AttachHandle attachHandle, omni::physics::parse::ObjectKey key,const usdparser::PhysxShapeDesc* desc)
{
    RuntimeExecutionScope execution(attachHandle);
    if (!execution)
        return nullptr;
    usdparser::AttachedStage* owner = UsdLoad::getUsdLoad()->getAttachedStageByHandle(attachHandle);
    if (!owner)
        return nullptr;
    return getCollisionRepresentation(*owner, key, desc);
}
