// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "PhysXDebugVisualization.h"

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

#include <omni/physics/ovstage/OvstageOutput.h> // buildPathList (checks for the source dictionary)

namespace
{
// Per-parameter visualization values (a magnitude multiplied by eSCALE, e.g. contact-
// normal length or axis size). Lets hosts scale gizmo groups independently while the
// bool setter keeps its on/off contract. Enabled params apply their cached value, else 0.
struct VizParamValues
{
    float v[omni::physx::ePhysXNumValues];
    VizParamValues()
    {
        for (float& x : v)
            x = 1.0f;
    }
};
VizParamValues g_vizParamValues;
} // namespace


using namespace PXR_NS;
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

static DebugVisualizationCache gDebugVisualizationCache;

// DebugVis Shape colors
static constexpr uint32_t gStaticShapeColor = PxU32(PxDebugColor::eARGB_MAGENTA);
static constexpr uint32_t gDynamicShapeColor = PxU32(PxDebugColor::eARGB_GREEN);
static constexpr uint32_t gKinematicShapeColor = PxU32(PxDebugColor::eARGB_YELLOW);
static constexpr uint32_t gDynamicTriMeshShapeColor = PxU32(PxDebugColor::eARGB_DARKRED);

uint32_t omni::physx::getDebugDrawCollShapeColor(const PXR_NS::SdfPath& primKey)
{
    uint32_t collColor = gStaticShapeColor;

    // All schema/attribute/structural reads (schema gates, approximation read, ancestor
    // walk) route through IPhysicsSource by ObjectKey, so no UsdPrim is materialized here.
    // Without an attached stage there is no source to consult.
    const usdparser::AttachedStage* attachedStage = usdparser::UsdLoad::getUsdLoad()->getActiveAttachedStage();
    const omni::physics::parse::IPhysicsSource* src = attachedStage ? attachedStage->getSource() : nullptr;
    if (!src)
        return collColor;

    const omni::physics::parse::ObjectKey key = attachedStage->keyFor(primKey);

    // Source-routed attribute read; getValue resolves the schema fallback, so
    // e.g. physicsRigidBodyEnabled correctly defaults to true when unauthored.
    auto readBool = [&](omni::physics::parse::ObjectKey k, const TfToken& attr)
    {
        bool v = false;
        internal::getValue(*attachedStage, k, attr, UsdTimeCode::Default(), v);
        return v;
    };

    bool isTrimesh = false;
    if (internal::isAType<UsdGeomMesh>(*src, key))
    {
        if (internal::hasAppliedSchema<UsdPhysicsMeshCollisionAPI>(*src, key))
        {
            TfToken approx;
            internal::getValue(*attachedStage, key, UsdPhysicsTokens->physicsApproximation, UsdTimeCode::Default(), approx);
            if (approx == UsdPhysicsTokens.Get()->none)
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
        if (internal::hasAppliedSchema<UsdPhysicsRigidBodyAPI>(*src, parent))
        {
            if (readBool(parent, UsdPhysicsTokens->physicsRigidBodyEnabled))
            {
                collColor = isTrimesh ? gDynamicTriMeshShapeColor : gDynamicShapeColor;

                if (readBool(parent, UsdPhysicsTokens->physicsKinematicEnabled))
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
struct VizScope
{
    std::unordered_set<ovx_primpath_t> paths;

    bool contains(ovx_primpath_t path) const
    {
        if (paths.empty())
            return true;
        return path != OVX_INVALID_PRIMPATH && paths.find(path) != paths.end();
    }
};
static VizScope g_vizScope;

// Returns the active source only when it is OVStage-backed. A non-OVStage
// ObjectKey is not comparable to dictionary-specific ovx_primpath_t handles.
static const omni::physics::parse::IPhysicsSource* vizOvstageSource()
{
    const usdparser::AttachedStage* stage =
        usdparser::UsdLoad::getUsdLoad()->getActiveAttachedStage();
    const omni::physics::parse::IPhysicsSource* src = stage ? stage->getSource() : nullptr;
    if (!src)
        return nullptr;
    ovx_path_dictionary_t* dict = nullptr;
    omni::physics::ovstage::buildPathList(*src, nullptr, 0, &dict);
    return dict ? src : nullptr;
}

static omni::physics::parse::ObjectKey vizActorKey(const PxActor* actor)
{
    if (!actor)
        return {};
    const std::vector<omni::physx::internal::InternalDatabase::Record>& records =
        OmniPhysX::getInstance().getInternalPhysXDatabase().getRecords();
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

static omni::physics::parse::ObjectKey vizShapeKey(const PxShape* shape)
{
    if (!shape)
        return {};
    const std::vector<omni::physx::internal::InternalDatabase::Record>& records =
        OmniPhysX::getInstance().getInternalPhysXDatabase().getRecords();
    const size_t recordIndex = reinterpret_cast<size_t>(shape->userData);
    if (recordIndex >= records.size())
        return {};
    const omni::physx::internal::InternalDatabase::Record& record = records[recordIndex];
    if (record.mType != ePTShape && record.mType != ePTCompoundShape)
        return {};
    return record.mKey;
}

static bool vizScopeContains(const omni::physics::parse::IPhysicsSource* source,
                             omni::physics::parse::ObjectKey primKey)
{
    if (g_vizScope.paths.empty())
        return true;
    if (!source || !primKey.valid())
        return false;
    const omni::physics::parse::ObjectKey canonical = source->canonicalKey(primKey);
    return canonical.valid() && g_vizScope.contains(static_cast<ovx_primpath_t>(canonical.handle));
}

static bool vizActorInScope(const omni::physics::parse::IPhysicsSource* source,
                            const ::physx::PxActor* actor)
{
    return vizScopeContains(source, vizActorKey(actor));
}

static bool vizShapeInScope(const omni::physics::parse::IPhysicsSource* source,
                            const ::physx::PxShape* shape)
{
    return vizScopeContains(source, vizShapeKey(shape));
}

static void enableActorVisualization(PxActor* actor,
                                     bool enableVis,
                                     const omni::physics::parse::IPhysicsSource* source)
{
    const bool actorInScope = enableVis && vizActorInScope(source, actor);
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
                const bool enabled = enableVis && vizShapeInScope(source, shape);
                shapeInScope = shapeInScope || enabled;
                if (shape->isExclusive())
                    shape->setFlag(PxShapeFlag::eVISUALIZATION, enabled);
            }
        }
    }
    actor->setActorFlag(PxActorFlag::eVISUALIZATION, actorInScope || shapeInScope);
}

void omni::physx::enableVisualization(bool enableVis)
{
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const PhysXSetup& physxSetup = omniPhysX.getPhysXSetup();
    omniPhysX.setDebugVisualizationEnabled(enableVis);
    waitForSimulationCompletion(false);
    const omni::physics::parse::IPhysicsSource* scopeSource =
        g_vizScope.paths.empty() ? nullptr : vizOvstageSource();

    for (PhysXScenesMap::const_reference ref : physxSetup.getPhysXScenes())
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
            enableActorVisualization(actor, enableVis, scopeSource);
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
                enableActorVisualization(link, enableVis, scopeSource);
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
            bool jointInScope = !enableVis || g_vizScope.paths.empty();
            if (!jointInScope)
            {
                PxRigidActor* a0 = nullptr;
                PxRigidActor* a1 = nullptr;
                constraint->getActors(a0, a1);
                jointInScope =
                    (a0 && vizActorInScope(scopeSource, a0)) ||
                    (a1 && vizActorInScope(scopeSource, a1));
            }
            constraint->setFlag(PxConstraintFlag::eVISUALIZATION,
                                enableVis && jointInScope);
        }

        if (enableVis)
        {
            const float gizmoScale = omniPhysX.getCachedSettings().viewportGizmoScale;
            scene->setVisualizationParameter(
                PxVisualizationParameter::eSCALE, gizmoScale * omniPhysX.getVisualizationScale());

            const uint32_t visMask = omniPhysX.getVisualizationBitMask();
            for (uint32_t i = 0; i < ePhysXNumValues; i++)
            {
                if (visMask & (1 << i))
                {
                    scene->setVisualizationParameter(PxVisualizationParameter::Enum(i), g_vizParamValues.v[i]);
                }
            }
        }
        else
        {
            scene->setVisualizationParameter(PxVisualizationParameter::eSCALE, 0.0f);
        }
    }
}

void omni::physx::enableNormalsVisualization(bool enableNormalsVis)
{
    OmniPhysX::getInstance().setNormalsVisualizationEnabled(enableNormalsVis);
}

void omni::physx::setVisualizationScale(float scale)
{
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const PhysXSetup& physxSetup = omniPhysX.getPhysXSetup();

    omniPhysX.setVisualizationScale(scale);
    const float gizmoScale = omniPhysX.getCachedSettings().viewportGizmoScale;

    if (omniPhysX.isDebugVisualizationEnabled())
    {
        waitForSimulationCompletion(false);

        for (PhysXScenesMap::const_reference ref : physxSetup.getPhysXScenes())
        {
            PxScene* scene = ref.second->getScene();
            if (scene)
                scene->setVisualizationParameter(PxVisualizationParameter::eSCALE, gizmoScale * scale);
        }
    }
}

void omni::physx::setVisualizationCullingBox(const carb::Float3& min, const carb::Float3& max)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const PhysXSetup& physxSetup = omniPhysX.getPhysXSetup();

    PxBounds3 bounds;
    bounds.minimum = (const PxVec3&)min;
    bounds.maximum = (const PxVec3&)max;

    if (bounds.isValid())
    {
        waitForSimulationCompletion(false);

        for (PhysXScenesMap::const_reference ref : physxSetup.getPhysXScenes())
        {
            PxScene* scene = ref.second->getScene();
            if (scene)
                scene->setVisualizationCullingBox(bounds);
        }
    }
}

void omni::physx::setVisualizationParameter(PhysXVisualizationParameter par, bool val)
{
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const PhysXSetup& physxSetup = omniPhysX.getPhysXSetup();

    uint64_t visMask = omniPhysX.getVisualizationBitMask();

    if (val)
        visMask |= (1ull << (uint64_t)par);
    else
        visMask &= ~(1ull << (uint64_t)par);

    omniPhysX.setVisualizationBitMask(visMask);

    waitForSimulationCompletion(false);

    if (par != eNone && par < ePhysXNumValues)
    {
        for (PhysXScenesMap::const_reference ref : physxSetup.getPhysXScenes())
        {
            PxScene* scene = ref.second->getScene();
            if (scene)
                scene->setVisualizationParameter(PxVisualizationParameter::Enum(par), val ? g_vizParamValues.v[par] : 0.0f);
        }
    }
    else
    {
        omniPhysX.getInternalPhysXDatabase().setVisualizationParameter(par, val);
    }
}

void omni::physx::setVisualizationParameterValue(PhysXVisualizationParameter par, float value)
{
    // Records the magnitude this parameter applies when enabled and re-applies it live
    // if currently on. Does not change the enabled state. Values must be finite and >= 0.
    if (par == eNone || par >= ePhysXNumValues)
        return;
    if (!(value >= 0.0f) || !std::isfinite(value))
        return;

    g_vizParamValues.v[par] = value;

    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const PhysXSetup& physxSetup = omniPhysX.getPhysXSetup();
    const uint64_t visMask = omniPhysX.getVisualizationBitMask();
    if ((visMask & (1ull << (uint64_t)par)) == 0)
        return; // parameter is off: the value applies on the next enable

    waitForSimulationCompletion(false);

    for (PhysXScenesMap::const_reference ref : physxSetup.getPhysXScenes())
    {
        PxScene* scene = ref.second->getScene();
        if (scene)
            scene->setVisualizationParameter(PxVisualizationParameter::Enum(par), value);
    }
}

bool omni::physx::setVisualizationScopeTokens(const ovx_primpath_t* tokens, uint32_t count)
{
    // Restrict eVISUALIZATION flag authoring to a set of interned prim paths, exact
    // membership (empty = every object). The caller expanded any hierarchy into its
    // object set and interned it against this instance's source dictionary; matching an
    // object canonicalizes its source key through that same dictionary.
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    if (count > 0u && !tokens)
        return false;
    std::unordered_set<ovx_primpath_t> pending;
    for (uint32_t i = 0; i < count; ++i)
    {
        if (tokens[i] == OVX_INVALID_PRIMPATH)
            return false;
        pending.insert(tokens[i]);
    }
    if (count > 0u && vizOvstageSource() == nullptr)
        return false;

    g_vizScope.paths.swap(pending);
    // Re-author the flags live so a scope change applies without a disable/enable
    // cycle, drawn on the next step. enableVisualization waits for completion itself.
    if (omniPhysX.isDebugVisualizationEnabled())
        omni::physx::enableVisualization(true);
    return true;
}

uint32_t omni::physx::getNbPoints()
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const PhysXSetup& physxSetup = omniPhysX.getPhysXSetup();
    uint32_t numPoints = 0;

    waitForSimulationCompletion(false);

    for (PhysXScenesMap::const_reference ref : physxSetup.getPhysXScenes())
    {
        PxScene* scene = ref.second->getScene();
        if (scene)
            numPoints += scene->getRenderBuffer().getNbPoints();
    }    
    return numPoints;
}

const DebugPoint* omni::physx::getPoints()
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const PhysXSetup& physxSetup = omniPhysX.getPhysXSetup();

    if (physxSetup.getPhysXScenes().empty())
        return nullptr;

    waitForSimulationCompletion(false);

    if (physxSetup.getPhysXScenes().size() == 1)
        return (const DebugPoint*)physxSetup.getPhysXScenes().begin()->second->getScene()->getRenderBuffer().getPoints();
    else
    {
        gDebugVisualizationCache.mPointsBuffer.resize(getNbPoints());
        uint32_t offset = 0;
        for (PhysXScenesMap::const_reference ref : physxSetup.getPhysXScenes())
        {
            PxScene* scene = ref.second->getScene();
            const uint32_t numPoints = scene->getRenderBuffer().getNbPoints();
            memcpy(gDebugVisualizationCache.mPointsBuffer.data() + offset, scene->getRenderBuffer().getPoints(), sizeof(DebugPoint)*numPoints);
            offset += numPoints;
        }
    }

    return gDebugVisualizationCache.mPointsBuffer.data();
}

uint32_t omni::physx::getNbLines()
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const PhysXSetup& physxSetup = omniPhysX.getPhysXSetup();
    uint32_t numLines = 0;

    waitForSimulationCompletion(false);

    for (PhysXScenesMap::const_reference ref : physxSetup.getPhysXScenes())
    {
        PxScene* scene = ref.second->getScene();
        if (scene)
            numLines += scene->getRenderBuffer().getNbLines();
    }
    numLines += omniPhysX.getInternalPhysXDatabase().getDebugRenderBuffer().getNbLines();
    return numLines;
}

const DebugLine* omni::physx::getLines()
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const PhysXSetup& physxSetup = omniPhysX.getPhysXSetup();

    if (physxSetup.getPhysXScenes().empty())
        return nullptr;

    waitForSimulationCompletion(false);

    if (physxSetup.getPhysXScenes().size() == 1 &&
        omniPhysX.getInternalPhysXDatabase().getDebugRenderBuffer().getNbLines() == 0)
        return (const DebugLine*)physxSetup.getPhysXScenes().begin()->second->getScene()->getRenderBuffer().getLines();
    else
    {
        gDebugVisualizationCache.mLinesBuffer.resize(getNbLines());
        uint32_t offset = 0;
        for (PhysXScenesMap::const_reference ref : physxSetup.getPhysXScenes())
        {
            PxScene* scene = ref.second->getScene();
            const uint32_t numLines = scene->getRenderBuffer().getNbLines();
            memcpy(gDebugVisualizationCache.mLinesBuffer.data() + offset, scene->getRenderBuffer().getLines(), sizeof(DebugLine) * numLines);
            offset += numLines;
        }

        if (omniPhysX.getInternalPhysXDatabase().getDebugRenderBuffer().getNbLines() > 0)
        {
            const uint32_t numLines = omniPhysX.getInternalPhysXDatabase().getDebugRenderBuffer().getNbLines();
            memcpy(gDebugVisualizationCache.mLinesBuffer.data() + offset,
                   omniPhysX.getInternalPhysXDatabase().getDebugRenderBuffer().getLines(),
                   sizeof(PxDebugLine) * numLines);
        }
    }

    return gDebugVisualizationCache.mLinesBuffer.data();
}

uint32_t omni::physx::getNbTriangles()
{    
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const PhysXSetup& physxSetup = omniPhysX.getPhysXSetup();
    uint32_t numTris = 0;

    waitForSimulationCompletion(false);

    for (PhysXScenesMap::const_reference ref : physxSetup.getPhysXScenes())
    {
        PxScene* scene = ref.second->getScene();
        if (scene)
            numTris += scene->getRenderBuffer().getNbTriangles();
    }
    return numTris;
}

const DebugTriangle* omni::physx::getTriangles()
{    
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const PhysXSetup& physxSetup = omniPhysX.getPhysXSetup();

    if (physxSetup.getPhysXScenes().empty())
        return nullptr;

    waitForSimulationCompletion(false);

    if (physxSetup.getPhysXScenes().size() == 1)
        return (const DebugTriangle*)physxSetup.getPhysXScenes().begin()->second->getScene()->getRenderBuffer().getTriangles();
    else
    {
        gDebugVisualizationCache.mTriangleBuffer.resize(getNbTriangles());
        uint32_t offset = 0;
        for (PhysXScenesMap::const_reference ref : physxSetup.getPhysXScenes())
        {
            PxScene* scene = ref.second->getScene();
            const uint32_t numTris = scene->getRenderBuffer().getNbTriangles();
            memcpy(gDebugVisualizationCache.mTriangleBuffer.data() + offset, scene->getRenderBuffer().getTriangles(), sizeof(DebugTriangle) * numTris);
            offset += numTris;
        }
    }

    return gDebugVisualizationCache.mTriangleBuffer.data();
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

void drawTriMesh(debugrender::RenderOutput& out, const PxTriangleMesh* triMesh, const PxVec3& scale)
{
    if ( !triMesh )
        return;

    UsdStageWeakPtr stage = usdparser::UsdLoad::getUsdLoad()->getActiveStage();
    if (!stage)
    {
        return;
    }

    const bool renderNormals = OmniPhysX::getInstance().isNormalsVisualizationEnabled();
    const float maxSize = 0.05f / usdparser::UsdLoad::getUsdLoad()->getActiveAttachedStage()->getSourceUnits().metersPerUnit;
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

static debugrender::RenderBuffer gRenderBuffer(64);


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
}

void DebugVisualizationCache::releasePath(const PXR_NS::SdfPath& path)
{
    {
        LineMap::iterator it = mLineMap.find(path);
        if (it != mLineMap.end())
        {
            delete[] it->second.mDebugLines;
            mLineMap.erase(it);
        }
    }

    {
        EdgeMap::iterator it = mEdgeMap.find(path);
        if (it != mEdgeMap.end())
        {
            delete[] it->second.mDebugEdges;
            mEdgeMap.erase(it);
        }
    }
}

const DebugLine* DebugVisualizationCache::getLines(const PXR_NS::SdfPath& path,
                                                   const PxTransform& transform,
                                                   uint32_t& numLines)
{
    LineMap::iterator it = mLineMap.find(path);
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

void DebugVisualizationCache::addLines(const PXR_NS::SdfPath& path,
                                       const PxTransform& transform,
                                       DebugLine* debugLines,
                                       uint32_t numLines)
{
    // TODO FIXME: release in case key exists (not an issue right now).
    mLineMap[path] = { debugLines, transform, numLines };
}

const DebugEdge* DebugVisualizationCache::getEdges(const PXR_NS::SdfPath& path, uint32_t& numEdges)
{
    EdgeMap::iterator it = mEdgeMap.find(path);
    if (it != mEdgeMap.end())
    {
        CachedEdges& edges = it->second;
        numEdges = edges.mNumEdges;
        return edges.mDebugEdges;
    }
    return nullptr;
}

void DebugVisualizationCache::addEdges(const PXR_NS::SdfPath& path, DebugEdge* debugEdges, uint32_t numEdges)
{
    mEdgeMap[path] = { debugEdges, numEdges };
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

const DebugLine* omni::physx::getShapeDebugDraw(const PXR_NS::SdfPath& primKey,
                                                const usdparser::PhysxShapeDesc* desc,
                                                uint32_t& numLines)
{
    gRenderBuffer.clear();
    const DebugLine* outLines = nullptr;
    numLines = 0;

    CookingDataAsync* cookingDataAsync = OmniPhysX::getInstance().getPhysXSetup().getCookingDataAsync();
    if (!cookingDataAsync || !desc)
    {
        return outLines;
    }

    UsdStageWeakPtr stage = usdparser::UsdLoad::getUsdLoad()->getActiveStage();
    if (!stage)
    {
        return outLines;
    }

    usdparser::AttachedStage* attachedStage = usdparser::UsdLoad::getUsdLoad()->getActiveAttachedStage();
    if (!attachedStage)
    {
        return outLines;
    }

    uint32_t collShapeColor = getDebugDrawCollShapeColor(primKey);
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
            const SpherePointsPhysxShapeDesc *sp = cookingDataAsync->getSpherePoints(*sphereDesc, attachedStage->keyFor(primKey), *attachedStage, true);
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

        if (OmniPhysX::getInstance().getISettings()->getAsBool(kSettingCollisionApproximateCylinders))
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

        if (OmniPhysX::getInstance().getISettings()->getAsBool(kSettingCollisionApproximateCones))
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

        PxConvexMesh* convexMesh = cookingDataAsync->getConvexMesh(*convexDesc, attachedStage->keyFor(primKey), *attachedStage, true);
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

        std::vector<PxConvexMesh*> convexMeshes = cookingDataAsync->getConvexMeshDecomposition(*convexDecompositionDesc, attachedStage->keyFor(primKey), *attachedStage, true);
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
        outLines = gDebugVisualizationCache.getLines(primKey, transform, numLines);
        if (!outLines)
        {
            TriangleMeshPhysxShapeDesc* meshDesc = (TriangleMeshPhysxShapeDesc*)desc;

            debugrender::RenderOutput out(gRenderBuffer);
            out << collShapeColor;

            out << transform;
            out << debugrender::RenderOutput::LINES;

            PxTriangleMesh* triMesh = cookingDataAsync->getTriangleMesh(*meshDesc, attachedStage->keyFor(primKey), *attachedStage, true);
            if ( triMesh )
            {
                drawTriMesh(out, triMesh, toPhysX(meshDesc->meshScale));
                if (!gRenderBuffer.empty())
                {
                    const size_t nLines = gRenderBuffer.getNbLines();
                    DebugLine* cachedLines = new DebugLine[nLines];
                    memcpy(cachedLines, gRenderBuffer.getLines(), sizeof(DebugLine) * nLines);
                    gDebugVisualizationCache.addLines(primKey, transform, cachedLines, (uint32_t)nLines);
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
        const float normalLength = 0.5f / usdparser::UsdLoad::getUsdLoad()->getActiveAttachedStage()->getSourceUnits().metersPerUnit;
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
        outLines = (const DebugLine*)(gRenderBuffer.getLines());
    }

    return outLines;
}

void omni::physx::clearDebugVisualizationData()
{
    gRenderBuffer.clear();
    gDebugVisualizationCache.release();
}

const omni::physx::CollisionRepresentation* omni::physx::getCollisionRepresentation(const PXR_NS::SdfPath& usdPath,const usdparser::PhysxShapeDesc* desc)
{
    const CollisionRepresentation *ret = nullptr;

    CookingDataAsync* cookingDataAsync = OmniPhysX::getInstance().getPhysXSetup().getCookingDataAsync();
    if (cookingDataAsync && desc)
    {
        ret = cookingDataAsync->getCollisionRepresentation(usdPath,*desc);
    }

    return ret;
}
void omni::physx::releaseCollisionRepresentation(const CollisionRepresentation *cr)
{
    CookingDataAsync* cookingDataAsync = OmniPhysX::getInstance().getPhysXSetup().getCookingDataAsync();
    if (cookingDataAsync)
    {
        cookingDataAsync->releaseCollisionRepresentation(cr);
    }
}

bool omni::physx::getMeshKey(const omni::physx::usdparser::PhysxShapeDesc &desc,omni::physx::usdparser::MeshKey &meshKey)
{
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
