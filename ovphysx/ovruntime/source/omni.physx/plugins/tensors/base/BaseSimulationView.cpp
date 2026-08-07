// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

/**
 * @implements REQ-REPLICATE-001
 * @covers AC-7
 */

// clang-format off
#include <UsdPCH.h>
#include <pxr/base/tf/patternMatcher.h>
#include <pxr/base/tf/stringUtils.h>
// clang-format on

#include "tensors/base/BaseSimulationView.h"
#include "tensors/base/PathPatternMatcher.h"
#include "tensors/base/BaseArticulationView.h"
#include "tensors/base/BaseRigidBodyView.h"
#include "tensors/base/BaseSdfShapeView.h"
#include "tensors/base/BaseVolumeDeformableBodyView.h"
#include "tensors/base/BaseSurfaceDeformableBodyView.h"
#include "tensors/base/BaseDeformableMaterialView.h"
#include "tensors/base/BaseRigidContactView.h"

#include "tensors/GlobalsAreBad.h"
#include "tensors/SimulationBackend.h"

#include "usdLoad/LoadUsd.h"
#include "usdLoad/AttachedStage.h"
#include "usdLoad/LoadTools.h"

#include "OmniPhysX.h"
#include <omni/physx/PhysXRuntime.h>
#include "internal/InternalScene.h"

#include <algorithm>
#include <functional>
#include <limits>
#include <set>
#include <unordered_set>

#include <carb/logging/Log.h>
#include <carb/settings/ISettings.h>
#include <private/omni/physx/IPhysxPrivate.h>

#include <PxPhysicsAPI.h>
#include <omniUsdPhysicsDeformableSchema/tokens.h>

#include <omni/physx/IPhysxSimulation.h>
#include <omni/physx/IPhysxJoint.h>
#include <omni/physx/IPhysxReplicator.h>
#include <omni/physx/PhysxTokens.h>


using namespace PXR_NS;
using namespace physx;
using namespace carb;
using omni::physics::tensors::ObjectType;


namespace omni
{
namespace physx
{
namespace tensors
{

namespace
{

// ------------------- recursive-leaf matching (legacy-compat) ----------------
//
// When enabled (default), the pre-leaf tokens are matched strictly (level by
// level) and only the final named or glob leaf token is searched at any depth
// beneath the matched parent, with same-name suppression. For example
// `/envs/*/Robot/link_0` finds a `link_0` nested anywhere under `Robot`, while
// `/envs/*/Robot/base_link` matches only the `base_link` under the strictly
// matched `Robot` and does not fan out to a buried `Group/Robot/base_link`.
// A same name nested inside its own match (`base_link/tool/base_link`) is
// suppressed. Use an explicit `**` to reach a buried pivot or every descendant.
//
// A bare `*` leaf is deliberately NOT promoted here -- silently turning
// `A/*` into "all descendants of A" changes the meaning of every
// pre-existing pattern that relied on strict per-level matching and would
// mask legitimate regressions. Callers who want to collect an entire
// subtree must spell it with `**`.
//
// Users who need strict semantics even for these leaves can disable it via
// kSettingRecursiveLeafPatternMatch; the explicit `**` token always works
// regardless.
//
// Default is kDefaultRecursiveLeafPatternMatch (see GlobalsAreBad.h); the
// same constant is passed to setDefaultBool in tensorsInit(). We guard
// the read with isAccessibleAs(eBool, ...) because getAsBool() returns
// false for a missing key, which would silently flip the documented
// default if this helper runs before tensorsInit() registers it.
bool isRecursiveLeafPatternMatchEnabled()
{
    auto* settings = carb::getCachedInterface<carb::settings::ISettings>();
    if (!settings)
    {
        return kDefaultRecursiveLeafPatternMatch;
    }
    if (!settings->isAccessibleAs(carb::dictionary::ItemType::eBool, kSettingRecursiveLeafPatternMatch))
    {
        return kDefaultRecursiveLeafPatternMatch;
    }
    return settings->getAsBool(kSettingRecursiveLeafPatternMatch);
}

// Wildcard matcher backed by the Omni PhysX internal path<->object DB (PrimHierarchyStorage).
// It reuses PathPatternMatcher's pure tokenization + recursive-leaf helpers.
// Matched rows are re-sorted by physics creation id (min ObjectId) to restore the numeric
// clone order (env0..env12): PrimHierarchyStorage children live in a lexicographic std::set,
// and on a replicated stage the numeric order lives in ObjectId (clone index), not the USD-side names.
//
// Only physics-registered paths (and their ancestors) are stored, so matches are
// narrowed to physics objects. Every view type getPhysXPtr-filters its matches
// downstream, so this is invisible -- except contact-filter patterns, which must
// therefore name a physics object (collider/body), not an arbitrary USD prim.
void findMatchingPathsInternalDb(const PXR_NS::UsdStageWeakPtr& stage,
                                 const std::string& pattern_,
                                 bool recursiveLeafPatternMatch,
                                 std::vector<SdfPath>& pathsRet)
{
    if (!stage)
    {
        return;
    }
    const long stageId = UsdUtilsStageCache::Get().GetId(stage).ToLongInt();
    usdparser::AttachedStage* attachedStage = usdparser::UsdLoad::getUsdLoad()->getAttachedStage(stageId);
    if (!attachedStage)
    {
        return;
    }
    const usdparser::ObjectDb* objectDb = attachedStage->getObjectDatabase();
    if (!objectDb)
    {
        return;
    }
    const PrimHierarchyStorage::StorageMap& storage = objectDb->getPrimHierarchyStorage().getStorageMap();
    if (storage.empty())
    {
        return;
    }

    // Fast path: a literal prim path resolves via an O(1) storage lookup and skips traversal.
    if (PXR_NS::SdfPath::IsValidPathString(pattern_))
    {
        const SdfPath p(pattern_);
        if (storage.find(p) != storage.end())
        {
            pathsRet.push_back(p);
            return;
        }
    }

    // Physics-object creation id at a path (min ObjectId); paths with no physics object
    // sort last. Used to restore numeric clone order on the replicated stage.
    auto creationKey = [&](const SdfPath& p) -> usdparser::ObjectId
    {
        const usdparser::ObjectIdMap* entries = objectDb->getEntries(p);
        if (!entries || entries->empty())
        {
            return std::numeric_limits<usdparser::ObjectId>::max();
        }
        usdparser::ObjectId minId = entries->begin()->second;
        for (const auto& kv : *entries)
        {
            minId = std::min(minId, kv.second);
        }
        return minId;
    };

    // children-of helper: the pseudo-root (empty path) maps to the stored top-level
    // prims (entries with an empty parent). Storage children come from a lexicographic
    // std::set; the ObjectId sort at the tail restores numeric clone order, so the
    // emission order here is not significant.
    auto childrenOf = [&](const SdfPath& p, std::vector<SdfPath>& out)
    {
        if (p.IsEmpty())
        {
            for (const auto& kv : storage)
            {
                if (kv.second.parent.IsEmpty())
                {
                    out.push_back(kv.first);
                }
            }
            return;
        }
        const PrimHierarchyStorage::StorageMap::const_iterator it = storage.find(p);
        if (it != storage.end())
        {
            out.insert(out.end(), it->second.children.begin(), it->second.children.end());
        }
    };

    // Storage analogues of PathPatternMatcher's USD traversal helpers, structurally
    // identical so the '**' / recursive-leaf semantics match the USD branch exactly.
    std::function<void(const SdfPath&, std::vector<SdfPath>&)> collectSelfAndDescendants =
        [&](const SdfPath& p, std::vector<SdfPath>& out)
    {
        if (!p.IsEmpty())
        {
            out.push_back(p);
        }
        std::vector<SdfPath> ch;
        childrenOf(p, ch);
        for (const SdfPath& c : ch)
        {
            collectSelfAndDescendants(c, out);
        }
    };
    // Leaf-recursive descent with same-name-on-path suppression -- the SdfPath
    // analogue of PathPatternMatcher::collectMatchingDescendants, so the USD and
    // internal-DB branches match at any depth identically.
    std::function<void(const SdfPath&, TfPatternMatcher&, std::unordered_set<std::string>&, std::vector<SdfPath>&)>
        collectMatchingDescendants =
            [&](const SdfPath& p, TfPatternMatcher& matcher, std::unordered_set<std::string>& onPath,
                std::vector<SdfPath>& out)
    {
        std::vector<SdfPath> ch;
        childrenOf(p, ch);
        for (const SdfPath& c : ch)
        {
            const std::string name = c.GetName();
            if (matcher.Match(name) && onPath.insert(name).second)
            {
                out.push_back(c);
                collectMatchingDescendants(c, matcher, onPath, out);
                onPath.erase(name);
            }
            else
            {
                collectMatchingDescendants(c, matcher, onPath, out);
            }
        }
    };

    const std::string pattern = TfStringTrim(pattern_, "/");
    const std::vector<std::string> tokens = splitPatternRespectingGroups(pattern);
    if (tokens.empty())
    {
        return;
    }

    std::vector<SdfPath> roots;
    std::vector<SdfPath> matches;
    roots.push_back(SdfPath()); // pseudo-root (empty path)

    const int numTokens = int(tokens.size());

    // See PathPatternMatcher: an explicit '**' already expands roots, so the leaf
    // falls back to strict matching to avoid re-scanning overlapping roots.
    bool patternHasRecursiveDescent = false;
    for (const std::string& t : tokens)
    {
        if (t == kRecursiveDescentToken)
        {
            patternHasRecursiveDescent = true;
            break;
        }
    }

    for (int i = 0; i < numTokens; i++)
    {
        matches.clear();

        const bool isLeaf = (i == numTokens - 1);
        const bool isRecursiveDescent = (tokens[i] == kRecursiveDescentToken);
        const bool isBareWildcard = (tokens[i] == "*");

        if (isRecursiveDescent)
        {
            for (const SdfPath& r : roots)
            {
                collectSelfAndDescendants(r, matches);
            }
        }
        else if (isLeaf && recursiveLeafPatternMatch && !isBareWildcard && !patternHasRecursiveDescent)
        {
            // Leaf-recursive: named/glob leaf searched at any depth beneath the
            // strictly-matched ancestor chain (with same-name suppression).
            TfPatternMatcher matcher(makeAnchoredTokenPattern(tokens[i]), true, true);
            for (const SdfPath& r : roots)
            {
                std::unordered_set<std::string> onPath;
                collectMatchingDescendants(r, matcher, onPath, matches);
            }
        }
        else
        {
            TfPatternMatcher matcher(makeAnchoredTokenPattern(tokens[i]), true, true);
            for (const SdfPath& r : roots)
            {
                std::vector<SdfPath> ch;
                childrenOf(r, ch);
                for (const SdfPath& c : ch)
                {
                    if (matcher.Match(c.GetName()))
                    {
                        matches.push_back(c);
                    }
                }
            }
        }

        if (i < numTokens - 1)
        {
            std::swap(roots, matches);
        }
    }

    // Restore numeric clone order: storage children are lexicographic, and on a
    // replicated stage the numeric env order lives in ObjectId (clone index).
    std::stable_sort(matches.begin(), matches.end(),
                     [&](const SdfPath& a, const SdfPath& b)
                     {
                         const usdparser::ObjectId ka = creationKey(a);
                         const usdparser::ObjectId kb = creationKey(b);
                         if (ka != kb)
                         {
                             return ka < kb;
                         }
                         return a < b;
                     });
    for (const SdfPath& p : matches)
    {
        pathsRet.push_back(p);
    }
}

bool isEnabledCollision(const UsdPrim prim)
{
    UsdPhysicsCollisionAPI collisionAPI(prim);
    if (collisionAPI)
    {
        bool isEnabled;
        collisionAPI.GetCollisionEnabledAttr().Get(&isEnabled);
        return isEnabled;
    }
    return false;
}

bool findDeformableMeshPaths(SdfPath& simMeshPath,
                             SdfPath& collMeshPath,
                             SdfPathSet& skinGeomPaths,
                             const SdfPath actualDeformableBodyPath,
                             const UsdStageWeakPtr stage)
{
    simMeshPath = SdfPath();
    collMeshPath = SdfPath();
    skinGeomPaths.clear();

    TfType volumeSimType =
        UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniUsdPhysicsDeformableSchemaTokens->OmniPhysicsVolumeDeformableSimAPI);
    TfType surfaceSimType =
        UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniUsdPhysicsDeformableSchemaTokens->OmniPhysicsSurfaceDeformableSimAPI);
    if (!stage)
    {
        return false;
    }

    UsdPrim bodyPrim = stage->GetPrimAtPath(actualDeformableBodyPath);
    UsdPrimRange prims(bodyPrim, PXR_NS::UsdPrimAllPrimsPredicate);
    for (PXR_NS::UsdPrimRange::const_iterator it = prims.begin(); it != prims.end(); ++it)
    {
        UsdPrim prim = *it;
        if (!prim.IsA<UsdGeomPointBased>())
        {
            continue;
        }

        if (prim != bodyPrim)
        {
            bool resetsXformStack = false;
            UsdGeomXformable(prim).GetOrderedXformOps(&resetsXformStack);
            if (resetsXformStack)
            {
                it.PruneChildren();
                continue;
            }
        }

        bool isSim = prim.HasAPI(volumeSimType) || prim.HasAPI(surfaceSimType);
        bool isColl = isEnabledCollision(prim);

        if (isSim)
        {
            if (!simMeshPath.IsEmpty())
            {
                return false;
            }
            if (prim != bodyPrim && prim.GetParent() != bodyPrim)
            {
                return false;
            }
            simMeshPath = prim.GetPath();
        }
        if (isColl)
        {
            if (!collMeshPath.IsEmpty())
            {
                return false;
            }
            collMeshPath = prim.GetPath();
        }
        if (!isSim && !isColl)
        {
            skinGeomPaths.insert(prim.GetPath());
        }
    }

    return true;
}

} // namespace

bool BaseSimulationView::isNoMatchLoggingQuiet() const
{
    return mNoMatchLoggingQuiet;
}

void BaseSimulationView::setNoMatchLoggingQuiet(bool quiet)
{
    mNoMatchLoggingQuiet = quiet;
}

BaseSimulationView::BaseSimulationView(UsdStageRefPtr stage)
    : mStage(stage), mSimData(std::make_shared<BaseSimulationData>())
{
    omni::physx::IPhysicsObjectChangeCallback callback;
    callback.objectDestructionNotifyFn = onPhysXObjectDeletedCallback;
    callback.allObjectsDestructionNotifyFn = onAllPhysXObjectDeletedCallback;
    callback.userData = this;
    subscriptionObjId = g_physx->subscribeObjectChangeNotifications(callback);
}

void BaseSimulationView::invalidate()
{
    std::lock_guard<std::mutex> guard(mMutex);
    if (isValid)
    {
        isValid = false;
        // Can't reset the backend here, as it will cause a deadlock since the SimulationBackend reset calls SimulationView invalidate function
        // GetSimulationBackend().reset();
        // only Reset the stage here if using shared_ptr i.e. TfRefPtr<UsdStage>
        // mStage.Reset();
        // Alternative is to use TfweakPtr<UsdStage> to avoid increasing the reference count
    }
}


void BaseSimulationView::InitializeKinematicBodies()
{
    // Intentionally a no-op. This previously seeded kinematic rigid-body
    // world/local transforms for the renderer's first frame -- a rendering-only
    // effect (it touches no PhysX/simulation state), so it never affects tensor
    // output. Dropped here (OMPE-96492). If a render path needs first-frame
    // seeding, that integration should own it outside this tensor runtime.
    // ovphysx is headless today and exposes no entry point for this. Retained
    // because it is part of the internal ISimulationView contract.
}


BaseSimulationView::~BaseSimulationView()
{
    // Guard against shutdown-order issues: if PhysX plugin has already unloaded,
    // skip cleanup that would access dangling pointers (Python finalization race)
    if (g_physx)
    {
        g_physx->unsubscribeObjectChangeNotifications(subscriptionObjId);
    }
    invalidate();
    
    // Guard against backend being destroyed during plugin shutdown
    if (auto* backend = GetSimulationBackend())
    {
        backend->removeSimulationView(this);
    }

    for (auto artiView : mArtiViews)
    {
        artiView->_onParentRelease();
    }

    for (auto rbView : mRbViews)
    {
        rbView->_onParentRelease();
    }

    for (auto vdbView : mVolumeDeformableBodyViews)
    {
        vdbView->_onParentRelease();
    }

    for (auto sdbView : mSurfaceDeformableBodyViews)
    {
        sdbView->_onParentRelease();
    }
    
    for (auto dMaterialView : mDeformableMaterialViews)
    {
        dMaterialView->_onParentRelease();
    }

    for (auto rcView : mRcViews)
    {
        rcView->_onParentRelease();
    }

    for (auto sdfView : mSDFViews)
    {
        sdfView->_onParentRelease();
    }
}

void BaseSimulationView::onPhysXObjectDeletedCallback(const PXR_NS::SdfPath& sdfPath,
                                                      usdparser::ObjectId objectId,
                                                      PhysXType type,
                                                      void* userData)
{
    auto sim = (BaseSimulationView*)userData;
    omni::physx::IPhysx* physx = omni::physx::tensors::g_physx;
    if (physx && sim)
    {
        // no need to check further and print warning if the sim is already invalidated
        if (!sim->getValid())
            return;

        if (type == ePTActor)
        {
            PxRigidBody* actor = static_cast<PxRigidBody*>(physx->getPhysXPtrFast(objectId));
            if (sim->hasRigidBody(actor))
            {
                CARB_LOG_WARN(
                    "prim '%s' was deleted while being used by a tensor view class. The physics.tensors simulationView was invalidated.",
                    sdfPath.GetText());
                sim->invalidate();
            }
        }
        if (type == ePTArticulation)
        {
            auto arti = static_cast<PxArticulationReducedCoordinate*>(physx->getPhysXPtrFast(objectId));
            if (sim->hasArticulation(arti))
            {
                CARB_LOG_WARN(
                    "prim '%s' was deleted while being used by a tensor view class. The physics.tensors simulationView was invalidated.",
                    sdfPath.GetText());
                sim->invalidate();
            }
        }
        if (type == ePTLink)
        {
            auto link = static_cast<PxArticulationLink*>(physx->getPhysXPtrFast(objectId));
            if (sim->hasLink(link))
            {
                CARB_LOG_WARN(
                    "prim '%s' was deleted while being used by a link in a tensor view class. The physics.tensors simulationView was invalidated.",
                    sdfPath.GetText());
                sim->invalidate();
            }
        }
        if (type == ePTShape)
        {
            auto shape = static_cast<PxShape*>(physx->getPhysXPtrFast(objectId));
            if (sim->hasShape(shape))
            {
                CARB_LOG_WARN(
                    "prim '%s' was deleted while being used by a shape in a tensor view class. The physics.tensors simulationView was invalidated.",
                    sdfPath.GetText());
                sim->invalidate();
            }
        }
        if (type == ePTDeformableVolume || type == ePTDeformableSurface)
        {
            auto body = static_cast<PxDeformableBody*>(physx->getPhysXPtrFast(objectId));
            if (sim->hasDeformableBody(body))
            {
                CARB_LOG_WARN(
                    "prim '%s' was deleted while being used by a deformable body in a tensor view class. The physics.tensors simulationView was invalidated.",
                    sdfPath.GetText());
                sim->invalidate();
            }
        }
    }
}

void BaseSimulationView::onAllPhysXObjectDeletedCallback(void* userData)
{
    auto sim = (BaseSimulationView*)userData;
    if (sim)
    {
        // no need to check further and print warning if the sim is already invalidated
        if (!sim->getValid())
            return;

        CARB_LOG_WARN(
            "All physics information was deleted while being used by a tensor view class. The physics.tensors simulationView was invalidated.");
        sim->invalidate();
    }
}

bool BaseSimulationView::setSubspaceRoots(const char* pattern)
{
    if (!pattern || !*pattern)
    {
        CARB_LOG_ERROR("Empty pattern not allowed");
        return false;
    }

    // we only support absolute paths atm
    if (pattern[0] != '/')
    {
        CARB_LOG_ERROR("Pattern must be an absolute USD path, got '%s'\n", pattern);
        return false;
    }

    std::vector<SdfPath> paths;
    findMatchingPaths(pattern, paths);

    for (unsigned i = 0; i < paths.size(); i++)
    {
        UsdPrim prim = mStage->GetPrimAtPath(paths[i]);
        UsdGeomXformable xf(prim);
        if (xf)
        {
            GfMatrix4d localToWorld = xf.ComputeLocalToWorldTransform(UsdTimeCode::Default());
            GfVec3d tran = GfTransform(localToWorld).GetTranslation();

            Subspace subspace;
            subspace.origin = { float(tran[0]), float(tran[1]), float(tran[2]) };

            mSimData->mSubspaces[paths[i]] = subspace;
        }
    }

    return true;
}

Subspace* BaseSimulationView::findSubspaceForPath(const PXR_NS::SdfPath& path) const
{
    for (auto& entry : mSimData->mSubspaces)
    {
        if (path.HasPrefix(entry.first))
        {
            return &entry.second;
        }
    }
    return nullptr;
}

void BaseSimulationView::findMatchingPaths(const std::string& pattern_, std::vector<SdfPath>& pathsRet)
{
    if (!mStage)
    {
        return;
    }

    // Resolves the pattern against the Omni PhysX internal path<->object DB (PrimHierarchyStorage)
	// and restores numeric clone order via ObjectId.
	// The USD-authored branch keeps the plain-USD matcher (authoring order).
    findMatchingUsdPaths(mStage, pattern_, isRecursiveLeafPatternMatchEnabled(), pathsRet);

    // PhysX-replicator clones are physics-only: registered in the internal path<->object DB but with
    // no authored USD prim, so findMatchingUsdPaths (which matches against stage prims) cannot see
    // them. Supplement with the internal-DB matcher, appending ONLY the matches the USD pass missed,
    // in physics-creation (numeric clone) order. On a non-replicated stage every internal match
    // already has a USD prim and is therefore already present, so nothing is appended and the
    // authored-prim ordering above is preserved unchanged.
    std::vector<SdfPath> internalMatches;
    findMatchingPathsInternalDb(mStage, pattern_, isRecursiveLeafPatternMatchEnabled(), internalMatches);
    if (!internalMatches.empty())
    {
        std::unordered_set<SdfPath, SdfPath::Hash> seen(pathsRet.begin(), pathsRet.end());
        for (const SdfPath& p : internalMatches)
        {
            if (seen.insert(p).second)
            {
                pathsRet.push_back(p);
            }
        }
    }
}


void BaseSimulationView::processArticulationEntries(const std::vector<std::string>& patterns,
                                                    std::vector<ArticulationEntry>& entries)
{
    // Dedup across the full pattern list: overlapping patterns like
    // "/envs/*/Robot" and "/envs/*/Robot/**/base_link" can resolve to the
    // same articulation, and findMatchingArticulations shares this set so
    // the second match is dropped rather than appended again.
    std::unordered_set<const ::physx::PxArticulationReducedCoordinate*> seenArtis;
    for (const auto& pattern : patterns)
    {
        size_t currentSize = entries.size();
        findMatchingArticulations(pattern, entries, seenArtis);
        if (entries.size() == currentSize)
        {
            if (isNoMatchLoggingQuiet())
            {
                CARB_LOG_INFO("Pattern '%s' did not match any articulations\n", pattern.c_str());
            }
            else
            {
                CARB_LOG_ERROR("Pattern '%s' did not match any articulations\n", pattern.c_str());
            }
        }
    }
}
void BaseSimulationView::findMatchingArticulations(const std::string& pattern,
                                                   std::vector<ArticulationEntry>& entriesRet,
                                                   std::unordered_set<const ::physx::PxArticulationReducedCoordinate*>& seenArtis)
{
    if (!mStage)
    {
        return;
    }
    if (pattern.empty())
    {
        CARB_LOG_ERROR("Empty pattern not allowed");
        return;
    }
    // we only support absolute paths atm
    if (pattern[0] != '/')
    {
        CARB_LOG_ERROR("Pattern must be an absolute USD path, got '%s'\n", pattern.c_str());
        return;
    }

    std::vector<SdfPath> paths;
    findMatchingPaths(pattern, paths);

    // Dedup by PxArticulation pointer using a set owned by the caller
    // (processArticulationEntries), so overlapping patterns that resolve
    // to the same articulation don't inflate the view. The set also
    // absorbs within-pattern duplicates from recursive leaf matching and
    // '**' expansion hitting several prims of the same articulation
    // (the root Xform plus any link), which getArticulationAtPath
    // resolves to the same articulation.
    for (unsigned i = 0; i < paths.size(); i++)
    {
        ArticulationEntry entry;
        if (getArticulationAtPath(paths[i], entry))
        {
            if (seenArtis.insert(entry.arti).second)
            {
                entriesRet.push_back(entry);
            }
        }
    }
}

void BaseSimulationView::processRigidBodyEntries(const std::vector<std::string>& patterns,
                                                 std::vector<RigidBodyEntry>& entries)
{
    // Dedup across the full pattern list: overlapping patterns that
    // resolve to the same PxRigidBody (for example an articulation-root
    // prim and its root link prim, or any pattern matched by two rules)
    // must not be appended twice.
    std::unordered_set<const ::physx::PxRigidBody*> seenBodies;
    for (const auto& pattern : patterns)
    {
        size_t currentSize = entries.size();
        findMatchingRigidBodies(pattern, entries, seenBodies);
        if (entries.size() == currentSize)
        {
            if (isNoMatchLoggingQuiet())
            {
                CARB_LOG_INFO("Pattern '%s' did not match any rigid bodies\n", pattern.c_str());
            }
            else
            {
                CARB_LOG_ERROR("Pattern '%s' did not match any rigid bodies\n", pattern.c_str());
            }
        }
    }
}

void BaseSimulationView::findMatchingRigidBodies(const std::string& pattern,
                                                 std::vector<RigidBodyEntry>& entriesRet,
                                                 std::unordered_set<const ::physx::PxRigidBody*>& seenBodies)
{
    if (!mStage)
    {
        return;
    }
    if (pattern.empty())
    {
        CARB_LOG_ERROR("Empty pattern not allowed");
        return;
    }
    // we only support absolute paths atm
    if (pattern[0] != '/')
    {
        CARB_LOG_ERROR("Pattern must be an absolute USD path, got '%s'\n", pattern.c_str());
        return;
    }

    std::vector<SdfPath> paths;
    findMatchingPaths(pattern, paths);

    // Dedup by body pointer using a set owned by the caller
    // (processRigidBodyEntries), so overlapping patterns that resolve to
    // the same PxRigidBody don't inflate the view. The set also absorbs
    // within-pattern duplicates from a '**' expansion hitting both an
    // articulation-root prim and its root link prim, which
    // getRigidBodyAtPath resolves to the same PxRigidBody.
    for (unsigned i = 0; i < paths.size(); i++)
    {
        RigidBodyEntry entry;
        if (getRigidBodyAtPath(paths[i], entry))
        {
            if (seenBodies.insert(entry.body).second)
            {
                entriesRet.push_back(entry);
            }
        }
    }
}

void BaseSimulationView::processVolumeDeformableBodyEntries(const std::vector<std::string>& patterns,
                                                           std::vector<DeformableBodyEntry>& entries)
{
    // Dedup across the full pattern list so overlapping patterns that
    // resolve to the same PxDeformableBody don't inflate the view.
    std::unordered_set<const ::physx::PxDeformableBody*> seenBodies;
    for (const auto& pattern : patterns)
    {
        size_t currentSize = entries.size();
        findMatchingVolumeDeformableBodies(pattern, entries, seenBodies);
        if (entries.size() == currentSize)
        {
            CARB_LOG_ERROR("Pattern '%s' did not match any volume deformable bodies\n", pattern.c_str());
        }
    }
}

void BaseSimulationView::processSurfaceDeformableBodyEntries(const std::vector<std::string>& patterns,
                                                            std::vector<DeformableBodyEntry>& entries)
{
    // Dedup across the full pattern list so overlapping patterns that
    // resolve to the same PxDeformableBody don't inflate the view.
    std::unordered_set<const ::physx::PxDeformableBody*> seenBodies;
    for (const auto& pattern : patterns)
    {
        size_t currentSize = entries.size();
        findMatchingSurfaceDeformableBodies(pattern, entries, seenBodies);
        if (entries.size() == currentSize)
        {
            CARB_LOG_ERROR("Pattern '%s' did not match any surface deformable bodies\n", pattern.c_str());
        }
    }
}

void BaseSimulationView::processDeformableMaterialEntries(const std::vector<std::string>& patterns,
                                                         std::vector<DeformableMaterialEntry>& entries)
{
    // Dedup across the full pattern list so overlapping patterns that
    // resolve to the same PxDeformableMaterial don't inflate the view.
    std::unordered_set<const ::physx::PxDeformableMaterial*> seenMaterials;
    for (const auto& pattern : patterns)
    {
        size_t currentSize = entries.size();
        findMatchingDeformableMaterials(pattern, entries, seenMaterials);
        if (entries.size() == currentSize)
        {
            CARB_LOG_ERROR("Pattern '%s' did not match any deformable materials\n", pattern.c_str());
        }
    }
}

void BaseSimulationView::findMatchingVolumeDeformableBodies(const std::string& pattern,
                                                            std::vector<DeformableBodyEntry>& entriesRet,
                                                            std::unordered_set<const ::physx::PxDeformableBody*>& seenBodies)
{
    if (!mStage)
    {
        return;
    }

    // we only support absolute paths atm
    if (pattern[0] != '/')
    {
        CARB_LOG_ERROR("Pattern must be an absolute USD path, got '%s'\n", pattern.c_str());
        return;
    }

    std::vector<SdfPath> paths;
    findMatchingPaths(pattern, paths);

    for (unsigned i = 0; i < paths.size(); i++)
    {
        DeformableBodyEntry entry;
        if (getVolumeDeformableBodyAtPath(paths[i], entry))
        {
            if (seenBodies.insert(entry.body).second)
            {
                entriesRet.push_back(entry);
            }
        }
    }
}

void BaseSimulationView::findMatchingSurfaceDeformableBodies(const std::string& pattern,
                                                             std::vector<DeformableBodyEntry>& entriesRet,
                                                             std::unordered_set<const ::physx::PxDeformableBody*>& seenBodies)
{
    if (!mStage)
    {
        return;
    }

    // we only support absolute paths atm
    if (pattern[0] != '/')
    {
        CARB_LOG_ERROR("Pattern must be an absolute USD path, got '%s'\n", pattern.c_str());
        return;
    }

    std::vector<SdfPath> paths;
    findMatchingPaths(pattern, paths);

    for (unsigned i = 0; i < paths.size(); i++)
    {
        DeformableBodyEntry entry;
        if (getSurfaceDeformableBodyAtPath(paths[i], entry))
        {
            if (seenBodies.insert(entry.body).second)
            {
                entriesRet.push_back(entry);
            }
        }
    }
}

void BaseSimulationView::findMatchingDeformableMaterials(const std::string& pattern,
                                                         std::vector<DeformableMaterialEntry>& entriesRet,
                                                         std::unordered_set<const ::physx::PxDeformableMaterial*>& seenMaterials)
{
    if (!mStage)
    {
        return;
    }

    // we only support absolute paths atm
    if (pattern[0] != '/')
    {
        CARB_LOG_ERROR("Pattern must be an absolute USD path, got '%s'\n", pattern.c_str());
        return;
    }

    std::vector<SdfPath> paths;
    findMatchingPaths(pattern, paths);

    for (unsigned i = 0; i < paths.size(); i++)
    {
        DeformableMaterialEntry entry;
        if (getDeformableMaterialAtPath(paths[i], entry))
        {
            if (seenMaterials.insert(entry.material).second)
            {
                entriesRet.push_back(entry);
            }
        }
    }
}

void BaseSimulationView::processRigidContactViewEntries(const std::vector<std::string>& patterns,
                                                         const std::vector<std::vector<std::string>>& _filterPatterns,
                                                         std::vector<RigidContactSensorEntry>& entries,
                                                         uint32_t& filterPatternSize)
{
    if (patterns.empty())
    {
        CARB_LOG_ERROR("Empty patterns not allowed");
        return;
    }
    std::vector<std::vector<std::string>> filterPatterns;
    if (patterns.size() != _filterPatterns.size())
    {
        // No filter pattern specified, pass an empty filter pattern list for each sub pattern
        if (_filterPatterns.empty())
        {
            filterPatterns.resize(patterns.size());
        }
        else
        {
            CARB_LOG_ERROR("Size of the filter pattern list must match the size of the sensor pattern list");
            return;
        }
    }
    else
    {
        filterPatterns = _filterPatterns;
    }

    filterPatternSize = 0;
    // Dedup sensor paths across the full pattern list: overlapping
    // patterns (or the new recursive / '**' expansions) can surface the
    // same SdfPath more than once, and findMatchingRigidContactSensors
    // pairs sensors with filters positionally — so leaving duplicates
    // would silently pair a sensor with the wrong filter.
    std::unordered_set<PXR_NS::SdfPath, PXR_NS::SdfPath::Hash> seenSensorPaths;
    for (size_t i = 0; i < patterns.size(); ++i)
    {
        size_t currentSize = entries.size();
        findMatchingRigidContactSensors(patterns[i], filterPatterns[i], entries, seenSensorPaths);
        if (entries.size() == currentSize)
        {
            CARB_LOG_ERROR("Pattern '%s' did not match any rigid contact for filters\n", patterns[i].c_str());
            for (size_t j = 0; j < filterPatterns[i].size(); ++j)
            {
                CARB_LOG_ERROR("%s", filterPatterns[i][j].c_str());
            }
        }
        filterPatternSize = uint32_t(filterPatterns[i].size());
        if (i > 0 && filterPatterns[i].size() != filterPatterns[i - 1].size())
        {
            CARB_LOG_ERROR(
                "Number of all filter patterns for sensors should be equal. Sensor %s has %d filters while sensor %s has %d filters\n",
                patterns[i].c_str(), int(filterPatterns[i].size()), patterns[i - 1].c_str(),
                int(filterPatterns[i - 1].size()));
        }
    }
}
void BaseSimulationView::findMatchingRigidContactSensors(const std::string& pattern,
                                                         const std::vector<std::string>& filterPatterns,
                                                         std::vector<RigidContactSensorEntry>& entriesRet,
                                                         std::unordered_set<PXR_NS::SdfPath, PXR_NS::SdfPath::Hash>& seenSensorPaths)
{
    if (!mStage)
    {
        return;
    }

    // Validate the sensor pattern syntactically. Check `empty()` before
    // indexing into the first character — the rigid-body and articulation
    // helpers do it in this order and it avoids UB on pattern[0].
    if (pattern.empty())
    {
        CARB_LOG_ERROR("Empty pattern not allowed");
        return;
    }
    if (pattern[0] != '/')
    {
        CARB_LOG_ERROR("Pattern must be an absolute USD path, got '%s'\n", pattern.c_str());
        return;
    }

    // Validate all filter patterns up-front and treat any malformed filter
    // pattern as fatal for this sensor pattern: continuing with an empty
    // filter column would silently produce sensor entries with a blank
    // filter mapping (easy to hit given the shared dedup set above). Bail
    // before we touch any shared state.
    const uint32_t numFilterPatterns = uint32_t(filterPatterns.size());
    for (uint32_t i = 0; i < numFilterPatterns; i++)
    {
        const std::string& filterPattern = filterPatterns[i];
        if (filterPattern.empty())
        {
            CARB_LOG_ERROR("Empty filter pattern not allowed");
            return;
        }
        if (filterPattern[0] != '/')
        {
            CARB_LOG_ERROR("Pattern must be an absolute USD path, got filter pattern '%s'\n", filterPattern.c_str());
            return;
        }
    }

    std::vector<SdfPath> paths;
    findMatchingPaths(pattern, paths);

    // Filter expansion below is positional: filterPaths[j][i] must line up
    // with paths[i], and the size-1 broadcast assumes paths.size() is the
    // intended sensor count. Duplicate paths — from overlapping patterns,
    // recursive leaf matching, or '**' expansions — break both invariants,
    // so drop duplicates (preserving order) before we build filterPaths.
    auto deduppedEnd = std::remove_if(paths.begin(), paths.end(),
        [&seenSensorPaths](const SdfPath& p) { return !seenSensorPaths.insert(p).second; });
    paths.erase(deduppedEnd, paths.end());

    std::vector<std::vector<SdfPath>> filterPaths(numFilterPatterns);
    for (uint32_t i = 0; i < numFilterPatterns; i++)
    {
        const std::string& filterPattern = filterPatterns[i];
        findMatchingPaths(filterPattern, filterPaths[i]);

        // Special case: if only a single match is found, then assume all sensors should report contacts with a single
        // object, like a common ground plane.
        if (filterPaths[i].size() == 1)
        {
            filterPaths[i].resize(paths.size(), filterPaths[i][0]);
        }
        else if (filterPaths[i].size() != paths.size())
        {
            // Size mismatch is fatal too: a blank filter column would be
            // the same partial-disable footgun as a malformed pattern.
            // Fail the call so the user notices.
            CARB_LOG_ERROR("Filter pattern '%s' did not match the correct number of entries (expected %u, found %u)",
                filterPattern.c_str(), unsigned(paths.size()), unsigned(filterPaths[i].size()));
            return;
        }
    }

    for (unsigned i = 0; i < paths.size(); i++)
    {
        RigidContactSensorEntry entry;
        if (getRigidContactSensorAtPath(paths[i], entry))
        {
            entriesRet.push_back(entry);

            // add contact filter mappings
            auto& e = entriesRet.back();
            for (uint32_t j = 0; j < numFilterPatterns; j++)
            {
                e.filterPaths.push_back(filterPaths[j][i]);
                if (!filterPaths[j][i].IsEmpty())
                {
                    uint64_t pathId = asInt(filterPaths[j][i]);
                    e.filterIndexMap[pathId] = j;
                }
            }
        }
    }
}

void BaseSimulationView::findMatchingSDFShapes(const std::string& pattern, std::vector<SdfShapeEntry>& entriesRet, uint32_t numSamplePoints)
{
    if (!mStage)
    {
        return;
    }

    // we only support absolute paths atm
    if (pattern[0] != '/')
    {
        CARB_LOG_ERROR("Pattern must be an absolute USD path, got '%s'\n", pattern.c_str());
        return;
    }

    std::vector<SdfPath> paths;
    findMatchingPaths(pattern, paths);

    for (unsigned i = 0; i < paths.size(); i++)
    {
        SdfShapeEntry entry;
        if (getSDFShapeAtPath(paths[i], entry))
        {
            entry.numSamplePoints = numSamplePoints;
            entriesRet.push_back(entry);
        }
    }
}

ObjectType BaseSimulationView::getObjectType(const char* path)
{
    if (!g_physx)
    {
        return ObjectType::eInvalid;
    }
    if (PXR_NS::SdfPath::IsValidPathString(path))
    {
        SdfPath sdfPath = SdfPath(path);

        UsdPrim prim = mStage->GetPrimAtPath(sdfPath);

        if (prim)
        {
            PxArticulationReducedCoordinate* arti = nullptr;
            // check if it's an articulation link
            PxArticulationLink* link = (PxArticulationLink*)g_physx->getPhysXPtr(sdfPath, omni::physx::ePTLink);
            if (link)
            {
                arti = &static_cast<PxArticulationReducedCoordinate&>(link->getArticulation());
                if (arti)
                {
                    PxU32 numLinks = arti->getNbLinks();
                    if (numLinks > 0)
                    {
                        const PxU32 linkIndex = link->getLinkIndex();
                        if (linkIndex == 0)
                            return ObjectType::eArticulationRootLink;
                        else
                            return ObjectType::eArticulationLink;
            
                    }
                }
            }
            // check if it's an articulation but not a link, i.e.
            arti = (PxArticulationReducedCoordinate*)g_physx->getPhysXPtr(sdfPath, omni::physx::ePTArticulation);
            if (arti)
            {
                return ObjectType::eArticulation;
            }

            // check if it's an articulation joint
            PxArticulationJointReducedCoordinate* joint =
                (PxArticulationJointReducedCoordinate*)g_physx->getPhysXPtr(sdfPath, omni::physx::ePTLinkJoint);
            if (joint)
            {
                return ObjectType::eArticulationJoint;
            }

            PxActor* actor = static_cast<PxActor*>(g_physx->getPhysXPtr(sdfPath, omni::physx::ePTActor));
            if (actor)
            {
                // check if it's a rigid dynamic
                if (actor->getType() == PxActorType::eRIGID_DYNAMIC)
                {
                    return ObjectType::eRigidBody;
                }
            }
        }
    }
    return ObjectType::eInvalid;
}

bool BaseSimulationView::getArticulationAtPath(const SdfPath& path, ArticulationEntry& entryRet)
{
    if (!g_physx)
    {
        return false;
    }
    // check if it's an articulation
    PxArticulationReducedCoordinate* arti =
        (PxArticulationReducedCoordinate*)g_physx->getPhysXPtr(path, omni::physx::ePTArticulation);
    if (arti)
    {
    }
    else
    {
        // check if it's an articulation link
        PxArticulationLink* link = (PxArticulationLink*)g_physx->getPhysXPtr(path, omni::physx::ePTLink);
        if (link)
        {
            arti = &static_cast<PxArticulationReducedCoordinate&>(link->getArticulation());
        }
        else
        {
            // check if it's an articulation joint
            PxArticulationJointReducedCoordinate* joint =
                (PxArticulationJointReducedCoordinate*)g_physx->getPhysXPtr(path, omni::physx::ePTLinkJoint);
            if (joint)
            {
                arti =
                    &static_cast<PxArticulationReducedCoordinate&>(joint->getChildArticulationLink().getArticulation());
            }
        }
    }

    if (!arti || arti->getConcreteType() != PxConcreteType::eARTICULATION_REDUCED_COORDINATE)
    {
        CARB_LOG_WARN("Failed to find articulation at '%s'", path.GetString().c_str());
        return false;
    }

    // TODO: handle this better
    if (!arti->getScene())
    {
        CARB_LOG_ERROR("Articulation is not in a scene!");
        return false;
    }

    // get links
    PxU32 numLinks = arti->getNbLinks();
    std::vector<PxArticulationLink*> links(numLinks);
    arti->getLinks(links.data(), numLinks);

    std::vector<PxU32> dofStarts(numLinks, 0);
    std::vector<FreeD6RotationAxesFlags> freeRotationAxes(numLinks, FreeD6RotationAxesFlags(0));

    dofStarts[0] = 0; // The root link never has an incoming articulation joint
    // get the ordering of the links in the articulation cache
    std::vector<PxArticulationLink*> orderedLinks(numLinks);
    for (PxU32 j = 0; j < numLinks; j++)
    {
        PxU32 linkIdx = links[j]->getLinkIndex();
        orderedLinks[linkIdx] = links[j];
        if (j > 0)
        {
            PxU32 dofs = links[j]->getInboundJointDof();
            dofStarts[linkIdx] = dofs;
        }
    }

    // this is important so that link and DOF traversals are done in articulation cache order
    links = orderedLinks;


    // count DOFs, and dofStarts scan
    PxU32 numDofs = 0;
    PxU32 count = 0;
    for (PxU32 j = 1; j < numLinks; j++)
    {
        numDofs += dofStarts[j];
        PxU32 dofs = dofStarts[j];
        dofStarts[j] = count;
        count += dofs;
    }
    entryRet.dofStarts = dofStarts;
    // figure out the canonical path that this articulation is mapped to in omni.physx
    size_t objectId = reinterpret_cast<size_t>(arti->userData);
    SdfPath canonicalPath;
    if (g_physx)
    {
        canonicalPath = g_physx->getPhysXObjectUsdPath(objectId);
    }

    //
    // figure out the metatype (kinematic desc)
    //

    ArticulationMetatype metatype;
    std::vector<DofImpl> dofImpls;

    // inverses of the rotations applied to incoming joint local poses during USD parsing.
    // needed to transform the output of linkIncomingJointForce to the right
    // local space because omniphysics rotates joint local poses internally during parsing.
    std::vector<PxQuat> physxToUsdRotations(numLinks);
    std::vector<PxTransform> jointChildxforms(numLinks);
    std::vector<PxTransform> jointParentxforms(numLinks);
    std::vector<PxU32> parentIndices(numLinks);
    std::vector<bool> isUsdBody0Parent(numLinks);
    std::vector<PxU32> linkIdMap(numLinks);
    metatype.setFixedBase(arti->getArticulationFlags().isSet(PxArticulationFlag::eFIX_BASE));
    for (PxU32 i = 0; i < numLinks; i++)
    {
        // get link info
        PxArticulationLink* link = links[i];
        size_t linkId = reinterpret_cast<size_t>(link->userData);
        SdfPath linkPath = g_physx->getPhysXObjectUsdPath(linkId);
        std::string linkName = linkPath.GetName();

        // To avoid duplicate name
        PxU32 result = metatype.findLinkIndex(linkName.c_str());
        if (result != -1)
        {
            std::string newLinkName;
            for (PxU32 j = 0; j < numLinks; j++)
            {
                newLinkName = linkName + "_" + std::to_string(j);
                result = metatype.findLinkIndex(newLinkName.c_str());
                if (result == -1)
                    break;
            }
            linkName = newLinkName;
        }
        linkIdMap[i] = static_cast<PxU32>(linkId);
        metatype.addLink(linkName);
    }

    for (PxU32 i = 0; i < numLinks; i++)
    {
        // get link info
        PxArticulationLink* link = links[i];
        // set to identity first.
        carb::Float4 physxToUsdRotation = carb::Float4{0, 0, 0, 1.f};
        PxQuat rotationQuat(0.0f, 0.0f, 0.0f, 1.0f);
        PxTransform jointChild(PxIdentity);
        PxTransform jointParent(PxIdentity);
        PxU32 ParentLinkIndex = 0xffffffff;

        bool body0IsParent = true;
        // get joint info
        PxArticulationJointReducedCoordinate* joint =
            static_cast<PxArticulationJointReducedCoordinate*>(link->getInboundJoint());
        if (joint)
        {
            size_t jointId = reinterpret_cast<size_t>(joint->userData);
            SdfPath jointPath = g_physx->getPhysXObjectUsdPath(jointId);
            std::string jointName = jointPath.GetName();

            // To avoid duplicate name
            PxU32 result = metatype.findJointIndex(jointName.c_str());
            if (result != -1)
            {
                std::string newJointName;
                for (PxU32 j = 0; j < numLinks; j++)
                {
                    newJointName = jointName + "_" + std::to_string(j);
                    result = metatype.findJointIndex(newJointName.c_str());
                    if (result == -1)
                        break;
                }
                jointName = newJointName;
            }

            UsdPrim jointPrim = mStage->GetPrimAtPath(jointPath);

            ArticulationMetatype::JointDesc jointDesc;
            jointDesc.name = jointName;
            const usdparser::ObjectId objectId = g_physx->getObjectId(jointPath, ePTLinkJoint);
            omni::physx::JointStateData jointStateData;
            g_physxJoint->getJointStateData(objectId, &jointStateData);

            // Parsed joint-drive descriptor from the internal DB (carries the
            // DrivePerformanceEnvelope flag). We read the envelope flag from here
            // instead of the USD prim so it resolves for runtime-replicated clones
            // whose joint prims are absent from the USD stage (mStage).
            const internal::InternalJoint* intJoint = nullptr;
            {
                const internal::InternalPhysXDatabase& physxDb = OmniPhysX::getInstance().getInternalPhysXDatabase();
                if (objectId < physxDb.getRecords().size())
                {
                    const internal::InternalDatabase::Record& rec = physxDb.getRecords()[objectId];
                    if (rec.mInternalPtr && rec.mType == ePTLinkJoint)
                        intJoint = reinterpret_cast<const internal::InternalJoint*>(rec.mInternalPtr);
                }
            }
            jointDesc.body0IsParent = jointStateData.body0IsParentLink;
            body0IsParent = jointStateData.body0IsParentLink;
            physxToUsdRotation = g_physxJoint->getJointFramePhysxToUsdQuat(jointId);
            rotationQuat = PxQuat(physxToUsdRotation.x, physxToUsdRotation.y, physxToUsdRotation.z, physxToUsdRotation.w);

            PxArticulationLink& parentLink = joint->getParentArticulationLink();
            PxArticulationLink& childLink = joint->getChildArticulationLink();
            size_t linkId = reinterpret_cast<size_t>(parentLink.userData);
            SdfPath linkPath = g_physx->getPhysXObjectUsdPath(linkId);
            // Search parent link index using linkId
            ParentLinkIndex = -1;
            for (PxU32 j = 0; j < numLinks; j++)
            {
                if (linkIdMap[j] == linkId)
                {
                    ParentLinkIndex = j;
                    break;
                }
            }
            std::string linkName = metatype.getLinkName(ParentLinkIndex);

            size_t childLinkId = reinterpret_cast<size_t>(childLink.userData);
            SdfPath childLinkPath = g_physx->getPhysXObjectUsdPath(childLinkId);
            PxU32 childLinkIndex = i;
            metatype.setLinkParentIndex(childLinkIndex, ParentLinkIndex);
            jointParent = joint->getParentPose();
            jointChild = joint->getChildPose();

            PxArticulationJointType::Enum jointType = joint->getJointType();
            PxU32 isEnvelopeUsed = 0;

            switch (jointType)
            {
            case PxArticulationJointType::eFIX:
                jointDesc.type = JointType::eFixed;
                break;
            case PxArticulationJointType::eREVOLUTE:
            case PxArticulationJointType::eREVOLUTE_UNWRAPPED:
                jointDesc.type = JointType::eRevolute;
                jointDesc.dofs.emplace_back(jointName, DofType::eRotation);

                if (intJoint && intJoint->mJointDrive.isEnvelopeUsed)
                    isEnvelopeUsed = 1;
                dofImpls.push_back({ joint, PxArticulationAxis::eTWIST, isEnvelopeUsed });
                break;
            case PxArticulationJointType::ePRISMATIC:
                jointDesc.type = JointType::ePrismatic;
                jointDesc.dofs.emplace_back(jointName, DofType::eTranslation);
                if (intJoint && intJoint->mJointDrive.isEnvelopeUsed)
                    isEnvelopeUsed = 1;
                dofImpls.push_back({ joint, PxArticulationAxis::eX, isEnvelopeUsed });
                break;
            case PxArticulationJointType::eSPHERICAL:
                if (!rotationQuat.isIdentity())
                    CARB_LOG_WARN("Using USD spherical joints with any axis except x is not currently supported.");
                // figure out which axes are unlocked
                jointDesc.type = JointType::eSpherical;
                if (joint->getMotion(PxArticulationAxis::eTWIST) != PxArticulationMotion::eLOCKED)
                {
                    // HACK? resolve custom DOF name from MJCF importer
                    freeRotationAxes[i].raise(FreeD6RotationAxesFlag::Enum::eTWIST);

                    {
                        static TfToken dofNameAttribToken("mjcf:rotX:name");
                        UsdAttribute dofNameAttrib = jointPrim ? jointPrim.GetAttribute(dofNameAttribToken) : UsdAttribute();

                        TfToken dofNameToken;
                        std::string dofName;
                        if (dofNameAttrib && dofNameAttrib.Get(&dofNameToken))
                        {
                            dofName = dofNameToken.GetString();
                        }
                        else
                        {
                            dofName = jointName + ":0";
                        }
                        jointDesc.dofs.emplace_back(dofName, DofType::eRotation);
                        if (intJoint && intJoint->mJointDrives[PxArticulationAxis::eTWIST].isEnvelopeUsed)
                            isEnvelopeUsed = 1;
                    }
                    dofImpls.push_back({joint, PxArticulationAxis::eTWIST, isEnvelopeUsed });
                    isEnvelopeUsed = 0;
                }
                if (joint->getMotion(PxArticulationAxis::eSWING1) != PxArticulationMotion::eLOCKED)
                {
                    // HACK? resolve custom DOF name from MJCF importer
                    freeRotationAxes[i].raise(FreeD6RotationAxesFlag::Enum::eSWING1);
                    {
                        static TfToken dofNameAttribToken("mjcf:rotY:name");
                        UsdAttribute dofNameAttrib = jointPrim ? jointPrim.GetAttribute(dofNameAttribToken) : UsdAttribute();
                        TfToken dofNameToken;
                        std::string dofName;
                        if (dofNameAttrib && dofNameAttrib.Get(&dofNameToken))
                        {
                            dofName = dofNameToken.GetString();
                        }
                        else
                        {
                            dofName = jointName + ":1";
                        }
                        jointDesc.dofs.emplace_back(dofName, DofType::eRotation);
                        if (intJoint && intJoint->mJointDrives[PxArticulationAxis::eSWING1].isEnvelopeUsed)
                            isEnvelopeUsed = 1;
                    }
                    dofImpls.push_back({joint, PxArticulationAxis::eSWING1, isEnvelopeUsed });
                    isEnvelopeUsed = 0;
                }
                if (joint->getMotion(PxArticulationAxis::eSWING2) != PxArticulationMotion::eLOCKED)
                {
                    // HACK? resolve custom DOF name from MJCF importer
                    freeRotationAxes[i].raise(FreeD6RotationAxesFlag::Enum::eSWING2);
                    {
                        static TfToken dofNameAttribToken("mjcf:rotZ:name");
                        UsdAttribute dofNameAttrib = jointPrim ? jointPrim.GetAttribute(dofNameAttribToken) : UsdAttribute();
                        TfToken dofNameToken;
                        std::string dofName;
                        if (dofNameAttrib && dofNameAttrib.Get(&dofNameToken))
                        {
                            dofName = dofNameToken.GetString();
                        }
                        else
                        {
                            dofName = jointName + ":2";
                        }
                        jointDesc.dofs.emplace_back(dofName, DofType::eRotation);
                        if (intJoint && intJoint->mJointDrives[PxArticulationAxis::eSWING2].isEnvelopeUsed)
                            isEnvelopeUsed = 1;
                    }
                    dofImpls.push_back({joint, PxArticulationAxis::eSWING2, isEnvelopeUsed });
                }
                break;
            case PxArticulationJointType::eUNDEFINED:
            default:
                CARB_LOG_ERROR("Unknown joint type for joint '%s'", jointPath.GetString().c_str());
                break;
            }

            if (jointDesc.type != JointType::eInvalid)
            {
                metatype.addJoint(jointDesc);
            }
        }

        CARB_ASSERT(rotationQuat.isUnit());
        physxToUsdRotations[i] = rotationQuat;
        jointChildxforms[i] = jointChild;
        jointParentxforms[i] = jointParent;
        parentIndices[i] = ParentLinkIndex;
        isUsdBody0Parent[i] = body0IsParent;
    }

    //
    // populate entry
    //

    entryRet.metatype = getUniqueArticulationMetatype(metatype);
    entryRet.arti = arti;
    entryRet.numLinks = numLinks;
    entryRet.numDofs = numDofs;
    entryRet.freeD6Axes = freeRotationAxes;

    entryRet.links = links;
    entryRet.dofImpls = dofImpls;
    entryRet.incomingJointPhysxToUsdRotations = physxToUsdRotations;
    entryRet.isIncomingJointBody0Parent = isUsdBody0Parent;
    entryRet.jointChild = jointChildxforms;
    entryRet.jointParent = jointParentxforms;
    entryRet.parentIndices = parentIndices;

    // shapes
    PxU32 numShapes = 0;
    for (PxU32 i = 0; i < numLinks; i++)
    {
        PxU32 linkNumShapes = links[i]->getNbShapes();
        if (linkNumShapes > 0)
        {
            std::vector<::physx::PxShape*> shapes(linkNumShapes);
            links[i]->getShapes(shapes.data(), linkNumShapes);
            entryRet.shapes.insert(entryRet.shapes.end(), shapes.begin(), shapes.end());
            numShapes += linkNumShapes;
        }
    }
    entryRet.numShapes = numShapes;

    // fixed tendons
    PxU32 numFixedTendons = arti->getNbFixedTendons();
    if (numFixedTendons > 0)
    {
        entryRet.numFixedTendons = numFixedTendons;
        entryRet.fixedTendons.resize(numFixedTendons);
        arti->getFixedTendons(entryRet.fixedTendons.data(), numFixedTendons);
    }

    // spatial tendons
    PxU32 numSpatialTendons = arti->getNbSpatialTendons();
    if (numSpatialTendons > 0)
    {
        entryRet.numSpatialTendons = numSpatialTendons;
        entryRet.spatialTendons.resize(numSpatialTendons);
        arti->getSpatialTendons(entryRet.spatialTendons.data(), numSpatialTendons);
    }

    if (!canonicalPath.IsEmpty())
    {
        entryRet.path = canonicalPath;
    }
    else
    {
        entryRet.path = path;
    }

    entryRet.subspace = findSubspaceForPath(entryRet.path);

    return true;
}

bool BaseSimulationView::getRigidBodyAtPath(const PXR_NS::SdfPath& path, RigidBodyEntry& entryRet)
{
    if (!g_physx)
    {
        return false;
    }

    PxRigidBody* body = nullptr;
    RigidBodyType type = RigidBodyType::eInvalid;

    // check if it's an articulation link
    PxArticulationLink* link = static_cast<PxArticulationLink*>(g_physx->getPhysXPtr(path, omni::physx::ePTLink));
    if (link)
    {
        body = link;
        type = RigidBodyType::eArticulationLink;
    }
    else
    {
        // check if it's an articulation, in which case we'll use the root link
        // NOTE: This is an important edge case when we instance single-body actors as articulations with a fixed base.
        //       (We can't use kinematic bodies, because OmniPhysX does not update kinematic transforms to USD/hydra.)
        PxArticulationReducedCoordinate* arti =
            (PxArticulationReducedCoordinate*)g_physx->getPhysXPtr(path, omni::physx::ePTArticulation);
        if (arti)
        {
            PxU32 numLinks = arti->getNbLinks();
            if (numLinks > 0)
            {
                std::vector<PxArticulationLink*> links(numLinks);
                arti->getLinks(links.data(), numLinks);
                body = links[0];
                type = RigidBodyType::eArticulationLink;
            }
        }
        else
        {
            // check if it's an actor
            PxActor* actor = static_cast<PxActor*>(g_physx->getPhysXPtr(path, omni::physx::ePTActor));
            if (actor)
            {
                // check if it's a rigid dynamic
                if (actor->getType() == PxActorType::eRIGID_DYNAMIC)
                {
                    PxRigidDynamic* rd = static_cast<PxRigidDynamic*>(actor);
                    body = rd;
                    type = RigidBodyType::eRigidDynamic;
                }
            }
        }
    }

    if (!body)
    {
        CARB_LOG_WARN("Failed to find rigid body at '%s'", path.GetString().c_str());
        return false;
    }

#if 0
    printf("Got rigid body of type %s at %p (%s)\n", body->getConcreteTypeName(), body, path.GetText());
    PxU32 numShapes = body->getNbShapes();
    std::vector<PxShape*> shapes(numShapes);
    body->getShapes(shapes.data(), numShapes);
    printf("  %u shapes\n", numShapes);
    for (PxU32 i = 0; i < numShapes; i++)
    {
        printf("    Shape %u\n", i);
        printf("      Contact offset: %f\n", shapes[i]->getContactOffset());
        printf("      Rest offset:    %f\n", shapes[i]->getRestOffset());
    }
#endif

    //
    // populate entry
    //

    entryRet.body = body;
    entryRet.type = type;
    entryRet.path = path;
    entryRet.subspace = findSubspaceForPath(entryRet.path);

    // shapes
    PxU32 numShapes = body->getNbShapes();
    if (numShapes > 0)
    {
        entryRet.shapes.resize(numShapes);
        body->getShapes(entryRet.shapes.data(), numShapes);
    }
    entryRet.numShapes = numShapes;

    return true;
}

bool BaseSimulationView::getVolumeDeformableBodyAtPath(const PXR_NS::SdfPath& path, DeformableBodyEntry& entryRet)
{
    if (!g_physx)
    {
        return false;
    }

    // check if it's a volume deformable body
    PxDeformableVolume* deformable = static_cast<PxDeformableVolume*>(g_physx->getPhysXPtr(path, omni::physx::ePTDeformableVolume));
    if (!deformable)
    {
        CARB_LOG_WARN("Failed to find volume deformable body at '%s'", path.GetString().c_str());
        return false;
    }

    TfType deformableBodyType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniUsdPhysicsDeformableSchemaTokens->OmniPhysicsDeformableBodyAPI);
    UsdPrim usdPrim = mStage->GetPrimAtPath(path);
    if (!usdPrim.IsValid() || !usdPrim.HasAPI(deformableBodyType))
    {
        CARB_LOG_WARN("Volume deformable body at '%s' requires OmniPhysicsDeformableBodyAPI", path.GetText());
        return false;
    }

    SdfPath simMeshPath;
    SdfPath collMeshPath;
    SdfPathSet skinMeshPaths;
    bool success = findDeformableMeshPaths(simMeshPath, collMeshPath, skinMeshPaths, path, mStage);
    if (!success)
    {
        CARB_LOG_WARN("Failed to find simulation or collision mesh for volume deformable body at '%s'", path.GetText());
        return false;
    }

    UsdGeomTetMesh simTetMesh(mStage->GetPrimAtPath(simMeshPath));
    if (!simTetMesh)
    {
        CARB_LOG_WARN("Simulation mesh at '%s' is not a UsdGeomTetMesh", simMeshPath.GetText());
        return false;
    }

    UsdGeomTetMesh collTetMesh(mStage->GetPrimAtPath(collMeshPath));
    if (!collTetMesh)
    {
        CARB_LOG_WARN("Collision mesh at '%s' is not a UsdGeomTetMesh", collMeshPath.GetText());
        return false;
    }

    VtArray<GfVec4i> simMeshIndices;
    simTetMesh.GetTetVertexIndicesAttr().Get(&simMeshIndices);

    // Read rest shape attributes and check on current restrictions
    VtArray<GfVec3f> restPositions;
    {
        VtArray<GfVec4i> restTetVtxIndices;
        simTetMesh.GetPrim().GetAttribute(OmniUsdPhysicsDeformableSchemaTokens->omniphysicsRestTetVtxIndices).Get(&restTetVtxIndices);

        if (simMeshIndices.size() != restTetVtxIndices.size() ||
            std::memcmp(simMeshIndices.data(), restTetVtxIndices.data(), sizeof(GfVec4i) * simMeshIndices.size()) != 0)
        {
            CARB_LOG_WARN("No support for distinct rest shape topology. The simulation mesh's tetVertexIndices need to "
                          "match up with VolumeDeformableSimAPI restTetVtxIndices at '%s'", simMeshPath.GetText());
            return false;
        }

        VtArray<GfVec3f> simPoints;
        simTetMesh.GetPointsAttr().Get(&simPoints);

        simTetMesh.GetPrim().GetAttribute(OmniUsdPhysicsDeformableSchemaTokens->omniphysicsRestShapePoints).Get(&restPositions);

        if (simPoints.size() != restPositions.size())
        {
            CARB_LOG_WARN("No support for distinct rest shape topology. The simulation mesh's points need to match up "
                          "with VolumeDeformableSimAPI restShapePoints at '%s'", simMeshPath.GetText());
        }
    }

    VtArray<GfVec4i> collMeshIndices;
    collTetMesh.GetTetVertexIndicesAttr().Get(&collMeshIndices);

    // populate entry
    entryRet.body = deformable;
    entryRet.path = path;
    entryRet.simMeshPath = simMeshPath;
    entryRet.collMeshPath = collMeshPath;
    entryRet.subspace = findSubspaceForPath(entryRet.path);

    entryRet.simIndices.resize(simMeshIndices.size() * 4);
    std::memcpy(entryRet.simIndices.data(), simMeshIndices.data(), sizeof(PxU32) * entryRet.simIndices.size());

    // transform restPositions into world space to account for scaling
    entryRet.restPositions.resize(restPositions.size());
    GfMatrix4d sim_to_world = UsdGeomXformable(simTetMesh).ComputeLocalToWorldTransform(UsdTimeCode::Default());
    for (size_t i = 0; i < entryRet.restPositions.size(); ++i)
    {
        GfVec3f restPoint = GfVec3f(sim_to_world.Transform(restPositions[i]));
        entryRet.restPositions[i] = { restPoint[0], restPoint[1], restPoint[2] };
    }

    entryRet.collIndices.resize(collMeshIndices.size() * 4);
    std::memcpy(entryRet.collIndices.data(), collMeshIndices.data(), sizeof(PxU32) * entryRet.collIndices.size());

    return true;
}

bool BaseSimulationView::getSurfaceDeformableBodyAtPath(const PXR_NS::SdfPath& path, DeformableBodyEntry& entryRet)
{
    if (!g_physx)
    {
        return false;
    }

    // check if it's a surface deformable body
    PxDeformableSurface* deformable = static_cast<PxDeformableSurface*>(g_physx->getPhysXPtr(path, omni::physx::ePTDeformableSurface));
    if (!deformable)
    {
        CARB_LOG_WARN("Failed to find surface deformable body at '%s'", path.GetString().c_str());
        return false;
    }

    TfType deformableBodyType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniUsdPhysicsDeformableSchemaTokens->OmniPhysicsDeformableBodyAPI);
    UsdPrim usdPrim = mStage->GetPrimAtPath(path);
    if (!usdPrim.IsValid() || !usdPrim.HasAPI(deformableBodyType))
    {
        CARB_LOG_WARN("Surface deformable body at '%s' requires OmniPhysicsDeformableBodyAPI", path.GetText());
        return false;
    }

    SdfPath simMeshPath;
    SdfPath collMeshPath;
    SdfPathSet skinMeshPaths;
    bool success = findDeformableMeshPaths(simMeshPath, collMeshPath, skinMeshPaths, path, mStage);
    if (!success)
    {
        CARB_LOG_WARN("Failed to find simulation or collision mesh for surface deformable body at '%s'", path.GetText());
        return false;
    }

    if (simMeshPath != collMeshPath)
    {
        CARB_LOG_WARN("Found surface deformable body with separate collision mesh, which is not supported '%s'", path.GetText());
        return false;
    }

    UsdGeomMesh simTriMesh(mStage->GetPrimAtPath(simMeshPath));
    if (!simTriMesh)
    {
        CARB_LOG_WARN("Simulation mesh at '%s' is not a UsdGeomMesh", simMeshPath.GetText());
        return false;
    }

    VtArray<int> simMeshIndices;
    simTriMesh.GetFaceVertexIndicesAttr().Get(&simMeshIndices);
    if (simMeshIndices.size() % 3 != 0)
    {
        CARB_LOG_WARN("Simulation mesh at '%s' has non-triangular faces", simMeshPath.GetText());
        return false;
    }

    // Read rest shape attributes and check on current restrictions
    VtArray<GfVec3f> restPositions;
    {
        VtArray<GfVec3i> restTriVtxIndices;
        simTriMesh.GetPrim().GetAttribute(OmniUsdPhysicsDeformableSchemaTokens->omniphysicsRestTriVtxIndices).Get(&restTriVtxIndices);

        if (simMeshIndices.size() != restTriVtxIndices.size()*3 ||
            std::memcmp(simMeshIndices.data(), restTriVtxIndices.data(), sizeof(int32_t) * simMeshIndices.size()) != 0)
        {
            CARB_LOG_WARN(
                "No support for distinct rest shape topology. The simulation mesh's faceVertexIndices need to "
                "match up with SurfaceDeformableSimAPI restTriVtxIndices at '%s'",
                simMeshPath.GetText());
            return false;
        }

        VtArray<GfVec3f> simPoints;
        simTriMesh.GetPointsAttr().Get(&simPoints);

        simTriMesh.GetPrim().GetAttribute(OmniUsdPhysicsDeformableSchemaTokens->omniphysicsRestShapePoints).Get(&restPositions);

        if (simPoints.size() != restPositions.size())
        {
            CARB_LOG_WARN(
                "No support for distinct rest shape topology. The simulation mesh's points need to match up "
                "with SurfaceDeformableSimAPI restShapePoints at '%s'",
                simMeshPath.GetText());
        }
    }

    // populate entry
    entryRet.body = deformable;
    entryRet.path = path;
    entryRet.simMeshPath = simMeshPath;
    entryRet.collMeshPath = simMeshPath;
    entryRet.subspace = findSubspaceForPath(entryRet.path);

    entryRet.simIndices.resize(simMeshIndices.size());
    std::memcpy(entryRet.simIndices.data(), simMeshIndices.data(), sizeof(PxU32) * entryRet.simIndices.size());

    //transform restPositions into world space to account for scaling
    entryRet.restPositions.resize(restPositions.size());
    GfMatrix4d sim_to_world = UsdGeomXformable(simTriMesh).ComputeLocalToWorldTransform(UsdTimeCode::Default());
    for (size_t i = 0; i < entryRet.restPositions.size(); ++i)
    {
        GfVec3f restPoint = GfVec3f(sim_to_world.Transform(restPositions[i]));
        entryRet.restPositions[i] = { restPoint[0], restPoint[1], restPoint[2] };
    }

    return true;
}

bool BaseSimulationView::getDeformableMaterialAtPath(const PXR_NS::SdfPath& path, DeformableMaterialEntry& entryRet)
{
    if (!g_physx)
    {
        return false;
    }

    PxDeformableMaterial* material = static_cast<PxDeformableMaterial*>(g_physx->getPhysXPtr(path, omni::physx::ePTDeformableVolumeMaterial));
    bool isSurface = false;
    if (!material)
    {
        material = static_cast<PxDeformableMaterial*>(g_physx->getPhysXPtr(path, omni::physx::ePTDeformableSurfaceMaterial));
        isSurface = (material != nullptr);
    }

    if (!material)
    {
        CARB_LOG_WARN("Failed to find deformable material at '%s'", path.GetText());
        return false;
    }

    // populate entry
    entryRet.material = material;
    entryRet.path = path;
    entryRet.isSurface = isSurface;

    return true;
}

bool BaseSimulationView::getRigidContactSensorAtPath(const PXR_NS::SdfPath& path, RigidContactSensorEntry& entryRet)
{
    UsdPrim sensorPrim = mStage->GetPrimAtPath(path);

    if (sensorPrim)
    {
        PhysxSchemaPhysxContactReportAPI crApi = PhysxSchemaPhysxContactReportAPI(sensorPrim);
        if (!crApi)
        {
            CARB_LOG_WARN("Failed to find contact report API at '%s'", path.GetText());
            return false;
        }
    }
    else
    {
        // No USD prim at this path. Runtime clones can have a real PhysX actor
        // without a USD prim exposing the copied PhysxContactReportAPI metadata;
        // the source actor's contact-report registration is mirrored by the
        // PhysX replicator, so such paths are validated by the runtime PhysX
        // pointer lookup below (which returns false if no link/actor/shape is
        // found). Real USD prims still take the strict schema path above.
    }

    // figure out if it's a rigid dynamic, articulation link, or shape
    PxArticulationLink* link = nullptr;
    PxRigidDynamic* rd = nullptr;
    PxShape* shape = nullptr;
    link = static_cast<PxArticulationLink*>(g_physx->getPhysXPtr(path, omni::physx::ePTLink));
    if (!link)
    {
        PxActor* actor = static_cast<PxActor*>(g_physx->getPhysXPtr(path, omni::physx::ePTActor));
        if (actor && actor->getType() == PxActorType::eRIGID_DYNAMIC)
        {
            rd = static_cast<PxRigidDynamic*>(actor);
        }
        else
        {
            shape = static_cast<PxShape*>(g_physx->getPhysXPtr(path, omni::physx::ePTShape));
            if (!shape)
            {
                return false;
            }
        }
    }
    else
    {
        // for articulation links specify names for later to match with the articulation meta type
        size_t linkId = reinterpret_cast<size_t>(link->userData);
        SdfPath linkPath = g_physx->getPhysXObjectUsdPath(linkId);
        std::string name = linkPath.GetName();
        uint32_t currSize = (uint32_t)mSimData->mUniqueRCNames2Idx.size();
        if (mSimData->mUniqueRCNames2Idx.find(name) == mSimData->mUniqueRCNames2Idx.end())
        {
            mSimData->mUniqueRCNames2Idx.insert(std::make_pair(name, currSize));
            mSimData->mUniqueRCIdx2Names.insert(std::make_pair(currSize, name));
            entryRet.nameID = currSize;
        }
        else
        {
            entryRet.nameID = mSimData->mUniqueRCNames2Idx[name];
        }
    }

    //
    // populate entry
    //

    entryRet.path = path;
    entryRet.referentId = asInt(path);
    entryRet.link = link;
    entryRet.rd = rd;
    entryRet.shape = shape;
    entryRet.subspace = findSubspaceForPath(entryRet.path);

    return true;
}

bool BaseSimulationView::getSDFShapeAtPath(const PXR_NS::SdfPath& path, SdfShapeEntry& entryRet)
{
    UsdPrim Prim = mStage->GetPrimAtPath(path);

    if (Prim)
    {
        UsdPhysicsCollisionAPI collisionApi = UsdPhysicsCollisionAPI(Prim);
        PhysxSchemaPhysxSDFMeshCollisionAPI SDFMeshApi = PhysxSchemaPhysxSDFMeshCollisionAPI(Prim);
        if (!SDFMeshApi || !collisionApi)
        {
            CARB_LOG_WARN("Failed to find CollisionAPI and PhysxSDFMeshCollisionAPI for prim at ('%s')", path.GetText());
            return false;
        }
    }
    else
    {
        // No USD prim at this path (e.g. a runtime clone). The shape is
        // validated by the PhysX pointer lookup below (returns false if no
        // shape is found) and the SDF-validity check that follows. Real USD
        // prims still take the strict schema path above.
    }

    PxShape* shape = static_cast<PxShape*>(g_physx->getPhysXPtr(path, omni::physx::ePTShape));
    if (!shape)
    {
        CARB_LOG_WARN("Failed to find a mesh at '%s'", path.GetText());
        return false;
    }

    bool hasSDF = false;
    const PxGeometry& geom = shape->getGeometry();
    if(geom.getType() == PxGeometryType::eTRIANGLEMESH)
    {
        const PxTriangleMeshGeometry& triangleMeshGeom = static_cast<const PxTriangleMeshGeometry&>(geom);
        if(triangleMeshGeom.isValid())
        {
            hasSDF = triangleMeshGeom.triangleMesh->getSDF() != NULL;

            if(hasSDF)
            {
                PxU32 dimX, dimY, dimZ;
                triangleMeshGeom.triangleMesh->getSDFDimensions(dimX, dimY, dimZ);
                hasSDF = dimX > 0 && dimY > 0 && dimZ > 0;
            }
        }
    }

    if (!hasSDF)
    {
        CARB_LOG_WARN("Failed to find a valid SDF mesh for prim at ('%s')", path.GetText());
        return false;
    }

    entryRet.shape = shape;
    entryRet.path = path;
    entryRet.subspace = findSubspaceForPath(entryRet.path);
    return true;
}

const ArticulationMetatype* BaseSimulationView::getUniqueArticulationMetatype(const ArticulationMetatype& metatype)
{
    auto it = mSimData->mUniqueTypes.insert(metatype).first;
    return &(*it);
}

bool BaseSimulationView::setGravity(const carb::Float3& gravity)
{
    if (!g_physxPrivate)
    {
        CARB_LOG_ERROR("%s: Failed to aquire PhysX private interface", __FUNCTION__);
        return false;
    }

    PxScene* scene = g_physxPrivate->getPhysXScene();
    if (!scene)
    {
        CARB_LOG_ERROR("%s: Failed to get physics scene. Is the simulation active?", __FUNCTION__);
        return false;
    }

    scene->setGravity((const PxVec3&)gravity);

    return true;
}

bool BaseSimulationView::getGravity(carb::Float3& gravity)
{
    if (!g_physxPrivate)
    {
        CARB_LOG_ERROR("%s: Failed to aquire PhysX private interface", __FUNCTION__);
        return false;
    }

    PxScene* scene = g_physxPrivate->getPhysXScene();
    if (!scene)
    {
        CARB_LOG_ERROR("%s: Failed to get physics scene. Is the simulation active?", __FUNCTION__);
        return false;
    }

    PxVec3 physxGravity = scene->getGravity();
    gravity.x = physxGravity.x;
    gravity.y = physxGravity.y;
    gravity.z = physxGravity.z;

    return true;
}

bool BaseSimulationView::check() const
{
    bool result = true;

    for (auto artiView : mArtiViews)
    {
        if (!artiView->check())
        {
            result = false;
        }
    }

    for (auto rbView : mRbViews)
    {
        if (!rbView->check())
        {
            result = false;
        }
    }

    for (auto vdbView : mVolumeDeformableBodyViews)
    {
        if (!vdbView->check())
        {
            result = false;
        }
    }

    for (auto sdbView : mSurfaceDeformableBodyViews)
    {
        if (!sdbView->check())
        {
            result = false;
        }
    }

    for (auto dMaterialView : mDeformableMaterialViews)
    {
        if (!dMaterialView->check())
        {
            result = false;
        }
    }

    for (auto rcView : mRcViews)
    {
        if (!rcView->check())
        {
            result = false;
        }
    }

    return result;
}

void BaseSimulationView::release(bool recursive)
{
    if (recursive)
    {
        for (auto artiView : mArtiViews)
        {
            artiView->release();
        }

        for (auto rbView : mRbViews)
        {
            rbView->release();
        }

        for (auto vdbView : mVolumeDeformableBodyViews)
        {
            vdbView->release();
        }

        for (auto sdbView : mSurfaceDeformableBodyViews)
        {
            sdbView->release();
        }

        for (auto dMaterialView : mDeformableMaterialViews)
        {
            dMaterialView->release();
        }

        for (auto rcView : mRcViews)
        {
            rcView->release();
        }

    }
    delete this;
}

void BaseSimulationView::_onChildRelease(const BaseSdfShapeView* sdfView)
{
    auto it = std::find(mSDFViews.begin(), mSDFViews.end(), sdfView);
    if (it != mSDFViews.end())
    {
        mSDFViews.erase(it);
    }
}


void BaseSimulationView::_onChildRelease(const BaseArticulationView* artiView)
{
    auto it = std::find(mArtiViews.begin(), mArtiViews.end(), artiView);
    if (it != mArtiViews.end())
    {
        mArtiViews.erase(it);
    }
}

void BaseSimulationView::_onChildRelease(const BaseRigidBodyView* rbView)
{
    auto it = std::find(mRbViews.begin(), mRbViews.end(), rbView);
    if (it != mRbViews.end())
    {
        mRbViews.erase(it);
    }
}

void BaseSimulationView::_onChildRelease(const BaseVolumeDeformableBodyView* deformableView)
{
    auto it = std::find(mVolumeDeformableBodyViews.begin(), mVolumeDeformableBodyViews.end(), deformableView);
    if (it != mVolumeDeformableBodyViews.end())
    {
        mVolumeDeformableBodyViews.erase(it);
    }
}

void BaseSimulationView::_onChildRelease(const BaseSurfaceDeformableBodyView* deformableView)
{
    auto it = std::find(mSurfaceDeformableBodyViews.begin(), mSurfaceDeformableBodyViews.end(), deformableView);
    if (it != mSurfaceDeformableBodyViews.end())
    {
        mSurfaceDeformableBodyViews.erase(it);
    }
}

void BaseSimulationView::_onChildRelease(const BaseDeformableMaterialView* deformableView)
{
    auto it = std::find(mDeformableMaterialViews.begin(), mDeformableMaterialViews.end(), deformableView);
    if (it != mDeformableMaterialViews.end())
    {
        mDeformableMaterialViews.erase(it);
    }
}

void BaseSimulationView::_onChildRelease(const BaseRigidContactView* rcView)
{
    auto it = std::find(mRcViews.begin(), mRcViews.end(), rcView);
    if (it != mRcViews.end())
    {
        mRcViews.erase(it);
    }
}

#define COLLECT_STEP_TIMINGS 0

void BaseSimulationView::step(float dt)
{
    if (!g_physxPrivate)
    {
        CARB_LOG_ERROR("%s: Failed to aquire PhysX private interface", __FUNCTION__);
        return;
    }

    PxScene* scene = g_physxPrivate->getPhysXScene();
    if (!scene)
    {
        CARB_LOG_ERROR("%s: Failed to get physics scene. Is the simulation active?", __FUNCTION__);
        return;
    }

#if COLLECT_STEP_TIMINGS
    using Clock = std::chrono::high_resolution_clock;
    using TimePoint = std::chrono::time_point<Clock>;

    static int stepno = 1;
    static double tsum = 0.0f;

    TimePoint t1 = Clock::now();
#endif

    // step the physx scene directly
    scene->simulate(dt);
    scene->fetchResults(true);

#if COLLECT_STEP_TIMINGS
    TimePoint t2 = Clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() * 0.001;
    tsum += duration;
    ++stepno;

    if (stepno == 600)
    {
        printf("Average step time: %.3f ms\n", tsum / stepno);
    }
#endif

    if (auto* backend = GetSimulationBackend())
    {
        backend->incrementStepCount();
    }
}

PxMaterial* BaseSimulationView::createSharedMaterial(float staticFriction,
                                                     float dynamicFriction,
                                                     float restitution,
                                                     float damping,
                                                     PxCombineMode::Enum frictionCombineMode,
                                                     PxCombineMode::Enum restitutionCombineMode,
                                                     PxCombineMode::Enum dampingCombineMode)
{
    if (static_cast<int>(frictionCombineMode) >= static_cast<int>(PxCombineMode::Enum::eN_VALUES))
    {
        CARB_LOG_WARN("frictionCombineMode mode is out of range. Using PxCombineMode::eAVERAGE instead.");
        frictionCombineMode = PxCombineMode::eAVERAGE;
    }
    if (static_cast<int>(restitutionCombineMode) >= static_cast<int>(PxCombineMode::Enum::eN_VALUES))
    {
        CARB_LOG_WARN("restitutionCombineMode mode is out of range. Using PxCombineMode::eAVERAGE instead.");
        restitutionCombineMode = PxCombineMode::eAVERAGE;
    }
    if (static_cast<int>(dampingCombineMode) >= static_cast<int>(PxCombineMode::Enum::eN_VALUES))
    {
        CARB_LOG_WARN("dampingCombineMode mode is out of range. Using PxCombineMode::eAVERAGE instead.");
        dampingCombineMode = PxCombineMode::eAVERAGE;
    }

    const std::string key = makeMaterialKey(staticFriction, dynamicFriction, restitution, damping,
                                            frictionCombineMode, restitutionCombineMode, dampingCombineMode);

    std::unordered_map<std::string, PxMaterial*>::const_iterator got = mMaterials.find(key);

    PxMaterial* material = nullptr;

    // Exact material does not exist. Assign parameters to existing or make new one
    if (got == mMaterials.end())
    {
        // Find a material not in use by a shape
        if (mUnusedMaterials.size() > 0)
        {
            material = *mUnusedMaterials.begin();
            material->setStaticFriction(staticFriction);
            material->setDynamicFriction(dynamicFriction);
            material->setRestitution(restitution);
            material->setDamping(damping);

            mUnusedMaterials.erase(mUnusedMaterials.begin());
        }
        else
        {
            if (g_physxPrivate)
            {
                PxPhysics& physics = g_physxPrivate->getPhysXScene()->getPhysics();
                material = physics.createMaterial(staticFriction, dynamicFriction, restitution);
                material->setDamping(damping);
            }
        }

        if (material) {
            // Assign the new values to the material
            material->setFrictionCombineMode(frictionCombineMode);
            material->setRestitutionCombineMode(restitutionCombineMode);
            material->setDampingCombineMode(dampingCombineMode);
            mMaterials[key] = material;
            mMaterialKeys[material] = key;
        }
    }
    else
    {
        material = got->second;
    }

    if (mMaterialsRefCount.find(material) == mMaterialsRefCount.end())
    {
        mMaterialsRefCount[material] = 1;
    }
    else
    {
        mMaterialsRefCount[material] += 1;
    }

    return material;
}

std::string BaseSimulationView::makeMaterialKey(float staticFriction,
                                                float dynamicFriction,
                                                float restitution,
                                                float damping,
                                                PxCombineMode::Enum frictionCombineMode,
                                                PxCombineMode::Enum restitutionCombineMode,
                                                PxCombineMode::Enum dampingCombineMode)
{
    // %.9g round-trips an IEEE-754 single-precision value exactly, so distinct
    // float32 inputs always produce distinct keys (unlike "%.6f", which
    // collapsed values differing by less than 1e-6 onto the same key).
    std::string key;
    char keybuffer[100];
    snprintf(keybuffer, 100, "%.9g", staticFriction);
    key += std::string(keybuffer) + "_";
    snprintf(keybuffer, 100, "%.9g", dynamicFriction);
    key += std::string(keybuffer) + "_";
    snprintf(keybuffer, 100, "%.9g", restitution);
    key += std::string(keybuffer) + "_";
    snprintf(keybuffer, 100, "%.9g", damping);
    key += std::string(keybuffer) + "_";
    snprintf(keybuffer, 100, "%1d", static_cast<uint8_t>(frictionCombineMode));
    key += std::string(keybuffer) + "_";
    snprintf(keybuffer, 100, "%1d", static_cast<uint8_t>(restitutionCombineMode));
    key += std::string(keybuffer) + "_";
    snprintf(keybuffer, 100, "%1d", static_cast<uint8_t>(dampingCombineMode));
    key += std::string(keybuffer);
    return key;
}

void BaseSimulationView::releaseSharedMaterial(PxMaterial* material)
{
    auto refIt = mMaterialsRefCount.find(material);
    if (refIt == mMaterialsRefCount.end())
    {
        // Not a pool-owned material (e.g. the shape's original authored
        // material); nothing to reference-count or recycle.
        return;
    }

    if (--refIt->second > 0)
    {
        return;
    }

    // Last reference released. Drop the pool entry using the exact key the
    // material was stored under (recorded in mMaterialKeys at insertion), rather
    // than reconstructing a key from the material's current properties. The old
    // reconstruction left the map entry dangling whenever the rebuilt key did
    // not match the one used at insertion (e.g. the rigid-body path rebuilt only
    // 3 of the 7 key components), so a later request for the same tuple returned
    // this recycled material with foreign property values (NVBugs 6489465).
    auto keyIt = mMaterialKeys.find(material);
    if (keyIt != mMaterialKeys.end())
    {
        mMaterials.erase(keyIt->second);
        mMaterialKeys.erase(keyIt);
    }
    else
    {
        // Defensive fallback: a pooled material should always have a recorded
        // key, but if the reverse index is ever out of sync, remove the entry by
        // pointer so we never leave a dangling map entry behind.
        for (auto it = mMaterials.begin(); it != mMaterials.end();)
        {
            if (it->second == material)
            {
                it = mMaterials.erase(it);
            }
            else
            {
                ++it;
            }
        }
    }

    mMaterialsRefCount.erase(refIt);
    mUnusedMaterials.insert(material);
}

bool BaseSimulationView::hasRigidBody(PxRigidBody* body) const
{
    if (rigidBodies.find(body) != rigidBodies.end())
        return true;
    else
        return false;
}

bool BaseSimulationView::hasArticulation(PxArticulationReducedCoordinate* arti) const
{
    if (articulations.find(arti) != articulations.end())
        return true;
    else
        return false;
}

bool BaseSimulationView::hasLink(PxArticulationLink* link) const
{
    if (links.find(link) != links.end())
        return true;
    else
        return false;
}

bool BaseSimulationView::hasShape(PxShape* shape) const
{
    if (shapes.find(shape) != shapes.end())
        return true;
    else
        return false;
}

bool BaseSimulationView::hasDeformableBody(PxDeformableBody* body) const
{
    if (deformableBodies.find(body) != deformableBodies.end())
        return true;
    else
        return false;
}

bool BaseSimulationView::hasfixedTendon(PxArticulationFixedTendon* tendon) const
{
    if (fixedTendons.find(tendon) != fixedTendons.end())
        return true;
    else
        return false;
}
bool BaseSimulationView::hasSpatialTendon(PxArticulationSpatialTendon* tendon) const
{
    if (spatialTendons.find(tendon) != spatialTendons.end())
        return true;
    else
        return false;
}

}
}
}
