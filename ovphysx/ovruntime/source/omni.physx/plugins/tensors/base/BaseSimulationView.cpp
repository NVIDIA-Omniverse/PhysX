// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-REPLICATE-001
 * @covers AC-7
 *
 * @implements REQ-READ-CORE-001
 * @covers AC-3
 *
 * @implements REQ-TENSOR-VIEW-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-10
 *
 * @implements REQ-TENSOR-PATH-001
 * @covers AC-2
 *
 * @implements REQ-TENSOR-IDENTITY-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7
 *
 * @implements REQ-TENSOR-OBJECTTYPE-001
 * @covers AC-2 AC-3 AC-4
 *
 * @implements REQ-TENSOR-ATTACH-001
 * @covers AC-1 AC-2
 *
 * @implements REQ-TENSOR-DIAGNOSTICS-001
 * @covers AC-1 AC-2 AC-3
 *
 * @implements REQ-PUBLICAPI-001
 * @covers AC-5 AC-34 AC-35 AC-36 AC-37 AC-38 AC-40 AC-47 AC-48 AC-49
 */

#include "tensors/base/BaseSimulationView.h"

#include "tensors/base/BasePointInstancerView.h"
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
#include "utils/PrimPathGrammar.h"
#include "PhysXTools.h"

#include "OmniPhysX.h"
#include <omni/physx/PhysXRuntime.h>
#include "internal/InternalScene.h"
#include "ObjectDataQuery.h"

#include <algorithm>
#include <chrono>
#include <functional>
#include <limits>
#include <set>
#include <unordered_map>
#include <unordered_set>

#include <carb/logging/Log.h>
#include <carb/settings/ISettings.h>
#include <private/omni/physx/IPhysxPrivate.h>

#include <PxPhysicsAPI.h>

#include <omni/physx/IPhysxSimulation.h>
#include <omni/physx/IPhysxJoint.h>
#include <omni/physx/IPhysxReplicator.h>
#include <omni/physx/PhysxTokens.h>


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
//
// ADR-0019 increment 8 note: this pass stays separate from the IPhysicsSource-routed
// PathPatternMatcher::findMatchingObjectKeys, rather than being retired in favor of it.
// PrimHierarchyStorage is the runtime's OWN path<->object registry, populated
// independently of any IPhysicsSource -- a PhysX-replicator clone (IPhysxReplicator /
// physxSimulationCloneEnvironments) is registered here but is never authored as a USD
// prim nor an ovstage row, so neither UsdSource nor OvstageSource ever sees it (proven
// by TestTensorReplicatedMatcher.cpp's `CHECK(!stage->GetPrimAtPath(...).IsValid())`
// assertions on clone paths, for BOTH backend templates -- OvstageReplicator included,
// since its harness keeps a resident backing stage via OvstageAttach::usdStageId, so
// even a "collapse to one IPhysicsSource-routed call" would not have been masked by
// that template arm). Retiring this pass would silently stop replicated clones with no
// authored source object from ever matching a wildcard tensor-view pattern.
// Last "/"-delimited component of a plain PrimHierarchyStorage path string --
// the std::string analogue of SdfPath::GetName(), for the storage keys below
// (always canonical absolute paths, never property/variant-selection paths).
std::string storagePathName(const std::string& p)
{
    const size_t lastSlash = p.find_last_of('/');
    return lastSlash == std::string::npos ? p : p.substr(lastSlash + 1);
}

void findMatchingKeysInternalDb(const usdparser::AttachedStage* attachedStage,
                                const std::string& pattern_,
                                bool recursiveLeafPatternMatch,
                                std::vector<omni::physics::parse::ObjectKey>& keysRet)
{
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
    // Storage keys are always already-canonical path strings, so pattern_ is canonical
    // whenever it would match. keyFor() below runs only after storage.find has confirmed an
    // exact match, so unlike getObjectType it never mints a key from an unvalidated string.
    if (looksLikePathString(pattern_) && storage.find(pattern_) != storage.end())
    {
        // Same invalid-key drop as the traversal path below.
        const omni::physics::parse::ObjectKey key = attachedStage->keyFor(pattern_);
        if (key.valid())
        {
            keysRet.push_back(key);
        }
        return;
    }

    // children-of helper: the pseudo-root (empty path) maps to the stored top-level
    // prims (entries with an empty parent). Storage children come from a lexicographic
    // std::set; the ObjectId sort at the tail restores numeric clone order, so the
    // emission order here is not significant.
    auto childrenOf = [&](const std::string& p, std::vector<std::string>& out)
    {
        if (p.empty())
        {
            for (const auto& kv : storage)
            {
                if (kv.second.parent.empty())
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
    std::function<void(const std::string&, std::vector<std::string>&)> collectSelfAndDescendants =
        [&](const std::string& p, std::vector<std::string>& out)
    {
        if (!p.empty())
        {
            out.push_back(p);
        }
        std::vector<std::string> ch;
        childrenOf(p, ch);
        for (const std::string& c : ch)
        {
            collectSelfAndDescendants(c, out);
        }
    };
    // Leaf-recursive descent with same-name-on-path suppression -- the SdfPath
    // analogue of PathPatternMatcher::collectMatchingDescendants, so the USD and
    // internal-DB branches match at any depth identically.
    std::function<void(const std::string&, const GlobRegex&, std::unordered_set<std::string>&, std::vector<std::string>&)>
        collectMatchingDescendants =
            [&](const std::string& p, const GlobRegex& matcher, std::unordered_set<std::string>& onPath,
                std::vector<std::string>& out)
    {
        std::vector<std::string> ch;
        childrenOf(p, ch);
        for (const std::string& c : ch)
        {
            const std::string name = storagePathName(c);
            if (matcher.match(name) && onPath.insert(name).second)
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

    const std::string pattern = trimSlashes(pattern_);
    const std::vector<std::string> tokens = splitPatternRespectingGroups(pattern);
    if (tokens.empty())
    {
        return;
    }

    std::vector<std::string> roots;
    std::vector<std::string> matches;
    roots.push_back(std::string()); // pseudo-root (empty path)

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
            for (const std::string& r : roots)
            {
                collectSelfAndDescendants(r, matches);
            }
        }
        else if (isLeaf && recursiveLeafPatternMatch && !isBareWildcard && !patternHasRecursiveDescent)
        {
            // Leaf-recursive: named/glob leaf searched at any depth beneath the
            // strictly-matched ancestor chain (with same-name suppression).
            const GlobRegex matcher(tokens[i]);
            for (const std::string& r : roots)
            {
                std::unordered_set<std::string> onPath;
                collectMatchingDescendants(r, matcher, onPath, matches);
            }
        }
        else
        {
            const GlobRegex matcher(tokens[i]);
            for (const std::string& r : roots)
            {
                std::vector<std::string> ch;
                childrenOf(r, ch);
                for (const std::string& c : ch)
                {
                    if (matcher.match(storagePathName(c)))
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

    // Resolve each path once before sorting. The comparator runs O(N log N)
    // times, while keyFor() crosses into the source's intern table. The
    // string_view overload is byte-identical to the USD source's SdfPath keying
    // and keeps this path pxr-free.
    struct OrderedMatch
    {
        const std::string* path;
        omni::physics::parse::ObjectKey key;
        usdparser::ObjectId creationId;
    };
    std::vector<OrderedMatch> orderedMatches;
    orderedMatches.reserve(matches.size());
    for (const std::string& path : matches)
    {
        const omni::physics::parse::ObjectKey key = attachedStage->keyFor(path);
        // keyFor is total but not always resolving: with no source, or on a source whose
        // findByPath misses, it answers the invalid sentinel. Such a key names nothing --
        // every downstream lookup on it fails, they all dedup onto one another, and
        // textFor() resolves it to "", which as a subspace root prefix-matches every path
        // (setSubspaceRoots). Drop it here rather than emit an unusable match.
        if (!key.valid())
        {
            continue;
        }
        usdparser::ObjectId creationId = std::numeric_limits<usdparser::ObjectId>::max();
        const usdparser::ObjectIdMap* entries = objectDb->getEntries(key);
        if (entries && !entries->empty())
        {
            creationId = entries->begin()->second;
            for (usdparser::ObjectIdMap::const_reference entry : *entries)
            {
                creationId = std::min(creationId, entry.second);
            }
        }
        orderedMatches.push_back({ &path, key, creationId });
    }

    // Restore numeric clone order: storage children are lexicographic, and on a
    // replicated stage the numeric env order lives in ObjectId (clone index).
    std::stable_sort(orderedMatches.begin(), orderedMatches.end(),
                     [](const OrderedMatch& a, const OrderedMatch& b)
                     {
                         if (a.creationId != b.creationId)
                         {
                             return a.creationId < b.creationId;
                         }
                         return *a.path < *b.path;
                     });
    for (const OrderedMatch& match : orderedMatches)
    {
        keysRet.push_back(match.key);
    }
}

// Destroy every child view in `views`, emptying it.
//
// A child's destructor calls back into BaseSimulationView::_onChildRelease, which erases the child
// from this same vector -- so each child must be popped BEFORE it is destroyed. Erasing from under a
// range-for invalidates the iterator and its cached end. Popping first also makes the child's
// self-erase a harmless no-op, and terminates even if a child fails to unregister.
template <typename ViewT>
void releaseChildViews(std::vector<ViewT*>& views)
{
    while (!views.empty())
    {
        ViewT* view = views.back();
        views.pop_back();
        view->release();
    }
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

BaseSimulationView::BaseSimulationView(usdparser::AttachedStage* attachedStage,
                                       PxScene* scene,
                                       bool notifyWhenSimStopped)
    : mAttachedStage(attachedStage)
    , mScene(scene)
    , mSimData(std::make_shared<BaseSimulationData>())
{
    omni::physx::IPhysicsObjectChangeCallback callback;
    callback.objectDestructionNotifyFn = onPhysXObjectDeletedCallback;
    callback.allObjectsDestructionNotifyFn = onAllPhysXObjectDeletedCallback;
    // OMPE-106802: opt out of the simulation-stopped delivery gate. The scenes this view caches are released
    // while object change notifications are turned off for the shutdown ("do not send these notifications when
    // the simulation is to end"), so with the default the callbacks above never fire on that path: the view
    // stays valid holding a freed PxScene and the next call through it dereferences the dead scene.
    callback.stopCallbackWhenSimStopped = false;
    callback.userData = this;
    callback.stopCallbackWhenSimStopped = !notifyWhenSimStopped;
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

        // Drop the attach handle. UsdLoad owns the AttachedStage and destroys it at
        // detach, and invalidate() is how the view learns that is happening, so the
        // pointer must not outlive this call -- every lookup below guards on it.
        mAttachedStage = nullptr;
    }
}

void* BaseSimulationView::resolvePhysXPtr(const usdparser::AttachedStage* attachedStage,
                                          omni::physics::parse::ObjectKey key, PhysXType type)
{
    if (!attachedStage)
    {
        return nullptr;
    }
    return (void*)(getObjectDataOrID<ObjectDataQueryType::ePHYSX_PTR>(
        key, type, OmniPhysX::getInstance().getInternalPhysXDatabase(), *attachedStage));
}

usdparser::ObjectId BaseSimulationView::resolveObjectId(const usdparser::AttachedStage* attachedStage,
                                                         omni::physics::parse::ObjectKey key, PhysXType type)
{
    if (!attachedStage)
    {
        return usdparser::kInvalidObjectId;
    }
    return usdparser::ObjectId(getObjectDataOrID<ObjectDataQueryType::eOBJECT_ID>(
        key, type, OmniPhysX::getInstance().getInternalPhysXDatabase(), *attachedStage));
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

    for (auto instancerView : mPointInstancerViews)
    {
        instancerView->_onParentRelease();
    }
}

void BaseSimulationView::onPhysXObjectDeletedCallback(omni::physics::parse::ObjectKey key,
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
                    physx->objectKeyToPath(key));
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
                    physx->objectKeyToPath(key));
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
                    physx->objectKeyToPath(key));
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
                    physx->objectKeyToPath(key));
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
                    physx->objectKeyToPath(key));
                sim->invalidate();
            }
        }
        // The scene outranks every object above: the view is bound to it and the GPU path caches
        // it as a raw pointer, so losing it strands the view on freed memory even though none of
        // its bodies or shapes were touched.
        if (type == ePTScene)
        {
            PxScene* scene = static_cast<PxScene*>(physx->getPhysXPtrFast(objectId));
            if (sim->hasScene(scene))
            {
                CARB_LOG_WARN(
                    "physics scene '%s' was deleted while being used by a tensor view class. The physics.tensors simulationView was invalidated.",
                    physx->objectKeyToPath(key));
                sim->invalidate();
                sim->mScene = nullptr;
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
    if (!mAttachedStage)
    {
        CARB_LOG_ERROR("Cannot set subspace roots: the simulation view has no attached stage");
        return false;
    }
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

    std::vector<omni::physics::parse::ObjectKey> keys;
    findMatchingPaths(pattern, keys);

    for (unsigned i = 0; i < keys.size(); i++)
    {
        // A key that resolves to no path text would be stored under "", and an empty root
        // sorts first in mSubspaces and prefix-matches EVERY absolute path -- a silent
        // catch-all that reframes unrelated objects. Reject it, loudly: dropping a root is
        // visible in the log, capturing the whole stage is not.
        const std::string_view rootText = mAttachedStage->textViewFor(keys[i]);
        if (!keys[i].valid() || rootText.empty())
        {
            CARB_LOG_WARN("Subspace root matched by pattern '%s' does not resolve to a path; skipping it.", pattern);
            continue;
        }

        // Read the root's world transform through the source rather than off a UsdPrim,
        // so it resolves under a stageless attach. This drops the old UsdGeomXformable
        // gate: a matched prim that is not xformable now gets a subspace at its
        // inherited world origin instead of none. Subspace roots are environment roots
        // (Xforms) in practice, and the source answers for any prim it knows.
        const ::physx::PxMat44d localToWorld =
            internal::getWorldTransform(*mAttachedStage, keys[i], omni::physics::parse::ReadTime::defaultTime());
        const ::physx::PxVec3d tran = localToWorld.getPosition();

        Subspace subspace;
        subspace.origin = { float(tran[0]), float(tran[1]), float(tran[2]) };

        mSimData->mSubspaces[std::string(rootText)] = subspace;
    }

    return true;
}

// True when `path` is `prefix` itself or a descendant of it (path-prefix hierarchy
// check, the std::string-native equivalent of pxr::SdfPath::HasPrefix).
static bool hasPathPrefix(const std::string& path, const std::string& prefix)
{
    if (path.size() < prefix.size() || path.compare(0, prefix.size(), prefix) != 0)
    {
        return false;
    }
    return path.size() == prefix.size() || path[prefix.size()] == '/';
}

Subspace* BaseSimulationView::findSubspaceForPath(const std::string& path) const
{
    for (auto& entry : mSimData->mSubspaces)
    {
        // Second line of defence behind setSubspaceRoots' insertion guard: an empty root is
        // a prefix of every absolute path and sorts first, so it would win over every real
        // root. Never treat it as a match.
        if (entry.first.empty())
        {
            continue;
        }
        if (hasPathPrefix(path, entry.first))
        {
            return &entry.second;
        }
    }
    return nullptr;
}

void BaseSimulationView::findMatchingPaths(const std::string& pattern_, std::vector<omni::physics::parse::ObjectKey>& keysRet)
{
    if (!mAttachedStage)
    {
        return;
    }

    // The source-routed pass (PathPatternMatcher::findMatchingObjectKeys) answers
    // uniformly for a USD-backed or ovstage-backed attach, with or without a resident
    // USD stage: IPhysicsSource abstracts over both (ADR-0019 decision 2), so this no
    // longer needs the old raw-UsdStage `if (mStage)` gate -- a stageless attach used
    // to skip straight to the internal-DB pass below, silently answering nothing for
    // any object the internal DB does not narrow to, and a resident-but-ovstage-backed
    // attach used to bypass OvstageSource entirely in favor of the raw cached stage.
    // Authoring order (when the backend publishes one -- REQ-PARSE-CORE-003 AC-6:
    // ovstage does not) still comes first here, exactly as it did through the old
    // stage-conditional branch.
    const omni::physics::parse::IPhysicsSource* src = mAttachedStage->getSource();
    if (src)
    {
        findMatchingObjectKeys(*src, pattern_, isRecursiveLeafPatternMatchEnabled(), keysRet);
    }

    // The internal path<->object DB is a SEPARATE registry from any IPhysicsSource: it
    // is the only index that sees physics-only objects (PhysX-replicator clones are
    // registered there with no authored source object at all -- see
    // findMatchingKeysInternalDb's header comment). Append ONLY the matches the source
    // pass missed, in physics-creation (numeric clone) order. On a non-replicated
    // scene every internal match already has a source object and is therefore already
    // present, so nothing is appended and the source-pass ordering above is preserved
    // unchanged.
    std::vector<omni::physics::parse::ObjectKey> internalMatches;
    findMatchingKeysInternalDb(mAttachedStage, pattern_, isRecursiveLeafPatternMatchEnabled(), internalMatches);
    if (!internalMatches.empty())
    {
        std::unordered_set<omni::physics::parse::ObjectKey, omni::physics::parse::ObjectKey::Hash> seen(
            keysRet.begin(), keysRet.end());
        for (const omni::physics::parse::ObjectKey& k : internalMatches)
        {
            if (seen.insert(k).second)
            {
                keysRet.push_back(k);
            }
        }
    }
}

void BaseSimulationView::findMatchingPathsBatch(const std::vector<std::string>& patterns,
                                                std::vector<std::vector<omni::physics::parse::ObjectKey>>& keysRet)
{
    keysRet.clear();
    keysRet.resize(patterns.size());
    if (!mAttachedStage)
    {
        return;
    }

    // Source-routed pass, batched across the whole pattern list -- see
    // findMatchingPaths's comment for why this runs unconditionally against
    // mAttachedStage->getSource() rather than being gated on a raw UsdStage.
    const omni::physics::parse::IPhysicsSource* src = mAttachedStage->getSource();
    if (src)
    {
        findMatchingObjectKeysBatch(*src, patterns, isRecursiveLeafPatternMatchEnabled(), keysRet);
    }

    // Internal-DB pass, same shape as findMatchingPaths: appended per-pattern
    // (it is not the source of the O(N) round-trip cost this batching exists
    // for, so it is not itself batched -- see findMatchingKeysInternalDb).
    for (size_t i = 0; i < patterns.size(); ++i)
    {
        std::vector<omni::physics::parse::ObjectKey> internalMatches;
        findMatchingKeysInternalDb(mAttachedStage, patterns[i], isRecursiveLeafPatternMatchEnabled(), internalMatches);
        if (internalMatches.empty())
        {
            continue;
        }
        std::unordered_set<omni::physics::parse::ObjectKey, omni::physics::parse::ObjectKey::Hash> seen(
            keysRet[i].begin(), keysRet[i].end());
        for (const omni::physics::parse::ObjectKey& k : internalMatches)
        {
            if (seen.insert(k).second)
            {
                keysRet[i].push_back(k);
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
    if (!mAttachedStage)
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

    std::vector<omni::physics::parse::ObjectKey> keys;
    findMatchingPaths(pattern, keys);

    // Pattern matching is intentionally type-agnostic. Wrong-type candidates are normal filtering;
    // processArticulationEntries reports when filtering adds no result for a pattern.
    // Dedup by PxArticulation pointer using a set owned by the caller
    // (processArticulationEntries), so overlapping patterns that resolve
    // to the same articulation don't inflate the view. The set also
    // absorbs within-pattern duplicates from recursive leaf matching and
    // '**' expansion hitting several prims of the same articulation
    // (the root Xform plus any link), which getArticulationAtPath
    // resolves to the same articulation.
    for (unsigned i = 0; i < keys.size(); i++)
    {
        ArticulationEntry entry;
        if (getArticulationAtPath(keys[i], entry))
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
    if (!mAttachedStage)
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

    std::vector<omni::physics::parse::ObjectKey> keys;
    findMatchingPaths(pattern, keys);

    // Pattern matching is intentionally type-agnostic. Wrong-type candidates are normal filtering;
    // processRigidBodyEntries reports when filtering adds no result for a pattern.
    // Dedup by body pointer using a set owned by the caller
    // (processRigidBodyEntries), so overlapping patterns that resolve to
    // the same PxRigidBody don't inflate the view. The set also absorbs
    // within-pattern duplicates from a '**' expansion hitting both an
    // articulation-root prim and its root link prim, which
    // getRigidBodyAtPath resolves to the same PxRigidBody.
    for (unsigned i = 0; i < keys.size(); i++)
    {
        RigidBodyEntry entry;
        if (getRigidBodyAtPath(keys[i], entry))
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
    if (!mAttachedStage)
    {
        return;
    }

    // we only support absolute paths atm
    if (pattern[0] != '/')
    {
        CARB_LOG_ERROR("Pattern must be an absolute USD path, got '%s'\n", pattern.c_str());
        return;
    }

    std::vector<omni::physics::parse::ObjectKey> keys;
    findMatchingPaths(pattern, keys);

    for (unsigned i = 0; i < keys.size(); i++)
    {
        DeformableBodyEntry entry;
        if (getVolumeDeformableBodyAtPath(keys[i], entry))
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
    if (!mAttachedStage)
    {
        return;
    }

    // we only support absolute paths atm
    if (pattern[0] != '/')
    {
        CARB_LOG_ERROR("Pattern must be an absolute USD path, got '%s'\n", pattern.c_str());
        return;
    }

    std::vector<omni::physics::parse::ObjectKey> keys;
    findMatchingPaths(pattern, keys);

    for (unsigned i = 0; i < keys.size(); i++)
    {
        DeformableBodyEntry entry;
        if (getSurfaceDeformableBodyAtPath(keys[i], entry))
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
    if (!mAttachedStage)
    {
        return;
    }

    // we only support absolute paths atm
    if (pattern[0] != '/')
    {
        CARB_LOG_ERROR("Pattern must be an absolute USD path, got '%s'\n", pattern.c_str());
        return;
    }

    std::vector<omni::physics::parse::ObjectKey> keys;
    findMatchingPaths(pattern, keys);

    for (unsigned i = 0; i < keys.size(); i++)
    {
        DeformableMaterialEntry entry;
        if (getDeformableMaterialAtPath(keys[i], entry))
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

    // A filter pattern's matches depend on the attached source, not on the
    // sensor pattern it accompanies. Resolve each distinct filter pattern
    // once for the whole view. This matters when every sensor has the same
    // literal filter list: resolving it independently for every sensor turns
    // one batch plus one internal-database pass into O(sensors * filters)
    // identity lookups.
    size_t totalFilterPatternCount = 0;
    for (const auto& sensorFilterPatterns : filterPatterns)
    {
        totalFilterPatternCount += sensorFilterPatterns.size();
    }

    std::vector<std::string> uniqueFilterPatterns;
    uniqueFilterPatterns.reserve(totalFilterPatternCount);
    std::unordered_map<std::string, size_t> uniqueFilterPatternIndices;
    uniqueFilterPatternIndices.reserve(totalFilterPatternCount);
    std::vector<std::vector<size_t>> filterPatternIndices(filterPatterns.size());

    for (size_t sensorIndex = 0; sensorIndex < filterPatterns.size(); ++sensorIndex)
    {
        auto& indices = filterPatternIndices[sensorIndex];
        indices.reserve(filterPatterns[sensorIndex].size());
        for (const std::string& filterPattern : filterPatterns[sensorIndex])
        {
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

            const size_t nextIndex = uniqueFilterPatterns.size();
            const auto insertion = uniqueFilterPatternIndices.emplace(filterPattern, nextIndex);
            if (insertion.second)
            {
                uniqueFilterPatterns.push_back(filterPattern);
            }
            indices.push_back(insertion.first->second);
        }
    }

    std::vector<std::vector<omni::physics::parse::ObjectKey>> resolvedFilterKeys;
    if (!uniqueFilterPatterns.empty())
    {
        findMatchingPathsBatch(uniqueFilterPatterns, resolvedFilterKeys);
    }

    // Bridge canonical matches to their display and legacy contact-report
    // identities once per distinct pattern result. The same filter list is
    // commonly reused by many sensors, and pathFor()/textFor() may cross the
    // source interner; repeating that bridge per sensor dominated view setup.
    std::vector<std::vector<std::string>> resolvedFilterPaths(resolvedFilterKeys.size());
    std::vector<std::vector<uint64_t>> resolvedFilterLegacyIds(resolvedFilterKeys.size());
    std::vector<std::vector<uint8_t>> resolvedFilterLegacyIdValid(resolvedFilterKeys.size());
    for (size_t patternIndex = 0; patternIndex < resolvedFilterKeys.size(); ++patternIndex)
    {
        const auto& keys = resolvedFilterKeys[patternIndex];
        auto& paths = resolvedFilterPaths[patternIndex];
        auto& legacyIds = resolvedFilterLegacyIds[patternIndex];
        auto& legacyIdValid = resolvedFilterLegacyIdValid[patternIndex];
        paths.reserve(keys.size());
        legacyIds.reserve(keys.size());
        legacyIdValid.reserve(keys.size());

        for (const omni::physics::parse::ObjectKey key : keys)
        {
            // legacyIds now uses the ObjectKey.handle encoding in both configs (see
            // CommonTypes.h's asInt(ObjectKey)); no backend-specific branch needed.
            const bool valid = key.valid();
            paths.push_back(mAttachedStage->textFor(key));
            legacyIdValid.push_back(valid ? 1 : 0);
            legacyIds.push_back(valid ? asInt(key) : 0);
        }
    }

    filterPatternSize = 0;
    // Dedup sensor keys across the full pattern list: overlapping
    // patterns (or the new recursive / '**' expansions) can surface the
    // same object more than once, and findMatchingRigidContactSensors
    // pairs sensors with filters positionally — so leaving duplicates
    // would silently pair a sensor with the wrong filter.
    std::unordered_set<omni::physics::parse::ObjectKey, omni::physics::parse::ObjectKey::Hash> seenSensorKeys;
    for (size_t i = 0; i < patterns.size(); ++i)
    {
        size_t currentSize = entries.size();
        findMatchingRigidContactSensors(patterns[i], filterPatterns[i], filterPatternIndices[i], resolvedFilterKeys,
                                        resolvedFilterPaths, resolvedFilterLegacyIds, resolvedFilterLegacyIdValid,
                                        entries, seenSensorKeys);
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
                                                         const std::vector<size_t>& filterPatternIndices,
                                                         const std::vector<std::vector<omni::physics::parse::ObjectKey>>& resolvedFilterKeys,
                                                         const std::vector<std::vector<std::string>>& resolvedFilterPaths,
                                                         const std::vector<std::vector<uint64_t>>& resolvedFilterLegacyIds,
                                                         const std::vector<std::vector<uint8_t>>& resolvedFilterLegacyIdValid,
                                                         std::vector<RigidContactSensorEntry>& entriesRet,
                                                         std::unordered_set<omni::physics::parse::ObjectKey,
                                                                            omni::physics::parse::ObjectKey::Hash>& seenSensorKeys)
{
    if (!mAttachedStage)
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

    const uint32_t numFilterPatterns = uint32_t(filterPatterns.size());

    std::vector<omni::physics::parse::ObjectKey> keys;
    findMatchingPaths(pattern, keys);

    // Filter expansion below is positional: filterKeys[j][i] must line up
    // with keys[i], and the size-1 broadcast assumes keys.size() is the
    // intended sensor count. Duplicate keys — from overlapping patterns,
    // recursive leaf matching, or '**' expansions — break both invariants,
    // so drop duplicates (preserving order) before we build filterKeys.
    auto deduppedEnd = std::remove_if(keys.begin(), keys.end(),
        [&seenSensorKeys](omni::physics::parse::ObjectKey k) { return !seenSensorKeys.insert(k).second; });
    keys.erase(deduppedEnd, keys.end());

    for (uint32_t i = 0; i < numFilterPatterns; i++)
    {
        const auto& matchingFilterKeys = resolvedFilterKeys[filterPatternIndices[i]];
        // Special case: if only a single match is found, then assume all sensors should report contacts with a single
        // object, like a common ground plane.
        if (matchingFilterKeys.size() != 1 && matchingFilterKeys.size() != keys.size())
        {
            // Size mismatch is fatal too: a blank filter column would be
            // the same partial-disable footgun as a malformed pattern.
            // Fail the call so the user notices.
            CARB_LOG_ERROR("Filter pattern '%s' did not match the correct number of entries (expected %u, found %u)",
                filterPatterns[i].c_str(), unsigned(keys.size()), unsigned(matchingFilterKeys.size()));
            return;
        }
    }

    for (unsigned i = 0; i < keys.size(); i++)
    {
        RigidContactSensorEntry entry;
        if (getRigidContactSensorAtPath(keys[i], entry))
        {
            entriesRet.push_back(entry);

            // Add contact filter mappings, bridging each filter key back to a path
            // once. filterPaths is a source-native display string; filterIndexMap
            // stays legacy asInt(ObjectKey)-keyed to match ContactEventHeader (see
            // CommonTypes.h's RigidContactSensorEntry comment).
            auto& e = entriesRet.back();
            e.filterPaths.reserve(numFilterPatterns);
            e.filterKeys.reserve(numFilterPatterns);
            e.filterIndexMap.reserve(numFilterPatterns);
            for (uint32_t j = 0; j < numFilterPatterns; j++)
            {
                const auto& matchingFilterKeys = resolvedFilterKeys[filterPatternIndices[j]];
                const size_t matchIndex = matchingFilterKeys.size() == 1 ? 0 : i;
                const omni::physics::parse::ObjectKey filterKey = matchingFilterKeys[matchIndex];
                e.filterKeys.push_back(filterKey);
                const size_t resolvedPatternIndex = filterPatternIndices[j];
                e.filterPaths.push_back(resolvedFilterPaths[resolvedPatternIndex][matchIndex]);
                if (resolvedFilterLegacyIdValid[resolvedPatternIndex][matchIndex])
                {
                    // Legacy encoding, must match ContactEventHeader -- see CommonTypes.h.
                    const uint64_t pathId = resolvedFilterLegacyIds[resolvedPatternIndex][matchIndex];
                    e.filterIndexMap[pathId] = j;
                }
            }
        }
    }
}

void BaseSimulationView::findMatchingSDFShapes(const std::string& pattern, std::vector<SdfShapeEntry>& entriesRet, uint32_t numSamplePoints)
{
    if (!mAttachedStage)
    {
        return;
    }

    // we only support absolute paths atm
    if (pattern[0] != '/')
    {
        CARB_LOG_ERROR("Pattern must be an absolute USD path, got '%s'\n", pattern.c_str());
        return;
    }

    std::vector<omni::physics::parse::ObjectKey> keys;
    findMatchingPaths(pattern, keys);

    for (unsigned i = 0; i < keys.size(); i++)
    {
        SdfShapeEntry entry;
        if (getSDFShapeAtPath(keys[i], entry))
        {
            entry.numSamplePoints = numSamplePoints;
            entriesRet.push_back(entry);
        }
    }
}

// Stand-in for SdfPath::IsValidPathString's grammar gate: reject a malformed path string (e.g.
// "/World/123abc") before it reaches AttachedStage::keyFor()'s unguarded SdfPath construction,
// which would emit a TfDiagnosticMgr warning. A rejection answers eInvalid without a lookup, so
// the grammar must stay a SUPERSET of every absolute prim path a source can hold (UTF-8 prim
// names included) -- see utils/PrimPathGrammar.h. One predicate for every source on purpose: an
// ovstage source has no SdfPath to warn, but a superset gate costs it nothing observable and a
// per-source gate would be a per-source answer. The bare root "/" is accepted (it is a valid
// absolute path; the lookup simply finds no object).
static bool looksLikeValidSdfPathString(std::string_view path)
{
    return omni::physx::looksLikeAbsolutePrimPath(path, /*allowRoot=*/true);
}

ObjectType BaseSimulationView::getObjectType(const char* path)
{
    if (!g_physx)
    {
        return ObjectType::eInvalid;
    }
    // Stricter than findMatchingKeysInternalDb's looksLikePathString: keyFor(path) below is
    // not gated by a prior storage.find match, so a path-shaped but invalid string (e.g.
    // "/World/123abc") would reach the USD arm's unguarded SdfPath construction and emit a
    // Tf diagnostic warning. Unlike that call site the gate DOES decide the return value --
    // a rejected path is never looked up -- so it logs rather than answering eInvalid mutely.
    if (!looksLikeValidSdfPathString(path ? path : ""))
    {
        CARB_LOG_WARN("getObjectType: '%s' is not a valid absolute prim path; reporting eInvalid without a lookup.",
                      path ? path : "");
        return ObjectType::eInvalid;
    }
    // "does this path name a real object" is answered by the source, so it also
    // answers under a stageless attach. Runtime clones with no authored prim are
    // covered by the physics-pointer lookups below, as before.
    const omni::physics::parse::IPhysicsSource* src = mAttachedStage ? mAttachedStage->getSource() : nullptr;
    // keyFor(std::string_view) forwards to keyFor(const SdfPath&) under a real USD
    // backend (byte-identical), so this is pxr-free without needing a fenced sibling.
    const omni::physics::parse::ObjectKey key =
        mAttachedStage ? mAttachedStage->keyFor(path) : omni::physics::parse::ObjectKey{};
    if (src && src->exists(key))
    {
        PxArticulationReducedCoordinate* arti = nullptr;
        // check if it's an articulation link
        PxArticulationLink* link = (PxArticulationLink*)resolvePhysXPtr(mAttachedStage, key, omni::physx::ePTLink);
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
        arti = (PxArticulationReducedCoordinate*)resolvePhysXPtr(mAttachedStage, key, omni::physx::ePTArticulation);
        if (arti)
        {
            return ObjectType::eArticulation;
        }

        // check if it's an articulation joint
        PxArticulationJointReducedCoordinate* linkJoint =
            (PxArticulationJointReducedCoordinate*)resolvePhysXPtr(mAttachedStage, key, omni::physx::ePTLinkJoint);
        if (linkJoint)
        {
            return ObjectType::eArticulationJoint;
        }

        // check if it's a maximal-coordinate (standalone) joint
        PxJoint* joint = static_cast<PxJoint*>(resolvePhysXPtr(mAttachedStage, key, omni::physx::ePTJoint));
        if (joint)
        {
            return ObjectType::eJoint;
        }

        // check if it's a plugin-registered custom joint (CustomPhysXJoint, not PxJoint)
        if (resolvePhysXPtr(mAttachedStage, key, omni::physx::ePTCustomJoint))
        {
            return ObjectType::eCustomJoint;
        }

        PxActor* actor = static_cast<PxActor*>(resolvePhysXPtr(mAttachedStage, key, omni::physx::ePTActor));
        if (actor)
        {
            // check if it's a rigid dynamic
            if (actor->getType() == PxActorType::eRIGID_DYNAMIC)
            {
                return ObjectType::eRigidBody;
            }
        }
    }
    return ObjectType::eInvalid;
}

bool BaseSimulationView::getArticulationAtPath(omni::physics::parse::ObjectKey key, ArticulationEntry& entryRet)
{
    if (!g_physx)
    {
        return false;
    }
    // check if it's an articulation
    PxArticulationReducedCoordinate* arti =
        (PxArticulationReducedCoordinate*)resolvePhysXPtr(mAttachedStage, key, omni::physx::ePTArticulation);
    if (arti)
    {
    }
    else
    {
        // check if it's an articulation link
        PxArticulationLink* link = (PxArticulationLink*)resolvePhysXPtr(mAttachedStage, key, omni::physx::ePTLink);
        if (link)
        {
            arti = &static_cast<PxArticulationReducedCoordinate&>(link->getArticulation());
        }
        else
        {
            // check if it's an articulation joint
            PxArticulationJointReducedCoordinate* joint =
                (PxArticulationJointReducedCoordinate*)resolvePhysXPtr(mAttachedStage, key, omni::physx::ePTLinkJoint);
            if (joint)
            {
                arti =
                    &static_cast<PxArticulationReducedCoordinate&>(joint->getChildArticulationLink().getArticulation());
            }
        }
    }

    if (!arti)
    {
        return false;
    }

    // The entry-building body is source-agnostic (it derives link/joint names, poses and DOF layout
    // from `arti` via the g_physx object DB), so it is shared with the stageless ovstage read path
    // that has only a PxArticulation*. `key` is passed as the entryRet.path fallback.
    return buildArticulationEntry(arti, key, entryRet);
}

bool BaseSimulationView::buildArticulationEntry(PxArticulationReducedCoordinate* arti,
                                                omni::physics::parse::ObjectKey fallbackKey,
                                                ArticulationEntry& entryRet)
{
    if (!arti || arti->getConcreteType() != PxConcreteType::eARTICULATION_REDUCED_COORDINATE)
    {
        CARB_LOG_WARN("buildArticulationEntry: not a reduced-coordinate articulation");
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
    // objectKeyToPath already returns the canonical path string, so no SdfPath round trip
    // is needed here -- pxr-free unconditionally.
    std::string canonicalPath;
    if (g_physx)
    {
        canonicalPath = g_physx->objectKeyToPath(g_physx->getObjectKeyForId(objectId));
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
        std::string linkName = storagePathName(g_physx->objectKeyToPath(g_physx->getObjectKeyForId(linkId)));

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
            const omni::physics::parse::ObjectKey jointKey = g_physx->getObjectKeyForId(jointId);
            // storagePathName/plain string are pxr-free and identical to SdfPath::GetName()/
            // GetString() for a plain canonical prim path, so this is unconditional.
            const std::string jointPath = g_physx->objectKeyToPath(jointKey);
            std::string jointName = storagePathName(jointPath);

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

            ArticulationMetatype::JointDesc jointDesc;
            jointDesc.name = jointName;
            const usdparser::ObjectId objectId = resolveObjectId(mAttachedStage, jointKey, ePTLinkJoint);
            omni::physx::JointStateData jointStateData;
            g_physxJoint->getJointStateData(objectId, &jointStateData);

            // Parsed joint-drive descriptor from the internal DB (carries the
            // DrivePerformanceEnvelope flag). We read the envelope flag from here
            // instead of the source so it resolves for runtime-replicated clones
            // whose joint prims are absent from the parse source.
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
            size_t linkId = reinterpret_cast<size_t>(parentLink.userData);
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
                    freeRotationAxes[i].raise(FreeD6RotationAxesFlag::Enum::eTWIST);

                    {
                        jointDesc.dofs.emplace_back(jointName + ":0", DofType::eRotation);
                        if (intJoint && intJoint->mJointDrives[PxArticulationAxis::eTWIST].isEnvelopeUsed)
                            isEnvelopeUsed = 1;
                    }
                    dofImpls.push_back({joint, PxArticulationAxis::eTWIST, isEnvelopeUsed });
                    isEnvelopeUsed = 0;
                }
                if (joint->getMotion(PxArticulationAxis::eSWING1) != PxArticulationMotion::eLOCKED)
                {
                    freeRotationAxes[i].raise(FreeD6RotationAxesFlag::Enum::eSWING1);
                    {
                        jointDesc.dofs.emplace_back(jointName + ":1", DofType::eRotation);
                        if (intJoint && intJoint->mJointDrives[PxArticulationAxis::eSWING1].isEnvelopeUsed)
                            isEnvelopeUsed = 1;
                    }
                    dofImpls.push_back({joint, PxArticulationAxis::eSWING1, isEnvelopeUsed });
                    isEnvelopeUsed = 0;
                }
                if (joint->getMotion(PxArticulationAxis::eSWING2) != PxArticulationMotion::eLOCKED)
                {
                    freeRotationAxes[i].raise(FreeD6RotationAxesFlag::Enum::eSWING2);
                    {
                        jointDesc.dofs.emplace_back(jointName + ":2", DofType::eRotation);
                        if (intJoint && intJoint->mJointDrives[PxArticulationAxis::eSWING2].isEnvelopeUsed)
                            isEnvelopeUsed = 1;
                    }
                    dofImpls.push_back({joint, PxArticulationAxis::eSWING2, isEnvelopeUsed });
                }
                break;
            case PxArticulationJointType::eUNDEFINED:
            default:
                CARB_LOG_ERROR("Unknown joint type for joint '%s'", jointPath.c_str());
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

    // Resolve the joint/link prim paths now: the topology is fixed for the view's lifetime, so
    // caching here lets the accessors return a pointer that outlives the call.
    for (DofImpl& dofImpl : entryRet.dofImpls)
    {
        if (dofImpl.joint)
        {
            dofImpl.path = g_physx->objectKeyToPath(
                g_physx->getObjectKeyForId(reinterpret_cast<size_t>(dofImpl.joint->userData)));
        }
    }
    entryRet.linkPaths.resize(entryRet.links.size());
    for (size_t linkIdx = 0; linkIdx < entryRet.links.size(); ++linkIdx)
    {
        if (entryRet.links[linkIdx])
        {
            entryRet.linkPaths[linkIdx] = g_physx->objectKeyToPath(
                g_physx->getObjectKeyForId(reinterpret_cast<size_t>(entryRet.links[linkIdx]->userData)));
        }
    }
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

    if (!canonicalPath.empty())
    {
        entryRet.path = canonicalPath;
    }
    else
    {
        // ArticulationEntry::path is a std::string (it feeds subspace matching, which is keyed by
        // string since the zero-USD build), so the key is resolved and stringified here rather than
        // at the caller. A stageless attach has no path to give and leaves it empty.
        entryRet.path = mAttachedStage ? mAttachedStage->textFor(fallbackKey) : "";
    }

    entryRet.subspace = findSubspaceForPath(entryRet.path);

    return true;
}

bool BaseSimulationView::getRigidBodyAtPath(omni::physics::parse::ObjectKey key, RigidBodyEntry& entryRet)
{
    if (!g_physx)
    {
        return false;
    }

    PxRigidBody* body = nullptr;
    RigidBodyType type = RigidBodyType::eInvalid;

    // check if it's an articulation link
    PxArticulationLink* link = static_cast<PxArticulationLink*>(resolvePhysXPtr(mAttachedStage, key, omni::physx::ePTLink));
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
            (PxArticulationReducedCoordinate*)resolvePhysXPtr(mAttachedStage, key, omni::physx::ePTArticulation);
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
            PxActor* actor = static_cast<PxActor*>(resolvePhysXPtr(mAttachedStage, key, omni::physx::ePTActor));
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
        return false;
    }

    const std::string path = mAttachedStage ? mAttachedStage->textFor(key) : "";

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

bool BaseSimulationView::getVolumeDeformableBodyAtPath(omni::physics::parse::ObjectKey key, DeformableBodyEntry& entryRet)
{
    if (!g_physx || !mAttachedStage)
    {
        return false;
    }

    // Only needed for the warning messages and entryRet.path below.
    const std::string path = mAttachedStage->textFor(key);

    // check if it's a volume deformable body
    PxDeformableVolume* deformable = static_cast<PxDeformableVolume*>(resolvePhysXPtr(mAttachedStage, key, omni::physx::ePTDeformableVolume));
    if (!deformable)
    {
        CARB_LOG_WARN("Failed to find volume deformable body at '%s'", path.c_str());
        return false;
    }

    const omni::physics::parse::IPhysicsSource* src = mAttachedStage->getSource();
    omni::physics::parse::KnownTokens tok;
    if (src)
    {
        tok.intern(*src);
    }
    const bool hasDeformableBodyApi = src && src->hasSchema(key, tok.omniphysicsDeformableBodyAPI);
    if (!hasDeformableBodyApi)
    {
        CARB_LOG_WARN("Volume deformable body at '%s' requires OmniPhysicsDeformableBodyAPI", path.c_str());
        return false;
    }

    // Mesh identity comes from the loader, not from a second walk of the scene graph.
    // The loader resolved the sim/collision meshes from the descriptor (both walkers
    // fill it) and stored them as source-agnostic ObjectKeys, so this resolves with or
    // without a backing USD stage and cannot disagree with what was actually simulated.
    const internal::InternalVolumeDeformableBody* internalBody =
        getInternalPtr<internal::InternalVolumeDeformableBody>(
            omni::physx::ePTDeformableVolume, resolveObjectId(mAttachedStage, key, omni::physx::ePTDeformableVolume));
    if (!internalBody || !internalBody->mSimMeshKey.valid() || !internalBody->mCollMeshKey.valid())
    {
        CARB_LOG_WARN("Failed to find simulation or collision mesh for volume deformable body at '%s'", path.c_str());
        return false;
    }

    const omni::physics::parse::ObjectKey simMeshKey = internalBody->mSimMeshKey;
    const omni::physics::parse::ObjectKey collMeshKey = internalBody->mCollMeshKey;
    const std::string simMeshPath = mAttachedStage->textFor(simMeshKey);
    const std::string collMeshPath = mAttachedStage->textFor(collMeshKey);

    // isTetMeshLike, not isA(UsdGeomTetMesh): ovstage reports a UsdGeomTetMesh as plain
    // "Mesh" (its populator has no TetMesh mapping), so the concrete-type check rejects
    // every volume deformable loaded from a non-USD source. Same gate the loader uses.
    // See PhysXTools.h::isTetMeshLike.
    if (!internal::isTetMeshLike(*mAttachedStage, simMeshKey))
    {
        CARB_LOG_WARN("Simulation mesh at '%s' is not a tetrahedral mesh", simMeshPath.c_str());
        return false;
    }

    if (!internal::isTetMeshLike(*mAttachedStage, collMeshKey))
    {
        CARB_LOG_WARN("Collision mesh at '%s' is not a tetrahedral mesh", collMeshPath.c_str());
        return false;
    }

    std::vector<carb::Int4> simMeshIndices;
    internal::getArrayValue(*mAttachedStage, simMeshKey, tok.tetVertexIndices,
                            omni::physics::parse::ReadTime::defaultTime(), simMeshIndices);
    // Explicit empty check: an unreadable array and an authored-empty one are
    // indistinguishable downstream, and every size comparison below would then pass
    // vacuously at 0 == 0, producing a view with no indices and no diagnostic.
    if (simMeshIndices.empty())
    {
        CARB_LOG_WARN("Simulation mesh at '%s' has no readable tetVertexIndices", simMeshPath.c_str());
        return false;
    }

    // Read rest shape attributes and check on current restrictions
    std::vector<carb::Float3> restPositions;
    {
        std::vector<carb::Int4> restTetVtxIndices;
        internal::getArrayValue(*mAttachedStage, simMeshKey, tok.omniphysicsRestTetVtxIndices,
                                omni::physics::parse::ReadTime::defaultTime(), restTetVtxIndices);

        if (simMeshIndices.size() != restTetVtxIndices.size() ||
            std::memcmp(simMeshIndices.data(), restTetVtxIndices.data(), sizeof(carb::Int4) * simMeshIndices.size()) != 0)
        {
            CARB_LOG_WARN("No support for distinct rest shape topology. The simulation mesh's tetVertexIndices need to "
                          "match up with VolumeDeformableSimAPI restTetVtxIndices at '%s'", simMeshPath.c_str());
            return false;
        }

        std::vector<carb::Float3> simPoints;
        internal::getArrayValue(*mAttachedStage, simMeshKey, tok.points,
                                omni::physics::parse::ReadTime::defaultTime(), simPoints);

        internal::getArrayValue(*mAttachedStage, simMeshKey, tok.omniphysicsRestShapePoints,
                                omni::physics::parse::ReadTime::defaultTime(), restPositions);

        if (simPoints.size() != restPositions.size())
        {
            CARB_LOG_WARN("No support for distinct rest shape topology. The simulation mesh's points need to match up "
                          "with VolumeDeformableSimAPI restShapePoints at '%s'", simMeshPath.c_str());
        }
    }

    std::vector<carb::Int4> collMeshIndices;
    internal::getArrayValue(*mAttachedStage, collMeshKey, tok.tetVertexIndices,
                            omni::physics::parse::ReadTime::defaultTime(), collMeshIndices);
    if (collMeshIndices.empty())
    {
        CARB_LOG_WARN("Collision mesh at '%s' has no readable tetVertexIndices", collMeshPath.c_str());
        return false;
    }

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
    const ::physx::PxMat44d simToWorld =
        internal::getWorldTransform(*mAttachedStage, simMeshKey, omni::physics::parse::ReadTime::defaultTime());
    for (size_t i = 0; i < entryRet.restPositions.size(); ++i)
    {
        const carb::Float3& rest = restPositions[i];
        const ::physx::PxVec3d restPoint = simToWorld.transform(::physx::PxVec3d(rest.x, rest.y, rest.z));
        entryRet.restPositions[i] = { float(restPoint.x), float(restPoint.y), float(restPoint.z) };
    }

    entryRet.collIndices.resize(collMeshIndices.size() * 4);
    std::memcpy(entryRet.collIndices.data(), collMeshIndices.data(), sizeof(PxU32) * entryRet.collIndices.size());

    return true;
}

bool BaseSimulationView::getSurfaceDeformableBodyAtPath(omni::physics::parse::ObjectKey key, DeformableBodyEntry& entryRet)
{
    if (!g_physx || !mAttachedStage)
    {
        return false;
    }

    // Only needed for the warning messages and entryRet.path below.
    const std::string path = mAttachedStage->textFor(key);

    // check if it's a surface deformable body
    PxDeformableSurface* deformable = static_cast<PxDeformableSurface*>(resolvePhysXPtr(mAttachedStage, key, omni::physx::ePTDeformableSurface));
    if (!deformable)
    {
        CARB_LOG_WARN("Failed to find surface deformable body at '%s'", path.c_str());
        return false;
    }

    const omni::physics::parse::IPhysicsSource* src = mAttachedStage->getSource();
    omni::physics::parse::KnownTokens tok;
    if (src)
    {
        tok.intern(*src);
    }
    const bool hasDeformableBodyApi = src && src->hasSchema(key, tok.omniphysicsDeformableBodyAPI);
    if (!hasDeformableBodyApi)
    {
        CARB_LOG_WARN("Surface deformable body at '%s' requires OmniPhysicsDeformableBodyAPI", path.c_str());
        return false;
    }

    // Mesh identity comes from the loader, not from a second walk of the scene graph --
    // see getVolumeDeformableBodyAtPath. A surface deformable has no separate collision
    // mesh: createSurfaceDeformableBody rejects a descriptor whose collision mesh differs
    // from its sim mesh, so the body existing at all means the two coincide.
    const internal::InternalSurfaceDeformableBody* internalBody =
        getInternalPtr<internal::InternalSurfaceDeformableBody>(
            omni::physx::ePTDeformableSurface, resolveObjectId(mAttachedStage, key, omni::physx::ePTDeformableSurface));
    if (!internalBody || !internalBody->mSimMeshKey.valid())
    {
        CARB_LOG_WARN("Failed to find simulation or collision mesh for surface deformable body at '%s'", path.c_str());
        return false;
    }

    const omni::physics::parse::ObjectKey simMeshKey = internalBody->mSimMeshKey;
    const std::string simMeshPath = mAttachedStage->textFor(simMeshKey);

    const bool simMeshIsMesh = src && src->isA(simMeshKey, tok.meshType);
    if (!simMeshIsMesh)
    {
        CARB_LOG_WARN("Simulation mesh at '%s' is not a UsdGeomMesh", simMeshPath.c_str());
        return false;
    }

    std::vector<int32_t> simMeshIndices;
    internal::getArrayValue(*mAttachedStage, simMeshKey, tok.faceVertexIndices,
                            omni::physics::parse::ReadTime::defaultTime(), simMeshIndices);
    // Explicit empty check: an unreadable array and an authored-empty one are
    // indistinguishable downstream, and every size comparison below would then pass
    // vacuously at 0 == 0 (including `size % 3`), producing a view with no indices and
    // no diagnostic.
    if (simMeshIndices.empty())
    {
        CARB_LOG_WARN("Simulation mesh at '%s' has no readable faceVertexIndices", simMeshPath.c_str());
        return false;
    }
    if (simMeshIndices.size() % 3 != 0)
    {
        CARB_LOG_WARN("Simulation mesh at '%s' has non-triangular faces", simMeshPath.c_str());
        return false;
    }

    // Read rest shape attributes and check on current restrictions
    std::vector<carb::Float3> restPositions;
    {
        std::vector<carb::Int3> restTriVtxIndices;
        internal::getArrayValue(*mAttachedStage, simMeshKey, tok.omniphysicsRestTriVtxIndices,
                                omni::physics::parse::ReadTime::defaultTime(), restTriVtxIndices);

        if (simMeshIndices.size() != restTriVtxIndices.size()*3 ||
            std::memcmp(simMeshIndices.data(), restTriVtxIndices.data(), sizeof(int32_t) * simMeshIndices.size()) != 0)
        {
            CARB_LOG_WARN(
                "No support for distinct rest shape topology. The simulation mesh's faceVertexIndices need to "
                "match up with SurfaceDeformableSimAPI restTriVtxIndices at '%s'",
                simMeshPath.c_str());
            return false;
        }

        std::vector<carb::Float3> simPoints;
        internal::getArrayValue(*mAttachedStage, simMeshKey, tok.points,
                                omni::physics::parse::ReadTime::defaultTime(), simPoints);

        internal::getArrayValue(*mAttachedStage, simMeshKey, tok.omniphysicsRestShapePoints,
                                omni::physics::parse::ReadTime::defaultTime(), restPositions);

        if (simPoints.size() != restPositions.size())
        {
            CARB_LOG_WARN(
                "No support for distinct rest shape topology. The simulation mesh's points need to match up "
                "with SurfaceDeformableSimAPI restShapePoints at '%s'",
                simMeshPath.c_str());
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
    const ::physx::PxMat44d simToWorld =
        internal::getWorldTransform(*mAttachedStage, simMeshKey, omni::physics::parse::ReadTime::defaultTime());
    for (size_t i = 0; i < entryRet.restPositions.size(); ++i)
    {
        const carb::Float3& rest = restPositions[i];
        const ::physx::PxVec3d restPoint = simToWorld.transform(::physx::PxVec3d(rest.x, rest.y, rest.z));
        entryRet.restPositions[i] = { float(restPoint.x), float(restPoint.y), float(restPoint.z) };
    }

    return true;
}

bool BaseSimulationView::getDeformableMaterialAtPath(omni::physics::parse::ObjectKey key, DeformableMaterialEntry& entryRet)
{
    if (!g_physx)
    {
        return false;
    }

    // Only needed for the warning message and entryRet.path below.
    const std::string path = mAttachedStage ? mAttachedStage->textFor(key) : "";
    PxDeformableMaterial* material = static_cast<PxDeformableMaterial*>(resolvePhysXPtr(mAttachedStage, key, omni::physx::ePTDeformableVolumeMaterial));
    bool isSurface = false;
    if (!material)
    {
        material = static_cast<PxDeformableMaterial*>(resolvePhysXPtr(mAttachedStage, key, omni::physx::ePTDeformableSurfaceMaterial));
        isSurface = (material != nullptr);
    }

    if (!material)
    {
        CARB_LOG_WARN("Failed to find deformable material at '%s'", path.c_str());
        return false;
    }

    // populate entry
    entryRet.material = material;
    entryRet.path = path;
    entryRet.isSurface = isSurface;

    return true;
}

bool BaseSimulationView::getRigidContactSensorAtPath(omni::physics::parse::ObjectKey key, RigidContactSensorEntry& entryRet)
{
    const omni::physics::parse::IPhysicsSource* src = mAttachedStage ? mAttachedStage->getSource() : nullptr;
    const std::string pathStr = mAttachedStage ? mAttachedStage->textFor(key) : "";

    if (src && src->exists(key))
    {
        omni::physics::parse::KnownTokens tok;
        tok.intern(*src);
        const bool hasContactReportApi = src->hasSchema(key, tok.physxContactReportAPI);
        if (!hasContactReportApi)
        {
            CARB_LOG_WARN("Failed to find contact report API at '%s'", pathStr.c_str());
            return false;
        }
    }
    else
    {
        // No object at this path in the source. Runtime clones can have a real
        // PhysX actor without a source object exposing the copied
        // PhysxContactReportAPI metadata; the source actor's contact-report
        // registration is mirrored by the PhysX replicator, so such paths are
        // validated by the runtime PhysX pointer lookup below (which returns
        // false if no link/actor/shape is found). Objects the source does know
        // still take the strict schema path above.
    }

    // figure out if it's a rigid dynamic, articulation link, or shape
    //
    // A runtime clone can have a real PhysX actor with no live source object (see the
    // comment above), so the PhysX pointer lookups below use `key` directly -- the
    // same key resolves both branches now, so the separate sensorKey/lookupKey split
    // a prior increment's keyFor-on-entry retype needed is gone (both were always the
    // same `mAttachedStage->keyFor(path)` computation).
    PxArticulationLink* link = nullptr;
    PxRigidDynamic* rd = nullptr;
    PxShape* shape = nullptr;
    link = static_cast<PxArticulationLink*>(resolvePhysXPtr(mAttachedStage, key, omni::physx::ePTLink));
    if (!link)
    {
        PxActor* actor = static_cast<PxActor*>(resolvePhysXPtr(mAttachedStage, key, omni::physx::ePTActor));
        if (actor && actor->getType() == PxActorType::eRIGID_DYNAMIC)
        {
            rd = static_cast<PxRigidDynamic*>(actor);
        }
        else
        {
            shape = static_cast<PxShape*>(resolvePhysXPtr(mAttachedStage, key, omni::physx::ePTShape));
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
        std::string name = storagePathName(g_physx->objectKeyToPath(g_physx->getObjectKeyForId(linkId)));
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

    entryRet.path = pathStr;
    // Legacy encoding, must match ContactEventHeader -- see CommonTypes.h.
    entryRet.referentId = asInt(key);
    entryRet.link = link;
    entryRet.rd = rd;
    entryRet.shape = shape;
    entryRet.subspace = findSubspaceForPath(entryRet.path);

    return true;
}

bool BaseSimulationView::getSDFShapeAtPath(omni::physics::parse::ObjectKey key, SdfShapeEntry& entryRet)
{
    const omni::physics::parse::IPhysicsSource* src = mAttachedStage ? mAttachedStage->getSource() : nullptr;
    // Only needed for the warning messages below.
    const std::string path = mAttachedStage ? mAttachedStage->textFor(key) : "";

    if (src && src->exists(key))
    {
        omni::physics::parse::KnownTokens tok;
        tok.intern(*src);
        const bool collisionApi = src->hasSchema(key, tok.physicsCollisionAPI);
        const bool SDFMeshApi = src->hasSchema(key, tok.physxSDFMeshCollisionAPI);
        if (!SDFMeshApi || !collisionApi)
        {
            CARB_LOG_WARN("Failed to find CollisionAPI and PhysxSDFMeshCollisionAPI for prim at ('%s')", path.c_str());
            return false;
        }
    }
    else
    {
        // No object at this path in the source (e.g. a runtime clone). The shape
        // is validated by the PhysX pointer lookup below (returns false if no
        // shape is found) and the SDF-validity check that follows. Objects the
        // source does know still take the strict schema path above.
    }

    // A runtime clone can have a real PhysX shape with no live source object, so the
    // PhysX pointer lookup below uses `key` directly -- the same key resolves both
    // branches now (see getRigidContactSensorAtPath's equivalent note).
    PxShape* shape = static_cast<PxShape*>(resolvePhysXPtr(mAttachedStage, key, omni::physx::ePTShape));
    if (!shape)
    {
        CARB_LOG_WARN("Failed to find a mesh at '%s'", path.c_str());
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
        CARB_LOG_WARN("Failed to find a valid SDF mesh for prim at ('%s')", path.c_str());
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
        releaseChildViews(mArtiViews);
        releaseChildViews(mRbViews);
        releaseChildViews(mVolumeDeformableBodyViews);
        releaseChildViews(mSurfaceDeformableBodyViews);
        releaseChildViews(mDeformableMaterialViews);
        releaseChildViews(mRcViews);
        releaseChildViews(mPointInstancerViews);
        releaseChildViews(mSDFViews);
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

void BaseSimulationView::_onChildRelease(const BasePointInstancerView* instancerView)
{
    auto it = std::find(mPointInstancerViews.begin(), mPointInstancerViews.end(), instancerView);
    if (it != mPointInstancerViews.end())
    {
        mPointInstancerViews.erase(it);
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

bool BaseSimulationView::hasScene(const PxScene* scene) const
{
    return mScene != nullptr && scene == mScene;
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
