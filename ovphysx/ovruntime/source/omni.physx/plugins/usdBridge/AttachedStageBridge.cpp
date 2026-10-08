// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-BUILD-UNIBUILD-001
 * @covers AC-3
 */

// pxr-free AttachedStage bridge (ADR-0027): names no pxr type, includes no pxr header, calls
// no omni::physics::usd:: symbol. With no seam installed (production / USD-free) every entry is
// a no-op; USD-loaded behaviour reaches USD only through the installed seams -- the parse backend
// registry (for the source rebuild), the opaque pxr-handle op table
// (omni::physics::parse::attachedStageUsdHandleOps(), ADR-0027 seam #3) for the
// AttachedStageUsdHandle special members, and the backing-author seam
// (omni::physics::parse::usdBackingAuthor()) for the resident-backing-stage authoring fallbacks.
//
// Path/key/token resolution is pxr-free here: keyFor(string_view) / textFor() / textViewFor();
// test code that needs pxr-typed views converts on its own side.

#include "usdLoad/AttachedStage.h"
#include "usdLoad/PrimUpdate.h" // onSourceChange / onSourceGroupComplete
#include "usdLoad/LoadTools.h"
#include "usdInterface/UsdInterface.h"
#include <omni/physics/parse/OpaquePxrHandleOps.h> // parse::attachedStageUsdHandleOps (seam #3)

#include <CookingDataAsync.h>
#include <OmniPhysX.h>

#include <omni/physics/parse/IParseBackend.h>
#include <omni/physics/parse/IChangeFeed.h>
#include <omni/physics/parse/UsdBackingAuthor.h> // omni::physics::parse::usdBackingAuthor (seam)
#include <omni/physics/parse/UsdReparse.h> // omni::physics::parse::usdReparse (seam #2)

#include <cstring>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace omni
{
namespace physx
{
namespace usdparser
{

// --- ctor slivers -----------------------------------------------------------------------

void AttachedStage::initUsdStageBinding(AttachedStageUsdHandle stage)
{
    // setStage() is pxr-free (it takes the opaque handle). For an empty handle -- the only
    // handle an ovstage attach constructs -- it clears external state and rebuildSource()
    // falls through to its no-source branch (a no-op). A non-empty handle
    // (a future plain-USD attach) binds it and rebuilds through the registry.
    setStage(stage);
}

void AttachedStage::initUsdChangeRegistrations()
{
    // Pxr-free intern of the change params the common ctor staged. Null-source (ovstage's
    // ctor-time state) makes this a no-op; a USD ctor whose setStage() above
    // already built a source interns now rather than waiting for the next rebuildSource().
    if (mSource)
        mPrimChangeMap.internRegisteredChanges(*mSource);
}

// --- source rebuild over a live stage handle (registry-routed) --------------------------

bool AttachedStage::rebuildUsdSource()
{
    // No live stage handle -> nothing to build; rebuildSource() takes its no-source teardown
    // branch. mStage's truthiness routes through the handle op table (empty/no-ops -> false).
    if (!mStage)
        return false;

    // The USD arm built the trio through a PRIVATE USD parse backend, never the active one,
    // because the active backend may be a data-plane backend (ovstage) that reads nativeStage
    // as its own payload. That private backend now lives behind the reparse seam
    // (usdReparse()->createSourceFromHandle), which uses its own USD backend and bypasses the
    // registered one -- so a foreign USD stage bound while an ovstage attach is live still
    // parses through USD instead of crashing in the ovstage backend
    // (TestOvstageStagelessForeignStageParse.cpp). The handle op tables outlive the functional
    // seams (process-lifetime, ADR-0027), so a non-empty handle with no reparse seam is a
    // legitimate state that fails closed here.
    omni::physics::parse::IUsdReparse* reparse = omni::physics::parse::usdReparse();
    if (!reparse)
        return false;

    omni::physics::parse::SourceBundle bundle = reparse->createSourceFromHandle(mStage.storage());
    mSource = std::move(bundle.source);
    mDataWrite = std::move(bundle.write);
    mChangeFeed = std::move(bundle.changeFeed);
    mUnits = mSource ? mSource->getSourceUnits() : omni::physics::parse::SourceUnits{};
    bindKnownTokens();
    if (mSource)
        mPrimChangeMap.internRegisteredChanges(*mSource);

    // Change feed (ADR-0003): a single wildcard interest receives every batch; the
    // group-complete callback flushes accumulated transform changes once per notice. Mirrors
    // the USD arm and the common ctor's external branch.
    if (mChangeFeed)
    {
        AttachedStage* self = this;
        mChangeFeed->registerInterest(omni::physics::parse::ObjectKey{}, omni::physics::parse::TokenId{}, -1,
                                      [self](const omni::physics::parse::ChangeBatch& batch)
                                      { return onSourceChange(*self, batch); },
                                      0);
        mChangeFeed->registerGroupComplete([self]() { onSourceGroupComplete(*self); });

        // The cooking driver also observes this feed for recook scheduling; the interest is
        // owned by mChangeFeed and released when the feed is rebuilt.
        if (cookingdataasync::CookingDataAsync* cookingDataAsync =
                omni::physx::OmniPhysX::getInstance().getPhysXSetup().getCookingDataAsync())
        {
            cookingDataAsync->registerOnChangeFeed(*this);
        }
    }

    return true;
}

void AttachedStage::fillUsdAttachTarget(omni::physics::parse::AttachTarget& target) const
{
    // The live stage handle's opaque storage, or null when the handle is empty -- so the
    // no-live-stage case leaves nativeStage null instead of pointing at
    // an empty handle's bytes.
    target.nativeStage = mStage ? mStage.storage() : nullptr;
}

// --- pxr-free key resolution ------------------------------------------------------------

/**
 * @implements REQ-PUBLICAPI-001
 * @covers AC-46
 */
omni::physics::parse::ObjectKey AttachedStage::keyFor(std::string_view path) const
{
    // Existence-independent mint (ADR-0027 / TestObjectKeyMinting): keyFor must return a stable
    // key even for a path that names no live object yet -- e.g. a runtime clone target, which
    // authors no USD prim. findByPath is NOT that: on a USD source it is existence-dependent and
    // returns the invalid sentinel for an unauthored path, so the twin used usd->keyFor(SdfPath)
    // instead. mintKeyForPath is the pxr-free seam for that mint (default findByPath for sources
    // whose findByPath already interns, e.g. ovstage). Total on empty input: no key names "".
    if (path.empty())
        return {};
    return mSource ? mSource->mintKeyForPath(path) : omni::physics::parse::ObjectKey{};
}

// --- resident-backing-stage authoring fallbacks -----------------------------------------
// Routed through the backing-author seam (ADR-0027). The USD arm authored a default
// PhysicsScene straight into a resident backing UsdStage's session layer
// (createDefaultPhysicsScene / UsdPhysicsDataWrite), used only on an ovstage attach that
// retained a backing USD stage but wires no write sink. Each entry drives the installed seam
// over mStage's opaque storage, and fails closed to a no-op when no seam is installed
// (production / USD-free).

bool AttachedStage::createDefaultPhysicsSceneOnStage(omni::physics::parse::ObjectKey /*sceneKey*/)
{
    // FLIP-TIME WIRING GAP: the seam authors at the fixed canonical default-scene path and takes
    // no sceneKey. The _usd arm authored at usdPathFor(*this, sceneKey), so a caller that passes a
    // non-canonical sceneKey is silently authored at the canonical path here. The seam signature
    // is bare by design (no pxr path crosses it); reconcile the caller's sceneKey at the flip.
    omni::physics::parse::IUsdBackingAuthor* author = omni::physics::parse::usdBackingAuthor();
    return author ? author->createDefaultScene(mStage.storage()) : false;
}

void AttachedStage::removeDefaultPhysicsSceneOnStage(omni::physics::parse::ObjectKey /*sceneKey*/)
{
    // Same bare-seam sceneKey gap as createDefaultPhysicsSceneOnStage: removal targets the
    // canonical default-scene path, not usdPathFor(*this, sceneKey).
    if (omni::physics::parse::IUsdBackingAuthor* author = omni::physics::parse::usdBackingAuthor())
        author->removeDefaultScene(mStage.storage());
}

std::unique_ptr<omni::physics::parse::IPhysicsDataWrite> AttachedStage::makeBackingStageDataWrite()
{
    // The sink has no UsdSource, so it resolves its ObjectKeys/TokenIds through the active
    // (non-USD) source's string identity -- threaded to the seam as the fallback source, matching
    // the _usd twin (UsdPhysicsDataWrite(usdStageOf(mStage), nullptr, mSource.get())). Without it
    // an ovstage attach's backing-stage authoring (anisotropy primvars, sampled particle points)
    // silently no-ops because primFor(key) cannot resolve a key the ovstage source minted.
    if (!mStage || !mSource)
        return nullptr;
    omni::physics::parse::IUsdBackingAuthor* author = omni::physics::parse::usdBackingAuthor();
    return author ? author->makeBackingDataWrite(mStage.storage(), mSource.get()) : nullptr;
}

// --- AttachedStageUsdHandle special members (ADR-0027 seam #3) ---------------------------
// Every operation forwards to the installed op table when present; with no table installed the
// handle is a permanently-empty two-pointer POD (zero-filled storage, trivial copy/destroy,
// false truthiness).
using omni::physics::parse::OpaquePxrHandleOps;
using omni::physics::parse::attachedStageUsdHandleOps;

AttachedStageUsdHandle::AttachedStageUsdHandle() noexcept
{
    if (const OpaquePxrHandleOps* ops = attachedStageUsdHandleOps())
        ops->construct(storage());
    else
        std::memset(mStorage, 0, sizeof(mStorage));
}

AttachedStageUsdHandle::AttachedStageUsdHandle(std::nullptr_t) noexcept
{
    if (const OpaquePxrHandleOps* ops = attachedStageUsdHandleOps())
        ops->construct(storage());
    else
        std::memset(mStorage, 0, sizeof(mStorage));
}

AttachedStageUsdHandle::AttachedStageUsdHandle(const AttachedStageUsdHandle& other) noexcept
{
    if (const OpaquePxrHandleOps* ops = attachedStageUsdHandleOps())
        ops->copyConstruct(storage(), other.storage());
    else
        std::memcpy(mStorage, other.mStorage, sizeof(mStorage));
}

AttachedStageUsdHandle& AttachedStageUsdHandle::operator=(const AttachedStageUsdHandle& other) noexcept
{
    if (this != &other)
    {
        if (const OpaquePxrHandleOps* ops = attachedStageUsdHandleOps())
            ops->copyAssign(storage(), other.storage());
        else
            std::memcpy(mStorage, other.mStorage, sizeof(mStorage));
    }
    return *this;
}

AttachedStageUsdHandle::~AttachedStageUsdHandle()
{
    if (const OpaquePxrHandleOps* ops = attachedStageUsdHandleOps())
        ops->destroy(storage());
    // ops null: empty POD, nothing to destroy.
}

AttachedStageUsdHandle::operator bool() const noexcept
{
    if (const OpaquePxrHandleOps* ops = attachedStageUsdHandleOps())
        return ops->toBool(storage());
    return false;
}

} // namespace usdparser
} // namespace physx
} // namespace omni
