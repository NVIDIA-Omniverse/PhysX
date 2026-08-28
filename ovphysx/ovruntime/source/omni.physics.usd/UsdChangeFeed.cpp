// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#include "UsdChangeFeed.h"

#include "UsdSource.h"

#include <pxr/usd/usd/notice.h>
#include <pxr/usd/usd/stage.h>

namespace omni
{
namespace physics
{
namespace usd
{

using namespace PXR_NS;
using parse::ChangeBatch;
using parse::ColumnType;
using parse::ColumnView;
using parse::ObjectKey;
using parse::TokenId;

UsdChangeFeed::UsdChangeFeed(UsdSource& source, UsdStageWeakPtr stage)
    : mSource(source), mStage(stage)
{
    // Filter the notice to this stage (the sender), so each per-source feed
    // only sees its own stage's changes — replacing the single global listener.
    mNoticeKey = TfNotice::Register(TfCreateWeakPtr(this), &UsdChangeFeed::onObjectsChanged, mStage);
}

UsdChangeFeed::~UsdChangeFeed()
{
    TfNotice::Revoke(mNoticeKey);
}

void UsdChangeFeed::registerInterest(
    ObjectKey objectType, TokenId prop, int /*device*/, parse::OnChangeFn cb, uint64_t userData)
{
    // USD is host-only; the requested device is ignored (no GPU columns).
    mRegistrations.push_back({ objectType, prop, std::move(cb), userData });
}

void UsdChangeFeed::registerGroupComplete(parse::OnGroupCompleteFn cb)
{
    mGroupComplete = std::move(cb);
}

bool UsdChangeFeed::drain()
{
    // Push-through: callbacks already fired synchronously from the notice
    // handler, preserving today's immediate processing timing. Nothing to
    // drain. Always succeeds (no incremental-delta failure mode for USD).
    return true;
}

void UsdChangeFeed::setEnabled(bool enabled)
{
    mEnabled = enabled;
}

void UsdChangeFeed::dispatch(const ChangeBatch& batch) const
{
    for (const Registration& reg : mRegistrations)
    {
        // Wildcard (invalid objectType + invalid prop) matches every batch;
        // otherwise match on the property the registration declared interest in.
        const bool wildcard = !reg.objectType.valid() && !reg.prop.valid();
        if (!wildcard && reg.prop != batch.property)
            continue;
        if (reg.cb)
        {
            ChangeBatch b = batch;
            b.userData = reg.userData;
            reg.cb(b);
        }
    }
}

void UsdChangeFeed::onObjectsChanged(const UsdNotice::ObjectsChanged& notice)
{
    if (!mEnabled)
        return;

    UsdStageWeakPtr stage = mStage;
    if (!stage)
        return;

    const SdfPath pseudoRoot = stage->GetPseudoRoot().GetPath();

    auto emitValue = [&](const SdfPath& primKey, const TfToken& token)
    {
        const ObjectKey key = mSource.keyFor(primKey);
        ChangeBatch batch;
        batch.property = mSource.tokenFor(token);
        batch.numChanges = 1;
        batch.keys = ColumnView{ ColumnType::eObjectKey, -1, 1, &key };
        dispatch(batch);
    };

    // --- Resynced (structural) paths ---
    for (const SdfPath& path : notice.GetResyncedPaths())
    {
        if (path.IsAbsoluteRootOrPrimPath())
        {
            const SdfPath primKey = (pseudoRoot == path) ? pseudoRoot : path.GetPrimPath();
            const UsdPrim prim = stage->GetPrimAtPath(primKey);
            const ObjectKey key = mSource.keyFor(primKey);

            if (!prim.IsValid() || !prim.IsActive())
            {
                // Removal → isDelete batch.
                ChangeBatch batch;
                batch.isDelete = true;
                batch.numChanges = 1;
                batch.keys = ColumnView{ ColumnType::eObjectKey, -1, 1, &key };
                dispatch(batch);
            }
            else
            {
                // Structural resync → invalid property, changed-field tokens in
                // `values`. The consumer reproduces the typeName/apiSchemas/kind
                // decision tree (it owns isInPrimAddMap / IsPrototype / pseudoroot
                // policy).
                const TfTokenVector& changedFields = notice.GetChangedFields(primKey);
                std::vector<TokenId> fieldTokens;
                fieldTokens.reserve(changedFields.size());
                for (const TfToken& f : changedFields)
                    fieldTokens.push_back(mSource.tokenFor(f));

                ChangeBatch batch;
                batch.numChanges = 1;
                batch.keys = ColumnView{ ColumnType::eObjectKey, -1, 1, &key };
                batch.values = ColumnView{ ColumnType::eToken, -1, fieldTokens.size(),
                                           fieldTokens.empty() ? nullptr : fieldTokens.data() };
                dispatch(batch);
            }
        }
        else if (path.IsPropertyPath())
        {
            emitValue(path.GetParentPath(), path.GetNameToken());
        }
    }

    // --- Changed-info-only (value) paths ---
    for (const SdfPath& path : notice.GetChangedInfoOnlyPaths())
    {
        const SdfPath primKey = (pseudoRoot == path) ? path : path.GetPrimPath();

        // Use `path` (not `primKey`) here: USD's changed-field queries are keyed
        // by the spec path that actually changed. For a property path like
        // /Prim.attr, HasChangedFields(/Prim) would ask about prim metadata, not
        // the property spec — the two are distinct entries in the change notice.
        if (notice.HasChangedFields(path))
        {
            const TfTokenVector changedFieldTokens = notice.GetChangedFields(path);
            for (const TfToken& f : changedFieldTokens)
                emitValue(primKey, f);
        }

        if (path.IsPropertyPath())
            emitValue(primKey, path.GetNameToken());
    }

    // End of this notice's delivery group: run consumer finalization once
    // (e.g. flush accumulated transform changes), mirroring the single
    // processTransformChanges() call at the tail of the legacy notice handler.
    if (mGroupComplete)
        mGroupComplete();
}

} // namespace usd
} // namespace physics
} // namespace omni
