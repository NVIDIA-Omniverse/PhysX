// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "Handles.h"

#include <cstddef>
#include <cstdint>
#include <functional>

namespace omni
{
namespace physics
{
namespace parse
{

/// @brief Element type carried by a ColumnView.
enum class ColumnType : uint8_t
{
    eNone,
    eBool,
    eInt32,
    eInt64,
    eFloat,
    eDouble,
    eFloat2,
    eFloat3,
    eFloat4,
    eToken,     //!< TokenId payload
    eObjectKey, //!< ObjectKey payload (used by the `keys` column)
};

/// @brief A typed, possibly device-resident column delivered by the feed.
///
/// `data` points to `count` contiguous elements of `type`; `device == -1`
/// means host/CPU, `>= 0` is a GPU device ordinal. The view is owned by the
/// feed and valid only for the duration of the OnChangeFn callback — callers
/// that need to retain it must copy (host) or take a tensor reference (GPU).
///
/// This is the parse-lib's minimal stand-in for a DLPack tensor; it is a
/// strict subset of ovstage's `ovstage_tensor_t` so an ovstage backend can
/// populate it from a read-group's `data` with no copy.
struct ColumnView
{
    ColumnType  type   = ColumnType::eNone;
    int         device = -1;
    size_t      count  = 0;
    const void* data   = nullptr;
};

/// @brief One change group, delivered per (objectType, property).
///
/// Sparsity is expressed in one of two mutually exclusive forms, mirroring
/// ovstage's read-group struct: `changedIndices` (gather — `data[j]` maps to
/// `keys[changedIndices[j]]`) or `mask` (dense bitmask — `data[i]` valid iff
/// bit `i` set). Both null ⇒ all rows in the group changed.
///
/// Structural vs value change (resolves ADR-0001 §9 / ADR-0002 open question):
///   - value change  → valid `property`, populated `values` column.
///   - structural     → `isDelete == true` (objects removed; `keys` lists
///                       them), or — where the backend models prim type /
///                       applied schemas as attributes (ovstage v2) — a value
///                       change on those attributes. No per-object kind enum.
struct ChangeBatch
{
    ObjectKey         objectType;             //!< schema/type bucket this group belongs to
    TokenId           property;               //!< changed property; invalid ⇒ structural change
    bool              isDelete      = false;  //!< true ⇒ tombstone group (objects removed)
    size_t            numChanges    = 0;
    ColumnView        values;                 //!< changed values; empty when property invalid / isDelete
    ColumnView        keys;                   //!< ObjectKey per row
    const ColumnView* changedIndices = nullptr; //!< sparse gather indices, or nullptr
    const ColumnView* mask           = nullptr; //!< dense bitmask, or nullptr (exclusive with changedIndices)
    uint64_t          userData       = 0;     //!< value passed at registration
};

/// @brief Callback invoked by the feed for each matching change group.
using OnChangeFn = std::function<void(const ChangeBatch&)>;

/// @brief Callback invoked once at the end of each delivery group, after all
/// of the group's `OnChangeFn`s have fired: for a push backend (USD) at the
/// end of each source notice, for a pull backend at the end of each `drain()`.
/// Lets a consumer run per-group finalization that must happen exactly once
/// after the group's changes are applied — e.g. flushing a set of transform
/// changes accumulated across the group.
using OnGroupCompleteFn = std::function<void()>;

/// @brief Backend-neutral feed of runtime change deltas.
///
/// A separate observer interface, deliberately not folded into IPhysicsSource
/// (which is a pull-only, const, stateless-per-call reader). The feed is push,
/// stateful, and optional, so it lives beside the source. A source backend
/// optionally vends one via `IPhysicsSource::createChangeFeed()`.
class IChangeFeed
{
public:
    virtual ~IChangeFeed() = default;

    /// @brief Declare interest in property `prop`, delivered to `cb`.
    /// `userData` is echoed back in every ChangeBatch. Pass an invalid `prop`
    /// to be notified of structural (resync) changes.
    ///
    /// `objectType` and `device` (-1 CPU, >=0 GPU ordinal) describe the intended
    /// type-scoped / GPU delivery contract for a columnar backend, but the
    /// current USD backend matches registrations purely by `prop`: it neither
    /// filters by `objectType` nor honors `device`, and does not populate
    /// `ChangeBatch::objectType`. A backend needing per-type or device routing
    /// must implement that filtering and populate the batch fields.
    virtual void registerInterest(ObjectKey objectType,
                                  TokenId prop,
                                  int device,
                                  OnChangeFn cb,
                                  uint64_t userData) = 0;

    /// @brief Register the per-group finalization callback (see
    /// OnGroupCompleteFn). At most one is used; the last registered wins.
    virtual void registerGroupComplete(OnGroupCompleteFn cb) = 0;

    /// @brief Process all changes accumulated since the last drain, invoking
    /// the registered callbacks in dependency-safe order (structural groups
    /// before value groups for the same type). Returns false if the backend
    /// cannot produce an incremental delta and a full rescan is required.
    virtual bool drain() = 0;

    /// @brief Process the changes within an explicit producer-supplied version
    /// range [from, to] (pull backends whose discovery is version/ordinal keyed,
    /// e.g. ovstage's range read). The default ignores the range and falls back
    /// to drain() — correct for push backends (USD), which buffer their own
    /// window. Returns false if the range could not be served (caller re-attaches).
    virtual bool drainRange(uint64_t from, uint64_t to)
    {
        (void)from;
        (void)to;
        return drain();
    }

    /// @brief Pause/resume change accumulation without dropping registrations.
    virtual void setEnabled(bool enabled) = 0;
};

} // namespace parse
} // namespace physics
} // namespace omni
