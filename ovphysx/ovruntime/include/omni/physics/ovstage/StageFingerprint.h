// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-SOURCE-IDENTITY-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-14
 * @implements REQ-CACHE-ATTACH-001
 * @covers AC-3
 */
#pragma once
#include <omni/physics/parse/CacheArchive.h>
#include <ovstage/ovstage.h>
#include <string>
#include <memory>
#include <optional>
#include <functional>
#include <vector>

namespace omni::physics::ovstage
{
class OvstageSource;
struct StageFingerprint
{
    parse::cache::Digest value{};
    size_t rows = 0;
    size_t reusedTypeRows = 0; // diagnostic: exact change-feed rows used without another backend read
    size_t reusedPathTexts = 0;
    bool reusedVocabulary = false;
    std::string bypassReason = "fingerprint has not been computed";
    explicit operator bool() const
    {
        return bypassReason.empty();
    }
};
// Native-instancing prototypes have generated `/__Prototype_<n>` roots that change
// between populations. canonicalPrototypePaths rewrites paths under them to
// `/__Prototype(<smallest instance root>)/...`; resolvePrototypePaths maps such names
// back through this population's instancing graph. Other paths are unchanged.
// Both return false when a prototype or instance root cannot be resolved. A
// snapshot captured from this population supplies the names its identity resolved.
class PhysicsInputSnapshot;
bool canonicalPrototypePaths(ovstage_instance_t* instance,
                             std::vector<std::string>& paths,
                             const PhysicsInputSnapshot* known = nullptr);
bool resolvePrototypePaths(ovstage_instance_t* instance,
                           std::vector<std::string>& paths,
                           const PhysicsInputSnapshot* known = nullptr);
// Owns exactly the selected read buffers until parsing or restoration completes.
// A stable source lease is required throughout its lifetime, as for ordinary attach.
class PhysicsInputSnapshot
{
public:
    ~PhysicsInputSnapshot();
    PhysicsInputSnapshot(const PhysicsInputSnapshot&) = delete;
    PhysicsInputSnapshot& operator=(const PhysicsInputSnapshot&) = delete;
    // A capture's first step on its own: the query that matches every prim, which
    // discovers the stage's attribute vocabulary and counts its prims (`prims`). A
    // caller that sizes the capture before starting it hands the result to capture(),
    // which continues from it instead of querying again; one query serves one capture.
    // Null when the query fails.
    struct StageQuery;
    static std::shared_ptr<StageQuery> queryStage(ovstage_instance_t* instance, size_t& prims);
    // sharedSource optionally supplies exact rows from the accepted current
    // drain and immutable dictionary mappings under the same stable source lease.
    // stageQuery, from queryStage() on the same instance, replaces the capture's own query.
    static std::shared_ptr<PhysicsInputSnapshot> capture(ovstage_instance_t* instance,
                                                         ovstage_ordinal_t ordinal,
                                                         const std::vector<std::string>& columns,
                                                         const std::vector<std::string>& paths = {},
                                                         std::string_view relativeRoot = {},
                                                         const OvstageSource* sharedSource = nullptr,
                                                         std::shared_ptr<StageQuery> stageQuery = nullptr);
    const StageFingerprint& fingerprint() const;
    // nullopt means the read plan did not declare this column. Declared missing
    // rows return false; present rows invoke fn with a one-row tensor view.
    std::optional<bool> withTensor(uint64_t path,
                                   std::string_view column,
                                   const std::function<void(const DLTensor&, uint32_t)>& fn) const;
    // Scatter retained fixed-column views into a parser batch. A null tensor
    // marks an array cell for ordinary scalar fallback. False means an undeclared
    // column or out-of-scope path requires the ordinary read path.
    bool withScalars(const std::vector<uint64_t>& paths,
                     const std::vector<std::string>& columns,
                     const std::function<void(size_t, size_t, const DLTensor*, int64_t, uint32_t)>& fn) const;
    // Visit logical token values from the retained read for capability auditing.
    // Path handles are local to this snapshot; no parser binding/index is built.
    bool visitTokens(std::string_view column, const std::function<bool(uint64_t, std::string_view)>& fn) const;
    // One distinct value of a type or schema column: the rows that hold it and the
    // path handle of the first of them, in logical path order.
    struct TokenName
    {
        std::string_view name;
        uint64_t rows = 0;
        uint64_t firstPath = 0;
    };
    // Visit the distinct values of usd-prim-type, usd-metadata:typeName or usd-schemas,
    // recorded while the capture hashed the column, without visiting its rows. A prim's
    // usd-metadata:typeName overrides its usd-prim-type, as the parser reads its type,
    // so usd-prim-type's values count only the prims without one. True when the column
    // has no rows; false for another column or when fn declines.
    bool visitTokenNames(std::string_view column, const std::function<bool(const TokenName&)>& fn) const;
    // Visit every captured relationship or connection target as (prim path
    // handle, column, target path). False if fn declines or a target is unreadable.
    bool visitPathTargets(const std::function<bool(uint64_t, std::string_view, std::string_view)>& fn) const;
    // Whole-stage membership determines derived hierarchy without asking the
    // backend to synthesize parent/children columns. Scoped snapshots cannot
    // prove absence of descendants outside their selected read set.
    std::optional<bool> hasDescendants(uint64_t path) const;
    // Global column presence was discovered even for a scoped capture. Reuse
    // it for declared presence checks under the same stable source lease.
    std::optional<bool> hasStageColumn(std::string_view column, bool prefix = false) const;
    bool complete() const;
    // Check coverage without consuming values or invalidating the snapshot.
    // Selected absent paths and declared absent columns are covered too.
    bool covers(const std::vector<uint64_t>& paths, const std::vector<std::string>& columns) const;
    void observeColumn(std::string_view column, bool prefix = false) const;
    // The parser used a value of this column that it read from the live stage, not
    // from these inputs. The identity does not cover it, so nothing is stored.
    void observeLiveRead(std::string_view column) const;

private:
    friend bool canonicalPrototypePaths(ovstage_instance_t*, std::vector<std::string>&, const PhysicsInputSnapshot*);
    friend bool resolvePrototypePaths(ovstage_instance_t*, std::vector<std::string>&, const PhysicsInputSnapshot*);
    PhysicsInputSnapshot();
    struct Impl;
    std::unique_ptr<Impl> mImpl;
};
} // namespace omni::physics::ovstage
