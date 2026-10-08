// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-FEED-002
 * @covers AC-3
 *
 * @implements REQ-PARSE-FEED-003
 * @covers AC-6 AC-7
 */
#pragma once

#include "OvstageSource.h"

#include <omni/physics/parse/IChangeFeed.h>

#include <ovstage/ovstage.h>
#include <ovstage/ovx_path_dictionary.h>

#include <cstdint>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace omni::physics::ovstage
{
using namespace omni::physics::parse;

class OvstageChangeFeed final : public IChangeFeed
{
public:
    // `source` must outlive the feed (it owns the path/token tables the feed
    // resolves through). `instance`/`dict` are the same non-owning handles the
    // source reads from.
    OvstageChangeFeed(OvstageSource& source, ovstage_instance_t* instance, ovx_path_dictionary_t* dict);
    ~OvstageChangeFeed() override;

    OvstageChangeFeed(const OvstageChangeFeed&) = delete;
    OvstageChangeFeed& operator=(const OvstageChangeFeed&) = delete;

    // --- IChangeFeed ---
    void registerInterest(ObjectKey objectType, TokenId prop, int device, OnChangeFn cb, uint64_t userData) override;
    void registerGroupComplete(OnGroupCompleteFn cb) override;
    bool drain() override; // no-op for the pull feed; the range comes via drainRange
    void setEnabled(bool enabled) override;

    // Read every changed column in the ordinal interval [ord0, ord1] and deliver
    // each as a ChangeBatch to the matching registrations, then fire the
    // group-complete callback once. Returns false if the range could not be read
    // (e.g. start precedes the oldest preserved ordinal -> caller must re-attach).
    // This is the ovstage realization of IChangeFeed::drainRange; the engine entry
    // IPhysxSimulation::updateFromOvStage forwards the producer's [ord0, ord1].
    bool drainRange(uint64_t ord0, uint64_t ord1) override;

private:
    struct Registration
    {
        ObjectKey  objectType;
        TokenId    prop;
        OnChangeFn cb;
        uint64_t   userData = 0;
    };

    struct CachedFilterQuery
    {
        ovstage_query_handle_t query = OVSTAGE_INVALID_QUERY_HANDLE;
        bool dirty = true;
        std::vector<ovx_token_t> discoveredValueAttributes;
        // Prims the filter matched when the handle was built. A re-execution that
        // disagrees means the family gained members (reconcileStructuralChanges).
        size_t primCount = 0;
    };

    // A known-path query kept as an immutable base plus a small delta of the paths added since
    // the last fold, so a structural add costs O(|added|) instead of re-interning n paths. The
    // adds are recorded at the insert site (noteAdded); a wholesale replacement of the set (seed,
    // rescan) is the only shape that needs the base rebuilt (noteReplaced).
    // A path-list query plus the list it wraps. The list is ours and immutable, so its
    // entries are captured once at build time: a read group over `list` (O(1) handle
    // compare), or over ovstage's equally sized copy of it, maps rows to paths by index
    // instead of re-fetching the list per group.
    struct PathListHandle
    {
        ovstage_query_handle_t query = OVSTAGE_INVALID_QUERY_HANDLE;
        ovx_primpath_list_t list = OVX_INVALID_PRIMPATH_LIST; // our reference, held for the query's lifetime
        std::vector<ovx_primpath_t> paths;                     // list order
        bool valid() const
        {
            return query != OVSTAGE_INVALID_QUERY_HANDLE;
        }
    };

    struct PathListQuery
    {
        PathListHandle base;
        PathListHandle delta;
        std::vector<std::string> deltaPaths; // the known paths `base` does not cover
        bool dirty = true;                    // `delta` does not match deltaPaths yet
        bool fold = true;                     // the set was replaced: rebuild `base` over all of it
        void noteAdded(const std::string& path)
        {
            deltaPaths.push_back(path);
            dirty = true;
        }
        void noteReplaced()
        {
            deltaPaths.clear();
            fold = true;
            dirty = true;
        }
        bool valid() const
        {
            return base.valid() || delta.valid();
        }
    };

    // Fan the batch out to the matching registrations. Returns false if a consumer signaled a
    // partial-commit failure (its OnChangeFn returned false) -- the drain must then hold the cursor
    // and redeliver the range. Returns true when every consumer accepted the batch.
    bool dispatch(const ChangeBatch& batch) const;
    void seedKnownPhysicsPaths();
    bool collectKnownPathsFromFilter(const char* attrName,
                                     ovstage_filter_op_t op,
                                     const ovx_string_t* values,
                                     size_t valueCount,
                                     CachedFilterQuery& query,
                                     std::unordered_set<std::string>& physicsPaths,
                                     std::unordered_set<std::string>& transformPaths,
                                     std::vector<ObjectKey>* newKeys = nullptr,
                                     const std::unordered_set<std::string>* oldPhysicsPaths = nullptr);
    void releaseCachedQuery(ovstage_query_handle_t& query);
    void releaseCachedQuery(CachedFilterQuery& query);
    void releaseCachedQuery(PathListHandle& handle);
    void releaseCachedQuery(PathListQuery& query);
    void dirtyCachedFamilyQueries();
    ovstage_query_handle_t queryForFilter(const char* attrName,
                                          ovstage_filter_op_t op,
                                          const ovx_string_t* values,
                                          size_t valueCount,
                                          CachedFilterQuery& query);
    // Executes the family filter once more as a throwaway query and reports how many
    // prims it matches now. With `refreshDiscovery` the cached discovered-attribute
    // set is replaced from the result; the cached query handle itself is untouched.
    bool executeFamilyQuery(const char* attrName,
                            ovstage_filter_op_t op,
                            const ovx_string_t* values,
                            size_t valueCount,
                            CachedFilterQuery& query,
                            bool refreshDiscovery,
                            size_t& primCount);
    bool familyMembershipUnchanged(bool& unchanged);
    // Rows of the structural columns written in the drained range, from one whole-stage
    // change-range read: the hierarchy delta, the growth verdict and the family
    // classification of every unfamiliar prim all come from the same rows. Tombstone groups
    // carry no live row -- the HAS(usd-path) query only matches live prims, so a delete cannot
    // be learned here (refreshHierarchyCache falls back to a full drop for it).
    struct StructuralRows
    {
        struct Row
        {
            ovx_primpath_t raw = 0; // first handle seen for the path (resolveGroupPathKey input)
            std::string path;
            bool bySchemas = false; // usd-schemas row names a known physics schema
            bool byType = false;    // usd-prim-type row is a known physics type
            bool physics() const { return bySchemas || byType; }
        };
        std::vector<ovx_primpath_t> added; // every live row (hierarchy delta; aliases allowed)
        // Live rows on paths outside mKnownPhysicsPaths. Deduped by path string, not handle:
        // get_paths / intern_path can hand back distinct handles for one path (OvstageSource.h,
        // canonicalKey), and the two columns come back in independent groups.
        std::vector<Row> unfamiliar;
        // The post-write usd-schemas list / usd-prim-type of every prim the range touched (empty
        // / 0 for a tombstone), for the source's schema-membership and prim-type patches.
        // Meaningful only when `exact`.
        std::vector<SchemaRowDelta> schemas;
        std::vector<PrimTypeRowDelta> types;
        // Known paths the read saw tombstoned (known-rows mode only).
        std::vector<ovx_primpath_t> tombstoned;
        // false: a row the classifier cannot decide (undecodable payload, packed array
        // transport, a known path in a group when known rows are not expected) -> full rescan.
        bool exact = true;
        void clear()
        {
            added.clear();
            unfamiliar.clear();
            schemas.clear();
            types.clear();
            tombstoned.clear();
            exact = true;
        }
    };
    // `knownRowsExpected`: the range is known to carry known-prim changes (the exact known-path
    // read saw them), so a known row is a patch input rather than a classification failure.
    bool readStructuralRows(uint64_t ord0, uint64_t ord1, StructuralRows& out, bool knownRowsExpected = false);
    void classifyStructuralGroup(const ovstage_read_group_t& group,
                                 ReadListMemo& listMemo,
                                 ovx_token_t schemasTok,
                                 ovx_token_t primTypeTok,
                                 std::unordered_map<std::string, size_t>& rowIndex,
                                 StructuralRows& out,
                                 bool knownRowsExpected);
    // Interned ids of kKnownPhysicsSchemas / kKnownPhysicsPrimTypes, built once: the probe
    // classifies rows by id compare through the same dictionary the producer wrote through.
    bool buildKnownFamilyTokens();
    // Exact, cheap in steady state: a change-range read of the structural columns over a
    // whole-stage query into mStructuralRows. Returns false if the probe could not run
    // (caller falls back to the family count compare).
    bool structuralGrowthProbe(uint64_t ord0, uint64_t ord1);
    // Whole-stage rebuild of the known sets from the family filters; the only path that
    // reconciles removals and known-prim edits. Resets the self-heal counter. Drops the
    // source's schema and hierarchy caches unless the known-change read patched them exactly
    // (`exactPatch`) and the rescan found nothing the drained rows did not show: every removed
    // known path was a tombstone that read saw, every new key a live row of the range.
    bool rescanKnownPaths(uint64_t ord0,
                          uint64_t ord1,
                          std::unordered_set<std::string>& deletedPaths,
                          bool cachesInvalidated,
                          bool exactPatch);
    // Insert the probe's physics rows into the known sets without a rescan, patching the
    // source's schema memo and hierarchy cache from the same rows.
    bool applyIncrementalGrowth(uint64_t ord0, uint64_t ord1);
    // Patch the source's schema / type / hierarchy / existence memos from mStructuralRows (drops
    // the schema caches when a row cannot be applied exactly). Every path that reads the range's
    // structural rows and keeps the caches goes through it, physics family growth or not.
    void patchSourceFromStructuralRows(uint64_t ord0, uint64_t ord1);
    // False if a consumer rejected the batch (see dispatch()).
    bool dispatchStructuralAdds(const std::vector<ObjectKey>& newKeys);
    // Full drop of the source's schema / type / existence memos and hierarchy cache, for a change
    // no drained range showed; the rebuild reads latest, so it also closes any ordinal gap.
    void dropSourceCaches();
    // Patch the source's hierarchy cache from the structural rows (reusing the probe's read
    // when it ran), or drop it when the delta cannot be exact: known tombstones the source did
    // not remove exactly, a failed read, or a delta the source rejects.
    void refreshHierarchyCache(uint64_t ord0,
                               uint64_t ord1,
                               const std::unordered_set<std::string>& knownDeletedPaths,
                               bool knownRowsExpected = false);
    // Discovery over both handles as one union; fails closed (previous set stands) if either fetch fails.
    bool refreshDiscoveryFromQuery(const PathListQuery& query);
    void setDiscoveredValueAttributes(std::vector<ovx_token_t>& target, const ovx_token_t* attrs, size_t count);
    PathListHandle buildPathListQuery(const std::vector<const std::string*>& paths);
    // Bring `query` up to date with `paths`: rebuild only the delta from the recorded adds while
    // it stays small, else fold everything into a new base. False leaves it dirty.
    bool syncPathListQuery(const std::unordered_set<std::string>& paths, PathListQuery& query);
    // Runs `read(handle, attrs, err)` over base then delta; a WRITE_FLOOR_VIOLATION /
    // OP_FAILED retries only that handle with the sealed subset (computed once).
    template <class ReadFn>
    bool readPathListQuery(const PathListQuery& query,
                           const std::vector<ovx_token_t>& attrs,
                           uint64_t ord0,
                           ReadFn read);
    ovx_token_t internOvxToken(std::string_view attrName) const;
    ObjectKey resolveGroupPathKey(ovx_primpath_t raw);
    TokenId propertyTokenFor(ovx_token_t attr);
    // Rows -> keys. A group over `handle`'s own list, or over ovstage's same-sized copy of it,
    // indexes the captured paths; any other list is fetched from the dictionary once per read.
    void collectGroupKeys(const ovstage_read_group_t& group,
                          std::vector<ObjectKey>& keys,
                          const PathListHandle* handle = nullptr);
    void rebuildRegisteredAttributeTokens();
    void rebuildValueReadAttributes();
    std::vector<ObjectKey> collectPathsFromGroup(const ovstage_read_group_t& group,
                                                 ReadListMemo& listMemo,
                                                 std::unordered_set<std::string>& physicsPaths,
                                                 std::unordered_set<std::string>& transformPaths,
                                                 const std::unordered_set<std::string>* oldPhysicsPaths);
    bool hasAttributeFloorAtOrAfter(ovx_token_t attr, uint64_t ordinal) const;
    // True iff `ord0` is at or above ovstage's retained-history frontier
    // (get_oldest_preserved_ordinal). Change membership is only exact at/above that
    // frontier; older history may be coalesced or discarded. Fails closed (false) when
    // the frontier cannot be determined.
    bool rangeWithinRetainedHistory(uint64_t ord0) const;
    bool structuralAttributesChanged(uint64_t ordinal);
    bool reconcileStructuralChanges(uint64_t ord0, uint64_t ord1);
    // `outExactPatch`: the source's schema / type / existence memos were patched exactly from the
    // known rows and tombstones (else they were dropped).
    bool readKnownStructuralChanges(uint64_t ord0,
                                    uint64_t ord1,
                                    bool* sawChange,
                                    std::unordered_set<std::string>& deletedPaths,
                                    bool* outInvalidatedCaches = nullptr,
                                    bool* outExactPatch = nullptr);
    bool rememberTransformPathAndAncestors(std::string_view path);
    // With `query`, every path newly inserted into `paths` is reported to it (noteAdded).
    bool rememberTransformPathAndAncestors(std::unordered_set<std::string>& paths,
                                           std::string_view path,
                                           PathListQuery* query = nullptr);
    bool readValueChanges(uint64_t ord0, uint64_t ord1);
    const std::vector<ovx_token_t>& transformAttrTokens();
    bool readKnownTransformChanges(uint64_t ord0, uint64_t ord1);

    OvstageSource&            mSource;
    ovstage_instance_t*       mInstance = nullptr;
    ovx_path_dictionary_t*    mDict = nullptr;
    std::vector<Registration> mRegistrations;
    std::vector<ovx_token_t> mRegisteredAttributeTokens;
    bool mRegisteredAttributeTokensDirty = true;
    bool mHasWildcardRegistration = false;
    // Registered attributes actually authored on the known physics prims: the only
    // columns a change-range read can return, so the only ones worth asking for.
    std::vector<ovx_token_t> mValueReadAttrs;
    bool mValueReadAttrsDirty = true;
    std::vector<ovx_token_t> mPathListDiscoveredAttrs;
    bool mUsePathListDiscovery = false;
    // Read groups are held for the duration of a read and released together (see the
    // definition for why the order matters).
    void releaseDeferredGroups();
    // false once a family query reported zero matched prims while its read produced
    // rows; the count compare is then untrustworthy and a floor advance always rescans.
    bool mFamilyPrimCountReliable = true;
    std::vector<ovx_token_t> mTransformAttrToks;
    std::unordered_map<ovx_token_t, TokenId> mPropertyTokens;
    std::vector<ObjectKey> mGroupKeys;
    // The current group's list entries as collectGroupKeys resolved them (a view into the
    // handle's captured paths, or into the per-read memo), for the bucket seed.
    const ovx_primpath_t* mGroupListPaths = nullptr;
    size_t mGroupListCount = 0;
    // Per read, per foreign list handle: either "same size as the query list, index the
    // captured entries" or the entries fetched once. Cleared with the deferred groups.
    struct ListMemo
    {
        bool sameAsHandle = false;
        std::vector<ovx_primpath_t> paths;
    };
    std::unordered_map<ovx_primpath_list_t, ListMemo> mListMemo;
    std::vector<ovstage_read_group_t> mDeferredGroups;
    // Every insert into a path set must reach the matching query's noteAdded, every wholesale
    // replacement its noteReplaced, or the base/delta handles silently stop covering the set.
    std::unordered_set<std::string> mKnownPhysicsPaths;
    std::unordered_set<std::string> mKnownTransformPaths;
    PathListQuery              mKnownPhysicsQuery;
    PathListQuery              mKnownTransformQuery;
    CachedFilterQuery          mSchemaFamilyQuery;
    CachedFilterQuery          mTypeFamilyQuery;
    std::unordered_map<uint64_t, ObjectKey> mResolvedPathKeys;
    // Per-drain: the structural rows the growth probe read, kept so the hierarchy
    // refresh applies them without a second ovstage read.
    StructuralRows            mStructuralRows;
    bool                      mStructuralRowsValid = false;
    std::vector<ovx_token_t>  mKnownSchemaTokens; // sorted
    std::vector<ovx_token_t>  mKnownTypeTokens;   // sorted
    bool                      mKnownFamilyTokensBuilt = false;
    // False while the known sets may be partial (a seed or rescan scan failed): the next
    // drain then rescans before its value and transform reads instead of extending a set
    // it cannot trust.
    bool                      mKnownSetAuthoritative = false;
    // Incremental reconciles since the last full rescan. Every K-th classified structural
    // drain (one whose probe saw an unfamiliar row, physics or not) rescans instead, so a
    // membership change the probe cannot see -- a classifier/filter divergence, or a
    // structural write in an ordinal the caller never passed to updateFromOvStage -- is
    // healed within K classified drains, and never if no further classified drain occurs.
    // Value-only and transform-only drains do not count and never trigger a rescan.
    uint32_t                  mIncrementalReconciles = 0;
    static constexpr uint32_t kStructuralSelfHealPeriod = 64;
    // End of the last successfully drained range (the attach ordinal before any drain). A drain
    // that does not start at mLastDrainedOrd1 + 1 skipped ordinals whose known-prim edits no
    // read of ours saw: the memos then stay suspect (mRangeGapSinceDrop) until a rescan drops
    // them, and no rescan in between may keep its exact patch.
    uint64_t                  mLastDrainedOrd1 = 0;
    bool                      mRangeGapSinceDrop = false;
    OnGroupCompleteFn         mGroupComplete;
    bool                      mEnabled = true;
};

} // namespace omni::physics::ovstage
