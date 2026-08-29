// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

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

class OvstageSource;

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
    };

    void dispatch(const ChangeBatch& batch) const;
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
    void dirtyCachedFamilyQueries();
    ovstage_query_handle_t queryForFilter(const char* attrName,
                                          ovstage_filter_op_t op,
                                          const ovx_string_t* values,
                                          size_t valueCount,
                                          CachedFilterQuery& query);
    ovstage_query_handle_t queryForPaths(const std::unordered_set<std::string>& paths,
                                         ovstage_query_handle_t& query,
                                         bool& dirty);
    ovx_token_t internOvxToken(std::string_view attrName) const;
    ObjectKey resolveGroupPathKey(ovx_primpath_t raw);
    void rebuildRegisteredAttributeTokens();
    std::vector<ovx_token_t> buildReadAttributes(const CachedFilterQuery& query);
    std::vector<ObjectKey> collectPathsFromGroup(const ovstage_read_group_t& group,
                                                 std::unordered_set<std::string>& physicsPaths,
                                                 std::unordered_set<std::string>& transformPaths,
                                                 const std::unordered_set<std::string>* oldPhysicsPaths);
    bool hasAttributeFloorAtOrAfter(ovx_token_t attr, uint64_t ordinal) const;
    bool anyAttributeFloorAtOrAfter(const std::vector<ovx_token_t>& attrs, uint64_t ordinal) const;
    bool structuralAttributesChanged(uint64_t ordinal);
    bool reconcileStructuralChanges(uint64_t ord0, uint64_t ord1);
    bool readKnownStructuralChanges(uint64_t ord0,
                                    uint64_t ord1,
                                    bool* sawChange,
                                    std::unordered_set<std::string>& deletedPaths);
    bool rememberTransformPathAndAncestors(std::string_view path);
    bool rememberTransformPathAndAncestors(std::unordered_set<std::string>& paths, std::string_view path);
    void forgetPathsFromGroup(const ovstage_read_group_t& group);
    bool readCachedFamilyChanges(CachedFilterQuery& query,
                                 uint64_t ord0,
                                 uint64_t ord1,
                                 std::unordered_map<ovx_token_t, bool>& attributeFloorCache);
    bool readKnownTransformChanges(uint64_t ord0, uint64_t ord1);

    OvstageSource&            mSource;
    ovstage_instance_t*       mInstance = nullptr;
    ovx_path_dictionary_t*    mDict = nullptr;
    std::vector<Registration> mRegistrations;
    std::vector<ovx_token_t> mRegisteredAttributeTokens;
    bool mRegisteredAttributeTokensDirty = true;
    bool mHasWildcardRegistration = false;
    std::unordered_set<std::string> mKnownPhysicsPaths;
    std::unordered_set<std::string> mKnownTransformPaths;
    ovstage_query_handle_t     mKnownPhysicsQuery = OVSTAGE_INVALID_QUERY_HANDLE;
    ovstage_query_handle_t     mKnownTransformQuery = OVSTAGE_INVALID_QUERY_HANDLE;
    bool                       mKnownPhysicsQueryDirty = true;
    bool                       mKnownTransformQueryDirty = true;
    CachedFilterQuery          mSchemaFamilyQuery;
    CachedFilterQuery          mTypeFamilyQuery;
    std::unordered_map<uint64_t, ObjectKey> mResolvedPathKeys;
    OnGroupCompleteFn         mGroupComplete;
    bool                      mEnabled = true;
};

} // namespace omni::physics::ovstage
