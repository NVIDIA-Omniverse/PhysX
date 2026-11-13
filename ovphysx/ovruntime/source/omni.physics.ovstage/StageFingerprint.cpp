// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @implements REQ-SOURCE-IDENTITY-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-9 AC-10 AC-11 AC-12 AC-14
 * @implements REQ-CACHE-ATTACH-001
 * @covers AC-3
 */
#include <omni/physics/ovstage/StageFingerprint.h>
#include "OvstageSource.h"
#include "ReadColumn.h"
#include "ReadGroupUtils.h"
#include <ovstage/ovstage_instancing.h>
#include <carb/logging/Log.h>
#include <carb/profiler/Profile.h>

#include <algorithm>
#include <atomic>
#include <cstring>
#include <limits>
#include <mutex>
#include <optional>
#include <future>
#include <thread>
#include <unordered_map>
#include <unordered_set>

namespace omni::physics::ovstage
{
namespace
{
using parse::cache::Digest;
using parse::cache::Writer;
std::atomic<size_t> captureCalls{ 0 };
std::atomic<size_t> aliasIndexBuilds{ 0 };
bool wait(ovstage_instance_t* instance, ovstage_enqueue_result_t op)
{
    if (op.status != OVSTAGE_OK)
        return false;
    ovstage_op_wait_result_t result{};
    const bool success = ovstage_wait_op(instance, op.op_index, OVSTAGE_TIMEOUT_INFINITE, &result) == OVSTAGE_OK &&
                         result.error_op_id_count == 0;
    ovstage_release_op(instance, op.op_index);
    return success;
}
struct ReadScope
{
    ovstage_instance_t* instance;
    ovstage_query_handle_t query = OVSTAGE_INVALID_QUERY_HANDLE;
    ovstage_read_handle_t read = OVSTAGE_INVALID_READ_HANDLE;
    ~ReadScope()
    {
        if (read != OVSTAGE_INVALID_READ_HANDLE)
            wait(instance, ovstage_release_read(instance, read));
        if (query != OVSTAGE_INVALID_QUERY_HANDLE)
            wait(instance, ovstage_release_query(instance, query));
    }
};
// Prim existence, hierarchy, type and schemas are always part of the identity.
constexpr const char* kStructuralColumns[] = { "usd-path", "usd-parent", "usd-children", "usd-prim-type", "usd-schemas" };
// Backend-computed hierarchy columns are determined by live path membership.
bool isDerivedHierarchyColumn(std::string_view name)
{
    return name == "usd-path" || name == "usd-parent" || name == "usd-children";
}
// The columns whose distinct values eligibility classifies; an instance root's
// authored type is in usd-metadata:typeName.
bool recordsNames(std::string_view name)
{
    return name == "usd-prim-type" || name == "usd-metadata:typeName" || name == "usd-schemas";
}
std::string relativePath(std::string_view path, std::string_view root)
{
    if (!root.empty() &&
        (path == root || (path.size() > root.size() && path.substr(0, root.size()) == root && path[root.size()] == '/')))
        return std::string(parse::cache::kUnitRoot).append(path.substr(root.size()));
    return std::string(path);
}
// Native-instancing prototypes get generated `/__Prototype_<n>` names that change
// between populations. Name each prototype by its smallest instance root's logical
// path instead; an instance root references exactly one prototype, so the name is
// unique, and nested prototypes resolve recursively. The full instance-root list of
// every named prototype enters the identity, so retargeting an instance changes it.
// Generated prototype root -> its canonical name, keyed by the root's path handle.
using KnownPrototypeNames = std::unordered_map<ovx_primpath_t, std::string>;
class PrototypeNames
{
public:
    // Known names, from an identity of this same population, are never re-queried.
    PrototypeNames(ovstage_instance_t* instance,
                   ovx_path_dictionary_t* dictionary,
                   const KnownPrototypeNames* known = nullptr)
        : mInstance(instance), mDictionary(dictionary)
    {
        if (known)
            for (const auto& [root, name] : *known)
                mPrototypes[root].name = name;
    }
    // False when a prototype has no instance roots or cannot be resolved.
    bool canonical(std::string_view path, std::string& out)
    {
        std::lock_guard<std::recursive_mutex> lock(mMutex);
        return resolve(path, out, 0);
    }
    // Every prototype named by this object with its sorted canonical instance roots,
    // in name order: the instancing links the identity hashes.
    std::vector<std::pair<std::string_view, const std::vector<std::string>*>> links() const
    {
        std::vector<std::pair<std::string_view, const std::vector<std::string>*>> out;
        for (const auto& entry : mPrototypes)
            if (!entry.second.instanceRoots.empty())
                out.emplace_back(entry.second.name, &entry.second.instanceRoots);
        std::sort(out.begin(), out.end());
        return out;
    }
    KnownPrototypeNames takeNames()
    {
        KnownPrototypeNames names;
        for (auto& [root, prototype] : mPrototypes)
            names.emplace(root, std::move(prototype.name));
        mPrototypes.clear();
        return names;
    }

private:
    struct Prototype
    {
        std::string name;
        std::vector<std::string> instanceRoots; // empty for a known name
    };
    static constexpr std::string_view kPrefix = "/__Prototype_";
    bool resolve(std::string_view path, std::string& out, int depth)
    {
        if (path.compare(0, kPrefix.size(), kPrefix) != 0)
        {
            out.assign(path);
            return true;
        }
        const size_t end = std::min(path.find('/', 1), path.size());
        ovx_primpath_t root = OVX_INVALID_PRIMPATH;
        if (ovx_path_dictionary_intern_path(mDictionary, { path.data(), end }, &root) != OVX_OK ||
            root == OVX_INVALID_PRIMPATH)
            return false;
        auto found = mPrototypes.find(root);
        if (found == mPrototypes.end())
        {
            Prototype prototype;
            if (depth > parse::cache::kMaxPrototypeNesting || !name(root, prototype, depth))
                return false;
            found = mPrototypes.emplace(root, std::move(prototype)).first;
        }
        out = found->second.name;
        out.append(path.substr(end));
        return true;
    }
    bool name(ovx_primpath_t root, Prototype& out, int depth)
    {
        ovx_primpath_list_t list = OVX_INVALID_PRIMPATH_LIST;
        if (ovstage_instancing_get_instance_roots(mInstance, root, &list) != OVSTAGE_OK)
            return false;
        const ovx_primpath_t* data = nullptr;
        size_t count = 0;
        std::vector<std::string> roots;
        const bool listed = ovx_path_dictionary_get_paths(mDictionary, list, &data, &count) == OVX_OK;
        if (listed)
            for (size_t i = 0; i < count; ++i)
            {
                ovx_string_t text{};
                if (ovx_path_dictionary_path_to_string(mDictionary, data[i], &text) != OVX_OK || !text.ptr)
                {
                    roots.clear();
                    break;
                }
                roots.emplace_back(text.ptr, text.length); // copy before the next dictionary call
            }
        ovx_path_dictionary_destroy_path_list(mDictionary, list);
        if (!listed || roots.empty() || roots.size() != count)
            return false;
        for (auto& instanceRoot : roots)
        {
            std::string canonicalRoot;
            if (!resolve(instanceRoot, canonicalRoot, depth + 1))
                return false;
            instanceRoot = std::move(canonicalRoot);
        }
        std::sort(roots.begin(), roots.end());
        out.name = "/__Prototype(" + roots.front() + ")";
        out.instanceRoots = std::move(roots);
        return true;
    }
    ovstage_instance_t* mInstance;
    ovx_path_dictionary_t* mDictionary;
    std::recursive_mutex mMutex;
    std::unordered_map<ovx_primpath_t, Prototype> mPrototypes;
};
// Dictionary IDs are local to this read. Resolve each once, but sort/hash the
// logical strings so allocation history never enters the persistent identity.
class TextMemo
{
public:
    explicit TextMemo(ovx_path_dictionary_t* dictionary, const TextMemo* sharedPaths = nullptr)
        : mDictionary(dictionary), mSharedPaths(sharedPaths)
    {
    }
    bool seedPaths(const std::vector<uint64_t>& ids, const OvstageSource* source = nullptr)
    {
        if (source)
        {
            mBorrowedPaths.reserve(ids.size());
            source->borrowCachedPathTexts(ids, mBorrowedPaths);
            std::vector<uint64_t> missing;
            for (const auto id : ids)
                if (!mBorrowedPaths.count(id))
                    missing.push_back(id);
            return seedPaths(missing);
        }
        mPaths.reserve(mPaths.size() + ids.size());
        if (ids.empty())
            return true;
        const size_t batchSize = std::min<size_t>(1024, ids.size());
        std::vector<ovx_primpath_t> handles(batchSize);
        std::vector<ovx_token_t> tokens(batchSize * 16);
        std::vector<ovx_token_t*> starts(batchSize);
        std::vector<size_t> sizes(batchSize);
        std::vector<ovx_string_t> strings;
        size_t offset = 0;
        while (offset < ids.size())
        {
            const size_t requested = std::min(batchSize, ids.size() - offset);
            for (size_t i = 0; i < requested; ++i)
                handles[i] = ovx_primpath_t{ ids[offset + i] };
            size_t processed = 0;
            if (ovstage_compat::status(mDictionary, path_dictionary_get_tokens_from_paths(
                                                        mDictionary, handles.data(), requested, tokens.data(),
                                                        tokens.size(), starts.data(), sizes.data(), &processed)) != OVX_OK)
                return false;
            if (!processed)
            {
                if (tokens.size() > std::numeric_limits<size_t>::max() / 2)
                    return false;
                tokens.resize(tokens.size() * 2);
                continue;
            }
            if (processed > requested)
                return false;
            size_t used = 0;
            for (size_t i = 0; i < processed; ++i)
            {
                if (sizes[i] && (!starts[i] || starts[i] < tokens.data() || starts[i] > tokens.data() + tokens.size() ||
                                 sizes[i] > size_t(tokens.data() + tokens.size() - starts[i])))
                    return false;
                if (sizes[i])
                    used = std::max(used, size_t(starts[i] - tokens.data()) + sizes[i]);
            }
            strings.resize(used);
            if (used &&
                ovstage_compat::status(mDictionary, path_dictionary_get_strings_from_tokens(
                                                        mDictionary, tokens.data(), used, strings.data())) != OVX_OK)
                return false;
            for (size_t i = 0; i < processed; ++i)
            {
                std::string text;
                if (!sizes[i])
                    text = "/";
                else
                    for (size_t j = 0; j < sizes[i]; ++j)
                    {
                        const auto& segment = strings[size_t(starts[i] - tokens.data()) + j];
                        text.push_back('/');
                        if (segment.length && !segment.ptr)
                            return false;
                        if (segment.length)
                            text.append(segment.ptr, segment.length);
                    }
                mPaths.emplace(ids[offset + i], std::move(text));
            }
            offset += processed;
        }
        return true;
    }
    size_t borrowedPathCount() const
    {
        return mBorrowedPaths.size();
    }
    const std::string* get(uint64_t id, bool path)
    {
        if (!id)
            return &mEmpty;
        if (path)
        {
            const auto found = mBorrowedPaths.find(id);
            if (found != mBorrowedPaths.end())
                return found->second;
        }
        if (path && mSharedPaths)
        {
            const auto borrowed = mSharedPaths->mBorrowedPaths.find(id);
            if (borrowed != mSharedPaths->mBorrowedPaths.end())
                return borrowed->second;
            const auto shared = mSharedPaths->mPaths.find(id);
            if (shared != mSharedPaths->mPaths.end())
                return &shared->second;
        }
        std::unordered_map<uint64_t, std::string>& table = path ? mPaths : mTokens;
        const auto found = table.find(id);
        if (found != table.end())
            return &found->second;
        ovx_string_t value{};
        const ovx_api_status_t status = path ? ovx_path_dictionary_path_to_string(mDictionary, ovx_primpath_t{ id }, &value) :
                                               ovx_path_dictionary_token_to_string(mDictionary, ovx_token_t{ id }, &value);
        if (status != OVX_OK || !value.ptr)
            return nullptr;
        // References remain valid across unordered_map rehashes.
        return &table.emplace(id, std::string(value.ptr, value.length)).first->second;
    }

private:
    ovx_path_dictionary_t* mDictionary;
    const TextMemo* mSharedPaths;
    std::string mEmpty;
    std::unordered_map<uint64_t, std::string> mPaths, mTokens;
    std::unordered_map<uint64_t, const std::string*> mBorrowedPaths;
};
// A distinct value of a type or schema column: the rows that hold it and the logical
// rank of the first of them.
struct CountedText
{
    std::string text;
    uint64_t rows = 0;
    uint32_t first = 0;
};
// Numeric text references use a sorted logical vocabulary, never ovstage IDs.
// Patch temporary codes after all retained groups have contributed their text.
class CanonicalTextCodes
{
public:
    explicit CanonicalTextCodes(std::string_view root, PrototypeNames* prototypes = nullptr)
        : mRoot(root), mPrototypes(prototypes)
    {
    }
    // Also count each value's rows and keep its first row, given by row() before the
    // row's values are appended, so a reader of the distinct values visits no row. A
    // row that row() leaves uncounted is still encoded.
    void countRows()
    {
        mCounting = true;
    }
    void row(uint32_t rank, bool counted = true)
    {
        mRow = rank;
        mCountRow = counted;
    }
    // The distinct texts of the counted rows with their counts; rows come in logical
    // order, so the first row of a text is its smallest counted rank.
    std::vector<CountedText> counted() const
    {
        std::vector<CountedText> out;
        if (!mCounting)
            return out;
        std::vector<uint32_t> order(mTexts.size());
        for (uint32_t i = 0; i < order.size(); ++i)
            order[i] = i;
        std::sort(order.begin(), order.end(), [&](uint32_t a, uint32_t b) { return mTexts[a] < mTexts[b]; });
        for (uint32_t code : order)
        {
            if (!mRows[code])
                continue;
            if (out.empty() || out.back().text != mTexts[code])
                out.push_back({ mTexts[code], 0, mFirstRows[code] });
            out.back().rows += mRows[code];
            out.back().first = std::min(out.back().first, mFirstRows[code]);
        }
        return out;
    }
    bool append(Writer& writer, const std::string* text, bool path)
    {
        auto& memo = path ? mPaths : mTokens;
        auto found = memo.find(text);
        if (found == memo.end())
        {
            if (mTexts.size() >= std::numeric_limits<uint32_t>::max())
                return false;
            const uint32_t code = uint32_t(mTexts.size());
            if (!path)
                mTexts.emplace_back(*text);
            else
            {
                std::string logical;
                if (mPrototypes)
                {
                    if (!mPrototypes->canonical(*text, logical))
                        return false;
                }
                else if (text->find("/__Prototype_") != std::string::npos)
                    return false;
                else
                    logical = *text;
                mTexts.emplace_back(relativePath(logical, mRoot));
            }
            found = memo.emplace(text, code).first;
            if (mCounting)
            {
                mRows.push_back(0);
                mFirstRows.push_back(0);
            }
        }
        if (mCounting && mCountRow && !mRows[found->second]++)
            mFirstRows[found->second] = mRow;
        mFixups.push_back({ writer.data().size(), found->second });
        writer.u32(found->second);
        return true;
    }
    void finish(parse::cache::Bytes& values, Writer& dictionary)
    {
        std::vector<uint32_t> order(mTexts.size()), rank(mTexts.size());
        for (uint32_t i = 0; i < order.size(); ++i)
            order[i] = i;
        std::sort(order.begin(), order.end(), [&](uint32_t a, uint32_t b) { return mTexts[a] < mTexts[b]; });
        std::vector<uint32_t> unique;
        for (uint32_t code : order)
        {
            if (unique.empty() || mTexts[code] != mTexts[unique.back()])
                unique.push_back(code);
            rank[code] = uint32_t(unique.size() - 1);
        }
        dictionary.u64(unique.size());
        for (uint32_t code : unique)
            dictionary.string(mTexts[code]);
        for (const auto& fixup : mFixups)
        {
            const uint32_t code = rank[fixup.second];
            for (size_t i = 0; i < 4; ++i)
                values[fixup.first + i] = uint8_t(code >> (8 * i));
        }
    }

private:
    std::string_view mRoot;
    PrototypeNames* mPrototypes;
    std::unordered_map<const std::string*, uint32_t> mPaths, mTokens;
    std::vector<std::string> mTexts;
    std::vector<std::pair<size_t, uint32_t>> mFixups;
    bool mCounting = false, mCountRow = true;
    uint32_t mRow = 0;
    std::vector<uint64_t> mRows;
    std::vector<uint32_t> mFirstRows;
};
bool encodeValues(Writer& w,
                  TextMemo& texts,
                  const uint8_t* data,
                  size_t count,
                  DLDataType dtype,
                  ovstage_attribute_semantic_t semantic,
                  const std::string& attribute,
                  CanonicalTextCodes& codes)
{
    const bool path = semantic == OVSTAGE_SEMANTIC_RELATIONSHIP_PATH_ID ||
                      semantic == OVSTAGE_SEMANTIC_CONNECTION_PATH_ID || isDerivedHierarchyColumn(attribute);
    const bool token = semantic == OVSTAGE_SEMANTIC_TOKEN_ID || semantic == OVSTAGE_SEMANTIC_ASSET_PATH_ID ||
                       semantic == OVSTAGE_SEMANTIC_PATH_EXPRESSION_STRING || attribute == "usd-prim-type" ||
                       attribute == "usd-schemas";
    if (semantic > OVSTAGE_SEMANTIC_FRAME || ((path || token) && (dtype.code != kDLUInt || dtype.bits != 64)))
        return false;
    // A connection row is one (path id, property token id) pair.
    const bool connection = semantic == OVSTAGE_SEMANTIC_CONNECTION_PATH_ID;
    if (connection && (dtype.lanes != 2 || count % 2))
        return false;
    // A 64-bit column without a semantic may hold dictionary IDs, which differ
    // between populations. The physics schemas declare one numeric uint64.
    if (!path && !token && semantic == OVSTAGE_SEMANTIC_NONE && dtype.code == kDLUInt && dtype.bits == 64 &&
        attribute != "physxScene:gpuTempBufferCapacity")
        return false;
    if (!path && !token && dtype.code != kDLFloat && dtype.code != kDLInt && dtype.code != kDLUInt &&
        dtype.code != kDLBool)
        return false;
    if (dtype.bits != 8 && dtype.bits != 16 && dtype.bits != 32 && dtype.bits != 64)
        return false;
    w.u64(count);
    const uint32_t endian = 1;
    if (!path && !token && *reinterpret_cast<const uint8_t*>(&endian))
    {
        if (count)
            w.bytes(data, count * (dtype.bits / 8));
        return true;
    }
    for (size_t i = 0; i < count; ++i)
    {
        if (path || token)
        {
            uint64_t id;
            std::memcpy(&id, data + i * 8, 8);
            const bool isPath = path && (!connection || i % 2 == 0);
            const std::string* text = texts.get(id, isPath);
            if (!text || !codes.append(w, text, isPath))
                return false;
        }
        else
        {
            // Read native integers of each width before emitting little-endian bytes;
            // this preserves every IEEE payload bit, including signed zero and NaNs.
            const uint8_t* scalar = data + i * (dtype.bits / 8);
            uint64_t value = 0;
            switch (dtype.bits)
            {
            case 8:
                value = *scalar;
                break;
            case 16: {
                uint16_t v;
                std::memcpy(&v, scalar, 2);
                value = v;
                break;
            }
            case 32: {
                uint32_t v;
                std::memcpy(&v, scalar, 4);
                value = v;
                break;
            }
            case 64:
                std::memcpy(&value, scalar, 8);
                break;
            }
            for (size_t byte = 0; byte < dtype.bits / 8; ++byte)
                w.u8(uint8_t(value >> (byte * 8)));
        }
    }
    return true;
}
} // namespace

namespace
{
// Own a compact scalar type column after releasing the backend's one-group-per-
// prim transport. Each worker uses a disjoint explicit path list and the same
// sealed ordinal range. No type is inferred from queries or runtime objects.
struct CompactTypes
{
    std::vector<uint64_t> paths, tokens;
    DLTensor tensor{};
    int64_t shape = 0;
    ovstage_attribute_semantic_t semantic{};
    bool valid = false, sealed = true;
    void publish()
    {
        shape = int64_t(tokens.size());
        tensor.data = tokens.data();
        tensor.device = { kDLCPU, 0 };
        tensor.ndim = 1;
        tensor.dtype = { kDLUInt, 64, 1 };
        tensor.shape = &shape;
        valid = true;
    }
};
bool readLivePaths(ovstage_instance_t* instance,
                   ovx_path_dictionary_t* dictionary,
                   ovstage_query_handle_t query,
                   ovx_token_t attribute,
                   ovstage_ordinal_t ordinal,
                   std::vector<uint64_t>& paths)
{
    ReadScope scope{ instance };
    if (!wait(instance, ovstage_read_attributes(instance, query, &attribute, 1, { 0, ordinal }, &scope.read)))
        return false;
    ReadListMemo lists(dictionary);
    ReadColumnCursor cursor(instance, scope.read);
    for (;;)
    {
        ReadGroupColumn group{};
        const auto fetched = cursor.next(group);
        if (fetched == OVSTAGE_ERROR_END_OF_ITERATION)
            return true;
        if (fetched != OVSTAGE_OK)
            return false;
        if (group.attribute != attribute)
            return false;
        if (group.is_delete)
            continue;
        ReadColumn column;
        const ovx_primpath_t* members = nullptr;
        size_t memberCount = 0;
        if (!readColumn(group, column) || column.components != 1 || column.tensor->dtype.code != kDLUInt ||
            column.tensor->dtype.bits != 64 || !lists.paths(group.prims.list, &members, &memberCount))
            return false;
        for (uint32_t i = 0; i < group.prims.count; ++i)
        {
            if (!column.present(i))
                continue;
            const uint64_t index = group.prims.index_map ? group.prims.index_map[i] : uint64_t(group.prims.offset) + i;
            if (index >= memberCount)
                return false;
            paths.push_back(members[index].raw_id);
        }
    }
}
std::unique_ptr<CompactTypes> readCompactTypes(ovstage_instance_t* instance,
                                               ovx_path_dictionary_t* dictionary,
                                               ovx_token_t attribute,
                                               ovstage_ordinal_t ordinal,
                                               const uint64_t* paths,
                                               size_t count)
{
    auto out = std::make_unique<CompactTypes>();
    out->paths.reserve(count);
    out->tokens.reserve(count);
    ReadScope scope{ instance };
    std::vector<ovx_primpath_t> handles;
    handles.reserve(count);
    for (size_t i = 0; i < count; ++i)
        handles.push_back(ovx_primpath_t{ paths[i] });
    ovx_primpath_list_t list = OVX_INVALID_PRIMPATH_LIST;
    if (ovx_path_dictionary_create_path_list(dictionary, handles.data(), count, &list) != OVX_OK)
        return out;
    const auto status = ovstage_query_from_path_list(instance, list, &scope.query);
    ovx_path_dictionary_destroy_path_list(dictionary, list);
    if (status != OVSTAGE_OK ||
        !wait(instance, ovstage_read_attributes(instance, scope.query, &attribute, 1, { 0, ordinal }, &scope.read)))
        return out;
    ReadListMemo indexedLists(dictionary);
    std::vector<ovx_primpath_t> memberBuffer;
    bool haveLayout = false;
    ReadColumnCursor cursor(instance, scope.read);
    for (;;)
    {
        ReadGroupColumn group{};
        const auto fetched = cursor.next(group);
        if (fetched == OVSTAGE_ERROR_END_OF_ITERATION)
            break;
        if (fetched != OVSTAGE_OK)
            return out;
        if (group.attribute != attribute)
            return out;
        if (group.meta.attribute_write_floor_ordinal < ordinal)
        {
            out->sealed = false;
            return out;
        }
        if (group.is_delete)
            continue;
        ReadColumn column;
        const ovx_primpath_t* members = nullptr;
        size_t memberCount = 0;
        if (!readColumn(group, column) || column.components != 1 || column.tensor->dtype.code != kDLUInt ||
            column.tensor->dtype.bits != 64 || column.tensor->dtype.lanes != 1)
            return out;
        if (group.prims.index_map)
        {
            if (!indexedLists.paths(group.prims.list, &members, &memberCount))
                return out;
        }
        else
        {
            // The fetched group pins its list until the cursor moves on. Copy only
            // its contiguous slice into reusable storage, then copy values out.
            memberBuffer.resize(group.prims.count);
            ovstage_compat::pathListFetchCounter().fetch_add(1, std::memory_order_relaxed);
            if (ovstage_compat::status(dictionary, path_dictionary_get_paths_from_path_list(
                                                       dictionary, group.prims.list, group.prims.offset,
                                                       group.prims.count, memberBuffer.data(), &memberCount)) != OVX_OK ||
                memberCount != group.prims.count)
                return out;
            members = memberBuffer.data();
        }
        if (haveLayout && out->semantic != group.semantic)
            return out;
        out->semantic = group.semantic;
        haveLayout = true;
        for (uint32_t i = 0; i < group.prims.count; ++i)
        {
            if (!column.present(i))
                continue;
            const uint64_t index = group.prims.index_map ? group.prims.index_map[i] : i;
            if (index >= memberCount || out->paths.size() >= count)
                return out;
            uint64_t token = 0;
            std::memcpy(&token, column.data(i), sizeof(token));
            out->paths.push_back(members[index].raw_id);
            out->tokens.push_back(token);
        }
    }
    out->publish();
    return out;
}
} // namespace
namespace
{
Digest columnDigest(size_t rows, const parse::cache::Bytes& bytes, const Writer& vocabulary)
{
    const auto vocabularyHash = parse::cache::digest(vocabulary.data().data(), vocabulary.data().size());
    const auto valuesHash = parse::cache::digest(bytes.data(), bytes.size());
    Writer column;
    column.u64(rows);
    column.u64(vocabulary.data().size());
    column.u64(bytes.size());
    column.bytes(vocabularyHash.data(), vocabularyHash.size());
    column.bytes(valuesHash.data(), valuesHash.size());
    return parse::cache::digest(column.data().data(), column.data().size());
}
const Digest& emptyColumnDigest()
{
    static const Digest value = []() {
        Writer vocabulary;
        vocabulary.u64(0);
        return columnDigest(0, {}, vocabulary);
    }();
    return value;
}
// usd-path values are exactly the canonical membership already committed.
const Digest& derivedPathDigest()
{
    static const Digest value = []() {
        constexpr char domain[] = "derived logical path membership";
        return parse::cache::digest(domain, sizeof(domain) - 1);
    }();
    return value;
}
// A bounded, immutable declaration plan owns text and canonical bytes only.
// No source, dictionary handle, discovered presence or payload survives here.
// Every capture still discovers the current vocabulary and reads current values.
struct InputColumnPlan
{
    InputColumnPlan() = default;
    // The index views the names: a copy would view the source's strings.
    InputColumnPlan(const InputColumnPlan&) = delete;
    InputColumnPlan& operator=(const InputColumnPlan&) = delete;
    std::vector<std::string> declarations, expanded, names, prefixes;
    // Views into names, so a read looks a column up without building a string.
    std::unordered_map<std::string_view, size_t> indices;
    Digest identity{};
    bool matchesPrefix(std::string_view column) const
    {
        for (const auto& prefix : prefixes)
            if (column.compare(0, prefix.size(), prefix) == 0)
                return true;
        return false;
    }
    bool declares(std::string_view column) const
    {
        return indices.count(column) || matchesPrefix(column);
    }
};
std::shared_ptr<const InputColumnPlan> inputColumnPlan(const std::vector<std::string>& columns,
                                                       const std::vector<std::string>& discovered)
{
    thread_local std::shared_ptr<const InputColumnPlan> previous;
    thread_local std::vector<std::string> previousDiscovered;
    // An unchanged sorted vocabulary expands to the same wildcard columns.
    const bool sameDeclarations = previous && previous->declarations == columns;
    if (sameDeclarations && previousDiscovered == discovered)
        return previous;
    // Both memos change only once the plan is complete, so a throw leaves no plan
    // that answers for vocabulary it was not built from.
    std::vector<std::string> discoveredCopy = discovered;
    const auto commit = [&](std::shared_ptr<const InputColumnPlan> plan) {
        previous = std::move(plan);
        previousDiscovered.swap(discoveredCopy);
        return previous;
    };
    std::vector<std::string> prefixes, expanded;
    if (sameDeclarations)
        prefixes = previous->prefixes;
    else
        for (const auto& column : columns)
            if (!column.empty() && column.back() == '*')
                prefixes.push_back(column.substr(0, column.size() - 1));
    for (const auto& name : discovered)
        for (const auto& prefix : prefixes)
            if (name.compare(0, prefix.size(), prefix) == 0)
            {
                expanded.push_back(name);
                break;
            }
    if (sameDeclarations && previous->expanded == expanded)
        return commit(previous);
    auto plan = std::make_shared<InputColumnPlan>();
    plan->declarations = columns;
    plan->expanded = std::move(expanded);
    plan->prefixes = std::move(prefixes);
    for (const auto& column : columns)
        if (column.empty() || column.back() != '*')
            plan->names.push_back(column);
    plan->names.insert(plan->names.end(), plan->expanded.begin(), plan->expanded.end());
    for (const char* name : kStructuralColumns)
        plan->names.emplace_back(name);
    std::sort(plan->names.begin(), plan->names.end());
    plan->names.erase(std::unique(plan->names.begin(), plan->names.end()), plan->names.end());
    Writer layout;
    layout.string("ovruntime.physics-input-column-plan.v1");
    layout.u64(plan->prefixes.size());
    for (const auto& prefix : plan->prefixes)
        layout.string(prefix);
    layout.u64(plan->names.size());
    for (size_t c = 0; c < plan->names.size(); ++c)
    {
        const std::string& name = plan->names[c];
        plan->indices.emplace(std::string_view(name), c);
        layout.string(name);
    }
    plan->identity = parse::cache::digest(layout.data().data(), layout.data().size());
    return commit(std::move(plan));
}
} // namespace
struct PhysicsInputSnapshot::Impl
{
    ovstage_instance_t* instance = nullptr;
    ovx_path_dictionary_t* dictionary = nullptr;
    StageFingerprint identity;
    // A column is listed with its layout in one step, so a failed append leaves
    // no entry half made. Fetched columns point into the groups `fetched` owns;
    // local ones point into compactTypes.
    struct Group
    {
        ReadGroupColumn read;
        ReadColumn layout;
    };
    std::vector<Group> groups;
    std::vector<ovstage_read_group_t> fetched;
    std::vector<std::unique_ptr<CompactTypes>> compactTypes;
    struct View
    {
        uint32_t path, group, row;
    };
    struct Column
    {
        std::vector<View> rows;
        Digest digest{};
        size_t groups = 0;
        bool valid = false;
        // A type or schema column's distinct values, recorded while it was hashed.
        std::vector<CountedText> names;
        bool named = false;
    };
    // Most declared columns are absent. Allocate retained row state only for
    // columns the backend actually returns, rather than zeroing every declaration.
    std::unordered_map<size_t, Column> data;
    std::unordered_map<uint64_t, size_t> columnIndex;
    // Canonical column rows are already sorted by logical path during hashing.
    // Reuse them on misses; only the raw/canonical handle aliases are lazy.
    std::unordered_map<uint64_t, uint32_t> values, aliases;
    std::vector<uint64_t> sourcePaths, rankedPaths;
    std::vector<bool> descendants;
    std::shared_ptr<const InputColumnPlan> plan;
    std::vector<std::string> vocabulary;
    std::unordered_set<uint64_t> scope;
    KnownPrototypeNames prototypeNames;
    mutable std::atomic<bool> complete{ true };
    std::once_flag indexOnce;
    void indexForParsing()
    {
        std::call_once(indexOnce, [&]() {
            aliasIndexBuilds.fetch_add(1, std::memory_order_relaxed);
            aliases.reserve(sourcePaths.size());
            for (const auto raw : sourcePaths)
            {
                ovx_string_t text{};
                ovx_primpath_t canonical = OVX_INVALID_PRIMPATH;
                if (ovx_path_dictionary_path_to_string(dictionary, ovx_primpath_t{ raw }, &text) != OVX_OK ||
                    ovx_path_dictionary_intern_path(dictionary, text, &canonical) != OVX_OK)
                {
                    complete = false;
                    continue;
                }
                const auto rank = values.at(raw);
                const auto entry = aliases.emplace(canonical.raw_id, rank);
                if (entry.first->second != rank)
                    complete = false;
            }
        });
    }
    std::optional<uint32_t> rankForPath(uint64_t path)
    {
        // Raw handles are immutable after capture, so capability auditing and
        // readers using them need no canonical alias construction or locking.
        const auto raw = values.find(path);
        if (raw != values.end())
            return raw->second;
        // Scoped canonical handles were bound during capture. A selected path
        // without a rank has no captured live row (for example the pseudo-root),
        // so proving its absence needs no whole-snapshot alias reconciliation.
        if (scope.count(path))
            return std::nullopt;
        indexForParsing();
        auto alias = aliases.find(path);
        if (alias != aliases.end())
            return alias->second;
        ovx_string_t text{};
        ovx_primpath_t canonical = OVX_INVALID_PRIMPATH;
        if (ovx_path_dictionary_path_to_string(dictionary, ovx_primpath_t{ path }, &text) == OVX_OK && text.ptr &&
            ovx_path_dictionary_intern_path(dictionary, text, &canonical) == OVX_OK)
        {
            alias = aliases.find(canonical.raw_id);
            if (alias != aliases.end())
                return alias->second;
        }
        return std::nullopt;
    }
    ~Impl()
    {
        if (!fetched.empty())
            ovstage_release_groups(instance, fetched.data(), fetched.size());
    }
};
PhysicsInputSnapshot::PhysicsInputSnapshot() : mImpl(new Impl)
{
}
PhysicsInputSnapshot::~PhysicsInputSnapshot() = default;
const StageFingerprint& PhysicsInputSnapshot::fingerprint() const
{
    return mImpl->identity;
}
bool PhysicsInputSnapshot::complete() const
{
    return mImpl->complete && bool(mImpl->identity);
}
bool PhysicsInputSnapshot::covers(const std::vector<uint64_t>& paths, const std::vector<std::string>& columns) const
{
    if (!complete())
        return false;
    for (const auto path : paths)
        if (!path || (!mImpl->scope.count(path) && !mImpl->rankForPath(path)))
            return false;
    for (const auto& name : columns)
    {
        const auto column = conv::toOvstageColumn(name);
        if (column == "usd-parent" || column == "usd-children" || !mImpl->plan->declares(column))
            return false;
    }
    return true;
}
std::optional<bool> PhysicsInputSnapshot::hasDescendants(uint64_t path) const
{
    if (!complete() || !mImpl->scope.empty())
        return std::nullopt;
    const auto rank = mImpl->rankForPath(path);
    return rank ? std::optional<bool>(mImpl->descendants[*rank]) : std::nullopt;
}
std::optional<bool> PhysicsInputSnapshot::hasStageColumn(std::string_view column, bool prefix) const
{
    if (!complete())
        return std::nullopt;
    const auto& vocabulary = mImpl->vocabulary;
    const auto it = std::lower_bound(vocabulary.begin(), vocabulary.end(), column);
    return it != vocabulary.end() && (prefix ? it->compare(0, column.size(), column) == 0 : *it == column);
}
void PhysicsInputSnapshot::observeColumn(std::string_view column, bool prefix) const
{
    if (!mImpl->plan)
    {
        mImpl->complete = false;
        return;
    }
    if ((!prefix && mImpl->plan->indices.count(column)) || mImpl->plan->matchesPrefix(column))
        return;
    CARB_LOG_VERBOSE("Compiled stage cache: the parser read undeclared column %.*s; nothing is stored",
                     int(column.size()), column.data());
    mImpl->complete = false;
}

void PhysicsInputSnapshot::observeLiveRead(std::string_view column) const
{
    CARB_LOG_VERBOSE("Compiled stage cache: the parser used column %.*s from the live stage; nothing is stored",
                     int(column.size()), column.data());
    mImpl->complete = false;
}

std::optional<bool> PhysicsInputSnapshot::withTensor(uint64_t path,
                                                     std::string_view column,
                                                     const std::function<void(const DLTensor&, uint32_t)>& fn) const
{
    Impl& impl = *mImpl;
    if (!complete())
        return std::nullopt;
    // These read-only backend columns are determined by the captured live path
    // membership. Miss-time consumers may request their ordinary lazy views;
    // no authored payload is being read again or omitted from the identity.
    if (isDerivedHierarchyColumn(column))
        return std::nullopt;
    const auto attr = impl.plan->indices.find(column);
    if (attr == impl.plan->indices.end())
    {
        if (impl.plan->matchesPrefix(column))
            return false;
        CARB_LOG_VERBOSE("Compiled stage cache: the parser read undeclared column %.*s; nothing is stored",
                         int(column.size()), column.data());
        impl.complete = false;
        return std::nullopt;
    }
    const auto prim = impl.rankForPath(path);
    if (!prim)
    {
        if (!impl.scope.empty() && !impl.scope.count(path))
        {
            CARB_LOG_VERBOSE(
                "Compiled stage cache: the parser read column %.*s outside the captured unit; nothing "
                "is stored",
                int(column.size()), column.data());
            impl.complete = false;
            return std::nullopt;
        }
        return false;
    }
    const auto captured = impl.data.find(attr->second);
    if (captured == impl.data.end())
        return false;
    const auto& rows = captured->second.rows;
    const auto entry = std::lower_bound(
        rows.begin(), rows.end(), *prim, [](const auto& row, uint32_t path) { return row.path < path; });
    if (entry == rows.end() || entry->path != *prim)
        return false;
    const auto& group = impl.groups[entry->group].read;
    const uint32_t row = entry->row;
    if (group.is_array)
    {
        const uint32_t index = group.data.index_map ? group.data.index_map[row] : row;
        fn(group.data.tensors[index], 1);
    }
    else
    {
        const ReadColumn& c = impl.groups[entry->group].layout;
        DLTensor tensor = *c.tensor;
        int64_t shape = c.components;
        tensor.ndim = 1;
        tensor.shape = &shape;
        tensor.strides = nullptr;
        tensor.data = const_cast<uint8_t*>(c.data(row));
        tensor.byte_offset = 0;
        tensor.dtype.lanes = 1;
        fn(tensor, 1);
    }
    return true;
}
bool PhysicsInputSnapshot::visitTokens(std::string_view column,
                                       const std::function<bool(uint64_t, std::string_view)>& fn) const
{
    Impl& impl = *mImpl;
    if (!complete())
        return false;
    const auto attr = impl.plan->indices.find(column);
    if (attr == impl.plan->indices.end())
    {
        impl.complete = false;
        return false;
    }
    const auto captured = impl.data.find(attr->second);
    if (captured == impl.data.end())
        return true;
    const auto& values = captured->second;
    TextMemo texts(impl.dictionary);
    for (const auto& row : values.rows)
    {
        const auto& group = impl.groups[row.group].read;
        const auto& layout = impl.groups[row.group].layout;
        const DLTensor* tensor = layout.tensor;
        const uint8_t* data = nullptr;
        size_t count = 0;
        if (group.is_array)
        {
            tensor = &group.data.tensors[group.data.index_map ? group.data.index_map[row.row] : row.row];
            count = tensor->dtype.lanes;
            // Capture already checked compactness, dimensions and overflow.
            for (int d = 0; d < tensor->ndim; ++d)
                count *= size_t(tensor->shape[d]);
            if (count)
                data = static_cast<const uint8_t*>(tensor->data) + tensor->byte_offset;
        }
        else
        {
            count = size_t(layout.components);
            data = layout.data(row.row);
        }
        if (tensor->dtype.code != kDLUInt || tensor->dtype.bits != 64)
            return false;
        for (size_t i = 0; i < count; ++i)
        {
            uint64_t value = 0;
            std::memcpy(&value, data + i * 8, 8);
            const auto* text = texts.get(value, false);
            if (!text || !fn(impl.rankedPaths[row.path], *text))
                return false;
        }
    }
    return true;
}
bool PhysicsInputSnapshot::visitTokenNames(std::string_view column, const std::function<bool(const TokenName&)>& fn) const
{
    Impl& impl = *mImpl;
    if (!complete() || !recordsNames(column))
        return false;
    const auto attr = impl.plan->indices.find(column);
    if (attr == impl.plan->indices.end())
        return false;
    const auto captured = impl.data.find(attr->second);
    if (captured == impl.data.end() || captured->second.rows.empty())
        return true;
    if (!captured->second.named)
        return false;
    for (const auto& entry : captured->second.names)
        if (!fn({ entry.text, entry.rows, impl.rankedPaths[entry.first] }))
            return false;
    return true;
}
bool PhysicsInputSnapshot::visitPathTargets(const std::function<bool(uint64_t, std::string_view, std::string_view)>& fn) const
{
    Impl& impl = *mImpl;
    if (!complete())
        return false;
    TextMemo texts(impl.dictionary);
    for (const auto& [column, values] : impl.data)
    {
        const std::string& name = impl.plan->names[column];
        if (isDerivedHierarchyColumn(name))
            continue;
        for (const auto& row : values.rows)
        {
            const auto& group = impl.groups[row.group].read;
            if (group.semantic != OVSTAGE_SEMANTIC_RELATIONSHIP_PATH_ID &&
                group.semantic != OVSTAGE_SEMANTIC_CONNECTION_PATH_ID)
                continue;
            const auto& layout = impl.groups[row.group].layout;
            const DLTensor* tensor = layout.tensor;
            const uint8_t* data = nullptr;
            size_t count = 0;
            if (group.is_array)
            {
                tensor = &group.data.tensors[group.data.index_map ? group.data.index_map[row.row] : row.row];
                count = tensor->dtype.lanes;
                for (int d = 0; d < tensor->ndim; ++d)
                    count *= size_t(tensor->shape[d]);
                if (count)
                    data = static_cast<const uint8_t*>(tensor->data) + tensor->byte_offset;
            }
            else
            {
                count = size_t(layout.components);
                data = layout.data(row.row);
            }
            if (tensor->dtype.code != kDLUInt || tensor->dtype.bits != 64)
                return false;
            // A connection row is a (path id, property token id) pair.
            const size_t stride = group.semantic == OVSTAGE_SEMANTIC_CONNECTION_PATH_ID ? 2 : 1;
            for (size_t i = 0; i < count; i += stride)
            {
                uint64_t value = 0;
                std::memcpy(&value, data + i * 8, 8);
                if (!value)
                    continue;
                const auto* text = texts.get(value, true);
                if (!text || !fn(impl.rankedPaths[row.path], name, *text))
                    return false;
            }
        }
    }
    return true;
}
bool PhysicsInputSnapshot::withScalars(const std::vector<uint64_t>& paths,
                                       const std::vector<std::string>& columns,
                                       const std::function<void(size_t, size_t, const DLTensor*, int64_t, uint32_t)>& fn) const
{
    Impl& impl = *mImpl;
    if (!complete())
        return false;
    for (const auto& column : columns)
        if (isDerivedHierarchyColumn(column))
            return false;
    struct Binding
    {
        size_t row, next;
    };
    std::vector<size_t> heads(impl.sourcePaths.size(), SIZE_MAX);
    std::vector<Binding> bindings;
    bindings.reserve(paths.size());
    for (size_t row = 0; row < paths.size(); ++row)
    {
        if (!paths[row])
            continue;
        const auto found = impl.rankForPath(paths[row]);
        if (!found)
        {
            if (!impl.scope.empty() && !impl.scope.count(paths[row]))
            {
                impl.complete = false;
                return false;
            }
            continue;
        }
        bindings.push_back({ row, heads[*found] });
        heads[*found] = bindings.size() - 1;
    }
    for (size_t a = 0; a < columns.size(); ++a)
    {
        const auto attr = impl.plan->indices.find(columns[a]);
        if (attr == impl.plan->indices.end())
        {
            if (impl.plan->matchesPrefix(columns[a]))
                continue;
            impl.complete = false;
            return false;
        }
        const auto captured = impl.data.find(attr->second);
        if (captured == impl.data.end())
            continue;
        for (const auto& cell : captured->second.rows)
        {
            if (heads[cell.path] == SIZE_MAX)
                continue;
            const auto& group = impl.groups[cell.group].read;
            const auto& layout = impl.groups[cell.group].layout;
            for (auto b = heads[cell.path]; b != SIZE_MAX; b = bindings[b].next)
            {
                if (group.is_array)
                    fn(bindings[b].row, a, nullptr, 0, 0);
                else
                    fn(bindings[b].row, a, layout.tensor, layout.components, layout.row(cell.row));
            }
        }
    }
    return true;
}
struct PhysicsInputSnapshot::StageQuery
{
    explicit StageQuery(ovstage_instance_t* instance) : scope{ instance }
    {
    }
    ReadScope scope; // holds the query open for the capture's reads
    ovstage_query_handle_t readQuery = OVSTAGE_INVALID_QUERY_HANDLE;
    size_t prims = 0;
    std::vector<std::string> discovered;
};
namespace
{
// False when the query fails or reports an attribute it cannot name: an incomplete
// discovery cannot prove absent inputs.
bool discoverStage(ovx_path_dictionary_t* dictionary, PhysicsInputSnapshot::StageQuery& stage)
{
    ovstage_instance_t* instance = stage.scope.instance;
    ovstage_filter_t filter{};
    if (!wait(instance, ovstage_query(instance, &filter, nullptr, 0, &stage.scope.query)))
        return false;
    ovstage_query_result_t queryResult{};
    if (ovstage_fetch_query_result(instance, stage.scope.query, OVSTAGE_TIMEOUT_INFINITE, &queryResult) != OVSTAGE_OK)
        return false;
    stage.prims = queryResult.total_prim_count;
    stage.readQuery = queryResult.all_handle != OVSTAGE_INVALID_QUERY_HANDLE ? queryResult.all_handle : stage.scope.query;
    bool complete = queryResult.attribute_count == 0 || queryResult.attributes;
    for (size_t i = 0; i < queryResult.attribute_count && queryResult.attributes; ++i)
    {
        ovx_string_t name{};
        if (ovx_path_dictionary_token_to_string(dictionary, queryResult.attributes[i], &name) == OVX_OK && name.ptr &&
            name.length)
            stage.discovered.emplace_back(name.ptr, name.length);
        else
            complete = false;
    }
    ovstage_release_query_result(instance, &queryResult);
    return complete;
}
} // namespace
std::shared_ptr<PhysicsInputSnapshot::StageQuery> PhysicsInputSnapshot::queryStage(ovstage_instance_t* instance,
                                                                                   size_t& prims)
{
    prims = 0;
    auto* dictionary = instance ? ovstage_get_path_dictionary(instance) : nullptr;
    if (!dictionary)
        return nullptr;
    auto stage = std::make_shared<StageQuery>(instance);
    if (!discoverStage(dictionary, *stage))
        return nullptr;
    prims = stage->prims;
    return stage;
}
std::shared_ptr<PhysicsInputSnapshot> PhysicsInputSnapshot::capture(ovstage_instance_t* instance,
                                                                    ovstage_ordinal_t ordinal,
                                                                    const std::vector<std::string>& columns,
                                                                    const std::vector<std::string>& selectedPaths,
                                                                    std::string_view relativeRoot,
                                                                    const OvstageSource* sharedSource,
                                                                    std::shared_ptr<StageQuery> stageQuery)
{
    CARB_PROFILE_ZONE(0, "PhysicsInputSnapshot::capture");
    captureCalls.fetch_add(1, std::memory_order_relaxed);
    auto snapshot = std::shared_ptr<PhysicsInputSnapshot>(new PhysicsInputSnapshot);
    Impl& impl = *snapshot->mImpl;
    StageFingerprint& result = impl.identity;
    impl.instance = instance;
    impl.dictionary = instance ? ovstage_get_path_dictionary(instance) : nullptr;
    result.bypassReason = "invalid selected input source";
    if (!instance || !ordinal || !impl.dictionary)
        return snapshot;
    auto* dictionary = impl.dictionary;
    if (sharedSource && (!sharedSource->physicsInputSharingEnabled() || sharedSource->instance() != instance ||
                         sharedSource->readOrdinal() != ordinal))
        sharedSource = nullptr;
    std::vector<ovx_primpath_t> selected;
    selected.reserve(selectedPaths.size());
    for (const auto& path : selectedPaths)
    {
        ovx_primpath_t key = OVX_INVALID_PRIMPATH;
        if (sharedSource)
            key = ovx_primpath_t{ sharedSource->findByPathRaw(path) };
        else if (ovx_path_dictionary_intern_path(dictionary, { path.data(), path.size() }, &key) != OVX_OK)
            return snapshot;
        if (key == OVX_INVALID_PRIMPATH)
            return snapshot;
        selected.push_back(key);
        impl.scope.insert(key.raw_id);
    }
    result.bypassReason = "selected input discovery failed";
    // Presence/prefix predicates are stage-global. Capture the column vocabulary
    // globally, but restrict payload reads to the unit and its context below.
    size_t discoveredPrims = selected.size();
    auto readQuery = OVSTAGE_INVALID_QUERY_HANDLE;
    std::vector<std::string> discovered;
    bool sharedVocabulary = false;
    // Only the feed's exact rows from this source/update can replace reads.
    const bool shareTypes = !selected.empty() && sharedSource && !sharedSource->sharedPhysicsInputTypes().empty();
    // A successful structural handoff proves that the feed entered this update
    // and invalidated the previous vocabulary. Reuse only a complete vocabulary
    // built since that invalidation, under the same stable source lease. A matching
    // ordinal alone cannot authorize reuse of a source's older presence answers.
    if (shareTypes)
    {
        std::lock_guard<std::recursive_mutex> lock(sharedSource->mMutex);
        if (sharedSource->mAttributeVocabularyBuilt)
        {
            discovered.assign(sharedSource->mAttributeVocabulary.begin(), sharedSource->mAttributeVocabulary.end());
            sharedVocabulary = true;
        }
    }
    if (!sharedVocabulary)
    {
        // The stage-wide discovery query, unless queryStage() ran it ahead.
        if (!stageQuery || stageQuery->scope.instance != instance)
        {
            stageQuery = std::make_shared<StageQuery>(instance);
            if (!discoverStage(dictionary, *stageQuery))
                return snapshot;
        }
        discoveredPrims = stageQuery->prims;
        readQuery = stageQuery->readQuery;
        discovered = std::move(stageQuery->discovered);
    }
    result.reusedVocabulary = sharedVocabulary;
    ReadScope selectedScope{ instance };
    if (!selected.empty())
    {
        ovx_primpath_list_t list = OVX_INVALID_PRIMPATH_LIST;
        if (ovx_path_dictionary_create_path_list(dictionary, selected.data(), selected.size(), &list) != OVX_OK)
            return snapshot;
        const auto status = ovstage_query_from_path_list(instance, list, &selectedScope.query);
        ovx_path_dictionary_destroy_path_list(dictionary, list);
        if (status != OVSTAGE_OK)
            return snapshot;
        readQuery = selectedScope.query;
    }
    std::vector<ovx_token_t> attributes;
    if (!std::is_sorted(discovered.begin(), discovered.end()))
        std::sort(discovered.begin(), discovered.end());
    impl.plan = inputColumnPlan(columns, discovered);
    const auto& names = impl.plan->names;
    std::vector<bool> present(names.size());
    std::vector<ovx_token_t> tokens(names.size(), OVX_INVALID_TOKEN);
    // Unchanged context and existing prims gaining schemas still need fresh types.
    std::unique_ptr<CompactTypes> sharedTypes;
    std::vector<uint64_t> missingTypes;
    if (shareTypes)
    {
        sharedTypes = std::make_unique<CompactTypes>();
        std::unordered_set<uint64_t> seen;
        for (const auto path : selected)
        {
            if (!seen.insert(path.raw_id).second)
                continue;
            OvstageSource::SharedInputTypeRow row;
            if (!sharedSource->sharedPhysicsInputType(path.raw_id, row))
            {
                missingTypes.push_back(path.raw_id);
                continue;
            }
            if (!row.token)
                continue; // authoritative absence in the complete type index
            if (!sharedTypes->paths.empty() && sharedTypes->semantic != row.semantic)
            {
                sharedTypes.reset();
                break;
            }
            sharedTypes->semantic = row.semantic;
            sharedTypes->paths.push_back(row.path);
            sharedTypes->tokens.push_back(row.token);
        }
        if (sharedTypes && sharedTypes->paths.empty())
            sharedTypes.reset();
    }
    const bool partitionTypes =
        !sharedTypes && (selectedPaths.empty() ? discoveredPrims >= 16384 : selectedPaths.size() >= 512);
    std::vector<size_t> presentColumns;
    for (const auto& name : discovered)
    {
        const auto found = impl.plan->indices.find(name);
        if (found == impl.plan->indices.end())
            continue;
        present[found->second] = true;
        presentColumns.push_back(found->second);
    }
    // Computed columns can be omitted from the authored vocabulary. All other
    // declared absences need neither tokens nor backend reads.
    for (const char* name : { "usd-path", "usd-prim-type", "usd-schemas" })
    {
        const auto c = impl.plan->indices.at(name);
        if (!present[c])
            presentColumns.push_back(c);
    }
    std::sort(presentColumns.begin(), presentColumns.end());
    for (const auto c : presentColumns)
    {
        const auto& name = names[c];
        auto& token = tokens[c];
        if (sharedSource)
            token = sharedSource->ovxToken(name);
        else if (ovx_path_dictionary_intern_token(dictionary, { name.data(), name.size() }, &token) != OVX_OK)
            return snapshot;
        if (token == OVX_INVALID_TOKEN)
            return snapshot;
        impl.columnIndex.emplace(token.raw_id, c);
        if (name != "usd-parent" && name != "usd-children" && (name != "usd-path" || !selectedPaths.empty()) &&
            (name != "usd-prim-type" || (!partitionTypes && !sharedTypes)))
            attributes.push_back(token);
    }
    result.bypassReason = "selected input read failed";
    const ovstage_ordinal_range_t range{ 0, ordinal };
    std::vector<uint64_t> typePaths;
    struct CanonicalPaths
    {
        explicit CanonicalPaths(ovx_path_dictionary_t* dictionary) : texts(dictionary)
        {
        }
        TextMemo texts;
        std::vector<std::string> names;
        std::vector<uint32_t> order, rank, unique;
        bool valid = false;
        bool prototype = false; // a prototype without a stable name (no resolvable instance roots)
    };
    PrototypeNames prototypes(instance, dictionary);
    auto preparePaths = [&](const std::vector<uint64_t>& paths) {
        auto data = std::make_unique<CanonicalPaths>(dictionary);
        if (paths.size() >= UINT32_MAX)
            return data;
        if (!data->texts.seedPaths(paths, sharedSource))
            return data;
        data->names.reserve(paths.size());
        for (const auto path : paths)
        {
            const auto* text = data->texts.get(path, true);
            if (!text)
                return data;
            std::string logical;
            if (!prototypes.canonical(*text, logical))
            {
                data->prototype = true;
                return data;
            }
            data->names.push_back(relativePath(logical, relativeRoot));
        }
        data->order.resize(paths.size());
        data->rank.resize(paths.size());
        for (uint32_t i = 0; i < data->order.size(); ++i)
            data->order[i] = i;
        std::sort(data->order.begin(), data->order.end(),
                  [&](uint32_t a, uint32_t b) { return data->names[a] < data->names[b]; });
        for (uint32_t code : data->order)
        {
            if (data->unique.empty() || data->names[code] != data->names[data->unique.back()])
                data->unique.push_back(code);
            data->rank[code] = uint32_t(data->unique.size() - 1);
        }
        data->valid = true;
        return data;
    };
    // This worker owns its memo/vectors and only borrows the stable membership
    // list. Its future joins before either the list or query lease is destroyed.
    std::future<std::unique_ptr<CanonicalPaths>> preparedPaths;
    std::vector<std::future<std::unique_ptr<CompactTypes>>> typeJobs;
    if (sharedTypes)
    {
        result.reusedTypeRows = sharedTypes->paths.size();
        if (!missingTypes.empty())
        {
            auto missing = readCompactTypes(instance, dictionary, tokens[impl.plan->indices.at("usd-prim-type")],
                                            ordinal, missingTypes.data(), missingTypes.size());
            if (!missing->valid || (!missing->paths.empty() && missing->semantic != sharedTypes->semantic))
                return snapshot;
            sharedTypes->paths.insert(sharedTypes->paths.end(), missing->paths.begin(), missing->paths.end());
            sharedTypes->tokens.insert(sharedTypes->tokens.end(), missing->tokens.begin(), missing->tokens.end());
        }
        sharedTypes->publish();
        typePaths = sharedTypes->paths;
        typeJobs.push_back(
            std::async(std::launch::deferred, [data = std::move(sharedTypes)]() mutable { return std::move(data); }));
    }
    else if (partitionTypes)
    {
        // Fetch membership once, then overlap type transport/coalescing with
        // the ordinary selected payload reads. Workers borrow this immutable
        // vector, not sourcePaths, which may gain additional raw aliases below.
        typePaths.reserve(selectedPaths.empty() ? discoveredPrims : selectedPaths.size());
        if (!readLivePaths(instance, dictionary, readQuery, tokens[impl.plan->indices.at("usd-path")], ordinal, typePaths))
            return snapshot;
        preparedPaths = std::async(std::launch::async, preparePaths, std::cref(typePaths));
        if (!typePaths.empty())
        {
            // Bound transport concurrency separately from column encoding:
            // many concurrent fetch/release loops contend inside the backend.
            const size_t chunkSize = selectedPaths.empty() ? 4096 : 1024;
            const size_t workers = std::min<size_t>(4, (typePaths.size() + chunkSize - 1) / chunkSize);
            const size_t chunk = (typePaths.size() + workers - 1) / workers;
            for (size_t offset = 0; offset < typePaths.size(); offset += chunk)
                typeJobs.push_back(std::async(std::launch::async, readCompactTypes, instance, dictionary,
                                              tokens[impl.plan->indices.at("usd-prim-type")], ordinal,
                                              typePaths.data() + offset, std::min(chunk, typePaths.size() - offset)));
        }
    }
    // Independent sealed column reads do not order with each other. Enqueue
    // bounded disjoint batches before waiting; every operation is joined and
    // every handle is released before the shared query leaves scope.
    const size_t readCount =
        discoveredPrims < 16384 || !selectedPaths.empty() ? 1 : std::min<size_t>(8, attributes.size());
    std::vector<std::vector<ovx_token_t>> batches(readCount);
    for (size_t i = 0; i < attributes.size(); ++i)
        batches[i % readCount].push_back(attributes[i]);
    std::vector<std::unique_ptr<ReadScope>> reads;
    for (size_t i = 0; i < readCount; ++i)
        reads.emplace_back(new ReadScope{ instance });
    std::vector<ovstage_enqueue_result_t> operations(readCount);
    for (size_t i = 0; i < readCount; ++i)
        operations[i] =
            ovstage_read_attributes(instance, readQuery, batches[i].data(), batches[i].size(), range, &reads[i]->read);
    bool readOk = true;
    for (auto operation : operations)
        readOk = wait(instance, operation) && readOk;
    if (!readOk)
        return snapshot;
    ReadListMemo lists(dictionary);
    TextMemo pathsText(dictionary);
    using Row = Impl::View;
    impl.data.reserve(presentColumns.size());
    auto& work = impl.data;
    std::unordered_map<uint64_t, uint32_t> pathIds;
    const size_t expectedPaths = selectedPaths.empty() ? discoveredPrims : selectedPaths.size();
    pathIds.reserve(expectedPaths);
    impl.sourcePaths.reserve(expectedPaths);
    for (const auto path : typePaths)
    {
        if (impl.sourcePaths.size() >= UINT32_MAX)
            return snapshot;
        if (pathIds.emplace(path, uint32_t(impl.sourcePaths.size())).second)
            impl.sourcePaths.push_back(path);
    }
    std::unordered_map<ovx_primpath_list_t, std::vector<uint32_t>> listRows;
    std::vector<std::string> pathNames;
    // Resolve each used logical path once. Raw dictionary IDs are only local
    // lookup accelerators and never enter the canonical bytes.
    size_t rowCount = 0, readIndex = 0;
    std::optional<ReadColumnCursor> cursor;
    auto gather = [&]() {
        while (readIndex < reads.size())
        {
            // Fetch only while another column can be indexed. A kept group is
            // released with the snapshot, also when capture unwinds; one the list
            // cannot take is released by the cursor, as in readScalarBatch.
            if (impl.groups.size() >= std::numeric_limits<uint32_t>::max())
                return false;
            if (!cursor)
                cursor.emplace(instance, reads[readIndex]->read);
            ReadGroupColumn group{};
            const auto status = cursor->next(group);
            if (status == OVSTAGE_ERROR_END_OF_ITERATION)
            {
                cursor.reset();
                ++readIndex;
                continue;
            }
            if (status != OVSTAGE_OK)
                return false;
            cursor->keep(impl.fetched);
            const auto g = uint32_t(impl.groups.size());
            impl.groups.push_back({ group });
            const auto found = impl.columnIndex.find(group.attribute.raw_id);
            if (found == impl.columnIndex.end())
                return false;
            const auto& attribute = names[found->second];
            ++work[found->second].groups;
            if (!isDerivedHierarchyColumn(attribute) && group.meta.attribute_write_floor_ordinal < ordinal)
            {
                result.bypassReason = "column is not sealed at the requested ordinal";
                return false;
            }
            if (attribute != "prototypes" && attribute != "physics:prototypes" && attribute != "prototypes:usd-targets" &&
                (attribute.find("prototype") != std::string::npos || attribute.find("Prototype") != std::string::npos))
            {
                result.bypassReason = "instance prototype identity is unsupported";
                return false;
            }
            if (group.is_delete)
                continue;
            result.bypassReason = "unsupported or malformed column representation";
            const ovx_primpath_t* paths = nullptr;
            size_t pathCount = 0;
            if (!lists.paths(group.prims.list, &paths, &pathCount) || (group.data.mask && group.data.index_map) ||
                ((group.data.mask || group.data.index_map) && group.data.count < group.prims.count))
                return false;
            if (!group.is_array && !readColumn(group, impl.groups[g].layout))
                return false;
            auto& mapped = listRows[group.prims.list];
            if (mapped.empty())
                mapped.resize(pathCount, UINT32_MAX);
            if (mapped.size() != pathCount)
                return false;
            auto& rows = work[found->second].rows;
            if (rows.capacity() < rows.size() + group.prims.count)
                rows.reserve(std::max(rows.size() + group.prims.count, rows.capacity() * 2));
            for (uint32_t i = 0; i < group.prims.count; ++i)
            {
                if (group.data.mask && !(group.data.mask[i / 64] & (uint64_t(1) << (i % 64))))
                    continue;
                const uint64_t index =
                    group.prims.index_map ? group.prims.index_map[i] : uint64_t(group.prims.offset) + i;
                if (index >= pathCount)
                    return false;
                auto& code = mapped[index];
                if (code == UINT32_MAX)
                {
                    auto entry = pathIds.find(paths[index].raw_id);
                    if (entry == pathIds.end())
                    {
                        if (impl.sourcePaths.size() >= std::numeric_limits<uint32_t>::max())
                            return false;
                        entry = pathIds.emplace(paths[index].raw_id, uint32_t(impl.sourcePaths.size())).first;
                        impl.sourcePaths.push_back(paths[index].raw_id);
                    }
                    code = entry->second;
                }
                rows.push_back({ code, g, i });
                ++rowCount;
            }
        }
        return true;
    };
    if (!gather())
        return snapshot;
    std::vector<uint32_t> order, rank, unique;
    auto canonicalPaths = [&]() {
        auto data = preparedPaths.valid() ? preparedPaths.get() : nullptr;
        // Payload transport can introduce additional raw aliases. In that case
        // canonicalize the complete gathered list before proving membership.
        if (!data || impl.sourcePaths != typePaths)
            data = preparePaths(impl.sourcePaths);
        if (!data->valid)
        {
            if (data->prototype)
                result.bypassReason = "instance prototype identity is unsupported";
            return false;
        }
        result.reusedPathTexts = data->texts.borrowedPathCount();
        pathsText = std::move(data->texts);
        pathNames = std::move(data->names);
        order = std::move(data->order);
        rank = std::move(data->rank);
        unique = std::move(data->unique);
        return true;
    };
    if (!canonicalPaths())
        return snapshot;
    for (auto& job : typeJobs)
    {
        auto compact = job.get();
        if (!compact->valid)
        {
            result.bypassReason = compact->sealed ? "unsupported type column representation" :
                                                    "column is not sealed at the requested ordinal";
            return snapshot;
        }
        if (compact->paths.empty())
            continue;
        if (impl.groups.size() >= std::numeric_limits<uint32_t>::max())
            return snapshot;
        const uint32_t g = uint32_t(impl.groups.size());
        ReadGroupColumn group{};
        group.attribute = tokens[impl.plan->indices.at("usd-prim-type")];
        group.semantic = compact->semantic;
        group.prims.count = uint32_t(compact->paths.size());
        group.data.count = group.prims.count;
        group.data.tensor_count = 1;
        group.data.tensors = &compact->tensor;
        ReadColumn layout;
        if (!readColumn(group, layout))
            return snapshot;
        impl.groups.push_back({ group, layout });
        auto& column = work[impl.columnIndex.at(group.attribute.raw_id)];
        ++column.groups;
        for (uint32_t i = 0; i < compact->paths.size(); ++i)
        {
            const auto path = pathIds.find(compact->paths[i]);
            if (path == pathIds.end())
                return snapshot;
            column.rows.push_back({ path->second, g, i });
            ++rowCount;
        }
        impl.compactTypes.push_back(std::move(compact));
    }
    if (selectedPaths.empty() && unique.size() != discoveredPrims)
    {
        // Every retained row belongs to the authoritative all-prims query. Equal
        // distinct LOGICAL path counts prove that its membership is complete.
        // Custom-only/untyped prims can be absent from selected payload columns;
        // only that case needs an explicit derived membership read.
        if (unique.size() > discoveredPrims)
            return snapshot;
        reads.emplace_back(new ReadScope{ instance });
        const auto pathColumn = tokens[impl.plan->indices.at("usd-path")];
        if (!wait(instance, ovstage_read_attributes(instance, readQuery, &pathColumn, 1, range, &reads.back()->read)) ||
            !gather() || !canonicalPaths() || unique.size() != discoveredPrims)
        {
            result.bypassReason = "incomplete live path membership";
            return snapshot;
        }
    }
    Writer canonical;
    canonical.string("ovruntime.physics-inputs.v8.column-plan");
    impl.descendants.resize(unique.size());
    for (size_t i = 0; i + 1 < unique.size(); ++i)
    {
        const auto& path = pathNames[unique[i]];
        const std::string prefix = path == "/" ? path : path + "/";
        impl.descendants[i] = pathNames[unique[i + 1]].compare(0, prefix.size(), prefix) == 0;
    }
    impl.rankedPaths.resize(unique.size());
    for (size_t i = 0; i < impl.sourcePaths.size(); ++i)
        impl.rankedPaths[rank[i]] = impl.sourcePaths[i];
    for (auto& entry : pathIds)
        entry.second = rank[entry.second];
    impl.values = std::move(pathIds);
    // Selected paths were already interned above. Bind those canonical handles
    // to the ranks established from transport aliases, without another complete
    // string-to-path round trip when the ordinary parser first asks for a row.
    for (size_t i = 0; i < selected.size(); ++i)
    {
        const auto name = relativePath(selectedPaths[i], relativeRoot);
        const auto found =
            std::lower_bound(unique.begin(), unique.end(), name,
                             [&](uint32_t index, const std::string& value) { return pathNames[index] < value; });
        if (found == unique.end() || pathNames[*found] != name)
            continue;
        const auto rank = uint32_t(found - unique.begin());
        const auto inserted = impl.values.emplace(selected[i].raw_id, rank);
        if (inserted.first->second != rank)
            return snapshot;
    }
    canonical.u64(unique.size());
    for (auto code : unique)
        canonical.string(pathNames[code]);
    // A prim's usd-metadata:typeName overrides its usd-prim-type, as the parser reads
    // its type: the usd-prim-type names count only the prims without one.
    std::vector<bool> authoredType;
    if (const auto authored = impl.plan->indices.find("usd-metadata:typeName"); authored != impl.plan->indices.end())
        if (const auto column = work.find(authored->second); column != work.end())
        {
            authoredType.resize(unique.size());
            for (const auto& row : column->second.rows)
                authoredType[rank[row.path]] = true;
        }
    // Columns are independent immutable reads. Bound concurrency and keep small
    // captures inline; all workers join before any group can be released.
    auto encodeColumn = [&](size_t c) {
        auto& item = work.at(c);
        auto& rows = item.rows;
        if (rows.size() >= 4096 && rows.size() >= unique.size() / 4)
        {
            // Logical path ranks form a dense bounded domain. Scatter dense
            // columns instead of comparing the same ranks O(N log N) times.
            if (rows.size() > UINT32_MAX)
                return;
            std::vector<uint32_t> positions(unique.size(), UINT32_MAX);
            for (uint32_t i = 0; i < rows.size(); ++i)
            {
                const uint32_t path = rank[rows[i].path];
                if (positions[path] != UINT32_MAX)
                    return; // duplicate logical row
                positions[path] = i;
            }
            std::vector<Row> sorted;
            sorted.reserve(rows.size());
            for (uint32_t path = 0; path < positions.size(); ++path)
                if (positions[path] != UINT32_MAX)
                {
                    const auto& row = rows[positions[path]];
                    sorted.push_back({ path, row.group, row.row });
                }
            rows = std::move(sorted);
        }
        else
        {
            for (auto& row : rows)
                row.path = rank[row.path];
            std::sort(rows.begin(), rows.end(), [](const Row& a, const Row& b) { return a.path < b.path; });
        }
        if (names[c] == "usd-path")
        {
            // The read-only value is exactly the row's logical path, already in
            // the canonical membership table. Its transport groups are optional.
            for (size_t n = 0; n < rows.size(); ++n)
            {
                const auto& group = impl.groups[rows[n].group].read;
                const auto& layout = impl.groups[rows[n].group].layout;
                if (group.is_array || !layout.tensor || layout.tensor->dtype.code != kDLUInt ||
                    layout.tensor->dtype.bits != 64 || layout.components != 1 || (n && rows[n - 1].path == rows[n].path))
                    return;
            }
            item.digest = derivedPathDigest();
            item.valid = true;
            return;
        }
        if (rows.empty())
        {
            item.digest = emptyColumnDigest();
            item.valid = true;
            return;
        }
        TextMemo texts(dictionary, &pathsText);
        CanonicalTextCodes codes(relativeRoot, &prototypes);
        // Eligibility classifies the distinct values of the type and schema columns,
        // counted here while they are hashed, instead of visiting their rows.
        bool named = recordsNames(names[c]);
        if (named)
            codes.countRows();
        const bool overridable = names[c] == "usd-prim-type" && !authoredType.empty();
        Writer values;
        values.reserve(rows.size() * 48);
        for (size_t n = 0; n < rows.size(); ++n)
        {
            const auto& row = rows[n];
            if (n && rows[n - 1].path == row.path)
                return; // ambiguous duplicate history
            const auto& group = impl.groups[row.group].read;
            const auto& column = impl.groups[row.group].layout;
            const DLTensor* tensor = column.tensor;
            const uint8_t* data = nullptr;
            size_t count = 0;
            if (group.is_array)
            {
                const uint32_t index = group.data.index_map ? group.data.index_map[row.row] : row.row;
                if (!group.data.tensors || index >= group.data.tensor_count)
                    return;
                tensor = &group.data.tensors[index];
                if (tensor->device.device_type != kDLCPU || !detail::isCompactReadTensor(*tensor))
                    return;
                count = tensor->dtype.lanes;
                for (int d = 0; d < tensor->ndim; ++d)
                {
                    if (tensor->shape[d] && count > std::numeric_limits<size_t>::max() / tensor->shape[d])
                        return;
                    count *= size_t(tensor->shape[d]);
                }
                if (count && !tensor->data)
                    return;
                if (tensor->byte_offset > std::numeric_limits<size_t>::max() ||
                    count > (std::numeric_limits<size_t>::max() - tensor->byte_offset) / 8)
                    return;
                if (tensor->data)
                    data = static_cast<const uint8_t*>(tensor->data) + tensor->byte_offset;
            }
            else
            {
                data = column.data(row.row);
                count = size_t(column.components);
            }
            values.u32(row.path);
            values.u8(group.is_array);
            values.u32(group.semantic);
            values.u8(tensor->dtype.code);
            values.u8(tensor->dtype.bits);
            values.u32(tensor->dtype.lanes);
            values.u32(group.is_array ? uint32_t(tensor->ndim) : 0);
            if (group.is_array)
                for (int d = 0; d < tensor->ndim; ++d)
                    values.u64(uint64_t(tensor->shape[d]));
            // Values that are not token IDs never reach the codes: leave the names
            // unrecorded, so eligibility cannot take them as absent.
            named = named && tensor->dtype.code == kDLUInt && tensor->dtype.bits == 64;
            if (named)
                codes.row(row.path, !overridable || !authoredType[row.path]);
            if (!encodeValues(values, texts, data, count, tensor->dtype, group.semantic, names[c], codes))
                return;
        }
        Writer vocabulary;
        auto bytes = values.take();
        codes.finish(bytes, vocabulary);
        if (named)
        {
            item.names = codes.counted();
            item.named = true;
        }
        // Length-delimited independent digests avoid copying every column into
        // another stage-sized buffer and permit parallel SHA-256 computation.
        item.digest = columnDigest(rows.size(), bytes, vocabulary);
        item.valid = true;
    };
    std::vector<size_t> tasks;
    for (const auto& entry : work)
        if (!entry.second.rows.empty())
            tasks.push_back(entry.first);
    if (rowCount >= 16384)
        std::stable_sort(tasks.begin(), tasks.end(),
                         [&](size_t a, size_t b) { return work.at(a).rows.size() > work.at(b).rows.size(); });
    std::atomic<size_t> next{ 0 };
    auto worker = [&]() {
        for (;;)
        {
            const auto c = next.fetch_add(1, std::memory_order_relaxed);
            if (c >= tasks.size())
                break;
            encodeColumn(tasks[c]);
        }
    };
    const size_t workers = rowCount < 16384 ? 1 :
                                              std::min<size_t>(rowCount > 1000000 ? 16 : 8,
                                                               std::max(1u, std::thread::hardware_concurrency()));
    std::vector<std::future<void>> jobs;
    for (size_t i = 1; i < workers; ++i)
        jobs.push_back(std::async(std::launch::async, worker));
    worker();
    for (auto& job : jobs)
        job.get();
    // Commit the complete declaration layout once, then encode only present or
    // computed columns. Omitted declarations have a proven global absence, not
    // an omitted validation step. Presence outside a selected unit still counts.
    canonical.bytes(impl.plan->identity.data(), impl.plan->identity.size());
    canonical.u64(presentColumns.size());
    for (const auto c : presentColumns)
    {
        const auto found = work.find(c);
        const auto* value = names[c] == "usd-path" ? &derivedPathDigest() : &emptyColumnDigest();
        if (found != work.end() && !found->second.rows.empty())
        {
            if (!found->second.valid)
                return snapshot;
            value = &found->second.digest;
        }
        canonical.u64(c);
        canonical.u8(present[c]);
        canonical.bytes(value->data(), value->size());
    }
    // Which instance roots share which prototype is ovstage instancing metadata,
    // not a column. Stages without prototypes keep their existing identity bytes.
    if (const auto links = prototypes.links(); !links.empty())
    {
        canonical.string("ovruntime.physics-inputs.instancing-links.v1");
        canonical.u64(links.size());
        for (const auto& [prototype, instanceRoots] : links)
        {
            canonical.string(prototype);
            canonical.u64(instanceRoots->size());
            for (const auto& instanceRoot : *instanceRoots)
                canonical.string(instanceRoot);
        }
    }
    result.value = parse::cache::digest(canonical.data().data(), canonical.data().size());
    impl.prototypeNames = prototypes.takeNames();
    impl.vocabulary = std::move(discovered);
    result.rows = rowCount;
    result.bypassReason.clear();
    return snapshot;
}
// Test-only count of physics-input captures, including failed ones.
void resetPhysicsInputCaptureCountForTest()
{
    captureCalls.store(0, std::memory_order_relaxed);
}
size_t getPhysicsInputCaptureCountForTest()
{
    return captureCalls.load(std::memory_order_relaxed);
}
// Test-only count of the whole-snapshot alias indexes that parse reads built.
void resetPhysicsInputAliasIndexCountForTest()
{
    aliasIndexBuilds.store(0, std::memory_order_relaxed);
}
size_t getPhysicsInputAliasIndexCountForTest()
{
    return aliasIndexBuilds.load(std::memory_order_relaxed);
}
bool canonicalPrototypePaths(ovstage_instance_t* instance,
                             std::vector<std::string>& paths,
                             const PhysicsInputSnapshot* known)
{
    auto* dictionary = instance ? ovstage_get_path_dictionary(instance) : nullptr;
    if (!dictionary)
        return std::none_of(paths.begin(), paths.end(),
                            [](const std::string& path) { return path.find("/__Prototype_") != std::string::npos; });
    PrototypeNames names(instance, dictionary, known ? &known->mImpl->prototypeNames : nullptr);
    for (auto& path : paths)
    {
        std::string canonical;
        if (!names.canonical(path, canonical))
            return false;
        path = std::move(canonical);
    }
    return true;
}
namespace
{
// `/__Prototype(<instance root>)` -> this population's prototype root of that instance.
bool resolvePrototypePath(
    ovstage_instance_t* instance, ovx_path_dictionary_t* dictionary, std::string_view path, std::string& out, int depth)
{
    constexpr std::string_view prefix = "/__Prototype(";
    if (path.compare(0, prefix.size(), prefix) != 0)
    {
        out.assign(path);
        return true;
    }
    if (depth > parse::cache::kMaxPrototypeNesting)
        return false;
    size_t end = prefix.size();
    int open = 1;
    for (; end < path.size() && open; ++end)
        open += path[end] == '(' ? 1 : path[end] == ')' ? -1 : 0;
    if (open)
        return false;
    std::string instanceRoot;
    if (!resolvePrototypePath(
            instance, dictionary, path.substr(prefix.size(), end - 1 - prefix.size()), instanceRoot, depth + 1))
        return false;
    ovx_primpath_t raw = OVX_INVALID_PRIMPATH, prototype = OVX_INVALID_PRIMPATH;
    if (ovx_path_dictionary_intern_path(dictionary, { instanceRoot.data(), instanceRoot.size() }, &raw) != OVX_OK ||
        raw == OVX_INVALID_PRIMPATH || ovstage_instancing_get_prototype_root(instance, raw, &prototype) != OVSTAGE_OK ||
        prototype == OVX_INVALID_PRIMPATH)
        return false;
    ovx_string_t text{};
    if (ovx_path_dictionary_path_to_string(dictionary, prototype, &text) != OVX_OK || !text.ptr)
        return false;
    out.assign(text.ptr, text.length);
    out.append(path.substr(end));
    return true;
}
} // namespace
bool resolvePrototypePaths(ovstage_instance_t* instance, std::vector<std::string>& paths, const PhysicsInputSnapshot* known)
{
    // ovstage_instancing_get_prototype_root rebuilds the whole instancing graph per
    // call, so names the identity already resolved come from its inverse instead.
    // Archived name -> this population's generated prototype root.
    std::unordered_map<std::string_view, ovx_primpath_t> generated;
    if (known)
        for (const auto& [root, name] : known->mImpl->prototypeNames)
            generated.emplace(name, root);
    auto* dictionary = instance ? ovstage_get_path_dictionary(instance) : nullptr;
    for (auto& path : paths)
    {
        if (path.compare(0, 13, "/__Prototype(") != 0)
            continue;
        size_t end = 13;
        int open = 1;
        for (; end < path.size() && open; ++end)
            open += path[end] == '(' ? 1 : path[end] == ')' ? -1 : 0;
        std::string actual;
        ovx_string_t root{};
        if (const auto found = generated.find(std::string_view(path).substr(0, end));
            !open && found != generated.end() && dictionary &&
            ovx_path_dictionary_path_to_string(dictionary, found->second, &root) == OVX_OK && root.ptr)
            actual.assign(root.ptr, root.length).append(path, end);
        else if (!dictionary || !resolvePrototypePath(instance, dictionary, path, actual, 0))
            return false;
        path = std::move(actual);
    }
    return true;
}
} // namespace omni::physics::ovstage
