// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-BUILD-UNIBUILD-001
 * @covers AC-7
 */

#pragma once

#include <functional>
#include <map>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

// Plain string-keyed prim hierarchy (ADR-0018/0019 USD-removal campaign): this
// structure's whole job is path-PREFIX arithmetic (parent/child by string
// prefix) over identity paths, and every method below is pure lexical
// string-slicing with no other USD semantics -- there is nothing here that
// actually needs an SdfPath. It is kept string- rather than ObjectKey-keyed on
// purpose: it also has to index SYNTHETIC clone identities that are never
// authored into the backing IPhysicsSource (replicator clones), so an
// ObjectKey-keyed structure could not enumerate them without a redundant
// side-table -- see PhysXReplicator.cpp's addPrimSubtree/mergeHierarchyStorage
// callers.
using StringPathSet = std::set<std::string>;

class PrimHierarchyStorage
{
public:
    struct Item
    {
        std::string         parent;
        StringPathSet       children;
    };
    using StorageMap = std::unordered_map<std::string, Item>;

    PrimHierarchyStorage();
    ~PrimHierarchyStorage();

    class Iterator
    {
    public:
        Iterator(const PrimHierarchyStorage& storage, const std::string& primPath);

        const std::vector<std::string>& getDescendentsPaths() const
        {
            return mIteratorPaths;
        }

        const std::string& getPrimPath() const
        {
            return mPrimPath;
        }

    private:
        void gatherChilds(const PrimHierarchyStorage& storage, const std::string& primPath);

        std::string              mPrimPath;
        std::vector<std::string> mIteratorPaths;
    };

    // Answers "does this path name a live object in the scene?". Paths that fail
    // it contribute no hierarchy row. An unset predicate admits every path.
    using ExistsFn = std::function<bool(const std::string&)>;

    void init(ExistsFn exists);
    void clear();

    // Whether init() has installed a predicate. Observable so a test can assert the owner
    // installed it, which an unset (admit-everything) predicate is otherwise indistinguishable
    // from -- see TEST-BUILD-UNIBUILD-001 AC-7.
    bool hasExistsPredicate() const
    {
        return static_cast<bool>(mExists);
    }

    void addPrim(const std::string& primPath);
    void addPrimSubtree(const std::string& primPath, const std::string& topPrim);
    void removePrim(const std::string& primPath); // removes prim and its childs

    void removeIteration(Iterator& it);

    void mergeHierarchyStorage(const std::string& topPath, const PrimHierarchyStorage& storage);

    const StorageMap& getStorageMap() const
    {
        return mStorageMap;
    }

private:
    void removePrimInternal(const std::string& primPath);

    friend class Iterator;

    StorageMap          mStorageMap;
    ExistsFn            mExists;
};
