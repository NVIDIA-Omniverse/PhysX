// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <map>

using SdfPathMap = std::set<pxr::SdfPath>;

class PrimHierarchyStorage
{
public:
    struct Item
    {
        pxr::SdfPath        parent;
        SdfPathMap          children;
    };
    using StorageMap = std::unordered_map<pxr::SdfPath, Item, pxr::SdfPath::Hash>;

    PrimHierarchyStorage();
    ~PrimHierarchyStorage();

    class Iterator
    {
    public:
        Iterator(const PrimHierarchyStorage& storage, const pxr::SdfPath& primPath);

        const pxr::SdfPathVector& getDescendentsPaths() const
        {
            return mIteratorPaths;
        }

        const pxr::SdfPath& getPrimPath() const
        {
            return mPrimPath;
        }

    private:
        void gatherChilds(const PrimHierarchyStorage& storage, const pxr::SdfPath& primPath);

        pxr::SdfPath        mPrimPath;
        pxr::SdfPathVector  mIteratorPaths;
    };

    void init(pxr::UsdStageRefPtr stage);
    void clear();

    void addPrim(const pxr::SdfPath& primPath);
    void addPrimSubtree(const pxr::SdfPath& primPath, const pxr::SdfPath& topPrim);
    void removePrim(const pxr::SdfPath& primPath); // removes prim and its childs

    void removeIteration(Iterator& it);

    void mergeHierarchyStorage(const pxr::SdfPath& topPath, const PrimHierarchyStorage& storage);

    const StorageMap& getStorageMap() const
    {
        return mStorageMap;
    }

private:    
    void removePrimInternal(const pxr::SdfPath& primPath);
    
    friend class Iterator;

    StorageMap          mStorageMap;
    pxr::UsdStageRefPtr mStage;
};
