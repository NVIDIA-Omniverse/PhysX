// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <map>

using SdfPathMap = std::set<PXR_NS::SdfPath>;

class PrimHierarchyStorage
{
public:
    struct Item
    {
        PXR_NS::SdfPath        parent;
        SdfPathMap          children;
    };
    using StorageMap = std::unordered_map<PXR_NS::SdfPath, Item, PXR_NS::SdfPath::Hash>;

    PrimHierarchyStorage();
    ~PrimHierarchyStorage();

    class Iterator
    {
    public:
        Iterator(const PrimHierarchyStorage& storage, const PXR_NS::SdfPath& primPath);

        const PXR_NS::SdfPathVector& getDescendentsPaths() const
        {
            return mIteratorPaths;
        }

        const PXR_NS::SdfPath& getPrimPath() const
        {
            return mPrimPath;
        }

    private:
        void gatherChilds(const PrimHierarchyStorage& storage, const PXR_NS::SdfPath& primPath);

        PXR_NS::SdfPath        mPrimPath;
        PXR_NS::SdfPathVector  mIteratorPaths;
    };

    void init(PXR_NS::UsdStageRefPtr stage);
    void clear();

    void addPrim(const PXR_NS::SdfPath& primPath);
    void addPrimSubtree(const PXR_NS::SdfPath& primPath, const PXR_NS::SdfPath& topPrim);
    void removePrim(const PXR_NS::SdfPath& primPath); // removes prim and its childs

    void removeIteration(Iterator& it);

    void mergeHierarchyStorage(const PXR_NS::SdfPath& topPath, const PrimHierarchyStorage& storage);

    const StorageMap& getStorageMap() const
    {
        return mStorageMap;
    }

private:    
    void removePrimInternal(const PXR_NS::SdfPath& primPath);
    
    friend class Iterator;

    StorageMap          mStorageMap;
    PXR_NS::UsdStageRefPtr mStage;
};
