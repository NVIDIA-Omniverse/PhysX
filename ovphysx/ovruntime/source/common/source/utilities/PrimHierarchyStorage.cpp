// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

/**
 * @implements REQ-REPLICATE-001
 * @covers AC-7
 */

#include "UsdPCH.h"

#include "PrimHierarchyStorage.h"

using namespace PXR_NS;

PrimHierarchyStorage::PrimHierarchyStorage()
    : mStage(nullptr)
{
}

PrimHierarchyStorage::~PrimHierarchyStorage()
{
    mStage = nullptr;
}

void PrimHierarchyStorage::init(PXR_NS::UsdStageRefPtr stage)
{
    mStage = stage;
}

void PrimHierarchyStorage::clear()
{
    mStorageMap.clear();
    mStage = nullptr;
}

void PrimHierarchyStorage::addPrimSubtree(const PXR_NS::SdfPath& primPath, const PXR_NS::SdfPath& topPrim)
{
    const SdfPath stopPath = topPrim;
    if (primPath != topPrim)
    {
        PXR_NS::SdfPath parentPath = primPath.GetParentPath();
        if (parentPath == stopPath)
        {
            mStorageMap[primPath].parent = stopPath;
            if (parentPath != SdfPath() && primPath != topPrim)
            {
                mStorageMap[parentPath].children.insert(primPath);
            }
            return;
        }
        Item* childItem = &mStorageMap[primPath];
        PXR_NS::SdfPath childPath = primPath;
        while (parentPath != topPrim && !parentPath.IsRootPrimPath())
        {
            childItem->parent = parentPath;

            Item* storageItem = &mStorageMap[parentPath];
            storageItem->children.insert(childPath);

            childItem = storageItem;
            childPath = parentPath;
            parentPath = parentPath.GetParentPath();
        }
        if (parentPath != SdfPath() && !parentPath.IsRootPrimPath())
        {
            mStorageMap[parentPath].children.insert(childPath);
            mStorageMap[childPath].parent = parentPath;
        }
        else
            childItem->parent = SdfPath();
    }
}

void PrimHierarchyStorage::mergeHierarchyStorage(const PXR_NS::SdfPath& topPath, const PrimHierarchyStorage& storage)
{
    mStorageMap.insert(storage.getStorageMap().begin(), storage.getStorageMap().end());
    if (topPath.IsEmpty())
        return;

    PXR_NS::SdfPath parentPath = topPath.GetParentPath();
    if (parentPath.IsAbsolutePath() && !parentPath.IsAbsoluteRootPath())
    {
        // Materialize and repair every lexical link up to the root prim.
        PXR_NS::SdfPath childPath = topPath;
        while (parentPath.IsAbsolutePath() && !parentPath.IsAbsoluteRootPath())
        {
            mStorageMap[childPath].parent = parentPath;
            mStorageMap[parentPath].children.insert(childPath);
            childPath = parentPath;
            parentPath = parentPath.GetParentPath();
        }
    }
    else if (parentPath.IsAbsoluteRootPath())
    {
        // Top-level target (parent is the absolute root "/"): consumers select roots by
        // parent.IsEmpty(), so linking under "/" would orphan the subtree instead. addPrimSubtree()
        // never creates a root-prim topPath, so materialize it explicitly here and re-home its
        // immediate children (left detached with an empty parent by addPrimSubtree) beneath it.
        Item& topItem = mStorageMap[topPath];
        topItem.parent = SdfPath();
        for (StorageMap::const_reference& entry : storage.getStorageMap())
        {
            if (entry.first != topPath && entry.second.parent.IsEmpty() &&
                entry.first.GetParentPath() == topPath)
            {
                mStorageMap[entry.first].parent = topPath;
                topItem.children.insert(entry.first);
            }
        }
    }
}

void PrimHierarchyStorage::addPrim(const PXR_NS::SdfPath& primPath)
{
    if (primPath.IsEmpty())
        return;

    if (!mStage)
    {
        // Ovstage-only attachments have no UsdStage to walk. Rebuild the same
        // parent/child links lexically so subtree removal remains valid without
        // dereferencing a null stage.
        Item* childItem = &mStorageMap[primPath];
        PXR_NS::SdfPath childPath = primPath;
        PXR_NS::SdfPath parentPath = primPath.GetParentPath();
        while (!parentPath.IsEmpty() && parentPath != SdfPath::AbsoluteRootPath())
        {
            childItem->parent = parentPath;

            Item* parentItem = &mStorageMap[parentPath];
            parentItem->children.insert(childPath);

            childItem = parentItem;
            childPath = parentPath;
            parentPath = parentPath.GetParentPath();
        }
        childItem->parent = SdfPath();
        return;
    }

    PXR_NS::UsdPrim prim = mStage->GetPrimAtPath(primPath);
    if (prim)
    {
        PXR_NS::UsdPrim parent = prim.GetParent();
        if (!parent)
        {
            mStorageMap[primPath].parent = SdfPath();
            return;
        }
        Item* childItem = &mStorageMap[primPath];
        PXR_NS::SdfPath childPath = primPath;
        while (parent && parent != mStage->GetPseudoRoot())
        {
            const PXR_NS::SdfPath parentPath = parent.GetPrimPath();
            childItem->parent = parentPath;

            Item* storageItem = &mStorageMap[parentPath];
            storageItem->children.insert(childPath);

            childItem = storageItem;
            childPath = parentPath;
            parent = parent.GetParent();
        }
        childItem->parent = SdfPath();
    }
}

void PrimHierarchyStorage::removePrim(const PXR_NS::SdfPath& primPath)
{    
    const Item& primItem = mStorageMap[primPath];

    if (primItem.parent != SdfPath())
    {
        Item& parentItem = mStorageMap[primItem.parent];
        parentItem.children.erase(primPath);
    }

    for (SdfPathMap::const_reference& ref : primItem.children)
    {
        removePrimInternal(ref);
    }
    mStorageMap.erase(primPath);
}

void PrimHierarchyStorage::removePrimInternal(const PXR_NS::SdfPath& primPath)
{
    const Item& primItem = mStorageMap[primPath];

    for (SdfPathMap::const_reference& ref : primItem.children)
    {
        removePrimInternal(ref);
    }
    mStorageMap.erase(primPath);
}

void PrimHierarchyStorage::removeIteration(Iterator& it)
{
    const SdfPathVector& childs = it.getDescendentsPaths();
    const SdfPath& primPath = it.getPrimPath();

    const Item& primItem = mStorageMap[primPath];

    if (primItem.parent != SdfPath())
    {
        Item& parentItem = mStorageMap[primItem.parent];
        parentItem.children.erase(primPath);
    }

    for (SdfPathVector::const_reference& ref : childs)
    {
        mStorageMap.erase(ref);
    }
    mStorageMap.erase(primPath);
}

void PrimHierarchyStorage::Iterator::gatherChilds(const PrimHierarchyStorage& storage, const PXR_NS::SdfPath& primPath)
{
    StorageMap::const_iterator it = storage.mStorageMap.find(primPath);
    if (it != storage.mStorageMap.end())
    {
        mIteratorPaths.push_back(primPath);

        for (SdfPathMap::const_reference& ref : it->second.children)
        {
            gatherChilds(storage, ref);
        }
    }
}

PrimHierarchyStorage::Iterator::Iterator(const PrimHierarchyStorage& storage, const PXR_NS::SdfPath& primPath)
    : mPrimPath(primPath)
{
    StorageMap::const_iterator it = storage.mStorageMap.find(primPath);
    if (it != storage.mStorageMap.end())
    {
        mIteratorPaths.reserve(storage.mStorageMap.size());

        gatherChilds(storage, primPath);
    }
}
