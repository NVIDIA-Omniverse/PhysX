// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "PrimHierarchyStorage.h"

using namespace pxr;

PrimHierarchyStorage::PrimHierarchyStorage()
    : mStage(nullptr)
{
}

PrimHierarchyStorage::~PrimHierarchyStorage()
{
    mStage = nullptr;
}

void PrimHierarchyStorage::init(pxr::UsdStageRefPtr stage)
{
    mStage = stage;
}

void PrimHierarchyStorage::clear()
{
    mStorageMap.clear();
    mStage = nullptr;
}

void PrimHierarchyStorage::addPrimSubtree(const pxr::SdfPath& primPath, const pxr::SdfPath& topPrim)
{
    const SdfPath stopPath = topPrim;
    if (primPath != topPrim)
    {
        pxr::SdfPath parentPath = primPath.GetParentPath();
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
        pxr::SdfPath childPath = primPath;
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

void PrimHierarchyStorage::mergeHierarchyStorage(const pxr::SdfPath& topPath, const PrimHierarchyStorage& storage)
{
    mStorageMap.insert(storage.getStorageMap().begin(), storage.getStorageMap().end());

    pxr::SdfPath parentPath = topPath.GetParentPath();
    if (parentPath != SdfPath())
    {
        mStorageMap[topPath].parent = parentPath;
        mStorageMap[parentPath].children.insert(topPath);
    }    
}

void PrimHierarchyStorage::addPrim(const pxr::SdfPath& primPath)
{
    pxr::UsdPrim prim = mStage->GetPrimAtPath(primPath);
    if (prim)
    {
        pxr::UsdPrim parent = prim.GetParent();
        if (!parent)
        {
            mStorageMap[primPath].parent = SdfPath();
            return;
        }
        Item* childItem = &mStorageMap[primPath];
        pxr::SdfPath childPath = primPath;
        while (parent && parent != mStage->GetPseudoRoot())
        {
            const pxr::SdfPath parentPath = parent.GetPrimPath();
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

void PrimHierarchyStorage::removePrim(const pxr::SdfPath& primPath)
{    
    const Item& primItem = mStorageMap[primPath];

    // remove from parent
    if (primItem.parent != SdfPath())
    {
        Item& parentItem = mStorageMap[primItem.parent];
        parentItem.children.erase(primPath);
    }

    // remove childs
    for (SdfPathMap::const_reference& ref : primItem.children)
    {
        removePrimInternal(ref);
    }
    mStorageMap.erase(primPath);
}

void PrimHierarchyStorage::removePrimInternal(const pxr::SdfPath& primPath)
{
    const Item& primItem = mStorageMap[primPath];

    // remove childs
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

    // remove from parent
    if (primItem.parent != SdfPath())
    {
        Item& parentItem = mStorageMap[primItem.parent];
        parentItem.children.erase(primPath);
    }

    // remove childs
    for (SdfPathVector::const_reference& ref : childs)
    {
        mStorageMap.erase(ref);
    }
    mStorageMap.erase(primPath);
}

void PrimHierarchyStorage::Iterator::gatherChilds(const PrimHierarchyStorage& storage, const pxr::SdfPath& primPath)
{
    StorageMap::const_iterator it = storage.mStorageMap.find(primPath);
    if (it != storage.mStorageMap.end())
    {
        mIteratorPaths.push_back(primPath);

        // gather childs
        for (SdfPathMap::const_reference& ref : it->second.children)
        {
            gatherChilds(storage, ref);
        }
    }
}

PrimHierarchyStorage::Iterator::Iterator(const PrimHierarchyStorage& storage, const pxr::SdfPath& primPath)
    : mPrimPath(primPath)
{
    StorageMap::const_iterator it = storage.mStorageMap.find(primPath);
    if (it != storage.mStorageMap.end())
    {
        mIteratorPaths.reserve(storage.mStorageMap.size());

        gatherChilds(storage, primPath);
    }
}
