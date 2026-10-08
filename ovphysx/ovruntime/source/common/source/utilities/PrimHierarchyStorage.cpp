// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "PrimHierarchyStorage.h"

namespace
{
// Lexical prim-path parent, mirroring PXR_NS::SdfPath::GetParentPath() for the
// plain "/A/B/C"-shaped identity paths this storage indexes: the absolute root
// path ("/") and the empty path have no parent (empty string); a root prim
// path ("/A") parents to "/"; anything deeper drops its last "/"-delimited
// component. A single-component relative path ("relative") also has no
// parent, matching SdfPath's own behavior for that shape.
std::string getParentPath(const std::string& path)
{
    if (path.empty() || path == "/")
        return std::string();

    const size_t lastSlash = path.find_last_of('/');
    if (lastSlash == std::string::npos)
        return std::string();
    if (lastSlash == 0)
        return std::string("/");

    return path.substr(0, lastSlash);
}

// Mirrors PXR_NS::SdfPath::IsRootPrimPath(): absolute, and exactly one path
// element -- a single leading "/" and no further "/" after it.
bool isRootPrimPath(const std::string& path)
{
    return !path.empty() && path.front() == '/' && path.size() > 1 && path.find('/', 1) == std::string::npos;
}

// Mirrors PXR_NS::SdfPath::IsAbsolutePath().
bool isAbsolutePath(const std::string& path)
{
    return !path.empty() && path.front() == '/';
}
} // namespace

PrimHierarchyStorage::PrimHierarchyStorage()
{
}

PrimHierarchyStorage::~PrimHierarchyStorage()
{
}

void PrimHierarchyStorage::init(ExistsFn exists)
{
    mExists = std::move(exists);
}

void PrimHierarchyStorage::clear()
{
    mStorageMap.clear();
    mExists = nullptr;
}

void PrimHierarchyStorage::addPrimSubtree(const std::string& primPath, const std::string& topPrim)
{
    const std::string stopPath = topPrim;
    if (primPath != topPrim)
    {
        std::string parentPath = getParentPath(primPath);
        if (parentPath == stopPath)
        {
            mStorageMap[primPath].parent = stopPath;
            if (!parentPath.empty() && primPath != topPrim)
            {
                mStorageMap[parentPath].children.insert(primPath);
            }
            return;
        }
        Item* childItem = &mStorageMap[primPath];
        std::string childPath = primPath;
        while (parentPath != topPrim && !isRootPrimPath(parentPath))
        {
            childItem->parent = parentPath;

            Item* storageItem = &mStorageMap[parentPath];
            storageItem->children.insert(childPath);

            childItem = storageItem;
            childPath = parentPath;
            parentPath = getParentPath(parentPath);
        }
        if (!parentPath.empty() && !isRootPrimPath(parentPath))
        {
            mStorageMap[parentPath].children.insert(childPath);
            mStorageMap[childPath].parent = parentPath;
        }
        else
            childItem->parent.clear();
    }
}

void PrimHierarchyStorage::mergeHierarchyStorage(const std::string& topPath, const PrimHierarchyStorage& storage)
{
    mStorageMap.insert(storage.getStorageMap().begin(), storage.getStorageMap().end());
    if (topPath.empty())
        return;

    std::string parentPath = getParentPath(topPath);
    if (isAbsolutePath(parentPath) && parentPath != "/")
    {
        // Materialize and repair every lexical link up to the root prim.
        std::string childPath = topPath;
        while (isAbsolutePath(parentPath) && parentPath != "/")
        {
            mStorageMap[childPath].parent = parentPath;
            mStorageMap[parentPath].children.insert(childPath);
            childPath = parentPath;
            parentPath = getParentPath(parentPath);
        }
    }
    else if (parentPath == "/")
    {
        // Top-level target (parent is the absolute root "/"): consumers select roots by
        // parent.empty(), so linking under "/" would orphan the subtree instead. addPrimSubtree()
        // never creates a root-prim topPath, so materialize it explicitly here and re-home its
        // immediate children (left detached with an empty parent by addPrimSubtree) beneath it.
        Item& topItem = mStorageMap[topPath];
        topItem.parent.clear();
        for (StorageMap::const_reference& entry : storage.getStorageMap())
        {
            if (entry.first != topPath && entry.second.parent.empty() && getParentPath(entry.first) == topPath)
            {
                mStorageMap[entry.first].parent = topPath;
                topItem.children.insert(entry.first);
            }
        }
    }
}

void PrimHierarchyStorage::addPrim(const std::string& primPath)
{
    if (primPath.empty())
        return;

    // A path the scene does not resolve to a live object gets no row at all, so
    // subtree queries never report prims that are not there.
    if (mExists && !mExists(primPath))
        return;

    // Ancestors of a live object are live, so the links are pure path arithmetic:
    // every step is the object at the parent path, up to (excluding) the root.
    Item* childItem = &mStorageMap[primPath];
    std::string childPath = primPath;
    std::string parentPath = getParentPath(primPath);
    while (!parentPath.empty() && parentPath != "/")
    {
        childItem->parent = parentPath;

        Item* parentItem = &mStorageMap[parentPath];
        parentItem->children.insert(childPath);

        childItem = parentItem;
        childPath = parentPath;
        parentPath = getParentPath(parentPath);
    }
    childItem->parent.clear();
}

void PrimHierarchyStorage::removePrim(const std::string& primPath)
{
    const Item& primItem = mStorageMap[primPath];

    if (!primItem.parent.empty())
    {
        Item& parentItem = mStorageMap[primItem.parent];
        parentItem.children.erase(primPath);
    }

    for (StringPathSet::const_reference& ref : primItem.children)
    {
        removePrimInternal(ref);
    }
    mStorageMap.erase(primPath);
}

void PrimHierarchyStorage::removePrimInternal(const std::string& primPath)
{
    const Item& primItem = mStorageMap[primPath];

    for (StringPathSet::const_reference& ref : primItem.children)
    {
        removePrimInternal(ref);
    }
    mStorageMap.erase(primPath);
}

void PrimHierarchyStorage::removeIteration(Iterator& it)
{
    const std::vector<std::string>& childs = it.getDescendentsPaths();
    const std::string& primPath = it.getPrimPath();

    const Item& primItem = mStorageMap[primPath];

    if (!primItem.parent.empty())
    {
        Item& parentItem = mStorageMap[primItem.parent];
        parentItem.children.erase(primPath);
    }

    for (std::vector<std::string>::const_reference& ref : childs)
    {
        mStorageMap.erase(ref);
    }
    mStorageMap.erase(primPath);
}

void PrimHierarchyStorage::Iterator::gatherChilds(const PrimHierarchyStorage& storage, const std::string& primPath)
{
    StorageMap::const_iterator it = storage.mStorageMap.find(primPath);
    if (it != storage.mStorageMap.end())
    {
        mIteratorPaths.push_back(primPath);

        for (StringPathSet::const_reference& ref : it->second.children)
        {
            gatherChilds(storage, ref);
        }
    }
}

PrimHierarchyStorage::Iterator::Iterator(const PrimHierarchyStorage& storage, const std::string& primPath)
    : mPrimPath(primPath)
{
    StorageMap::const_iterator it = storage.mStorageMap.find(primPath);
    if (it != storage.mStorageMap.end())
    {
        mIteratorPaths.reserve(storage.mStorageMap.size());

        gatherChilds(storage, primPath);
    }
}
