// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <vector>
#include <unordered_map>

#include <omni/fabric/IToken.h>
#include <omni/fabric/IFabric.h>
#include <omni/fabric/connectivity/Connectivity.h>
#include <omni/fabric/stage/StageReaderWriter.h>

#include <carb/profiler/Profile.h>


namespace omni
{
namespace physx
{

// Currently it implements only custom object storage -> Fabric mapping, only for GPU
class FabricMapper
{
public:
    using AttributePtrType = char*;

    void bindFabric(const omni::fabric::FabricId& fabricId);
    void bindDevice(int deviceId);

    // Builds the mapping from index range [0, maxTargetIndex) to Fabric GPU pointers of attributeToken
    //   based on the provided inverse (path -> index) mapping.
    // The result is a GPU array of size maxTargetIndex, every element points to the corresponding attribute

    template <typename IndexHolder, typename ElementType>
    void build(const std::unordered_map<omni::fabric::Path, IndexHolder>& indexMap,
               size_t maxTargetIndex,
               std::function<size_t(const IndexHolder&)>&& extractIndexFunc,
               const omni::fabric::Token& attributeToken,
               omni::fabric::PrimBucketList& primBuckets);


    // Similar to build but the resulting mapping points to the attribute of the parent prim
    template <typename IndexHolder, typename ElementType>
    void buildParent(const std::unordered_map<omni::fabric::Path, IndexHolder>& indexMap,
                     std::unordered_map<omni::fabric::Path, FabricMapper::AttributePtrType>& parentsMap,
                     size_t maxTargetIndex,
                     std::function<size_t(const IndexHolder&)>&& extractIndexFunc,
                     const omni::fabric::Token& attributeToken,
                     omni::fabric::PrimBucketList& parentsPrimBuckets);

    // Returns the number of elements in the mapping
    size_t getSize(const omni::fabric::Token& attributeToken) const;

    // Returns the pointer to the device array that contains the mapping as an array of pointers
    AttributePtrType* getPtr(const omni::fabric::Token& attributeToken) const;
    // Returns the list of attributes for which mapping is already built
    std::vector<omni::fabric::Token> getAttributes() const;
    size_t getParentSize(const omni::fabric::Token& attributeToken) const;
    AttributePtrType* getParentPtr(const omni::fabric::Token& attributeToken) const;
    std::vector<omni::fabric::Token> getParentAttributes() const;
    // Returns if any attribute mapping was added
    bool empty() const;

    int deviceId() const;

    // Clears all internal data
    void clear();

protected:
    struct MappingStorage
    {
        ~MappingStorage();
        void reset(size_t size);
        void releaseGpu();

        std::vector<AttributePtrType> m_objectToFabricCpu; // don't let the name fool you, pointers point to GPU, only
                                                           // the storage is on CPU
        AttributePtrType* m_objectToFabricGpu = nullptr;
        size_t m_size = 0;
    };

    MappingStorage& initialize(const omni::fabric::Token& attributeToken,
                               size_t maxTargetIndex,
                               std::unordered_map<omni::fabric::Token, MappingStorage>& map);


    void finalize(MappingStorage& storage);

    omni::fabric::StageReaderWriter m_srw;
    std::unordered_map<omni::fabric::Token, MappingStorage> m_mappingStorage;
    std::unordered_map<omni::fabric::Token, MappingStorage> m_parentMappingStorage;
    int m_deviceId = -1;
};

template <typename IndexHolder, typename ElementType>
void FabricMapper::build(const std::unordered_map<omni::fabric::Path, IndexHolder>& indexMap,
                         size_t maxTargetIndex,
                         std::function<size_t(const IndexHolder&)>&& extractIndexFunc,
                         const omni::fabric::Token& attributeToken,
                         omni::fabric::PrimBucketList& primBuckets)
{
    CARB_PROFILE_ZONE(1, "FabricMapper build %s !", omni::fabric::Token(attributeToken).getText());
    if (m_srw.getId() == omni::fabric::kInvalidStageReaderWriterId)
    {
        CARB_LOG_ERROR("FabricMapper was not initialize, please call bindFabric to bind a valid Fabric cache");
        return;
    }
    if (m_deviceId < 0)
    {
        CARB_LOG_ERROR("FabricMapper must be bound to a valid device. Call bindDevice before building the mapping.");
        return;
    }

    MappingStorage& storage = m_mappingStorage[attributeToken].m_size == maxTargetIndex ?
                                  m_mappingStorage[attributeToken] :
                                  initialize(attributeToken, maxTargetIndex, m_mappingStorage);


    size_t bucketCount = primBuckets.bucketCount();
    for (size_t i = 0; i != bucketCount; i++)
    {
        gsl::span<const omni::fabric::Path> paths = m_srw.getPathArray(primBuckets, i);
        gsl::span<ElementType> attributesArray = m_srw.getAttributeArrayWrGpu<ElementType>(primBuckets, i, attributeToken, m_deviceId);
        if (paths.size() != attributesArray.size())
        {
            CARB_LOG_ERROR("FabricMapper: paths and attributes arrays size mismatch for %s", omni::fabric::Token(attributeToken).getText());
            return;
        }

        for (size_t index = 0; index != paths.size(); index++)
        {
            auto pathc = paths[index];
            auto indexHolder = indexMap.find(pathc);
            if(indexHolder == indexMap.end()) continue;
            size_t mappedIndex = extractIndexFunc(indexHolder->second);
            if (storage.m_objectToFabricCpu.size() <= mappedIndex) continue;
            auto ptr = &(attributesArray[index]);
            storage.m_objectToFabricCpu[mappedIndex] = reinterpret_cast<AttributePtrType>(ptr);
        }
    }

    finalize(storage);
}

template <typename IndexHolder, typename ElementType>
void FabricMapper::buildParent(const std::unordered_map<omni::fabric::Path, IndexHolder>& indexMap,
                               std::unordered_map<omni::fabric::Path, FabricMapper::AttributePtrType>& parentsMap,
                               size_t maxTargetIndex,
                               std::function<size_t(const IndexHolder&)>&& extractIndexFunc,
                               const omni::fabric::Token& attributeToken,
                               omni::fabric::PrimBucketList& parentsPrimBuckets )
{
    CARB_PROFILE_ZONE(1, "FabricMapper build parent %s", omni::fabric::Token(attributeToken).getText());
    if (m_srw.getId() == omni::fabric::kInvalidStageReaderWriterId)
    {
        CARB_LOG_ERROR("FabricMapper was not initialize, please call bindFabric to bind a valid Fabric cache");
        return;
    }
    if (m_deviceId < 0)
    {
        CARB_LOG_ERROR("FabricMapper must be bound to a valid device. Call bindDevice before building the mapping.");
        return;
    }

    MappingStorage& storage = m_parentMappingStorage[attributeToken].m_size == maxTargetIndex ?
                                  m_parentMappingStorage[attributeToken] :
                                  initialize(attributeToken, maxTargetIndex, m_parentMappingStorage);
    
    // iterate through all prims in scene and fill the parentsMap with pointer to attribute in case if prim path is presented in parentsMap keys
    size_t bucketCount = parentsPrimBuckets.bucketCount();
    for (size_t i = 0; i != bucketCount; i++)
    {
        gsl::span<const omni::fabric::Path> parentPaths = m_srw.getPathArray(parentsPrimBuckets, i);
        gsl::span<ElementType> attributesArray = m_srw.getAttributeArrayWrGpu<ElementType>(parentsPrimBuckets, i, attributeToken, m_deviceId);
        if (parentPaths.size() != attributesArray.size())
        {
            CARB_LOG_ERROR("FabricMapper: parentPaths and attributes arrays size mismatch for %s", omni::fabric::Token(attributeToken).getText());
            return;
        }

        for (size_t index = 0; index != parentPaths.size(); index++)
        {
            auto& pathc = parentPaths[index];
            auto parentIterator = parentsMap.find(pathc);
            if(parentIterator != parentsMap.end())
            {
                auto ptr = &(attributesArray[index]);
                parentIterator->second = reinterpret_cast<AttributePtrType>(ptr);
            }
        }
    }

    // iterate through indexMap and map storage with pointer to parent's attribute
    for (const auto& mapIt : indexMap)
    {
        auto parentIterator = mapIt.second.parentLink;
        auto parentPath = omni::fabric::Path(parentIterator->first);
        if (parentPath != omni::fabric::Path())
        {
            size_t mappedIndex = extractIndexFunc(mapIt.second);
            if (storage.m_objectToFabricCpu.size() <= mappedIndex)
            {
                continue;
            }
            auto ptr = parentIterator->second;
            storage.m_objectToFabricCpu[mappedIndex] = ptr;
        }
    }

    finalize(storage);
}


} // namespace physx
} // namespace omni
