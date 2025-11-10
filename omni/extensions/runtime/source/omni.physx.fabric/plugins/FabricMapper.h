// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
    void build(const std::unordered_map<omni::fabric::PathC, IndexHolder>& indexMap,
               size_t maxTargetIndex,
               std::function<size_t(const IndexHolder&)>&& extractIndexFunc,
               const omni::fabric::TokenC& attributeToken);
    template <typename IndexHolder, typename ElementType>

    // Similar to build but the resulting mapping points to the attribute of the parent prim
    void buildParent(const std::unordered_map<omni::fabric::PathC, IndexHolder>& indexMap,
                     size_t maxTargetIndex,
                     std::function<size_t(const IndexHolder&)>&& extractIndexFunc,
                     const omni::fabric::TokenC& attributeToken);

    // Returns the number of elements in the mapping
    size_t getSize(const omni::fabric::TokenC& attributeToken) const;

    // Returns the pointer to the device array that contains the mapping as an array of pointers
    AttributePtrType* getPtr(const omni::fabric::TokenC& attributeToken) const;
    // Returns the list of attributes for which mapping is already built
    std::vector<omni::fabric::TokenC> getAttributes() const;
    size_t getParentSize(const omni::fabric::TokenC& attributeToken) const;
    AttributePtrType* getParentPtr(const omni::fabric::TokenC& attributeToken) const;
    std::vector<omni::fabric::TokenC> getParentAttributes() const;
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

    MappingStorage& initialize(const omni::fabric::TokenC& attributeToken,
                               size_t maxTargetIndex,
                               std::unordered_map<omni::fabric::TokenC, MappingStorage>& map);
    template <typename ElementType>
    void registerMapping(MappingStorage& storage,
                         const omni::fabric::TokenC& attributeToken,
                         size_t mappedIndex,
                         const omni::fabric::PathC& path)
    {
        if (storage.m_objectToFabricCpu.size() <= mappedIndex)
        {
            return;
        }
        storage.m_objectToFabricCpu[mappedIndex] =
            reinterpret_cast<AttributePtrType>(m_srw.getAttributeWrGpu<ElementType>(path, attributeToken, m_deviceId));
    }
    void finalize(MappingStorage& storage);

    omni::fabric::StageReaderWriter m_srw;
    std::unordered_map<omni::fabric::TokenC, MappingStorage> m_mappingStorage;
    std::unordered_map<omni::fabric::TokenC, MappingStorage> m_parentMappingStorage;
    int m_deviceId = -1;
};


template <typename IndexHolder, typename ElementType>
void FabricMapper::build(const std::unordered_map<omni::fabric::PathC, IndexHolder>& indexMap,
                         size_t maxTargetIndex,
                         std::function<size_t(const IndexHolder&)>&& extractIndexFunc,
                         const omni::fabric::TokenC& attributeToken)
{
    CARB_PROFILE_ZONE(1, "FabricMapper build %s", omni::fabric::Token(attributeToken).getText());
    if (m_srw.getId() == omni::fabric::kInvalidStageReaderWriterId)
    {
        CARB_LOG_ERROR("FabricMapper was not initialize, please call bindFabric to bind a valid Fabric cache");
        return;
    }
    MappingStorage& storage = initialize(attributeToken, maxTargetIndex, m_mappingStorage);

    for (const auto& mapIt : indexMap)
    {
        size_t mappedIndex = extractIndexFunc(mapIt.second);
        registerMapping<ElementType>(storage, attributeToken, mappedIndex, mapIt.first);
    }

    finalize(storage);
}

template <typename IndexHolder, typename ElementType>
void FabricMapper::buildParent(const std::unordered_map<omni::fabric::PathC, IndexHolder>& indexMap,
                               size_t maxTargetIndex,
                               std::function<size_t(const IndexHolder&)>&& extractIndexFunc,
                               const omni::fabric::TokenC& attributeToken)
{
    CARB_PROFILE_ZONE(1, "FabricMapper build parent %s", omni::fabric::Token(attributeToken).getText());
    if (m_srw.getId() == omni::fabric::kInvalidStageReaderWriterId)
    {
        CARB_LOG_ERROR("FabricMapper was not initialize, please call bindFabric to bind a valid Fabric cache");
        return;
    }
    MappingStorage& storage = initialize(attributeToken, maxTargetIndex, m_parentMappingStorage);
    omni::fabric::USDHierarchy usdHierarchy(m_srw.getFabricId());

    for (const auto& mapIt : indexMap)
    {
        omni::fabric::Path parentPath = usdHierarchy.getParent(omni::fabric::Path(mapIt.first));
        if (parentPath != omni::fabric::kUninitializedPath)
        {
            size_t mappedIndex = extractIndexFunc(mapIt.second);
            registerMapping<ElementType>(storage, attributeToken, mappedIndex, parentPath.asPathC());
        }
    }

    finalize(storage);
}


} // namespace physx
} // namespace omni
