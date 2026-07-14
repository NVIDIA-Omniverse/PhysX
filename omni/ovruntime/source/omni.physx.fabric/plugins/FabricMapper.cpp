// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "FabricMapper.h"

#include <cuda_runtime.h>


namespace omni
{
namespace physx
{

FabricMapper::MappingStorage::~MappingStorage()
{
    releaseGpu();
}

void FabricMapper::MappingStorage::reset(size_t size)
{
    m_objectToFabricCpu.resize(size, nullptr);
    releaseGpu();
    cudaMalloc(&m_objectToFabricGpu, size * sizeof(AttributePtrType));
    m_size = size;

    cudaError_t err = cudaGetLastError();
    if (err != cudaSuccess)
    {
        CARB_LOG_ERROR("FabricMapper - Cuda error in allocate on device: %s", cudaGetErrorString(err));
    }
}

void FabricMapper::MappingStorage::releaseGpu()
{
    if (m_objectToFabricGpu)
    {
        cudaFree(m_objectToFabricGpu);
        m_objectToFabricGpu = nullptr;
    }
}

FabricMapper::MappingStorage& FabricMapper::initialize(const omni::fabric::Token& attributeToken,
                                                       size_t maxTargetIndex,
                                                       std::unordered_map<omni::fabric::Token, MappingStorage>& map)
{
    MappingStorage& storage = map[attributeToken];
    storage.reset(maxTargetIndex);
    return storage;
}

void FabricMapper::bindFabric(const omni::fabric::FabricId& fabricId)
{
    if (fabricId != omni::fabric::kInvalidFabricId)
    {
        m_srw = omni::fabric::StageReaderWriter(fabricId);
    }
}

void FabricMapper::bindDevice(int deviceId)
{
    if (deviceId != m_deviceId)
    {
        clear();
        m_deviceId = deviceId;
    }
}

void FabricMapper::finalize(MappingStorage& storage)
{
    CARB_ASSERT(storage.m_objectToFabricGpu);
    cudaMemcpy(storage.m_objectToFabricGpu, storage.m_objectToFabricCpu.data(),
               storage.m_objectToFabricCpu.size() * sizeof(storage.m_objectToFabricCpu[0]), cudaMemcpyHostToDevice);

    cudaError_t err = cudaGetLastError();
    if (err != cudaSuccess)
    {
        CARB_LOG_ERROR("FabricMapper - Cuda error in copy to device: %s", cudaGetErrorString(err));
    }
}

size_t FabricMapper::getSize(const omni::fabric::Token& attributeToken) const
{
    const auto& it = m_mappingStorage.find(attributeToken);
    if (it != m_mappingStorage.end())
    {
        return it->second.m_size;
    }
    return 0;
}

FabricMapper::AttributePtrType* FabricMapper::getPtr(const omni::fabric::Token& attributeToken) const
{
    const auto& it = m_mappingStorage.find(attributeToken);
    if (it != m_mappingStorage.end())
    {
        return it->second.m_objectToFabricGpu;
    }
    return nullptr;
}

std::vector<omni::fabric::Token> FabricMapper::getAttributes() const
{
    std::vector<omni::fabric::Token> result;
    result.reserve(m_mappingStorage.size());

    for (const auto& stIt : m_mappingStorage)
    {
        result.push_back(stIt.first);
    }
    return result;
}

void FabricMapper::clear()
{
    m_mappingStorage.clear();
    m_deviceId = -1;
}

bool FabricMapper::empty() const
{
    return m_mappingStorage.empty();
}

int FabricMapper::deviceId() const
{
    return m_deviceId;
}

} // namespace physx
} // namespace omni
