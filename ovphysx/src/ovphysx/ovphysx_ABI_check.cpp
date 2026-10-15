// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include <stddef.h>
#include "ovphysx_ABI_check.h"
#include "ovphysx/dlpack/dlpack.h"
#include "ovphysx/ovphysx_legacy_types.h"
#include "ovphysx/ovphysx_types.h"


extern "C" {

size_t ovphysx_internal_create_args_sizeof()
{
    return sizeof(ovphysx_create_args);
}

size_t ovphysx_internal_create_args_alignof()
{
    return alignof(ovphysx_create_args);
}

size_t ovphysx_internal_create_args_offset(int field)
{
    switch (field)
    {
    case OVPHYSX_CA_BUNDLED_DEPS_PATH: return offsetof(ovphysx_create_args, bundled_deps_path);
    case OVPHYSX_CA_CONFIG_ENTRIES: return offsetof(ovphysx_create_args, config_entries);
    case OVPHYSX_CA_CONFIG_ENTRY_COUNT: return offsetof(ovphysx_create_args, config_entry_count);
    case OVPHYSX_CA_ACTIVE_CUDA_GPUS: return offsetof(ovphysx_create_args, active_cuda_gpus);
    default: return (size_t)~0ULL;
    }
}

// ovphysx_config_entry_t
size_t ovphysx_internal_config_entry_sizeof()
{
    return sizeof(ovphysx_config_entry_t);
}

size_t ovphysx_internal_config_entry_alignof()
{
    return alignof(ovphysx_config_entry_t);
}

size_t ovphysx_internal_config_entry_offset(int field)
{
    switch (field)
    {
    case OVPHYSX_CE_KEY_TYPE: return offsetof(ovphysx_config_entry_t, key_type);
    case OVPHYSX_CE_KEY: return offsetof(ovphysx_config_entry_t, key);
    case OVPHYSX_CE_VALUE: return offsetof(ovphysx_config_entry_t, value);
    default: return (size_t)~0ULL;
    }
}

// ovphysx_config_value_t
size_t ovphysx_internal_config_value_sizeof()
{
    return sizeof(ovphysx_config_value_t);
}

size_t ovphysx_internal_config_value_alignof()
{
    return alignof(ovphysx_config_value_t);
}

size_t ovphysx_internal_config_value_offset(int field)
{
    switch (field)
    {
    case OVPHYSX_CV_TYPE: return offsetof(ovphysx_config_value_t, type);
    case OVPHYSX_CV_VALUE: return offsetof(ovphysx_config_value_t, value);
    default: return (size_t)~0ULL;
    }
}

// ovphysx_omnipvd_destination_t
size_t ovphysx_internal_omnipvd_destination_sizeof()
{
    return sizeof(ovphysx_omnipvd_destination_t);
}

size_t ovphysx_internal_omnipvd_destination_alignof()
{
    return alignof(ovphysx_omnipvd_destination_t);
}

size_t ovphysx_internal_omnipvd_destination_offset(int field)
{
    switch (field)
    {
    case OVPHYSX_OD_TRANSPORT: return offsetof(ovphysx_omnipvd_destination_t, transport);
    case OVPHYSX_OD_FILE_PATH: return offsetof(ovphysx_omnipvd_destination_t, file_path);
    case OVPHYSX_OD_TCP_ADDRESS: return offsetof(ovphysx_omnipvd_destination_t, tcp_address);
    case OVPHYSX_OD_TCP_PORT: return offsetof(ovphysx_omnipvd_destination_t, tcp_port);
    case OVPHYSX_OD_TCP_TIMEOUT_MS: return offsetof(ovphysx_omnipvd_destination_t, tcp_timeout_ms);
    default: return (size_t)~0ULL;
    }
}

// DLDataType
size_t ovphysx_internal_dl_data_type_sizeof()
{
    return sizeof(DLDataType);
}

size_t ovphysx_internal_dl_data_type_alignof()
{
    return alignof(DLDataType);
}

size_t ovphysx_internal_dl_data_type_offset(int field)
{
    switch (field)
    {
    case DL_DT_CODE: return offsetof(DLDataType, code);
    case DL_DT_BITS: return offsetof(DLDataType, bits);
    case DL_DT_LANES: return offsetof(DLDataType, lanes);
    default: return (size_t)~0ULL;
    }
}

// DLDevice
size_t ovphysx_internal_dl_device_sizeof()
{
    return sizeof(DLDevice);
}

size_t ovphysx_internal_dl_device_alignof()
{
    return alignof(DLDevice);
}

size_t ovphysx_internal_dl_device_offset(int field)
{
    switch (field)
    {
    case DL_DEV_DEVICE_TYPE: return offsetof(DLDevice, device_type);
    case DL_DEV_DEVICE_ID: return offsetof(DLDevice, device_id);
    default: return (size_t)~0ULL;
    }
}

// DLTensor
size_t ovphysx_internal_dl_tensor_sizeof()
{
    return sizeof(DLTensor);
}

size_t ovphysx_internal_dl_tensor_alignof()
{
    return alignof(DLTensor);
}

size_t ovphysx_internal_dl_tensor_offset(int field)
{
    switch (field)
    {
    case DL_TENSOR_DATA: return offsetof(DLTensor, data);
    case DL_TENSOR_DEVICE: return offsetof(DLTensor, device);
    case DL_TENSOR_NDIM: return offsetof(DLTensor, ndim);
    case DL_TENSOR_DTYPE: return offsetof(DLTensor, dtype);
    case DL_TENSOR_SHAPE: return offsetof(DLTensor, shape);
    case DL_TENSOR_STRIDES: return offsetof(DLTensor, strides);
    case DL_TENSOR_BYTE_OFFSET: return offsetof(DLTensor, byte_offset);
    default: return (size_t)~0ULL;
    }
}

size_t ovphysx_internal_read_attribute_result_sizeof()
{
    return sizeof(ovphysx_read_attribute_result_t);
}

size_t ovphysx_internal_read_attribute_result_alignof()
{
    return alignof(ovphysx_read_attribute_result_t);
}

size_t ovphysx_internal_read_attribute_result_offset(int field)
{
    switch (field)
    {
    case OVPHYSX_RA_ATTRIBUTE: return offsetof(ovphysx_read_attribute_result_t, attribute);
    case OVPHYSX_RA_GROUP_COUNT: return offsetof(ovphysx_read_attribute_result_t, group_count);
    case OVPHYSX_RA_OMISSION_REASONS: return offsetof(ovphysx_read_attribute_result_t, omission_reasons);
    default: return (size_t)~0ULL;
    }
}

size_t ovphysx_internal_read_result_sizeof()
{
    return sizeof(ovphysx_read_result_t);
}

size_t ovphysx_internal_read_result_alignof()
{
    return alignof(ovphysx_read_result_t);
}

size_t ovphysx_internal_read_result_offset(int field)
{
    switch (field)
    {
    case OVPHYSX_RR_TOTAL_PRIM_COUNT: return offsetof(ovphysx_read_result_t, total_prim_count);
    case OVPHYSX_RR_ATTRIBUTE_COUNT: return offsetof(ovphysx_read_result_t, attribute_count);
    case OVPHYSX_RR_ATTRIBUTES: return offsetof(ovphysx_read_result_t, attributes);
    default: return (size_t)~0ULL;
    }
}

size_t ovphysx_internal_write_attribute_result_sizeof()
{
    return sizeof(ovphysx_write_attribute_result_t);
}

size_t ovphysx_internal_write_attribute_result_alignof()
{
    return alignof(ovphysx_write_attribute_result_t);
}

size_t ovphysx_internal_write_attribute_result_offset(int field)
{
    switch (field)
    {
    case OVPHYSX_WA_ATTRIBUTE: return offsetof(ovphysx_write_attribute_result_t, attribute);
    case OVPHYSX_WA_GROUP_COUNT: return offsetof(ovphysx_write_attribute_result_t, group_count);
    default: return (size_t)~0ULL;
    }
}

size_t ovphysx_internal_write_result_sizeof()
{
    return sizeof(ovphysx_write_result_t);
}

size_t ovphysx_internal_write_result_alignof()
{
    return alignof(ovphysx_write_result_t);
}

size_t ovphysx_internal_write_result_offset(int field)
{
    switch (field)
    {
    case OVPHYSX_WR_TOTAL_PRIM_COUNT: return offsetof(ovphysx_write_result_t, total_prim_count);
    case OVPHYSX_WR_ATTRIBUTE_COUNT: return offsetof(ovphysx_write_result_t, attribute_count);
    case OVPHYSX_WR_ATTRIBUTES: return offsetof(ovphysx_write_result_t, attributes);
    default: return (size_t)~0ULL;
    }
}

size_t ovphysx_internal_fetched_write_group_sizeof()
{
    return sizeof(ovphysx_fetched_write_group_t);
}

size_t ovphysx_internal_fetched_write_group_alignof()
{
    return alignof(ovphysx_fetched_write_group_t);
}

size_t ovphysx_internal_fetched_write_group_offset(int field)
{
    switch (field)
    {
    case OVPHYSX_FWG_GROUP: return offsetof(ovphysx_fetched_write_group_t, group);
    case OVPHYSX_FWG_ATTRIBUTE_INDEX: return offsetof(ovphysx_fetched_write_group_t, attribute_index);
    default: return (size_t)~0ULL;
    }
}

}
