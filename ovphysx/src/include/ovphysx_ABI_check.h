// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef OVPHYSX_ABI_CHECK_H
#define OVPHYSX_ABI_CHECK_H

#include <stddef.h>
#include <stdint.h>
#include "ovphysx/ovphysx_export.h"

#ifdef __cplusplus
extern "C" {
#endif

// Field indices for ovphysx_create_args
enum ovphysx_create_args_field
{
    OVPHYSX_CA_BUNDLED_DEPS_PATH = 0,
    OVPHYSX_CA_CONFIG_ENTRIES = 1,
    OVPHYSX_CA_CONFIG_ENTRY_COUNT = 2,
    OVPHYSX_CA_ACTIVE_CUDA_GPUS = 3
};

// Field indices for ovphysx_config_entry_t
enum ovphysx_config_entry_field
{
    OVPHYSX_CE_KEY_TYPE = 0,
    OVPHYSX_CE_KEY = 1,
    OVPHYSX_CE_VALUE = 2
};

// Field indices for ovphysx_config_value_t
enum ovphysx_config_value_field
{
    OVPHYSX_CV_TYPE = 0,
    OVPHYSX_CV_VALUE = 1
};

// Field indices for ovphysx_omnipvd_destination_t
enum ovphysx_omnipvd_destination_field
{
    OVPHYSX_OD_TRANSPORT = 0,
    OVPHYSX_OD_FILE_PATH = 1,
    OVPHYSX_OD_TCP_ADDRESS = 2,
    OVPHYSX_OD_TCP_PORT = 3,
    OVPHYSX_OD_TCP_TIMEOUT_MS = 4
};


// Field indices for DLDataType
enum dl_data_type_field
{
    DL_DT_CODE = 0,
    DL_DT_BITS = 1,
    DL_DT_LANES = 2
};

// Field indices for DLDevice
enum dl_device_field
{
    DL_DEV_DEVICE_TYPE = 0,
    DL_DEV_DEVICE_ID = 1
};

// Field indices for DLTensor
enum dl_tensor_field
{
    DL_TENSOR_DATA = 0,
    DL_TENSOR_DEVICE = 1,
    DL_TENSOR_NDIM = 2,
    DL_TENSOR_DTYPE = 3,
    DL_TENSOR_SHAPE = 4,
    DL_TENSOR_STRIDES = 5,
    DL_TENSOR_BYTE_OFFSET = 6
};

// Field indices for ovphysx_read_attribute_result_t
enum ovphysx_read_attribute_result_field
{
    OVPHYSX_RA_ATTRIBUTE = 0,
    OVPHYSX_RA_GROUP_COUNT = 1,
    OVPHYSX_RA_OMISSION_REASONS = 2
};

// Field indices for ovphysx_read_result_t
enum ovphysx_read_result_field
{
    OVPHYSX_RR_TOTAL_PRIM_COUNT = 0,
    OVPHYSX_RR_ATTRIBUTE_COUNT = 1,
    OVPHYSX_RR_ATTRIBUTES = 2
};

// Field indices for ovphysx_write_attribute_result_t
enum ovphysx_write_attribute_result_field
{
    OVPHYSX_WA_ATTRIBUTE = 0,
    OVPHYSX_WA_GROUP_COUNT = 1
};

// Field indices for ovphysx_write_result_t
enum ovphysx_write_result_field
{
    OVPHYSX_WR_TOTAL_PRIM_COUNT = 0,
    OVPHYSX_WR_ATTRIBUTE_COUNT = 1,
    OVPHYSX_WR_ATTRIBUTES = 2
};

// Field indices for ovphysx_fetched_write_group_t
enum ovphysx_fetched_write_group_field
{
    OVPHYSX_FWG_GROUP = 0,
    OVPHYSX_FWG_ATTRIBUTE_INDEX = 1
};

// ABI helpers for struct layout verification from Python (internal)
OVPHYSX_API size_t ovphysx_internal_create_args_sizeof();
OVPHYSX_API size_t ovphysx_internal_create_args_alignof();
OVPHYSX_API size_t ovphysx_internal_create_args_offset(int field);

OVPHYSX_API size_t ovphysx_internal_config_entry_sizeof();
OVPHYSX_API size_t ovphysx_internal_config_entry_alignof();
OVPHYSX_API size_t ovphysx_internal_config_entry_offset(int field);

OVPHYSX_API size_t ovphysx_internal_config_value_sizeof();
OVPHYSX_API size_t ovphysx_internal_config_value_alignof();
OVPHYSX_API size_t ovphysx_internal_config_value_offset(int field);

OVPHYSX_API size_t ovphysx_internal_omnipvd_destination_sizeof();
OVPHYSX_API size_t ovphysx_internal_omnipvd_destination_alignof();
OVPHYSX_API size_t ovphysx_internal_omnipvd_destination_offset(int field);

OVPHYSX_API size_t ovphysx_internal_dl_data_type_sizeof();
OVPHYSX_API size_t ovphysx_internal_dl_data_type_alignof();
OVPHYSX_API size_t ovphysx_internal_dl_data_type_offset(int field);

OVPHYSX_API size_t ovphysx_internal_dl_device_sizeof();
OVPHYSX_API size_t ovphysx_internal_dl_device_alignof();
OVPHYSX_API size_t ovphysx_internal_dl_device_offset(int field);

OVPHYSX_API size_t ovphysx_internal_dl_tensor_sizeof();
OVPHYSX_API size_t ovphysx_internal_dl_tensor_alignof();
OVPHYSX_API size_t ovphysx_internal_dl_tensor_offset(int field);

OVPHYSX_API size_t ovphysx_internal_read_attribute_result_sizeof();
OVPHYSX_API size_t ovphysx_internal_read_attribute_result_alignof();
OVPHYSX_API size_t ovphysx_internal_read_attribute_result_offset(int field);

OVPHYSX_API size_t ovphysx_internal_read_result_sizeof();
OVPHYSX_API size_t ovphysx_internal_read_result_alignof();
OVPHYSX_API size_t ovphysx_internal_read_result_offset(int field);

OVPHYSX_API size_t ovphysx_internal_write_attribute_result_sizeof();
OVPHYSX_API size_t ovphysx_internal_write_attribute_result_alignof();
OVPHYSX_API size_t ovphysx_internal_write_attribute_result_offset(int field);

OVPHYSX_API size_t ovphysx_internal_write_result_sizeof();
OVPHYSX_API size_t ovphysx_internal_write_result_alignof();
OVPHYSX_API size_t ovphysx_internal_write_result_offset(int field);

OVPHYSX_API size_t ovphysx_internal_fetched_write_group_sizeof();
OVPHYSX_API size_t ovphysx_internal_fetched_write_group_alignof();
OVPHYSX_API size_t ovphysx_internal_fetched_write_group_offset(int field);

#ifdef __cplusplus
}
#endif

#endif // OVPHYSX_ABI_CHECK_H
