// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

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

// Field indices for ovphysx_contact_event_header_t
// (must match the field order in python/ovphysx/contact_types.py:ContactEventHeader)
enum ovphysx_contact_event_header_field
{
    OVPHYSX_CEH_TYPE = 0,
    OVPHYSX_CEH_STAGE_ID = 1,
    OVPHYSX_CEH_ACTOR0 = 2,
    OVPHYSX_CEH_ACTOR1 = 3,
    OVPHYSX_CEH_COLLIDER0 = 4,
    OVPHYSX_CEH_COLLIDER1 = 5,
    OVPHYSX_CEH_CONTACT_DATA_OFFSET = 6,
    OVPHYSX_CEH_NUM_CONTACT_DATA = 7,
    OVPHYSX_CEH_FRICTION_ANCHORS_DATA_OFFSET = 8,
    OVPHYSX_CEH_NUM_FRICTION_ANCHORS_DATA = 9,
    OVPHYSX_CEH_PROTO_INDEX0 = 10,
    OVPHYSX_CEH_PROTO_INDEX1 = 11
};

// Field indices for ovphysx_contact_point_t
enum ovphysx_contact_point_field
{
    OVPHYSX_CP_POSITION = 0,
    OVPHYSX_CP_NORMAL = 1,
    OVPHYSX_CP_IMPULSE = 2,
    OVPHYSX_CP_SEPARATION = 3,
    OVPHYSX_CP_FACE_INDEX0 = 4,
    OVPHYSX_CP_FACE_INDEX1 = 5,
    OVPHYSX_CP_MATERIAL0 = 6,
    OVPHYSX_CP_MATERIAL1 = 7
};

// Field indices for ovphysx_friction_anchor_t
enum ovphysx_friction_anchor_field
{
    OVPHYSX_FA_POSITION = 0,
    OVPHYSX_FA_IMPULSE = 1
};

// ABI helpers for struct layout verification from Python (internal)
OVPHYSX_API size_t ovphysx_internal_create_args_sizeof();
OVPHYSX_API size_t ovphysx_internal_create_args_alignof();
OVPHYSX_API size_t ovphysx_internal_create_args_offset(int field);

OVPHYSX_API size_t ovphysx_internal_config_entry_sizeof();
OVPHYSX_API size_t ovphysx_internal_config_entry_alignof();
OVPHYSX_API size_t ovphysx_internal_config_entry_offset(int field);

OVPHYSX_API size_t ovphysx_internal_dl_data_type_sizeof();
OVPHYSX_API size_t ovphysx_internal_dl_data_type_alignof();
OVPHYSX_API size_t ovphysx_internal_dl_data_type_offset(int field);

OVPHYSX_API size_t ovphysx_internal_dl_device_sizeof();
OVPHYSX_API size_t ovphysx_internal_dl_device_alignof();
OVPHYSX_API size_t ovphysx_internal_dl_device_offset(int field);

OVPHYSX_API size_t ovphysx_internal_dl_tensor_sizeof();
OVPHYSX_API size_t ovphysx_internal_dl_tensor_alignof();
OVPHYSX_API size_t ovphysx_internal_dl_tensor_offset(int field);

OVPHYSX_API size_t ovphysx_internal_contact_event_header_sizeof();
OVPHYSX_API size_t ovphysx_internal_contact_event_header_alignof();
OVPHYSX_API size_t ovphysx_internal_contact_event_header_offset(int field);

OVPHYSX_API size_t ovphysx_internal_contact_point_sizeof();
OVPHYSX_API size_t ovphysx_internal_contact_point_alignof();
OVPHYSX_API size_t ovphysx_internal_contact_point_offset(int field);

OVPHYSX_API size_t ovphysx_internal_friction_anchor_sizeof();
OVPHYSX_API size_t ovphysx_internal_friction_anchor_alignof();
OVPHYSX_API size_t ovphysx_internal_friction_anchor_offset(int field);

#ifdef __cplusplus
}
#endif

#endif // OVPHYSX_ABI_CHECK_H
