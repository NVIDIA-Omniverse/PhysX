// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#include <stddef.h>
#include "ovphysx_ABI_check.h"
#include "ovphysx/dlpack/dlpack.h"
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

// ovphysx_contact_event_header_t
size_t ovphysx_internal_contact_event_header_sizeof()
{
    return sizeof(ovphysx_contact_event_header_t);
}

size_t ovphysx_internal_contact_event_header_alignof()
{
    return alignof(ovphysx_contact_event_header_t);
}

size_t ovphysx_internal_contact_event_header_offset(int field)
{
    switch (field)
    {
    case OVPHYSX_CEH_TYPE: return offsetof(ovphysx_contact_event_header_t, type);
    case OVPHYSX_CEH_STAGE_ID: return offsetof(ovphysx_contact_event_header_t, stageId);
    case OVPHYSX_CEH_ACTOR0: return offsetof(ovphysx_contact_event_header_t, actor0);
    case OVPHYSX_CEH_ACTOR1: return offsetof(ovphysx_contact_event_header_t, actor1);
    case OVPHYSX_CEH_COLLIDER0: return offsetof(ovphysx_contact_event_header_t, collider0);
    case OVPHYSX_CEH_COLLIDER1: return offsetof(ovphysx_contact_event_header_t, collider1);
    case OVPHYSX_CEH_CONTACT_DATA_OFFSET: return offsetof(ovphysx_contact_event_header_t, contactDataOffset);
    case OVPHYSX_CEH_NUM_CONTACT_DATA: return offsetof(ovphysx_contact_event_header_t, numContactData);
    case OVPHYSX_CEH_FRICTION_ANCHORS_DATA_OFFSET: return offsetof(ovphysx_contact_event_header_t, frictionAnchorsDataOffset);
    case OVPHYSX_CEH_NUM_FRICTION_ANCHORS_DATA: return offsetof(ovphysx_contact_event_header_t, numfrictionAnchorsData);
    case OVPHYSX_CEH_PROTO_INDEX0: return offsetof(ovphysx_contact_event_header_t, protoIndex0);
    case OVPHYSX_CEH_PROTO_INDEX1: return offsetof(ovphysx_contact_event_header_t, protoIndex1);
    default: return (size_t)~0ULL;
    }
}

// ovphysx_contact_point_t
size_t ovphysx_internal_contact_point_sizeof()
{
    return sizeof(ovphysx_contact_point_t);
}

size_t ovphysx_internal_contact_point_alignof()
{
    return alignof(ovphysx_contact_point_t);
}

size_t ovphysx_internal_contact_point_offset(int field)
{
    switch (field)
    {
    case OVPHYSX_CP_POSITION: return offsetof(ovphysx_contact_point_t, position);
    case OVPHYSX_CP_NORMAL: return offsetof(ovphysx_contact_point_t, normal);
    case OVPHYSX_CP_IMPULSE: return offsetof(ovphysx_contact_point_t, impulse);
    case OVPHYSX_CP_SEPARATION: return offsetof(ovphysx_contact_point_t, separation);
    case OVPHYSX_CP_FACE_INDEX0: return offsetof(ovphysx_contact_point_t, faceIndex0);
    case OVPHYSX_CP_FACE_INDEX1: return offsetof(ovphysx_contact_point_t, faceIndex1);
    case OVPHYSX_CP_MATERIAL0: return offsetof(ovphysx_contact_point_t, material0);
    case OVPHYSX_CP_MATERIAL1: return offsetof(ovphysx_contact_point_t, material1);
    default: return (size_t)~0ULL;
    }
}

// ovphysx_friction_anchor_t
size_t ovphysx_internal_friction_anchor_sizeof()
{
    return sizeof(ovphysx_friction_anchor_t);
}

size_t ovphysx_internal_friction_anchor_alignof()
{
    return alignof(ovphysx_friction_anchor_t);
}

size_t ovphysx_internal_friction_anchor_offset(int field)
{
    switch (field)
    {
    case OVPHYSX_FA_POSITION: return offsetof(ovphysx_friction_anchor_t, position);
    case OVPHYSX_FA_IMPULSE: return offsetof(ovphysx_friction_anchor_t, impulse);
    default: return (size_t)~0ULL;
    }
}

}
