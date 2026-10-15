// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-COMP-OVD-001
// @covers AC-2 AC-4
// @implements REQ-CAPI-COMPANION-001
// @covers AC-1
#ifndef OVPHYSX_COMPANION_OVD_H
#define OVPHYSX_COMPANION_OVD_H
#include "companion.h"
#ifdef __cplusplus
extern "C" {
#endif
#define OVPHYSX_COMPANION_OVD_LIMIT 8u
#define OVPHYSX_COMPANION_OVD_MALFORMED 9u
typedef struct ovphysx_companion_ovd_limits {
    uint32_t size, version;
    uint64_t input_bytes;
    uint32_t records, payload_bytes, string_bytes, reserved;
} ovphysx_companion_ovd_limits_t;
typedef struct ovphysx_companion_ovd_diagnostic {
    uint32_t size, version;
    uint64_t offset;
    char message[160];
} ovphysx_companion_ovd_diagnostic_t;
typedef struct ovphysx_companion_ovd_info {
    uint32_t size, version, major, minor, patch, reserved;
    uint64_t records;
} ovphysx_companion_ovd_info_t;
typedef struct ovphysx_companion_ovd_record {
    uint32_t size, version, command, reserved;
    uint64_t offset, context, object, timestamp;
    uint32_t class_handle, base_class, attribute, data_type, elements, attribute_class;
    uint32_t enum_value, line, message_type, name_bytes, file_bytes, path_count, payload_bytes;
} ovphysx_companion_ovd_record_t;
/** Stopped native OVD decoder: no instance/attachment/USD required. Records retain
 * schema/context/object identifiers, raw frame timestamps and full attribute paths.
 * This is not a viewer or a geometry converter. Supported little-endian format:
 * 0.1.42 through 0.4.0; unsupported commands/version return UNAVAILABLE.
 * decode copies data before returning; it never starts/stops core recording.
 * Bounds: input<=512MiB, records<=1M, strings<=65535, payload<=input limit.
 * Successful documents own all memory until release and survive owner shutdown.
 * On decode failure, document is unchanged and diagnostic gives a bounded message
 * and byte offset. LIMIT=8 and MALFORMED=9 are table-specific status extensions.
 * copy kinds: 0 name bytes, 1 file bytes, 2 uint32 attribute path, 3 payload bytes.
 * No string terminator is counted. NULL/0 queries required bytes. Insufficient
 * capacity updates required_bytes only and writes no partial data.
 */
typedef struct ovphysx_companion_ovd_api {
    uint32_t size, version;
    uint64_t capabilities; /* bit0 stopped native CPU decoding */
    ovphysx_companion_status_t (*decode)(const void* data,uint64_t bytes,const ovphysx_companion_ovd_limits_t* limits,
        uint64_t* document,ovphysx_companion_ovd_diagnostic_t* diagnostic);
    ovphysx_companion_status_t (*describe)(uint64_t document,uint32_t size,ovphysx_companion_ovd_info_t* output);
    ovphysx_companion_status_t (*record)(uint64_t document,uint64_t index,uint32_t size,ovphysx_companion_ovd_record_t* output);
    ovphysx_companion_status_t (*copy)(uint64_t document,uint64_t index,uint32_t kind,void* data,uint64_t capacity_bytes,uint64_t* required_bytes);
    ovphysx_companion_status_t (*release)(uint64_t document);
} ovphysx_companion_ovd_api_t;
OVPHYSX_API ovphysx_companion_status_t ovphysx_companion_ovd_acquire(
    uint32_t version,uint32_t size,ovphysx_companion_ovd_api_t* output);
#ifdef __cplusplus
}
#endif
#endif
