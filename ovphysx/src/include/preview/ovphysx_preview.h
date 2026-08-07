// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "ovphysx/ovphysx_export.h"
#include "ovphysx/ovphysx_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Preview prim query APIs.
 *
 * These functions are not yet implemented and currently return
 * OVPHYSX_API_NOT_IMPLEMENTED. They remain available here for internal
 * experimentation and may change without notice.
 */

/*
* Find all prim paths matching a glob pattern that have the specified attribute.
* 
* Returns a list of USD prim paths that
* a) match the given glob pattern and
* b) have a specified attribute and
* c) are simulated by physx.
* The returned prim list must be destroyed with ovphysx_destroy_prim_list().
* 
* Glob pattern syntax:
*   - '*' matches any characters within a path segment (e.g., "/World/env*" matches "/World/env_0", "/World/environment")
*   - '?' matches a single character
*   - '**' matches across path separators recursively (e.g., "/World/ * * /robot" matches "/World/a/b/c/robot")
* 
* Examples (where h is handle, paths is output):
*   Find all env* prims with velocity attribute:
*     ovphysx_find_prims(h, "/World/env*", "physics:velocity", &paths);
*   Find all robot* prims at any depth with transform attribute:
*     ovphysx_find_prims(h, "/World/ * * /robot*", "xformOp:transform", &paths);
*   Find all single-char suffixed cubes:
*     ovphysx_find_prims(h, "/World/Cube_?", empty_string, &paths);
* 
* @param handle The ovphysx instance
* @param path_pattern Glob pattern for prim paths (e.g., "/World/env_*")
* @param attribute_name Attribute name to filter by (empty string matches all prims matching the pattern)
* @param out_prim_list [out] Result prim list (must destroy with ovphysx_destroy_prim_list)
* @return ovphysx_result_t with status and error info
*/
OVPHYSX_API ovphysx_result_t ovphysx_find_prims(
    ovphysx_handle_t handle,
    ovphysx_string_t path_pattern,
    ovphysx_string_t attribute_name,
    ovphysx_prim_list_t* out_prim_list
);

/*
* Destroy a prim list returned by ovphysx_find_prims().
* 
* Releases all memory associated with the prim list, including the path strings and array.
* After this call, the prim_list structure is zeroed and should not be used.
* 
* @param handle The ovphysx instance
* @param prim_list Pointer to prim list to destroy (can be NULL)
* @return ovphysx_result_t with status and error info
*/
OVPHYSX_API ovphysx_result_t ovphysx_destroy_prim_list(
    ovphysx_handle_t handle,
    ovphysx_prim_list_t* prim_list
);


#ifdef __cplusplus
} // extern "C"
#endif
