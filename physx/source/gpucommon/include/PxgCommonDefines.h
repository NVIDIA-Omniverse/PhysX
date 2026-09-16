// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_COMMON_DEFINES_H
#define PXG_COMMON_DEFINES_H

// !! No includes here, only preprocessor definitions!

// A place for shared defines for the GPU libs

#define PXG_MAX_NUM_POINTS_PER_CONTACT_PATCH 6 // corresponding CPU define is CONTACT_REDUCTION_MAX_CONTACTS
#define LOG2_WARP_SIZE 5
#define WARP_SIZE (1U << LOG2_WARP_SIZE)
#define	FULL_MASK 0xffffffff //full mask for 32 thread in a warp


#endif