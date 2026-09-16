// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef OMNI_PVD_HELPERS_H
#define OMNI_PVD_HELPERS_H

#include <stdint.h>

extern uint8_t OmniPvdCompressInt(uint64_t handle, uint8_t *bytes);
extern uint64_t OmniPvdDeCompressInt(uint8_t *bytes, uint8_t maxBytes);

#endif