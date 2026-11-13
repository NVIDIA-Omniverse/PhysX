// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "PvdDomDocument.h"
#include <tuple>

namespace PvdDomInspect
{
struct DocumentAccess
{
    static const std::vector<size_t>* scopedWrites(const Document& document, const ObjectHistory& record,
        uint64_t attribute, uint64_t recordingSegment, uint64_t context);
};
}
