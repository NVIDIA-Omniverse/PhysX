// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_INTERSECTION_RAY_H
#define GU_INTERSECTION_RAY_H

// PT: small distance between a ray origin and a potentially hit surface. Should be small enough to
// limit accuracy issues coming from large distance values, but not too close to the surface to make
// sure we don't start inside the shape.
#define GU_RAY_SURFACE_OFFSET	10.0f

#endif
