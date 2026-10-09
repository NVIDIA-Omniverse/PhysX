// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

// Clamps a (count, startIndex) contact/friction layout pair to a capacity, in a form both the
// host and the CUDA kernel compile, so the two backends cannot drift. fetchRawRigidContactData
// (and its CPU-side equivalent) can increment countBuffer[i] past the buffer cap before the
// bounds check runs, so after a fill pass count[i] may still report the pre-clamp value even
// when only a smaller prefix was actually written. This corrects both arrays so that
// start[i] + count[i] <= cap for all i.
//
// No pxr and no PhysX includes: this header is pulled into a .cu.

#if defined(__CUDACC__)
#    define OVX_CONTACT_LAYOUT_HD __host__ __device__
#else
#    define OVX_CONTACT_LAYOUT_HD
#endif

namespace omni
{
namespace physx
{
namespace tensors
{

OVX_CONTACT_LAYOUT_HD inline void clampContactLayoutEntry(unsigned int* counts,
                                                            unsigned int* startIndices,
                                                            unsigned int i,
                                                            unsigned int cap)
{
    const unsigned int start = startIndices[i];
    if (start >= cap)
    {
        startIndices[i] = cap;
        counts[i] = 0;
    }
    else if (start + counts[i] > cap)
    {
        counts[i] = cap - start;
    }
}

} // namespace tensors
} // namespace physx
} // namespace omni

// Scoped to this header: every use is above, and leaving it defined would put a bare OVX_* macro in
// every translation unit that includes this one.
#undef OVX_CONTACT_LAYOUT_HD
