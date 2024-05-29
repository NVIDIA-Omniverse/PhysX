// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2016-2024 NVIDIA Corporation. All rights reserved.


#include "TestAssets.h"
#include "AssetGenerator.h"
#include <algorithm>

const NvBlastChunkDesc g_cube1ChunkDescs[9] =
{
//    centroid              volume  parent idx  flags                           ID
    { { 0.0f, 0.0f, 0.0f }, 8.0f,   UINT32_MAX, NvBlastChunkDesc::NoFlags,      0 },
    { {-0.5f,-0.5f,-0.5f }, 1.0f,   0,          NvBlastChunkDesc::SupportFlag,  1 },
    { { 0.5f,-0.5f,-0.5f }, 1.0f,   0,          NvBlastChunkDesc::SupportFlag,  2 },
    { {-0.5f, 0.5f,-0.5f }, 1.0f,   0,          NvBlastChunkDesc::SupportFlag,  3 },
    { { 0.5f, 0.5f,-0.5f }, 1.0f,   0,          NvBlastChunkDesc::SupportFlag,  4 },
    { {-0.5f,-0.5f, 0.5f }, 1.0f,   0,          NvBlastChunkDesc::SupportFlag,  5 },
    { { 0.5f,-0.5f, 0.5f }, 1.0f,   0,          NvBlastChunkDesc::SupportFlag,  6 },
    { {-0.5f, 0.5f, 0.5f }, 1.0f,   0,          NvBlastChunkDesc::SupportFlag,  7 },
    { { 0.5f, 0.5f, 0.5f }, 1.0f,   0,          NvBlastChunkDesc::SupportFlag,  8 },
};

const NvBlastBondDesc g_cube1BondDescs[12] =
{
//      normal                area  centroid              userData chunks
    { { { 1.0f, 0.0f, 0.0f }, 1.0f, { 0.0f,-0.5f,-0.5f }, 0 }, { 1, 2 } },
    { { { 1.0f, 0.0f, 0.0f }, 1.0f, { 0.0f, 0.5f,-0.5f }, 0 }, { 3, 4 } },
    { { { 1.0f, 0.0f, 0.0f }, 1.0f, { 0.0f,-0.5f, 0.5f }, 0 }, { 5, 6 } },
    { { { 1.0f, 0.0f, 0.0f }, 1.0f, { 0.0f, 0.5f, 0.5f }, 0 }, { 7, 8 } },

    { { { 0.0f, 1.0f, 0.0f }, 1.0f, {-0.5f, 0.0f,-0.5f }, 0 }, { 1, 3 } },
    { { { 0.0f, 1.0f, 0.0f }, 1.0f, { 0.5f, 0.0f,-0.5f }, 0 }, { 2, 4 } },
    { { { 0.0f, 1.0f, 0.0f }, 1.0f, {-0.5f, 0.0f, 0.5f }, 0 }, { 5, 7 } },
    { { { 0.0f, 1.0f, 0.0f }, 1.0f, { 0.5f, 0.0f, 0.5f }, 0 }, { 6, 8 } },

    { { { 0.0f, 0.0f, 1.0f }, 1.0f, {-0.5f,-0.5f, 0.0f }, 0 }, { 1, 5 } },
    { { { 0.0f, 0.0f, 1.0f }, 1.0f, { 0.5f,-0.5f, 0.0f }, 0 }, { 2, 6 } },
    { { { 0.0f, 0.0f, 1.0f }, 1.0f, {-0.5f, 0.5f, 0.0f }, 0 }, { 3, 7 } },
    { { { 0.0f, 0.0f, 1.0f }, 1.0f, { 0.5f, 0.5f, 0.0f }, 0 }, { 4, 8 } },
};

const NvBlastBondDesc g_cube1BondDescs_wb[16] =
{
//      normal                area  centroid              userData chunks
    { { { 1.0f, 0.0f, 0.0f }, 1.0f, { 0.0f,-0.5f,-0.5f }, 0 }, { 1, 2 }          },
    { { { 1.0f, 0.0f, 0.0f }, 1.0f, { 0.0f, 0.5f,-0.5f }, 0 }, { 3, 4 }          },
    { { { 1.0f, 0.0f, 0.0f }, 1.0f, { 0.0f,-0.5f, 0.5f }, 0 }, { 5, 6 }          },
    { { { 1.0f, 0.0f, 0.0f }, 1.0f, { 0.0f, 0.5f, 0.5f }, 0 }, { 7, 8 }          },
                                                                            
    { { { 0.0f, 1.0f, 0.0f }, 1.0f, {-0.5f, 0.0f,-0.5f }, 0 }, { 1, 3 }          },
    { { { 0.0f, 1.0f, 0.0f }, 1.0f, { 0.5f, 0.0f,-0.5f }, 0 }, { 2, 4 }          },
    { { { 0.0f, 1.0f, 0.0f }, 1.0f, {-0.5f, 0.0f, 0.5f }, 0 }, { 5, 7 }          },
    { { { 0.0f, 1.0f, 0.0f }, 1.0f, { 0.5f, 0.0f, 0.5f }, 0 }, { 6, 8 }          },
                                                                            
    { { { 0.0f, 0.0f, 1.0f }, 1.0f, {-0.5f,-0.5f, 0.0f }, 0 }, { 1, 5 }          },
    { { { 0.0f, 0.0f, 1.0f }, 1.0f, { 0.5f,-0.5f, 0.0f }, 0 }, { 2, 6 }          },
    { { { 0.0f, 0.0f, 1.0f }, 1.0f, {-0.5f, 0.5f, 0.0f }, 0 }, { 3, 7 }          },
    { { { 0.0f, 0.0f, 1.0f }, 1.0f, { 0.5f, 0.5f, 0.0f }, 0 }, { 4, 8 }          },

    { { { 0.0f, 0.0f,-1.0f }, 1.0f, {-0.5f,-0.5f,-1.0f }, 0 }, { 1, UINT32_MAX } },
    { { { 0.0f, 0.0f,-1.0f }, 1.0f, { 0.5f,-0.5f,-1.0f }, 0 }, { 2, UINT32_MAX } },
    { { { 0.0f, 0.0f,-1.0f }, 1.0f, {-0.5f, 0.5f,-1.0f }, 0 }, { 3, UINT32_MAX } },
    { { { 0.0f, 0.0f,-1.0f }, 1.0f, { 0.5f, 0.5f,-1.0f }, 0 }, { 4, UINT32_MAX } },
};

const NvBlastChunkDesc g_cube2ChunkDescs[73] =
{
//    centroid                                volume  parent idx  flags                           ID
    { { 0.0f,  0.0f,  0.0f },                 8.0f,   UINT32_MAX, NvBlastChunkDesc::NoFlags,      0  },
    { {-0.5f, -0.5f, -0.5f },                 1.0f,   0,          NvBlastChunkDesc::SupportFlag,  1  },
    { { 0.5f, -0.5f, -0.5f },                 1.0f,   0,          NvBlastChunkDesc::SupportFlag,  2  },
    { {-0.5f,  0.5f, -0.5f },                 1.0f,   0,          NvBlastChunkDesc::SupportFlag,  3  },
    { { 0.5f,  0.5f, -0.5f },                 1.0f,   0,          NvBlastChunkDesc::SupportFlag,  4  },
    { {-0.5f, -0.5f,  0.5f },                 1.0f,   0,          NvBlastChunkDesc::SupportFlag,  5  },
    { { 0.5f, -0.5f,  0.5f },                 1.0f,   0,          NvBlastChunkDesc::SupportFlag,  6  },
    { {-0.5f,  0.5f,  0.5f },                 1.0f,   0,          NvBlastChunkDesc::SupportFlag,  7  },
    { { 0.5f,  0.5f,  0.5f },                 1.0f,   0,          NvBlastChunkDesc::SupportFlag,  8  },
    { {-0.25f-0.5f,-0.25f-0.5f,-0.25f-0.5f }, 0.125f, 1,          NvBlastChunkDesc::NoFlags,      9  },
    { { 0.25f-0.5f,-0.25f-0.5f,-0.25f-0.5f }, 0.125f, 1,          NvBlastChunkDesc::NoFlags,      10 },
    { {-0.25f-0.5f, 0.25f-0.5f,-0.25f-0.5f }, 0.125f, 1,          NvBlastChunkDesc::NoFlags,      11 },
    { { 0.25f-0.5f, 0.25f-0.5f,-0.25f-0.5f }, 0.125f, 1,          NvBlastChunkDesc::NoFlags,      12 },
    { {-0.25f-0.5f,-0.25f-0.5f, 0.25f-0.5f }, 0.125f, 1,          NvBlastChunkDesc::NoFlags,      13 },
    { { 0.25f-0.5f,-0.25f-0.5f, 0.25f-0.5f }, 0.125f, 1,          NvBlastChunkDesc::NoFlags,      14 },
    { {-0.25f-0.5f, 0.25f-0.5f, 0.25f-0.5f }, 0.125f, 1,          NvBlastChunkDesc::NoFlags,      15 },
    { { 0.25f-0.5f, 0.25f-0.5f, 0.25f-0.5f }, 0.125f, 1,          NvBlastChunkDesc::NoFlags,      16 },
    { {-0.25f+0.5f,-0.25f-0.5f,-0.25f-0.5f }, 0.125f, 2,          NvBlastChunkDesc::NoFlags,      17 },
    { { 0.25f+0.5f,-0.25f-0.5f,-0.25f-0.5f }, 0.125f, 2,          NvBlastChunkDesc::NoFlags,      18 },
    { {-0.25f+0.5f, 0.25f-0.5f,-0.25f-0.5f }, 0.125f, 2,          NvBlastChunkDesc::NoFlags,      19 },
    { { 0.25f+0.5f, 0.25f-0.5f,-0.25f-0.5f }, 0.125f, 2,          NvBlastChunkDesc::NoFlags,      20 },
    { {-0.25f+0.5f,-0.25f-0.5f, 0.25f-0.5f }, 0.125f, 2,          NvBlastChunkDesc::NoFlags,      21 },
    { { 0.25f+0.5f,-0.25f-0.5f, 0.25f-0.5f }, 0.125f, 2,          NvBlastChunkDesc::NoFlags,      22 },
    { {-0.25f+0.5f, 0.25f-0.5f, 0.25f-0.5f }, 0.125f, 2,          NvBlastChunkDesc::NoFlags,      23 },
    { { 0.25f+0.5f, 0.25f-0.5f, 0.25f-0.5f }, 0.125f, 2,          NvBlastChunkDesc::NoFlags,      24 },
    { {-0.25f-0.5f,-0.25f+0.5f,-0.25f-0.5f }, 0.125f, 3,          NvBlastChunkDesc::NoFlags,      25 },
    { { 0.25f-0.5f,-0.25f+0.5f,-0.25f-0.5f }, 0.125f, 3,          NvBlastChunkDesc::NoFlags,      26 },
    { {-0.25f-0.5f, 0.25f+0.5f,-0.25f-0.5f }, 0.125f, 3,          NvBlastChunkDesc::NoFlags,      27 },
    { { 0.25f-0.5f, 0.25f+0.5f,-0.25f-0.5f }, 0.125f, 3,          NvBlastChunkDesc::NoFlags,      28 },
    { {-0.25f-0.5f,-0.25f+0.5f, 0.25f-0.5f }, 0.125f, 3,          NvBlastChunkDesc::NoFlags,      29 },
    { { 0.25f-0.5f,-0.25f+0.5f, 0.25f-0.5f }, 0.125f, 3,          NvBlastChunkDesc::NoFlags,      30 },
    { {-0.25f-0.5f, 0.25f+0.5f, 0.25f-0.5f }, 0.125f, 3,          NvBlastChunkDesc::NoFlags,      31 },
    { { 0.25f-0.5f, 0.25f+0.5f, 0.25f-0.5f }, 0.125f, 3,          NvBlastChunkDesc::NoFlags,      32 },
    { {-0.25f+0.5f,-0.25f+0.5f,-0.25f-0.5f }, 0.125f, 4,          NvBlastChunkDesc::NoFlags,      33 },
    { { 0.25f+0.5f,-0.25f+0.5f,-0.25f-0.5f }, 0.125f, 4,          NvBlastChunkDesc::NoFlags,      34 },
    { {-0.25f+0.5f, 0.25f+0.5f,-0.25f-0.5f }, 0.125f, 4,          NvBlastChunkDesc::NoFlags,      35 },
    { { 0.25f+0.5f, 0.25f+0.5f,-0.25f-0.5f }, 0.125f, 4,          NvBlastChunkDesc::NoFlags,      36 },
    { {-0.25f+0.5f,-0.25f+0.5f, 0.25f-0.5f }, 0.125f, 4,          NvBlastChunkDesc::NoFlags,      37 },
    { { 0.25f+0.5f,-0.25f+0.5f, 0.25f-0.5f }, 0.125f, 4,          NvBlastChunkDesc::NoFlags,      38 },
    { {-0.25f+0.5f, 0.25f+0.5f, 0.25f-0.5f }, 0.125f, 4,          NvBlastChunkDesc::NoFlags,      39 },
    { { 0.25f+0.5f, 0.25f+0.5f, 0.25f-0.5f }, 0.125f, 4,          NvBlastChunkDesc::NoFlags,      40 },
    { {-0.25f-0.5f,-0.25f-0.5f,-0.25f+0.5f }, 0.125f, 5,          NvBlastChunkDesc::NoFlags,      41 },
    { { 0.25f-0.5f,-0.25f-0.5f,-0.25f+0.5f }, 0.125f, 5,          NvBlastChunkDesc::NoFlags,      42 },
    { {-0.25f-0.5f, 0.25f-0.5f,-0.25f+0.5f }, 0.125f, 5,          NvBlastChunkDesc::NoFlags,      43 },
    { { 0.25f-0.5f, 0.25f-0.5f,-0.25f+0.5f }, 0.125f, 5,          NvBlastChunkDesc::NoFlags,      44 },
    { {-0.25f-0.5f,-0.25f-0.5f, 0.25f+0.5f }, 0.125f, 5,          NvBlastChunkDesc::NoFlags,      45 },
    { { 0.25f-0.5f,-0.25f-0.5f, 0.25f+0.5f }, 0.125f, 5,          NvBlastChunkDesc::NoFlags,      46 },
    { {-0.25f-0.5f, 0.25f-0.5f, 0.25f+0.5f }, 0.125f, 5,          NvBlastChunkDesc::NoFlags,      47 },
    { { 0.25f-0.5f, 0.25f-0.5f, 0.25f+0.5f }, 0.125f, 5,          NvBlastChunkDesc::NoFlags,      48 },
    { {-0.25f+0.5f,-0.25f-0.5f,-0.25f+0.5f }, 0.125f, 6,          NvBlastChunkDesc::NoFlags,      49 },
    { { 0.25f+0.5f,-0.25f-0.5f,-0.25f+0.5f }, 0.125f, 6,          NvBlastChunkDesc::NoFlags,      50 },
    { {-0.25f+0.5f, 0.25f-0.5f,-0.25f+0.5f }, 0.125f, 6,          NvBlastChunkDesc::NoFlags,      51 },
    { { 0.25f+0.5f, 0.25f-0.5f,-0.25f+0.5f }, 0.125f, 6,          NvBlastChunkDesc::NoFlags,      52 },
    { {-0.25f+0.5f,-0.25f-0.5f, 0.25f+0.5f }, 0.125f, 6,          NvBlastChunkDesc::NoFlags,      53 },
    { { 0.25f+0.5f,-0.25f-0.5f, 0.25f+0.5f }, 0.125f, 6,          NvBlastChunkDesc::NoFlags,      54 },
    { {-0.25f+0.5f, 0.25f-0.5f, 0.25f+0.5f }, 0.125f, 6,          NvBlastChunkDesc::NoFlags,      55 },
    { { 0.25f+0.5f, 0.25f-0.5f, 0.25f+0.5f }, 0.125f, 6,          NvBlastChunkDesc::NoFlags,      56 },
    { {-0.25f-0.5f,-0.25f+0.5f,-0.25f+0.5f }, 0.125f, 7,          NvBlastChunkDesc::NoFlags,      57 },
    { { 0.25f-0.5f,-0.25f+0.5f,-0.25f+0.5f }, 0.125f, 7,          NvBlastChunkDesc::NoFlags,      58 },
    { {-0.25f-0.5f, 0.25f+0.5f,-0.25f+0.5f }, 0.125f, 7,          NvBlastChunkDesc::NoFlags,      59 },
    { { 0.25f-0.5f, 0.25f+0.5f,-0.25f+0.5f }, 0.125f, 7,          NvBlastChunkDesc::NoFlags,      60 },
    { {-0.25f-0.5f,-0.25f+0.5f, 0.25f+0.5f }, 0.125f, 7,          NvBlastChunkDesc::NoFlags,      61 },
    { { 0.25f-0.5f,-0.25f+0.5f, 0.25f+0.5f }, 0.125f, 7,          NvBlastChunkDesc::NoFlags,      62 },
    { {-0.25f-0.5f, 0.25f+0.5f, 0.25f+0.5f }, 0.125f, 7,          NvBlastChunkDesc::NoFlags,      63 },
    { { 0.25f-0.5f, 0.25f+0.5f, 0.25f+0.5f }, 0.125f, 7,          NvBlastChunkDesc::NoFlags,      64 },
    { {-0.25f+0.5f,-0.25f+0.5f,-0.25f+0.5f }, 0.125f, 8,          NvBlastChunkDesc::NoFlags,      65 },
    { { 0.25f+0.5f,-0.25f+0.5f,-0.25f+0.5f }, 0.125f, 8,          NvBlastChunkDesc::NoFlags,      66 },
    { {-0.25f+0.5f, 0.25f+0.5f,-0.25f+0.5f }, 0.125f, 8,          NvBlastChunkDesc::NoFlags,      67 },
    { { 0.25f+0.5f, 0.25f+0.5f,-0.25f+0.5f }, 0.125f, 8,          NvBlastChunkDesc::NoFlags,      68 },
    { {-0.25f+0.5f,-0.25f+0.5f, 0.25f+0.5f }, 0.125f, 8,          NvBlastChunkDesc::NoFlags,      69 },
    { { 0.25f+0.5f,-0.25f+0.5f, 0.25f+0.5f }, 0.125f, 8,          NvBlastChunkDesc::NoFlags,      70 },
    { {-0.25f+0.5f, 0.25f+0.5f, 0.25f+0.5f }, 0.125f, 8,          NvBlastChunkDesc::NoFlags,      71 },
    { { 0.25f+0.5f, 0.25f+0.5f, 0.25f+0.5f }, 0.125f, 8,          NvBlastChunkDesc::NoFlags,      72 },
};

const NvBlastChunkDesc g_cube3ChunkDescs[11] =
{
//    centroid              volume  parent idx  flags                           ID
    { { 0.0f, 0.0f, 0.0f }, 4.0f,   UINT32_MAX, NvBlastChunkDesc::NoFlags,      0 },
    { { 0.0f, 0.0f, 0.0f }, 3.0f,   UINT32_MAX, NvBlastChunkDesc::NoFlags,      1 },
    { { 0.0f, 0.0f, 0.0f }, 1.0f,   UINT32_MAX, NvBlastChunkDesc::NoFlags,      2 },
    { {-0.5f,-0.5f,-0.5f }, 1.0f,   0,          NvBlastChunkDesc::SupportFlag,  3 },
    { { 0.5f,-0.5f,-0.5f }, 1.0f,   0,          NvBlastChunkDesc::SupportFlag,  4 },
    { {-0.5f, 0.5f,-0.5f }, 1.0f,   0,          NvBlastChunkDesc::SupportFlag,  5 },
    { { 0.5f, 0.5f,-0.5f }, 1.0f,   1,          NvBlastChunkDesc::SupportFlag,  6 },
    { {-0.5f,-0.5f, 0.5f }, 1.0f,   1,          NvBlastChunkDesc::SupportFlag,  7 },
    { { 0.5f,-0.5f, 0.5f }, 1.0f,   1,          NvBlastChunkDesc::SupportFlag,  8 },
    { {-0.5f, 0.5f, 0.5f }, 1.0f,   2,          NvBlastChunkDesc::SupportFlag,  9 },
    { { 0.5f, 0.5f, 0.5f }, 1.0f,   2,          NvBlastChunkDesc::SupportFlag, 10 },
};

const NvBlastBondDesc g_cube3BondDescs[12] =
{
//      normal                area centroid              userData chunks
    { { { 1.0f, 0.0f, 0.0f }, 1.0f,{ 0.0f,-0.5f,-0.5f }, 0 }, { 3, 4 } },
    { { { 1.0f, 0.0f, 0.0f }, 1.0f,{ 0.0f, 0.5f,-0.5f }, 0 }, { 5, 6 } },
    { { { 1.0f, 0.0f, 0.0f }, 1.0f,{ 0.0f,-0.5f, 0.5f }, 0 }, { 7, 8 } },
    { { { 1.0f, 0.0f, 0.0f }, 1.0f,{ 0.0f, 0.5f, 0.5f }, 0 }, { 9, 10} },
                                                                    
    { { { 0.0f, 1.0f, 0.0f }, 1.0f,{-0.5f, 0.0f,-0.5f }, 0 }, { 3, 5 } },
    { { { 0.0f, 1.0f, 0.0f }, 1.0f,{ 0.5f, 0.0f,-0.5f }, 0 }, { 4, 6 } },
    { { { 0.0f, 1.0f, 0.0f }, 1.0f,{-0.5f, 0.0f, 0.5f }, 0 }, { 7, 9 } },
    { { { 0.0f, 1.0f, 0.0f }, 1.0f,{ 0.5f, 0.0f, 0.5f }, 0 }, { 8, 10} },
                                                                    
    { { { 0.0f, 0.0f, 1.0f }, 1.0f,{-0.5f,-0.5f, 0.0f }, 0 }, { 3, 7 } },
    { { { 0.0f, 0.0f, 1.0f }, 1.0f,{ 0.5f,-0.5f, 0.0f }, 0 }, { 4, 8 } },
    { { { 0.0f, 0.0f, 1.0f }, 1.0f,{-0.5f, 0.5f, 0.0f }, 0 }, { 5, 9 } },
    { { { 0.0f, 0.0f, 1.0f }, 1.0f,{ 0.5f, 0.5f, 0.0f }, 0 }, { 6, 10} },
};

const NvBlastBondDesc g_cube3BondDescs_wb[16] =
{
//    normal                  area centroid              userData chunks
    { { { 1.0f, 0.0f, 0.0f }, 1.0f,{ 0.0f,-0.5f,-0.5f }, 0 }, { 3, 4 }         },
    { { { 1.0f, 0.0f, 0.0f }, 1.0f,{ 0.0f, 0.5f,-0.5f }, 0 }, { 5, 6 }         },
    { { { 1.0f, 0.0f, 0.0f }, 1.0f,{ 0.0f,-0.5f, 0.5f }, 0 }, { 7, 8 }         },
    { { { 1.0f, 0.0f, 0.0f }, 1.0f,{ 0.0f, 0.5f, 0.5f }, 0 }, { 9, 10}         },
                                                                           
    { { { 0.0f, 1.0f, 0.0f }, 1.0f,{-0.5f, 0.0f,-0.5f }, 0 }, { 3, 5 }         },
    { { { 0.0f, 1.0f, 0.0f }, 1.0f,{ 0.5f, 0.0f,-0.5f }, 0 }, { 4, 6 }         },
    { { { 0.0f, 1.0f, 0.0f }, 1.0f,{-0.5f, 0.0f, 0.5f }, 0 }, { 7, 9 }         },
    { { { 0.0f, 1.0f, 0.0f }, 1.0f,{ 0.5f, 0.0f, 0.5f }, 0 }, { 8, 10}         },
                                                                           
    { { { 0.0f, 0.0f, 1.0f }, 1.0f,{-0.5f,-0.5f, 0.0f }, 0 }, { 3, 7 }         },
    { { { 0.0f, 0.0f, 1.0f }, 1.0f,{ 0.5f,-0.5f, 0.0f }, 0 }, { 4, 8 }         },
    { { { 0.0f, 0.0f, 1.0f }, 1.0f,{-0.5f, 0.5f, 0.0f }, 0 }, { 5, 9 }         },
    { { { 0.0f, 0.0f, 1.0f }, 1.0f,{ 0.5f, 0.5f, 0.0f }, 0 }, { 6, 10}         },

    { { { 0.0f, 0.0f, 1.0f }, 1.0f,{-0.5f,-0.5f,-1.0f }, 0 }, { 3, UINT32_MAX} },
    { { { 0.0f, 0.0f, 1.0f }, 1.0f,{ 0.5f,-0.5f,-1.0f }, 0 }, { 4, UINT32_MAX} },
    { { { 0.0f, 0.0f, 1.0f }, 1.0f,{-0.5f, 0.5f,-1.0f }, 0 }, { 5, UINT32_MAX} },
    { { { 0.0f, 0.0f, 1.0f }, 1.0f,{ 0.5f, 0.5f,-1.0f }, 0 }, { 6, UINT32_MAX} },
};

const NvBlastAssetDesc g_assetDescs[6] =
{
    // 2x2x2 axis-aligned cube centered at the origin, split into 8 depth-1 1x1x1 child chunks
    { sizeof(g_cube1ChunkDescs) / sizeof(g_cube1ChunkDescs[0]), g_cube1ChunkDescs, sizeof(g_cube1BondDescs) / sizeof(g_cube1BondDescs[0]), g_cube1BondDescs },

    // 2x2x2 axis-aligned cube centered at the origin, split into 8 depth-1 1x1x1 child chunks which are then split into 8 depth-2 (1/2)x(1/2)x(1/2) child chunks each
    // Support is at depth-1, so the g_cube1BondDescs are used
    { sizeof(g_cube2ChunkDescs) / sizeof(g_cube2ChunkDescs[0]), g_cube2ChunkDescs, sizeof(g_cube1BondDescs) / sizeof(g_cube1BondDescs[0]), g_cube1BondDescs },

    // 2x2x2 axis-aligned cube centered at the origin, split into 8 depth-1 1x1x1 child chunks with multiple roots
    { sizeof(g_cube3ChunkDescs) / sizeof(g_cube3ChunkDescs[0]), g_cube3ChunkDescs, sizeof(g_cube3BondDescs) / sizeof(g_cube3BondDescs[0]), g_cube3BondDescs },

    // 2x2x2 axis-aligned cube centered at the origin, split into 8 depth-1 1x1x1 child chunks - contains world-bound chunks
    { sizeof(g_cube1ChunkDescs) / sizeof(g_cube1ChunkDescs[0]), g_cube1ChunkDescs, sizeof(g_cube1BondDescs_wb) / sizeof(g_cube1BondDescs_wb[0]), g_cube1BondDescs_wb },

    // 2x2x2 axis-aligned cube centered at the origin, split into 8 depth-1 1x1x1 child chunks which are then split into 8 depth-2 (1/2)x(1/2)x(1/2) child chunks each - contains world-bound chunks
    // Support is at depth-1, so the g_cube1BondDescs_wb are used
    { sizeof(g_cube2ChunkDescs) / sizeof(g_cube2ChunkDescs[0]), g_cube2ChunkDescs, sizeof(g_cube1BondDescs_wb) / sizeof(g_cube1BondDescs_wb[0]), g_cube1BondDescs_wb },

    // 2x2x2 axis-aligned cube centered at the origin, split into 8 depth-1 1x1x1 child chunks with multiple roots - contains world-bound chunks
    { sizeof(g_cube3ChunkDescs) / sizeof(g_cube3ChunkDescs[0]), g_cube3ChunkDescs, sizeof(g_cube3BondDescs_wb) / sizeof(g_cube3BondDescs_wb[0]), g_cube3BondDescs_wb },
};



struct ExpectedValues
{
    uint32_t    totalChunkCount;
    uint32_t    graphNodeCount;
    uint32_t    leafChunkCount;
    uint32_t    bondCount;
    uint32_t    subsupportChunkCount;
};

const ExpectedAssetValues g_assetExpectedValues[6] =
{
//    total graph   leaves  bonds   sub
    { 9,    8,      8,      12,     0   },
    { 73,   8,      64,     12,     64  },
    { 11,   8,      8,      12,     0   },
    { 9,    9,      8,      16,     0   },
    { 73,   9,      64,     16,     64  },
    { 11,   9,      8,      16,     0   },
};


///////////// Badly-formed asset descs below //////////////

const NvBlastChunkDesc g_cube1ChunkDescsMissingCoverage[9] =
{
//    centroid              volume  parent idx    flags                             ID
    { { 0.0f, 0.0f, 0.0f }, 8.0f,   UINT32_MAX,   NvBlastChunkDesc::NoFlags,        0 },
    { {-0.5f,-0.5f,-0.5f }, 1.0f,   0,            NvBlastChunkDesc::SupportFlag,    1 },
    { { 0.5f,-0.5f,-0.5f }, 1.0f,   0,            NvBlastChunkDesc::SupportFlag,    2 },
    { {-0.5f, 0.5f,-0.5f }, 1.0f,   0,            NvBlastChunkDesc::SupportFlag,    3 },
    { { 0.5f, 0.5f,-0.5f }, 1.0f,   0,            NvBlastChunkDesc::SupportFlag,    4 },
    { {-0.5f,-0.5f, 0.5f }, 1.0f,   0,            NvBlastChunkDesc::SupportFlag,    5 },
    { { 0.5f,-0.5f, 0.5f }, 1.0f,   0,            NvBlastChunkDesc::SupportFlag,    6 },
    { {-0.5f, 0.5f, 0.5f }, 1.0f,   0,            NvBlastChunkDesc::SupportFlag,    7 },
    { { 0.5f, 0.5f, 0.5f }, 1.0f,   0,            NvBlastChunkDesc::NoFlags,        8 },
};


const NvBlastChunkDesc g_cube2ChunkDescsMissingCoverage1[17] =
{
//    centroid                                volume  parent idx    flags                             ID
    { { 0.0f, 0.0f, 0.0f },                   8.0f,   UINT32_MAX,   NvBlastChunkDesc::NoFlags,        0  },
    { {-0.5f,-0.5f,-0.5f },                   1.0f,   0,            NvBlastChunkDesc::NoFlags,        1  },
    { { 0.5f,-0.5f,-0.5f },                   1.0f,   0,            NvBlastChunkDesc::SupportFlag,    2  },
    { {-0.5f, 0.5f,-0.5f },                   1.0f,   0,            NvBlastChunkDesc::SupportFlag,    3  },
    { { 0.5f, 0.5f,-0.5f },                   1.0f,   0,            NvBlastChunkDesc::SupportFlag,    4  },
    { {-0.5f,-0.5f, 0.5f },                   1.0f,   0,            NvBlastChunkDesc::SupportFlag,    5  },
    { { 0.5f,-0.5f, 0.5f },                   1.0f,   0,            NvBlastChunkDesc::SupportFlag,    6  },
    { {-0.5f, 0.5f, 0.5f },                   1.0f,   0,            NvBlastChunkDesc::SupportFlag,    7  },
    { { 0.5f, 0.5f, 0.5f },                   1.0f,   0,            NvBlastChunkDesc::NoFlags,        8  },
    { {-0.25f-0.5f,-0.25f-0.5f,-0.25f-0.5f }, 0.125f, 1,            NvBlastChunkDesc::NoFlags,        9  },
    { { 0.25f-0.5f,-0.25f-0.5f,-0.25f-0.5f }, 0.125f, 1,            NvBlastChunkDesc::NoFlags,        10 },
    { {-0.25f-0.5f, 0.25f-0.5f,-0.25f-0.5f }, 0.125f, 1,            NvBlastChunkDesc::NoFlags,        11 },
    { { 0.25f-0.5f, 0.25f-0.5f,-0.25f-0.5f }, 0.125f, 1,            NvBlastChunkDesc::NoFlags,        12 },
    { {-0.25f-0.5f,-0.25f-0.5f, 0.25f-0.5f }, 0.125f, 1,            NvBlastChunkDesc::NoFlags,        13 },
    { { 0.25f-0.5f,-0.25f-0.5f, 0.25f-0.5f }, 0.125f, 1,            NvBlastChunkDesc::NoFlags,        14 },
    { {-0.25f-0.5f, 0.25f-0.5f, 0.25f-0.5f }, 0.125f, 1,            NvBlastChunkDesc::NoFlags,        15 },
    { { 0.25f-0.5f, 0.25f-0.5f, 0.25f-0.5f }, 0.125f, 1,            NvBlastChunkDesc::NoFlags,        16 },
};


const NvBlastChunkDesc g_cube2ChunkDescsMissingCoverage2[17] =
{
//    centroid                                volume  parent idx    flags                             ID
    { { 0.0f, 0.0f, 0.0f },                   8.0f,   UINT32_MAX,   NvBlastChunkDesc::NoFlags,        0  },
    { {-0.5f,-0.5f,-0.5f },                   1.0f,   0,            NvBlastChunkDesc::NoFlags,        1  },
    { { 0.5f,-0.5f,-0.5f },                   1.0f,   0,            NvBlastChunkDesc::SupportFlag,    2  },
    { {-0.5f, 0.5f,-0.5f },                   1.0f,   0,            NvBlastChunkDesc::SupportFlag,    3  },
    { { 0.5f, 0.5f,-0.5f },                   1.0f,   0,            NvBlastChunkDesc::SupportFlag,    4  },
    { {-0.5f,-0.5f, 0.5f },                   1.0f,   0,            NvBlastChunkDesc::SupportFlag,    5  },
    { { 0.5f,-0.5f, 0.5f },                   1.0f,   0,            NvBlastChunkDesc::SupportFlag,    6  },
    { {-0.5f, 0.5f, 0.5f },                   1.0f,   0,            NvBlastChunkDesc::SupportFlag,    7  },
    { { 0.5f, 0.5f, 0.5f },                   1.0f,   0,            NvBlastChunkDesc::NoFlags,        8  },
    { {-0.25f-0.5f,-0.25f-0.5f,-0.25f-0.5f }, 0.125f, 1,            NvBlastChunkDesc::NoFlags,        9  },
    { { 0.25f-0.5f,-0.25f-0.5f,-0.25f-0.5f }, 0.125f, 1,            NvBlastChunkDesc::NoFlags,        10 },
    { {-0.25f-0.5f, 0.25f-0.5f,-0.25f-0.5f }, 0.125f, 1,            NvBlastChunkDesc::NoFlags,        11 },
    { { 0.25f-0.5f, 0.25f-0.5f,-0.25f-0.5f }, 0.125f, 1,            NvBlastChunkDesc::NoFlags,        12 },
    { {-0.25f-0.5f,-0.25f-0.5f, 0.25f-0.5f }, 0.125f, 1,            NvBlastChunkDesc::NoFlags,        13 },
    { { 0.25f-0.5f,-0.25f-0.5f, 0.25f-0.5f }, 0.125f, 1,            NvBlastChunkDesc::NoFlags,        14 },
    { {-0.25f-0.5f, 0.25f-0.5f, 0.25f-0.5f }, 0.125f, 1,            NvBlastChunkDesc::NoFlags,        15 },
    { { 0.25f-0.5f, 0.25f-0.5f, 0.25f-0.5f }, 0.125f, 1,            NvBlastChunkDesc::SupportFlag,    16 },
};


const NvBlastAssetDesc g_assetDescsMissingCoverage[6] =
{
    { sizeof(g_cube1ChunkDescsMissingCoverage) / sizeof(g_cube1ChunkDescsMissingCoverage[0]), g_cube1ChunkDescsMissingCoverage, sizeof(g_cube1BondDescs) / sizeof(g_cube1BondDescs[0]), g_cube1BondDescs },
    { sizeof(g_cube2ChunkDescsMissingCoverage1) / sizeof(g_cube2ChunkDescsMissingCoverage1[0]), g_cube2ChunkDescsMissingCoverage1, sizeof(g_cube1BondDescs) / sizeof(g_cube1BondDescs[0]), g_cube1BondDescs },
    { sizeof(g_cube2ChunkDescsMissingCoverage2) / sizeof(g_cube2ChunkDescsMissingCoverage2[0]), g_cube2ChunkDescsMissingCoverage2, sizeof(g_cube1BondDescs) / sizeof(g_cube1BondDescs[0]), g_cube1BondDescs },
    { sizeof(g_cube1ChunkDescsMissingCoverage) / sizeof(g_cube1ChunkDescsMissingCoverage[0]), g_cube1ChunkDescsMissingCoverage, sizeof(g_cube1BondDescs_wb) / sizeof(g_cube1BondDescs_wb[0]), g_cube1BondDescs_wb },
    { sizeof(g_cube2ChunkDescsMissingCoverage1) / sizeof(g_cube2ChunkDescsMissingCoverage1[0]), g_cube2ChunkDescsMissingCoverage1, sizeof(g_cube1BondDescs_wb) / sizeof(g_cube1BondDescs_wb[0]), g_cube1BondDescs_wb },
    { sizeof(g_cube2ChunkDescsMissingCoverage2) / sizeof(g_cube2ChunkDescsMissingCoverage2[0]), g_cube2ChunkDescsMissingCoverage2, sizeof(g_cube1BondDescs_wb) / sizeof(g_cube1BondDescs_wb[0]), g_cube1BondDescs_wb },
};

extern const ExpectedAssetValues g_assetsFromMissingCoverageExpectedValues[6] =
{
//    total graph   leaves  bonds   sub
    { 9,    8,      8,      12,     0   },
    { 17,   8,      15,     12,     8   },
    { 17,   15,     15,     9,      0   },
    { 9,    9,      8,      16,     0   },
    { 17,   9,      15,     16,     8   },
    { 17,   16,     15,     12,     0   },
};


void generateCube(GeneratorAsset& cubeAsset, NvBlastAssetDesc& assetDesc, size_t maxDepth, size_t width, int32_t supportDepth, CubeAssetGenerator::BondFlags bondFlags)
{
    CubeAssetGenerator::Settings settings;
    settings.extents = GeneratorAsset::Vec3(1, 1, 1);
    CubeAssetGenerator::DepthInfo depthInfo;
    depthInfo.slicesPerAxis = GeneratorAsset::Vec3(1, 1, 1);
    depthInfo.flag = NvBlastChunkDesc::Flags::NoFlags;
    settings.depths.push_back(depthInfo);
    settings.bondFlags = bondFlags;

    for (uint32_t depth = 1; depth < maxDepth; ++depth)
    {
        depthInfo.slicesPerAxis = GeneratorAsset::Vec3((float)width, (float)width, (float)width);
        settings.depths.push_back(depthInfo);
    }
    settings.depths[(supportDepth > 0 ? supportDepth : maxDepth) - 1].flag = NvBlastChunkDesc::SupportFlag; // Leaves are support

    CubeAssetGenerator::generate(cubeAsset, settings);

    assetDesc.bondCount = (uint32_t)cubeAsset.solverBonds.size();
    assetDesc.bondDescs = cubeAsset.solverBonds.data();
    assetDesc.chunkCount = (uint32_t)cubeAsset.chunks.size();
    assetDesc.chunkDescs = cubeAsset.solverChunks.data();
}

void generateRandomCube(GeneratorAsset& cubeAsset, NvBlastAssetDesc& desc, uint32_t minChunkCount, uint32_t maxChunkCount)
{
    CubeAssetGenerator::Settings settings;
    settings.extents = GeneratorAsset::Vec3(1, 1, 1);
    CubeAssetGenerator::DepthInfo depthInfo;
    depthInfo.slicesPerAxis = GeneratorAsset::Vec3(1, 1, 1);
    depthInfo.flag = NvBlastChunkDesc::Flags::NoFlags;
    settings.depths.push_back(depthInfo);
    uint32_t chunkCount = 1;
    while (chunkCount < minChunkCount)
    {
        uint32_t chunkMul;
        do
        {
            depthInfo.slicesPerAxis = GeneratorAsset::Vec3((float)(1 + rand() % 4), (float)(1 + rand() % 4), (float)(1 + rand() % 4));
            chunkMul = (uint32_t)(depthInfo.slicesPerAxis.x * depthInfo.slicesPerAxis.y * depthInfo.slicesPerAxis.z);
        } while (chunkMul == 1);
        if (chunkCount*chunkMul > maxChunkCount)
        {
            break;
        }
        chunkCount *= chunkMul;
        settings.depths.push_back(depthInfo);
        settings.extents = settings.extents * depthInfo.slicesPerAxis;
    }
    settings.depths.back().flag = NvBlastChunkDesc::SupportFlag;    // Leaves are support

                                                                    // Make largest direction unit size
    settings.extents = settings.extents * (1.0f / std::max(settings.extents.x, std::max(settings.extents.y, settings.extents.z)));

    // Create asset
    CubeAssetGenerator::generate(cubeAsset, settings);

    desc.chunkDescs = cubeAsset.solverChunks.data();
    desc.chunkCount = (uint32_t)cubeAsset.solverChunks.size();
    desc.bondDescs = cubeAsset.solverBonds.data();
    desc.bondCount = (uint32_t)cubeAsset.solverBonds.size();
}