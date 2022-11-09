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
// Copyright (c) 2014-2022 NVIDIA Corporation. All rights reserved.

#include "ImguiParams.h"

ConstantBuffer<ImguiRendererParams> paramsIn;

StructuredBuffer<int4> treeIn;
StructuredBuffer<uint> tileCountIn;
StructuredBuffer<ImguiRendererDrawCmd> drawCmdsIn;

RWStructuredBuffer<uint> triangleOut;
RWStructuredBuffer<uint2> triangleRangeOut;

#include "ImguiBlockScan.hlsli"

bool overlapTest(int4 minMaxA, int4 minMaxB)
{
	bool ret;
	if (minMaxB.x == minMaxB.z &&
		minMaxB.y == minMaxB.w)
	{
		ret = false;
	}
	else
	{
		ret = !(
			minMaxA.x > minMaxB.z || minMaxB.x > minMaxA.z ||
			minMaxA.y > minMaxB.w || minMaxB.y > minMaxA.w
			);
	}
	return ret;
}

void writeTriangle(inout uint hitIdx, uint triangleWriteOffset, uint triangleIdx)
{
	uint index = 3u * triangleIdx;
	uint drawCmdIdx = paramsIn.numDrawCmds - 1u;
	for (; drawCmdIdx < paramsIn.numDrawCmds; drawCmdIdx--)
	{
		if (index >= drawCmdsIn[drawCmdIdx].indexOffset)
		{
			break;
		}
	}

	triangleOut[hitIdx + triangleWriteOffset] = triangleIdx | (drawCmdIdx << 24u);
	hitIdx++;
}

void countHits(inout uint hitIdx, int4 idxMinMax, uint blockIdx, uint triangleWriteOffset)
{
	uint treeBaseOffset = (1u + 4u + 16u + 64u + 256u) * blockIdx;

	int4 minMaxPos = treeIn[treeBaseOffset];

	if (overlapTest(idxMinMax, minMaxPos))
	{
		uint treeletOffset0 = treeBaseOffset + (1u);
		uint treeletOffset1 = treeBaseOffset + (1u + 4u);
		uint treeletOffset2 = treeBaseOffset + (1u + 4u + 16u);
		uint treeletOffset3 = treeBaseOffset + (1u + 4u + 16u + 64u);
		for (uint childIdx0 = 0u; childIdx0 < 4u; childIdx0++)
		{
			uint idx0 = childIdx0;
			int4 minMaxPos = treeIn[idx0 + treeletOffset0];
			if (overlapTest(idxMinMax, minMaxPos))
			{
				for (uint childIdx1 = 0u; childIdx1 < 4u; childIdx1++)
				{
					uint idx1 = 4u * idx0 + childIdx1;
					int4 minMaxPos = treeIn[idx1 + treeletOffset1];
					if (overlapTest(idxMinMax, minMaxPos))
					{
						for (uint childIdx2 = 0u; childIdx2 < 4u; childIdx2++)
						{
							uint idx2 = 4u * idx1 + childIdx2;
							int4 minMaxPos = treeIn[idx2 + treeletOffset2];
							if (overlapTest(idxMinMax, minMaxPos))
							{
								for (uint childIdx3 = 0u; childIdx3 < 4u; childIdx3++)
								{
									uint idx3 = 4u * idx2 + childIdx3;
									int4 minMaxPos = treeIn[idx3 + treeletOffset3];
									if (overlapTest(idxMinMax, minMaxPos))
									{
										uint triangleIdx = (blockIdx << 8u) + idx3;

										writeTriangle(hitIdx, triangleWriteOffset, triangleIdx);
									}
								}
							}
						}
					}
				}
			}
		}
	}
}

[numthreads(256, 1, 1)]
void main(uint3 dispatchThreadID : SV_DispatchThreadID)
{
	int tidx = int(dispatchThreadID.x);
	uint threadIdx = uint(tidx) & 255u;

	int2 tileIdx = int2(
		tidx % paramsIn.tileGridDim_x,
		tidx / paramsIn.tileGridDim_x
	);

	if (tileIdx.y < int(paramsIn.tileGridDim_y))
	{
		uint hitIdx = 0u;

		// add local and global offsets together
		uint globalOffset = tileCountIn[(tidx >> 8u) + paramsIn.tileGlobalScanOffset];
		uint localOffset = tileCountIn[tidx + paramsIn.tileLocalScanOffset];
		uint triangleWriteOffset = globalOffset + localOffset;

		uint tileDim = 1u << paramsIn.tileDimBits;

		int4 idxMinMax = int4(
			tileIdx * tileDim - int2(1, 1),
			tileIdx * tileDim + tileDim - int2(1, 1)
		);

		for (uint blockIdx = 0u; blockIdx < paramsIn.numBlocks; blockIdx++)
		{
			countHits(hitIdx, idxMinMax, blockIdx, triangleWriteOffset);
		}

		// write out range
		triangleRangeOut[tidx] = uint2(triangleWriteOffset, hitIdx);
	}
}