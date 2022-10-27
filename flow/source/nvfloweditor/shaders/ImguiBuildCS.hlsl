
#include "ImguiParams.h"

ConstantBuffer<ImguiRendererParams> paramsIn;

StructuredBuffer<float4> vertexPosTexCoordIn;
StructuredBuffer<uint> vertexColorIn;
StructuredBuffer<uint> indicesIn;
StructuredBuffer<ImguiRendererDrawCmd> drawCmdsIn;

RWStructuredBuffer<int4> treeOut;

groupshared int4 sdata0[256];
groupshared int4 sdata1[64];

int4 accumMinMax(int4 a, int4 b)
{
	if (b.x == b.z && b.y == b.w)
	{
		return a;
	}
	else
	{
		return int4(
			min(a.xy, b.xy),
			max(a.zw, b.zw)
		);
	}
}

[numthreads(256, 1, 1)]
void main(uint3 dispatchThreadID : SV_DispatchThreadID)
{
	int tidx = int(dispatchThreadID.x);
	uint threadIdx = uint(tidx) & 255u;

	int4 minMaxPos = int4(0, 0, 0, 0);
	if ((3 * tidx) < int(paramsIn.numIndices))
	{
		uint indexOffset = 3u * uint(tidx);

		// TODO: lookup vertexOffset in drawCmd
		uint vertexOffset = 0u;

		uint idx0 = indicesIn[indexOffset + 0] + vertexOffset;
		uint idx1 = indicesIn[indexOffset + 1] + vertexOffset;
		uint idx2 = indicesIn[indexOffset + 2] + vertexOffset;

		float2 pos0 = vertexPosTexCoordIn[idx0].xy;
		float2 pos1 = vertexPosTexCoordIn[idx1].xy;
		float2 pos2 = vertexPosTexCoordIn[idx2].xy;

		float2 minPos = min(pos0, min(pos1, pos2));
		float2 maxPos = max(pos0, max(pos1, pos2));

		minPos = floor(minPos);
		maxPos = -floor(-maxPos) + float2(1.f, 1.f);

		minMaxPos = int4(minPos, maxPos);
	}

	uint treeBaseIdx = (1u + 4u + 16u + 64u + 256u) * (tidx >> 8u);

	sdata0[threadIdx] = minMaxPos;
	treeOut[treeBaseIdx + threadIdx + (1u + 4u + 16u + 64u)] = minMaxPos;

	GroupMemoryBarrierWithGroupSync();

	if (threadIdx < 64u)
	{
		minMaxPos = sdata0[4u * threadIdx + 0u];
		minMaxPos = accumMinMax(minMaxPos, sdata0[4u * threadIdx + 1u]);
		minMaxPos = accumMinMax(minMaxPos, sdata0[4u * threadIdx + 2u]);
		minMaxPos = accumMinMax(minMaxPos, sdata0[4u * threadIdx + 3u]);

		sdata1[threadIdx] = minMaxPos;
		treeOut[treeBaseIdx + threadIdx + (1u + 4u + 16u)] = minMaxPos;
	}

	GroupMemoryBarrierWithGroupSync();

	if (threadIdx < 16u)
	{
		minMaxPos = sdata1[4u * threadIdx + 0u];
		minMaxPos = accumMinMax(minMaxPos, sdata1[4u * threadIdx + 1u]);
		minMaxPos = accumMinMax(minMaxPos, sdata1[4u * threadIdx + 2u]);
		minMaxPos = accumMinMax(minMaxPos, sdata1[4u * threadIdx + 3u]);

		sdata0[threadIdx] = minMaxPos;
		treeOut[treeBaseIdx + threadIdx + (1u + 4u)] = minMaxPos;
	}

	GroupMemoryBarrierWithGroupSync();

	if (threadIdx < 4u)
	{
		minMaxPos = sdata0[4u * threadIdx + 0u];
		minMaxPos = accumMinMax(minMaxPos, sdata0[4u * threadIdx + 1u]);
		minMaxPos = accumMinMax(minMaxPos, sdata0[4u * threadIdx + 2u]);
		minMaxPos = accumMinMax(minMaxPos, sdata0[4u * threadIdx + 3u]);

		sdata1[threadIdx] = minMaxPos;
		treeOut[treeBaseIdx + threadIdx + (1u)] = minMaxPos;
	}

	GroupMemoryBarrierWithGroupSync();

	if (threadIdx < 1u)
	{
		minMaxPos = sdata1[4u * threadIdx + 0u];
		minMaxPos = accumMinMax(minMaxPos, sdata1[4u * threadIdx + 1u]);
		minMaxPos = accumMinMax(minMaxPos, sdata1[4u * threadIdx + 2u]);
		minMaxPos = accumMinMax(minMaxPos, sdata1[4u * threadIdx + 3u]);

		//sdata0[threadIdx] = minMaxPos;
		treeOut[treeBaseIdx + threadIdx + (0u)] = minMaxPos;
	}
}