
#include "ImguiParams.h"

ConstantBuffer<ImguiRendererParams> paramsIn;

StructuredBuffer<float4> vertexPosTexCoordIn;
StructuredBuffer<uint> vertexColorIn;
StructuredBuffer<uint> indicesIn;
StructuredBuffer<ImguiRendererDrawCmd> drawCmdsIn;

Texture2D<float4> textureIn;
SamplerState samplerIn;

StructuredBuffer<uint> triangleIn;
StructuredBuffer<uint2> triangleRangeIn;

Texture2D<float4> colorIn;

RWTexture2D<float4> colorOut;

float4 convertColor(uint val)
{
	return float4(
		((val >>  0) & 255) * (1.f / 255.f),
		((val >>  8) & 255) * (1.f / 255.f),
		((val >> 16) & 255) * (1.f / 255.f),
		((val >> 24) & 255) * (1.f / 255.f)
	);
}

float3 computeBary(float2 p1, float2 p2, float2 p3, float2 p)
{
	float det = (p2.y - p3.y) * (p1.x - p3.x) + (p3.x - p2.x) * (p1.y - p3.y);
	float b1 = (p2.y - p3.y) * (p.x - p3.x) + (p3.x - p2.x) * (p.y - p3.y);
	float b2 = (p3.y - p1.y) * (p.x - p3.x) + (p1.x - p3.x) * (p.y - p3.y);
	b1 = b1 / det;
	b2 = b2 / det;
	float b3 = 1.f - b1 - b2;
	return float3(b1, b2, b3);
}

bool edgeTest(float2 edgeA_fp, float2 edgeB_fp, float2 inside_fp, float2 pt_fp)
{
	int2 edgeA = int2(edgeA_fp);
	int2 edgeB = int2(edgeB_fp);
	int2 inside = int2(inside_fp);
	int2 pt = int2(pt_fp);

	int2 m = edgeB - edgeA;
	bool isBelow;
	if (abs(m.y) > abs(m.x))
	{
		int insX_num = (edgeB.y - inside.y) * edgeA.x - (edgeA.y - inside.y) * edgeB.x;
		int insX = int(float(insX_num) / float(m.y));
		bool insideIsBelow = inside.x < insX;

		int cmpX_num = (edgeB.y - pt.y) * edgeA.x - (edgeA.y - pt.y) * edgeB.x;
		int cmpX = int(float(cmpX_num) / float(m.y));
		isBelow = insideIsBelow ? pt.x < cmpX : pt.x >= cmpX;
	}
	else
	{
		int insY_num = (edgeB.x - inside.x) * edgeA.y - (edgeA.x - inside.x) * edgeB.y;
		int insY = int(float(insY_num) / float(m.x));
		bool insideIsBelow = inside.y < insY;

		int cmpY_num = (edgeB.x - pt.x) * edgeA.y - (edgeA.x - pt.x) * edgeB.y;
		int cmpY = int(float(cmpY_num) / float(m.x));
		isBelow = insideIsBelow ? pt.y < cmpY : pt.y >= cmpY;
	}
	return isBelow;
}

float4 leaf(float2 p, uint triangleIdx)
{
	uint indexOffsetLocal = 3u * triangleIdx;

	uint idx0 = indicesIn[indexOffsetLocal + 0];
	uint idx1 = indicesIn[indexOffsetLocal + 1];
	uint idx2 = indicesIn[indexOffsetLocal + 2];

	float4 pos0 = vertexPosTexCoordIn[idx0];
	float4 pos1 = vertexPosTexCoordIn[idx1];
	float4 pos2 = vertexPosTexCoordIn[idx2];

	float2 p1 = 16.f * pos0.xy;
	float2 p2 = 16.f * pos1.xy;
	float2 p3 = 16.f * pos2.xy;

	bool passed = edgeTest(p1, p2, p3, p);
	passed = passed && edgeTest(p2, p3, p1, p); 
	passed = passed && edgeTest(p3, p1, p2, p);

	float4 c = float4(0.f, 0.f, 0.f, 0.f);
	if (passed)
	{
		float3 b = computeBary(p1, p2, p3, p);

		c = b.x * convertColor(vertexColorIn[idx0]) +
			b.y * convertColor(vertexColorIn[idx1]) +
			b.z * convertColor(vertexColorIn[idx2]);

		float2 uv = b.x * pos0.zw + b.y * pos1.zw + b.z * pos2.zw;

		c *= textureIn.SampleLevel(samplerIn, uv, 0.0);
	}
	return c;
}

[numthreads(8, 8, 1)]
void main(uint3 dispatchThreadID : SV_DispatchThreadID)
{
	int2 tidx = int2(dispatchThreadID.xy);

	float4 color = colorIn[tidx];

	int2 tileIdx = int2(
		tidx.x >> paramsIn.tileDimBits,
		tidx.y >> paramsIn.tileDimBits
	);
	int tileIdx1D = int(tileIdx.y * paramsIn.tileGridDim_x + tileIdx.x);

	if (tileIdx.y >= int(paramsIn.tileGridDim_y))
	{
		color.g = 1.f;
	}

	uint2 triangleListRange = triangleRangeIn[tileIdx1D];

	float2 tidxf = float2(tidx) + float2(0.5f, 0.5f);
	float2 p = 16.f * tidxf;

	for (uint listIdx = 0u; listIdx < triangleListRange.y; listIdx++)
	{
		uint triangleIdxRaw = triangleIn[triangleListRange.x + listIdx];
		uint triangleIdx = triangleIdxRaw & 0x00FFFFFF;
		uint drawCmdIdx = triangleIdxRaw >> 24u;

		float4 c = leaf(p, triangleIdx);

		//color = (1.f - c.w) * color + c.w * c;

		float4 clipRect = drawCmdsIn[drawCmdIdx].clipRect;
		// (x1, y1, x2, y2)
		if (tidx.x >= clipRect.x && tidx.y >= clipRect.y &&
			tidx.x < clipRect.z && tidx.y < clipRect.w)
		{
			color = (1.f - c.w) * color + c.w * c;
		}
	}

	colorOut[tidx] = color;
}
