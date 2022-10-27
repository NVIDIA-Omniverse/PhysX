
#include "ShapeParams.h"

ConstantBuffer<ShapeRendererParams> paramsIn;

StructuredBuffer<float4> spherePositionRadiusIn;

RWTexture2D<float> depthOut;
RWTexture2D<float4> colorOut;

bool raySphereIntersection(float3 o, float3 d, float3 c, float r, inout float t)
{
	bool ret = false;
	float3 l = c - o;
	float s = dot(l, d);
	float l2 = dot(l, l);
	float r2 = r * r;
	if (s >= 0.f || l2 <= r2)
	{
		float s2 = s * s;
		float m2 = l2 - s2;
		if (m2 <= r2)
		{
			float q = sqrt(r2 - m2);
			if (l2 > r2)
			{
				t = s - q;
			}
			else
			{
				t = s + q;
			}
			ret = t > 0.f;
		}
	}
	return ret;
}

[numthreads(8, 8, 1)]
void main(uint3 dispatchThreadID : SV_DispatchThreadID)
{
	int2 tidx = int2(dispatchThreadID.xy);

	float2 pixelCenter = float2(tidx) + 0.5f;
	float2 inUV = pixelCenter * float2(paramsIn.widthInv, paramsIn.heightInv);

	float w00 = (1.f - inUV.x) * (1.f - inUV.y);
	float w10 = (inUV.x) * (1.f - inUV.y);
	float w01 = (1.f - inUV.x) * (inUV.y);
	float w11 = (inUV.x) * (inUV.y);

	float3 rayDir = normalize(w00 * paramsIn.rayDir00.xyz + w10 * paramsIn.rayDir10.xyz + w01 * paramsIn.rayDir01.xyz + w11 * paramsIn.rayDir11.xyz);
	float3 rayOrigin = w00 * paramsIn.rayOrigin00.xyz + w10 * paramsIn.rayOrigin10.xyz + w01 * paramsIn.rayOrigin01.xyz + w11 * paramsIn.rayOrigin11.xyz;

	float globalDepth = paramsIn.clearDepth;
	float4 globalColor = paramsIn.clearColor;

	for (uint sphereIdx = 0u; sphereIdx < paramsIn.numSpheres; sphereIdx++)
	{
		float4 spherePositionRadius = spherePositionRadiusIn[sphereIdx];
		float hitT = -1.f;
		if (raySphereIntersection(rayOrigin, rayDir, spherePositionRadius.xyz, spherePositionRadius.w, hitT))
		{
			float4 worldPos = float4(rayDir * hitT + rayOrigin, 1.f);

			float4 clipPos = mul(worldPos, paramsIn.view);
			clipPos = mul(clipPos, paramsIn.projection);
			
			float depth = clipPos.z / clipPos.w;

			bool isVisible = bool(paramsIn.isReverseZ) ? (depth > globalDepth) : (depth < globalDepth);
			if (isVisible)
			{
				float3 n = normalize(worldPos.xyz - spherePositionRadius.xyz);

				float4 color = float4(abs(n), 1.f);

				globalDepth = depth;
				globalColor = color;
			}
		}
	}

	depthOut[tidx] = globalDepth;
	colorOut[tidx] = globalColor;
}
