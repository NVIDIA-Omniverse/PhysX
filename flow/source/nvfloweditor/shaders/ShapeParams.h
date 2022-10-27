
#include "NvFlowShaderTypes.h"

struct ShapeRendererParams
{
	NvFlowFloat4x4 projection;
	NvFlowFloat4x4 view;
	NvFlowFloat4x4 projectionInv;
	NvFlowFloat4x4 viewInv;

	NvFlowFloat4 rayDir00;
	NvFlowFloat4 rayDir10;
	NvFlowFloat4 rayDir01;
	NvFlowFloat4 rayDir11;

	NvFlowFloat4 rayOrigin00;
	NvFlowFloat4 rayOrigin10;
	NvFlowFloat4 rayOrigin01;
	NvFlowFloat4 rayOrigin11;

	float width;
	float height;
	float widthInv;
	float heightInv;

	NvFlowUint numSpheres;
	float clearDepth;
	NvFlowUint isReverseZ;
	NvFlowUint pad3;

	NvFlowFloat4 clearColor;
};