
#include "NvFlowShaderTypes.h"

struct ImguiRendererParams
{
	NvFlowUint numVertices;
	NvFlowUint numIndices;
	NvFlowUint numDrawCmds;
	NvFlowUint numBlocks;

	float width;
	float height;
	float widthInv;
	float heightInv;

	NvFlowUint tileGridDim_x;
	NvFlowUint tileGridDim_y;
	NvFlowUint tileGridDim_xy;
	NvFlowUint tileDimBits;

	NvFlowUint maxTriangles;
	NvFlowUint tileNumTrianglesOffset;
	NvFlowUint tileLocalScanOffset;
	NvFlowUint tileLocalTotalOffset;

	NvFlowUint tileGlobalScanOffset;
	NvFlowUint numTileBuckets;
	NvFlowUint numTileBucketPasses;
	NvFlowUint pad3;
};

struct ImguiRendererDrawCmd
{
	NvFlowFloat4 clipRect;
	NvFlowUint elemCount;
	NvFlowUint userTexture;
	NvFlowUint vertexOffset;
	NvFlowUint indexOffset;
};