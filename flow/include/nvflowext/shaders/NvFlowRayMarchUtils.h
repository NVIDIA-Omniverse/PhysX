/*
 * Copyright (c) 2014-2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef NV_FLOW_RAY_MARCH_UTILS_H
#define NV_FLOW_RAY_MARCH_UTILS_H

#if defined(__cplusplus)

#include "NvFlowExt.h"
#include "NvFlowRayMarchParams.h"
#include "NvFlowMath.h"

NV_FLOW_INLINE void NvFlowRayMarchLayerShaderParams_populate(
	NvFlowRayMarchLayerShaderParams* dst, 
	NvFlowUint velocityLevelIdx, 
	NvFlowUint densityLevelIdx, 
	NvFlowUint layerParamIdx, 
	const NvFlowSparseParams* sparseParams, 
	const NvFlowRayMarchParams* rayMarchParams
)
{
	const NvFlowSparseLevelParams* levelParamsVelocity = &sparseParams->levels[velocityLevelIdx];
	const NvFlowSparseLevelParams* levelParamsDensity = &sparseParams->levels[densityLevelIdx];
	const NvFlowSparseLayerParams* layerParams = &sparseParams->layers[layerParamIdx];

	NvFlowFloat3 blockSizeWorld = layerParams->blockSizeWorld;
	NvFlowFloat3 blockSizeWorldInv = layerParams->blockSizeWorldInv;
	float minBlockSizeWorld = fminf(blockSizeWorld.x, fminf(blockSizeWorld.y, blockSizeWorld.z));
	float maxBlockSizeWorld = fmaxf(blockSizeWorld.x, fmaxf(blockSizeWorld.y, blockSizeWorld.z));

	NvFlowFloat3 cellSize = NvFlowFloat3{
		blockSizeWorld.x / float(levelParamsDensity->blockDimLessOne.x + 1u),
		blockSizeWorld.y / float(levelParamsDensity->blockDimLessOne.y + 1u),
		blockSizeWorld.z / float(levelParamsDensity->blockDimLessOne.z + 1u)
	};
	NvFlowFloat3 cellSizeInv = { 1.f / cellSize.x, 1.f / cellSize.y, 1.f / cellSize.z };
	float cellSizeMin = fminf(fminf(cellSize.x, cellSize.y), cellSize.z);

	NvFlowFloat3 velocityCellSizeInv = NvFlowFloat3{
		float(levelParamsVelocity->blockDimLessOne.x + 1u) / blockSizeWorld.x,
		float(levelParamsVelocity->blockDimLessOne.y + 1u) / blockSizeWorld.y,
		float(levelParamsVelocity->blockDimLessOne.z + 1u) / blockSizeWorld.z
	};

	float stepSize = rayMarchParams->stepSizeScale * cellSizeMin;
	float stepSizeInv = 1.f / stepSize;

	// normalize alphaScale based on stepSize
	float alphaScale = 1.f - expf(-rayMarchParams->attenuation * stepSize);
	float layerColormapV = (float(layerParamIdx) + 0.5f) / float(sparseParams->layerCount);

	dst->blockSizeWorld = blockSizeWorld;
	dst->minBlockSizeWorld = minBlockSizeWorld;
	dst->blockSizeWorldInv = blockSizeWorldInv;
	dst->maxBlockSizeWorld = maxBlockSizeWorld;
	dst->cellSize = cellSize;
	dst->stepSize = stepSize;
	dst->cellSizeInv = cellSizeInv;
	dst->stepSizeInv = stepSizeInv;
	dst->locationMin = layerParams->locationMin;
	dst->locationMax = layerParams->locationMax;
	dst->worldMin.x = (layerParams->locationMin.x - 0.5f) * layerParams->blockSizeWorld.x;
	dst->worldMin.y = (layerParams->locationMin.y - 0.5f) * layerParams->blockSizeWorld.y;
	dst->worldMin.z = (layerParams->locationMin.z - 0.5f) * layerParams->blockSizeWorld.z;
	dst->enableBlockWireframe = rayMarchParams->enableBlockWireframe;
	dst->worldMax.x = (layerParams->locationMax.x + 0.5f) * layerParams->blockSizeWorld.x;
	dst->worldMax.y = (layerParams->locationMax.y + 0.5f) * layerParams->blockSizeWorld.y;
	dst->worldMax.z = (layerParams->locationMax.z + 0.5f) * layerParams->blockSizeWorld.z;
	dst->enableRawMode = rayMarchParams->enableRawMode;

	dst->velocityCellSizeInv = velocityCellSizeInv;
	dst->deltaTime = layerParams->deltaTime;

	dst->layer = layerParams->layer;
	dst->layerColormapV = layerColormapV;
	dst->alphaScale = alphaScale;
	dst->colorScale = rayMarchParams->colorScale;

	dst->shadowFactor = rayMarchParams->shadowFactor;
	dst->pad1 = 0.f;
	dst->pad2 = 0.f;
	dst->pad3 = 0.f;

	dst->cloud.densityMultiplier = rayMarchParams->cloud.densityMultiplier;
	dst->cloud.enableCloudMode = rayMarchParams->cloud.enableCloudMode;
	dst->cloud.pad1 = 0.f;
	dst->cloud.pad2 = 0.f;
	dst->cloud.ambientColor = rayMarchParams->cloud.ambientColor;
	dst->cloud.ambientMultiplier = rayMarchParams->cloud.ambientMultiplier;
	dst->cloud.volumeBaseColor = rayMarchParams->cloud.volumeBaseColor;
	dst->cloud.volumeColorMultiplier = rayMarchParams->cloud.volumeColorMultiplier;
	dst->cloud.sunDirection = rayMarchParams->cloud.sunDirection;
	dst->cloud.shadowStepMultiplier = rayMarchParams->cloud.shadowStepMultiplier;
	dst->cloud.attenuationMultiplier = rayMarchParams->cloud.attenuationMultiplier;
	dst->cloud.numShadowSteps = rayMarchParams->cloud.numShadowSteps;
}

NV_FLOW_INLINE void NvFlowRayMarchIsosurfaceLayerShaderParams_populate(
	NvFlowRayMarchIsosurfaceLayerShaderParams* dst,
	NvFlowUint densityLevelIdx,
	NvFlowUint layerParamIdx,
	const NvFlowSparseParams* sparseParams,
	const NvFlowRayMarchIsosurfaceParams* rayMarchParams
)
{
	const NvFlowSparseLevelParams* levelParams = &sparseParams->levels[densityLevelIdx];
	const NvFlowSparseLayerParams* layerParams = &sparseParams->layers[layerParamIdx];

	NvFlowFloat3 blockSizeWorld = layerParams->blockSizeWorld;
	NvFlowFloat3 blockSizeWorldInv = layerParams->blockSizeWorldInv;
	float minBlockSizeWorld = fminf(blockSizeWorld.x, fminf(blockSizeWorld.y, blockSizeWorld.z));
	float maxBlockSizeWorld = fmaxf(blockSizeWorld.x, fmaxf(blockSizeWorld.y, blockSizeWorld.z));

	NvFlowFloat3 cellSize = NvFlowFloat3{
		blockSizeWorld.x / float(levelParams->blockDimLessOne.x + 1u),
		blockSizeWorld.y / float(levelParams->blockDimLessOne.y + 1u),
		blockSizeWorld.z / float(levelParams->blockDimLessOne.z + 1u)
	};
	NvFlowFloat3 cellSizeInv = { 1.f / cellSize.x, 1.f / cellSize.y, 1.f / cellSize.z };
	float cellSizeMin = fminf(fminf(cellSize.x, cellSize.y), cellSize.z);

	float stepSize = rayMarchParams->stepSizeScale * cellSizeMin;
	float stepSizeInv = 1.f / stepSize;

	dst->blockSizeWorld = blockSizeWorld;
	dst->minBlockSizeWorld = minBlockSizeWorld;
	dst->blockSizeWorldInv = blockSizeWorldInv;
	dst->maxBlockSizeWorld = maxBlockSizeWorld;
	dst->cellSize = cellSize;
	dst->stepSize = stepSize;
	dst->cellSizeInv = cellSizeInv;
	dst->stepSizeInv = stepSizeInv;
	dst->locationMin = layerParams->locationMin;
	dst->locationMax = layerParams->locationMax;
	dst->worldMin.x = (layerParams->locationMin.x - 0.5f) * layerParams->blockSizeWorld.x;
	dst->worldMin.y = (layerParams->locationMin.y - 0.5f) * layerParams->blockSizeWorld.y;
	dst->worldMin.z = (layerParams->locationMin.z - 0.5f) * layerParams->blockSizeWorld.z;
	dst->enableBlockWireframe = rayMarchParams->enableBlockWireframe;
	dst->worldMax.x = (layerParams->locationMax.x + 0.5f) * layerParams->blockSizeWorld.x;
	dst->worldMax.y = (layerParams->locationMax.y + 0.5f) * layerParams->blockSizeWorld.y;
	dst->worldMax.z = (layerParams->locationMax.z + 0.5f) * layerParams->blockSizeWorld.z;
	dst->visualizeNormals = rayMarchParams->visualizeNormals;

	dst->layer = layerParams->layer;
	dst->densityThreshold = rayMarchParams->densityThreshold;
	dst->refractionMode = rayMarchParams->refractionMode;
	dst->pad2 = 0u;

	dst->fluidColor = rayMarchParams->fluidColor;
	dst->fluidIoR = rayMarchParams->fluidIoR;
	dst->fluidSpecularReflectance = rayMarchParams->fluidSpecularReflectance;
	dst->fluidAbsorptionCoefficient = rayMarchParams->fluidAbsorptionCoefficient;
	dst->fluidDiffuseReflectance = rayMarchParams->fluidDiffuseReflectance;
	dst->pad3 = 0.f;
	dst->fluidRadiance = rayMarchParams->fluidRadiance;
	dst->pad4 = 0.f;
}

NV_FLOW_INLINE void NvFlowRayMarchShaderParams_populate(
	NvFlowRayMarchShaderParams* dst,
	NvFlowUint velocityLevelIdx,
	NvFlowUint densityLevelIdx,
	const NvFlowSparseParams* sparseParams,
	const NvFlowFloat4x4* view,
	const NvFlowFloat4x4* projection,
	const NvFlowFloat4x4* projectionJittered,
	NvFlowUint textureWidth,
	NvFlowUint textureHeight,
	NvFlowUint sceneDepthWidth,
	NvFlowUint sceneDepthHeight,
	float compositeColorScale
)
{
	using namespace NvFlowMath;

	NvFlowFloat4x4 projectionInv = matrixInverse(*projection);
	NvFlowFloat4x4 projectionJitteredInv = matrixInverse(*projectionJittered);
	NvFlowFloat4x4 viewInv = matrixInverse(*view);

	FrustumRays frustumRays = {};
	computeFrustumRays(&frustumRays, viewInv, projectionInv);

	const NvFlowSparseLevelParams* levelParamsVelocity = &sparseParams->levels[velocityLevelIdx];
	const NvFlowSparseLevelParams* levelParamsDensity = &sparseParams->levels[densityLevelIdx];

	dst->levelParamsVelocity = *levelParamsVelocity;
	dst->levelParamsDensity = *levelParamsDensity;

	dst->projection = NvFlowMath::matrixTranspose(*projection);
	dst->view = NvFlowMath::matrixTranspose(*view);
	dst->projectionJitteredInv = NvFlowMath::matrixTranspose(projectionJitteredInv);
	dst->viewInv = NvFlowMath::matrixTranspose(viewInv);

	dst->rayDir00 = frustumRays.rayDir00;
	dst->rayDir10 = frustumRays.rayDir10;
	dst->rayDir01 = frustumRays.rayDir01;
	dst->rayDir11 = frustumRays.rayDir11;

	dst->rayOrigin00 = frustumRays.rayOrigin00;
	dst->rayOrigin10 = frustumRays.rayOrigin10;
	dst->rayOrigin01 = frustumRays.rayOrigin01;
	dst->rayOrigin11 = frustumRays.rayOrigin11;

	dst->width = float(textureWidth);
	dst->height = float(textureHeight);
	dst->widthInv = 1.f / float(textureWidth);
	dst->heightInv = 1.f / float(textureHeight);

	dst->depthWidth = float(sceneDepthWidth);
	dst->depthHeight = float(sceneDepthHeight);
	dst->depthWidthInv = 1.f / float(sceneDepthWidth);
	dst->depthHeightInv = 1.f / float(sceneDepthHeight);

	dst->numLayers = sparseParams->layerCount;
	dst->maxWorldDistance = INFINITY;
	dst->isReverseZ = frustumRays.isReverseZ;
	dst->compositeColorScale = compositeColorScale;
}

NV_FLOW_INLINE void NvFlowSelfShadowLayerShaderParams_populate(
	NvFlowSelfShadowLayerShaderParams* dst,
	NvFlowUint coarseLevelIdx,
	NvFlowUint fineLevelIdx,
	NvFlowUint layerParamIdx,
	const NvFlowSparseParams* sparseParams,
	const NvFlowShadowParams* shadowParams
)
{
	bool isCoarse = coarseLevelIdx != fineLevelIdx;

	const NvFlowSparseLevelParams* coarseDensityLevelParams = &sparseParams->levels[coarseLevelIdx];
	const NvFlowSparseLayerParams* layerParams = &sparseParams->layers[layerParamIdx];

	int layer = layerParams->layer;
	NvFlowFloat3 blockSizeWorld = layerParams->blockSizeWorld;
	NvFlowFloat3 blockSizeWorldInv = layerParams->blockSizeWorldInv;
	float minBlockSizeWorld = fminf(blockSizeWorld.x, fminf(blockSizeWorld.y, blockSizeWorld.z));
	float maxBlockSizeWorld = fmaxf(blockSizeWorld.x, fmaxf(blockSizeWorld.y, blockSizeWorld.z));

	NvFlowFloat3 cellSize = NvFlowFloat3{
		blockSizeWorld.x / float(coarseDensityLevelParams->blockDimLessOne.x + 1u),
		blockSizeWorld.y / float(coarseDensityLevelParams->blockDimLessOne.y + 1u),
		blockSizeWorld.z / float(coarseDensityLevelParams->blockDimLessOne.z + 1u)
	};
	NvFlowFloat3 cellSizeInv = { 1.f / cellSize.x, 1.f / cellSize.y, 1.f / cellSize.z };
	float cellSizeMin = fminf(fminf(cellSize.x, cellSize.y), cellSize.z);

	float stepSize = shadowParams->stepSizeScale * cellSizeMin;
	float stepSizeInv = 1.f / stepSize;
	float stepOffset = shadowParams->stepOffsetScale * cellSizeMin;

	// normalize alphaScale based on stepSize
	float alphaScale = 1.f - expf(-shadowParams->attenuation * stepSize);
	float layerColormapV = (float(layerParamIdx) + 0.5f) / float(sparseParams->layerCount);

	dst->base.blockSizeWorld = blockSizeWorld;
	dst->base.minBlockSizeWorld = minBlockSizeWorld;
	dst->base.blockSizeWorldInv = blockSizeWorldInv;
	dst->base.maxBlockSizeWorld = maxBlockSizeWorld;
	dst->base.cellSize = cellSize;
	dst->base.stepSize = stepSize;
	dst->base.cellSizeInv = cellSizeInv;
	dst->base.stepSizeInv = stepSizeInv;
	dst->base.locationMin = layerParams->locationMin;
	dst->base.locationMax = layerParams->locationMax;
	dst->base.worldMin.x = (layerParams->locationMin.x - 0.5f) * layerParams->blockSizeWorld.x;
	dst->base.worldMin.y = (layerParams->locationMin.y - 0.5f) * layerParams->blockSizeWorld.y;
	dst->base.worldMin.z = (layerParams->locationMin.z - 0.5f) * layerParams->blockSizeWorld.z;
	dst->base.enableBlockWireframe = NV_FLOW_FALSE;
	dst->base.worldMax.x = (layerParams->locationMax.x + 0.5f) * layerParams->blockSizeWorld.x;
	dst->base.worldMax.y = (layerParams->locationMax.y + 0.5f) * layerParams->blockSizeWorld.y;
	dst->base.worldMax.z = (layerParams->locationMax.z + 0.5f) * layerParams->blockSizeWorld.z;
	dst->base.enableRawMode = NV_FLOW_FALSE;

	dst->base.velocityCellSizeInv = cellSizeInv;
	dst->base.deltaTime = layerParams->deltaTime;

	dst->base.layer = layer;
	dst->base.layerColormapV = layerColormapV;
	dst->base.alphaScale = alphaScale;
	dst->base.colorScale = 1.f;

	dst->base.shadowFactor = 0.f;
	dst->base.pad1 = 0.f;
	dst->base.pad2 = 0.f;
	dst->base.pad3 = 0.f;

	// Set cloud mode to default
	dst->base.cloud.densityMultiplier = 0.5f;
	dst->base.cloud.enableCloudMode = NV_FLOW_FALSE;
	dst->base.cloud.pad1 = 0.f;
	dst->base.cloud.pad2 = 0.f;
	dst->base.cloud.ambientColor.x = 0.4f;
	dst->base.cloud.ambientColor.y = 0.55f;
	dst->base.cloud.ambientColor.z = 0.9f;
	dst->base.cloud.ambientMultiplier = 1.f;
	dst->base.cloud.volumeBaseColor.x = 1.1f;
	dst->base.cloud.volumeBaseColor.y = 1.f;
	dst->base.cloud.volumeBaseColor.z = 0.9f;
	dst->base.cloud.volumeColorMultiplier = 1.f;
	dst->base.cloud.sunDirection.x = 1.f;
	dst->base.cloud.sunDirection.y = 1.f;
	dst->base.cloud.sunDirection.z = 1.f;
	dst->base.cloud.shadowStepMultiplier = 1.f;
	dst->base.cloud.attenuationMultiplier.x = 1.f;
	dst->base.cloud.attenuationMultiplier.y = 1.f;
	dst->base.cloud.attenuationMultiplier.z = 1.f;
	dst->base.cloud.numShadowSteps = 10u;

	dst->minIntensity = shadowParams->minIntensity;
	dst->numSteps = isCoarse ? (shadowParams->numSteps / 2u) : shadowParams->numSteps;
	dst->isPointLight = shadowParams->isPointLight;
	dst->stepOffset = stepOffset;

	dst->lightDirection = shadowParams->lightDirection;
	dst->enabled = shadowParams->enabled;
	dst->lightPosition = shadowParams->lightPosition;
	dst->pad3 = 0.f;
}

NV_FLOW_INLINE void NvFlowSelfShadowShaderParams_populate(
	NvFlowSelfShadowShaderParams* dst,
	NvFlowUint coarseLevelIdx,
	NvFlowUint fineLevelIdx,
	const NvFlowSparseParams* sparseParams,
	NvFlowUint blockIdxOffset
)
{
	dst->blockIdxOffset = blockIdxOffset;
	dst->pad1 = 0u;
	dst->pad2 = 0u;
	dst->pad3 = 0u;
	dst->coarseDensityTable = sparseParams->levels[coarseLevelIdx];
	dst->densityTable = sparseParams->levels[fineLevelIdx];
}

#endif

#endif