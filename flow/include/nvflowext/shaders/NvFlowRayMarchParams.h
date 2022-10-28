/*
 * Copyright (c) 2014-2020, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef NV_FLOW_RAY_MARCH_PARAMS_H
#define NV_FLOW_RAY_MARCH_PARAMS_H

#include "NvFlowShaderTypes.h"

struct NvFlowRayMarchCloudLayerShaderParams
{
	float densityMultiplier;
	NvFlowUint enableCloudMode;
	float pad1;
	float pad2;
	NvFlowFloat3 ambientColor;
	float ambientMultiplier;
	NvFlowFloat3 volumeBaseColor;
	float volumeColorMultiplier;
	NvFlowFloat3 sunDirection;
	float shadowStepMultiplier;
	NvFlowFloat3 attenuationMultiplier;
	int numShadowSteps;
};
#ifdef NV_FLOW_CPU
typedef struct NvFlowRayMarchCloudLayerShaderParams NvFlowRayMarchCloudLayerShaderParams;
#endif

struct NvFlowRayMarchLayerShaderParams
{
	NvFlowFloat3 blockSizeWorld;
	float minBlockSizeWorld;
	NvFlowFloat3 blockSizeWorldInv;
	float maxBlockSizeWorld;
	NvFlowFloat3 cellSize;
	float stepSize;
	NvFlowFloat3 cellSizeInv;
	float stepSizeInv;	
	NvFlowInt4 locationMin;
	NvFlowInt4 locationMax;
	NvFlowFloat3 worldMin;
	NvFlowUint enableBlockWireframe;
	NvFlowFloat3 worldMax;
	NvFlowUint enableRawMode;

	NvFlowFloat3 velocityCellSizeInv;
	float deltaTime;

	int layer;
	float layerColormapV;
	float alphaScale;
	float colorScale;

	float shadowFactor;
	float pad1;
	float pad2;
	float pad3;

	NvFlowRayMarchCloudLayerShaderParams cloud;
};
#ifdef NV_FLOW_CPU
typedef struct NvFlowRayMarchLayerShaderParams NvFlowRayMarchLayerShaderParams;
#endif

struct NvFlowRayMarchIsosurfaceLayerShaderParams
{
	NvFlowFloat3 blockSizeWorld;
	float minBlockSizeWorld;
	NvFlowFloat3 blockSizeWorldInv;
	float maxBlockSizeWorld;
	NvFlowFloat3 cellSize;
	float stepSize;
	NvFlowFloat3 cellSizeInv;
	float stepSizeInv;
	NvFlowInt4 locationMin;
	NvFlowInt4 locationMax;
	NvFlowFloat3 worldMin;
	NvFlowUint enableBlockWireframe;
	NvFlowFloat3 worldMax;
	NvFlowUint visualizeNormals;

	int layer;
	float densityThreshold;
	NvFlowUint refractionMode;
	NvFlowUint pad2;

	NvFlowFloat3 fluidColor;
	float fluidIoR;
	NvFlowFloat3 fluidSpecularReflectance;
	float fluidAbsorptionCoefficient;
	NvFlowFloat3 fluidDiffuseReflectance;
	float pad3;
	NvFlowFloat3 fluidRadiance;
	float pad4;
};
#ifdef NV_FLOW_CPU
typedef struct NvFlowRayMarchIsosurfaceLayerShaderParams NvFlowRayMarchIsosurfaceLayerShaderParams;
#endif

struct NvFlowRayMarchShaderParams
{
	NvFlowSparseLevelParams levelParamsVelocity;
	NvFlowSparseLevelParams levelParamsDensity;

	NvFlowFloat4x4 projection;
	NvFlowFloat4x4 view;
	NvFlowFloat4x4 projectionJitteredInv;
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

	float depthWidth;
	float depthHeight;
	float depthWidthInv;
	float depthHeightInv;

	NvFlowUint numLayers;
	float maxWorldDistance;
	NvFlowUint isReverseZ;
	float compositeColorScale;
};
#ifdef NV_FLOW_CPU
typedef struct NvFlowRayMarchShaderParams NvFlowRayMarchShaderParams;
#endif

struct NvFlowSelfShadowLayerShaderParams
{
	NvFlowRayMarchLayerShaderParams base;
	float minIntensity;
	NvFlowUint numSteps;
	NvFlowUint isPointLight;
	float stepOffset;
	NvFlowFloat3 lightDirection;
	NvFlowUint enabled;
	NvFlowFloat3 lightPosition;
	float pad3;
};
#ifdef NV_FLOW_CPU
typedef struct NvFlowSelfShadowLayerShaderParams NvFlowSelfShadowLayerShaderParams;
#endif

struct NvFlowSelfShadowShaderParams
{
	NvFlowUint blockIdxOffset;
	NvFlowUint pad1;
	NvFlowUint pad2;
	NvFlowUint pad3;
	NvFlowSparseLevelParams coarseDensityTable;
	NvFlowSparseLevelParams densityTable;
};
#ifdef NV_FLOW_CPU
typedef struct NvFlowSelfShadowShaderParams NvFlowSelfShadowShaderParams;
#endif

#endif