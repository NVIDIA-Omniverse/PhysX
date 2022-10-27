/*
 * Copyright (c) 2014-2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#include "NvFlowLoader.h"

#include "ShapeRenderer.h"

#include "NvFlowUploadBuffer.h"
#include "NvFlowDynamicBuffer.h"

#include "NvFlowMath.h"

#include "shaders/ShapeParams.h"
#include "shaders/ShapeCS.hlsl.h"

namespace NvFlowShapeRendererDefault
{
	struct Renderer
	{
		NvFlowContextInterface contextInterface = {};

		ShapeCS_Pipeline shapeCS = {};

		NvFlowUploadBuffer spherePositionBuffer = {};
		NvFlowUploadBuffer constantBuffer = {};
	};

	NV_FLOW_CAST_PAIR(NvFlowShapeRenderer, Renderer)

	NvFlowShapeRenderer* create(NvFlowContextInterface* contextInterface, NvFlowContext* context)
	{
		auto ptr = new Renderer();

		NvFlowContextInterface_duplicate(&ptr->contextInterface, contextInterface);

		ShapeCS_init(&ptr->contextInterface, context, &ptr->shapeCS);

		NvFlowBufferUsageFlags bufferUsage = eNvFlowBufferUsage_structuredBuffer | eNvFlowBufferUsage_bufferCopySrc;

		NvFlowUploadBuffer_init(&ptr->contextInterface, context, &ptr->spherePositionBuffer, bufferUsage, eNvFlowFormat_unknown, sizeof(NvFlowFloat4));
		NvFlowUploadBuffer_init(&ptr->contextInterface, context, &ptr->constantBuffer, eNvFlowBufferUsage_constantBuffer, eNvFlowFormat_unknown, 0u);

		return cast(ptr);
	}

	void destroy(NvFlowContext* context, NvFlowShapeRenderer* renderer)
	{
		auto ptr = cast(renderer);

		NvFlowUploadBuffer_destroy(context, &ptr->spherePositionBuffer);
		NvFlowUploadBuffer_destroy(context, &ptr->constantBuffer);

		ShapeCS_destroy(context, &ptr->shapeCS);

		delete ptr;
	}

	void render(
		NvFlowContext* context,
		NvFlowShapeRenderer* renderer, const NvFlowShapeRendererParams* params,
		const NvFlowFloat4x4* view,
		const NvFlowFloat4x4* projection,
		NvFlowUint textureWidth,
		NvFlowUint textureHeight,
		NvFlowTextureTransient* depthOut,
		NvFlowTextureTransient* colorOut
	)
	{
		auto ptr = cast(renderer);

		using namespace NvFlowMath;

		NvFlowFloat4x4 projectionInv = matrixInverse(*projection);
		NvFlowFloat4x4 viewInv = matrixInverse(*view);

		FrustumRays frustumRays = {};
		computeFrustumRays(&frustumRays, viewInv, projectionInv);

		NvFlowUint64 numBytesSpherePositions = (params->numSpheres + 1u) * sizeof(NvFlowFloat4);

		auto mappedSpherePos = (NvFlowFloat4*)NvFlowUploadBuffer_map(context, &ptr->spherePositionBuffer, numBytesSpherePositions);

		for (NvFlowUint idx = 0u; idx < params->numSpheres; idx++)
		{
			mappedSpherePos[idx] = params->spherePositionRadius[idx];
		}

		NvFlowBufferTransient* spherePositionRadiusTransient = NvFlowUploadBuffer_unmap(context, &ptr->spherePositionBuffer);

		auto mapped = (ShapeRendererParams*)NvFlowUploadBuffer_map(context, &ptr->constantBuffer, sizeof(ShapeRendererParams));

		mapped->projection = NvFlowMath::matrixTranspose(*projection);
		mapped->view = NvFlowMath::matrixTranspose(*view);
		mapped->projectionInv = NvFlowMath::matrixTranspose(projectionInv);
		mapped->viewInv = NvFlowMath::matrixTranspose(viewInv);

		mapped->rayDir00 = frustumRays.rayDir00;
		mapped->rayDir10 = frustumRays.rayDir10;
		mapped->rayDir01 = frustumRays.rayDir01;
		mapped->rayDir11 = frustumRays.rayDir11;

		mapped->rayOrigin00 = frustumRays.rayOrigin00;
		mapped->rayOrigin10 = frustumRays.rayOrigin10;
		mapped->rayOrigin01 = frustumRays.rayOrigin01;
		mapped->rayOrigin11 = frustumRays.rayOrigin11;

		mapped->width = float(textureWidth);
		mapped->height = float(textureHeight);
		mapped->widthInv = 1.f / float(textureWidth);
		mapped->heightInv = 1.f / float(textureHeight);

		mapped->numSpheres = params->numSpheres;
		mapped->clearDepth = 1.f - frustumRays.nearZ;
		mapped->isReverseZ = frustumRays.nearZ > 0.5f ? 1u : 0u;
		mapped->pad3 = 0u;

		mapped->clearColor = NvFlowFloat4{ 0.f, 0.f, 0.f, 1.f };

		NvFlowBufferTransient* paramsInTransient = NvFlowUploadBuffer_unmap(context, &ptr->constantBuffer);

		// render
		{
			NvFlowUint3 gridDim = {
				(textureWidth + 7u) / 8u,
				(textureHeight + 7u) / 8u,
				1u
			};

			ShapeCS_PassParams passParams = {};
			passParams.paramsIn = paramsInTransient;
			passParams.spherePositionRadiusIn = spherePositionRadiusTransient;
			passParams.depthOut = depthOut;
			passParams.colorOut = colorOut;

			ShapeCS_addPassCompute(context, &ptr->shapeCS, gridDim, &passParams);
		}
	}
}

NvFlowShapeRendererInterface* NvFlowGetShapeRendererInterface()
{
	using namespace NvFlowShapeRendererDefault;
	static NvFlowShapeRendererInterface iface = { NV_FLOW_REFLECT_INTERFACE_INIT(NvFlowShapeRendererInterface) };
	iface.create = create;
	iface.destroy = destroy;
	iface.render = render;
	return &iface;
}