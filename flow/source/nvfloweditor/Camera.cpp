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

#include "Camera.h"

#include "NvFlowMath.h"

struct NvFlowCamera
{
	// Settings
	NvFlowCameraConfig config = {};
	// Camera state
	NvFlowCameraState state = {};

	// Mouse state
	int mouseXprev = 0;
	int mouseYprev = 0;
	NvFlowBool32 rotationActive = false;
	NvFlowBool32 zoomActive = false;
	NvFlowBool32 translateActive = false;
	NvFlowUint keyTranslateActiveMask = 0u;
};

void NvFlowCamera_computeRotationBasis(NvFlowCamera* ptr, NvFlowFloat4* pXAxis, NvFlowFloat4* pYAxis, NvFlowFloat4* pZAxis)
{
	using namespace NvFlowMath;

	NvFlowFloat4 zAxis = vector3Normalize(make_float4(ptr->state.eyeDirection, 0.f));
	// RH Z is negative going into screen, so reverse eyeDirectionVector, after building basis
	if (ptr->config.isProjectionRH)
	{
		zAxis.x = -zAxis.x;
		zAxis.y = -zAxis.y;
		zAxis.z = -zAxis.z;
	}
	NvFlowFloat4 yAxis = make_float4(ptr->state.eyeUp, 0.f);

	// force yAxis to orthogonal
	yAxis = vector3Normalize(yAxis - vector3Dot(zAxis, yAxis) * zAxis);

	// generate third basis vector
	NvFlowFloat4 xAxis = vector3Cross(yAxis, zAxis);

	if (pXAxis)
	{
		*pXAxis = xAxis;
	}
	if (pYAxis)
	{
		*pYAxis = yAxis;
	}
	if (pZAxis)
	{
		*pZAxis = zAxis;
	}
}

NvFlowCamera* NvFlowCameraCreate(int winw, int winh)
{
	auto ptr = new NvFlowCamera();

	NvFlowCameraGetDefaultState(&ptr->state, false);
	NvFlowCameraGetDefaultConfig(&ptr->config);

	return ptr;
}

void NvFlowCameraDestroy(NvFlowCamera* ptr)
{
	delete ptr;
}

void NvFlowCameraGetDefaultState(NvFlowCameraState* ptr, bool yUp)
{
	ptr->position = { 0.f, 0.f, 0.f };
	if (yUp)
	{
		ptr->eyeDirection = { 0.f, 0.f, 1.f };
		ptr->eyeUp = { 0.f, 1.f, 0.f };
	}
	else
	{
		ptr->eyeDirection = { 0.f, 1.f, 0.f };
		ptr->eyeUp = { 0.f, 0.f, 1.f };
	}
	ptr->eyeDistanceFromPosition = -700.f;
}

void NvFlowCameraGetDefaultConfig(NvFlowCameraConfig* ptr)
{
	using namespace NvFlowMath;

	ptr->isProjectionRH = NV_FLOW_TRUE;
	ptr->isOrthographic = NV_FLOW_FALSE;
	ptr->isReverseZ = NV_FLOW_TRUE;
	ptr->nearPlane = 0.1f;
	ptr->farPlane = INFINITY;
	ptr->fovAngleY = pi / 4.f;
	ptr->orthographicY = 500.f;
	ptr->panRate = 1.f;
	ptr->tiltRate = 1.f;
	ptr->zoomRate = 1.f;
	ptr->keyTranslationRate = 800.f;
}

void NvFlowCameraGetState(NvFlowCamera* ptr, NvFlowCameraState* state)
{
	*state = ptr->state;
}

void NvFlowCameraSetState(NvFlowCamera* ptr, const NvFlowCameraState* state)
{
	ptr->state = *state;
}

void NvFlowCameraGetConfig(NvFlowCamera* ptr, NvFlowCameraConfig* config)
{
	*config = ptr->config;
}

void NvFlowCameraSetConfig(NvFlowCamera* ptr, const NvFlowCameraConfig* config)
{
	ptr->config = *config;
}

void NvFlowCameraGetView(NvFlowCamera* ptr, NvFlowFloat4x4* viewMatrix)
{
	using namespace NvFlowMath;

	auto state = &ptr->state;

	float eyeDistanceWithDepth = state->eyeDistanceFromPosition;

	NvFlowFloat3 eyePosition = state->position;
	eyePosition.x -= state->eyeDirection.x * state->eyeDistanceFromPosition;
	eyePosition.y -= state->eyeDirection.y * state->eyeDistanceFromPosition;
	eyePosition.z -= state->eyeDirection.z * state->eyeDistanceFromPosition;

	NvFlowFloat4x4 translate = matrixTranslation(eyePosition.x, eyePosition.y, eyePosition.z);

	// derive rotation from eyeDirection, eyeUp vectors
	NvFlowFloat4x4 rotation = {};
	{
		NvFlowFloat4 zAxis = {};
		NvFlowFloat4 xAxis = {};
		NvFlowFloat4 yAxis = {};
		NvFlowCamera_computeRotationBasis(ptr, &xAxis, &yAxis, &zAxis);

		rotation = NvFlowFloat4x4{
			xAxis.x, yAxis.x, zAxis.x, 0.f,
			xAxis.y, yAxis.y, zAxis.y, 0.f,
			xAxis.z, yAxis.z, zAxis.z, 0.f,
			0.f, 0.f, 0.f, 1.f
		};
	}

	NvFlowFloat4x4 view = matrixMultiply(translate, rotation);

	*viewMatrix = view;
}

void NvFlowCameraGetProjection(NvFlowCamera* ptr, NvFlowFloat4x4* projMatrix, float aspectWidth, float aspectHeight)
{
	using namespace NvFlowMath;

	float aspectRatio = aspectWidth / aspectHeight;

	NvFlowFloat4x4 projection = {};
	if (ptr->config.isOrthographic)
	{
		if (ptr->config.isProjectionRH)
		{
			if (ptr->config.isReverseZ)
			{
				projection = matrixOrthographicRH(ptr->config.orthographicY * aspectRatio, ptr->config.orthographicY, ptr->config.farPlane, ptr->config.nearPlane);
			}
			else
			{
				projection = matrixOrthographicRH(ptr->config.orthographicY * aspectRatio, ptr->config.orthographicY, ptr->config.nearPlane, ptr->config.farPlane);
			}
			*projMatrix = projection;
		}
		else
		{
			if (ptr->config.isReverseZ)
			{
				projection = matrixOrthographicLH(ptr->config.orthographicY * aspectRatio, ptr->config.orthographicY, ptr->config.farPlane, ptr->config.nearPlane);
			}
			else
			{
				projection = matrixOrthographicLH(ptr->config.orthographicY * aspectRatio, ptr->config.orthographicY, ptr->config.nearPlane, ptr->config.farPlane);
			}
			*projMatrix = projection;
		}
	}
	else
	{
		if (ptr->config.isProjectionRH)
		{
			if (ptr->config.isReverseZ)
			{
				projection = matrixPerspectiveFovRH(ptr->config.fovAngleY, aspectRatio, ptr->config.farPlane, ptr->config.nearPlane);
			}
			else
			{
				projection = matrixPerspectiveFovRH(ptr->config.fovAngleY, aspectRatio, ptr->config.nearPlane, ptr->config.farPlane);
			}
			*projMatrix = projection;
		}
		else
		{
			if (ptr->config.isReverseZ)
			{
				projection = matrixPerspectiveFovLH(ptr->config.fovAngleY, aspectRatio, ptr->config.farPlane, ptr->config.nearPlane);
			}
			else
			{
				projection = matrixPerspectiveFovLH(ptr->config.fovAngleY, aspectRatio, ptr->config.nearPlane, ptr->config.farPlane);
			}
			*projMatrix = projection;
		}
	}
}

void NvFlowCameraMouseUpdate(NvFlowCamera* ptr, NvFlowCameraMouseButton button, NvFlowCameraAction action, int mouseX, int mouseY, int winw, int winh)
{
	using namespace NvFlowMath;

	// transient mouse state
	float rotationDx = 0.f;
	float rotationDy = 0.f;
	float translateDx = 0.f;
	float translateDy = 0.f;
	int translateWinW = 1024;
	int translateWinH = 1024;
	float zoomDy = 0.f;

	// process event
	if (action == eNvFlowCameraAction_down)
	{
		if (button == eNvFlowCameraMouseButton_left)
		{
			ptr->rotationActive = true;
			rotationDx = 0.f;
			rotationDy = 0.f;
		}
		else if (button == eNvFlowCameraMouseButton_middle)
		{
			ptr->translateActive = true;
			translateDx = 0.f;
			translateDy = 0.f;
		}
		else if (button == eNvFlowCameraMouseButton_right)
		{
			ptr->zoomActive = true;
			zoomDy = 0.f;
		}
	}
	else if (action == eNvFlowCameraAction_up)
	{
		if (button == eNvFlowCameraMouseButton_left)
		{
			ptr->rotationActive = false;
			rotationDx = 0.f;
			rotationDy = 0.f;
		}
		else if (button == eNvFlowCameraMouseButton_middle)
		{
			ptr->translateActive = false;
			translateDx = 0.f;
			translateDy = 0.f;
		}
		else if (button == eNvFlowCameraMouseButton_right)
		{
			ptr->zoomActive = false;
			zoomDy = 0.f;
		}
	}
	else if (action == eNvFlowCameraAction_unknown)
	{
		if (ptr->rotationActive)
		{
			int dx = +(mouseX - ptr->mouseXprev);
			int dy = +(mouseY - ptr->mouseYprev);

			rotationDx = float(dx) * 2.f * 3.14f / (winw);
			rotationDy = float(dy) * 2.f * 3.14f / (winh);
		}
		if (ptr->translateActive)
		{
			float dx = float(mouseX - ptr->mouseXprev);
			float dy = -float(mouseY - ptr->mouseYprev);

			translateDx = dx * 2.f / (winw);
			translateDy = dy * 2.f / (winh);

			translateWinW = winw;
			translateWinH = winh;
		}
		if (ptr->zoomActive)
		{
			float dy = -float(mouseY - ptr->mouseYprev);

			zoomDy = dy * 3.14f / float(winh);
		}
	}

	// keep current mouse position for next previous
	ptr->mouseXprev = mouseX;
	ptr->mouseYprev = mouseY;

	// apply rotation
	if (rotationDx != 0.f || rotationDy != 0.f)
	{
		float dx = rotationDx;
		float dy = rotationDy;

		if (ptr->config.isProjectionRH)
		{
			dx = -dx;
			dy = -dy;
		}

		float rotTilt = ptr->config.tiltRate * float(dy);
		float rotPan = ptr->config.panRate * float(dx);

		const float eyeDotLimit = 0.99f;

		// tilt
		{
			NvFlowFloat4 eyeDirection4 = make_float4(ptr->state.eyeDirection, 0.f);

			NvFlowFloat4 rotVec = {};

			NvFlowCamera_computeRotationBasis(ptr, &rotVec, nullptr, nullptr);

			const float angle = rotTilt;
			NvFlowFloat4x4 dtilt = matrixRotationAxis(rotVec, angle);

			eyeDirection4 = vector4Transform(eyeDirection4, dtilt);

			// make sure eye direction stays normalized
			eyeDirection4 = vector3Normalize(eyeDirection4);

			// check dot of eyeDirection and eyeUp, and avoid commit if value is very low
			float eyeDot = fabsf(
				eyeDirection4.x * ptr->state.eyeUp.x +
				eyeDirection4.y * ptr->state.eyeUp.y +
				eyeDirection4.z * ptr->state.eyeUp.z
			);

			if (eyeDot < eyeDotLimit)
			{
				ptr->state.eyeDirection = float4_to_float3(eyeDirection4);
			}
		}
		// pan
		{
			NvFlowFloat4 eyeDirection4 = make_float4(ptr->state.eyeDirection, 0.f);

			NvFlowFloat4 rotVec = make_float4(ptr->state.eyeUp, 0.f);
			const float angle = rotPan;
			NvFlowFloat4x4 dpan = matrixRotationAxis(rotVec, angle);

			eyeDirection4 = vector4Transform(eyeDirection4, dpan);

			// make sure eye direction stays normalized
			eyeDirection4 = vector3Normalize(eyeDirection4);

			ptr->state.eyeDirection = float4_to_float3(eyeDirection4);
		}
	}

	// apply translation
	if (translateDx != 0.f || translateDy != 0.f)
	{
		// goal here is to apply an NDC offset, to the position value in world space
		NvFlowFloat4x4 projection = {};
		NvFlowCameraGetProjection(ptr, &projection, float(translateWinW), float(translateWinH));

		NvFlowFloat4x4 view = {};
		NvFlowCameraGetView(ptr, &view);

		// project position to NDC
		NvFlowFloat4 positionNDC = make_float4(ptr->state.position, 1.f);
		positionNDC = vector4Transform(positionNDC, view);
		positionNDC = vector4Transform(positionNDC, projection);

		// normalize
		if (positionNDC.w > 0.f)
		{
			positionNDC = positionNDC / vectorSplatW(positionNDC);
		}

		// offset using mouse data
		positionNDC.x += translateDx;
		positionNDC.y += translateDy;

		// move back to world space
		NvFlowFloat4x4 projViewInverse = matrixInverse(
			matrixMultiply(view, projection)
		);

		NvFlowFloat4 positionWorld = vector4Transform(positionNDC, projViewInverse);

		// normalize
		if (positionWorld.w > 0.f)
		{
			positionWorld = positionWorld / vectorSplatW(positionWorld);
		}

		// commit update
		ptr->state.position = float4_to_float3(positionWorld);
	}

	// apply zoom
	if (zoomDy != 0.f)
	{
		ptr->state.eyeDistanceFromPosition *= (1.f + ptr->config.zoomRate * zoomDy);
	}
}

void NvFlowCameraKeyUpdate(NvFlowCamera* ptr, NvFlowCameraKey key, NvFlowCameraAction action)
{
	if (action == eNvFlowCameraAction_down)
	{
		if (key == eNvFlowCameraKey_up)
		{
			ptr->keyTranslateActiveMask |= 2u;
		}
		if (key == eNvFlowCameraKey_down)
		{
			ptr->keyTranslateActiveMask |= 4u;
		}
		if (key == eNvFlowCameraKey_left)
		{
			ptr->keyTranslateActiveMask |= 8u;
		}
		if (key == eNvFlowCameraKey_right)
		{
			ptr->keyTranslateActiveMask |= 16u;
		}
	}
	else if (action == eNvFlowCameraAction_up)
	{
		if (key == eNvFlowCameraKey_up)
		{
			ptr->keyTranslateActiveMask &= ~2u;
		}
		if (key == eNvFlowCameraKey_down)
		{
			ptr->keyTranslateActiveMask &= ~4u;
		}
		if (key == eNvFlowCameraKey_left)
		{
			ptr->keyTranslateActiveMask &= ~8u;
		}
		if (key == eNvFlowCameraKey_right)
		{
			ptr->keyTranslateActiveMask &= ~16u;
		}
	}
}

void NvFlowCameraAnimationTick(NvFlowCamera* ptr, float deltaTime)
{
	using namespace NvFlowMath;

	float x = 0.f;
	float y = 0.f;
	float z = 0.f;

	float rate = ptr->config.keyTranslationRate * deltaTime;

	if (ptr->keyTranslateActiveMask & 2u)
	{
		z += -rate;
	}
	if (ptr->keyTranslateActiveMask & 4u)
	{
		z += +rate;
	}
	if (ptr->keyTranslateActiveMask & 8)
	{
		x += +rate;
	}
	if (ptr->keyTranslateActiveMask & 16)
	{
		x += -rate;
	}

	if (ptr->keyTranslateActiveMask)
	{
		ptr->state.position.x += ptr->state.eyeDirection.x * z;
		ptr->state.position.y += ptr->state.eyeDirection.y * z;
		ptr->state.position.z += ptr->state.eyeDirection.z * z;

		// compute xaxis
		NvFlowFloat4 xAxis{};
		NvFlowCamera_computeRotationBasis(ptr, &xAxis, nullptr, nullptr);

		ptr->state.position.x += xAxis.x * x;
		ptr->state.position.y += xAxis.y * x;
		ptr->state.position.z += xAxis.z * x;
	}
}