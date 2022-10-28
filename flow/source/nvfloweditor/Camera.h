#pragma once

#include "NvFlowTypes.h"

struct NvFlowCamera;

/// ********************* Camera *******************

enum NvFlowCameraAction
{
	eNvFlowCameraAction_unknown = 0,
	eNvFlowCameraAction_down = 1,
	eNvFlowCameraAction_up = 2,

	eNvFlowCameraAction_maxEnum = 0x7FFFFFFF
};

enum NvFlowCameraMouseButton
{
	eNvFlowCameraMouseButton_unknown = 0,
	eNvFlowCameraMouseButton_left = 1,
	eNvFlowCameraMouseButton_middle = 2,
	eNvFlowCameraMouseButton_right = 3,

	eNvFlowCameraMouseButton_maxEnum = 0x7FFFFFFF
};

enum NvFlowCameraKey
{
	eNvFlowCameraKey_unknown = 0,
	eNvFlowCameraKey_up = 1,
	eNvFlowCameraKey_down = 2,
	eNvFlowCameraKey_left = 3,
	eNvFlowCameraKey_right = 4,

	eNvFlowCameraKey_maxEnum = 0x7FFFFFFF
};

struct NvFlowCameraState
{
	NvFlowFloat3 position;
	NvFlowFloat3 eyeDirection;
	NvFlowFloat3 eyeUp;
	float eyeDistanceFromPosition;
};

struct NvFlowCameraConfig
{
	NvFlowBool32 isProjectionRH;
	NvFlowBool32 isOrthographic;
	NvFlowBool32 isReverseZ;
	float nearPlane;
	float farPlane;
	float fovAngleY;
	float orthographicY;
	float panRate;
	float tiltRate;
	float zoomRate;
	float keyTranslationRate;
};

NV_FLOW_API NvFlowCamera* NvFlowCameraCreate(int winw, int winh);

NV_FLOW_API void NvFlowCameraDestroy(NvFlowCamera* camera);

NV_FLOW_API void NvFlowCameraGetDefaultState(NvFlowCameraState* state, bool yUp);

NV_FLOW_API void NvFlowCameraGetDefaultConfig(NvFlowCameraConfig* config);

NV_FLOW_API void NvFlowCameraGetState(NvFlowCamera* camera, NvFlowCameraState* state);

NV_FLOW_API void NvFlowCameraSetState(NvFlowCamera* camera, const NvFlowCameraState* state);

NV_FLOW_API void NvFlowCameraGetConfig(NvFlowCamera* camera, NvFlowCameraConfig* config);

NV_FLOW_API void NvFlowCameraSetConfig(NvFlowCamera* camera, const NvFlowCameraConfig* config);

NV_FLOW_API void NvFlowCameraGetView(NvFlowCamera* camera, NvFlowFloat4x4* view);

NV_FLOW_API void NvFlowCameraGetProjection(NvFlowCamera* camera, NvFlowFloat4x4* projection, float aspectWidth, float aspectHeight);

NV_FLOW_API void NvFlowCameraMouseUpdate(NvFlowCamera* camera, NvFlowCameraMouseButton button, NvFlowCameraAction action, int mouseX, int mouseY, int winw, int winh);

NV_FLOW_API void NvFlowCameraKeyUpdate(NvFlowCamera* camera, NvFlowCameraKey key, NvFlowCameraAction action);

NV_FLOW_API void NvFlowCameraAnimationTick(NvFlowCamera* camera, float deltaTime);