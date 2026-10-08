// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PHYSX_SNIPPET_CAMERA_H
#define PHYSX_SNIPPET_CAMERA_H

#include "foundation/PxTransform.h"

namespace Snippets
{
class Camera
{
public:
						Camera(const physx::PxVec3& eye, const physx::PxVec3& dir);

	void				handleMouse(int button, int state, int x, int y);
	bool				handleKey(unsigned char key, int x, int y, float speed = 0.5f);
	void				handleMotion(int x, int y);
	void				handleAnalogMove(float x, float y);

	physx::PxVec3		getEye()	const;
	physx::PxVec3		getDir()	const;
	physx::PxTransform	getTransform() const;

	void				setPose(const physx::PxVec3& eye, const physx::PxVec3& dir);
	void				setSpeed(float speed);
private:
	physx::PxVec3		mEye;
	physx::PxVec3		mDir;
	int					mMouseX;
	int					mMouseY;
	float				mSpeed;
};

}

#endif //PHYSX_SNIPPET_CAMERA_H
