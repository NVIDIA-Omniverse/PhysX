// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifdef RENDER_SNIPPET

#include "PxPhysicsAPI.h"

#include "../snippetrender/SnippetRender.h"
#include "../snippetrender/SnippetCamera.h"
#include "../snippetutils/SnippetUtils.h"

using namespace physx;

extern void initPhysics(bool interactive);
extern void stepPhysics(bool interactive);	
extern void cleanupPhysics(bool interactive);
extern void keyPress(unsigned char key, const PxTransform& camera);
extern void renderScene();

namespace
{
Snippets::Camera* sCamera;

void renderCallback()
{
	stepPhysics(true);

	Snippets::startRender(sCamera);

//		PxVec3 camPos = sCamera->getEye();
//		PxVec3 camDir = sCamera->getDir();
//		printf("camPos: (%ff, %ff, %ff)\n", camPos.x, camPos.y, camPos.z);
//		printf("camDir: (%ff, %ff, %ff)\n", camDir.x, camDir.y, camDir.z);
	
	renderScene();

	Snippets::finishRender();
}

void exitCallback()
{
	delete sCamera;
	cleanupPhysics(true);
}
}

void renderLoop()
{
	sCamera = new Snippets::Camera(PxVec3(-1.301793f, 2.118334f, 7.282349f), PxVec3(0.209045f, -0.311980f, -0.926806f));

	Snippets::setupDefault("PhysX Snippet Frustum Query", sCamera, keyPress, renderCallback, exitCallback);

	initPhysics(true);
	glutMainLoop();
}
#endif
