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
//	sCamera = new Snippets::Camera(PxVec3(5.330070f, 5.598980f, -7.194576f), PxVec3(-0.436862f, -0.552957f, 0.709500f));
//	sCamera = new Snippets::Camera(PxVec3(-2.840040f, 2.347654f, -6.658016f), PxVec3(0.361939f, -0.232584f, 0.902721f));
	sCamera = new Snippets::Camera(PxVec3(-6.085999f, 4.210001f, 4.160744f), PxVec3(0.694034f, -0.479131f, -0.537355f));

	Snippets::setupDefault("PhysX Snippet PathTracing", sCamera, keyPress, renderCallback, exitCallback);

	initPhysics(true);
	glutMainLoop();
}
#endif
