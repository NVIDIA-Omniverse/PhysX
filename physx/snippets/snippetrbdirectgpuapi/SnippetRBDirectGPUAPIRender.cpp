// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifdef RENDER_SNIPPET

#include "PxPhysicsAPI.h"

#include "../snippetrender/SnippetRender.h"
#include "../snippetrender/SnippetCamera.h"

using namespace physx;

extern void initPhysics(bool interactive);
extern void stepPhysics(bool interactive);
extern void cleanupPhysics(bool interactive);
extern void keyPress(unsigned char key, const PxTransform& camera);

extern PxU32 getRBCount();
extern const PxGeometryHolder* getRBGeometries();
extern const PxTransform* getRBPoses();

namespace
{
	Snippets::Camera* sCamera;

	void renderCallback()
	{
		stepPhysics(true);

		Snippets::startRender(sCamera);

		if (getRBCount())
			Snippets::renderGeoms(getRBCount(), getRBGeometries(), getRBPoses(), true, PxVec3(1));

		Snippets::finishRender();
	}

	void cleanup()
	{
		delete sCamera;
		cleanupPhysics(true);
	}

	void exitCallback()
	{
	}
}

void renderLoop()
{
	sCamera = new Snippets::Camera(PxVec3(50.0f, 50.0f, 50.0f), PxVec3(-0.6f, -0.2f, -0.7f));

	Snippets::setupDefault("PhysX Snippet RBDirectGPUAPI", sCamera, keyPress, renderCallback, exitCallback);

	initPhysics(true);
	glutMainLoop();

	cleanup();
}
#endif
