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

extern PxU32 getRBCount();
extern const PxGeometryHolder* getRBGeometries();
extern const PxTransform* getRBPoses();

extern PxU32 getLinkCount();
extern const PxGeometryHolder* getLinkGeometries();
extern const PxTransform* getLinkPoses();

namespace
{
Snippets::Camera* sCamera;

void renderCallback()
{
	stepPhysics(true);

	Snippets::startRender(sCamera);

    const PxVec3 dynColor(1.0f, 0.5f, 0.25f);
    const PxVec3 rcaColor(0.6f*0.75f, 0.8f*0.75f, 1.0f*0.75f);

	if (getRBCount())
		Snippets::renderGeoms(getRBCount(), getRBGeometries(), getRBPoses(), true, dynColor);

	if (getLinkCount())
		Snippets::renderGeoms(getLinkCount(), getLinkGeometries(), getLinkPoses(), true, rcaColor);

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

const PxVec3 gCamEyeLift(-5.858525f, 6.079476f, 1.546743f);
const PxVec3 gCamDirLift(0.927923f, -0.356565f, -0.108720f);

void renderLoop()
{
	sCamera = new Snippets::Camera(gCamEyeLift, gCamDirLift);

	Snippets::setupDefault("PhysX Snippet Direct GPU API Articulation", sCamera, NULL, renderCallback, exitCallback);

	initPhysics(true);
	glutMainLoop();

	cleanup();
}

#endif
