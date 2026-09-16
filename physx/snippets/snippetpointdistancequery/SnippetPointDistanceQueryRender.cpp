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
extern const PxGeometry& getTestGeometry();
extern PxU32 getNbPoints();
extern PxVec3 getPoint(PxU32 i);
extern void renderText();

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
	
	const PxVec3 color(1.0f, 0.5f, 0.25f);

	const PxGeometry& geom = getTestGeometry();
	const PxGeometryHolder gh(geom);

	static float time = 0.0f;
	time += 0.003f;

	const PxQuat qx = PxGetRotXQuat(time);
	const PxQuat qy = PxGetRotYQuat(time*1.7f);
	const PxQuat qz = PxGetRotZQuat(time*1.33f);

	const PxTransform pose(PxVec3(0.0f), qx*qy*qz);
	Snippets::renderGeoms(1, &gh, &pose, false, color);

	const PxVec3 lineColor(1.0f);

	const PxU32 nbQueries = getNbPoints();
	for(PxU32 i=0;i<nbQueries;i++)
	{
		const PxVec3 pt = getPoint(i);

		PxVec3 cp;
		float d2 = PxGeometryQuery::pointDistance(pt, geom, pose, &cp);
		(void)d2;

		Snippets::DrawLine(pt, cp, lineColor);
		Snippets::DrawFrame(pt, 0.1f);
		Snippets::DrawFrame(cp, 0.1f);
	}

	renderText();

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

	Snippets::setupDefault("PhysX Snippet PointDistanceQuery", sCamera, keyPress, renderCallback, exitCallback);

	initPhysics(true);
	glutMainLoop();
}
#endif
