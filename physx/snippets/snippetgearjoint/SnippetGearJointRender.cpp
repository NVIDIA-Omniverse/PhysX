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

namespace
{
Snippets::Camera* sCamera;

void renderCallback()
{
	stepPhysics(true);

/*	if(0)
	{
		PxVec3 camPos = sCamera->getEye();
		PxVec3 camDir = sCamera->getDir();
		printf("camPos: (%ff, %ff, %ff)\n", camPos.x, camPos.y, camPos.z);
		printf("camDir: (%ff, %ff, %ff)\n", camDir.x, camDir.y, camDir.z);
	}*/

	Snippets::startRender(sCamera);

	PxScene* scene;
	PxGetPhysics().getScenes(&scene,1);
	PxU32 nbActors = scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC);
	if(nbActors)
	{
		const PxVec3 dynColor(1.0f, 0.5f, 0.25f);

		PxArray<PxRigidActor*> actors(nbActors);
		scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC, reinterpret_cast<PxActor**>(&actors[0]), nbActors);
		Snippets::renderActors(&actors[0], static_cast<PxU32>(actors.size()), true, dynColor);
	}

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
	sCamera = new Snippets::Camera(PxVec3(7.401237f, 11.224086f, 15.145986f), PxVec3(-0.315603f, -0.041489f, -0.947984f));

	Snippets::setupDefault("PhysX Snippet GearJoint", sCamera, NULL, renderCallback, exitCallback);

	initPhysics(true);
	glutMainLoop();
}
#endif
