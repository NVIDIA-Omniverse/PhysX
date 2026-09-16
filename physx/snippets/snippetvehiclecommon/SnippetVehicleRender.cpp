// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifdef RENDER_SNIPPET

#include "PxPhysicsAPI.h"

#include "../snippetrender/SnippetRender.h"
#include "../snippetrender/SnippetCamera.h"

using namespace physx;

extern bool initPhysics();
extern void stepPhysics();	
extern void cleanupPhysics();

extern PxScene* gScene;

namespace
{
Snippets::Camera* sCamera;

void renderCallback()
{
	stepPhysics();

	Snippets::startRender(sCamera);

	PxU32 nbActors = gScene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC);
	if(nbActors)
	{
		const PxVec3 dynColor(1.0f, 0.5f, 0.25f);

		PxArray<PxRigidActor*> actors(nbActors);
		gScene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC, reinterpret_cast<PxActor**>(&actors[0]), nbActors);
		Snippets::renderActors(&actors[0], static_cast<PxU32>(actors.size()), true, dynColor);
	}

	Snippets::finishRender();
}

void exitCallback()
{
	delete sCamera;
	cleanupPhysics();
}
}

void renderLoop(const char* name)
{
	sCamera = new Snippets::Camera(PxVec3(10.0f, 10.0f, 10.0f), PxVec3(-0.6f,-0.2f,-0.7f));

	Snippets::setupDefault(name, sCamera, NULL, renderCallback, exitCallback);

	if (initPhysics())
	{
		glutMainLoop();
	}
}

#endif
