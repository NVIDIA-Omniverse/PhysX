// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifdef RENDER_SNIPPET

#if PX_SUPPORT_OMNI_PVD

#include <stdio.h>
#include "PxPhysicsAPI.h"

#include "../snippetrender/SnippetRender.h"
#include "../snippetrender/SnippetCamera.h"

using namespace physx;

extern bool initPhysicsWithOmniPvd();
extern void stepPhysics();	
extern bool cleanupPhysics();
extern void keyPress(unsigned char key, const PxTransform& camera);

namespace
{
static Snippets::Camera* sCamera;

void renderCallback()
{
	stepPhysics();

	Snippets::startRender(sCamera);

	PxScene* scene;
	PxGetPhysics().getScenes(&scene,1);
	PxU32 nbActors = scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC);
	if(nbActors)
	{
		PxArray<PxRigidActor*> actors(nbActors);
		scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC, reinterpret_cast<PxActor**>(&actors[0]), nbActors);
		Snippets::renderActors(&actors[0], static_cast<PxU32>(actors.size()), true);
	}

	Snippets::finishRender();
}

void exitCallback()
{
	delete sCamera;
	sCamera = NULL;
	if (!cleanupPhysics())
		fprintf(stderr, "Error: could not finalize the OmniPvd output stream\n");
}
}

bool renderLoop()
{
	sCamera = new Snippets::Camera(PxVec3(50.0f, 50.0f, 50.0f), PxVec3(-0.6f,-0.2f,-0.7f));

	Snippets::setupDefault("PhysX Snippet OmniPvd", sCamera, keyPress, renderCallback, exitCallback);

	if (!initPhysicsWithOmniPvd())
	{
		exitCallback();
		return false;
	}

	glutMainLoop();
	return true;
}
#endif  // PX_SUPPORT_OMNI_PVD

#endif
