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

	Snippets::startRender(sCamera);

	PxScene* scene;
	PxGetPhysics().getScenes(&scene,1);
	PxU32 nbActors = scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC);
	if(nbActors)
	{
		const PxVec3 dynColor(1.0f, 0.5f, 0.25f);
		const PxVec3 kinematicColor(0.6f*0.5f, 0.8f*0.5f, 1.0f*0.5f);
		PxArray<PxRigidActor*> actors(nbActors);
		scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC, reinterpret_cast<PxActor**>(&actors[0]), nbActors);
		for(PxU32 i=0; i<nbActors; ++i)
		{
			PxRigidActor* actor = actors[i];
			PxRigidDynamic* dyn = actor->is<PxRigidDynamic>();
			if(dyn->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC)
				Snippets::renderActors(&actor, 1, true, kinematicColor, NULL, false);
			else
				Snippets::renderActors(&actor, 1, true, dynColor, NULL, false);
		}
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
	sCamera = new Snippets::Camera(PxVec3(34.110470f, 22.652895f, 19.877836f), PxVec3(-0.709846f, -0.583771f, -0.394120f));

	Snippets::setupDefault("PhysX Snippet Split Sim", sCamera, NULL, renderCallback, exitCallback);

	initPhysics(true);
	glutMainLoop();
}
#endif
