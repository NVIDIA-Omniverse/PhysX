// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifdef RENDER_SNIPPET

#include "PxPhysicsAPI.h"

#include "../snippetrender/SnippetRender.h"
#include "../snippetrender/SnippetCamera.h"
#include "SnippetDeformableVolume.h"

using namespace physx;

extern void initPhysics(bool interactive);
extern void stepPhysics(bool interactive);	
extern void cleanupPhysics(bool interactive);
extern PxArray<DeformableVolume> gDeformableVolumes;

namespace
{
Snippets::Camera* sCamera;

void renderCallback()
{
	stepPhysics(true);

	Snippets::startRender(sCamera);

	const PxVec3 dynColor(1.0f, 0.5f, 0.25f);
	const PxVec3 rcaColor(0.6f*0.75f, 0.8f*0.75f, 1.0f*0.75f);

	PxScene* scene;
	PxGetPhysics().getScenes(&scene,1);
	PxU32 nbActors = scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC);
	if(nbActors)
	{
		PxArray<PxRigidActor*> actors(nbActors);
		scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC, reinterpret_cast<PxActor**>(&actors[0]), nbActors);
		Snippets::renderActors(&actors[0], static_cast<PxU32>(actors.size()), true, dynColor);
	}
	for (PxU32 i = 0; i < gDeformableVolumes.size(); i++)
	{
		DeformableVolume* dv = &gDeformableVolumes[i];
		Snippets::renderDeformableVolume(dv->mDeformableVolume, dv->mPositionsInvMass, true, rcaColor);
	}

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
	sCamera = new Snippets::Camera(PxVec3(10.0f, 10.0f, 10.0f), PxVec3(-0.6f, -0.2f, -0.7f));

	Snippets::setupDefault("PhysX Snippet Deformable Volume", sCamera, NULL, renderCallback, exitCallback);

	initPhysics(true);
	glutMainLoop();

	cleanup();
}

#endif
