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

	PxU32 nbArticulations = scene->getNbArticulations();
	for(PxU32 i=0;i<nbArticulations;i++)
	{
		PxArticulationReducedCoordinate* articulation;
		scene->getArticulations(&articulation, 1, i);

		const PxU32 nbLinks = articulation->getNbLinks();
		PxArray<PxArticulationLink*> links(nbLinks);
		articulation->getLinks(&links[0], nbLinks);

		Snippets::renderActors(reinterpret_cast<PxRigidActor**>(&links[0]), static_cast<PxU32>(links.size()), true, rcaColor);
	}

	Snippets::finishRender();
}

void exitCallback()
{
	delete sCamera;
	cleanupPhysics(true);
}
}

//const PxVec3 gCamEyeLift(8.605188f, 4.050591f, 0.145860f);
//const PxVec3 gCamDirLift(-0.999581f, -0.026449f, 0.011790f);
const PxVec3 gCamEyeLift(-5.858525f, 6.079476f, 1.546743f);
const PxVec3 gCamDirLift(0.927923f, -0.356565f, -0.108720f);

void renderLoop()
{
	sCamera = new Snippets::Camera(gCamEyeLift, gCamDirLift);

	Snippets::setupDefault("PhysX Snippet RC Articulation", sCamera, NULL, renderCallback, exitCallback);

	initPhysics(true);
	glutMainLoop();
}

#endif
