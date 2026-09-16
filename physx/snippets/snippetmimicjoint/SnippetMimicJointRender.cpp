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
	const PxVec3 dynLinkColor(1.0f, 0.5f, 0.25f);
	const PxVec3 baseLinkColor(0.5f, 0.25f, 1.0f);

	PxScene* scene;
	PxGetPhysics().getScenes(&scene,1);

	PxU32 nbArticulations = scene->getNbArticulations();
	for(PxU32 i=0;i<nbArticulations;i++)
	{
		PxArticulationReducedCoordinate* articulation;
		scene->getArticulations(&articulation, 1, i);

		const PxU32 nbLinks = articulation->getNbLinks();
		PxArray<PxArticulationLink*> links(nbLinks);
		articulation->getLinks(&links[0], nbLinks);

		PxReal colorScale = 1.0f;
		if(articulation->isSleeping())
			colorScale = 0.4f;
		
		const PxU32 numLinks = static_cast<PxU32>(links.size());
		Snippets::renderActors(reinterpret_cast<PxRigidActor**>(&links[0]), 1, true, colorScale * baseLinkColor);
		Snippets::renderActors(reinterpret_cast<PxRigidActor**>(&links[1]), numLinks - 1, true, colorScale * dynLinkColor);
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
	const PxVec3 camEye(0.0f, 0.0f, 15.0f);
	const PxVec3 camDir(0.0f, 0.0f, -1.0f);

	sCamera = new Snippets::Camera(camEye, camDir);

	Snippets::setupDefault("PhysX Snippet Articulation Mimic Joint", sCamera, NULL, renderCallback, exitCallback);

	initPhysics(true);
	glutMainLoop();
}

#endif
