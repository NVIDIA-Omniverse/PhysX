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
extern void renderText();

namespace
{
Snippets::Camera* sCamera;

void renderCallback()
{
	stepPhysics(true);

	Snippets::startRender(sCamera, 0.01f, 100.0f);

	const PxVec3 baseColor(0.45f, 0.45f, 0.45f);
	const PxVec3 platformColor(0.6f, 0.4f, 0.2f);
	const PxVec3 pendulumColor(0.85f, 0.75f, 0.4f);

	PxScene* scene;
	PxGetPhysics().getScenes(&scene, 1);
	const PxU32 nbArticulations = scene->getNbArticulations();
	for(PxU32 i=0;i<nbArticulations;i++)
	{
		PxArticulationReducedCoordinate* articulation;
		scene->getArticulations(&articulation, 1, i);

		const PxU32 nbLinks = articulation->getNbLinks();
		PxArray<PxArticulationLink*> links(nbLinks);
		articulation->getLinks(&links[0], nbLinks);

		// Links are returned in creation order: base, platform, pendulums.
		Snippets::renderActors(reinterpret_cast<PxRigidActor**>(&links[0]), 1, true, baseColor);
		Snippets::renderActors(reinterpret_cast<PxRigidActor**>(&links[1]), 1, true, platformColor);
		Snippets::renderActors(reinterpret_cast<PxRigidActor**>(&links[2]), nbLinks - 2, true, pendulumColor);
	}

	renderText();

	Snippets::finishRender();
}

void exitCallback()
{
}
}

void renderLoop()
{
	sCamera = new Snippets::Camera(PxVec3(0.0f, 0.55f, 0.75f), PxVec3(0.0f, -0.5f, -0.85f).getNormalized());
	sCamera->setSpeed(0.05f);

	Snippets::setupDefault("PhysX Snippet Metronomes", sCamera, keyPress, renderCallback, exitCallback);

	initPhysics(true);
	glutMainLoop();

	delete sCamera;
	cleanupPhysics(true);
}

#endif
