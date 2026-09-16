// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifdef RENDER_SNIPPET

#include "PxPhysicsAPI.h"

#include "../snippetrender/SnippetRender.h"
#include "../snippetrender/SnippetCamera.h"
#include "SnippetDeformableSurface.h"

using namespace physx;

extern void initPhysics(bool interactive);
extern void stepPhysics(bool interactive);	
extern void cleanupPhysics(bool interactive);
extern PxArray<TestSurface> gTestSurfaces;

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
	for (PxU32 i = 0; i < gTestSurfaces.size(); i++)
	{
		TestSurface* testSurface = &gTestSurfaces[i];
		PxTriangleMesh* mesh = testSurface->mTriangleMesh;
		Snippets::renderMesh(mesh->getNbVertices(), testSurface->mPositionsInvMass, mesh->getNbTriangles(), mesh->getTriangles(),
			mesh->getTriangleMeshFlags() & PxTriangleMeshFlag::e16_BIT_INDICES, rcaColor, NULL, false, true);
		Snippets::renderMesh(mesh->getNbVertices(), testSurface->mPositionsInvMass, mesh->getNbTriangles(), mesh->getTriangles(),
			mesh->getTriangleMeshFlags() & PxTriangleMeshFlag::e16_BIT_INDICES, rcaColor, NULL, true, true);
	}

	Snippets::showFPS();
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
	sCamera = new Snippets::Camera(PxVec3(15.0f, 10.0f, 15.0f), PxVec3(-0.6f, -0.2f, -0.6f));

	Snippets::setupDefault("PhysX Snippet Deformable Surface", sCamera, NULL, renderCallback, exitCallback);

	initPhysics(true);
	Snippets::initFPS();
	glutMainLoop();

	cleanup();
}

#endif
