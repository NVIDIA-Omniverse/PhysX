// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifdef RENDER_SNIPPET

#include <vector>

#include "PxPhysicsAPI.h"

#include "../snippetrender/SnippetRender.h"
#include "../snippetrender/SnippetCamera.h"
#include "SnippetDeformableSurfaceSkinning.h"

using namespace physx;

extern void initPhysics(bool interactive);
extern void stepPhysics(bool interactive);	
extern void cleanupPhysics(bool interactive);
extern PxArray<DeformableSurface> gDeformableSurfaces;
extern PxArray<SkinnedMesh> gSkinnedMeshes;
extern BasePostSolveCallback* gSkinning;

namespace
{
Snippets::Camera* sCamera;

void renderCallback()
{
	stepPhysics(true);

	Snippets::startRender(sCamera);

	const PxVec3 dynColor(1.0f, 0.5f, 0.25f);
	const PxVec3 rcaColor(0.6f*0.75f, 0.8f*0.75f, 1.0f*0.75f);
	const PxVec3 rcaColor2(0.8f * 0.75f, 0.8f * 0.75f, 0.5f * 0.75f);

	PxScene* scene;
	PxGetPhysics().getScenes(&scene,1);
	PxU32 nbActors = scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC);
	if(nbActors)
	{
		std::vector<PxRigidActor*> actors(nbActors);
		scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC, reinterpret_cast<PxActor**>(&actors[0]), nbActors);
		Snippets::renderActors(&actors[0], static_cast<PxU32>(actors.size()), true, dynColor);
	}

	gSkinning->synchronize();
	
	for (PxU32 i = 0; i < gDeformableSurfaces.size(); i++)
	{
		SkinnedMesh& skinnedMesh = gSkinnedMeshes[i];

		const PxVec3* skinnedVertices = gSkinning->getSkinnedVertices(i);

		Snippets::renderMesh(PxU32(skinnedMesh.mVertices.size()), skinnedVertices, PxU32(skinnedMesh.mTriangles.size()) / 3, &skinnedMesh.mTriangles[0],
			rcaColor, NULL, false);
		Snippets::renderMesh(PxU32(skinnedMesh.mVertices.size()), skinnedVertices, PxU32(skinnedMesh.mTriangles.size()) / 3, &skinnedMesh.mTriangles[0],
			rcaColor, NULL, true);

		DeformableSurface* c = &gDeformableSurfaces[i];
		Snippets::renderMesh(c->mTriangleMesh->getNbVertices(), c->mPositionsInvMass, c->mTriangleMesh->getNbTriangles(), c->mTriangleMesh->getTriangles(),
			c->mTriangleMesh->getTriangleMeshFlags() & PxTriangleMeshFlag::e16_BIT_INDICES, rcaColor2, NULL, false, true);
		Snippets::renderMesh(c->mTriangleMesh->getNbVertices(), c->mPositionsInvMass, c->mTriangleMesh->getNbTriangles(), c->mTriangleMesh->getTriangles(),
			c->mTriangleMesh->getTriangleMeshFlags() & PxTriangleMeshFlag::e16_BIT_INDICES, rcaColor2, NULL, true, true);
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

	Snippets::setupDefault("PhysX Snippet Deformable Surface Skinning", sCamera, NULL, renderCallback, exitCallback);

	initPhysics(true);
	Snippets::initFPS();
	glutMainLoop();

	cleanup();
}

#endif
