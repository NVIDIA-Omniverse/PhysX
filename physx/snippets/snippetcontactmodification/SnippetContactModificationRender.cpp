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

extern PxArray<PxVec3> gContactPositions;
extern PxArray<PxVec3> gContactImpulses;
PxArray<PxVec3> gContactVertices;

namespace
{
Snippets::Camera* sCamera;

void renderCallback()
{
	stepPhysics(true);

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

	if(gContactPositions.size())
	{
		gContactVertices.clear();
		for(PxU32 i=0;i<gContactPositions.size();i++)
		{
			gContactVertices.pushBack(gContactPositions[i]);
			gContactVertices.pushBack(gContactPositions[i]+gContactImpulses[i]*0.1f);
		}
		glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
		glEnableClientState(GL_VERTEX_ARRAY);
		glVertexPointer(3, GL_FLOAT, 0, &gContactVertices[0]);
		glDrawArrays(GL_LINES, 0, GLint(gContactVertices.size()));
		glDisableClientState(GL_VERTEX_ARRAY);
	}

	Snippets::finishRender();
}

void exitCallback()
{
	delete sCamera;
	gContactVertices.reset();
	cleanupPhysics(true);
}
}

void renderLoop()
{
	sCamera = new Snippets::Camera(PxVec3(50.0f, 50.0f, 50.0f), PxVec3(-0.6f,-0.2f,-0.7f));

	Snippets::setupDefault("PhysX Snippet ContactReport", sCamera, NULL, renderCallback, exitCallback);

	initPhysics(true);
	glutMainLoop();
}
#endif
