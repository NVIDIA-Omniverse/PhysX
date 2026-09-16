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
extern PxArray<PxVec3> gContactSphereActorPositions;
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
		for(PxU32 i=0; i < gContactPositions.size(); i++)
		{
			gContactVertices.pushBack(gContactPositions[i]);
			gContactVertices.pushBack(gContactPositions[i]-gContactImpulses[i]*0.0001f);
		}
		glDisable(GL_LIGHTING);
		glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
		glEnableClientState(GL_VERTEX_ARRAY);
		glVertexPointer(3, GL_FLOAT, 0, &gContactVertices[0]);
		glDrawArrays(GL_LINES, 0, GLint(gContactVertices.size()));
		glDisableClientState(GL_VERTEX_ARRAY);
		glEnable(GL_LIGHTING);
	}

	if(gContactSphereActorPositions.size())
	{
		gContactVertices.clear();
		for(PxU32 i=0; i < gContactSphereActorPositions.size() - 1; i++)
		{
			gContactVertices.pushBack(gContactSphereActorPositions[i]);
			gContactVertices.pushBack(gContactSphereActorPositions[i+1]);
		}
		glDisable(GL_LIGHTING);
		glColor4f(1.0f, 1.0f, 0.0f, 1.0f);
		glEnableClientState(GL_VERTEX_ARRAY);
		glVertexPointer(3, GL_FLOAT, 0, &gContactVertices[0]);
		glDrawArrays(GL_LINES, 0, GLint(gContactVertices.size()));
		glDisableClientState(GL_VERTEX_ARRAY);
		glEnable(GL_LIGHTING);
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
	sCamera = new Snippets::Camera(PxVec3(-1.5f, 6.0f, 14.0f), PxVec3(-0.1f,0.0f,-0.7f));

	Snippets::setupDefault("PhysX Snippet ContactReport CCD", sCamera, NULL, renderCallback, exitCallback);

	initPhysics(true);
	glutMainLoop();
}
#endif
