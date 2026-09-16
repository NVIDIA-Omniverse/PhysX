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
extern void keyPress(unsigned char key, const PxTransform& camera);
extern void debugRender();

namespace
{
	Snippets::Camera* sCamera;

	void renderCallback()
	{
		stepPhysics(true);

		Snippets::startRender(sCamera);

		PxScene* scene;
		PxGetPhysics().getScenes(&scene, 1);
		PxU32 nbActors = scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC);
		if (nbActors)
		{
			const PxVec3 dynColor(1.0f, 0.5f, 0.25f);

			PxArray<PxRigidActor*> actors(nbActors);
			scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC, reinterpret_cast<PxActor**>(&actors[0]), nbActors);
			Snippets::renderActors(&actors[0], static_cast<PxU32>(actors.size()), true, dynColor);
		}

		debugRender();

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
	sCamera = new Snippets::Camera(PxVec3(30.0f, 30.0f, 30.0f), PxVec3(-0.6f, -0.5f, -0.6f));

	Snippets::setupDefault("PhysX Snippet CustomGeometryQueries", sCamera, keyPress, renderCallback, exitCallback);

	initPhysics(true);
	glutMainLoop();
}

static void PxVertex3f(const PxVec3& v) { ::glVertex3f(v.x, v.y, v.z); };
static void PxScalef(const PxVec3& v) { ::glScalef(v.x, v.y, v.z); };

void renderRaycast(const PxVec3& origin, const PxVec3& unitDir, float maxDist, const PxRaycastHit* hit)
{
	glDisable(GL_LIGHTING);
	if (hit)
	{
		glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
		glBegin(GL_LINES);
		PxVertex3f(origin);
		PxVertex3f(origin + unitDir * hit->distance);
		PxVertex3f(hit->position);
		PxVertex3f(hit->position + hit->normal);
		glEnd();
	}
	else
	{
		glColor4f(0.6f, 0.0f, 0.0f, 1.0f);
		glBegin(GL_LINES);
		PxVertex3f(origin);
		PxVertex3f(origin + unitDir * maxDist);
		glEnd();
	}
	glEnable(GL_LIGHTING);
}

void renderSweepBox(const PxVec3& origin, const PxVec3& unitDir, float maxDist, const PxVec3& halfExtents, const PxSweepHit* hit)
{
	glDisable(GL_LIGHTING);
	if (hit)
	{
		glColor4f(0.0f, 0.6f, 0.0f, 1.0f);
		glBegin(GL_LINES);
		PxVertex3f(origin);
		PxVertex3f(origin + unitDir * hit->distance);
		PxVertex3f(hit->position);
		PxVertex3f(hit->position + hit->normal);
		glEnd();
		PxTransform boxPose(origin + unitDir * hit->distance);
		PxMat44 boxMat(boxPose);
		glPushMatrix();
		glMultMatrixf(&boxMat.column0.x);
		PxScalef(halfExtents * 2);
		glutWireCube(1);
		glPopMatrix();
	}
	else
	{
		glColor4f(0.0f, 0.3f, 0.0f, 1.0f);
		glBegin(GL_LINES);
		PxVertex3f(origin);
		PxVertex3f(origin + unitDir * maxDist);
		glEnd();
	}
	glEnable(GL_LIGHTING);
}

void renderOverlapBox(const PxVec3& origin, const PxVec3& halfExtents, bool hit)
{
	glDisable(GL_LIGHTING);
	if (hit)
	{
		glColor4f(0.0f, 0.0f, 1.0f, 1.0f);
		PxTransform boxPose(origin);
		PxMat44 boxMat(boxPose);
		glPushMatrix();
		glMultMatrixf(&boxMat.column0.x);
		PxScalef(halfExtents * 2);
		glutWireCube(1);
		glPopMatrix();
	}
	else
	{
		glColor4f(0.0f, 0.0f, 0.6f, 1.0f);
		PxTransform boxPose(origin);
		PxMat44 boxMat(boxPose);
		glPushMatrix();
		glMultMatrixf(&boxMat.column0.x);
		PxScalef(halfExtents * 2);
		glutWireCube(1);
		glPopMatrix();
	}
	glEnable(GL_LIGHTING);
}

#endif
