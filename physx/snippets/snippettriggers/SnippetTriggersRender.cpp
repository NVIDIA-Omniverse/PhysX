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
bool isTrigger(const PxFilterData& data);
bool isTriggerShape(PxShape* shape);

namespace
{
Snippets::Camera* sCamera;

	class MyTriggerRender : public Snippets::TriggerRender
	{
		public:
		virtual	bool	isTrigger(physx::PxShape* shape)	const
		{
			return ::isTriggerShape(shape);
		}
	};

void renderCallback()
{
	stepPhysics(true);

	if(0)
	{
		PxVec3 camPos = sCamera->getEye();
		PxVec3 camDir = sCamera->getDir();
		printf("camPos: (%ff, %ff, %ff)\n", double(camPos.x), double(camPos.y), double(camPos.z));
		printf("camDir: (%ff, %ff, %ff)\n", double(camDir.x), double(camDir.y), double(camDir.z));
	}

	Snippets::startRender(sCamera);

	PxScene* scene;
	PxGetPhysics().getScenes(&scene,1);
	PxU32 nbActors = scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC);
	if(nbActors)
	{
		PxArray<PxRigidActor*> actors(nbActors);
		scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC, reinterpret_cast<PxActor**>(&actors[0]), nbActors);

		MyTriggerRender tr;
		Snippets::renderActors(&actors[0], static_cast<PxU32>(actors.size()), true, PxVec3(0.0f, 0.75f, 0.0f), &tr);
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
	sCamera = new Snippets::Camera(PxVec3(8.757190f, 12.367847f, 23.541956f), PxVec3(-0.407947f, -0.042438f, -0.912019f));

	Snippets::setupDefault("PhysX Snippet Triggers", sCamera, keyPress, renderCallback, exitCallback);

	initPhysics(true);
	glutMainLoop();
}
#endif
