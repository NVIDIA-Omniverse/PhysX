// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifdef RENDER_SNIPPET

#include <vector>

#include "PxPhysicsAPI.h"
#include "../snippetrender/SnippetRender.h"
#include "../snippetrender/SnippetCamera.h"

using namespace physx;

extern void initPhysics(bool interactive);
extern void stepPhysics(bool interactive);	
extern void cleanupPhysics(bool interactive);

extern std::vector<PxVec3> gContactPositions;
extern std::vector<PxVec3> gContactImpulses;
extern std::vector<PxVec3> gContactSphereActorPositions;
std::vector<PxVec3> gContactVertices;

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
		std::vector<PxRigidActor*> actors(nbActors);
		scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC, reinterpret_cast<PxActor**>(&actors[0]), nbActors);
		Snippets::renderActors(&actors[0], static_cast<PxU32>(actors.size()), true);
	}

	if(gContactPositions.size())
	{
		gContactVertices.clear();
		for(PxU32 i=0; i < gContactPositions.size(); i++)
		{
			gContactVertices.push_back(gContactPositions[i]);
			gContactVertices.push_back(gContactPositions[i]-gContactImpulses[i]*0.0001f);
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
			gContactVertices.push_back(gContactSphereActorPositions[i]);
			gContactVertices.push_back(gContactSphereActorPositions[i+1]);
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

void exitCallback(void)
{
	delete sCamera;
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
