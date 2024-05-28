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
extern void keyPress(unsigned char key, const PxTransform& camera);
extern void renderText();

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
		const PxVec3 dynColor(1.0f, 0.5f, 0.25f);

		std::vector<PxRigidActor*> actors(nbActors);
		scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC, reinterpret_cast<PxActor**>(&actors[0]), nbActors);
		Snippets::renderActors(&actors[0], static_cast<PxU32>(actors.size()), true, dynColor);
	}

	{
		const PxRenderBuffer& renderBuffer = scene->getRenderBuffer();
		const PxDebugLine* lines = renderBuffer.getLines();
		const PxU32 nbLines = renderBuffer.getNbLines();
		for(PxU32 i=0;i<nbLines;i++)
		{
			const float b = float((lines[i].color0 & 0xff))/255.0f;
			const float g = float(((lines[i].color0>>8) & 0xff))/255.0f;
			const float r = float(((lines[i].color0>>16) & 0xff))/255.0f;

			Snippets::DrawLine(lines[i].pos0, lines[i].pos1, PxVec3(r, g, b));
		}
	}

	renderText();

	Snippets::finishRender();
}

void cleanup()
{
	delete sCamera;
	cleanupPhysics(true);
}

void exitCallback(void)
{
}
}

void renderLoop()
{
	sCamera = new Snippets::Camera(PxVec3(1.847750f, 2.8f, -15.241850f), PxVec3(-0.219386f, -0.289038f, -0.931841f));

	Snippets::setupDefault("PhysX Snippet Joint Drive", sCamera, keyPress, renderCallback, exitCallback);

	initPhysics(true);
	glutMainLoop();

	cleanup();
}
#endif
