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
		std::vector<PxArticulationLink*> links(nbLinks);
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

void exitCallback(void)
{
	delete sCamera;
	cleanupPhysics(true);
}
}

void renderLoop()
{
	const PxVec3 camEye(3.3f, 2.95f, 3.2f);
	const PxVec3 camDir(-0.5f, -0.078f, -0.86f);

	sCamera = new Snippets::Camera(camEye, camDir);

	Snippets::setupDefault("PhysX Snippet Articulation Fixed Tendon", sCamera, NULL, renderCallback, exitCallback);

	initPhysics(true);
	glutMainLoop();
}

#endif
