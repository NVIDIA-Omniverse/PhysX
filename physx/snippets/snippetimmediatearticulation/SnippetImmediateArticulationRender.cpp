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

#include "PxPhysicsAPI.h"
#include "PxImmediateMode.h"

#include "../snippetrender/SnippetRender.h"
#include "../snippetrender/SnippetCamera.h"

using namespace physx;
using namespace immediate;

extern void initPhysics(bool interactive);
extern void stepPhysics(bool interactive);	
extern void cleanupPhysics(bool interactive);
extern void keyPress(unsigned char key, const PxTransform& camera);
extern PxU32 getNbGeoms();
extern const PxGeometryHolder* getGeoms();
extern const PxTransform* getGeomPoses();
extern PxU32 getNbContacts();
extern const PxContactPoint* getContacts();
extern PxU32 getNbArticulations();
extern PxArticulationHandle* getArticulations();
extern PxU32 getNbBounds();
extern const PxBounds3* getBounds();
extern void renderText();

namespace
{
Snippets::Camera* sCamera;

void renderCallback()
{
	stepPhysics(true);

/*	if(0)
	{
		PxVec3 camPos = sCamera->getEye();
		PxVec3 camDir = sCamera->getDir();
		printf("camPos: (%ff, %ff, %ff)\n", camPos.x, camPos.y, camPos.z);
		printf("camDir: (%ff, %ff, %ff)\n", camDir.x, camDir.y, camDir.z);
	}*/

	Snippets::startRender(sCamera);

	const PxVec3 color(0.6f, 0.8f, 1.0f);
//	const PxVec3 color(0.75f, 0.75f, 1.0f);
//	const PxVec3 color(1.0f);

	Snippets::renderGeoms(getNbGeoms(), getGeoms(), getGeomPoses(), true, color);

/*	PxU32 getNbGeoms();
	const PxGeometry* getGeoms();
	const PxTransform* getGeomPoses();
	Snippets::renderGeoms(getNbGeoms(), getGeoms(), getGeomPoses(), true, PxVec3(1.0f));*/

/*	PxBoxGeometry boxGeoms[10];
	for(PxU32 i=0;i<10;i++)
		boxGeoms[i].halfExtents = PxVec3(1.0f);

	PxTransform poses[10];
	for(PxU32 i=0;i<10;i++)
	{
		poses[i] = PxTransform(PxIdentity);
		poses[i].p.y += 1.5f;
		poses[i].p.x = float(i)*2.5f;
	}

	Snippets::renderGeoms(10, boxGeoms, poses, true, PxVec3(1.0f));*/

	if(1)
	{				
		const PxU32 nbContacts = getNbContacts();
		const PxContactPoint* contacts = getContacts();
		for(PxU32 j=0;j<nbContacts;j++)
		{
			Snippets::DrawFrame(contacts[j].point, 1.0f);
		}
	}

	if(0)
	{
		const PxU32 nbArticulations = getNbArticulations();
		PxArticulationHandle* articulations = getArticulations();
		for(PxU32 j=0;j<nbArticulations;j++)
		{
			immediate::PxArticulationLinkDerivedDataRC data[64];
			const PxU32 nbLinks = immediate::PxGetAllLinkData(articulations[j], data);
			for(PxU32 i=0;i<nbLinks;i++)
			{
				Snippets::DrawFrame(data[i].pose.p, 1.0f);
			}
		}
	}

	const PxBounds3* bounds = getBounds();
	const PxU32 nbBounds = getNbBounds();
	for(PxU32 i=0;i<nbBounds;i++)
	{
		Snippets::DrawBounds(bounds[i]);
	}

	renderText();

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
	sCamera = new Snippets::Camera(	PxVec3(8.526230f, 5.546278f, 5.448466f),
									PxVec3(-0.784231f, -0.210605f, -0.583632f));

	Snippets::setupDefault("PhysX Snippet Immediate Articulation", sCamera, keyPress, renderCallback, exitCallback);

	initPhysics(true);
	glutMainLoop();
}

#endif
