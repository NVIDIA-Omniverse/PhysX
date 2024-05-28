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
extern const PxGeometry& getTestGeometry();
extern PxU32 getNbPoints();
extern PxVec3 getPoint(PxU32 i);
extern void renderText();

namespace
{
Snippets::Camera* sCamera;

void renderCallback()
{
	stepPhysics(true);

	Snippets::startRender(sCamera);

//		PxVec3 camPos = sCamera->getEye();
//		PxVec3 camDir = sCamera->getDir();
//		printf("camPos: (%ff, %ff, %ff)\n", camPos.x, camPos.y, camPos.z);
//		printf("camDir: (%ff, %ff, %ff)\n", camDir.x, camDir.y, camDir.z);
	
	const PxVec3 color(1.0f, 0.5f, 0.25f);

	const PxGeometry& geom = getTestGeometry();
	const PxGeometryHolder gh(geom);

	static float time = 0.0f;
	time += 0.003f;

	const PxQuat qx = PxGetRotXQuat(time);
	const PxQuat qy = PxGetRotYQuat(time*1.7f);
	const PxQuat qz = PxGetRotZQuat(time*1.33f);

	const PxTransform pose(PxVec3(0.0f), qx*qy*qz);
	Snippets::renderGeoms(1, &gh, &pose, false, color);

	const PxVec3 lineColor(1.0f);

	const PxU32 nbQueries = getNbPoints();
	for(PxU32 i=0;i<nbQueries;i++)
	{
		const PxVec3 pt = getPoint(i);

		PxVec3 cp;
		float d2 = PxGeometryQuery::pointDistance(pt, geom, pose, &cp);
		(void)d2;

		Snippets::DrawLine(pt, cp, lineColor);
		Snippets::DrawFrame(pt, 0.1f);
		Snippets::DrawFrame(cp, 0.1f);
	}

	renderText();

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
	sCamera = new Snippets::Camera(PxVec3(-1.301793f, 2.118334f, 7.282349f), PxVec3(0.209045f, -0.311980f, -0.926806f));

	Snippets::setupDefault("PhysX Snippet PointDistanceQuery", sCamera, keyPress, renderCallback, exitCallback);

	initPhysics(true);
	glutMainLoop();
}
#endif
