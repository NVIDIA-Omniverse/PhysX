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

#include "../snippetrender/SnippetRender.h"
#include "../snippetrender/SnippetCamera.h"
#include "../snippetutils/SnippetUtils.h"

using namespace physx;

extern void initPhysics(bool interactive);
extern void stepPhysics(bool interactive);	
extern void cleanupPhysics(bool interactive);
extern void keyPress(unsigned char key, const PxTransform& camera);
extern const PxGeometry& getTestGeometry();
extern void renderText();

namespace
{
Snippets::Camera* sCamera;

void renderCallback()
{
	stepPhysics(true);

	static float time = 0.0f;
	time += 0.003f;

	const PxQuat qx = PxGetRotXQuat(time);
	const PxQuat qy = PxGetRotYQuat(time*1.7f);
	const PxQuat qz = PxGetRotZQuat(time*1.33f);

	const PxTransform pose(PxVec3(0.0f), qx*qy*qz);

	Snippets::startRender(sCamera);

	const PxGeometry& geom = getTestGeometry();
	const PxU32 screenWidth = Snippets::getScreenWidth();
	const PxU32 screenHeight = Snippets::getScreenHeight();
	const PxVec3 camPos = sCamera->getEye();
	const PxVec3 camDir = sCamera->getDir();
	const PxU32	RAYTRACING_RENDER_WIDTH = 256;
	const PxU32	RAYTRACING_RENDER_HEIGHT = 256;

	const PxU32 textureWidth = RAYTRACING_RENDER_WIDTH;
	const PxU32 textureHeight = RAYTRACING_RENDER_HEIGHT;

	GLubyte* pixels = new GLubyte[textureWidth*textureHeight*4];

	const float fScreenWidth = float(screenWidth)/float(RAYTRACING_RENDER_WIDTH);
	const float fScreenHeight = float(screenHeight)/float(RAYTRACING_RENDER_HEIGHT);

	GLubyte* buffer = pixels;
	for(PxU32 j=0;j<RAYTRACING_RENDER_HEIGHT;j++)
	{
		const PxU32 yi = PxU32(fScreenHeight*float(j));
		for(PxU32 i=0;i<RAYTRACING_RENDER_WIDTH;i++)
		{
			const PxU32 xi = PxU32(fScreenWidth*float(i));
			const PxVec3 dir = Snippets::computeWorldRay(xi, yi, camDir);

			PxGeomRaycastHit hit;
			if(PxGeometryQuery::raycast(camPos, dir, geom, pose, 5000.0f, PxHitFlag::eDEFAULT, 1, &hit))
			{
				buffer[0] = 128+GLubyte(hit.normal.x*127.0f);
				buffer[1] = 128+GLubyte(hit.normal.y*127.0f);
				buffer[2] = 128+GLubyte(hit.normal.z*127.0f);
				buffer[3] = 255;
			}
			else
			{
				buffer[0] = 0;
				buffer[1] = 0;
				buffer[2] = 0;
				buffer[3] = 255;
			}

			buffer+=4;
		}
	}

	const GLuint texID = Snippets::CreateTexture(textureWidth, textureHeight, pixels, false);

	Snippets::DisplayTexture(texID, RAYTRACING_RENDER_WIDTH, 10);

	delete [] pixels;
	Snippets::ReleaseTexture(texID);


//	Snippets::DrawRectangle(0.0f, 1.0f, 0.0f, 1.0f, PxVec3(0.0f), PxVec3(1.0f), 1.0f, 768, 768, false, false);

//		PxVec3 camPos = sCamera->getEye();
//		PxVec3 camDir = sCamera->getDir();
//		printf("camPos: (%ff, %ff, %ff)\n", camPos.x, camPos.y, camPos.z);
//		printf("camDir: (%ff, %ff, %ff)\n", camDir.x, camDir.y, camDir.z);
	
	const PxVec3 color(1.0f, 0.5f, 0.25f);
	const PxGeometryHolder gh(geom);
	Snippets::renderGeoms(1, &gh, &pose, false, color);

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
	sCamera = new Snippets::Camera(PxVec3(-1.301793f, 2.118334f, 7.282349f), PxVec3(0.209045f, -0.311980f, -0.926806f));

	Snippets::setupDefault("PhysX Snippet GeometryQuery", sCamera, keyPress, renderCallback, exitCallback);

	initPhysics(true);
	glutMainLoop();
}
#endif
