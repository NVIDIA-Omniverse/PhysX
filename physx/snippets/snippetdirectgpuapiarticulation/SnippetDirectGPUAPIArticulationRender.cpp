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

using namespace physx;

extern void initPhysics(bool interactive);
extern void stepPhysics(bool interactive);
extern void cleanupPhysics(bool interactive);

extern PxU32 getRBCount();
extern const PxGeometryHolder* getRBGeometries();
extern const PxTransform* getRBPoses();

extern PxU32 getLinkCount();
extern const PxGeometryHolder* getLinkGeometries();
extern const PxTransform* getLinkPoses();

namespace
{
Snippets::Camera* sCamera;

void renderCallback()
{
	stepPhysics(true);

	Snippets::startRender(sCamera);

    const PxVec3 dynColor(1.0f, 0.5f, 0.25f);
    const PxVec3 rcaColor(0.6f*0.75f, 0.8f*0.75f, 1.0f*0.75f);

	if (getRBCount())
		Snippets::renderGeoms(getRBCount(), getRBGeometries(), getRBPoses(), true, dynColor);

	if (getLinkCount())
		Snippets::renderGeoms(getLinkCount(), getLinkGeometries(), getLinkPoses(), true, rcaColor);

	Snippets::finishRender();
}

void cleanup()
{
	delete sCamera;
    cleanupPhysics(true);
}

void exitCallback()
{
}
}

const PxVec3 gCamEyeLift(-5.858525f, 6.079476f, 1.546743f);
const PxVec3 gCamDirLift(0.927923f, -0.356565f, -0.108720f);

void renderLoop()
{
	sCamera = new Snippets::Camera(gCamEyeLift, gCamDirLift);

	Snippets::setupDefault("PhysX Snippet Direct GPU API Articulation", sCamera, NULL, renderCallback, exitCallback);

	initPhysics(true);
	glutMainLoop();

	cleanup();
}

#endif
