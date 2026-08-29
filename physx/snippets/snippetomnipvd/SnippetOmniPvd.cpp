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
// Copyright (c) 2008-2026 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.

#include <ctype.h>
#include <stdlib.h>
#include "PxPhysicsAPI.h"
#include "../snippetutils/SnippetUtils.h"
#include "omnipvd/PxOmniPvd.h"

#if PX_SUPPORT_OMNI_PVD
#include "../pvdruntime/include/OmniPvdWriter.h"
#include "../pvdruntime/include/OmniPvdFileWriteStream.h"
#include "../pvdruntime/include/OmniPvdSocketWriteStream.h"

using namespace physx;

static PxDefaultAllocator		gAllocator;
static PxDefaultErrorCallback	gErrorCallback;
static PxFoundation*			gFoundation = NULL;
static PxPhysics*				gPhysics = NULL;
static PxDefaultCpuDispatcher*	gDispatcher = NULL;
static PxScene*					gScene = NULL;
static PxMaterial*				gMaterial = NULL;

static PxOmniPvd*				gOmniPvd = NULL;
const char*						gOmniPvdPath = NULL;
// When no --omnipvdfile is given, the snippet streams the OmniPVD data live over
// TCP to a listening reader (for example the PVD viewer) instead of to a file.
// The producer is the client; the reader must already be listening on this
// address:port. Overridable with --omnipvdip / --omnipvdport.
static OmniPvdSocketWriteStream*	gOmniPvdSocketStream = NULL; // owned by gOmniPvd, released in cleanup
const char*						gOmniPvdHost = "127.0.0.1";
static PxU16					gOmniPvdPort = 5425;

static PxRigidDynamic* createDynamic(const PxTransform& t, const PxGeometry& geometry, const PxVec3& velocity = PxVec3(0))
{
	PxRigidDynamic* dynamic = PxCreateDynamic(*gPhysics, t, geometry, *gMaterial, 10.0f);
	dynamic->setAngularDamping(0.5f);
	dynamic->setLinearVelocity(velocity);
	gScene->addActor(*dynamic);
	return dynamic;
}

static void initPhysXScene()
{
	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
	gDispatcher = PxDefaultCpuDispatcherCreate(2);
	sceneDesc.cpuDispatcher = gDispatcher;
	sceneDesc.filterShader = PxDefaultSimulationFilterShader;
	gScene = gPhysics->createScene(sceneDesc);

	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);

	PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0, 1, 0, 0), *gMaterial);
	gScene->addActor(*groundPlane);
	createDynamic(PxTransform(PxVec3(0, 40, 100)), PxSphereGeometry(10), PxVec3(0, -50, -100));
}

void initPhysicsWithOmniPvd()
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
	if (!gFoundation)
	{
		printf("Error : could not create PxFoundation!\n");
		return;
	}

	gOmniPvd = PxCreateOmniPvd(*gFoundation);
	if (!gOmniPvd)
	{
		printf("Error : could not create PxOmniPvd!\n");
		return;
	}
	OmniPvdWriter* omniWriter = gOmniPvd->getWriter();
	if (!omniWriter)
	{
		printf("Error : could not get an instance of PxOmniPvdWriter!\n");
		return;
	}
	if (gOmniPvdPath)
	{
		// Record to a file (the well supported, platform-independent stream).
		OmniPvdFileWriteStream* fStream = gOmniPvd->getFileWriteStream();
		if (!fStream)
		{
			printf("Error : could not get an instance of PxOmniPvdFileWriteStream!\n");
			return;
		}
		fStream->setFileName(gOmniPvdPath);
		omniWriter->setWriteStream(static_cast<OmniPvdWriteStream&>(*fStream));
	}
	else
	{
		// No file given: stream live over TCP to a listening reader (e.g. the PVD
		// viewer). The producer is the client; createSocketWriteStream fixes the
		// endpoint and the caller owns the stream (released in cleanupPhysics).
		// Follow the same create -> open -> bind order as the read stream: open
		// (connect) the stream first, then bind it to the writer.
		gOmniPvdSocketStream = gOmniPvd->createSocketWriteStream(gOmniPvdHost, gOmniPvdPort);
		if (!gOmniPvdSocketStream)
		{
			printf("Error : could not create an OmniPvd socket write stream to %s:%d!\n", gOmniPvdHost, int(gOmniPvdPort));
			return;
		}
		if (!gOmniPvdSocketStream->openStream())
		{
			printf("Error : could not connect the OmniPvd socket write stream to %s:%d (is a reader listening?)\n", gOmniPvdHost, int(gOmniPvdPort));
			return;
		}
		omniWriter->setWriteStream(static_cast<OmniPvdWriteStream&>(*gOmniPvdSocketStream));
	}

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, NULL, gOmniPvd);
	if (!gPhysics)
	{
		printf("Error : could not create a PhysX instance!\n");
		return;
	}

	if (gPhysics->getOmniPvd())
	{
		if (!gPhysics->getOmniPvd()->startSampling())
		{
			printf("Error : could not start OmniPvd sampling\n");
		}
	}
	else
	{
		printf("Error : could not start OmniPvd sampling\n");
		return;
	}

	initPhysXScene();
}

void cleanupPhysics()
{
	PX_RELEASE(gScene);
	PX_RELEASE(gDispatcher);
	PX_RELEASE(gPhysics);
	// Close (symmetric with openStream) and release the caller-owned socket stream
	// before the OmniPvd instance.
	if (gOmniPvd && gOmniPvdSocketStream)
	{
		gOmniPvdSocketStream->closeStream();
		gOmniPvd->releaseSocketWriteStream(*gOmniPvdSocketStream);
		gOmniPvdSocketStream = NULL;
	}
	PX_RELEASE(gOmniPvd);
	PX_RELEASE(gFoundation);
}

bool parseOmniPvdOutputFile(int argc, const char *const* argv)
{
	// With --omnipvdfile the snippet records to that file; with no --omnipvdfile it
	// streams the OmniPVD data live over TCP to a listening reader (e.g. the PVD
	// viewer) at --omnipvdip:--omnipvdport (default 127.0.0.1:5425).
	for (int i = 1; i < argc; i++)
	{
		if (0 == strncmp(argv[i], "--omnipvdfile=", strlen("--omnipvdfile=")))
			gOmniPvdPath = argv[i] + strlen("--omnipvdfile=");
		else if (0 == strncmp(argv[i], "--omnipvdip=", strlen("--omnipvdip=")))
			gOmniPvdHost = argv[i] + strlen("--omnipvdip=");
		else if (0 == strncmp(argv[i], "--omnipvdport=", strlen("--omnipvdport=")))
		{
			const char* portStr = argv[i] + strlen("--omnipvdport=");
			char* end = NULL;
			unsigned long port = strtoul(portStr, &end, 10);
			if (end == portStr || *end != '\0' || port == 0 || port > 65535)
			{
				printf("SnippetOmniPvd: --omnipvdport must be an integer in [1, 65535], got '%s'\n", portStr);
				return false;
			}
			gOmniPvdPort = (PxU16)port;
		}
		else
		{
			printf("SnippetOmniPvd usage:\n"
				"SnippetOmniPvd [--omnipvdfile=<full path and fileName of the output OmniPvd file>]\n"
				"               [--omnipvdip=<listening reader IP, default 127.0.0.1>] [--omnipvdport=<reader port, default 5425>]\n"
				"With no --omnipvdfile the snippet streams live over TCP to a listening reader (for example the PVD viewer).\n");
			return false;
		}
	}
	return true;
}

void stepPhysics()
{
	gScene->simulate(1.0f / 60.0f);
	gScene->fetchResults(true);
}

void keyPress(unsigned char key, const PxTransform& camera)
{
	switch (toupper(key))
	{
	case ' ':	createDynamic(camera, PxSphereGeometry(3.0f), camera.rotate(PxVec3(0, 0, -1)) * 200);	break;
	}
}
#endif  // PX_SUPPORT_OMNI_PVD

int snippetMain(int argc, const char *const* argv)
{
#if PX_SUPPORT_OMNI_PVD
	if (!parseOmniPvdOutputFile(argc, argv))
	{ 
		return 1;
	}
#ifdef RENDER_SNIPPET
	extern void renderLoop();
	renderLoop();
#else
	initPhysicsWithOmniPvd();
	static const PxU32 frameCount = 100;
	for (PxU32 i = 0; i < frameCount; i++)
		stepPhysics();
	cleanupPhysics();
#endif
#else
	PX_UNUSED(argc);
	PX_UNUSED(argv);

	printf("PVD is not supported in release build configuration. Please use any of the other build configurations to run this snippet.\n");
#endif  // PX_SUPPORT_OMNI_PVD

	return 0;
}
