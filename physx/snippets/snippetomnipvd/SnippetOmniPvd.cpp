// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include "PxPhysicsAPI.h"
#include "../snippetutils/SnippetUtils.h"
#include "omnipvd/PxOmniPvd.h"

#if PX_SUPPORT_OMNI_PVD
#include "../pvdruntime/include/OmniPvdLibraryFunctions.h"
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
static OmniPvdFileWriteStream*	gOmniPvdFileStream = NULL;
static const char*				gOmniPvdPath = NULL;
// When no --omnipvdfile is given, the snippet streams the OmniPVD data live over
// TCP to a listening reader (for example the PVD viewer) instead of to a file.
// The producer is the client; the reader must already be listening on this
// address:port. Overridable with --omnipvdip / --omnipvdport.
static OmniPvdSocketWriteStream*	gOmniPvdSocketStream = NULL;
static const char*				gOmniPvdHost = "127.0.0.1";
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

bool initPhysicsWithOmniPvd()
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
	if (!gFoundation)
	{
		printf("Error : could not create PxFoundation!\n");
		return false;
	}

	gOmniPvd = PxCreateOmniPvd(*gFoundation);
	if (!gOmniPvd)
	{
		printf("Error : could not create PxOmniPvd!\n");
		return false;
	}
	OmniPvdWriter* omniWriter = gOmniPvd->getWriter();
	if (!omniWriter)
	{
		printf("Error : could not get an instance of PxOmniPvdWriter!\n");
		return false;
	}
	if (gOmniPvdPath)
	{
		// Record to a file (the well supported, platform-independent stream).
		gOmniPvdFileStream = createOmniPvdFileWriteStream();
		if (!gOmniPvdFileStream)
		{
			printf("Error : could not create an instance of PxOmniPvdFileWriteStream!\n");
			return false;
		}
		gOmniPvdFileStream->setFileName(gOmniPvdPath);
		// Validate the output path before binding; the writer's lazy open is idempotent.
		if (!gOmniPvdFileStream->openStream())
		{
			printf("Error: could not open OmniPvd output file stream for '%s'\n", gOmniPvdPath);
			return false;
		}
		omniWriter->setWriteStream(*gOmniPvdFileStream);
	}
	else
	{
		// No file given: stream live over TCP to a listening reader (e.g. the PVD
		// viewer). The producer is the client; createOmniPvdSocketWriteStream fixes
		// the endpoint and the caller owns the stream.
		// Follow the same create -> open -> bind order as the read stream: open
		// (connect) the stream first, then bind it to the writer.
		gOmniPvdSocketStream = createOmniPvdSocketWriteStream(gOmniPvdHost, gOmniPvdPort, 3000);
		if (!gOmniPvdSocketStream)
		{
			printf("Error : could not create an OmniPvd socket write stream to %s:%d!\n", gOmniPvdHost, int(gOmniPvdPort));
			return false;
		}
		if (!gOmniPvdSocketStream->openStream())
		{
			printf("Error : could not connect the OmniPvd socket write stream to %s:%d (is a reader listening?)\n", gOmniPvdHost, int(gOmniPvdPort));
			return false;
		}
		omniWriter->setWriteStream(*gOmniPvdSocketStream);
	}

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, NULL, gOmniPvd);
	if (!gPhysics)
	{
		printf("Error : could not create a PhysX instance!\n");
		return false;
	}

	if (gPhysics->getOmniPvd())
	{
		if (!gPhysics->getOmniPvd()->startSampling())
		{
			printf("Error : could not start OmniPvd sampling\n");
			return false;
		}
	}
	else
	{
		printf("Error : could not start OmniPvd sampling\n");
		return false;
	}

	initPhysXScene();
	return true;
}

bool cleanupPhysics()
{
	bool streamClosed = true;
	PX_RELEASE(gScene);
	PX_RELEASE(gDispatcher);
	PX_RELEASE(gPhysics);
	PX_RELEASE(gOmniPvd);
	if (gOmniPvdFileStream)
	{
		streamClosed = gOmniPvdFileStream->closeStream();
		destroyOmniPvdFileWriteStream(*gOmniPvdFileStream);
		gOmniPvdFileStream = NULL;
	}
	// Close (symmetric with openStream) and destroy the caller-owned socket stream.
	else if (gOmniPvdSocketStream)
	{
		streamClosed = gOmniPvdSocketStream->closeStream();
		destroyOmniPvdSocketWriteStream(*gOmniPvdSocketStream);
		gOmniPvdSocketStream = NULL;
	}
	PX_RELEASE(gFoundation);
	return streamClosed;
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
	extern bool renderLoop();
	if (!renderLoop())
	{
		return 1;
	}
#else
	if (!initPhysicsWithOmniPvd())
	{
		cleanupPhysics();
		return 1;
	}
	static const PxU32 frameCount = 100;
	for (PxU32 i = 0; i < frameCount; i++)
		stepPhysics();
	if (!cleanupPhysics())
	{
		fprintf(stderr, "Error: could not finalize the OmniPvd output stream\n");
		return 1;
	}
#endif
#else
	PX_UNUSED(argc);
	PX_UNUSED(argv);

	printf("PVD is not supported in release build configuration. Please use any of the other build configurations to run this snippet.\n");
#endif  // PX_SUPPORT_OMNI_PVD

	return 0;
}
