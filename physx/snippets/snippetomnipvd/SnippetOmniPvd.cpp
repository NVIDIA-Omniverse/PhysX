// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include <ctype.h>
#include "PxPhysicsAPI.h"
#include "../snippetutils/SnippetUtils.h"
#include "omnipvd/PxOmniPvd.h"

#if PX_SUPPORT_OMNI_PVD
#include "../pvdruntime/include/OmniPvdWriter.h"
#include "../pvdruntime/include/OmniPvdFileWriteStream.h"

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
	OmniPvdFileWriteStream* fStream = gOmniPvd->getFileWriteStream();
	if (!fStream)
	{
		printf("Error : could not get an instance of PxOmniPvdFileWriteStream!\n");
		return;
	}
	fStream->setFileName(gOmniPvdPath);
	omniWriter->setWriteStream(static_cast<OmniPvdWriteStream&>(*fStream));

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
			printf("Error : could not start OmniPvd sampling to file(%s)\n", gOmniPvdPath);
		}
	}
	else
	{
		printf("Error : could not start OmniPvd sampling to file(%s)\n", gOmniPvdPath);
		return;
	}

	initPhysXScene();
}

void cleanupPhysics()
{
	PX_RELEASE(gScene);
	PX_RELEASE(gDispatcher);
	PX_RELEASE(gPhysics);
	PX_RELEASE(gOmniPvd);
	PX_RELEASE(gFoundation);	
}

bool parseOmniPvdOutputFile(int argc, const char *const* argv)
{
	if (argc != 2 || 0 != strncmp(argv[1], "--omnipvdfile", strlen("--omnipvdfile")))
	{
		printf("SnippetOmniPvd usage:\n"
			"SnippetOmniPvd "
			"[--omnipvdfile=<full path and fileName of the output OmniPvd file> ] \n");
		return false;
	}
	gOmniPvdPath = argv[1] + strlen("--omnipvdfile=");
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
