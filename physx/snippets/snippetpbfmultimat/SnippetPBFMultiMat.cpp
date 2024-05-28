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

// ****************************************************************************
// This snippet illustrates fluid simulation using multiple materials. It 
// creates a container and drops a body of water. The dynamics of the fluid
// is computed using Position-based Fluid (PBF) which is a purely
// particle-based algorithm.
// ****************************************************************************

#include <ctype.h>
#include "PxPhysicsAPI.h"
#include "../snippetcommon/SnippetPrint.h"
#include "../snippetcommon/SnippetPVD.h"
#include "../snippetutils/SnippetUtils.h"
#include "extensions/PxParticleExt.h"
#include "extensions/PxCudaHelpersExt.h"

using namespace physx;
using namespace ExtGpu;

static PxDefaultAllocator		gAllocator;
static PxDefaultErrorCallback	gErrorCallback;
static PxFoundation*			gFoundation			= NULL;
static PxPhysics*				gPhysics			= NULL;
static PxDefaultCpuDispatcher*	gDispatcher			= NULL;
static PxCudaContextManager*	gCudaContextManager	= NULL;
static PxScene*					gScene				= NULL;
static PxMaterial*				gMaterial			= NULL;
static PxPvd*					gPvd				= NULL;
static PxPBDParticleSystem*		gParticleSystem		= NULL;
static PxParticleBuffer*		gParticleBuffer		= NULL;
static bool						gIsRunning			= true;

static void initScene()
{
	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
	gDispatcher = PxDefaultCpuDispatcherCreate(2);
	sceneDesc.cpuDispatcher = gDispatcher;
	sceneDesc.filterShader = PxDefaultSimulationFilterShader;
	sceneDesc.cudaContextManager = gCudaContextManager;
	sceneDesc.staticStructure = PxPruningStructureType::eDYNAMIC_AABB_TREE;
	sceneDesc.flags |= PxSceneFlag::eENABLE_PCM;
	sceneDesc.flags |= PxSceneFlag::eENABLE_GPU_DYNAMICS;
	sceneDesc.broadPhaseType = PxBroadPhaseType::eGPU;
	gScene = gPhysics->createScene(sceneDesc);
}

static void initParticles(const PxU32 numX, const PxU32 numY, const PxU32 numZ, const PxVec3& position = PxVec3(0, 0, 0), const PxReal particleSpacing = 0.2f, const PxReal fluidDensity = 1000.f)
{
	PxCudaContextManager* cudaContextManager = gScene->getCudaContextManager();
	if (cudaContextManager == NULL)
		return;

	const PxU32 maxParticles = numX * numY * numZ;
	gParticleSystem = gPhysics->createPBDParticleSystem(*cudaContextManager, 96);

	// General particle system setting
	const PxReal restOffset = 0.5f * particleSpacing / 0.6f;
	const PxReal solidRestOffset = restOffset;
	const PxReal fluidRestOffset = restOffset * 0.6f;
	const PxReal particleMass = fluidDensity * 1.333f * 3.14159f * particleSpacing * particleSpacing * particleSpacing;
	gParticleSystem->setRestOffset(restOffset);
	gParticleSystem->setContactOffset(restOffset + 0.01f);
	gParticleSystem->setParticleContactOffset(PxMax(solidRestOffset + 0.01f, fluidRestOffset / 0.6f));
	gParticleSystem->setSolidRestOffset(solidRestOffset);
	gParticleSystem->setFluidRestOffset(fluidRestOffset);
	gParticleSystem->setParticleFlag(PxParticleFlag::eENABLE_SPECULATIVE_CCD, false);
	
	gScene->addActor(*gParticleSystem);
	
	// Create particles and add them to the particle system

	PxU32* phase = PX_EXT_PINNED_MEMORY_ALLOC(PxU32, *cudaContextManager, maxParticles);
	PxVec4* positionInvMass = PX_EXT_PINNED_MEMORY_ALLOC(PxVec4, *cudaContextManager, maxParticles);
	PxVec4* velocity = PX_EXT_PINNED_MEMORY_ALLOC(PxVec4, *cudaContextManager, maxParticles);

	// We are applying different material parameters for each section
	const PxU32 maxMaterials = 3;
	PxU32 phases[maxMaterials];
	for (PxU32 i = 0; i < maxMaterials; ++i)
	{
		PxPBDMaterial* mat = gPhysics->createPBDMaterial(0.05f, i / (maxMaterials - 1.0f), 0.f, 10.002f* (i + 1), 0.5f, 0.005f * i, 0.01f, 0.f, 0.f);
		phases[i] = gParticleSystem->createPhase(mat, PxParticlePhaseFlags(PxParticlePhaseFlag::eParticlePhaseFluid | PxParticlePhaseFlag::eParticlePhaseSelfCollide));
	}

	PxReal x = position.x;
	PxReal y = position.y;
	PxReal z = position.z;

	for (PxU32 i = 0; i < numX; ++i)
	{
		for (PxU32 j = 0; j < numY; ++j)
		{
			for (PxU32 k = 0; k < numZ; ++k)
			{
				const PxU32 index = i * (numY * numZ) + j * numZ + k;
				const PxU16 matIndex = (PxU16)(i * maxMaterials / numX);
				const PxVec4 pos(x, y, z, 1.0f / particleMass);
				phase[index] = phases[matIndex];
				positionInvMass[index] = pos;
				velocity[index] = PxVec4(0.0f);

				z += particleSpacing;
			}
			z = position.z;
			y += particleSpacing;
		}
		y = position.y;
		x += particleSpacing;
	}

	ExtGpu::PxParticleBufferDesc bufferDesc;
	bufferDesc.maxParticles = maxParticles;
	bufferDesc.numActiveParticles = maxParticles;

	bufferDesc.positions = positionInvMass;
	bufferDesc.velocities = velocity;
	bufferDesc.phases = phase;

	gParticleBuffer = physx::ExtGpu::PxCreateAndPopulateParticleBuffer(bufferDesc, cudaContextManager);
	gParticleSystem->addParticleBuffer(gParticleBuffer);

	PX_EXT_PINNED_MEMORY_FREE(*cudaContextManager, positionInvMass);
	PX_EXT_PINNED_MEMORY_FREE(*cudaContextManager, velocity);
	PX_EXT_PINNED_MEMORY_FREE(*cudaContextManager, phase);
}

PxParticleSystem* getParticleSystem()
{
	return gParticleSystem;
}

PxParticleBuffer* getParticleBuffer()
{
	return gParticleBuffer;
}

void initPhysics(bool /*interactive*/)
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);

	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	gPvd->connect(*transport,PxPvdInstrumentationFlag::eALL);

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, gPvd);
	
	// initialize cuda
	PxCudaContextManagerDesc cudaContextManagerDesc;
	gCudaContextManager = PxCreateCudaContextManager(*gFoundation, cudaContextManagerDesc, PxGetProfilerCallback());
	if (gCudaContextManager && !gCudaContextManager->contextIsValid())
	{
		PX_RELEASE(gCudaContextManager);
		printf("Failed to initialize cuda context.\n");
		printf("The particle feature is currently only supported on GPU.\n");
	}

	initScene();

	PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
	if(pvdClient)
	{
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}
	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);

	// Setup PBF
	const PxReal fluidDensity = 1000.0f;
	initParticles(60, 80, 30, PxVec3(-2.5f, 3.f, -2.5f), 0.1f, fluidDensity);
	
	// Setup container
	gScene->addActor(*PxCreatePlane(*gPhysics, PxPlane(0.f, 1.f, 0.f, 0.f), *gMaterial));
	gScene->addActor(*PxCreatePlane(*gPhysics, PxPlane(1.f, 0.f, 0.f, 6.f), *gMaterial));
	gScene->addActor(*PxCreatePlane(*gPhysics, PxPlane(-1.f, 0.f, 0.f, 6.f), *gMaterial));
	gScene->addActor(*PxCreatePlane(*gPhysics, PxPlane(0.f, 0.f, 1.f, 6.f), *gMaterial));
	gScene->addActor(*PxCreatePlane(*gPhysics, PxPlane(0.f, 0.f, -1.f, 6.f), *gMaterial));

	// Setup rigid bodies
	const PxReal dynamicsDensity = fluidDensity * 0.2f;
	const PxReal boxSize = 1.0f;
	const PxReal boxMass = boxSize * boxSize * boxSize * dynamicsDensity;
	PxShape* shape = gPhysics->createShape(PxBoxGeometry(0.5f * boxSize, 0.5f * boxSize, 0.5f * boxSize), *gMaterial);
	for (int i = 0; i < 5; ++i)
	{
		PxRigidDynamic* body = gPhysics->createRigidDynamic(PxTransform(PxVec3(i - 3.0f, 10, 1)));
		body->attachShape(*shape);
		PxRigidBodyExt::updateMassAndInertia(*body, boxMass);
		gScene->addActor(*body);
	}
	shape->release();
}

void stepPhysics(bool /*interactive*/)
{
	if (gIsRunning)
	{
		gScene->simulate(1.0f / 60.0f);
		gScene->fetchResults(true);
		gScene->fetchResultsParticleSystem();
	}
}
	
void cleanupPhysics(bool /*interactive*/)
{
	PX_RELEASE(gScene);
	PX_RELEASE(gDispatcher);
	PX_RELEASE(gPhysics);
	PX_RELEASE(gCudaContextManager);
	if(gPvd)
	{
		PxPvdTransport* transport = gPvd->getTransport();
		PX_RELEASE(gPvd);
		PX_RELEASE(transport);
	}
	PX_RELEASE(gFoundation);
	
	printf("SnippetPBFMultiMat done.\n");
}

void keyPress(unsigned char key, const PxTransform& /*camera*/)
{
	switch(toupper(key))
	{
	case 'P':	gIsRunning = !gIsRunning;	break;
	}
}

int snippetMain(int, const char*const*)
{
#ifdef RENDER_SNIPPET
	extern void renderLoop();
	renderLoop();
#else
	static const PxU32 frameCount = 100;
	initPhysics(false);
	for(PxU32 i=0; i<frameCount; i++)
		stepPhysics(false);
	cleanupPhysics(false);
#endif

	return 0;
}
