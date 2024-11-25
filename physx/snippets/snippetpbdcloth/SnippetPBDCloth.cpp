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
// NOTE: Particle cloth has been DEPRECATED. Please use PxDeformableSurface instead.
// This snippet illustrates cloth simulation using position-based dynamics
// particle simulation. It creates a piece of cloth that drops onto a rotating
// sphere. 
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

static PxDefaultAllocator			gAllocator;
static PxDefaultErrorCallback		gErrorCallback;
static PxFoundation*				gFoundation			= NULL;
static PxPhysics*					gPhysics			= NULL;
static PxDefaultCpuDispatcher*		gDispatcher			= NULL;
static PxCudaContextManager*		gCudaContextManager	= NULL;
static PxScene*						gScene				= NULL;
static PxMaterial*					gMaterial			= NULL;
static PxPvd*						gPvd				= NULL;
static PxPBDParticleSystem*			gParticleSystem		= NULL;
static PxParticleClothBuffer*		gClothBuffer		= NULL;
static bool							gIsRunning			= true;

PxRigidDynamic* sphere;

static void initObstacles()
{
	PxShape* shape = gPhysics->createShape(PxSphereGeometry(3.0f), *gMaterial);
	sphere = gPhysics->createRigidDynamic(PxTransform(PxVec3(0.f, 5.0f, 0.f)));
	sphere->attachShape(*shape);
	sphere->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
	gScene->addActor(*sphere);
	shape->release();
}

// -----------------------------------------------------------------------------------------------------------------
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
	sceneDesc.solverType = PxSolverType::eTGS;
	gScene = gPhysics->createScene(sceneDesc);
}

// -----------------------------------------------------------------------------------------------------------------
static PX_FORCE_INLINE PxU32 id(PxU32 x, PxU32 y, PxU32 numY)
{
	return x * numY + y;
}

static void initCloth(const PxU32 numX, const PxU32 numZ, const PxVec3& position = PxVec3(0, 0, 0), const PxReal particleSpacing = 0.2f, const PxReal totalClothMass = 10.f)
{
	PxCudaContextManager* cudaContextManager = gScene->getCudaContextManager();
	if (cudaContextManager == NULL)
		return;

	const PxU32 numParticles = numX * numZ;
	const PxU32 numSprings = (numX - 1) * (numZ - 1) * 4 + (numX - 1) + (numZ - 1);
	const PxU32 numTriangles = (numX - 1) * (numZ - 1) * 2;

	const PxReal restOffset = particleSpacing;
	
	const PxReal stretchStiffness = 10000.f;
	const PxReal shearStiffness = 100.f;
	const PxReal springDamping = 0.001f;

	// Material setup
	PxPBDMaterial* defaultMat = gPhysics->createPBDMaterial(0.8f, 0.05f, 1e+6f, 0.001f, 0.5f, 0.005f, 0.05f, 0.f, 0.f);

	PxPBDParticleSystem *particleSystem = gPhysics->createPBDParticleSystem(*cudaContextManager);
	gParticleSystem = particleSystem;

	// General particle system setting
	
	const PxReal particleMass = totalClothMass / numParticles;
	particleSystem->setRestOffset(restOffset);
	particleSystem->setContactOffset(restOffset + 0.02f);
	particleSystem->setParticleContactOffset(restOffset + 0.02f);
	particleSystem->setSolidRestOffset(restOffset);
	particleSystem->setFluidRestOffset(0.0f);

	gScene->addActor(*particleSystem);

	// Create particles and add them to the particle system
	const PxU32 particlePhase = particleSystem->createPhase(defaultMat, PxParticlePhaseFlags(PxParticlePhaseFlag::eParticlePhaseSelfCollideFilter | PxParticlePhaseFlag::eParticlePhaseSelfCollide));

	PxParticleClothBufferHelper* clothBuffers = PxCreateParticleClothBufferHelper(1, numTriangles, numSprings, numParticles, cudaContextManager);

	PxU32* phase = PX_EXT_PINNED_MEMORY_ALLOC(PxU32, *cudaContextManager, numParticles);
	PxVec4* positionInvMass = PX_EXT_PINNED_MEMORY_ALLOC(PxVec4, *cudaContextManager, numParticles);
	PxVec4* velocity = PX_EXT_PINNED_MEMORY_ALLOC(PxVec4, *cudaContextManager, numParticles);
	
	PxReal x = position.x;
	PxReal y = position.y;
	PxReal z = position.z;

	// Define springs and triangles
	PxArray<PxParticleSpring> springs;
	springs.reserve(numSprings);
	PxArray<PxU32> triangles;
	triangles.reserve(numTriangles * 3);

	for (PxU32 i = 0; i < numX; ++i)
	{
		for (PxU32 j = 0; j < numZ; ++j)
		{
			const PxU32 index = i * numZ + j;

			PxVec4 pos(x, y, z, 1.0f / particleMass);
			phase[index] = particlePhase;
			positionInvMass[index] = pos;
			velocity[index] = PxVec4(0.0f);
				
			if (i > 0)
			{
				PxParticleSpring spring = { id(i - 1, j, numZ), id(i, j, numZ), particleSpacing, stretchStiffness, springDamping, 0 };
				springs.pushBack(spring);
			}
			if (j > 0)
			{
				PxParticleSpring spring = { id(i, j - 1, numZ), id(i, j, numZ), particleSpacing, stretchStiffness, springDamping, 0 };
				springs.pushBack(spring);
			}
			
			if (i > 0 && j > 0) 
			{
				PxParticleSpring spring0 = { id(i - 1, j - 1, numZ), id(i, j, numZ), PxSqrt(2.0f) * particleSpacing, shearStiffness, springDamping, 0 };
				springs.pushBack(spring0);
				PxParticleSpring spring1 = { id(i - 1, j, numZ), id(i, j - 1, numZ), PxSqrt(2.0f) * particleSpacing, shearStiffness, springDamping, 0 };
				springs.pushBack(spring1);

				//Triangles are used to compute approximated aerodynamic forces for cloth falling down
				triangles.pushBack(id(i - 1, j - 1, numZ));
				triangles.pushBack(id(i - 1, j, numZ));
				triangles.pushBack(id(i, j - 1, numZ));

				triangles.pushBack(id(i - 1, j, numZ));
				triangles.pushBack(id(i, j - 1, numZ));
				triangles.pushBack(id(i, j, numZ));
			}

			z += particleSpacing;
		}
		z = position.z;
		x += particleSpacing;
	}

	PX_ASSERT(numSprings == springs.size());
	PX_ASSERT(numTriangles == triangles.size()/3);
	
	clothBuffers->addCloth(0.0f, 0.0f, 0.0f, triangles.begin(), numTriangles, springs.begin(), numSprings, positionInvMass, numParticles);

	ExtGpu::PxParticleBufferDesc bufferDesc;
	bufferDesc.maxParticles = numParticles;
	bufferDesc.numActiveParticles = numParticles;
	bufferDesc.positions = positionInvMass;
	bufferDesc.velocities = velocity;
	bufferDesc.phases = phase;

	const PxParticleClothDesc& clothDesc = clothBuffers->getParticleClothDesc();
	PxParticleClothPreProcessor* clothPreProcessor = PxCreateParticleClothPreProcessor(cudaContextManager);

	PxPartitionedParticleCloth output;
	clothPreProcessor->partitionSprings(clothDesc, output);
	clothPreProcessor->release();

	gClothBuffer = physx::ExtGpu::PxCreateAndPopulateParticleClothBuffer(bufferDesc, clothDesc, output, cudaContextManager);
	gParticleSystem->addParticleBuffer(gClothBuffer);

	clothBuffers->release();

	PX_EXT_PINNED_MEMORY_FREE(*cudaContextManager, positionInvMass);
	PX_EXT_PINNED_MEMORY_FREE(*cudaContextManager, velocity);
	PX_EXT_PINNED_MEMORY_FREE(*cudaContextManager, phase);
}

PxPBDParticleSystem* getParticleSystem()
{
	return gParticleSystem;
}

PxParticleClothBuffer* getUserClothBuffer()
{
	return gClothBuffer;
}

// -----------------------------------------------------------------------------------------------------------------
void initPhysics(bool /*interactive*/)
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);

	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	gPvd->connect(*transport, PxPvdInstrumentationFlag::eALL);

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, gPvd);

	// initialize cuda
	PxCudaContextManagerDesc cudaContextManagerDesc;
	gCudaContextManager = PxCreateCudaContextManager(*gFoundation, cudaContextManagerDesc, PxGetProfilerCallback());
	if (gCudaContextManager && !gCudaContextManager->contextIsValid())
	{
		PX_RELEASE(gCudaContextManager);
		printf("Failed to initialize cuda context.\n");
		printf("The particle cloth feature is currently only supported on GPU.\n");
	}

	initScene();

	PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
	if (pvdClient)
	{
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}
	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);

	// Setup Cloth
	const PxReal totalClothMass = 10.0f;

	PxU32 numPointsX = 250;
	PxU32 numPointsZ = 250;
	PxReal particleSpacing = 0.05f;
	initCloth(numPointsX, numPointsZ, PxVec3(-0.5f*numPointsX*particleSpacing, 8.f, -0.5f*numPointsZ*particleSpacing), particleSpacing, totalClothMass);

	initObstacles();

	gScene->addActor(*PxCreatePlane(*gPhysics, PxPlane(0.f, 1.f, 0.f, 0.0f), *gMaterial));
	

	// Setup rigid bodies
	const PxReal boxSize = 1.0f;
	const PxReal boxMass = 1.0f;
	PxShape* shape = gPhysics->createShape(PxBoxGeometry(0.5f * boxSize, 0.5f * boxSize, 0.5f * boxSize), *gMaterial);
	for (int i = 0; i < 5; ++i)
	{
		PxRigidDynamic* body = gPhysics->createRigidDynamic(PxTransform(PxVec3(i - 3.0f, 10, 4.0f)));
		body->attachShape(*shape);
		PxRigidBodyExt::updateMassAndInertia(*body, boxMass);
		gScene->addActor(*body);
	}
	shape->release();
}

// ---------------------------------------------------
PxReal simTime = 0;
void stepPhysics(bool /*interactive*/)
{
	if (gIsRunning)
	{
		const PxReal dt = 1.0f / 60.0f;

		bool rotatingSphere = true;
		if (rotatingSphere)
		{
			const PxReal speed = 2.0f;
			PxTransform pose = sphere->getGlobalPose();			
			sphere->setKinematicTarget(PxTransform(pose.p, PxQuat(PxCos(simTime*speed), PxVec3(0,1,0))));
		}

		gScene->simulate(dt);
		gScene->fetchResults(true);
		gScene->fetchResultsParticleSystem();
		simTime += dt;
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
	
	printf("SnippetPBDCloth done.\n");
}

void keyPress(unsigned char key, const PxTransform& camera)
{
	(void)camera;

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
