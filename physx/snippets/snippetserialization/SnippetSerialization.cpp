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
// Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

// ****************************************************************************
// This snippet illustrates the use of binary and xml serialization
//
// Note: RepX/Xml serialization has been DEPRECATED.
//
// It creates a chain of boxes and serializes them as two collections: 
// a collection with shared objects and a collection with actors and joints
// which can be instantiated multiple times.
//
// Then physics is setup based on the serialized data. The collection with the 
// actors and the joints is instantiated multiple times with different 
// transforms.
//
// Finally phyics is teared down again, including deallocation of memory
// occupied by deserialized objects (in the case of binary serialization).
//
// ****************************************************************************

#include "PxPhysicsAPI.h"
#include "foundation/PxMemory.h"
#include "../snippetutils/SnippetUtils.h"
#include "../snippetcommon/SnippetPrint.h"
#include "../snippetcommon/SnippetPVD.h"

using namespace physx;

static bool						gUseBinarySerialization = false;

static PxDefaultAllocator		gAllocator;
static PxDefaultErrorCallback	gErrorCallback;
static PxFoundation*			gFoundation = NULL;
static PxPhysics*				gPhysics	= NULL;
static PxDefaultCpuDispatcher*	gDispatcher = NULL;
static PxScene*					gScene		= NULL;
static PxPvd*					gPvd        = NULL;

#define MAX_MEMBLOCKS 10
static PxU8*					gMemBlocks[MAX_MEMBLOCKS];
static PxU32					gMemBlockCount = 0;

/**
Creates two example collections: 
- collection with actors and joints that can be instantiated multiple times in the scene
- collection with shared objects
*/
void createCollections(PxCollection*& sharedCollection, PxCollection*& actorCollection, PxSerializationRegistry& sr)
{
	PxMaterial* material = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);

	PxReal halfLength = 2.0f, height = 25.0f;
	PxVec3 offset(halfLength, 0, 0);
	PxRigidActor* prevActor = PxCreateStatic(*gPhysics, PxTransform(PxVec3(0,height,0)), PxSphereGeometry(halfLength), *material, PxTransform(offset));

	PxShape* shape = gPhysics->createShape(PxBoxGeometry(halfLength, 1.0f, 1.0f), *material);
	for(PxU32 i=1; i<8;i++)
	{
		PxTransform tm(PxVec3(PxReal(i*2)* halfLength, height, 0));	
		PxRigidDynamic* dynamic = gPhysics->createRigidDynamic(tm);
		dynamic->attachShape(*shape);
		PxRigidBodyExt::updateMassAndInertia(*dynamic, 10.0f);

		PxSphericalJointCreate(*gPhysics, prevActor, PxTransform(offset), dynamic, PxTransform(-offset));
		prevActor = dynamic;
	}
		
	sharedCollection = PxCreateCollection();		// collection for all the shared objects
	actorCollection = PxCreateCollection();			// collection for all the nonshared objects

	sharedCollection->add(*shape);
	PxSerialization::complete(*sharedCollection, sr);									// chases the pointer from shape to material, and adds it
	PxSerialization::createSerialObjectIds(*sharedCollection, PxSerialObjectId(77));	// arbitrary choice of base for references to shared objects

	actorCollection->add(*prevActor);
	PxSerialization::complete(*actorCollection, sr, sharedCollection, true);			// chases all pointers and recursively adds actors and joints
}

/**
Allocates 128 byte aligned memory block for binary serialized data 
Stores pointer to memory in gMemBlocks for later deallocation
*/
void* createAlignedBlock(PxU32 size)
{
	PX_ASSERT(gMemBlockCount < MAX_MEMBLOCKS);
	PxU8* baseAddr = static_cast<PxU8*>(malloc(size+PX_SERIAL_FILE_ALIGN-1));
	gMemBlocks[gMemBlockCount++] = baseAddr;
	void* alignedBlock = reinterpret_cast<void*>((size_t(baseAddr)+PX_SERIAL_FILE_ALIGN-1)&~(PX_SERIAL_FILE_ALIGN-1));
	return alignedBlock;
}

/**
Create objects, add them to collections and serialize the collections to the steams gSharedStream and gActorStream
This function doesn't setup the gPhysics global as the corresponding physics object is only used locally 
*/
void serializeObjects(PxOutputStream& sharedStream, PxOutputStream& actorStream)
{
	PxSerializationRegistry* sr = PxSerialization::createSerializationRegistry(*gPhysics);

	PxCollection* sharedCollection = NULL;
	PxCollection* actorCollection = NULL;
	createCollections(sharedCollection, actorCollection, *sr);

	// Alternatively to using PxDefaultMemoryOutputStream it would be possible to serialize to files using 
	// PxDefaultFileOutputStream or a similar implementation of PxOutputStream.
	if (gUseBinarySerialization)
	{
		PxSerialization::serializeCollectionToBinary(sharedStream, *sharedCollection, *sr);
		PxSerialization::serializeCollectionToBinary(actorStream, *actorCollection, *sr, sharedCollection);	
	}
	else
	{
		PxSerialization::serializeCollectionToXml(sharedStream, *sharedCollection, *sr);
		PxSerialization::serializeCollectionToXml(actorStream, *actorCollection, *sr, NULL, sharedCollection);	
	}

	actorCollection->release();
	sharedCollection->release();

	sr->release();
}

/**
Deserialize shared data and use resulting collection to deserialize and instance actor collections
*/
void deserializeObjects(PxInputData& sharedData, PxInputData& actorData, const PxCookingParams& params)
{
	PxSerializationRegistry* sr = PxSerialization::createSerializationRegistry(*gPhysics);

	PxCollection* sharedCollection = NULL;
	{
		if (gUseBinarySerialization)
		{
			void* alignedBlock = createAlignedBlock(sharedData.getLength());
			sharedData.read(alignedBlock, sharedData.getLength());
			sharedCollection = PxSerialization::createCollectionFromBinary(alignedBlock, *sr);
		}
		else
		{
			sharedCollection = PxSerialization::createCollectionFromXml(sharedData, params, *sr);
		}
	}

	// Deserialize collection and instantiate objects twice, each time with a different transform
	PxTransform transforms[2] = { PxTransform(PxVec3(-5.0f, 0.0f, 0.0f)), PxTransform(PxVec3(5.0f, 0.0f, 0.0f)) };
	
	for (PxU32 i = 0; i < 2; i++)
	{
		PxCollection* collection = NULL;

		// If the PxInputData actorData would refer to a file, it would be better to avoid reading from it twice.
		// This could be achieved by reading the file once to memory, and then working with copies.
		// This is particulary practical when using binary serialization, where the data can be directly 
		// converted to physics objects.
		actorData.seek(0);		

		if (gUseBinarySerialization)
		{
			void* alignedBlock = createAlignedBlock(actorData.getLength());
			actorData.read(alignedBlock, actorData.getLength());
			collection = PxSerialization::createCollectionFromBinary(alignedBlock, *sr, sharedCollection);
		}
		else
		{
			collection = PxSerialization::createCollectionFromXml(actorData, params, *sr, sharedCollection);
		}

		for (PxU32 o = 0; o < collection->getNbObjects(); o++)
		{
			PxRigidActor* rigidActor = collection->getObject(o).is<PxRigidActor>();
			if (rigidActor)
			{
				PxTransform globalPose = rigidActor->getGlobalPose();
				globalPose = globalPose.transform(transforms[i]);
				rigidActor->setGlobalPose(globalPose);
			}
		}

		gScene->addCollection(*collection);
		collection->release();
	}
	sharedCollection->release();

	PxMaterial* material;
	gPhysics->getMaterials(&material,1);
	PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0,1,0,0), *material);
	gScene->addActor(*groundPlane);
	sr->release();
}

/**
Initializes physics and creates a scene
*/
void initPhysics()
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	gPvd->connect(*transport,PxPvdInstrumentationFlag::eALL);

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, gPvd);
	PxInitExtensions(*gPhysics, gPvd);
	
	PxU32 numCores = SnippetUtils::getNbPhysicalCores();
	gDispatcher = PxDefaultCpuDispatcherCreate(numCores == 0 ? 0 : numCores - 1);
	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0, -9.81f, 0);
	sceneDesc.cpuDispatcher = gDispatcher;
	sceneDesc.filterShader	= PxDefaultSimulationFilterShader;
	gScene = gPhysics->createScene(sceneDesc);

	PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
	if(pvdClient)
	{
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}
}

void stepPhysics()
{
	gScene->simulate(1.0f/60.0f);
	gScene->fetchResults(true);
}

/**
Releases all physics objects, including memory blocks containing deserialized data
*/
void cleanupPhysics()
{
	PX_RELEASE(gScene);
	PX_RELEASE(gDispatcher);
	PxCloseExtensions();

	PX_RELEASE(gPhysics);	// releases all objects	
	if(gPvd)
	{
		PxPvdTransport* transport = gPvd->getTransport();
		gPvd->release();	gPvd = NULL;
		PX_RELEASE(transport);
	}

	// Now that the objects have been released, it's safe to release the space they occupy
	for (PxU32 i = 0; i < gMemBlockCount; i++)
		free(gMemBlocks[i]);

	gMemBlockCount = 0;

	PX_RELEASE(gFoundation);
}

int snippetMain(int, const char*const*)
{
	initPhysics();
	// Alternatively PxDefaultFileOutputStream could be used 
	PxDefaultMemoryOutputStream sharedOutputStream;
	PxDefaultMemoryOutputStream actorOutputStream;
	serializeObjects(sharedOutputStream, actorOutputStream);
	cleanupPhysics();

	initPhysics();
	// Alternatively PxDefaultFileInputData could be used 
	PxDefaultMemoryInputData sharedInputStream(sharedOutputStream.getData(), sharedOutputStream.getSize());
	PxDefaultMemoryInputData actorInputStream(actorOutputStream.getData(), actorOutputStream.getSize());

	PxTolerancesScale scale;
	const PxCookingParams params(scale);

	deserializeObjects(sharedInputStream, actorInputStream, params);
#ifdef RENDER_SNIPPET
	extern void renderLoop();
	renderLoop();
#else
	static const PxU32 frameCount = 250;
	for(PxU32 i=0; i<frameCount; i++)
		stepPhysics();
	cleanupPhysics();
	printf("SnippetSerialization done.\n");
#endif

	return 0;
}
