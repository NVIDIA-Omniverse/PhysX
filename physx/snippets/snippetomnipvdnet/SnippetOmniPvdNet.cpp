// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// ****************************************************************************
// SnippetOmniPvdNet shows live OmniPVD streaming over a TCP socket, with both ends in one
// process over loopback so it is a single runnable binary. A reader thread listens as the
// TCP server and decodes the incoming OVD command stream; the main thread builds a small
// simulation, runs it for a while with no recording, then connects a client socket write
// stream (createOmniPvdSocketWriteStream), binds it, and calls startSampling(). Because
// startSampling() records the current state of the already-built world before recording the
// changes that follow, this "late attach" produces a recording that is complete on its own.
// ****************************************************************************

#include <stdio.h>
#include "PxPhysicsAPI.h"
#include "foundation/PxThread.h"
#include "../snippetutils/SnippetUtils.h"
#include "omnipvd/PxOmniPvd.h"

#if PX_SUPPORT_OMNI_PVD
#include "../pvdruntime/include/OmniPvdWriter.h"
#include "../pvdruntime/include/OmniPvdSocketWriteStream.h"
// The reader side uses the pvdruntime API directly (no PhysX SDK), as a real ingester would.
#include "../pvdruntime/include/OmniPvdLibraryFunctions.h"
#include "../pvdruntime/include/OmniPvdReader.h"
#include "../pvdruntime/include/OmniPvdSocketReadStream.h"
#include "../pvdruntime/include/OmniPvdCommands.h"
#include "../pvdruntime/include/OmniPvdDefines.h"

using namespace physx;

static const PxU16 gPort = 5425;          // loopback TCP port the reader listens on
static const PxU32 gPrerollFrames = 30;   // frames simulated with no recording before the attach
static const PxU32 gStreamFrames = 120;   // frames streamed after the attach

static PxDefaultAllocator		gAllocator;
static PxDefaultErrorCallback	gErrorCallback;
static PxFoundation*			gFoundation = NULL;
static PxPhysics*				gPhysics = NULL;
static PxDefaultCpuDispatcher*	gDispatcher = NULL;
static PxScene*					gScene = NULL;
static PxMaterial*				gMaterial = NULL;
static PxOmniPvd*				gOmniPvd = NULL;
static OmniPvdSocketWriteStream*	gWriteStream = NULL; // caller-owned (createOmniPvdSocketWriteStream)
static PxThread*				gReaderThread = NULL;

// ---------------------------------------------------------------------------
// Reader side (TCP server): listen, accept the producer, decode the OVD stream.
// Runs on its own PxThread so it can listen while the producer connects below.
// ---------------------------------------------------------------------------
static void* readerThreadEntry(void* /*arg*/)
{
	OmniPvdReader* reader = createOmniPvdReader();
	// The read stream LISTENS as the TCP server on the given port; the producer is the client.
	OmniPvdSocketReadStream* readStream = createOmniPvdSocketReadStream(gPort);
	if (!reader || !readStream)
	{
		printf("[reader] Error : could not create the reader / socket read stream.\n");
		if (reader) destroyOmniPvdReader(*reader);
		if (readStream) destroyOmniPvdSocketReadStream(*readStream);
		return NULL;
	}

	printf("[reader] Listening for the producer on port %u ...\n", PxU32(gPort));
	fflush(stdout);
	// Open (server listen + accept + handshake validation; blocks until the producer connects),
	// then bind to the reader -- symmetric with the producer's openStream() + setWriteStream().
	if (!readStream->openStream())
	{
		printf("[reader] Error : could not listen / accept on port %u.\n", PxU32(gPort));
		destroyOmniPvdReader(*reader);
		destroyOmniPvdSocketReadStream(*readStream);
		return NULL;
	}
	reader->setReadStream(*readStream);

	OmniPvdVersionType major = 0, minor = 0, patch = 0;
	if (!reader->startReading(major, minor, patch))
	{
		printf("[reader] Error : handshake / OVD version check failed (incompatible or no producer).\n");
		destroyOmniPvdReader(*reader);
		readStream->closeStream();
		destroyOmniPvdSocketReadStream(*readStream);
		return NULL;
	}
	printf("[reader] Connected. OVD stream version %u.%u.%u. Decoding the live stream ...\n",
		PxU32(major), PxU32(minor), PxU32(patch));
	fflush(stdout);

	PxU64 total = 0, classes = 0, attributes = 0, objects = 0, frames = 0;
	OmniPvdCommand::Enum cmd;
	while ((cmd = reader->getNextCommand()) != OmniPvdCommand::eINVALID)
	{
		total++;
		switch (cmd)
		{
		case OmniPvdCommand::eREGISTER_CLASS:               classes++;    break;
		case OmniPvdCommand::eREGISTER_ATTRIBUTE:
		case OmniPvdCommand::eREGISTER_CLASS_ATTRIBUTE:     attributes++; break;
		case OmniPvdCommand::eCREATE_OBJECT:                objects++;    break;
		case OmniPvdCommand::eSTART_FRAME:                  frames++;     break;
		default: break;
		}
	}

	printf("[reader] Stream ended (producer disconnected). commands=%llu classes=%llu attributes=%llu objects=%llu frames=%llu\n",
		(unsigned long long)total, (unsigned long long)classes, (unsigned long long)attributes,
		(unsigned long long)objects, (unsigned long long)frames);
	fflush(stdout);

	// Destroy the reader before the stream it points at, and close the connection symmetrically.
	destroyOmniPvdReader(*reader);
	readStream->closeStream();
	destroyOmniPvdSocketReadStream(*readStream);
	return NULL;
}

static void createStack(const PxTransform& t, PxU32 size, PxReal halfExtent)
{
	PxShape* shape = gPhysics->createShape(PxBoxGeometry(halfExtent, halfExtent, halfExtent), *gMaterial);
	for (PxU32 i = 0; i < size; i++)
	{
		for (PxU32 j = 0; j < size - i; j++)
		{
			PxTransform localTm(PxVec3(PxReal(j * 2) - PxReal(size - i), PxReal(i * 2 + 1), 0.0f) * halfExtent);
			PxRigidDynamic* body = gPhysics->createRigidDynamic(t.transform(localTm));
			body->attachShape(*shape);
			PxRigidBodyExt::updateMassAndInertia(*body, 10.0f);
			gScene->addActor(*body);
		}
	}
	shape->release();
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
	gScene->addActor(*PxCreatePlane(*gPhysics, PxPlane(0.0f, 1.0f, 0.0f, 0.0f), *gMaterial));
	// Drop the stack from above the ground so the live recording shows the boxes falling and settling.
	createStack(PxTransform(PxVec3(0.0f, 20.0f, 0.0f)), 5, 2.0f);
}

static void stepPhysics()
{
	gScene->simulate(1.0f / 60.0f);
	gScene->fetchResults(true);
}

// Connect a client write stream to the listening reader, bind it, and start sampling.
// startSampling() records the current state of the already-built world, then the changes after it.
static bool attachAndStartSampling()
{
	gWriteStream = createOmniPvdSocketWriteStream("127.0.0.1", gPort, 3000);
	if (!gWriteStream)
	{
		printf("[producer] Error : socket write stream creation failed.\n");
		return false;
	}
	// openStream() makes the TCP connection to the listening reader (retrying internally until
	// it is accepting); pair it with closeStream() at teardown.
	if (!gWriteStream->openStream())
	{
		printf("[producer] Error : could not connect to the reader on port %u.\n", PxU32(gPort));
		return false;
	}
	gOmniPvd->getWriter()->setWriteStream(*gWriteStream);
	if (!gPhysics->getOmniPvd()->startSampling())
	{
		printf("[producer] Error : startSampling failed (stream error).\n");
		return false;
	}
	return true;
}

// Create PhysX with an OmniPvd bound up front (no stream bound, no sampling yet); recording
// does not begin until startSampling().
static bool createPhysics()
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
	if (!gFoundation) { printf("[producer] Error : could not create PxFoundation!\n"); return false; }
	gOmniPvd = PxCreateOmniPvd(*gFoundation);
	if (!gOmniPvd) { printf("[producer] Error : could not create PxOmniPvd!\n"); return false; }
	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, NULL, gOmniPvd);
	if (!gPhysics) { printf("[producer] Error : could not create a PhysX instance!\n"); return false; }
	return true;
}

static void cleanupPhysics()
{
	// Release the scene and physics first: their teardown still emits OVD commands to the bound
	// stream, including the deletes for every actor, so those are recorded too. We do not stop
	// sampling beforehand (stopSampling is optional and would otherwise hide the teardown). Then
	// destroy the OmniPvd (and its borrowing writer) before destroying the caller-owned stream.
	PX_RELEASE(gScene);
	PX_RELEASE(gDispatcher);
	PX_RELEASE(gPhysics);
	PX_RELEASE(gOmniPvd);
	if (gWriteStream)
	{
		gWriteStream->closeStream();
		destroyOmniPvdSocketWriteStream(*gWriteStream);
		gWriteStream = NULL;
	}
	if (gReaderThread)
	{
		// Closing the connected producer stream above gives the reader EOF. A transport failure before
		// the producer connects can leave the reader blocked in accept() here; a read-side timeout or
		// cancellation mechanism is separate scope from this lifecycle example.
		gReaderThread->waitForQuit();
		PX_DELETE(gReaderThread);
		gReaderThread = NULL;
	}
	// PxThread allocation/deallocation uses the foundation allocator, so release it after the thread.
	PX_RELEASE(gFoundation);
}
#endif // PX_SUPPORT_OMNI_PVD

int snippetMain(int /*argc*/, const char* const* /*argv*/)
{
#if PX_SUPPORT_OMNI_PVD
	// Create PhysX first (this can fail) so the reader thread is only started once we know the
	// producer will reach the connect step; otherwise a failed init would leave the reader
	// blocked in accept() and the join below would hang.
	if (!createPhysics())
	{
		cleanupPhysics();
		return 1;
	}

	// Start the reader (TCP server) on a background thread so it is listening before the producer
	// (this thread, the TCP client) connects to it over loopback. openStream() retries the
	// connect internally, so a brief startup race with accept() is fine.
	gReaderThread = PX_NEW(PxThread)(readerThreadEntry, NULL, "OvdReader");

	// Build a small world and simulate it with NO recording, so the attach below is a late attach
	// against an already-running simulation.
	initPhysXScene();
	for (PxU32 i = 0; i < gPrerollFrames; ++i)
		stepPhysics();

	// Connect, bind, and start sampling: startSampling() sends the current state of the
	// already-built world, then the changes that follow. Carry the outcome as the process exit code
	// so launchers and sample automation see a failed live-streaming run as a failure, not success.
	int exitStatus = 0;
	if (attachAndStartSampling())
	{
		printf("[producer] Late attach on port %u: current state sent, streaming %u frames.\n",
			PxU32(gPort), PxU32(gStreamFrames));
		fflush(stdout);
		for (PxU32 i = 0; i < gStreamFrames; ++i)
			stepPhysics();
		printf("[producer] Done streaming.\n");   // gated on a successful attach: only true if we streamed
		fflush(stdout);
	}
	else
	{
		// The attach failed, so nothing was ever recorded: the live-streaming feature this snippet
		// demonstrates did not work. Report it as a failure. If the transport failed before connecting,
		// cleanupPhysics() can remain blocked waiting for the reader's accept(); read-side timeout/cancel
		// support is deliberately left as separate scope.
		exitStatus = 1;
	}

	cleanupPhysics(); // closes the stream, joins the reader, then releases the foundation
	return exitStatus;
#else
	printf("OmniPVD is not supported in this build configuration. Use a non-release configuration on Windows or Linux.\n");
	return 0;
#endif
}
