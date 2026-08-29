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

// ****************************************************************************
// SnippetOmniPvdNet shows live OmniPVD streaming over a TCP socket, with both ends in one
// process over loopback so it is a single runnable binary. A reader thread listens as the
// TCP server and decodes the incoming OVD command stream; the main thread builds a small
// simulation, runs it for a while with no recording, then connects a client socket write
// stream (PxOmniPvd::createSocketWriteStream), binds it, and calls startSampling(). Because
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
#include "../pvdruntime/include/OmniPvdLoader.h"
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
static OmniPvdSocketWriteStream*	gWriteStream = NULL; // caller-owned (createSocketWriteStream)

// ---------------------------------------------------------------------------
// Reader side (TCP server): listen, accept the producer, decode the OVD stream.
// Runs on its own PxThread so it can listen while the producer connects below.
// ---------------------------------------------------------------------------
static void* readerThreadEntry(void* /*arg*/)
{
#if defined(_WIN64) || defined(_WIN32)
	const char* pvdLib = "PVDRuntime_64.dll";
#else
	const char* pvdLib = "libPVDRuntime_64.so";
#endif
	OmniPvdLoader loader;
	if (!loader.loadOmniPvd(pvdLib))
	{
		printf("[reader] Error : could not load %s\n", pvdLib);
		return NULL;
	}
	if (!loader.mCreateOmniPvdSocketReadStream || !loader.mDestroyOmniPvdSocketReadStream ||
		!loader.mCreateOmniPvdReader || !loader.mDestroyOmniPvdReader)
	{
		printf("[reader] Error : this PVDRuntime is missing the socket read stream and reader factories.\n");
		return NULL;
	}

	OmniPvdReader* reader = loader.mCreateOmniPvdReader();
	// The read stream LISTENS as the TCP server on the given port; the producer is the client.
	OmniPvdSocketReadStream* readStream = loader.mCreateOmniPvdSocketReadStream(gPort);
	if (!reader || !readStream)
	{
		printf("[reader] Error : could not create the reader / socket read stream.\n");
		if (reader) loader.mDestroyOmniPvdReader(*reader);
		if (readStream) loader.mDestroyOmniPvdSocketReadStream(*readStream);
		return NULL;
	}

	printf("[reader] Listening for the producer on port %u ...\n", PxU32(gPort));
	fflush(stdout);
	// Open (server listen + accept + handshake validation; blocks until the producer connects),
	// then bind to the reader -- symmetric with the producer's openStream() + setWriteStream().
	if (!readStream->openStream())
	{
		printf("[reader] Error : could not listen / accept on port %u.\n", PxU32(gPort));
		loader.mDestroyOmniPvdReader(*reader);
		loader.mDestroyOmniPvdSocketReadStream(*readStream);
		return NULL;
	}
	reader->setReadStream(*readStream);

	OmniPvdVersionType major = 0, minor = 0, patch = 0;
	if (!reader->startReading(major, minor, patch))
	{
		printf("[reader] Error : handshake / OVD version check failed (incompatible or no producer).\n");
		loader.mDestroyOmniPvdReader(*reader);
		readStream->closeStream();
		loader.mDestroyOmniPvdSocketReadStream(*readStream);
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
	loader.mDestroyOmniPvdReader(*reader);
	readStream->closeStream();
	loader.mDestroyOmniPvdSocketReadStream(*readStream);
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
	gWriteStream = gOmniPvd->createSocketWriteStream("127.0.0.1", gPort);
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
		gOmniPvd->releaseSocketWriteStream(*gWriteStream);
		gWriteStream = NULL;
		return false;
	}
	gOmniPvd->getWriter()->setWriteStream(*gWriteStream);
	if (!gPhysics->getOmniPvd()->startSampling())
	{
		printf("[producer] Error : startSampling failed (stream error).\n");
		gWriteStream->closeStream();
		gOmniPvd->releaseSocketWriteStream(*gWriteStream);
		gWriteStream = NULL;
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
	// release the caller-owned stream, then the OmniPvd. See releaseSocketWriteStream().
	PX_RELEASE(gScene);
	PX_RELEASE(gDispatcher);
	PX_RELEASE(gPhysics);
	if (gWriteStream)
	{
		gWriteStream->closeStream();
		gOmniPvd->releaseSocketWriteStream(*gWriteStream);
		gWriteStream = NULL;
	}
	PX_RELEASE(gOmniPvd);
	PX_RELEASE(gFoundation);
}

// If the attach failed before the producer ever connected, the reader is still blocked in its
// accept(). Make a throwaway connection (connect, then immediately close) so the reader's accept
// returns and the thread finishes, letting waitForQuit() return instead of hanging. Safe to call when
// the reader already connected or exited: the connect just retries briefly (the socket's bounded
// ~2 second connect retry) and then fails, and the probe stream is released either way.
static void unblockStrandedReader()
{
	OmniPvdSocketWriteStream* probe = gOmniPvd->createSocketWriteStream("127.0.0.1", gPort);
	if (probe)
	{
		if (probe->openStream())
			probe->closeStream();
		gOmniPvd->releaseSocketWriteStream(*probe);
	}
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
	PxThread readerThread(readerThreadEntry, NULL, "OvdReader");

	// Build a small world and simulate it with NO recording, so the attach below is a late attach
	// against an already-running simulation.
	initPhysXScene();
	for (PxU32 i = 0; i < gPrerollFrames; ++i)
		stepPhysics();

	// Connect, bind, and start sampling: startSampling() sends the current state of the
	// already-built world, then the changes that follow.
	if (attachAndStartSampling())
	{
		printf("[producer] Late attach on port %u: current state sent, streaming %u frames.\n",
			PxU32(gPort), PxU32(gStreamFrames));
		fflush(stdout);
		for (PxU32 i = 0; i < gStreamFrames; ++i)
			stepPhysics();
	}
	else
	{
		// The producer never established the stream, so the reader may still be blocked in accept().
		// Unblock it before the join so waitForQuit() does not hang.
		unblockStrandedReader();
	}

	printf("[producer] Done streaming.\n");
	fflush(stdout);
	cleanupPhysics();             // closes the stream -> the reader sees EOF and finishes
	readerThread.waitForQuit();   // join the reader thread
#else
	printf("OmniPVD is not supported in this build configuration. Use a non-release configuration on Windows or Linux.\n");
#endif
	return 0;
}
