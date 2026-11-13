// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// ****************************************************************************
// This snippet illustrates how metronomes standing on a common platform
// synchronize through the mechanical coupling provided by the platform.
//
// Each row of metronomes is an articulation with a fixed base, a platform
// attached to the base, and one pendulum per metronome attached to the
// platform with a revolute joint. The platform of the front row is attached
// with a prismatic joint and slides freely along the row, like a board resting
// on two soda cans. The platform of the back row is attached with a fixed
// joint, like a board resting on a table.
//
// Each swinging pendulum pushes the platform back and forth, and the motion of
// the platform acts back on all pendulums of the row. In the front row this
// coupling pulls the metronomes into phase within a few dozen swings. The back
// row starts from the same phases, but its metronomes do not interact and keep
// drifting apart, because their tempos differ slightly.
//
// The escapement that keeps a real metronome going is modeled as in
// J. Pantaleone, "Synchronization of metronomes", Am. J. Phys. 70, 992 (2002).
// It is a joint torque that adds energy at small swing angles and removes it
// at large ones, so that each metronome settles on a stable amplitude. The
// torque is applied through PxArticulationCache::jointForce.
//
// The phase coherence displayed for each row is the magnitude of the average
// of exp(i*phase) over its metronomes. It is 1 when all metronomes swing in
// phase, and close to 0 when their phases are spread out.
// ****************************************************************************

#include <ctype.h>
#include "PxPhysicsAPI.h"
#include "../snippetcommon/SnippetPrint.h"
#include "../snippetcommon/SnippetPVD.h"
#include "../snippetutils/SnippetUtils.h"
#ifdef RENDER_SNIPPET
	#include "../snippetrender/SnippetRender.h"
#endif

using namespace physx;

static PxDefaultAllocator		gAllocator;
static PxDefaultErrorCallback	gErrorCallback;
static PxFoundation*			gFoundation	= NULL;
static PxPhysics*				gPhysics	= NULL;
static PxDefaultCpuDispatcher*	gDispatcher	= NULL;
static PxScene*					gScene		= NULL;
static PxMaterial*				gMaterial	= NULL;
static PxPvd*					gPvd		= NULL;
static PxConvexMesh*			gCaseMesh	= NULL;

static const PxReal	gGravity			= 9.81f;
static const PxReal	gTimeStep			= 1.0f/60.0f;
static const PxU32	gNbMetronomes		= 5;
static const PxReal	gSpacing			= 0.11f;	// Distance between neighboring metronomes, in meters.
static const PxReal	gBaseHeight			= 0.02f;	// Height of the rails or the table under the board.
static const PxReal	gBoardHalfThickness	= 0.006f;
static const PxReal	gBoardHalfDepth		= 0.07f;

// Mass of the platform including the metronome cases, in kilograms. A lighter platform couples the metronomes more strongly.
static const PxReal	gPlatformMass		= 0.25f;

// Maximum relative offset of the tempo weights from their nominal position. It makes the tempos differ by up to about 1.7 percent.
static const PxReal	gTempoSpread		= 0.01f;

// The swing amplitude converges to about twice gEscapementAngle, and gEscapementRate sets how fast it does.
static const PxReal	gEscapementAngle	= 0.45f;	// Radians.
static const PxReal	gEscapementRate		= 0.25f;	// 1/seconds.
static const PxReal	gInitialAmplitude	= 0.8f;		// Radians.

struct MetronomeRow
{
	PxArticulationReducedCoordinate*	articulation;
	PxArticulationCache*				cache;
	bool								platformCanSlide;
	PxU32								platformDof;
	PxReal								movingMass;						// Platform and pendulums.
	PxU32								pendulumDofs[gNbMetronomes];
	PxReal								inertias[gNbMetronomes];		// Moments of inertia about the pivots.
	PxReal								firstMoments[gNbMetronomes];	// Masses times the distances from the pivots down to the centers of mass.
	PxReal								frequencies[gNbMetronomes];		// Small-amplitude angular frequencies.
	PxReal								coherence;
};

static const PxU32				gNbRows = 2;
static MetronomeRow				gRows[gNbRows];
// Draws the tempo offsets, then the initial phases. With this seed the tempos differ by up to 1.5 percent, so that the
// metronomes of the back row drift apart within the first minute.
static SnippetUtils::BasicRandom	gRandom(19);
static PxReal					gTime		= 0.0f;
static bool						gPause		= false;
static bool						gOneFrame	= false;

// Case of a metronome, a truncated pyramid standing on its base.
static PxConvexMesh* createCaseMesh()
{
	const PxVec3 verts[] =
	{
		PxVec3(-0.04f, 0.0f, -0.03f), PxVec3(0.04f, 0.0f, -0.03f), PxVec3(0.04f, 0.0f, 0.03f), PxVec3(-0.04f, 0.0f, 0.03f),
		PxVec3(-0.015f, 0.17f, -0.015f), PxVec3(0.015f, 0.17f, -0.015f), PxVec3(0.015f, 0.17f, 0.015f), PxVec3(-0.015f, 0.17f, 0.015f)
	};

	PxConvexMeshDesc desc;
	desc.points.count	= 8;
	desc.points.stride	= sizeof(PxVec3);
	desc.points.data	= verts;
	desc.flags			= PxConvexFlag::eCOMPUTE_CONVEX;

	const PxCookingParams params(gPhysics->getTolerancesScale());
	return PxCreateConvexMesh(params, desc, gPhysics->getPhysicsInsertionCallback());
}

// Position of a metronome along the row, relative to the center of the board.
static PxReal getMetronomeX(PxU32 index)
{
	return gSpacing*(PxReal(index) - 0.5f*PxReal(gNbMetronomes - 1));
}

static void createRow(MetronomeRow& row, PxReal z, bool platformCanSlide, const PxReal* tempoOffsets)
{
	const PxReal boardHalfLength = getMetronomeX(gNbMetronomes - 1) + 0.08f;

	PxArticulationReducedCoordinate* articulation = gPhysics->createArticulationReducedCoordinate();
	articulation->setArticulationFlag(PxArticulationFlag::eFIX_BASE, true);
	articulation->setArticulationFlag(PxArticulationFlag::eDISABLE_SELF_COLLISION, true);

	// The base carries two rails under a sliding platform, or a table under a fixed one.
	PxArticulationLink* base = articulation->createLink(NULL, PxTransform(PxVec3(0.0f, 0.0f, z)));
	if(platformCanSlide)
	{
		const PxBoxGeometry rail(boardHalfLength + 0.1f, 0.5f*gBaseHeight, 0.006f);
		PxRigidActorExt::createExclusiveShape(*base, rail, *gMaterial)->setLocalPose(PxTransform(PxVec3(0.0f, 0.5f*gBaseHeight, -0.045f)));
		PxRigidActorExt::createExclusiveShape(*base, rail, *gMaterial)->setLocalPose(PxTransform(PxVec3(0.0f, 0.5f*gBaseHeight, 0.045f)));
	}
	else
	{
		const PxBoxGeometry table(boardHalfLength, 0.5f*gBaseHeight, gBoardHalfDepth);
		PxRigidActorExt::createExclusiveShape(*base, table, *gMaterial)->setLocalPose(PxTransform(PxVec3(0.0f, 0.5f*gBaseHeight, 0.0f)));
	}

	// The platform is the board and the metronome cases.
	const PxVec3 boardCenter(0.0f, gBaseHeight + gBoardHalfThickness, 0.0f);
	PxArticulationLink* platform = articulation->createLink(base, PxTransform(PxVec3(0.0f, 0.0f, z) + boardCenter));
	PxRigidActorExt::createExclusiveShape(*platform, PxBoxGeometry(boardHalfLength, gBoardHalfThickness, gBoardHalfDepth), *gMaterial);
	for(PxU32 i=0;i<gNbMetronomes;i++)
	{
		const PxVec3 casePos(getMetronomeX(i), gBoardHalfThickness, 0.0f);
		PxRigidActorExt::createExclusiveShape(*platform, PxConvexMeshGeometry(gCaseMesh), *gMaterial)->setLocalPose(PxTransform(casePos));
	}
	PxRigidBodyExt::setMassAndUpdateInertia(*platform, gPlatformMass);
	platform->setLinearDamping(0.0f);
	platform->setAngularDamping(0.0f);

	PxArticulationJointReducedCoordinate* platformJoint = platform->getInboundJoint();
	platformJoint->setParentPose(PxTransform(boardCenter));
	platformJoint->setChildPose(PxTransform(PxIdentity));
	if(platformCanSlide)
	{
		platformJoint->setJointType(PxArticulationJointType::ePRISMATIC);
		platformJoint->setMotion(PxArticulationAxis::eX, PxArticulationMotion::eFREE);

		// A soft spring keeps the platform centered, against a slow drift caused by integration errors. Its natural
		// frequency is about 0.1 Hz, far below the tempo, so the platform still moves freely at the tempo.
		platformJoint->setDriveParams(PxArticulationAxis::eX, PxArticulationDrive(0.3f, 0.0f, PxPerformanceEnvelope(PX_MAX_F32)));
	}
	else
	{
		platformJoint->setJointType(PxArticulationJointType::eFIX);
	}

	// Each pendulum swings in front of its case, about a pivot axis along z. It carries a brass counterweight below the
	// pivot and a steel tempo weight on an aluminum rod above it. Raising the tempo weight slows the metronome down.
	row.movingMass = gPlatformMass;
	for(PxU32 i=0;i<gNbMetronomes;i++)
	{
		const PxVec3 pivot(getMetronomeX(i), gBoardHalfThickness + 0.06f, 0.042f);
		PxArticulationLink* pendulum = articulation->createLink(platform, platform->getGlobalPose().transform(PxTransform(pivot)));
		PxRigidActorExt::createExclusiveShape(*pendulum, PxBoxGeometry(0.015f, 0.01f, 0.008f), *gMaterial)->setLocalPose(PxTransform(PxVec3(0.0f, -0.04f, 0.0f)));
		PxRigidActorExt::createExclusiveShape(*pendulum, PxBoxGeometry(0.001f, 0.11f, 0.001f), *gMaterial)->setLocalPose(PxTransform(PxVec3(0.0f, 0.07f, 0.0f)));
		PxRigidActorExt::createExclusiveShape(*pendulum, PxBoxGeometry(0.01f, 0.008f, 0.005f), *gMaterial)->setLocalPose(PxTransform(PxVec3(0.0f, 0.06f*(1.0f + tempoOffsets[i]), 0.0f)));
		const PxReal densities[] = { 8500.0f, 2700.0f, 7800.0f };	// Brass, aluminum and steel, in kg/m^3.
		PxRigidBodyExt::updateMassAndInertia(*pendulum, densities, 3);
		pendulum->setLinearDamping(0.0f);
		pendulum->setAngularDamping(0.0f);

		PxArticulationJointReducedCoordinate* joint = pendulum->getInboundJoint();
		joint->setJointType(PxArticulationJointType::eREVOLUTE);
		joint->setMotion(PxArticulationAxis::eSWING2, PxArticulationMotion::eFREE);
		joint->setParentPose(PxTransform(pivot));
		joint->setChildPose(PxTransform(PxIdentity));

		// Pendulum properties about the pivot, which is the origin of the link frame.
		const PxReal mass = pendulum->getMass();
		const PxTransform massFrame = pendulum->getCMassLocalPose();
		const PxVec3 axis = massFrame.q.rotateInv(PxVec3(0.0f, 0.0f, 1.0f));
		const PxVec3 com = massFrame.p;
		row.inertias[i] = pendulum->getMassSpaceInertiaTensor().dot(axis.multiply(axis)) + mass*(com.x*com.x + com.y*com.y);
		row.firstMoments[i] = -mass*com.y;
		row.frequencies[i] = PxSqrt(row.firstMoments[i]*gGravity/row.inertias[i]);
		row.movingMass += mass;
	}

	gScene->addArticulation(*articulation);

	// Cache indices of the joint DOFs, see the "Cache Indexing" section of the articulation documentation.
	// Links are returned in creation order: base, platform, pendulums. Each inbound joint has at most one DOF.
	PxArticulationLink* links[gNbMetronomes + 2];
	const PxU32 nbLinks = articulation->getLinks(links, gNbMetronomes + 2);
	PxU32 dofStarts[gNbMetronomes + 2];
	for(PxU32 i=0;i<nbLinks;i++)
		dofStarts[links[i]->getLinkIndex()] = links[i]->getInboundJointDof();
	PxU32 nbDofs = 0;
	for(PxU32 i=0;i<nbLinks;i++)
	{
		const PxU32 linkDofs = dofStarts[i];
		dofStarts[i] = nbDofs;
		nbDofs += linkDofs;
	}
	row.platformDof = dofStarts[platform->getLinkIndex()];
	for(PxU32 i=0;i<gNbMetronomes;i++)
		row.pendulumDofs[i] = dofStarts[links[i + 2]->getLinkIndex()];

	row.articulation = articulation;
	row.platformCanSlide = platformCanSlide;
	row.cache = articulation->createCache();
	PxMemZero(row.cache->jointForce, sizeof(PxReal)*nbDofs);
	row.coherence = 0.0f;
}

static void setPhases(MetronomeRow& row, const PxReal* phases)
{
	PxArticulationCache& cache = *row.cache;
	const PxU32 nbDofs = row.articulation->getDofs();
	PxMemZero(cache.jointPosition, sizeof(PxReal)*nbDofs);
	PxMemZero(cache.jointVelocity, sizeof(PxReal)*nbDofs);

	PxReal pendulumMomentum = 0.0f;
	for(PxU32 i=0;i<gNbMetronomes;i++)
	{
		const PxReal angle = gInitialAmplitude*PxCos(phases[i]);
		const PxReal angularVelocity = -gInitialAmplitude*row.frequencies[i]*PxSin(phases[i]);
		cache.jointPosition[row.pendulumDofs[i]] = angle;
		cache.jointVelocity[row.pendulumDofs[i]] = angularVelocity;
		pendulumMomentum += row.firstMoments[i]*PxCos(angle)*angularVelocity;
	}

	// The platform velocity cancels the horizontal momentum of the pendulums, so that the row as a whole starts at rest.
	if(row.platformCanSlide)
		cache.jointVelocity[row.platformDof] = -pendulumMomentum/row.movingMass;

	row.articulation->applyCache(cache, PxArticulationCacheFlag::ePOSITION | PxArticulationCacheFlag::eVELOCITY);
}

static PxReal computeCoherence(MetronomeRow& row)
{
	PxArticulationCache& cache = *row.cache;
	row.articulation->copyInternalStateToCache(cache, PxArticulationCacheFlag::ePOSITION | PxArticulationCacheFlag::eVELOCITY);

	PxReal sumCos = 0.0f;
	PxReal sumSin = 0.0f;
	for(PxU32 i=0;i<gNbMetronomes;i++)
	{
		const PxU32 dof = row.pendulumDofs[i];
		const PxReal phase = PxAtan2(-cache.jointVelocity[dof]/row.frequencies[i], cache.jointPosition[dof]);
		sumCos += PxCos(phase);
		sumSin += PxSin(phase);
	}
	return PxSqrt(sumCos*sumCos + sumSin*sumSin)/PxReal(gNbMetronomes);
}

static void restart()
{
	PxReal phases[gNbMetronomes];
	for(PxU32 i=0;i<gNbMetronomes;i++)
		phases[i] = gRandom.randomFloat32(0.0f, PxTwoPi);

	for(PxU32 i=0;i<gNbRows;i++)
	{
		setPhases(gRows[i], phases);
		gRows[i].coherence = computeCoherence(gRows[i]);
	}

	gTime = 0.0f;
}

// The torque of each metronome depends only on its own swing angle and velocity, and both rows use the same escapement
// law and parameters. The metronomes interact only through the motion of the platform.
static void applyEscapementTorques(MetronomeRow& row)
{
	PxArticulationCache& cache = *row.cache;
	row.articulation->copyInternalStateToCache(cache, PxArticulationCacheFlag::ePOSITION | PxArticulationCacheFlag::eVELOCITY);
	for(PxU32 i=0;i<gNbMetronomes;i++)
	{
		const PxU32 dof = row.pendulumDofs[i];
		const PxReal ratio = cache.jointPosition[dof]/gEscapementAngle;
		cache.jointForce[dof] = gEscapementRate*row.inertias[i]*(1.0f - ratio*ratio)*cache.jointVelocity[dof];
	}
	row.articulation->applyCache(cache, PxArticulationCacheFlag::eFORCE);
}

void renderText()
{
#ifdef RENDER_SNIPPET
	char buffer[256];
	Pxsnprintf(buffer, sizeof(buffer), "Time: %.1f s", double(gTime));
	Snippets::print(buffer);
	Pxsnprintf(buffer, sizeof(buffer), "Front row, sliding platform: phase coherence %.2f", double(gRows[0].coherence));
	Snippets::print(buffer);
	Pxsnprintf(buffer, sizeof(buffer), "Back row, fixed platform: phase coherence %.2f", double(gRows[1].coherence));
	Snippets::print(buffer);
	Snippets::print("Press R to restart from random phases, P to pause, O to step.");
#endif
}

void initPhysics(bool /*interactive*/)
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);

	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	gPvd->connect(*transport, PxPvdInstrumentationFlag::eALL);

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, gPvd);

	const PxU32 numCores = SnippetUtils::getNbPhysicalCores();
	gDispatcher = PxDefaultCpuDispatcherCreate(numCores == 0 ? 0 : numCores - 1);

	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity		= PxVec3(0.0f, -gGravity, 0.0f);
	sceneDesc.cpuDispatcher	= gDispatcher;
	sceneDesc.filterShader	= PxDefaultSimulationFilterShader;
	// The amplitude stated for gEscapementAngle assumes PGS. Under TGS with PxSceneFlag::eENABLE_EXTERNAL_FORCES_EVERY_ITERATION_TGS,
	// which is on by default, the pendulums gain energy and swing wider.
	sceneDesc.solverType	= PxSolverType::ePGS;
	gScene = gPhysics->createScene(sceneDesc);

	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.0f);
	gCaseMesh = createCaseMesh();

	// Both rows use the same tempos.
	PxReal tempoOffsets[gNbMetronomes];
	for(PxU32 i=0;i<gNbMetronomes;i++)
		tempoOffsets[i] = gRandom.randomFloat32(-gTempoSpread, gTempoSpread);

	createRow(gRows[0], 0.18f, true, tempoOffsets);
	createRow(gRows[1], -0.18f, false, tempoOffsets);

	restart();
}

void stepPhysics(bool /*interactive*/)
{
	if(gPause && !gOneFrame)
		return;
	gOneFrame = false;

	for(PxU32 i=0;i<gNbRows;i++)
		applyEscapementTorques(gRows[i]);

	gScene->simulate(gTimeStep);
	gScene->fetchResults(true);
	gTime += gTimeStep;

	for(PxU32 i=0;i<gNbRows;i++)
		gRows[i].coherence = computeCoherence(gRows[i]);
}

void cleanupPhysics(bool /*interactive*/)
{
	for(PxU32 i=0;i<gNbRows;i++)
	{
		PX_RELEASE(gRows[i].cache);
		PX_RELEASE(gRows[i].articulation);
	}
	PX_RELEASE(gCaseMesh);
	PX_RELEASE(gScene);
	PX_RELEASE(gDispatcher);
	PX_RELEASE(gPhysics);
	if(gPvd)
	{
		PxPvdTransport* transport = gPvd->getTransport();
		PX_RELEASE(gPvd);
		PX_RELEASE(transport);
	}
	PX_RELEASE(gFoundation);

	printf("SnippetMetronomes done.\n");
}

void keyPress(unsigned char key, const PxTransform& /*camera*/)
{
	switch(toupper(key))
	{
		case 'P':	gPause = !gPause;					break;
		case 'O':	gPause = true; gOneFrame = true;	break;
		case 'R':	restart();							break;
	}
}

int snippetMain(int, const char*const*)
{
#ifdef RENDER_SNIPPET
	extern void renderLoop();
	renderLoop();
#else
	initPhysics(false);
	const PxU32 framesPerReport = 300;
	for(PxU32 i=1; i<=12*framesPerReport; i++)
	{
		stepPhysics(false);
		if(i % framesPerReport == 0)
			printf("Time %4.1f s, phase coherence: sliding platform %.2f, fixed platform %.2f\n", double(gTime), double(gRows[0].coherence), double(gRows[1].coherence));
	}
	cleanupPhysics(false);
#endif

	return 0;
}
