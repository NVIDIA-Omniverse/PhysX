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
// This snippet illustrates the use of simple contact reports and contact modification.
//
// It defines a filter shader function that requests contact modification and 
// touch reports for all pairs, and a contact callback function that saves 
// the contact points. It configures the scene to use this filter and callback, 
// and prints the number of contact reports each frame. If rendering, it renders 
// each contact as a line whose length and direction are defined by the contact 
// impulse.
// This test sets up a situation that would be unstable without contact modification
// due to very large mass ratios. This test uses local mass modification to make
// the configuration stable. It also demonstrates how to interpret contact impulses
// when local mass modification is used.
// Local mass modification can be disabled with the MODIFY_MASS_PROPERTIES #define 
// to demonstrate the instability if it was not used.
// 
// ****************************************************************************

#include <vector>
#include "PxPhysicsAPI.h"
#include "../snippetutils/SnippetUtils.h"
#include "../snippetcommon/SnippetPrint.h"
#include "../snippetcommon/SnippetPVD.h"

using namespace physx;

#define MODIFY_MASS_PROPERTIES 1

static PxDefaultAllocator		gAllocator;
static PxDefaultErrorCallback	gErrorCallback;
static PxFoundation*			gFoundation = NULL;
static PxPhysics*				gPhysics	= NULL;
static PxDefaultCpuDispatcher*	gDispatcher = NULL;
static PxScene*					gScene		= NULL;
static PxMaterial*				gMaterial	= NULL;
static PxPvd*					gPvd        = NULL;

std::vector<PxVec3> gContactPositions;
std::vector<PxVec3> gContactImpulses;
std::vector<PxVec3> gContactLinearImpulses[2];
std::vector<PxVec3> gContactAngularImpulses[2];

static PxFilterFlags contactReportFilterShader(	PxFilterObjectAttributes attributes0, PxFilterData filterData0, 
												PxFilterObjectAttributes attributes1, PxFilterData filterData1,
												PxPairFlags& pairFlags, const void* constantBlock, PxU32 constantBlockSize)
{
	PX_UNUSED(attributes0);
	PX_UNUSED(attributes1);
	PX_UNUSED(filterData0);
	PX_UNUSED(filterData1);
	PX_UNUSED(constantBlockSize);
	PX_UNUSED(constantBlock);

	// all initial and persisting reports for everything, with per-point data
	pairFlags = PxPairFlag::eSOLVE_CONTACT | PxPairFlag::eDETECT_DISCRETE_CONTACT
			  |	PxPairFlag::eNOTIFY_TOUCH_FOUND 
			  | PxPairFlag::eNOTIFY_TOUCH_PERSISTS
			  | PxPairFlag::eNOTIFY_CONTACT_POINTS
			  | PxPairFlag::eMODIFY_CONTACTS;
	return PxFilterFlag::eDEFAULT;
}

class ContactModifyCallback: public PxContactModifyCallback
{
	void onContactModify(PxContactModifyPair* const pairs, PxU32 count)
	{
#if MODIFY_MASS_PROPERTIES
		//We define a maximum mass ratio that we will accept in this test, which is a ratio of 2
		const PxReal maxMassRatio = 2.f;

		for(PxU32 i = 0; i < count; i++)
		{
			const PxRigidDynamic* dynamic0 = pairs[i].actor[0]->is<PxRigidDynamic>();
			const PxRigidDynamic* dynamic1 = pairs[i].actor[1]->is<PxRigidDynamic>();
			if(dynamic0 != NULL && dynamic1 != NULL)
			{
				//We only want to perform local mass modification between 2 dynamic bodies because we intend on 
				//normalizing the mass ratios between the pair within a tolerable range

				PxReal mass0 = dynamic0->getMass();
				PxReal mass1 = dynamic1->getMass();

				if(mass0 > mass1)
				{
					//dynamic0 is heavier than dynamic1 so we will locally increase the mass of dynamic1 
					//to be half the mass of dynamic0.
					PxReal ratio = mass0/mass1;
					if(ratio > maxMassRatio)
					{
						PxReal invMassScale = maxMassRatio/ratio;
						pairs[i].contacts.setInvMassScale1(invMassScale);
						pairs[i].contacts.setInvInertiaScale1(invMassScale);
					}
				}
				else
				{
					//dynamic1 is heavier than dynamic0 so we will locally increase the mass of dynamic0 
					//to be half the mass of dynamic1.
					PxReal ratio = mass1/mass0;
					if(ratio > maxMassRatio)
					{
						PxReal invMassScale = maxMassRatio/ratio;
						pairs[i].contacts.setInvMassScale0(invMassScale);
						pairs[i].contacts.setInvInertiaScale0(invMassScale);
					}
				}
			}
		}
#endif
	}
};

ContactModifyCallback gContactModifyCallback;

static PxU32 extractContactsWithMassScale(const PxContactPair& pair, PxContactPairPoint* userBuffer, PxU32 bufferSize, PxReal& invMassScale0, PxReal& invMassScale1)
{
	const PxU8* contactStream = pair.contactPoints;
	const PxU8* patchStream = pair.contactPatches;
	const PxU32* faceIndices = pair.getInternalFaceIndices();

	PxU32 nbContacts = 0;

	if(pair.contactCount && bufferSize)
	{
		PxContactStreamIterator iter(patchStream, contactStream, faceIndices, pair.patchCount, pair.contactCount);

		const PxReal* impulses = reinterpret_cast<const PxReal*>(pair.contactImpulses);

		PxU32 flippedContacts = (pair.flags & PxContactPairFlag::eINTERNAL_CONTACTS_ARE_FLIPPED);
		PxU32 hasImpulses = (pair.flags & PxContactPairFlag::eINTERNAL_HAS_IMPULSES);


		invMassScale0 = iter.getInvMassScale0();
		invMassScale1 = iter.getInvMassScale1();
		while(iter.hasNextPatch())
		{
			iter.nextPatch();
			while(iter.hasNextContact())
			{
				iter.nextContact();
				PxContactPairPoint& dst = userBuffer[nbContacts];
				dst.position = iter.getContactPoint();
				dst.separation = iter.getSeparation();
				dst.normal = iter.getContactNormal();
				if (!flippedContacts)
				{
					dst.internalFaceIndex0 = iter.getFaceIndex0();
					dst.internalFaceIndex1 = iter.getFaceIndex1();
				}
				else
				{
					dst.internalFaceIndex0 = iter.getFaceIndex1();
					dst.internalFaceIndex1 = iter.getFaceIndex0();
				}

				if (hasImpulses)
				{
					PxReal impulse = impulses[nbContacts];
					dst.impulse = dst.normal * impulse;
				}
				else
					dst.impulse = PxVec3(0.0f);
				++nbContacts;
				if(nbContacts == bufferSize)
					return nbContacts;
			}
		}
	}

	return nbContacts;
}

class ContactReportCallback: public PxSimulationEventCallback
{
	void onConstraintBreak(PxConstraintInfo* constraints, PxU32 count)	{ PX_UNUSED(constraints); PX_UNUSED(count); }
	void onWake(PxActor** actors, PxU32 count)							{ PX_UNUSED(actors); PX_UNUSED(count); }
	void onSleep(PxActor** actors, PxU32 count)							{ PX_UNUSED(actors); PX_UNUSED(count); }
	void onTrigger(PxTriggerPair* pairs, PxU32 count)					{ PX_UNUSED(pairs); PX_UNUSED(count); }
	void onAdvance(const PxRigidBody*const*, const PxTransform*, const PxU32) {}
	void onContact(const PxContactPairHeader& pairHeader, const PxContactPair* pairs, PxU32 nbPairs) 
	{
		PX_UNUSED((pairHeader));
		std::vector<PxContactPairPoint> contactPoints;
	

		for(PxU32 i=0;i<nbPairs;i++)
		{
			PxU32 contactCount = pairs[i].contactCount;
			if(contactCount)
			{
				contactPoints.resize(contactCount);
				PxReal invMassScale[2];
				extractContactsWithMassScale(pairs[i], &contactPoints[0], contactCount, invMassScale[0], invMassScale[1]);

				for(PxU32 j=0;j<contactCount;j++)
				{
					gContactPositions.push_back(contactPoints[j].position);
					//Push back reported contact impulses
					gContactImpulses.push_back(contactPoints[j].impulse);

					//Compute the effective linear/angular impulses for each body.
					//Note that the local mass scaling permits separate scales for invMass and invInertia.
					for(PxU32 k = 0; k < 2; ++k)
					{
						const PxRigidDynamic* dynamic = pairHeader.actors[k]->is<PxRigidDynamic>();
						PxVec3 linImpulse(0.f), angImpulse(0.f);
						if(dynamic != NULL)
						{
							PxRigidBodyExt::computeLinearAngularImpulse(*dynamic, dynamic->getGlobalPose(), contactPoints[j].position, 
								k == 0 ? contactPoints[j].impulse : -contactPoints[j].impulse, invMassScale[k], invMassScale[k], linImpulse, angImpulse);
						}
						gContactLinearImpulses[k].push_back(linImpulse);
						gContactAngularImpulses[k].push_back(angImpulse);
					}
				}
			}
		}
	}
};

static ContactReportCallback gContactReportCallback;

static void createStack(const PxTransform& t, PxU32 size, PxReal halfExtent)
{
	PxShape* shape = gPhysics->createShape(PxBoxGeometry(halfExtent, halfExtent, halfExtent), *gMaterial);
	for(PxU32 i=0; i<size;i++)
	{
		PxTransform localTm(PxVec3(0, PxReal(i*2+1), 0) * halfExtent);
		PxRigidDynamic* body = gPhysics->createRigidDynamic(t.transform(localTm));
		body->attachShape(*shape);
		PxRigidBodyExt::updateMassAndInertia(*body, (i+1)*(i+1)*(i+1)*10.0f);
		gScene->addActor(*body);
	}
	shape->release();
}

void initPhysics(bool /*interactive*/)
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
	sceneDesc.cpuDispatcher = gDispatcher;
	sceneDesc.gravity = PxVec3(0, -9.81f, 0);
	sceneDesc.filterShader	= contactReportFilterShader;			
	sceneDesc.simulationEventCallback = &gContactReportCallback;	
	sceneDesc.contactModifyCallback = &gContactModifyCallback;
	gScene = gPhysics->createScene(sceneDesc);

	PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
	if(pvdClient)
	{
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
	}

	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);

	PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0,1,0,0), *gMaterial);
	gScene->addActor(*groundPlane);
	createStack(PxTransform(PxVec3(0,0.0f,10.0f)), 5, 2.0f);
}

void stepPhysics(bool /*interactive*/)
{
	gContactPositions.clear();
	gContactImpulses.clear();

	gScene->simulate(1.0f/60.0f);
	gScene->fetchResults(true);
	printf("%d contact reports\n", PxU32(gContactPositions.size()));
}
	
void cleanupPhysics(bool /*interactive*/)
{
	PX_RELEASE(gScene);
	PX_RELEASE(gDispatcher);
	PxCloseExtensions();
	PX_RELEASE(gPhysics);
	if(gPvd)
	{
		PxPvdTransport* transport = gPvd->getTransport();
		PX_RELEASE(gPvd);
		PX_RELEASE(transport);
	}
	PX_RELEASE(gFoundation);
	
	printf("SnippetContactModification done.\n");
}

int snippetMain(int, const char*const*)
{
#ifdef RENDER_SNIPPET
	extern void renderLoop();
	renderLoop();
#else
	initPhysics(false);
	for(PxU32 i=0; i<250; i++)
		stepPhysics(false);
	cleanupPhysics(false);
#endif

	return 0;
}

