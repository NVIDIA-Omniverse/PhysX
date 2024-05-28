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
// This snippet illustrates the use of built-in triggers, and how to emulate
// them with regular shapes if you need CCD or trigger-trigger notifications.
// 
// ****************************************************************************

#include "PxPhysicsAPI.h"
#include "../snippetutils/SnippetUtils.h"
#include "../snippetcommon/SnippetPrint.h"
#include "../snippetcommon/SnippetPVD.h"

using namespace physx;

enum TriggerImpl
{
	// Uses built-in triggers (PxShapeFlag::eTRIGGER_SHAPE).
	REAL_TRIGGERS,

	// Emulates triggers using a filter shader. Needs one reserved value in PxFilterData.
	FILTER_SHADER,

	// Emulates triggers using a filter callback. Doesn't use PxFilterData but needs user-defined way to mark a shape as a trigger.
	FILTER_CALLBACK,
};

struct ScenarioData
{
	TriggerImpl	mImpl;
	bool		mCCD;
	bool		mTriggerTrigger;
};

#define SCENARIO_COUNT	9

static ScenarioData gData[SCENARIO_COUNT] = {
	{REAL_TRIGGERS,		false,	false},
	{FILTER_SHADER,		false,	false},
	{FILTER_CALLBACK,	false,	false},
	{REAL_TRIGGERS,		true,	false},
	{FILTER_SHADER,		true,	false},
	{FILTER_CALLBACK,	true,	false},
	{REAL_TRIGGERS,		false,	true},
	{FILTER_SHADER,		false,	true},
	{FILTER_CALLBACK,	false,	true},
};

static PxU32 gScenario = 0;

static PX_FORCE_INLINE	TriggerImpl	getImpl()				{ return gData[gScenario].mImpl;			}
static PX_FORCE_INLINE	bool		usesCCD()				{ return gData[gScenario].mCCD;				}
static PX_FORCE_INLINE	bool		usesTriggerTrigger()	{ return gData[gScenario].mTriggerTrigger;	}

static	PxDefaultAllocator		gAllocator;
static	PxDefaultErrorCallback	gErrorCallback;

static	PxFoundation*			gFoundation = NULL;
static	PxPhysics*				gPhysics	= NULL;

static	PxDefaultCpuDispatcher*	gDispatcher = NULL;
static	PxScene*				gScene		= NULL;
static	PxMaterial*				gMaterial	= NULL;
static	PxPvd*                  gPvd        = NULL;

static bool	gPause		= false;
static bool	gOneFrame	= false;

// Detects a trigger using the shape's simulation filter data. See createTriggerShape() function.
bool isTrigger(const PxFilterData& data)
{
	if(data.word0!=0xffffffff)
		return false;
	if(data.word1!=0xffffffff)
		return false;
	if(data.word2!=0xffffffff)
		return false;
	if(data.word3!=0xffffffff)
		return false;
	return true;
}

bool isTriggerShape(PxShape* shape)
{
	const TriggerImpl impl = getImpl();

	// Detects native built-in triggers.
	if(impl==REAL_TRIGGERS && (shape->getFlags() & PxShapeFlag::eTRIGGER_SHAPE))
		return true;

	// Detects our emulated triggers using the simulation filter data. See createTriggerShape() function.
	if(impl==FILTER_SHADER && ::isTrigger(shape->getSimulationFilterData()))
		return true;

	// Detects our emulated triggers using the simulation filter callback. See createTriggerShape() function.
	if(impl==FILTER_CALLBACK && shape->userData)
		return true;

	return false;
}

static	PxFilterFlags triggersUsingFilterShader(PxFilterObjectAttributes /*attributes0*/, PxFilterData filterData0, 
												PxFilterObjectAttributes /*attributes1*/, PxFilterData filterData1,
												PxPairFlags& pairFlags, const void* /*constantBlock*/, PxU32 /*constantBlockSize*/)
{
//	printf("contactReportFilterShader\n");

	PX_ASSERT(getImpl()==FILTER_SHADER);

	// We need to detect whether one of the shapes is a trigger.
	const bool isTriggerPair = isTrigger(filterData0) || isTrigger(filterData1);

	// If we have a trigger, replicate the trigger codepath from PxDefaultSimulationFilterShader
	if(isTriggerPair)
	{
		pairFlags = PxPairFlag::eTRIGGER_DEFAULT;

		if(usesCCD())
			pairFlags |= PxPairFlag::eDETECT_CCD_CONTACT;

		return PxFilterFlag::eDEFAULT;
	}
	else
	{
		// Otherwise use the default flags for regular pairs
		pairFlags = PxPairFlag::eCONTACT_DEFAULT;
		return PxFilterFlag::eDEFAULT;
	}
}

static	PxFilterFlags triggersUsingFilterCallback(PxFilterObjectAttributes /*attributes0*/, PxFilterData /*filterData0*/, 
												PxFilterObjectAttributes /*attributes1*/, PxFilterData /*filterData1*/,
												PxPairFlags& pairFlags, const void* /*constantBlock*/, PxU32 /*constantBlockSize*/)
{
//	printf("contactReportFilterShader\n");

	PX_ASSERT(getImpl()==FILTER_CALLBACK);

	pairFlags = PxPairFlag::eCONTACT_DEFAULT;

	if(usesCCD())
		pairFlags |= PxPairFlag::eDETECT_CCD_CONTACT|PxPairFlag::eNOTIFY_TOUCH_CCD;

	return PxFilterFlag::eCALLBACK;
}

class TriggersFilterCallback : public PxSimulationFilterCallback
{
	virtual		PxFilterFlags	pairFound(	PxU64 /*pairID*/,
											PxFilterObjectAttributes /*attributes0*/, PxFilterData /*filterData0*/, const PxActor* /*a0*/, const PxShape* s0,
											PxFilterObjectAttributes /*attributes1*/, PxFilterData /*filterData1*/, const PxActor* /*a1*/, const PxShape* s1,
											PxPairFlags& pairFlags)	PX_OVERRIDE
	{
//		printf("pairFound\n");

		if(s0->userData || s1->userData)	// See createTriggerShape() function
		{
			pairFlags = PxPairFlag::eTRIGGER_DEFAULT;

			if(usesCCD())
				pairFlags |= PxPairFlag::eDETECT_CCD_CONTACT|PxPairFlag::eNOTIFY_TOUCH_CCD;
		}
		else
			pairFlags = PxPairFlag::eCONTACT_DEFAULT;

		return PxFilterFlags();
	}

	virtual		void	pairLost(	PxU64 /*pairID*/,
									PxFilterObjectAttributes /*attributes0*/, PxFilterData /*filterData0*/,
									PxFilterObjectAttributes /*attributes1*/, PxFilterData /*filterData1*/,
									bool /*objectRemoved*/)	PX_OVERRIDE
	{
//		printf("pairLost\n");
	}

	virtual		bool	statusChange(PxU64& /*pairID*/, PxPairFlags& /*pairFlags*/, PxFilterFlags& /*filterFlags*/)	PX_OVERRIDE
	{
//		printf("statusChange\n");
		return false;
	}
}gTriggersFilterCallback;

class ContactReportCallback: public PxSimulationEventCallback
{
	void onConstraintBreak(PxConstraintInfo* /*constraints*/, PxU32 /*count*/)	PX_OVERRIDE
	{
		printf("onConstraintBreak\n");
	}

	void onWake(PxActor** /*actors*/, PxU32 /*count*/)	PX_OVERRIDE
	{
		printf("onWake\n");
	}

	void onSleep(PxActor** /*actors*/, PxU32 /*count*/)	PX_OVERRIDE
	{
		printf("onSleep\n");
	}

	void onTrigger(PxTriggerPair* pairs, PxU32 count)	PX_OVERRIDE
	{
//		printf("onTrigger: %d trigger pairs\n", count);
		while(count--)
		{
			const PxTriggerPair& current = *pairs++;
			if(current.status & PxPairFlag::eNOTIFY_TOUCH_FOUND)
				printf("Shape is entering trigger volume\n");
			if(current.status & PxPairFlag::eNOTIFY_TOUCH_LOST)
				printf("Shape is leaving trigger volume\n");
		}
	}

	void onAdvance(const PxRigidBody*const*, const PxTransform*, const PxU32)	PX_OVERRIDE
	{
		printf("onAdvance\n");
	}

	void onContact(const PxContactPairHeader& /*pairHeader*/, const PxContactPair* pairs, PxU32 count)	PX_OVERRIDE
	{
//		printf("onContact: %d pairs\n", count);

		while(count--)
		{
			const PxContactPair& current = *pairs++;

			// The reported pairs can be trigger pairs or not. We only enabled contact reports for
			// trigger pairs in the filter shader, so we don't need to do further checks here. In a
			// real-world scenario you would probably need a way to tell whether one of the shapes
			// is a trigger or not. You could e.g. reuse the PxFilterData like we did in the filter
			// shader, or maybe use the shape's userData to identify triggers, or maybe put triggers
			// in a hash-set and test the reported shape pointers against it. Many options here.

			if(current.events & (PxPairFlag::eNOTIFY_TOUCH_FOUND|PxPairFlag::eNOTIFY_TOUCH_CCD))
				printf("Shape is entering trigger volume\n");
			if(current.events & PxPairFlag::eNOTIFY_TOUCH_LOST)
				printf("Shape is leaving trigger volume\n");

			if(isTriggerShape(current.shapes[0]) && isTriggerShape(current.shapes[1]))
				printf("Trigger-trigger overlap detected\n");
		}
	}
};

static ContactReportCallback gContactReportCallback;

static PxShape* createTriggerShape(const PxGeometry& geom, bool isExclusive)
{
	const TriggerImpl impl = getImpl();

	PxShape* shape = nullptr;
	if(impl==REAL_TRIGGERS)
	{
		const PxShapeFlags shapeFlags = PxShapeFlag::eVISUALIZATION | PxShapeFlag::eTRIGGER_SHAPE;
		shape = gPhysics->createShape(geom, *gMaterial, isExclusive, shapeFlags);
	}
	else if(impl==FILTER_SHADER)
	{
		PxShapeFlags shapeFlags = PxShapeFlag::eVISUALIZATION | PxShapeFlag::eSIMULATION_SHAPE;
		shape = gPhysics->createShape(geom, *gMaterial, isExclusive, shapeFlags);

		// For this method to work, you need a way to mark shapes as triggers without using PxShapeFlag::eTRIGGER_SHAPE
		// (so that trigger-trigger pairs are reported), and without calling a PxShape function (so that the data is
		// available in a filter shader).
		//
		// One way is to reserve a special PxFilterData value/mask for triggers. It may not always be possible depending
		// on how you otherwise use the filter data).
		const PxFilterData triggerFilterData(0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff);
		shape->setSimulationFilterData(triggerFilterData);
	}
	else if(impl==FILTER_CALLBACK)
	{
		// We will have access to shape pointers in the filter callback so we just mark triggers in an arbitrary way here,
		// for example using the shape's userData.
		shape = gPhysics->createShape(geom, *gMaterial, isExclusive);
		shape->userData = shape;	// Arbitrary rule: it's a trigger if non null
	}
	return shape;
}

static void createDefaultScene()
{
	const bool ccd = usesCCD();

	// Create trigger shape
	{
		const PxVec3 halfExtent(10.0f, ccd ? 0.01f : 1.0f, 10.0f);
		PxShape* shape = createTriggerShape(PxBoxGeometry(halfExtent), false);

		if(shape)
		{
			PxRigidStatic* body = gPhysics->createRigidStatic(PxTransform(0.0f, 10.0f, 0.0f));
			body->attachShape(*shape);
			gScene->addActor(*body);
			shape->release();
		}
	}

	// Create falling rigid body
	{
		const PxVec3 halfExtent(ccd ? 0.1f : 1.0f);

		PxShape* shape = gPhysics->createShape(PxBoxGeometry(halfExtent), *gMaterial);

		PxRigidDynamic* body = gPhysics->createRigidDynamic(PxTransform(0.0f, ccd ? 30.0f : 20.0f, 0.0f));
		body->attachShape(*shape);

		PxRigidBodyExt::updateMassAndInertia(*body, 1.0f);
		gScene->addActor(*body);
		shape->release();

		if(ccd)
		{
			body->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, true);
			body->setLinearVelocity(PxVec3(0.0f, -140.0f, 0.0f));
		}
	}
}

static void createTriggerTriggerScene()
{
	struct Local
	{
		static void createSphereActor(const PxVec3& pos, const PxVec3& linVel)
		{
			PxShape* sphereShape = gPhysics->createShape(PxSphereGeometry(1.0f), *gMaterial, false);

			PxRigidDynamic* body = gPhysics->createRigidDynamic(PxTransform(pos));
			body->attachShape(*sphereShape);

			PxShape* triggerShape = createTriggerShape(PxSphereGeometry(4.0f), true);
			body->attachShape(*triggerShape);

			const bool isTriggershape = triggerShape->getFlags() & PxShapeFlag::eTRIGGER_SHAPE;
			if(!isTriggershape)
				triggerShape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false);
			PxRigidBodyExt::updateMassAndInertia(*body, 1.0f);
			if(!isTriggershape)
				triggerShape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, true);
			gScene->addActor(*body);
			sphereShape->release();
			triggerShape->release();

			body->setLinearVelocity(linVel);
		}
	};

	Local::createSphereActor(PxVec3(-5.0f, 1.0f, 0.0f), PxVec3( 1.0f, 0.0f, 0.0f));
	Local::createSphereActor(PxVec3( 5.0f, 1.0f, 0.0f), PxVec3(-1.0f, 0.0f, 0.0f));
}

static void initScene()
{
	const TriggerImpl impl = getImpl();

	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
//	sceneDesc.flags &= ~PxSceneFlag::eENABLE_PCM;
	sceneDesc.cpuDispatcher = gDispatcher;
	sceneDesc.gravity = PxVec3(0, -9.81f, 0);
	sceneDesc.simulationEventCallback = &gContactReportCallback;
	if(impl==REAL_TRIGGERS)
	{
		sceneDesc.filterShader		= PxDefaultSimulationFilterShader;
		printf("- Using built-in triggers.\n");
	}
	else if(impl==FILTER_SHADER)
	{
		sceneDesc.filterShader		= triggersUsingFilterShader;
		printf("- Using regular shapes emulating triggers with a filter shader.\n");
	}
	else if(impl==FILTER_CALLBACK)
	{
		sceneDesc.filterShader		= triggersUsingFilterCallback;
		sceneDesc.filterCallback	= &gTriggersFilterCallback;
		printf("- Using regular shapes emulating triggers with a filter callback.\n");
	}

	if(usesCCD())
	{
		sceneDesc.flags |= PxSceneFlag::eENABLE_CCD;
		printf("- Using CCD.\n");
	}
	else
	{
		printf("- Using no CCD.\n");
	}

	gScene = gPhysics->createScene(sceneDesc);

	PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
	if(pvdClient)
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);

	PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0,1,0,0), *gMaterial);
	gScene->addActor(*groundPlane);

	if(usesTriggerTrigger())
		createTriggerTriggerScene();
	else
		createDefaultScene();
}

static void releaseScene()
{
	PX_RELEASE(gScene);
}

void stepPhysics(bool /*interactive*/)
{
	if(gPause && !gOneFrame)
		return;
	gOneFrame = false;

	if(gScene)
	{
//		printf("Update...\n");
		gScene->simulate(1.0f/60.0f);
		gScene->fetchResults(true);
	}
}

void initPhysics(bool /*interactive*/)
{
	printf("Press keys F1 to F9 to select a scenario.\n");

	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	gPvd->connect(*transport,PxPvdInstrumentationFlag::eALL);
	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, gPvd);
	PxInitExtensions(*gPhysics,gPvd);
	const PxU32 numCores = SnippetUtils::getNbPhysicalCores();
	gDispatcher = PxDefaultCpuDispatcherCreate(numCores == 0 ? 0 : numCores - 1);
	gMaterial = gPhysics->createMaterial(1.0f, 1.0f, 0.0f);

	initScene();
}
	
void cleanupPhysics(bool /*interactive*/)
{
	releaseScene();

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
	
	printf("SnippetTriggers done.\n");
}

void keyPress(unsigned char key, const PxTransform& /*camera*/)
{
	if(key=='p' || key=='P')
		gPause = !gPause;

	if(key=='o' || key=='O')
	{
		gPause = true;
		gOneFrame = true;
	}

	if(gScene)
	{
		if(key>=1 && key<=SCENARIO_COUNT)
		{
			gScenario = key-1;
			releaseScene();
			initScene();
		}

		if(key=='r' || key=='R')
		{
			releaseScene();
			initScene();
		}
	}
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

