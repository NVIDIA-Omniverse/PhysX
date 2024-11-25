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

#include "foundation/PxIO.h"
#include "common/PxMetaData.h"
#include "common/PxSerializer.h"
#include "extensions/PxExtensionsAPI.h"
#include "extensions/PxRepXSerializer.h"

#include "ExtDistanceJoint.h"
#include "ExtD6Joint.h"
#include "ExtFixedJoint.h"
#include "ExtPrismaticJoint.h"
#include "ExtRevoluteJoint.h"
#include "ExtSphericalJoint.h"
#include "ExtGearJoint.h"
#include "ExtRackAndPinionJoint.h"
#include "ExtSerialization.h"
#include "SnRepXCoreSerializer.h"
#include "SnJointRepXSerializer.h"
#include "PxExtensionMetaDataObjects.h"

#if PX_SUPPORT_PVD
#include "ExtPvd.h"
#include "PxPvdDataStream.h"
#include "PxPvdClient.h"
#include "PsPvd.h"
#endif

#if PX_SUPPORT_OMNI_PVD
#	include "omnipvd/PxOmniPvd.h"
#	include "omnipvd/OmniPvdPxExtensionsSampler.h"
#endif

using namespace physx;
using namespace physx::pvdsdk;

#if PX_SUPPORT_PVD
struct JointConnectionHandler : public PvdClient
{
	JointConnectionHandler() : mPvd(NULL),mConnected(false){}

	PvdDataStream*		getDataStream()
	{
		return NULL;
	}	

	void onPvdConnected()
	{
		PvdDataStream* stream = PvdDataStream::create(mPvd);
		if(stream)
		{
			mConnected = true;
			Ext::Pvd::sendClassDescriptions(*stream);	
			stream->release();
		}		
	}

	bool isConnected() const
	{
		return mConnected;
	}

	void onPvdDisconnected()
	{
		mConnected = false;
	}

	void flush()
	{
	}

	PsPvd* mPvd;
	bool mConnected;
};

static JointConnectionHandler gPvdHandler;
#endif

bool PxInitExtensions(PxPhysics& physics, PxPvd* pvd)
{
	PX_ASSERT(&physics.getFoundation() == &PxGetFoundation());
	PX_UNUSED(physics);
	PX_UNUSED(pvd);
	PxIncFoundationRefCount();

#if PX_SUPPORT_PVD
	if(pvd)
	{
		gPvdHandler.mPvd = static_cast<PsPvd*>(pvd);
		gPvdHandler.mPvd->addClient(&gPvdHandler);
	}
#endif

#if PX_SUPPORT_OMNI_PVD
	if (physics.getOmniPvd() && physics.getOmniPvd()->getWriter())
	{
		if (OmniPvdPxExtensionsSampler::createInstance())
		{
			OmniPvdPxExtensionsSampler::getInstance()->setOmniPvdInstance(physics.getOmniPvd());
			OmniPvdPxExtensionsSampler::getInstance()->registerClasses();
		}
	}
#endif
	return true;
}

static PxArray<PxSceneQuerySystem*>*	gExternalSQ = NULL;

void addExternalSQ(PxSceneQuerySystem* added)
{
	if(!gExternalSQ)
		gExternalSQ = new PxArray<PxSceneQuerySystem*>;

	gExternalSQ->pushBack(added);
}

void removeExternalSQ(PxSceneQuerySystem* removed)
{
	if(gExternalSQ)
	{
		const PxU32 nb = gExternalSQ->size();
		for(PxU32 i=0;i<nb;i++)
		{
			PxSceneQuerySystem* sq = (*gExternalSQ)[i];
			if(sq==removed)
			{
				gExternalSQ->replaceWithLast(i);
				return;
			}
		}
	}
}

static void releaseExternalSQ()
{
	if(gExternalSQ)
	{
		PxArray<PxSceneQuerySystem*>* copy = gExternalSQ;
		gExternalSQ = NULL;

		const PxU32 nb = copy->size();
		for(PxU32 i=0;i<nb;i++)
		{
			PxSceneQuerySystem* sq = (*copy)[i];
			sq->release();
		}
		PX_DELETE(copy);
	}
}

void PxCloseExtensions()
{
	releaseExternalSQ();

	PxDecFoundationRefCount();

#if PX_SUPPORT_PVD
	if(gPvdHandler.mConnected)
	{	
		PX_ASSERT(gPvdHandler.mPvd);
		gPvdHandler.mPvd->removeClient(&gPvdHandler);
		gPvdHandler.mPvd = NULL;
	}
#endif

#if PX_SUPPORT_OMNI_PVD
	if (OmniPvdPxExtensionsSampler::getInstance())
	{
		OmniPvdPxExtensionsSampler::destroyInstance();
	}
#endif
}

void Ext::RegisterExtensionsSerializers(PxSerializationRegistry& sr)
{
	//for repx serialization
	sr.registerRepXSerializer(PxConcreteType::eMATERIAL,						PX_NEW_REPX_SERIALIZER( PxMaterialRepXSerializer ));
	sr.registerRepXSerializer(PxConcreteType::eSHAPE,							PX_NEW_REPX_SERIALIZER( PxShapeRepXSerializer ));	
	sr.registerRepXSerializer(PxConcreteType::eTRIANGLE_MESH_BVH33,				PX_NEW_REPX_SERIALIZER( PxBVH33TriangleMeshRepXSerializer ));
	sr.registerRepXSerializer(PxConcreteType::eTRIANGLE_MESH_BVH34,				PX_NEW_REPX_SERIALIZER( PxBVH34TriangleMeshRepXSerializer ));
	sr.registerRepXSerializer(PxConcreteType::eHEIGHTFIELD,						PX_NEW_REPX_SERIALIZER( PxHeightFieldRepXSerializer ));
	sr.registerRepXSerializer(PxConcreteType::eCONVEX_MESH,						PX_NEW_REPX_SERIALIZER( PxConvexMeshRepXSerializer ));
	sr.registerRepXSerializer(PxConcreteType::eRIGID_STATIC,					PX_NEW_REPX_SERIALIZER( PxRigidStaticRepXSerializer ));	
	sr.registerRepXSerializer(PxConcreteType::eRIGID_DYNAMIC,					PX_NEW_REPX_SERIALIZER( PxRigidDynamicRepXSerializer ));
	sr.registerRepXSerializer(PxConcreteType::eARTICULATION_REDUCED_COORDINATE,	PX_NEW_REPX_SERIALIZER( PxArticulationReducedCoordinateRepXSerializer));
	sr.registerRepXSerializer(PxConcreteType::eAGGREGATE,						PX_NEW_REPX_SERIALIZER( PxAggregateRepXSerializer ));
	
	sr.registerRepXSerializer(PxJointConcreteType::eFIXED,						PX_NEW_REPX_SERIALIZER( PxJointRepXSerializer<PxFixedJoint> ));
	sr.registerRepXSerializer(PxJointConcreteType::eDISTANCE,					PX_NEW_REPX_SERIALIZER( PxJointRepXSerializer<PxDistanceJoint> ));
	sr.registerRepXSerializer(PxJointConcreteType::eD6,							PX_NEW_REPX_SERIALIZER( PxJointRepXSerializer<PxD6Joint> ));
	sr.registerRepXSerializer(PxJointConcreteType::ePRISMATIC,					PX_NEW_REPX_SERIALIZER( PxJointRepXSerializer<PxPrismaticJoint> ));
	sr.registerRepXSerializer(PxJointConcreteType::eREVOLUTE,					PX_NEW_REPX_SERIALIZER( PxJointRepXSerializer<PxRevoluteJoint> ));
	sr.registerRepXSerializer(PxJointConcreteType::eSPHERICAL,					PX_NEW_REPX_SERIALIZER( PxJointRepXSerializer<PxSphericalJoint> ));

	//for binary serialization
	sr.registerSerializer(PxJointConcreteType::eFIXED,							PX_NEW_SERIALIZER_ADAPTER( FixedJoint ));
	sr.registerSerializer(PxJointConcreteType::eDISTANCE,						PX_NEW_SERIALIZER_ADAPTER( DistanceJoint ));
	sr.registerSerializer(PxJointConcreteType::eD6,								PX_NEW_SERIALIZER_ADAPTER( D6Joint) );
	sr.registerSerializer(PxJointConcreteType::ePRISMATIC,						PX_NEW_SERIALIZER_ADAPTER( PrismaticJoint ));
	sr.registerSerializer(PxJointConcreteType::eREVOLUTE,						PX_NEW_SERIALIZER_ADAPTER( RevoluteJoint ));
	sr.registerSerializer(PxJointConcreteType::eSPHERICAL,						PX_NEW_SERIALIZER_ADAPTER( SphericalJoint ));
	sr.registerSerializer(PxJointConcreteType::eGEAR,							PX_NEW_SERIALIZER_ADAPTER( GearJoint ));
	sr.registerSerializer(PxJointConcreteType::eRACK_AND_PINION,				PX_NEW_SERIALIZER_ADAPTER( RackAndPinionJoint ));
}

void Ext::UnregisterExtensionsSerializers(PxSerializationRegistry& sr)
{
	PX_DELETE_SERIALIZER_ADAPTER(sr.unregisterSerializer(PxJointConcreteType::eFIXED));
	PX_DELETE_SERIALIZER_ADAPTER(sr.unregisterSerializer(PxJointConcreteType::eDISTANCE));
	PX_DELETE_SERIALIZER_ADAPTER(sr.unregisterSerializer(PxJointConcreteType::eD6 ));
	PX_DELETE_SERIALIZER_ADAPTER(sr.unregisterSerializer(PxJointConcreteType::ePRISMATIC));
	PX_DELETE_SERIALIZER_ADAPTER(sr.unregisterSerializer(PxJointConcreteType::eREVOLUTE));
	PX_DELETE_SERIALIZER_ADAPTER(sr.unregisterSerializer(PxJointConcreteType::eSPHERICAL));
	PX_DELETE_SERIALIZER_ADAPTER(sr.unregisterSerializer(PxJointConcreteType::eGEAR));
	PX_DELETE_SERIALIZER_ADAPTER(sr.unregisterSerializer(PxJointConcreteType::eRACK_AND_PINION));

	PX_DELETE_REPX_SERIALIZER(sr.unregisterRepXSerializer(PxConcreteType::eMATERIAL));
	PX_DELETE_REPX_SERIALIZER(sr.unregisterRepXSerializer(PxConcreteType::eSHAPE));	
//	PX_DELETE_REPX_SERIALIZER(sr.unregisterRepXSerializer(PxConcreteType::eTRIANGLE_MESH));
	PX_DELETE_REPX_SERIALIZER(sr.unregisterRepXSerializer(PxConcreteType::eTRIANGLE_MESH_BVH33));
	PX_DELETE_REPX_SERIALIZER(sr.unregisterRepXSerializer(PxConcreteType::eTRIANGLE_MESH_BVH34));
	PX_DELETE_REPX_SERIALIZER(sr.unregisterRepXSerializer(PxConcreteType::eHEIGHTFIELD));
	PX_DELETE_REPX_SERIALIZER(sr.unregisterRepXSerializer(PxConcreteType::eCONVEX_MESH));
	PX_DELETE_REPX_SERIALIZER(sr.unregisterRepXSerializer(PxConcreteType::eRIGID_STATIC));	
	PX_DELETE_REPX_SERIALIZER(sr.unregisterRepXSerializer(PxConcreteType::eRIGID_DYNAMIC));
	PX_DELETE_REPX_SERIALIZER(sr.unregisterRepXSerializer(PxConcreteType::eARTICULATION_REDUCED_COORDINATE));
	PX_DELETE_REPX_SERIALIZER(sr.unregisterRepXSerializer(PxConcreteType::eAGGREGATE));

	PX_DELETE_REPX_SERIALIZER(sr.unregisterRepXSerializer(PxJointConcreteType::eFIXED));
	PX_DELETE_REPX_SERIALIZER(sr.unregisterRepXSerializer(PxJointConcreteType::eDISTANCE));
	PX_DELETE_REPX_SERIALIZER(sr.unregisterRepXSerializer(PxJointConcreteType::eD6));
	PX_DELETE_REPX_SERIALIZER(sr.unregisterRepXSerializer(PxJointConcreteType::ePRISMATIC));
	PX_DELETE_REPX_SERIALIZER(sr.unregisterRepXSerializer(PxJointConcreteType::eREVOLUTE));
	PX_DELETE_REPX_SERIALIZER(sr.unregisterRepXSerializer(PxJointConcreteType::eSPHERICAL));
}
