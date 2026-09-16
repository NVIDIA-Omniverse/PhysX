// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// PX_DUMMY_SYMBOL

#include "foundation/PxPreprocessor.h"
#if PX_SUPPORT_PVD

#include "pvd/PxPvdTransport.h"
#include "PxPhysics.h"
#include "PxPvdClient.h"
#include "PxPvdDataStream.h"
#include "PxPvdObjectModelBaseTypes.h"
#include "PvdPhysicsClient.h"
#include "PvdTypeNames.h"

using namespace physx;
using namespace physx::Vd;

PvdPhysicsClient::PvdPhysicsClient(PsPvd* pvd)
: mPvd(pvd), mPvdDataStream(NULL), mIsConnected(false)
{
}

PvdPhysicsClient::~PvdPhysicsClient()
{
	mPvd->removeClient(this);	
}

PvdDataStream* PvdPhysicsClient::getDataStream()
{
	return mPvdDataStream;
}

bool PvdPhysicsClient::isConnected() const
{
	return mIsConnected;
}

void PvdPhysicsClient::onPvdConnected()
{
	if(mIsConnected || !mPvd)
		return;

	mIsConnected = true;	
	mPvdDataStream = PvdDataStream::create(mPvd); 	
	sendEntireSDK();
}

void PvdPhysicsClient::onPvdDisconnected()
{
	if(!mIsConnected)
		return;
	mIsConnected = false;

	mPvdDataStream->release();
	mPvdDataStream = NULL;	
}

void PvdPhysicsClient::flush()
{
}

void PvdPhysicsClient::sendEntireSDK()
{
	PxPhysics& physics = PxGetPhysics();
	
	mMetaDataBinding.registerSDKProperties(*mPvdDataStream);
	mPvdDataStream->createInstance(&physics);
	
	mPvdDataStream->setIsTopLevelUIElement(&physics, true);
	mMetaDataBinding.sendAllProperties(*mPvdDataStream, physics);

#define SEND_BUFFER_GROUP(type, name)                   \
	{                                                   \
		physx::PxArray<type*> buffers;            \
		PxU32 numBuffers = physics.getNb##name();       \
		buffers.resize(numBuffers);                     \
		physics.get##name(buffers.begin(), numBuffers);	\
		for(PxU32 i = 0; i < numBuffers; i++)           \
		{                                               \
		if(mPvd->registerObject(buffers[i]))            \
				createPvdInstance(buffers[i]);          \
		}                                               \
	}
	
	SEND_BUFFER_GROUP(PxMaterial, Materials);
	SEND_BUFFER_GROUP(PxTriangleMesh, TriangleMeshes);
	SEND_BUFFER_GROUP(PxConvexMesh, ConvexMeshes);
	SEND_BUFFER_GROUP(PxTetrahedronMesh, TetrahedronMeshes);
	SEND_BUFFER_GROUP(PxHeightField, HeightFields);
}

void PvdPhysicsClient::destroyPvdInstance(const PxPhysics* physics)
{
	if(mPvdDataStream)
	     mPvdDataStream->destroyInstance(physics);
}

void PvdPhysicsClient::createPvdInstance(const PxTriangleMesh* triMesh)
{
	mMetaDataBinding.createInstance(*mPvdDataStream, *triMesh, PxGetPhysics());
}

void PvdPhysicsClient::destroyPvdInstance(const PxTriangleMesh* triMesh)
{
	mMetaDataBinding.destroyInstance(*mPvdDataStream, *triMesh, PxGetPhysics());
}

void PvdPhysicsClient::createPvdInstance(const PxTetrahedronMesh* tetMesh)
{
	mMetaDataBinding.createInstance(*mPvdDataStream, *tetMesh, PxGetPhysics());
}

void PvdPhysicsClient::destroyPvdInstance(const PxTetrahedronMesh* tetMesh)
{
	mMetaDataBinding.destroyInstance(*mPvdDataStream, *tetMesh, PxGetPhysics());
}

void PvdPhysicsClient::createPvdInstance(const PxConvexMesh* convexMesh)
{
	mMetaDataBinding.createInstance(*mPvdDataStream, *convexMesh, PxGetPhysics());
}

void PvdPhysicsClient::destroyPvdInstance(const PxConvexMesh* convexMesh)
{
	mMetaDataBinding.destroyInstance(*mPvdDataStream, *convexMesh, PxGetPhysics());
}

void PvdPhysicsClient::createPvdInstance(const PxHeightField* heightField)
{
	mMetaDataBinding.createInstance(*mPvdDataStream, *heightField, PxGetPhysics());
}

void PvdPhysicsClient::destroyPvdInstance(const PxHeightField* heightField)
{
	mMetaDataBinding.destroyInstance(*mPvdDataStream, *heightField, PxGetPhysics());
}

void PvdPhysicsClient::createPvdInstance(const PxMaterial* mat)
{
	mMetaDataBinding.createInstance(*mPvdDataStream, *mat, PxGetPhysics());
}

void PvdPhysicsClient::updatePvdProperties(const PxMaterial* mat)
{
	mMetaDataBinding.sendAllProperties(*mPvdDataStream, *mat);
}

void PvdPhysicsClient::destroyPvdInstance(const PxMaterial* mat)
{
	mMetaDataBinding.destroyInstance(*mPvdDataStream, *mat, PxGetPhysics());
}
#if PX_SUPPORT_GPU_PHYSX
void PvdPhysicsClient::createPvdInstance(const PxDeformableSurfaceMaterial* mat)
{
	mMetaDataBinding.createInstance(*mPvdDataStream, *mat, PxGetPhysics());
}

void PvdPhysicsClient::updatePvdProperties(const PxDeformableSurfaceMaterial* mat)
{
	mMetaDataBinding.sendAllProperties(*mPvdDataStream, *mat);
}

void PvdPhysicsClient::destroyPvdInstance(const PxDeformableSurfaceMaterial* mat)
{
	mMetaDataBinding.destroyInstance(*mPvdDataStream, *mat, PxGetPhysics());
}

void PvdPhysicsClient::createPvdInstance(const PxDeformableVolumeMaterial* mat)
{
	mMetaDataBinding.createInstance(*mPvdDataStream, *mat, PxGetPhysics());
}

void PvdPhysicsClient::updatePvdProperties(const PxDeformableVolumeMaterial* mat)
{
	mMetaDataBinding.sendAllProperties(*mPvdDataStream, *mat);
}

void PvdPhysicsClient::destroyPvdInstance(const PxDeformableVolumeMaterial* mat)
{
	mMetaDataBinding.destroyInstance(*mPvdDataStream, *mat, PxGetPhysics());
}

void PvdPhysicsClient::createPvdInstance(const PxPBDMaterial* mat)
{
	mMetaDataBinding.createInstance(*mPvdDataStream, *mat, PxGetPhysics());
}

void PvdPhysicsClient::updatePvdProperties(const PxPBDMaterial* mat)
{
	mMetaDataBinding.sendAllProperties(*mPvdDataStream, *mat);
}

void PvdPhysicsClient::destroyPvdInstance(const PxPBDMaterial* mat)
{
	mMetaDataBinding.destroyInstance(*mPvdDataStream, *mat, PxGetPhysics());
}
#endif

void PvdPhysicsClient::onMeshFactoryBufferRelease(const PxBase* object, PxType typeID)
{
	if(!mIsConnected || !mPvd)
		return;

	if(mPvd->unRegisterObject(object))
	{
		switch(typeID)
		{
		case PxConcreteType::eHEIGHTFIELD:
			destroyPvdInstance(static_cast<const PxHeightField*>(object));
			break;

		case PxConcreteType::eCONVEX_MESH:
			destroyPvdInstance(static_cast<const PxConvexMesh*>(object));
			break;

		case PxConcreteType::eTRIANGLE_MESH_BVH33:
		case PxConcreteType::eTRIANGLE_MESH_BVH34:
			destroyPvdInstance(static_cast<const PxTriangleMesh*>(object));
			break;

		case PxConcreteType::eTETRAHEDRON_MESH:
			destroyPvdInstance(static_cast<const PxTetrahedronMesh*>(object));
			break;

		default:
			break;
		}
	}
}

void PvdPhysicsClient::reportError(PxErrorCode::Enum code, const char* message, const char* file, int line)
{
    if(mIsConnected)
	{
		mPvdDataStream->sendErrorMessage(code, message, file, PxU32(line));
	}
}

#endif // PX_SUPPORT_PVD
