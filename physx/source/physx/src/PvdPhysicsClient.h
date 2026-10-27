// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PVD_PHYSICS_CLIENT_H
#define PVD_PHYSICS_CLIENT_H
#if PX_SUPPORT_PVD
#include "foundation/PxErrorCallback.h"
#include "foundation/PxHashMap.h"
#include "PxPvdClient.h"
#include "PvdMetaDataPvdBinding.h"
#include "NpFactory.h"
#include "foundation/PxMutex.h"
#include "PsPvd.h"

namespace physx
{
class PxProfileMemoryEventBuffer;

namespace Vd
{

class PvdPhysicsClient : public PvdClient, public PxErrorCallback, public NpFactoryListener, public PxUserAllocated
{
	PX_NOCOPY(PvdPhysicsClient)
  public:
	PvdPhysicsClient(PsPvd* pvd);
	virtual ~PvdPhysicsClient();

	virtual	bool isConnected() const	PX_OVERRIDE;
	virtual	void onPvdConnected()	PX_OVERRIDE;
	virtual	void onPvdDisconnected()	PX_OVERRIDE;
	virtual	void flush()	PX_OVERRIDE;

	virtual	physx::pvdsdk::PvdDataStream* getDataStream()	PX_OVERRIDE;
	
	void sendEntireSDK();	
	void destroyPvdInstance(const PxPhysics* physics);

	// NpFactoryListener
	virtual void onMeshFactoryBufferRelease(const PxBase* object, PxType typeID) PX_OVERRIDE;
	/// NpFactoryListener

	// PxErrorCallback
	virtual	void reportError(PxErrorCode::Enum code, const char* message, const char* file, int line)	PX_OVERRIDE;

  private:
	void createPvdInstance(const PxTriangleMesh* triMesh);
	void destroyPvdInstance(const PxTriangleMesh* triMesh);
	void createPvdInstance(const PxTetrahedronMesh* tetMesh);
	void destroyPvdInstance(const PxTetrahedronMesh* tetMesh);
	void createPvdInstance(const PxConvexMesh* convexMesh);
	void destroyPvdInstance(const PxConvexMesh* convexMesh);
	void createPvdInstance(const PxHeightField* heightField);
	void destroyPvdInstance(const PxHeightField* heightField);
	void createPvdInstance(const PxMaterial* mat);
	void destroyPvdInstance(const PxMaterial* mat);
	void updatePvdProperties(const PxMaterial* mat);
#if PX_SUPPORT_GPU_PHYSX
	void createPvdInstance(const PxDeformableSurfaceMaterial* mat);
	void destroyPvdInstance(const PxDeformableSurfaceMaterial* mat);
	void updatePvdProperties(const PxDeformableSurfaceMaterial* mat);

	void createPvdInstance(const PxDeformableVolumeMaterial* mat);
	void destroyPvdInstance(const PxDeformableVolumeMaterial* mat);
	void updatePvdProperties(const PxDeformableVolumeMaterial* mat);

	void createPvdInstance(const PxPBDMaterial* mat);
	void destroyPvdInstance(const PxPBDMaterial* mat);
	void updatePvdProperties(const PxPBDMaterial* mat);
#endif

	PsPvd*  mPvd;
	PvdDataStream* mPvdDataStream;
	PvdMetaDataBinding mMetaDataBinding;	
	bool mIsConnected;	
};

} // namespace Vd
} // namespace physx

#endif // PX_SUPPORT_PVD
#endif // PVD_PHYSICS_CLIENT_H
