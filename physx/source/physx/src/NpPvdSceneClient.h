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

#ifndef NP_SCENE_PVD_CLIENT_H
#define NP_SCENE_PVD_CLIENT_H

#include "PxPhysXConfig.h"

#if PX_SUPPORT_PVD

#include "foundation/PxStrideIterator.h"
#include "pvd/PxPvdTransport.h"
#include "pvd/PxPvdSceneClient.h"

#include "PvdMetaDataPvdBinding.h"
#include "foundation/PxBitMap.h"
#include "PxPvdClient.h"
#include "PxPvdUserRenderer.h"
#include "PsPvd.h"

#include "PxsMaterialCore.h"
#include "PxsFEMSoftBodyMaterialCore.h"
#include "PxsFEMClothMaterialCore.h"
#include "PxsPBDMaterialCore.h"

namespace physx
{
class PxActor;
class PxArticulationLink;
class PxRenderBuffer;
class NpConstraint;
class NpShape;
class NpAggregate;
class NpRigidStatic;
class NpRigidDynamic;
class NpArticulationLink;
class NpArticulationJointReducedCoordinate;
class NpArticulationReducedCoordinate;
class NpArticulationSpatialTendon;
class NpArticulationFixedTendon;
class NpArticulationMimicJoint;
class NpActor;
class NpScene;

#if PX_SUPPORT_GPU_PHYSX
class NpSoftBody;
class NpFEMCloth;
class NpPBDParticleSystem;
class NpHairSystem;
#endif

namespace Sc
{
class ConstraintCore;
}

namespace Vd
{
class PvdSceneClient : public PxPvdSceneClient, public PvdClient, public PvdVisualizer
{
	PX_NOCOPY(PvdSceneClient)
  public:
							PvdSceneClient(NpScene& scene);
	virtual					~PvdSceneClient();

	// PxPvdSceneClient
	virtual	void			setScenePvdFlag(PxPvdSceneFlag::Enum flag, bool value);
	virtual	void			setScenePvdFlags(PxPvdSceneFlags flags)				{ mFlags = flags;	}
	virtual	PxPvdSceneFlags	getScenePvdFlags()							const	{ return mFlags;	}
	virtual	void			updateCamera(const char* name, const PxVec3& origin, const PxVec3& up, const PxVec3& target);
	virtual	void			drawPoints(const PxDebugPoint* points, PxU32 count);
	virtual	void			drawLines(const PxDebugLine* lines, PxU32 count);
	virtual	void			drawTriangles(const PxDebugTriangle* triangles, PxU32 count);
	virtual	void			drawText(const PxDebugText& text);
	virtual	PvdClient*		getClientInternal()									{ return this;		}
	//~PxPvdSceneClient
	
	// pvdClient	
	virtual	PvdDataStream*		getDataStream()			{ return mPvdDataStream;	}
	virtual bool                isConnected()	const	{ return mIsConnected;		}
	virtual void                onPvdConnected();
	virtual void                onPvdDisconnected();
	virtual void                flush()					{}
	//~pvdClient

	PX_FORCE_INLINE bool checkPvdDebugFlag()	const
	{
		return mIsConnected && (mPvd->getInstrumentationFlags() & PxPvdInstrumentationFlag::eDEBUG);
	}

	PX_FORCE_INLINE	PxPvdSceneFlags	getScenePvdFlagsFast() const	{ return mFlags;	}
	PX_FORCE_INLINE	void             setPsPvd(PsPvd* pvd)			{ mPvd = pvd;		}

	void frameStart(PxReal simulateElapsedTime);
	void frameEnd();

	void updatePvdProperties();
	void releasePvdInstance();

	void createPvdInstance	(const PxActor* actor); // temporary for deformables and particle systems - sschirm: deformables and particles are gone...
	void updatePvdProperties(const PxActor* actor);
	void releasePvdInstance	(const PxActor* actor); // temporary for deformables and particle systems - sschirm: deformables and particles are gone...

	void createPvdInstance	(const NpActor* actor); // temporary for deformables and particle systems - sschirm: deformables and particles are gone...
	void updatePvdProperties(const NpActor* actor);
	void releasePvdInstance	(const NpActor* actor); // temporary for deformables and particle systems - sschirm: deformables and particles are gone...

	void createPvdInstance		(const NpRigidDynamic* body);
	void createPvdInstance		(const NpArticulationLink* body);
	void updatePvdProperties	(const NpRigidDynamic* body);
	void updatePvdProperties	(const NpArticulationLink* body);
	void releasePvdInstance		(const NpRigidDynamic* body);
	void releasePvdInstance		(const NpArticulationLink* body);
	void updateBodyPvdProperties(const NpActor* body);
	void updateKinematicTarget	(const NpActor* body, const PxTransform& p);

	void createPvdInstance		(const NpRigidStatic* rigidStatic);
	void updatePvdProperties	(const NpRigidStatic* rigidStatic);
	void releasePvdInstance		(const NpRigidStatic* rigidStatic);

	void createPvdInstance	(const NpConstraint* constraint);
	void updatePvdProperties(const NpConstraint* constraint);
	void releasePvdInstance	(const NpConstraint* constraint);

	void createPvdInstance	(const NpArticulationReducedCoordinate* articulation);
	void updatePvdProperties(const NpArticulationReducedCoordinate* articulation);
	void releasePvdInstance	(const NpArticulationReducedCoordinate* articulation);

	void createPvdInstance	(const NpArticulationJointReducedCoordinate* articulationJoint);
	void updatePvdProperties(const NpArticulationJointReducedCoordinate* articulationJoint);
	void releasePvdInstance	(const NpArticulationJointReducedCoordinate* articulationJoint);

	void createPvdInstance(const NpArticulationSpatialTendon* articulationTendon);
	void updatePvdProperties(const NpArticulationSpatialTendon* articulationTendon);
	void releasePvdInstance(const NpArticulationSpatialTendon* articulationTendon);

	void createPvdInstance(const NpArticulationFixedTendon* articulationTendon);
	void updatePvdProperties(const NpArticulationFixedTendon* articulationTendon);
	void releasePvdInstance(const NpArticulationFixedTendon* articulationTendon);

	void createPvdInstance(const NpArticulationMimicJoint* mimicJoint);
	void updatePvdProperties(const NpArticulationMimicJoint* mimicJoint);
	void releasePvdInstance(const NpArticulationMimicJoint* mimicJoint);

	///////////////////////////////////////////////////////////////////////////

	void createPvdInstance	(const PxsMaterialCore* materialCore);
	void updatePvdProperties(const PxsMaterialCore* materialCore);
	void releasePvdInstance	(const PxsMaterialCore* materialCore);

	void createPvdInstance	(const PxsFEMSoftBodyMaterialCore* materialCore);
	void updatePvdProperties(const PxsFEMSoftBodyMaterialCore* materialCore);
	void releasePvdInstance	(const PxsFEMSoftBodyMaterialCore* materialCore);

	void createPvdInstance	(const PxsFEMClothMaterialCore* materialCore);
	void updatePvdProperties(const PxsFEMClothMaterialCore* materialCore);
	void releasePvdInstance	(const PxsFEMClothMaterialCore* materialCore);

	void createPvdInstance	(const PxsPBDMaterialCore* materialCore);
	void updatePvdProperties(const PxsPBDMaterialCore* materialCore);
	void releasePvdInstance	(const PxsPBDMaterialCore* materialCore);

	///////////////////////////////////////////////////////////////////////////

	void createPvdInstance			(const NpShape* shape, PxActor& owner);
	void updateMaterials			(const NpShape* shape);
	void updatePvdProperties		(const NpShape* shape);
	void releaseAndRecreateGeometry	(const NpShape* shape);
	void releasePvdInstance			(const NpShape* shape, PxActor& owner);
	void addBodyAndShapesToPvd		(NpRigidDynamic& b);
	void addStaticAndShapesToPvd	(NpRigidStatic& s);

	void createPvdInstance		(const NpAggregate* aggregate);
	void updatePvdProperties	(const NpAggregate* aggregate);
	void attachAggregateActor	(const NpAggregate* aggregate, NpActor* actor);
	void detachAggregateActor	(const NpAggregate* aggregate, NpActor* actor);
	void releasePvdInstance		(const NpAggregate* aggregate);

#if PX_SUPPORT_GPU_PHYSX
	void createPvdInstance(const NpSoftBody* softBody);
	void updatePvdProperties(const NpSoftBody* softBody);
	void attachAggregateActor(const NpSoftBody* softBody, NpActor* actor);
	void detachAggregateActor(const NpSoftBody* softBody, NpActor* actor);
	void releasePvdInstance(const NpSoftBody* softBody);

	void createPvdInstance(const NpFEMCloth* femCloth);
	void updatePvdProperties(const NpFEMCloth* femCloth);
	void attachAggregateActor(const NpFEMCloth* femCloth, NpActor* actor);
	void detachAggregateActor(const NpFEMCloth* femCloth, NpActor* actor);
	void releasePvdInstance(const NpFEMCloth* femCloth);

	void createPvdInstance(const NpPBDParticleSystem* particleSystem);
	void updatePvdProperties(const NpPBDParticleSystem* particleSystem);
	void attachAggregateActor(const NpPBDParticleSystem* particleSystem, NpActor* actor);
	void detachAggregateActor(const NpPBDParticleSystem* particleSystem, NpActor* actor);
	void releasePvdInstance(const NpPBDParticleSystem* particleSystem);

	void createPvdInstance(const NpHairSystem* hairSystem);
	void updatePvdProperties(const NpHairSystem* hairSystem);
	void attachAggregateActor(const NpHairSystem* hairSystem, NpActor* actor);
	void detachAggregateActor(const NpHairSystem* hairSystem, NpActor* actor);
	void releasePvdInstance(const NpHairSystem* hairSystem);
#endif

	void originShift(PxVec3 shift);
	void updateJoints();
	void updateContacts();
	void updateSceneQueries();

	// PvdVisualizer
	void visualize(PxArticulationLink& link);
	void visualize(const PxRenderBuffer& debugRenderable);

  private:

	void				sendEntireScene();
	void				updateConstraint(const Sc::ConstraintCore& scConstraint, PxU32 updateType);

	PxPvdSceneFlags			mFlags;
	PsPvd*					mPvd;
	NpScene&				mScene;
	
	PvdDataStream*			mPvdDataStream;
	PvdMetaDataBinding		mMetaDataBinding;
	PvdUserRenderer*		mUserRender;
	RendererEventClient*	mRenderClient;
	bool					mIsConnected;
};

} // pvd

} // physx
#endif // PX_SUPPORT_PVD

#endif // NP_SCENE_PVD_CLIENT_H
