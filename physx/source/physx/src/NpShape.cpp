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

#include "NpShape.h"
#include "NpCheck.h"
#include "NpRigidStatic.h"
#include "NpRigidDynamic.h"
#include "NpArticulationLink.h"
#include "GuConvexMesh.h"
#include "GuTriangleMesh.h"
#include "GuTetrahedronMesh.h"
#include "GuBounds.h"
#include "NpFEMCloth.h"
#include "NpSoftBody.h"

#include "omnipvd/OmniPvdPxSampler.h"

using namespace physx;
using namespace Sq;
using namespace Cm;

///////////////////////////////////////////////////////////////////////////////

PX_IMPLEMENT_OUTPUT_ERROR

///////////////////////////////////////////////////////////////////////////////

// PT: we're using mFreeSlot as a replacement for previous mExclusiveAndActorCount
static PX_FORCE_INLINE void increaseActorCount(PxU32* count)
{
	volatile PxI32* val = reinterpret_cast<volatile PxI32*>(count);
	PxAtomicIncrement(val);
}

static PX_FORCE_INLINE void decreaseActorCount(PxU32* count)
{
	volatile PxI32* val = reinterpret_cast<volatile PxI32*>(count);
	PxAtomicDecrement(val);
}

NpShape::NpShape(const PxGeometry& geometry, PxShapeFlags shapeFlags, const PxU16* materialIndices, PxU16 materialCount, bool isExclusive, PxShapeCoreFlag::Enum flag) :
	PxShape	(PxConcreteType::eSHAPE, PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE),
	NpBase					(NpType::eSHAPE),
	mExclusiveShapeActor	(NULL),
	mCore					(geometry, shapeFlags, materialIndices, materialCount, isExclusive, flag)
{
	//actor count
	mFreeSlot = 0;

	PX_ASSERT(mCore.getPxShape() == static_cast<PxShape*>(this));
	PX_ASSERT(!PxShape::userData);

	mCore.mName = NULL;

	incMeshRefCount();
}

NpShape::~NpShape()
{
	decMeshRefCount();

	const PxU32 nbMaterials = scGetNbMaterials();
	PxShapeCoreFlags flags = mCore.getCore().mShapeCoreFlags;

	if (flags & PxShapeCoreFlag::eCLOTH_SHAPE)
	{
#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION && PX_SUPPORT_GPU_PHYSX
		for (PxU32 i = 0; i < nbMaterials; i++)
			RefCountable_decRefCount(*scGetMaterial<NpFEMClothMaterial>(i));
#endif
	}
	else if(flags & PxShapeCoreFlag::eSOFT_BODY_SHAPE)
	{ 
#if PX_SUPPORT_GPU_PHYSX
		for (PxU32 i = 0; i < nbMaterials; i++)
			RefCountable_decRefCount(*scGetMaterial<NpFEMSoftBodyMaterial>(i));
#endif
	}
	else
	{
		for (PxU32 i = 0; i < nbMaterials; i++)
			RefCountable_decRefCount(*scGetMaterial<NpMaterial>(i));
	}
}

void NpShape::onRefCountZero()
{
	NpFactory::getInstance().onShapeRelease(this);
	// see NpShape.h for ref counting semantics for shapes
	NpDestroyShape(this);
}

// PX_SERIALIZATION

NpShape::NpShape(PxBaseFlags baseFlags) : PxShape(baseFlags), NpBase(PxEmpty), mCore(PxEmpty), mQueryFilterData(PxEmpty)
{
}

void NpShape::preExportDataReset()
{
	RefCountable_preExportDataReset(*this);
	mExclusiveShapeActor = NULL;
	mFreeSlot = 0;
}

void NpShape::exportExtraData(PxSerializationContext& context)
{	
	mCore.exportExtraData(context);
	context.writeName(mCore.mName);
}

void NpShape::importExtraData(PxDeserializationContext& context)
{
	mCore.importExtraData(context);
	context.readName(mCore.mName);
}

void NpShape::requiresObjects(PxProcessPxBaseCallback& c)
{
	//meshes
	PxBase* mesh = NULL;
	const PxGeometry& geometry = mCore.getGeometry();
	switch(PxU32(mCore.getGeometryType()))
	{
	case PxGeometryType::eCONVEXMESH:		mesh = static_cast<const PxConvexMeshGeometry&>(geometry).convexMesh;			break;
	case PxGeometryType::eHEIGHTFIELD:		mesh = static_cast<const PxHeightFieldGeometry&>(geometry).heightField;			break;
	case PxGeometryType::eTRIANGLEMESH:		mesh = static_cast<const PxTriangleMeshGeometry&>(geometry).triangleMesh;		break;
	case PxGeometryType::eTETRAHEDRONMESH:	mesh = static_cast<const PxTetrahedronMeshGeometry&>(geometry).tetrahedronMesh;	break;
	}
	
	if(mesh)
		c.process(*mesh);

	//material
	const PxU32 nbMaterials = scGetNbMaterials();
	for (PxU32 i=0; i < nbMaterials; i++)
	{
		NpMaterial* mat = scGetMaterial<NpMaterial>(i);
		c.process(*mat);
	}
}

void NpShape::resolveReferences(PxDeserializationContext& context)
{	
	// getMaterials() only works after material indices have been patched. 
	// in order to get to the new material indices, we need access to the new materials.
	// this only leaves us with the option of acquiring the material through the context given an old material index (we do have the mapping)
	{
		PxU32 nbIndices = mCore.getNbMaterialIndices();
		const PxU16* indices = mCore.getMaterialIndices();

		for (PxU32 i=0; i < nbIndices; i++)
		{
			PxBase* base = context.resolveReference(PX_SERIAL_REF_KIND_MATERIAL_IDX, size_t(indices[i]));
			PX_ASSERT(base && base->is<PxMaterial>());

			NpMaterial& material = *static_cast<NpMaterial*>(base);
			mCore.resolveMaterialReference(i, material.mMaterial.mMaterialIndex);
		}
	}

	//we don't resolve mExclusiveShapeActor because it's set to NULL on export.
	//it's recovered when the actors resolveReferences attaches to the shape

	mCore.resolveReferences(context);	

	incMeshRefCount();

	// Increment materials' refcounts in a second pass. Works better in case of failure above.
	const PxU32 nbMaterials = scGetNbMaterials();
	for (PxU32 i=0; i < nbMaterials; i++)
		RefCountable_incRefCount(*scGetMaterial<NpMaterial>(i));
}

NpShape* NpShape::createObject(PxU8*& address, PxDeserializationContext& context)
{
	NpShape* obj = PX_PLACEMENT_NEW(address, NpShape(PxBaseFlag::eIS_RELEASABLE));
	address += sizeof(NpShape);	
	obj->importExtraData(context);
	obj->resolveReferences(context);
	return obj;
}
//~PX_SERIALIZATION

PxU32 NpShape::getReferenceCount() const
{
	return RefCountable_getRefCount(*this);
}

void NpShape::acquireReference()
{
	RefCountable_incRefCount(*this);
}

void NpShape::release()
{
	NP_WRITE_CHECK(getNpScene());
	PX_CHECK_AND_RETURN(RefCountable_getRefCount(*this) > 1 || getActorCount() == 0, "PxShape::release: last reference to a shape released while still attached to an actor!");

	releaseInternal();
}

void NpShape::releaseInternal()
{
	RefCountable_decRefCount(*this);
}

Sc::RigidCore& NpShape::getScRigidObjectExclusive() const
{
	const PxType actorType = mExclusiveShapeActor->getConcreteType();

	if (actorType == PxConcreteType::eRIGID_DYNAMIC)
		return static_cast<NpRigidDynamic&>(*mExclusiveShapeActor).getCore();
	else if (actorType == PxConcreteType::eARTICULATION_LINK)
		return static_cast<NpArticulationLink&>(*mExclusiveShapeActor).getCore();
	else
		return static_cast<NpRigidStatic&>(*mExclusiveShapeActor).getCore();
}

void NpShape::updateSQ(const char* errorMessage)
{
	PxRigidActor* actor = getActor();
	if(actor && (mCore.getFlags() & PxShapeFlag::eSCENE_QUERY_SHAPE))
	{
		NpScene* scene = NpActor::getNpSceneFromActor(*actor);
		NpShapeManager* shapeManager = NpActor::getShapeManager_(*actor);
		if(scene)
			shapeManager->markShapeForSQUpdate(scene->getSQAPI(), *this, static_cast<const PxRigidActor&>(*actor));

		// invalidate the pruning structure if the actor bounds changed
		if(shapeManager->getPruningStructure())
		{
			outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, errorMessage);
			shapeManager->getPruningStructure()->invalidate(mExclusiveShapeActor);
		}
	}
}

#if PX_CHECKED
bool checkShape(const PxGeometry& g, const char* errorMsg);
#endif

void NpShape::setGeometry(const PxGeometry& g)
{
	NpScene* ownerScene = getNpScene();
	NP_WRITE_CHECK(ownerScene);
	PX_CHECK_AND_RETURN(isWritable(), "PxShape::setGeometry: shared shapes attached to actors are not writable.");
#if PX_CHECKED
	if(!checkShape(g, "PxShape::setGeometry(): Invalid geometry!"))
		return;
#endif

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(ownerScene, "PxShape::setGeometry() not allowed while simulation is running. Call will be ignored.")

	// PT: fixes US2117
	if(g.getType() != getGeometryTypeFast())
	{
		outputError<PxErrorCode::eINVALID_PARAMETER>(__LINE__, "PxShape::setGeometry(): Invalid geometry type. Changing the type of the shape is not supported.");
		return;
	}

	PX_SIMD_GUARD;

	//Do not decrement ref count here, but instead cache the refcountable mesh pointer if we had one.
	//We instead decrement the ref counter after incrementing the ref counter on the new geometry.
	//This resolves a case where the user changed a property of a mesh geometry and set the same mesh geometry. If the user had
	//called release() on the mesh, the ref count could hit 0 and be destroyed and then crash when we call incMeshRefCount().
	PxRefCounted* mesh = getMeshRefCountable();

	{
		Sc::RigidCore* rigidCore = getScRigidObjectSLOW();

		if(rigidCore)
			rigidCore->unregisterShapeFromNphase(mCore);

		mCore.setGeometry(g);	

		if (rigidCore)
		{
			rigidCore->registerShapeInNphase(mCore);
			rigidCore->onShapeChange(mCore, Sc::ShapeChangeNotifyFlag::eGEOMETRY);
		}

#if PX_SUPPORT_PVD
		NpScene* npScene = getNpScene();
		if(npScene)
			npScene->getScenePvdClientInternal().releaseAndRecreateGeometry(this);
#endif
	}

	incMeshRefCount();

	if(mesh)
		RefCountable_decRefCount(*mesh);

	updateSQ("PxShape::setGeometry: Shape is a part of pruning structure, pruning structure is now invalid!");
}

const PxGeometry& NpShape::getGeometry() const
{
	NP_READ_CHECK(getNpScene());
	return mCore.getGeometry();
}

PxRigidActor* NpShape::getActor() const
{
	NP_READ_CHECK(getNpScene());
	return mExclusiveShapeActor ? mExclusiveShapeActor->is<PxRigidActor>() : NULL;
}

void NpShape::setLocalPose(const PxTransform& newShape2Actor)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);
	PX_CHECK_AND_RETURN(newShape2Actor.isSane(), "PxShape::setLocalPose: pose is not valid.");
	PX_CHECK_AND_RETURN(isWritable(), "PxShape::setLocalPose: shared shapes attached to actors are not writable.");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxShape::setLocalPose() not allowed while simulation is running. Call will be ignored.");

	PxTransform normalizedTransform = newShape2Actor.getNormalized();
	mCore.setShape2Actor(normalizedTransform);

	notifyActorAndUpdatePVD(Sc::ShapeChangeNotifyFlag::eSHAPE2BODY);

	OMNI_PVD_SET(PxShape, translation, static_cast<PxShape &>(*this), normalizedTransform.p)
	OMNI_PVD_SET(PxShape, rotation, static_cast<PxShape &>(*this), normalizedTransform.q)
	updateSQ("PxShape::setLocalPose: Shape is a part of pruning structure, pruning structure is now invalid!");
}

PxTransform NpShape::getLocalPose() const
{
	NP_READ_CHECK(getNpScene());
	return mCore.getShape2Actor();
}

///////////////////////////////////////////////////////////////////////////////

void NpShape::setSimulationFilterData(const PxFilterData& data)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);
	PX_CHECK_AND_RETURN(isWritable(), "PxShape::setSimulationFilterData: shared shapes attached to actors are not writable.");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxShape::setSimulationFilterData() not allowed while simulation is running. Call will be ignored.")

	mCore.setSimulationFilterData(data);

	notifyActorAndUpdatePVD(Sc::ShapeChangeNotifyFlag::eFILTERDATA);

	OMNI_PVD_SET(PxShape, simulationFilterData, static_cast<PxShape&>(*this), data);
}

PxFilterData NpShape::getSimulationFilterData() const
{
	NP_READ_CHECK(getNpScene());
	return mCore.getSimulationFilterData();
}

void NpShape::setQueryFilterData(const PxFilterData& data)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);
	PX_CHECK_AND_RETURN(isWritable(), "PxShape::setQueryFilterData: shared shapes attached to actors are not writable.");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxShape::setQueryFilterData() not allowed while simulation is running. Call will be ignored.")

	OMNI_PVD_SET(PxShape, queryFilterData, static_cast<PxShape&>(*this), data);

	mQueryFilterData = data;
	UPDATE_PVD_PROPERTY
}

PxFilterData NpShape::getQueryFilterData() const
{
	NP_READ_CHECK(getNpScene());
	return getQueryFilterDataFast();
}

///////////////////////////////////////////////////////////////////////////////

template<typename PxMaterialType, typename NpMaterialType> 
void NpShape::setMaterialsInternal(PxMaterialType* const * materials, PxU16 materialCount)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);
	PX_CHECK_AND_RETURN(isWritable(), "PxShape::setMaterials: shared shapes attached to actors are not writable.");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxShape::setMaterials() not allowed while simulation is running. Call will be ignored.")

#if PX_CHECKED
	if (!NpShape::checkMaterialSetup(mCore.getGeometry(), "PxShape::setMaterials()", materials, materialCount))
		return;
#endif

	const PxU32 oldMaterialCount = scGetNbMaterials();
	PX_ALLOCA(oldMaterials, PxMaterialType*, oldMaterialCount);
	PxU32 tmp = scGetMaterials<PxMaterialType, NpMaterialType>(oldMaterials, oldMaterialCount);
	PX_ASSERT(tmp == oldMaterialCount);
	PX_UNUSED(tmp);

	const bool ret = setMaterialsHelper<PxMaterialType, NpMaterialType>(materials, materialCount);
#if PX_SUPPORT_PVD
	if (npScene)
		npScene->getScenePvdClientInternal().updateMaterials(this);
#endif

#if PX_SUPPORT_OMNI_PVD
	streamShapeMaterials(static_cast<PxShape*>(this), materials, materialCount);
#endif

	if (ret)
	{
		for (PxU32 i = 0; i < materialCount; i++)
			RefCountable_incRefCount(*materials[i]);

		for (PxU32 i = 0; i < oldMaterialCount; i++)
			RefCountable_decRefCount(*oldMaterials[i]);
	}
}

void NpShape::setMaterials(PxMaterial*const* materials, PxU16 materialCount)
{
	PX_CHECK_AND_RETURN(!(mCore.getCore().mShapeCoreFlags & PxShapeCoreFlag::eSOFT_BODY_SHAPE), "NpShape::setMaterials: cannot set rigid body materials to a soft body shape!");
	PX_CHECK_AND_RETURN(!(mCore.getCore().mShapeCoreFlags & PxShapeCoreFlag::eCLOTH_SHAPE), "NpShape::setMaterials: cannot set rigid body materials to a cloth shape!");
	setMaterialsInternal<PxMaterial, NpMaterial>(materials, materialCount);
}

void NpShape::setSoftBodyMaterials(PxFEMSoftBodyMaterial*const* materials, PxU16 materialCount)
{
#if PX_SUPPORT_GPU_PHYSX
	PX_CHECK_AND_RETURN((mCore.getCore().mShapeCoreFlags & PxShapeCoreFlag::eSOFT_BODY_SHAPE), "NpShape::setMaterials: can only apply soft body materials to a soft body shape!");

	setMaterialsInternal<PxFEMSoftBodyMaterial, NpFEMSoftBodyMaterial>(materials, materialCount);
	if (this->mExclusiveShapeActor)
	{
		static_cast<NpSoftBody*>(mExclusiveShapeActor)->updateMaterials();
	}
#else
	PX_UNUSED(materials);
	PX_UNUSED(materialCount);
#endif
}

void NpShape::setClothMaterials(PxFEMClothMaterial*const* materials, PxU16 materialCount)
{
#if PX_SUPPORT_GPU_PHYSX && PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
	PX_CHECK_AND_RETURN((mCore.getCore().mShapeCoreFlags & PxShapeCoreFlag::eCLOTH_SHAPE), "NpShape::setMaterials: can only apply cloth materials to a cloth shape!");

	setMaterialsInternal<PxFEMClothMaterial, NpFEMClothMaterial>(materials, materialCount);
#else
	PX_UNUSED(materials);
	PX_UNUSED(materialCount);
#endif
}

PxU16 NpShape::getNbMaterials() const
{
	NP_READ_CHECK(getNpScene());
	return scGetNbMaterials();
}

PxU32 NpShape::getMaterials(PxMaterial** userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
	NP_READ_CHECK(getNpScene());
	return scGetMaterials<PxMaterial, NpMaterial>(userBuffer, bufferSize, startIndex);
}

#if PX_SUPPORT_GPU_PHYSX
PxU32 NpShape::getSoftBodyMaterials(PxFEMSoftBodyMaterial** userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
	NP_READ_CHECK(getNpScene());
	return scGetMaterials<PxFEMSoftBodyMaterial, NpFEMSoftBodyMaterial>(userBuffer, bufferSize, startIndex);
}
#else
PxU32 NpShape::getSoftBodyMaterials(PxFEMSoftBodyMaterial**, PxU32, PxU32) const
{
	return 0;
}
#endif

#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION && PX_SUPPORT_GPU_PHYSX
PxU32 NpShape::getClothMaterials(PxFEMClothMaterial** userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
	NP_READ_CHECK(getNpScene());
	return scGetMaterials<PxFEMClothMaterial, NpFEMClothMaterial>(userBuffer, bufferSize, startIndex);
}
#else
PxU32 NpShape::getClothMaterials(PxFEMClothMaterial**, PxU32, PxU32) const
{
	return 0;
}
#endif

PxBaseMaterial* NpShape::getMaterialFromInternalFaceIndex(PxU32 faceIndex) const
{
	NP_READ_CHECK(getNpScene());

	const PxGeometry& geom = mCore.getGeometry();
	const PxGeometryType::Enum geomType = geom.getType();
	bool isHf = (geomType == PxGeometryType::eHEIGHTFIELD);
	bool isMesh = (geomType == PxGeometryType::eTRIANGLEMESH);
	
	// if SDF tri-mesh, where no multi-material setup is allowed, return zero-index material
	if (isMesh)
	{
		const PxTriangleMeshGeometry& triGeo = static_cast<const PxTriangleMeshGeometry&>(geom);
		if (triGeo.triangleMesh->getSDF())
		{
			return getMaterial<PxMaterial, NpMaterial>(0);
		}
	}

	if( faceIndex == 0xFFFFffff && (isHf || isMesh) )
	{
		outputError<PxErrorCode::eDEBUG_WARNING>(__LINE__, "PxShape::getMaterialFromInternalFaceIndex received 0xFFFFffff as input - returning NULL.");
		return NULL;
	}

	PxMaterialTableIndex hitMatTableId = 0;

	if(isHf)
	{
		const PxHeightFieldGeometry& hfGeom = static_cast<const PxHeightFieldGeometry&>(geom);

		hitMatTableId = hfGeom.heightField->getTriangleMaterialIndex(faceIndex);
	}
	else if(isMesh)
	{
		const PxTriangleMeshGeometry& triGeo = static_cast<const PxTriangleMeshGeometry&>(geom);

		Gu::TriangleMesh* tm = static_cast<Gu::TriangleMesh*>(triGeo.triangleMesh);
		if(tm->hasPerTriangleMaterials())
			hitMatTableId = triGeo.triangleMesh->getTriangleMaterialIndex(faceIndex);
	}

	// PT: TODO: what's going on here?
	return getMaterial<PxMaterial, NpMaterial>(hitMatTableId);
}

void NpShape::setContactOffset(PxReal contactOffset)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);
	PX_CHECK_AND_RETURN(PxIsFinite(contactOffset), "PxShape::setContactOffset: invalid float");
	PX_CHECK_AND_RETURN((contactOffset >= 0.0f && contactOffset > mCore.getRestOffset()), "PxShape::setContactOffset: contactOffset should be positive, and greater than restOffset!");
	PX_CHECK_AND_RETURN(isWritable(), "PxShape::setContactOffset: shared shapes attached to actors are not writable.");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxShape::setContactOffset() not allowed while simulation is running. Call will be ignored.")

	mCore.setContactOffset(contactOffset);

	notifyActorAndUpdatePVD(Sc::ShapeChangeNotifyFlag::eCONTACTOFFSET);

	OMNI_PVD_SET(PxShape, contactOffset, static_cast<PxShape&>(*this), contactOffset)
}

PxReal NpShape::getContactOffset() const
{
	NP_READ_CHECK(getNpScene());
	return mCore.getContactOffset();
}

void NpShape::setRestOffset(PxReal restOffset)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);
	PX_CHECK_AND_RETURN(PxIsFinite(restOffset), "PxShape::setRestOffset: invalid float");
	PX_CHECK_AND_RETURN((restOffset < mCore.getContactOffset()), "PxShape::setRestOffset: restOffset should be less than contactOffset!");
	PX_CHECK_AND_RETURN(isWritable(), "PxShape::setRestOffset: shared shapes attached to actors are not writable.");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxShape::setRestOffset() not allowed while simulation is running. Call will be ignored.")

	mCore.setRestOffset(restOffset);

	notifyActorAndUpdatePVD(Sc::ShapeChangeNotifyFlag::eRESTOFFSET);

	OMNI_PVD_SET(PxShape, restOffset, static_cast<PxShape&>(*this), restOffset)
}

PxReal NpShape::getRestOffset() const
{
	NP_READ_CHECK(getNpScene());
	return mCore.getRestOffset();
}

void NpShape::setDensityForFluid(PxReal densityForFluid)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);
	PX_CHECK_AND_RETURN(PxIsFinite(densityForFluid), "PxShape::setDensityForFluid: invalid float");
	PX_CHECK_AND_RETURN(isWritable(), "PxShape::setDensityForFluid: shared shapes attached to actors are not writable.");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxShape::setDensityForFluid() not allowed while simulation is running. Call will be ignored.")

	mCore.setDensityForFluid(densityForFluid);

	///notifyActorAndUpdatePVD(Sc::ShapeChangeNotifyFlag::eRESTOFFSET);

	OMNI_PVD_SET(PxShape, densityForFluid, static_cast<PxShape &>(*this), densityForFluid);
}

PxReal NpShape::getDensityForFluid() const
{
	NP_READ_CHECK(getNpScene());
	return mCore.getDensityForFluid();
}

void NpShape::setTorsionalPatchRadius(PxReal radius)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);
	PX_CHECK_AND_RETURN(PxIsFinite(radius), "PxShape::setTorsionalPatchRadius: invalid float");
	PX_CHECK_AND_RETURN((radius >= 0.f), "PxShape::setTorsionalPatchRadius: must be >= 0.f");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxShape::setTorsionalPatchRadius() not allowed while simulation is running. Call will be ignored.")

//	const PxShapeFlags oldShapeFlags = mShape.getFlags();
	mCore.setTorsionalPatchRadius(radius);

	OMNI_PVD_SET(PxShape, torsionalPatchRadius, static_cast<PxShape &>(*this), radius);

// shared shapes return NULL. But shared shapes aren't mutable when attached to an actor, so no notification needed.
//	Sc::RigidCore* rigidCore = NpShapeGetScRigidObjectFromScSLOW();
//	if(rigidCore)
//		rigidCore->onShapeChange(mShape, Sc::ShapeChangeNotifyFlag::eFLAGS, oldShapeFlags);

	UPDATE_PVD_PROPERTY
}

PxReal NpShape::getTorsionalPatchRadius() const
{
	NP_READ_CHECK(getNpScene());
	return mCore.getTorsionalPatchRadius();
}

void NpShape::setMinTorsionalPatchRadius(PxReal radius)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);
	PX_CHECK_AND_RETURN(PxIsFinite(radius), "PxShape::setMinTorsionalPatchRadius: invalid float");
	PX_CHECK_AND_RETURN((radius >= 0.f), "PxShape::setMinTorsionalPatchRadius: must be >= 0.f");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxShape::setMinTorsionalPatchRadius() not allowed while simulation is running. Call will be ignored.")

//	const PxShapeFlags oldShapeFlags = mShape.getFlags();
	mCore.setMinTorsionalPatchRadius(radius);

	OMNI_PVD_SET(PxShape, minTorsionalPatchRadius, static_cast<PxShape &>(*this), radius);

//	Sc::RigidCore* rigidCore = NpShapeGetScRigidObjectFromSbSLOW();
//	if(rigidCore)
//		rigidCore->onShapeChange(mShape, Sc::ShapeChangeNotifyFlag::eFLAGS, oldShapeFlags);

	UPDATE_PVD_PROPERTY
}

PxReal NpShape::getMinTorsionalPatchRadius() const
{
	NP_READ_CHECK(getNpScene());	
	return mCore.getMinTorsionalPatchRadius();
}

PxU32 NpShape::getInternalShapeIndex() const
{
	NP_READ_CHECK(getNpScene());
	if (getNpScene())
	{
		PxsSimulationController* simulationController = getNpScene()->getSimulationController();
		if (simulationController)
			return mCore.getInternalShapeIndex(*simulationController);
	}
	return PX_INVALID_NODE;
}

void NpShape::setFlagsInternal(PxShapeFlags inFlags)
{
	const bool hasMeshTypeGeom = mCore.getGeometryType() == PxGeometryType::eTRIANGLEMESH || mCore.getGeometryType() == PxGeometryType::eHEIGHTFIELD;

	if(hasMeshTypeGeom && (inFlags & PxShapeFlag::eTRIGGER_SHAPE))
	{
		outputError<PxErrorCode::eINVALID_PARAMETER>(__LINE__, "PxShape::setFlag(s): triangle mesh and heightfield triggers are not supported!");
		return;
	}

	if((inFlags & PxShapeFlag::eSIMULATION_SHAPE) && (inFlags & PxShapeFlag::eTRIGGER_SHAPE))
	{
		outputError<PxErrorCode::eINVALID_PARAMETER>(__LINE__, "PxShape::setFlag(s): shapes cannot simultaneously be trigger shapes and simulation shapes.");
		return;
	}

	const PxShapeFlags oldFlags = mCore.getFlags();

	const bool oldIsSimShape = oldFlags & PxShapeFlag::eSIMULATION_SHAPE;
	const bool isSimShape = inFlags & PxShapeFlag::eSIMULATION_SHAPE;

	if(mExclusiveShapeActor)
	{
		const PxType type = mExclusiveShapeActor->getConcreteType();

		// PT: US5732 - support kinematic meshes
		bool isKinematic = false;
		if(type==PxConcreteType::eRIGID_DYNAMIC)
		{
			PxRigidDynamic* rigidDynamic = static_cast<PxRigidDynamic*>(mExclusiveShapeActor);
			isKinematic = rigidDynamic->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC;
		}

		if((type != PxConcreteType::eRIGID_STATIC) && !isKinematic && isSimShape && !oldIsSimShape && (hasMeshTypeGeom || mCore.getGeometryType() == PxGeometryType::ePLANE))
		{
			outputError<PxErrorCode::eINVALID_PARAMETER>(__LINE__, "PxShape::setFlag(s): triangle mesh, heightfield and plane shapes can only be simulation shapes if part of a PxRigidStatic!");
			return;
		}
	}

	const bool oldHasSceneQuery = oldFlags & PxShapeFlag::eSCENE_QUERY_SHAPE;
	const bool hasSceneQuery = inFlags & PxShapeFlag::eSCENE_QUERY_SHAPE;

	{
		PX_ASSERT(!isAPIWriteForbidden());
		const PxShapeFlags oldShapeFlags = mCore.getFlags();
		mCore.setFlags(inFlags);

		notifyActorAndUpdatePVD(oldShapeFlags);
	}

	PxRigidActor* actor = getActor();
	if(oldHasSceneQuery != hasSceneQuery && actor)
	{
		NpScene* npScene = getNpScene();
		NpShapeManager* shapeManager = NpActor::getShapeManager_(*actor);
		if(npScene)
		{
			if(hasSceneQuery)
			{
				// PT: SQ_CODEPATH3
				shapeManager->setupSceneQuery(npScene->getSQAPI(), NpActor::getFromPxActor(*actor), *actor, *this);
			}
			else
			{
				shapeManager->teardownSceneQuery(npScene->getSQAPI(), *actor, *this);
			}
		}

		// invalidate the pruning structure if the actor bounds changed
		if(shapeManager->getPruningStructure())
		{
			outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, "PxShape::setFlag: Shape is a part of pruning structure, pruning structure is now invalid!");
			shapeManager->getPruningStructure()->invalidate(mExclusiveShapeActor);
		}
	}
}

void NpShape::setFlag(PxShapeFlag::Enum flag, bool value)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);
	PX_CHECK_AND_RETURN(isWritable(), "PxShape::setFlag: shared shapes attached to actors are not writable.");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxShape::setFlag() not allowed while simulation is running. Call will be ignored.")

	PX_SIMD_GUARD;

	PxShapeFlags shapeFlags = mCore.getFlags();
	shapeFlags = value ? shapeFlags | flag : shapeFlags & ~flag;
	
	setFlagsInternal(shapeFlags);

	OMNI_PVD_SET(PxShape, shapeFlags, static_cast<PxShape&>(*this), shapeFlags);
}

void NpShape::setFlags(PxShapeFlags inFlags)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);
	PX_CHECK_AND_RETURN(isWritable(), "PxShape::setFlags: shared shapes attached to actors are not writable.");

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxShape::setFlags() not allowed while simulation is running. Call will be ignored.")

	PX_SIMD_GUARD;

	setFlagsInternal(inFlags);

	OMNI_PVD_SET(PxShape, shapeFlags, static_cast<PxShape&>(*this), inFlags);
}

PxShapeFlags NpShape::getFlags() const
{
	NP_READ_CHECK(getNpScene());
	return mCore.getFlags();
}

bool NpShape::isExclusive() const
{
	NP_READ_CHECK(getNpScene());
	return isExclusiveFast();
}

void NpShape::onActorAttach(PxActor& actor)
{
	RefCountable_incRefCount(*this);
	if(isExclusiveFast())
		mExclusiveShapeActor = &actor;
	increaseActorCount(&mFreeSlot);
}

void NpShape::onActorDetach()
{
	PX_ASSERT(getActorCount() > 0);
	decreaseActorCount(&mFreeSlot);
	if(isExclusiveFast())
		mExclusiveShapeActor = NULL;
	RefCountable_decRefCount(*this);
}

void NpShape::incActorCount()
{
	RefCountable_incRefCount(*this);
	increaseActorCount(&mFreeSlot);
}

void NpShape::decActorCount()
{
	PX_ASSERT(getActorCount() > 0);
	decreaseActorCount(&mFreeSlot);
	RefCountable_decRefCount(*this);
}

void NpShape::setName(const char* debugName)		
{
	NP_WRITE_CHECK(getNpScene());
	PX_CHECK_AND_RETURN(isWritable(), "PxShape::setName: shared shapes attached to actors are not writable.");

	mCore.mName = debugName;

	UPDATE_PVD_PROPERTY
}

const char* NpShape::getName() const
{
	NP_READ_CHECK(getNpScene());
	return mCore.mName;
}

///////////////////////////////////////////////////////////////////////////////

// see NpConvexMesh.h, NpHeightField.h, NpTriangleMesh.h for details on how ref counting works for meshes
PxRefCounted* NpShape::getMeshRefCountable()
{
	const PxGeometry& geometry = mCore.getGeometry();
	switch(PxU32(mCore.getGeometryType()))
	{
		case PxGeometryType::eCONVEXMESH:		return static_cast<const PxConvexMeshGeometry&>(geometry).convexMesh;
		case PxGeometryType::eHEIGHTFIELD:		return static_cast<const PxHeightFieldGeometry&>(geometry).heightField;
		case PxGeometryType::eTRIANGLEMESH:		return static_cast<const PxTriangleMeshGeometry&>(geometry).triangleMesh;
		case PxGeometryType::eTETRAHEDRONMESH:	return static_cast<const PxTetrahedronMeshGeometry&>(geometry).tetrahedronMesh;
		default:
			break;
	}
	return NULL;
}

bool NpShape::isWritable()
{
	// a shape is writable if it's exclusive, or it's not connected to any actors (which is true if the ref count is 1 and the user ref is not released.)
	return isExclusiveFast() || (RefCountable_getRefCount(*this)==1 && (mBaseFlags & PxBaseFlag::eIS_RELEASABLE));
}

void NpShape::incMeshRefCount()
{
	PxRefCounted* mesh = getMeshRefCountable();
	if(mesh)
		RefCountable_incRefCount(*mesh);
}

void NpShape::decMeshRefCount()
{
	PxRefCounted* mesh = getMeshRefCountable();
	if(mesh)
		RefCountable_decRefCount(*mesh);
}

///////////////////////////////////////////////////////////////////////////////

template <typename PxMaterialType, typename NpMaterialType>
bool NpShape::setMaterialsHelper(PxMaterialType* const* materials, PxU16 materialCount)
{
	PX_ASSERT(!isAPIWriteForbidden());

	if(materialCount == 1)
	{
		const PxU16 materialIndex = static_cast<NpMaterialType*>(materials[0])->mMaterial.mMaterialIndex;

		mCore.setMaterialIndices(&materialIndex, 1);
	}
	else
	{
		PX_ASSERT(materialCount > 1);

		PX_ALLOCA(materialIndices, PxU16, materialCount);

		if(materialIndices)
		{
			NpMaterialType::getMaterialIndices(materials, materialIndices, materialCount);
			mCore.setMaterialIndices(materialIndices, materialCount);
		}
		else
			return outputError<PxErrorCode::eOUT_OF_MEMORY>(__LINE__, "PxShape::setMaterials() failed. Out of memory. Call will be ignored.");
	}

	NpScene* npScene = getNpScene();
	if(npScene)
		npScene->getScScene().notifyNphaseOnUpdateShapeMaterial(mCore);

	return true;
}

void NpShape::notifyActorAndUpdatePVD(Sc::ShapeChangeNotifyFlags notifyFlags)
{
	// shared shapes return NULL. But shared shapes aren't mutable when attached to an actor, so no notification needed.
	if(mExclusiveShapeActor)
	{
		Sc::RigidCore* rigidCore = getScRigidObjectSLOW();
		if(rigidCore)
			rigidCore->onShapeChange(mCore, notifyFlags);

#if PX_SUPPORT_GPU_PHYSX
		const PxType type = mExclusiveShapeActor->getConcreteType();
	#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
		if(type==PxConcreteType::eFEM_CLOTH)
			static_cast<NpFEMCloth*>(mExclusiveShapeActor)->getCore().onShapeChange(mCore, notifyFlags);
	#endif
		if(type==PxConcreteType::eSOFT_BODY)
			static_cast<NpSoftBody*>(mExclusiveShapeActor)->getCore().onShapeChange(mCore, notifyFlags);
#endif
	}

	UPDATE_PVD_PROPERTY
}

void NpShape::notifyActorAndUpdatePVD(const PxShapeFlags oldShapeFlags)
{
	// shared shapes return NULL. But shared shapes aren't mutable when attached to an actor, so no notification needed.
	Sc::RigidCore* rigidCore = getScRigidObjectSLOW();
	if(rigidCore)
		rigidCore->onShapeFlagsChange(mCore, oldShapeFlags);

	UPDATE_PVD_PROPERTY
}
