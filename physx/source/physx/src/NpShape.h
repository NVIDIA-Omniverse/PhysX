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

#ifndef NP_SHAPE_H
#define NP_SHAPE_H

#include "common/PxMetaData.h"
#include "PxShape.h"
#include "NpBase.h"
#include "ScShapeCore.h"
#include "NpPhysics.h"
#include "CmPtrTable.h"

namespace physx
{
class NpScene;

class NpShape : public PxShape, public NpBase
{
public:
// PX_SERIALIZATION
												NpShape(PxBaseFlags baseFlags);
					void						preExportDataReset();
	virtual			void						exportExtraData(PxSerializationContext& context);
					void						importExtraData(PxDeserializationContext& context);
	virtual			void						requiresObjects(PxProcessPxBaseCallback& c);
					void						resolveReferences(PxDeserializationContext& context);
	static			NpShape*					createObject(PxU8*& address, PxDeserializationContext& context);
	static			void						getBinaryMetaData(PxOutputStream& stream);
//~PX_SERIALIZATION
												NpShape(const PxGeometry& geometry,
													PxShapeFlags shapeFlags,
													const PxU16* materialIndices,
													PxU16 materialCount, 
													bool isExclusive,
													PxShapeCoreFlag::Enum flag = PxShapeCoreFlag::Enum(0));

	virtual										~NpShape();

	// PxRefCounted
	virtual			PxU32						getReferenceCount() const	PX_OVERRIDE;
	virtual			void						acquireReference()	PX_OVERRIDE;
	//~PxRefCounted

	// PxShape
	virtual			void						release()	PX_OVERRIDE; //!< call to release from actor
	virtual			void						setGeometry(const PxGeometry&)	PX_OVERRIDE;
	virtual			const PxGeometry&			getGeometry() const	PX_OVERRIDE;
	virtual			PxRigidActor*				getActor() const	PX_OVERRIDE;
	virtual			void						setLocalPose(const PxTransform& pose)	PX_OVERRIDE;
	virtual			PxTransform					getLocalPose() const	PX_OVERRIDE;
	virtual			void						setSimulationFilterData(const PxFilterData& data)	PX_OVERRIDE;
	virtual			PxFilterData				getSimulationFilterData() const	PX_OVERRIDE;
	virtual			void						setQueryFilterData(const PxFilterData& data)	PX_OVERRIDE;
	virtual			PxFilterData				getQueryFilterData() const	PX_OVERRIDE;
	virtual			void						setMaterials(PxMaterial*const* materials, PxU16 materialCount)	PX_OVERRIDE;
	virtual			void						setSoftBodyMaterials(PxFEMSoftBodyMaterial*const* materials, PxU16 materialCount)	PX_OVERRIDE;
	virtual			void						setClothMaterials(PxFEMClothMaterial*const* materials, PxU16 materialCount)	PX_OVERRIDE;
	virtual			PxU16						getNbMaterials()															const	PX_OVERRIDE;
	virtual			PxU32						getMaterials(PxMaterial** userBuffer, PxU32 bufferSize, PxU32 startIndex=0)	const	PX_OVERRIDE;
	virtual			PxU32						getSoftBodyMaterials(PxFEMSoftBodyMaterial** userBuffer, PxU32 bufferSize, PxU32 startIndex = 0)	const	PX_OVERRIDE;
	virtual			PxU32						getClothMaterials(PxFEMClothMaterial** userBuffer, PxU32 bufferSize, PxU32 startIndex = 0)	const	PX_OVERRIDE;
	virtual			PxBaseMaterial*				getMaterialFromInternalFaceIndex(PxU32 faceIndex)							const	PX_OVERRIDE;
	virtual			void						setContactOffset(PxReal)	PX_OVERRIDE;
	virtual			PxReal						getContactOffset() const	PX_OVERRIDE;
	virtual			void						setRestOffset(PxReal)	PX_OVERRIDE;
	virtual			PxReal						getRestOffset() const	PX_OVERRIDE;
	virtual			void						setDensityForFluid(PxReal)	PX_OVERRIDE;
	virtual			PxReal						getDensityForFluid() const	PX_OVERRIDE;
	virtual			void						setTorsionalPatchRadius(PxReal)	PX_OVERRIDE;
	virtual			PxReal						getTorsionalPatchRadius() const	PX_OVERRIDE;
	virtual			void						setMinTorsionalPatchRadius(PxReal)	PX_OVERRIDE;
	virtual			PxReal						getMinTorsionalPatchRadius() const	PX_OVERRIDE;
	virtual			PxU32						getInternalShapeIndex() const	PX_OVERRIDE;
	virtual			void						setFlag(PxShapeFlag::Enum flag, bool value)	PX_OVERRIDE;
	virtual			void						setFlags(PxShapeFlags inFlags)	PX_OVERRIDE;
	virtual			PxShapeFlags				getFlags() const	PX_OVERRIDE;
	virtual			bool						isExclusive() const	PX_OVERRIDE;
	virtual			void						setName(const char* debugName)	PX_OVERRIDE;
	virtual			const char*					getName() const	PX_OVERRIDE;
	//~PxShape

	// Ref counting for shapes works like this: 
	// * for exclusive shapes the actor has a counted reference
	// * for shared shapes, each actor has a counted reference, and the user has a counted reference
	// * for either kind, each instance of the shape in a scene (i.e. each shapeSim) causes the reference count to be incremented by 1.
	// Because these semantics aren't clear to users, this reference count should not be exposed in the API

	// PxBase
	virtual			void						onRefCountZero()	PX_OVERRIDE;
	//~PxBase

	PX_FORCE_INLINE	PxShapeFlags				getFlagsFast()			const	{ return mCore.getFlags();									}
	PX_FORCE_INLINE const PxTransform&			getLocalPoseFast()		const	{ return mCore.getShape2Actor();							}
	PX_FORCE_INLINE	PxGeometryType::Enum		getGeometryTypeFast()	const	{ return mCore.getGeometryType();							}
	PX_FORCE_INLINE	const PxFilterData&			getQueryFilterDataFast() const	{ return mQueryFilterData;									}

	PX_FORCE_INLINE PxU32						getActorCount()			const	{ return mFreeSlot;																	}
	PX_FORCE_INLINE bool						isExclusiveFast()		const	{ return mCore.getCore().mShapeCoreFlags.isSet(PxShapeCoreFlag::eIS_EXCLUSIVE);		}

	PX_FORCE_INLINE	const Sc::ShapeCore&		getCore()				const	{ return mCore;	}
	PX_FORCE_INLINE	Sc::ShapeCore&				getCore()						{ return mCore;	}
	static PX_FORCE_INLINE size_t				getCoreOffset()					{ return PX_OFFSET_OF_RT(NpShape, mCore); }


	// PT: TODO: this one only used internally and by NpFactory
	template <typename PxMaterialType, typename NpMaterialType>
	PX_INLINE		PxMaterialType*				getMaterial(PxU32 index) const { return scGetMaterial<NpMaterialType>(index); }

	PX_FORCE_INLINE	void						setSceneIfExclusive(NpScene* s)
												{
													if(isExclusiveFast())
														setNpScene(s);
												}

					void						releaseInternal();	// PT: it's "internal" but called by the NpFactory

#if PX_CHECKED
	template <typename PxMaterialType>
	static			bool						checkMaterialSetup(const PxGeometry& geom, const char* errorMsgPrefix, PxMaterialType*const* materials, PxU16 materialCount);
#endif
					void						onActorAttach(PxActor& actor);
					void						onActorDetach();

					void						incActorCount();
					void						decActorCount();

	//Always returns 0xffffffff for shared shapes.
	PX_FORCE_INLINE PxU32						getShapeManagerArrayIndex(const Cm::PtrTable& shapes)  const
												{
													if(isExclusiveFast())
													{
														PX_ASSERT(isExclusiveFast() || NP_UNUSED_BASE_INDEX == getBaseIndex());
														PX_ASSERT(!isExclusiveFast() || NP_UNUSED_BASE_INDEX != getBaseIndex());
														const PxU32 index = getBaseIndex();
														return index!=NP_UNUSED_BASE_INDEX ? index : 0xffffffff;
													}
													else
														return shapes.find(this);
												}
	PX_FORCE_INLINE bool						checkShapeManagerArrayIndex(const Cm::PtrTable& shapes)  const
												{
													return 
														((!isExclusiveFast() && NP_UNUSED_BASE_INDEX==getBaseIndex()) || 
														((getBaseIndex() < shapes.getCount()) && (shapes.getPtrs()[getBaseIndex()] == this)));
												}
	PX_FORCE_INLINE	void						setShapeManagerArrayIndex(const PxU32 id)
												{
													setBaseIndex(isExclusiveFast() ? id : NP_UNUSED_BASE_INDEX);
												}
	PX_FORCE_INLINE	void						clearShapeManagerArrayIndex()
												{
													setBaseIndex(NP_UNUSED_BASE_INDEX);
												}

private:
					PxActor*					mExclusiveShapeActor;
					Sc::ShapeCore				mCore;
					PxFilterData				mQueryFilterData;	// Query filter data PT: TODO: consider moving this to SQ structures

private:
					void						notifyActorAndUpdatePVD(Sc::ShapeChangeNotifyFlags notifyFlags);
					void						notifyActorAndUpdatePVD(const PxShapeFlags oldShapeFlags);	// PT: for shape flags change
					void						incMeshRefCount();
					void						decMeshRefCount();
					PxRefCounted*				getMeshRefCountable();
					bool						isWritable();
					void						updateSQ(const char* errorMessage);
					template <typename PxMaterialType, typename NpMaterialType>
					bool						setMaterialsHelper(PxMaterialType* const* materials, PxU16 materialCount);
					void						setFlagsInternal(PxShapeFlags inFlags);
					Sc::RigidCore&				getScRigidObjectExclusive() const;
	PX_FORCE_INLINE	Sc::RigidCore*				getScRigidObjectSLOW()
												{
													return NpShape::getActor() ? &getScRigidObjectExclusive() : NULL;
												}

	PX_INLINE		PxU16						scGetNbMaterials() const
												{
													return mCore.getNbMaterialIndices();
												}

	template <typename Material>
	PX_INLINE		Material*					scGetMaterial(PxU32 index) const
												{
													PX_ASSERT(index < scGetNbMaterials());

													NpMaterialManager<Material>& matManager = NpMaterialAccessor<Material>::getMaterialManager(NpPhysics::getInstance());
													// PT: TODO: revisit this indirection
													const PxU16 matTableIndex = mCore.getMaterialIndices()[index];
													return matManager.getMaterial(matTableIndex);
												}

	// PT: TODO: this one only used internally
	template <typename PxMaterialType, typename NpMaterialType>
	PX_INLINE		PxU32						scGetMaterials(PxMaterialType** buffer, PxU32 bufferSize, PxU32 startIndex=0) const
												{
													const PxU16* materialIndices;
													PxU32 matCount;
													NpMaterialManager<NpMaterialType>& matManager = NpMaterialAccessor<NpMaterialType>::getMaterialManager(NpPhysics::getInstance());
													materialIndices = mCore.getMaterialIndices();
													matCount = mCore.getNbMaterialIndices();

													// PT: this is copied from Cm::getArrayOfPointers(). We cannot use the Cm function here
													// because of the extra indirection needed to access the materials.
													PxU32 size = matCount;
													const PxU32 remainder = PxU32(PxMax<PxI32>(PxI32(size - startIndex), 0));
													const PxU32 writeCount = PxMin(remainder, bufferSize);
													materialIndices += startIndex;
													for(PxU32 i=0;i<writeCount;i++)
														buffer[i] = matManager.getMaterial(materialIndices[i]);

													return writeCount;
												}

	template<typename PxMaterialType, typename NpMaterialType> void setMaterialsInternal(PxMaterialType* const * materials, PxU16 materialCount);
};

#if PX_CHECKED
template <typename PxMaterialType>
bool NpShape::checkMaterialSetup(const PxGeometry& geom, const char* errorMsgPrefix, PxMaterialType*const* materials, PxU16 materialCount)
{
	for(PxU32 i=0; i<materialCount; ++i)
	{
		if(!materials[i])
		{
			PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL,
				"material pointer %d is NULL!", i);
			return false;
		}
	}

	if(materialCount > 1)
	{
		const PxGeometryType::Enum type = geom.getType();

		//  verify we provide all materials required
		if(type == PxGeometryType::eTRIANGLEMESH)
		{
			const PxTriangleMeshGeometry& meshGeom = static_cast<const PxTriangleMeshGeometry&>(geom);
			const PxTriangleMesh& mesh = *meshGeom.triangleMesh;

			// do not allow SDF multi-material tri-meshes:
			if(mesh.getSDF())
			{
				PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL,
					"%s: multiple materials defined for an SDF triangle-mesh geometry!", errorMsgPrefix);
				return false;
			}

			const Gu::TriangleMesh& tmesh = static_cast<const Gu::TriangleMesh&>(mesh);
			if(tmesh.hasPerTriangleMaterials())
			{
				const PxU32 nbTris = tmesh.getNbTrianglesFast();
				for(PxU32 i=0; i<nbTris; i++)
				{
					const PxMaterialTableIndex meshMaterialIndex = mesh.getTriangleMaterialIndex(i);
					if(meshMaterialIndex >= materialCount)
					{
						PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL,
							"%s: PxTriangleMesh material indices reference more materials than provided!", errorMsgPrefix);
						break;
					}
				}
			}
			else
			{
				PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL,
					"%s: multiple materials defined for a triangle-mesh that does not have per-triangle materials!", errorMsgPrefix);
			}
		}
		else if(type == PxGeometryType::eTETRAHEDRONMESH)
		{
			const PxTetrahedronMeshGeometry& meshGeom = static_cast<const PxTetrahedronMeshGeometry&>(geom);
			const PxTetrahedronMesh& mesh = *meshGeom.tetrahedronMesh;
			PX_UNUSED(mesh);
			//Need to fill in material
			/*if (mesh.getTriangleMaterialIndex(0) != 0xffff)
			{
				for (PxU32 i = 0; i < mesh.getNbTriangles(); i++)
				{
					const PxMaterialTableIndex meshMaterialIndex = mesh.getTriangleMaterialIndex(i);
					if (meshMaterialIndex >= materialCount)
					{
						PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL,
							"%s: PxTriangleMesh material indices reference more materials than provided!", errorMsgPrefix);
						break;
					}
				}
			}*/
		}
		else if(type == PxGeometryType::eHEIGHTFIELD)
		{
			const PxHeightFieldGeometry& meshGeom = static_cast<const PxHeightFieldGeometry&>(geom);
			const PxHeightField& mesh = *meshGeom.heightField;
			if (mesh.getTriangleMaterialIndex(0) != 0xffff)
			{
				const PxU32 nbTris = mesh.getNbColumns()*mesh.getNbRows() * 2;
				for (PxU32 i = 0; i < nbTris; i++)
				{
					const PxMaterialTableIndex meshMaterialIndex = mesh.getTriangleMaterialIndex(i);
					if (meshMaterialIndex != PxHeightFieldMaterial::eHOLE && meshMaterialIndex >= materialCount)
					{
						PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL,
							"%s: PxHeightField material indices reference more materials than provided!", errorMsgPrefix);
						break;
					}
				}
			}
			else
			{
				PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL,
					"%s: multiple materials defined for a heightfield that does not have per-triangle materials!", errorMsgPrefix);
			}
		}
		else
		{
			// check that simple shapes don't get assigned multiple materials
			PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL,
				"%s: multiple materials defined for single material geometry!", errorMsgPrefix);
			return false;
		}
	}
	return true;
}
#endif

}

#endif
