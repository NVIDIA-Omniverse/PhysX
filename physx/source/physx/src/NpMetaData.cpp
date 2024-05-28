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
#include "PxPhysicsSerialization.h"
#include "NpShape.h"
#include "NpShapeManager.h"
#include "NpConstraint.h"
#include "NpRigidStatic.h"
#include "NpRigidDynamic.h"
#include "NpArticulationReducedCoordinate.h"
#include "NpArticulationLink.h"
#include "NpArticulationJointReducedCoordinate.h"
#include "NpArticulationTendon.h"
#include "NpAggregate.h"
#include "NpPruningStructure.h"
#include "NpMaterial.h"
#include "NpFEMSoftBodyMaterial.h"
#include "NpFEMClothMaterial.h"
#include "NpPBDMaterial.h"
#include "GuConvexMesh.h"
#include "GuTriangleMesh.h"
#include "GuTriangleMeshBV4.h"
#include "GuTriangleMeshRTree.h"
#include "GuHeightField.h"
#include "GuPrunerMergeData.h"

using namespace physx;
using namespace Cm;
using namespace Gu;

///////////////////////////////////////////////////////////////////////////////

// PT: the offsets can be different for different templated classes so I need macros here.

#define DefineMetaData_PxActor(x) \
	PX_DEF_BIN_METADATA_ITEM(stream,	x, void,			userData,		PxMetaDataFlag::ePTR)

#define DefineMetaData_NpRigidActorTemplate(x) \
	PX_DEF_BIN_METADATA_ITEM(stream,	x, NpShapeManager,	mShapeManager,	0)
//	PX_DEF_BIN_METADATA_ITEM(stream,	x, PxU32,			mIndex,			0)

#define DefineMetaData_NpRigidBodyTemplate(x) \
	PX_DEF_BIN_METADATA_ITEM(stream,	x, Sc::BodyCore,	mCore,			0)


///////////////////////////////////////////////////////////////////////////////

static void getBinaryMetaData_PxVec3(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream,	PxVec3)
	PX_DEF_BIN_METADATA_ITEM(stream,	PxVec3,		PxReal,	x,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,	PxVec3,		PxReal,	y,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,	PxVec3,		PxReal,	z,	0)
}

static void getBinaryMetaData_PxVec4(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream,	PxVec4)
	PX_DEF_BIN_METADATA_ITEM(stream,	PxVec4,		PxReal,	x,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,	PxVec4,		PxReal,	y,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,	PxVec4,		PxReal,	z,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,	PxVec4,		PxReal,	w,	0)
} 

static void getBinaryMetaData_PxQuat(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream,	PxQuat)
	PX_DEF_BIN_METADATA_ITEM(stream,	PxQuat,		PxReal,	x,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,	PxQuat,		PxReal,	y,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,	PxQuat,		PxReal,	z,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,	PxQuat,		PxReal,	w,	0)
}

static void getBinaryMetaData_PxBounds3(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream,	PxBounds3)
	PX_DEF_BIN_METADATA_ITEM(stream,	PxBounds3,	PxVec3,	minimum,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,	PxBounds3,	PxVec3,	maximum,	0)
}

static void getBinaryMetaData_PxTransform(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream,	PxTransform)
	PX_DEF_BIN_METADATA_ITEM(stream,	PxTransform,	PxQuat,	q,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,	PxTransform,	PxVec3,	p,	0)
}

static void getBinaryMetaData_PxTransform32(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream,	PxTransform32)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream, PxTransform32, PxTransform)
	PX_DEF_BIN_METADATA_ITEM(stream,	PxTransform32,	PxU32,		padding,	0)
}

static void getBinaryMetaData_PxMat33(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream,	PxMat33)
	PX_DEF_BIN_METADATA_ITEM(stream,	PxMat33,	PxVec3,	column0,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,	PxMat33,	PxVec3,	column1,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,	PxMat33,	PxVec3,	column2,	0)
}

static void getBinaryMetaData_SpatialVectorF(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream, Cm::SpatialVectorF)
	PX_DEF_BIN_METADATA_ITEM(stream, Cm::SpatialVectorF, PxVec3, top, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, Cm::SpatialVectorF, PxReal, pad0, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, Cm::SpatialVectorF, PxVec3, bottom, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, Cm::SpatialVectorF, PxReal, pad1, 0)
}

namespace
{
	class ShadowBitMap : public PxBitMap
	{
	public:
		static void getBinaryMetaData(PxOutputStream& stream_)
		{
			PX_DEF_BIN_METADATA_CLASS(stream_,		ShadowBitMap)
			PX_DEF_BIN_METADATA_ITEM(stream_,		ShadowBitMap, PxU32,		mMap,		PxMetaDataFlag::ePTR)
			PX_DEF_BIN_METADATA_ITEM(stream_,		ShadowBitMap, PxU32,		mWordCount,	0)
			PX_DEF_BIN_METADATA_ITEM(stream_,		ShadowBitMap, PxAllocator,	mAllocator,	0)
			PX_DEF_BIN_METADATA_ITEMS_AUTO(stream_,	ShadowBitMap, PxU8,			mPadding,	PxMetaDataFlag::ePADDING)

			//------ Extra-data ------

			// mMap
			PX_DEF_BIN_METADATA_EXTRA_ARRAY(stream_,	ShadowBitMap, PxU32, mWordCount, PX_SERIAL_ALIGN, PxMetaDataFlag::eCOUNT_MASK_MSB)
		}
	};
}

static void getBinaryMetaData_BitMap(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_TYPEDEF(stream, PxAllocator, PxU8)
	ShadowBitMap::getBinaryMetaData(stream);
	PX_DEF_BIN_METADATA_TYPEDEF(stream,	BitMap, ShadowBitMap)
}

static void getBinaryMetaData_PxPlane(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream,	PxPlane)
	PX_DEF_BIN_METADATA_ITEM(stream,	PxPlane,	PxVec3,	n,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,	PxPlane,	PxReal,	d,	0)	
}

static void getBinaryMetaData_PxConstraintInvMassScale(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream,	PxConstraintInvMassScale)

	PX_DEF_BIN_METADATA_ITEM(stream,	PxConstraintInvMassScale,	PxReal,		linear0,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,	PxConstraintInvMassScale,	PxReal,		angular0,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,	PxConstraintInvMassScale,	PxReal,		linear1,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,	PxConstraintInvMassScale,	PxReal,		angular1,	0)
}

///////////////////////////////////////////////////////////////////////////////

void NpBase::getBinaryMetaData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_TYPEDEF(stream,	NpType::Enum, PxU32)

	PX_DEF_BIN_METADATA_CLASS(stream,	NpBase)

	PX_DEF_BIN_METADATA_ITEM(stream,	NpBase, NpScene,		mScene,				PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream,	NpBase, PxU32,			mBaseIndexAndType,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,	NpBase, PxU32,			mFreeSlot,			0)
}

///////////////////////////////////////////////////////////////////////////////

void NpActor::getBinaryMetaData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream,		NpActor)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	NpActor, NpBase)
	PX_DEF_BIN_METADATA_ITEM(stream,		NpActor, char,				mName,				PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream,		NpActor, NpConnectorArray,	mConnectorArray,	PxMetaDataFlag::ePTR)
}

///////////////////////////////////////////////////////////////////////////////

void NpMaterial::getBinaryMetaData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_VCLASS(stream,		NpMaterial)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	NpMaterial, PxBase)

	PX_DEF_BIN_METADATA_ITEM(stream,		NpMaterial, void,				userData,	PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream,		NpMaterial, PxsMaterialCore,	mMaterial,	0)
}

///////////////////////////////////////////////////////////////////////////////

void NpFEMSoftBodyMaterial::getBinaryMetaData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_VCLASS(stream, NpFEMSoftBodyMaterial)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream, NpFEMSoftBodyMaterial, PxBase)

	PX_DEF_BIN_METADATA_ITEM(stream, NpFEMSoftBodyMaterial, void, userData, PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream, NpFEMSoftBodyMaterial, PxsFEMSoftBodyMaterialCore, mMaterial, 0)
}

///////////////////////////////////////////////////////////////////////////////

#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
void NpFEMClothMaterial::getBinaryMetaData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_VCLASS(stream, NpFEMClothMaterial)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream, NpFEMClothMaterial, PxBase)

	PX_DEF_BIN_METADATA_ITEM(stream, NpFEMClothMaterial, void, userData, PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream, NpFEMClothMaterial, PxsFEMClothMaterialCore, mMaterial, 0)
}
#endif

///////////////////////////////////////////////////////////////////////////////

void NpPBDMaterial::getBinaryMetaData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_VCLASS(stream, NpPBDMaterial)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream, NpPBDMaterial, PxBase)

	PX_DEF_BIN_METADATA_ITEM(stream, NpPBDMaterial, void, userData, PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream, NpPBDMaterial, PxsPBDMaterialCore, mMaterial, 0)
}

///////////////////////////////////////////////////////////////////////////////

void NpConstraint::getBinaryMetaData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_VCLASS(stream,		NpConstraint)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	NpConstraint, PxBase)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	NpConstraint, NpBase)

	// PxConstraint
	PX_DEF_BIN_METADATA_ITEM(stream,		NpConstraint, void,				userData,	PxMetaDataFlag::ePTR)

	// NpConstraint
	PX_DEF_BIN_METADATA_ITEM(stream,		NpConstraint, PxRigidActor,		mActor0,	PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream,		NpConstraint, PxRigidActor,		mActor1,	PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream,		NpConstraint, ConstraintCore,	mCore,		0)
}

///////////////////////////////////////////////////////////////////////////////

void NpShapeManager::getBinaryMetaData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream,	NpShapeManager)
	PX_DEF_BIN_METADATA_ITEM(stream,	NpShapeManager, PtrTable,	mShapes,			0)
//	PX_DEF_BIN_METADATA_ITEM(stream,	NpShapeManager, PxU32,		mSqCompoundId,		0)
	PX_DEF_BIN_METADATA_ITEM(stream,	NpShapeManager, Sq::PruningStructure, mPruningStructure, PxMetaDataFlag::ePTR)
}

///////////////////////////////////////////////////////////////////////////////

void NpShape::getBinaryMetaData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_VCLASS(stream,		NpShape)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	NpShape, PxBase)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	NpShape, NpBase)

	// PxShape
	PX_DEF_BIN_METADATA_ITEM(stream,		NpShape, void,			userData,				PxMetaDataFlag::ePTR)

	// NpShape
	PX_DEF_BIN_METADATA_ITEM(stream,		NpShape, PxRigidActor,	mExclusiveShapeActor,	PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream,		NpShape, ShapeCore,		mCore,					0)
	PX_DEF_BIN_METADATA_ITEM(stream,		NpShape, PxFilterData,	mQueryFilterData,		0)
}

///////////////////////////////////////////////////////////////////////////////

void NpRigidStatic::getBinaryMetaData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_VCLASS(stream,		NpRigidStatic)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	NpRigidStatic, PxBase)
//	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	NpRigidStatic, NpRigidStaticT)	// ### ???
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	NpRigidStatic, NpActor)

	DefineMetaData_PxActor(NpRigidStatic)
	DefineMetaData_NpRigidActorTemplate(NpRigidStatic)

	// NpRigidStatic
	PX_DEF_BIN_METADATA_ITEM(stream,		NpRigidStatic, Sc::StaticCore,	mCore,		0)

	//------ Extra-data ------

	PX_DEF_BIN_METADATA_EXTRA_ITEM(stream,	NpRigidStatic, NpConnectorArray,	mConnectorArray, PX_SERIAL_ALIGN)
	PX_DEF_BIN_METADATA_EXTRA_NAME(stream,	NpRigidStatic, mName, 0)
}

///////////////////////////////////////////////////////////////////////////////

void NpConnector::getBinaryMetaData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream,		NpConnector)
	PX_DEF_BIN_METADATA_ITEM(stream,		NpConnector, PxU8,		mType,		0)
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	NpConnector, PxU8,		mPadding,	PxMetaDataFlag::ePADDING)
	PX_DEF_BIN_METADATA_ITEM(stream,		NpConnector, PxBase,	mObject,	PxMetaDataFlag::ePTR)
}

void NpConnectorArray::getBinaryMetaData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream,		NpConnectorArray)
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	NpConnectorArray, NpConnector,	mBuffer,		0)
	PX_DEF_BIN_METADATA_ITEM(stream,		NpConnectorArray, bool,			mBufferUsed,	0)
	// PT: OMG this is so painful... I can't put the padding explicitly in the template
	{ PxMetaDataEntry tmp = {"char", "mPadding", 1 + PxU32(PX_OFFSET_OF_RT(NpConnectorArray, mBufferUsed)), 3, 3, 0, PxMetaDataFlag::ePADDING, 0}; PX_STORE_METADATA(stream, tmp); }
	PX_DEF_BIN_METADATA_ITEM(stream,		NpConnectorArray, NpConnector,	mData,			PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream,		NpConnectorArray, PxU32,		mSize,			0)
	PX_DEF_BIN_METADATA_ITEM(stream,		NpConnectorArray, PxU32,		mCapacity,		0)

	//------ Extra-data ------

	PX_DEF_BIN_METADATA_EXTRA_ITEMS(stream,	NpConnectorArray, NpConnector, mBufferUsed, mCapacity, PxMetaDataFlag::eCONTROL_FLIP|PxMetaDataFlag::eCOUNT_MASK_MSB, 0)
}

///////////////////////////////////////////////////////////////////////////////

void NpRigidDynamic::getBinaryMetaData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_VCLASS(stream,		NpRigidDynamic)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	NpRigidDynamic, PxBase)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	NpRigidDynamic, NpActor)

	DefineMetaData_PxActor(NpRigidDynamic)
	DefineMetaData_NpRigidActorTemplate(NpRigidDynamic)
	DefineMetaData_NpRigidBodyTemplate(NpRigidDynamic)

	// NpRigidDynamic

	//------ Extra-data ------

// Extra data:
// - inline array from shape manager
// - optional constraint array

//	PX_DEF_BIN_METADATA_ITEM(stream,NpRigidDynamic, NpShapeManager,	mShapeManager.mShapes,	0)

/*
		virtual	void				exportExtraData(PxOutputStream& stream)
				{
					mShapeManager.exportExtraData(stream);
					ActorTemplateClass::exportExtraData(stream);
				}
void NpActorTemplate<APIClass, LeafClass>::exportExtraData(PxOutputStream& stream)					
{
	if(mConnectorArray)
	{
		stream.storeBuffer(mConnectorArray, sizeof(NpConnectorArray)
		mConnectorArray->exportExtraData(stream);
	}
}
*/
	PX_DEF_BIN_METADATA_EXTRA_ITEM(stream,	NpRigidDynamic, NpConnectorArray, mConnectorArray, PX_SERIAL_ALIGN)
//### missing inline array data here... only works for "buffered" arrays so far
/*
	Big issue: we can't output the "offset of" the inline array within the class, since the inline array itself is extra-data. But we need to read
	the array itself to know if it's inline or not (the "is buffered" bool). So we need to read from the extra data!
*/

/*
[17:41:39] Gordon Yeoman nvidia: PxsBodyCore need to be 16-byte aligned for spu.  If it is 128-byte aligned then that is a mistake.  Feel free to change it to 16.
*/
	PX_DEF_BIN_METADATA_EXTRA_NAME(stream,	NpRigidDynamic, mName, 0)
}

///////////////////////////////////////////////////////////////////////////////

void NpArticulationLinkArray::getBinaryMetaData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream,	NpArticulationLinkArray)
	PX_DEF_BIN_METADATA_ITEMS(stream,	NpArticulationLinkArray,	NpArticulationLink,	mBuffer,		PxMetaDataFlag::ePTR, 4)
	PX_DEF_BIN_METADATA_ITEM(stream,	NpArticulationLinkArray,	bool,				mBufferUsed,	0)
	// PT: OMG this is so painful... I can't put the padding explicitely in the template
	{ PxMetaDataEntry tmp = {"char", "mPadding", 1 + PxU32(PX_OFFSET_OF_RT(NpArticulationLinkArray, mBufferUsed)), 3, 3, 0, PxMetaDataFlag::ePADDING, 0}; PX_STORE_METADATA(stream, tmp); }
	PX_DEF_BIN_METADATA_ITEM(stream,	NpArticulationLinkArray,	NpArticulationLink,	mData,		PxMetaDataFlag::ePTR)	// ###
	PX_DEF_BIN_METADATA_ITEM(stream,	NpArticulationLinkArray,	PxU32,				mSize,			0)
	PX_DEF_BIN_METADATA_ITEM(stream,	NpArticulationLinkArray,	PxU32,				mCapacity,		0)

	//------ Extra-data ------

	PX_DEF_BIN_METADATA_EXTRA_ITEMS(stream,	NpArticulationLinkArray, NpArticulationLink, mBufferUsed, mCapacity, PxMetaDataFlag::eCONTROL_FLIP|PxMetaDataFlag::eCOUNT_MASK_MSB|PxMetaDataFlag::ePTR, 0)
}

void NpArticulationAttachmentArray::getBinaryMetaData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream,	NpArticulationAttachmentArray)
	PX_DEF_BIN_METADATA_ITEMS(stream,	NpArticulationAttachmentArray,	NpArticulationAttachment,	mBuffer,		PxMetaDataFlag::ePTR, 4)
	PX_DEF_BIN_METADATA_ITEM(stream,	NpArticulationAttachmentArray,	bool,				mBufferUsed,	0)
	// PT: OMG this is so painful... I can't put the padding explicitely in the template
	{ PxMetaDataEntry tmp = {"char", "mPadding", 1 + PxU32(PX_OFFSET_OF_RT(NpArticulationAttachmentArray, mBufferUsed)), 3, 3, 0, PxMetaDataFlag::ePADDING, 0}; PX_STORE_METADATA(stream, tmp); }
	PX_DEF_BIN_METADATA_ITEM(stream,	NpArticulationAttachmentArray,	NpArticulationAttachment,	mData,		PxMetaDataFlag::ePTR)	// ###
	PX_DEF_BIN_METADATA_ITEM(stream,	NpArticulationAttachmentArray,	PxU32,				mSize,			0)
	PX_DEF_BIN_METADATA_ITEM(stream,	NpArticulationAttachmentArray,	PxU32,				mCapacity,		0)

	//------ Extra-data ------

	PX_DEF_BIN_METADATA_EXTRA_ITEMS(stream,	NpArticulationAttachmentArray, NpArticulationAttachment, mBufferUsed, mCapacity, PxMetaDataFlag::eCONTROL_FLIP|PxMetaDataFlag::eCOUNT_MASK_MSB|PxMetaDataFlag::ePTR, 0)
}

namespace
{
	struct ShadowLoopJointArray : public PxArray<PxConstraint*>
	{
		static void getBinaryMetaData(PxOutputStream& stream)
		{
			PX_DEF_BIN_METADATA_CLASS(stream, ShadowLoopJointArray)

			PX_DEF_BIN_METADATA_ITEM(stream, ShadowLoopJointArray, NpConstraint, mData, PxMetaDataFlag::ePTR)
			PX_DEF_BIN_METADATA_ITEM(stream, ShadowLoopJointArray, PxU32, mSize, 0)
			PX_DEF_BIN_METADATA_ITEM(stream, ShadowLoopJointArray, PxU32, mCapacity, 0)
		}
	};

#define DECL_SHADOW_PTR_ARRAY(T)														\
struct ShadowArray##T : public PxArray<T*>												\
{																						\
	static void getBinaryMetaData(PxOutputStream& stream)								\
	{																					\
		PX_DEF_BIN_METADATA_CLASS(stream, ShadowArray##T)								\
		PX_DEF_BIN_METADATA_ITEM(stream, ShadowArray##T, T, mData, PxMetaDataFlag::ePTR)\
		PX_DEF_BIN_METADATA_ITEM(stream, ShadowArray##T, PxU32, mSize, 0)				\
		PX_DEF_BIN_METADATA_ITEM(stream, ShadowArray##T, PxU32, mCapacity, 0)			\
		PX_DEF_BIN_METADATA_EXTRA_ITEMS(stream,	ShadowArray##T, T, mData, mCapacity, PxMetaDataFlag::eCOUNT_MASK_MSB|PxMetaDataFlag::ePTR, 0) \
	}																					\
};

DECL_SHADOW_PTR_ARRAY(NpArticulationSpatialTendon)
DECL_SHADOW_PTR_ARRAY(NpArticulationFixedTendon)
DECL_SHADOW_PTR_ARRAY(NpArticulationMimicJoint)

}

void NpArticulationReducedCoordinate::getBinaryMetaData(PxOutputStream& stream)
{
	ShadowLoopJointArray::getBinaryMetaData(stream);
	ShadowArrayNpArticulationSpatialTendon::getBinaryMetaData(stream);
	ShadowArrayNpArticulationFixedTendon::getBinaryMetaData(stream);
	ShadowArrayNpArticulationMimicJoint::getBinaryMetaData(stream);

	//sticking this here, since typedefs are only allowed to declared once.
	PX_DEF_BIN_METADATA_TYPEDEF(stream, ArticulationTendonHandle, PxU32)

	PX_DEF_BIN_METADATA_VCLASS(stream, NpArticulationReducedCoordinate)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream, NpArticulationReducedCoordinate, PxBase)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream, NpArticulationReducedCoordinate, NpBase)

	PX_DEF_BIN_METADATA_ITEM(stream, NpArticulationReducedCoordinate, void, userData, PxMetaDataFlag::ePTR)

	PX_DEF_BIN_METADATA_ITEM(stream, NpArticulationReducedCoordinate, ArticulationCore, mCore, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, NpArticulationReducedCoordinate, NpArticulationLinkArray, mArticulationLinks, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, NpArticulationReducedCoordinate, PxU32, mNumShapes, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, NpArticulationReducedCoordinate, NpAggregate, mAggregate, PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream, NpArticulationReducedCoordinate, char, mName, PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_EXTRA_NAME(stream, NpArticulationReducedCoordinate, mName, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, NpArticulationReducedCoordinate, PxU32, mCacheVersion, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, NpArticulationReducedCoordinate, bool, mTopologyChanged, 0)

	PX_DEF_BIN_METADATA_ITEM(stream, NpArticulationReducedCoordinate, ShadowLoopJointArray, mLoopJoints, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, NpArticulationReducedCoordinate, ShadowArrayNpArticulationSpatialTendon, mSpatialTendons, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, NpArticulationReducedCoordinate, ShadowArrayNpArticulationFixedTendon, mFixedTendons, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, NpArticulationReducedCoordinate, ShadowArrayNpArticulationMimicJoint, mMimicJoints, 0)

}

void NpArticulationMimicJoint::getBinaryMetaData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_VCLASS(stream, NpArticulationMimicJoint)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream, NpArticulationMimicJoint, PxBase)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream, NpArticulationMimicJoint, NpBase)

	PX_DEF_BIN_METADATA_ITEM(stream, NpArticulationMimicJoint, PxArticulationLink, mLinkA, PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream, NpArticulationMimicJoint, PxArticulationLink, mLinkB, PxMetaDataFlag::ePTR)

	PX_DEF_BIN_METADATA_ITEM(stream, NpArticulationMimicJoint, Sc::ArticulationMimicJointCore, mCore, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, NpArticulationMimicJoint, PxU32, mHandle, 0)
}


void NpArticulationAttachment::getBinaryMetaData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_VCLASS(stream, NpArticulationAttachment)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream, NpArticulationAttachment, PxBase)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream, NpArticulationAttachment, NpBase)

	PX_DEF_BIN_METADATA_TYPEDEF(stream, ArticulationAttachmentHandle, PxU32);

	PX_DEF_BIN_METADATA_ITEM(stream, NpArticulationAttachment, void, userData, PxMetaDataFlag::ePTR)

	PX_DEF_BIN_METADATA_ITEM(stream, NpArticulationAttachment, PxArticulationLink, mLink, PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream, NpArticulationAttachment, PxArticulationAttachment, mParent, PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream, NpArticulationAttachment, ArticulationAttachmentHandle, mHandle, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, NpArticulationAttachment, NpArticulationAttachmentArray, mChildren, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, NpArticulationAttachment, NpArticulationSpatialTendon, mTendon, PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream, NpArticulationAttachment, ArticulationAttachmentCore, mCore, 0)
}

void NpArticulationTendonJoint::getBinaryMetaData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_VCLASS(stream, NpArticulationTendonJoint)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream, NpArticulationTendonJoint, PxBase)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream, NpArticulationTendonJoint, NpBase)

	PX_DEF_BIN_METADATA_ITEM(stream, NpArticulationTendonJoint, void, userData, PxMetaDataFlag::ePTR)

	PX_DEF_BIN_METADATA_ITEM(stream, NpArticulationTendonJoint, PxArticulationLink, mLink, PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream, NpArticulationTendonJoint, PxArticulationTendonJoint, mParent, PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream, NpArticulationTendonJoint, NpArticulationTendonJointArray, mChildren, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, NpArticulationTendonJoint, NpArticulationFixedTendon, mTendon, PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream, NpArticulationTendonJoint, ArticulationTendonJointCore, mCore, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, NpArticulationTendonJoint, PxU32, mHandle, 0)
}

void NpArticulationTendonJointArray::getBinaryMetaData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream,	NpArticulationTendonJointArray)
	PX_DEF_BIN_METADATA_ITEMS(stream,	NpArticulationTendonJointArray,	NpArticulationTendonJoint,	mBuffer, PxMetaDataFlag::ePTR, 4)
	PX_DEF_BIN_METADATA_ITEM(stream,	NpArticulationTendonJointArray,	bool,				mBufferUsed,	0)
		// PT: OMG this is so painful... I can't put the padding explicitely in the template
	{ PxMetaDataEntry tmp = {"char", "mPadding", 1 + PxU32(PX_OFFSET_OF_RT(NpArticulationTendonJointArray, mBufferUsed)), 3, 3, 0, PxMetaDataFlag::ePADDING, 0}; PX_STORE_METADATA(stream, tmp); }
	PX_DEF_BIN_METADATA_ITEM(stream,	NpArticulationTendonJointArray,	NpArticulationTendonJoint,	mData, PxMetaDataFlag::ePTR)	// ###
	PX_DEF_BIN_METADATA_ITEM(stream,	NpArticulationTendonJointArray,	PxU32,				mSize,			0)
	PX_DEF_BIN_METADATA_ITEM(stream,	NpArticulationTendonJointArray,	PxU32,				mCapacity,		0)

	//------ Extra-data ------

	PX_DEF_BIN_METADATA_EXTRA_ITEMS(stream,	NpArticulationTendonJointArray, NpArticulationTendonJoint, mBufferUsed, mCapacity, PxMetaDataFlag::eCONTROL_FLIP|PxMetaDataFlag::eCOUNT_MASK_MSB|PxMetaDataFlag::ePTR, 0)
}

void NpArticulationSpatialTendon::getBinaryMetaData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_VCLASS(stream, NpArticulationSpatialTendon)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream, NpArticulationSpatialTendon, PxBase)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream, NpArticulationSpatialTendon, NpBase)

	PX_DEF_BIN_METADATA_ITEM(stream, NpArticulationSpatialTendon, void, userData, PxMetaDataFlag::ePTR)

	PX_DEF_BIN_METADATA_ITEM(stream, NpArticulationSpatialTendon, NpArticulationAttachmentArray, mAttachments, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, NpArticulationSpatialTendon, NpArticulationReducedCoordinate, mArticulation, PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream, NpArticulationSpatialTendon, PxU32, mLLIndex, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, NpArticulationSpatialTendon, ArticulationSpatialTendonCore, mCore, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, NpArticulationSpatialTendon, ArticulationTendonHandle, mHandle, 0)
}

void NpArticulationFixedTendon::getBinaryMetaData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_VCLASS(stream, NpArticulationFixedTendon)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream, NpArticulationFixedTendon, PxBase)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream, NpArticulationFixedTendon, NpBase)

	PX_DEF_BIN_METADATA_ITEM(stream, NpArticulationFixedTendon, void, userData, PxMetaDataFlag::ePTR)

	PX_DEF_BIN_METADATA_ITEM(stream, NpArticulationFixedTendon, NpArticulationTendonJointArray, mTendonJoints, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, NpArticulationFixedTendon, NpArticulationReducedCoordinate, mArticulation, PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream, NpArticulationFixedTendon, PxU32, mLLIndex, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, NpArticulationFixedTendon, ArticulationFixedTendonCore, mCore, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, NpArticulationFixedTendon, ArticulationTendonHandle, mHandle, 0)
}

void NpArticulationLink::getBinaryMetaData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_VCLASS(stream,		NpArticulationLink)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	NpArticulationLink, PxBase)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	NpArticulationLink, NpActor)

	DefineMetaData_PxActor(NpArticulationLink)
	DefineMetaData_NpRigidActorTemplate(NpArticulationLink)
	DefineMetaData_NpRigidBodyTemplate(NpArticulationLink)

	// NpArticulationLink
	PX_DEF_BIN_METADATA_ITEM(stream,		NpArticulationLink, NpArticulation,				mRoot,				PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream,		NpArticulationLink, NpArticulationJoint,		mInboundJoint,		PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream,		NpArticulationLink, NpArticulationLink,			mParent,			PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream,		NpArticulationLink, NpArticulationLinkArray,	mChildLinks,		0)
	PX_DEF_BIN_METADATA_ITEM(stream,		NpArticulationLink, PxU32,						mLLIndex,			0)
	PX_DEF_BIN_METADATA_ITEM(stream,		NpArticulationLink, PxU32,						mInboundJointDof,	0);

	//------ Extra-data ------

	PX_DEF_BIN_METADATA_EXTRA_ITEM(stream,	NpArticulationLink, NpConnectorArray,			mConnectorArray, PX_SERIAL_ALIGN)
	PX_DEF_BIN_METADATA_EXTRA_NAME(stream,	NpArticulationLink, mName, 0)
}


void NpArticulationJointReducedCoordinate::getBinaryMetaData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_VCLASS(stream, NpArticulationJointReducedCoordinate)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream, NpArticulationJointReducedCoordinate, NpBase)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream, NpArticulationJointReducedCoordinate, PxBase)

	PX_DEF_BIN_METADATA_ITEM(stream, NpArticulationJointReducedCoordinate, void, userData, PxMetaDataFlag::ePTR)

	PX_DEF_BIN_METADATA_ITEM(stream, NpArticulationJointReducedCoordinate, ArticulationJointCore, mCore, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, NpArticulationJointReducedCoordinate, NpArticulationLink, mParent, PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream, NpArticulationJointReducedCoordinate, NpArticulationLink, mChild, PxMetaDataFlag::ePTR)

}

///////////////////////////////////////////////////////////////////////////////

void NpAggregate::getBinaryMetaData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_VCLASS(stream,		NpAggregate)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	NpAggregate, PxBase)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	NpAggregate, NpBase)

	// PxAggregate
	PX_DEF_BIN_METADATA_ITEM(stream,		NpAggregate, void,		userData,		PxMetaDataFlag::ePTR)

	// NpAggregate
	PX_DEF_BIN_METADATA_ITEM(stream,		NpAggregate, PxU32,		mAggregateID,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,		NpAggregate, PxU32,		mMaxNbActors,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,		NpAggregate, PxU32,		mMaxNbShapes,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,		NpAggregate, PxU32,		mFilterHint,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,		NpAggregate, PxU32,		mNbActors,		0)
	PX_DEF_BIN_METADATA_ITEM(stream,		NpAggregate, PxU32,		mNbShapes,		0)
	PX_DEF_BIN_METADATA_ITEM(stream,		NpAggregate, PxActor,	mActors,		PxMetaDataFlag::ePTR)

	//------ Extra-data ------

	// mActors
	PX_DEF_BIN_METADATA_EXTRA_ITEMS(stream,	NpAggregate, PxActor,			mActors,		mNbActors, PxMetaDataFlag::ePTR, PX_SERIAL_ALIGN)
}

///////////////////////////////////////////////////////////////////////////////

static void getBinaryMetaData_PxMeshScale(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream,	PxMeshScale)
	PX_DEF_BIN_METADATA_ITEM(stream,	PxMeshScale,		PxVec3,	scale,		0)
	PX_DEF_BIN_METADATA_ITEM(stream,	PxMeshScale,		PxQuat,	rotation,	0)
}

static void getBinaryMetaData_AABBPrunerMergeData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream,	AABBPrunerMergeData)
	PX_DEF_BIN_METADATA_ITEM(stream,	AABBPrunerMergeData,	PxU32,			mNbNodes,			0)
	PX_DEF_BIN_METADATA_ITEM(stream,	AABBPrunerMergeData,	Gu::BVHNode,	mAABBTreeNodes,		PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream,	AABBPrunerMergeData,	PxU32,			mNbObjects,			0)
	PX_DEF_BIN_METADATA_ITEM(stream,	AABBPrunerMergeData,	PxU32,			mAABBTreeIndices,	PxMetaDataFlag::ePTR)
}

void Sq::PruningStructure::getBinaryMetaData(PxOutputStream& stream)
{	
	getBinaryMetaData_AABBPrunerMergeData(stream);

	PX_DEF_BIN_METADATA_VCLASS(stream, PruningStructure)
		PX_DEF_BIN_METADATA_BASE_CLASS(stream, PruningStructure, PxBase)		

		PX_DEF_BIN_METADATA_ITEM(stream, PruningStructure, AABBPrunerMergeData, mData[0], 0)
		PX_DEF_BIN_METADATA_ITEM(stream, PruningStructure, AABBPrunerMergeData, mData[1], 0)

		PX_DEF_BIN_METADATA_ITEM(stream, PruningStructure, PxU32, mNbActors, 0)
		PX_DEF_BIN_METADATA_ITEM(stream, PruningStructure, PxActor*, mActors, PxMetaDataFlag::ePTR)
		PX_DEF_BIN_METADATA_ITEM(stream, PruningStructure, bool, mValid, 0)
}

///////////////////////////////////////////////////////////////////////////////
namespace physx
{
void getBinaryMetaData_PxBase(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_TYPEDEF(stream,	PxBaseFlags,	PxU16)
	PX_DEF_BIN_METADATA_TYPEDEF(stream,	PxType,			PxU16)
	PX_DEF_BIN_METADATA_VCLASS(stream,	PxBase)
	PX_DEF_BIN_METADATA_ITEM(stream,	PxBase,			PxType,			mConcreteType,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,	PxBase,			PxBaseFlags,	mBaseFlags,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,	PxBase,			PxI32,			mBuiltInRefCount,	0)
}
}
void RefCountable::getBinaryMetaData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_VCLASS(stream,	RefCountable)
	PX_DEF_BIN_METADATA_ITEM(stream,	RefCountable,	PxI32,			mRefCount,		0)
}

static void getFoundationMetaData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_TYPEDEF(stream,	PxU8,	char)
	PX_DEF_BIN_METADATA_TYPEDEF(stream,	PxI8,	char)
	PX_DEF_BIN_METADATA_TYPEDEF(stream,	PxU16,	short)
	PX_DEF_BIN_METADATA_TYPEDEF(stream,	PxI16,	short)
	PX_DEF_BIN_METADATA_TYPEDEF(stream,	PxU32,	int)
	PX_DEF_BIN_METADATA_TYPEDEF(stream,	PxI32,	int)
	PX_DEF_BIN_METADATA_TYPEDEF(stream,	PxReal,	float)

	getBinaryMetaData_PxVec3(stream);
	getBinaryMetaData_PxVec4(stream);
	getBinaryMetaData_PxQuat(stream);
	getBinaryMetaData_PxBounds3(stream);
	getBinaryMetaData_PxTransform(stream);
	getBinaryMetaData_PxTransform32(stream);
	getBinaryMetaData_PxMat33(stream);
	getBinaryMetaData_SpatialVectorF(stream);
	getBinaryMetaData_BitMap(stream);
	Cm::PtrTable::getBinaryMetaData(stream);
	getBinaryMetaData_PxPlane(stream);
	getBinaryMetaData_PxConstraintInvMassScale(stream);

	getBinaryMetaData_PxBase(stream);
	RefCountable::getBinaryMetaData(stream);
}

///////////////////////////////////////////////////////////////////////////////

namespace physx
{
template<> void PxsMaterialCore::getBinaryMetaData(PxOutputStream& stream);
template<> void PxsFEMSoftBodyMaterialCore::getBinaryMetaData(PxOutputStream& stream);
template<> void PxsFEMClothMaterialCore::getBinaryMetaData(PxOutputStream& stream);
template<> void PxsPBDMaterialCore::getBinaryMetaData(PxOutputStream& stream);
}

void PxGetPhysicsBinaryMetaData(PxOutputStream& stream)
{
	getFoundationMetaData(stream);
	getBinaryMetaData_PxMeshScale(stream);

	Gu::ConvexMesh::getBinaryMetaData(stream);
	Gu::TriangleMesh::getBinaryMetaData(stream);
	Gu::RTreeTriangleMesh::getBinaryMetaData(stream);
	Gu::BV4TriangleMesh::getBinaryMetaData(stream);
	Gu::HeightField::getBinaryMetaData(stream);

	PxsMaterialCore::getBinaryMetaData(stream);
	PxsFEMSoftBodyMaterialCore::getBinaryMetaData(stream);
	PxsFEMClothMaterialCore::getBinaryMetaData(stream);
	PxsPBDMaterialCore::getBinaryMetaData(stream);

	MaterialIndicesStruct::getBinaryMetaData(stream);
	GeometryUnion::getBinaryMetaData(stream);
	Sc::ActorCore::getBinaryMetaData(stream);
	Sc::RigidCore::getBinaryMetaData(stream);
	Sc::StaticCore::getBinaryMetaData(stream);
	Sc::BodyCore::getBinaryMetaData(stream);
	Sc::ShapeCore::getBinaryMetaData(stream);
	Sc::ConstraintCore::getBinaryMetaData(stream);
	Sc::ArticulationCore::getBinaryMetaData(stream);
	Sc::ArticulationJointCore::getBinaryMetaData(stream);
	Sc::ArticulationTendonCore::getBinaryMetaData(stream);
	Sc::ArticulationSpatialTendonCore::getBinaryMetaData(stream);
	Sc::ArticulationAttachmentCore::getBinaryMetaData(stream);
	Sc::ArticulationFixedTendonCore::getBinaryMetaData(stream);
	Sc::ArticulationTendonJointCore::getBinaryMetaData(stream);

	NpConnector::getBinaryMetaData(stream);
	NpConnectorArray::getBinaryMetaData(stream);
	NpBase::getBinaryMetaData(stream);
	NpActor::getBinaryMetaData(stream);
	NpMaterial::getBinaryMetaData(stream);
	NpFEMSoftBodyMaterial::getBinaryMetaData(stream);
#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
	NpFEMClothMaterial::getBinaryMetaData(stream);
#endif
	NpPBDMaterial::getBinaryMetaData(stream);
	NpRigidDynamic::getBinaryMetaData(stream);
	NpRigidStatic::getBinaryMetaData(stream);
	NpShape::getBinaryMetaData(stream);
	NpConstraint::getBinaryMetaData(stream);
	NpArticulationReducedCoordinate::getBinaryMetaData(stream);
	NpArticulationLink::getBinaryMetaData(stream);
	NpArticulationJointReducedCoordinate::getBinaryMetaData(stream);
	NpArticulationLinkArray::getBinaryMetaData(stream);
	NpArticulationSpatialTendon::getBinaryMetaData(stream);
	NpArticulationFixedTendon::getBinaryMetaData(stream);
	NpArticulationAttachment::getBinaryMetaData(stream);
	NpArticulationAttachmentArray::getBinaryMetaData(stream);
	NpArticulationTendonJoint::getBinaryMetaData(stream);
	NpArticulationTendonJointArray::getBinaryMetaData(stream);
	NpShapeManager::getBinaryMetaData(stream);
	NpAggregate::getBinaryMetaData(stream);
}

