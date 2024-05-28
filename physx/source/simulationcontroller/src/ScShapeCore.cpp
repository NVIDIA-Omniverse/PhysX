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

#include "foundation/PxErrorCallback.h"
#include "ScShapeSim.h"
#include "ScPhysics.h"
#include "GuConvexMesh.h"
#include "GuTriangleMesh.h"
#include "GuHeightField.h"
#include "GuTetrahedronMesh.h"

using namespace physx;
using namespace Gu;
using namespace Cm;
using namespace Sc;

static PX_FORCE_INLINE Gu::ConvexMesh& getConvexMesh(PxConvexMesh* pxcm)
{ 
	return *static_cast<Gu::ConvexMesh*>(pxcm);
}

// PT: TODO: optimize all these data copies
void GeometryUnion::set(const PxGeometry& g)
{
	// PT: preserve this field that can be used by higher-level code to store useful data
	const float saved = reinterpret_cast<const PxGeometry&>(mGeometry).mTypePadding;

	switch(g.getType())
	{
		case PxGeometryType::eBOX:
		{
			reinterpret_cast<PxBoxGeometry&>(mGeometry) = static_cast<const PxBoxGeometry&>(g);
		}
		break;

		case PxGeometryType::eCAPSULE:
		{
			reinterpret_cast<PxCapsuleGeometry&>(mGeometry) = static_cast<const PxCapsuleGeometry&>(g);
		}
		break;

		case PxGeometryType::eSPHERE:
		{
			reinterpret_cast<PxSphereGeometry&>(mGeometry) = static_cast<const PxSphereGeometry&>(g);
			reinterpret_cast<PxCapsuleGeometry&>(mGeometry).halfHeight = 0.0f;		//AM: make sphere geometry also castable as a zero height capsule.
		}
		break;

		case PxGeometryType::ePLANE:
		{
			reinterpret_cast<PxPlaneGeometry&>(mGeometry) = static_cast<const PxPlaneGeometry&>(g);
		}
		break;

		case PxGeometryType::eCONVEXMESH:
		{
			reinterpret_cast<PxConvexMeshGeometry&>(mGeometry) = static_cast<const PxConvexMeshGeometry&>(g);
			reinterpret_cast<PxConvexMeshGeometryLL&>(mGeometry).gpuCompatible = ::getConvexMesh(get<PxConvexMeshGeometryLL>().convexMesh).isGpuCompatible();
		}
		break;

		case PxGeometryType::ePARTICLESYSTEM:
		{
			reinterpret_cast<PxParticleSystemGeometry&>(mGeometry) = static_cast<const PxParticleSystemGeometry&>(g);
			reinterpret_cast<PxParticleSystemGeometryLL&>(mGeometry).materialsLL = MaterialIndicesStruct();
		}
		break;

		case PxGeometryType::eTRIANGLEMESH:
		{
			reinterpret_cast<PxTriangleMeshGeometry&>(mGeometry) = static_cast<const PxTriangleMeshGeometry&>(g);
			reinterpret_cast<PxTriangleMeshGeometryLL&>(mGeometry).materialsLL = MaterialIndicesStruct();
		}
		break;

		case PxGeometryType::eTETRAHEDRONMESH:
		{
			reinterpret_cast<PxTetrahedronMeshGeometry&>(mGeometry) = static_cast<const PxTetrahedronMeshGeometry&>(g);
			reinterpret_cast<PxTetrahedronMeshGeometryLL&>(mGeometry).materialsLL = MaterialIndicesStruct();
		}
		break;

		case PxGeometryType::eHEIGHTFIELD:
		{
			reinterpret_cast<PxHeightFieldGeometry&>(mGeometry) = static_cast<const PxHeightFieldGeometry&>(g);
			reinterpret_cast<PxHeightFieldGeometryLL&>(mGeometry).materialsLL = MaterialIndicesStruct();
		}
		break;

		case PxGeometryType::eHAIRSYSTEM:
		{
			reinterpret_cast<PxHairSystemGeometry&>(mGeometry) = static_cast<const PxHairSystemGeometry&>(g);
		}
		break;

		case PxGeometryType::eCUSTOM:
		{
			reinterpret_cast<PxCustomGeometry&>(mGeometry) = static_cast<const PxCustomGeometry&>(g);
		}
		break;

		case PxGeometryType::eGEOMETRY_COUNT:
		case PxGeometryType::eINVALID:
			PX_ALWAYS_ASSERT_MESSAGE("geometry type not handled");
		break;
	}

	reinterpret_cast<PxGeometry&>(mGeometry).mTypePadding = saved;
}

static PxConvexMeshGeometryLL extendForLL(const PxConvexMeshGeometry& hlGeom)
{
	PxConvexMeshGeometryLL llGeom;
	static_cast<PxConvexMeshGeometry&>(llGeom) = hlGeom;

	llGeom.gpuCompatible = hlGeom.convexMesh->isGpuCompatible();

	return llGeom;
}

static PxTriangleMeshGeometryLL extendForLL(const PxTriangleMeshGeometry& hlGeom)
{
	PxTriangleMeshGeometryLL llGeom;
	static_cast<PxTriangleMeshGeometry&>(llGeom) = hlGeom;

	llGeom.materialsLL = static_cast<const PxTriangleMeshGeometryLL&>(hlGeom).materialsLL;

	return llGeom;
}

static PxHeightFieldGeometryLL extendForLL(const PxHeightFieldGeometry& hlGeom)
{
	PxHeightFieldGeometryLL llGeom;
	static_cast<PxHeightFieldGeometry&>(llGeom) = hlGeom;

	llGeom.materialsLL = static_cast<const PxHeightFieldGeometryLL&>(hlGeom).materialsLL;

	return llGeom;
}

ShapeCore::ShapeCore(const PxGeometry& geometry, PxShapeFlags shapeFlags, const PxU16* materialIndices, PxU16 materialCount, bool isExclusive, PxShapeCoreFlag::Enum coreFlags) :
	mExclusiveSim(NULL)
{
	mCore.mShapeCoreFlags |= PxShapeCoreFlag::eOWNS_MATERIAL_IDX_MEMORY;
	if(isExclusive)
		mCore.mShapeCoreFlags |= PxShapeCoreFlag::eIS_EXCLUSIVE;

	mCore.mShapeCoreFlags |= coreFlags;

	PX_ASSERT(materialCount > 0);

	const PxTolerancesScale& scale = Physics::getInstance().getTolerancesScale();
	mCore.mGeometry.set(geometry);
	mCore.setTransform(PxTransform(PxIdentity));
	mCore.mContactOffset			= 0.02f * scale.length;
	mCore.mRestOffset				= 0.0f;
	mCore.mTorsionalRadius			= 0.0f;
	mCore.mMinTorsionalPatchRadius	= 0.0f;
	mCore.mShapeFlags				= shapeFlags;

	setMaterialIndices(materialIndices, materialCount);
}

// PX_SERIALIZATION
ShapeCore::ShapeCore(const PxEMPTY) : 
	mSimulationFilterData	(PxEmpty),
	mCore					(PxEmpty),
	mExclusiveSim			(NULL)
{ 
	mCore.mShapeCoreFlags.clear(PxShapeCoreFlag::eOWNS_MATERIAL_IDX_MEMORY);
}
//~PX_SERIALIZATION

static PX_FORCE_INLINE const MaterialIndicesStruct* getMaterials(const GeometryUnion& gu)
{
	const PxGeometryType::Enum type = gu.getType();
	if(type == PxGeometryType::eTRIANGLEMESH)
		return &gu.get<PxTriangleMeshGeometryLL>().materialsLL;
	else if(type == PxGeometryType::eHEIGHTFIELD)
		return &gu.get<PxHeightFieldGeometryLL>().materialsLL;
	else if(type == PxGeometryType::eTETRAHEDRONMESH)
		return &gu.get<PxTetrahedronMeshGeometryLL>().materialsLL;
	else if(type == PxGeometryType::ePARTICLESYSTEM)
		return &gu.get<PxParticleSystemGeometryLL>().materialsLL;
	else
		return NULL;
}

ShapeCore::~ShapeCore()
{
	if(mCore.mShapeCoreFlags.isSet(PxShapeCoreFlag::eOWNS_MATERIAL_IDX_MEMORY))
	{
		MaterialIndicesStruct* materialsLL = const_cast<MaterialIndicesStruct*>(getMaterials(mCore.mGeometry));
		if(materialsLL)
			materialsLL->deallocate();
	}
}

PxU16 Sc::ShapeCore::getNbMaterialIndices() const
{
	const MaterialIndicesStruct* materialsLL = getMaterials(mCore.mGeometry);
	return materialsLL ? materialsLL->numIndices : 1;
}

const PxU16* Sc::ShapeCore::getMaterialIndices() const
{
	const MaterialIndicesStruct* materialsLL = getMaterials(mCore.mGeometry);
	return materialsLL ? materialsLL->indices : &mCore.mMaterialIndex;
}

PX_FORCE_INLINE void setMaterialsHelper(MaterialIndicesStruct& materials, const PxU16* materialIndices, PxU16 materialIndexCount, PxShapeCoreFlags& shapeCoreFlags)
{
	if(materials.numIndices < materialIndexCount)
	{
		if(materials.indices && shapeCoreFlags.isSet(PxShapeCoreFlag::eOWNS_MATERIAL_IDX_MEMORY))
			materials.deallocate();
		materials.allocate(materialIndexCount);
		shapeCoreFlags |= PxShapeCoreFlag::eOWNS_MATERIAL_IDX_MEMORY;
	}
	PxMemCopy(materials.indices, materialIndices, sizeof(PxU16)*materialIndexCount);
	materials.numIndices = materialIndexCount;
}

void ShapeCore::setMaterialIndices(const PxU16* materialIndices, PxU16 materialIndexCount)
{
	mCore.mMaterialIndex = materialIndices[0];

	MaterialIndicesStruct* materialsLL = const_cast<MaterialIndicesStruct*>(getMaterials(mCore.mGeometry));
	if(materialsLL)
		setMaterialsHelper(*materialsLL, materialIndices, materialIndexCount, mCore.mShapeCoreFlags);
}

void ShapeCore::setGeometry(const PxGeometry& geom)
{
	const PxGeometryType::Enum newGeomType = geom.getType();

	// copy material related data to restore it after the new geometry has been set
	MaterialIndicesStruct materials;
	PX_ASSERT(materials.numIndices == 0);
	
	const MaterialIndicesStruct* materialsLL = getMaterials(mCore.mGeometry);
	if(materialsLL)
		materials = *materialsLL;

	mCore.mGeometry.set(geom);

	if((newGeomType == PxGeometryType::eTRIANGLEMESH) || (newGeomType == PxGeometryType::eHEIGHTFIELD) 
		|| (newGeomType == PxGeometryType::eTETRAHEDRONMESH)|| (newGeomType == PxGeometryType::ePARTICLESYSTEM))
	{
		MaterialIndicesStruct* newMaterials = const_cast<MaterialIndicesStruct*>(getMaterials(mCore.mGeometry));
		PX_ASSERT(newMaterials);

		if(materials.numIndices != 0)  // old type was mesh type
			*newMaterials = materials;
		else
		{   // old type was non-mesh type
			newMaterials->allocate(1);
			*newMaterials->indices = mCore.mMaterialIndex;
			mCore.mShapeCoreFlags |= PxShapeCoreFlag::eOWNS_MATERIAL_IDX_MEMORY;
		}
	}
	else if((materials.numIndices != 0) && mCore.mShapeCoreFlags.isSet(PxShapeCoreFlag::eOWNS_MATERIAL_IDX_MEMORY))
	{
		// geometry changed to non-mesh type
		materials.deallocate();
	}
}

PxShape* ShapeCore::getPxShape()
{
	return Sc::gOffsetTable.convertScShape2Px(this);
}

const PxShape* ShapeCore::getPxShape() const
{
	return Sc::gOffsetTable.convertScShape2Px(this);
}

void ShapeCore::setContactOffset(const PxReal offset)
{
	mCore.mContactOffset = offset;

	ShapeSim* exclusiveSim = getExclusiveSim();
	if(exclusiveSim)
		exclusiveSim->getScene().updateContactDistance(exclusiveSim->getElementID(), offset);
}

// PX_SERIALIZATION

PX_FORCE_INLINE void exportExtraDataMaterials(PxSerializationContext& stream, const MaterialIndicesStruct& materials)
{
	stream.alignData(PX_SERIAL_ALIGN);
	stream.writeData(materials.indices, sizeof(PxU16)*materials.numIndices);
}

void ShapeCore::exportExtraData(PxSerializationContext& stream)
{
	const MaterialIndicesStruct* materialsLL = getMaterials(mCore.mGeometry);
	if(materialsLL)
		exportExtraDataMaterials(stream, *materialsLL);
}

void ShapeCore::importExtraData(PxDeserializationContext& context)
{
	MaterialIndicesStruct* materialsLL = const_cast<MaterialIndicesStruct*>(getMaterials(mCore.mGeometry));
	if(materialsLL)
		materialsLL->indices = context.readExtraData<PxU16, PX_SERIAL_ALIGN>(materialsLL->numIndices);
}

void ShapeCore::resolveMaterialReference(PxU32 materialTableIndex, PxU16 materialIndex)
{
	if(materialTableIndex == 0)
		mCore.mMaterialIndex = materialIndex;

	MaterialIndicesStruct* materialsLL = const_cast<MaterialIndicesStruct*>(getMaterials(mCore.mGeometry));
	if(materialsLL)
		materialsLL->indices[materialTableIndex] = materialIndex;
}

void ShapeCore::resolveReferences(PxDeserializationContext& context)
{
	// Resolve geometry pointers if needed
	PxGeometry& geom = const_cast<PxGeometry&>(mCore.mGeometry.getGeometry());	
	
	switch(geom.getType())
	{
	case PxGeometryType::eCONVEXMESH:
	{
		PxConvexMeshGeometryLL& convexGeom = static_cast<PxConvexMeshGeometryLL&>(geom);
		context.translatePxBase(convexGeom.convexMesh);		

		// update the hullData pointer
		static_cast<PxConvexMeshGeometryLL&>(geom) = extendForLL(convexGeom);
	}
	break;

	case PxGeometryType::eHEIGHTFIELD:
	{
		PxHeightFieldGeometryLL& hfGeom = static_cast<PxHeightFieldGeometryLL&>(geom);
		context.translatePxBase(hfGeom.heightField);

		// update hf pointers
		static_cast<PxHeightFieldGeometryLL&>(geom) = extendForLL(hfGeom);
	}
	break;

	case PxGeometryType::eTRIANGLEMESH:
	{
		PxTriangleMeshGeometryLL& meshGeom = static_cast<PxTriangleMeshGeometryLL&>(geom);
		context.translatePxBase(meshGeom.triangleMesh);		

		// update mesh pointers
		static_cast<PxTriangleMeshGeometryLL&>(geom) = extendForLL(meshGeom);
	}
	break;
	case PxGeometryType::eTETRAHEDRONMESH:
	case PxGeometryType::ePARTICLESYSTEM:
	case PxGeometryType::eHAIRSYSTEM:
	case PxGeometryType::eCUSTOM:
	{
		// implement
		PX_ASSERT(0);
	}
	break;
	case PxGeometryType::eSPHERE:
	case PxGeometryType::ePLANE:
	case PxGeometryType::eCAPSULE:
	case PxGeometryType::eBOX:
	case PxGeometryType::eGEOMETRY_COUNT:
	case PxGeometryType::eINVALID:
	break;
	}	
}

//~PX_SERIALIZATION
