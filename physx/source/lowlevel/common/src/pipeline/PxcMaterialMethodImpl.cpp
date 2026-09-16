// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "PxcMaterialMethodImpl.h"
#include "PxvGeometry.h"
#include "PxcNpThreadContext.h"
#include "PxsMaterialManager.h"
#include "GuTriangleMesh.h"
#include "GuHeightField.h"

using namespace physx;
using namespace Gu;

// PT: moved these functions to same file for improving code locality and easily reusing code (calling smaller functions from larger ones, see below)

///////////////////////////////////////////////////////////////////////////////

static void PxcGetMaterialShape(const PxsShapeCore* shape, const PxU32 index, const PxContactBuffer& contactBuffer, PxsMaterialInfo* materialInfo)
{
	const PxU16 materialIndex = shape->mMaterialIndex;
	const PxU32 count = contactBuffer.count;
	PX_ASSERT(index==0 || index==1);
	for(PxU32 i=0; i<count; i++)
		(&materialInfo[i].mMaterialIndex0)[index] = materialIndex;
}

static void PxcGetMaterialShapeShape(const PxsShapeCore* shape0, const PxsShapeCore* shape1, const PxContactBuffer& contactBuffer, PxsMaterialInfo* materialInfo)
{
	const PxU16 materialIndex0 = shape0->mMaterialIndex;
	const PxU16 materialIndex1 = shape1->mMaterialIndex;
	const PxU32 count = contactBuffer.count;
	for(PxU32 i=0; i<count; i++)
	{
		materialInfo[i].mMaterialIndex0 = materialIndex0;
		materialInfo[i].mMaterialIndex1 = materialIndex1;
	}
}

///////////////////////////////////////////////////////////////////////////////

static PX_FORCE_INLINE const PxU16* getMaterialIndicesLL(const PxTriangleMeshGeometry& meshGeom)
{
	return static_cast<const Gu::TriangleMesh*>(meshGeom.triangleMesh)->getMaterials();
}

// PT: a contact is not guaranteed to carry a face index. Plane vs triangle mesh is the generator that
// legitimately cannot provide one: it emits a contact per mesh *vertex* clipped against the plane, so there
// is no single triangle to attribute the contact to. More generally
// PxContactPoint::internalFaceIndex1 is documented as PXC_CONTACT_NO_FACE_INDEX whenever the generator does
// not supply it. Using that value to index a per-triangle material array reads wildly out of bounds, so fall
// back to the first material instead. See OMPE-103060.
static PX_FORCE_INLINE PxU32 getLocalMaterialIndex(const PxU16* eaMaterialIndices, PxU32 faceIndex1)
{
	return (eaMaterialIndices && faceIndex1 != PXC_CONTACT_NO_FACE_INDEX) ? eaMaterialIndices[faceIndex1] : 0;
}

static void PxcGetMaterialMesh(const PxsShapeCore* shape, const PxU32 index, const PxContactBuffer& contactBuffer, PxsMaterialInfo* materialInfo)
{
	PX_ASSERT(index == 0 || index == 1);
	const PxTriangleMeshGeometryLL& shapeMesh = shape->mGeometry.get<const PxTriangleMeshGeometryLL>();
	if(shapeMesh.materialsLL.numIndices <= 1)
	{
		PxcGetMaterialShape(shape, index, contactBuffer, materialInfo);
	}
	else
	{
		const PxU32 count = contactBuffer.count;
		const PxU16* eaMaterialIndices = getMaterialIndicesLL(shapeMesh);
		const PxU16* indices = shapeMesh.materialsLL.indices;
		for(PxU32 i=0; i<count; i++)
		{
			const PxContactPoint& contact = contactBuffer.contacts[i];
			const PxU32 localMaterialIndex = getLocalMaterialIndex(eaMaterialIndices, contact.internalFaceIndex1);
			(&materialInfo[i].mMaterialIndex0)[index] = indices[localMaterialIndex];
		}
	}
}

static void PxcGetMaterialShapeMesh(const PxsShapeCore* shape0, const PxsShapeCore* shape1, const PxContactBuffer& contactBuffer, PxsMaterialInfo* materialInfo)
{
	const PxTriangleMeshGeometryLL& shapeMesh = shape1->mGeometry.get<const PxTriangleMeshGeometryLL>();
	if(shapeMesh.materialsLL.numIndices <= 1)
	{
		PxcGetMaterialShapeShape(shape0, shape1, contactBuffer, materialInfo);
	}
	else
	{
		const PxU32 count = contactBuffer.count;
		const PxU16* eaMaterialIndices = getMaterialIndicesLL(shapeMesh);
		const PxU16* indices = shapeMesh.materialsLL.indices;
		const PxU16 materialIndex0 = shape0->mMaterialIndex;
		for(PxU32 i=0; i<count; i++)
		{
			const PxContactPoint& contact = contactBuffer.contacts[i];
			materialInfo[i].mMaterialIndex0 = materialIndex0;

			const PxU32 localMaterialIndex = getLocalMaterialIndex(eaMaterialIndices, contact.internalFaceIndex1);
			materialInfo[i].mMaterialIndex1 = indices[localMaterialIndex];
		}
	}
}

static void PxcGetMaterialSoftBodyMesh(const PxsShapeCore* shape0, const PxsShapeCore* shape1, const PxContactBuffer& contactBuffer, PxsMaterialInfo* materialInfo)
{
	// PT: TODO: check this, it reads shape0 and labels it shapeMesh1? It's otherwise the same code as PxcGetMaterialShapeMesh ?
	const PxTriangleMeshGeometryLL& shapeMesh1 = shape0->mGeometry.get<const PxTriangleMeshGeometryLL>();
	if (shapeMesh1.materialsLL.numIndices <= 1)
	{
		PxcGetMaterialShapeShape(shape0, shape1, contactBuffer, materialInfo);
	}
	else
	{
		const PxU32 count = contactBuffer.count;
		const PxU16* eaMaterialIndices = getMaterialIndicesLL(shapeMesh1);
		const PxU16* indices = shapeMesh1.materialsLL.indices;
		const PxU16 materialIndex0 = shape0->mMaterialIndex;
		for (PxU32 i = 0; i<count; i++)
		{
			const PxContactPoint& contact = contactBuffer.contacts[i];
			materialInfo[i].mMaterialIndex0 = materialIndex0;

			const PxU32 localMaterialIndex = getLocalMaterialIndex(eaMaterialIndices, contact.internalFaceIndex1);
			materialInfo[i].mMaterialIndex1 = indices[localMaterialIndex];
		}
	}
}

///////////////////////////////////////////////////////////////////////////////

static PxU32 getMaterialIndex(const Gu::HeightFieldData* hfData, PxU32 triangleIndex)
{
	// PT: defensive, see getLocalMaterialIndex() above. All callers pass PxContactPoint::internalFaceIndex1,
	// which is only set by generators that can provide it. No heightfield generator is known to leave it unset
	// today, but the field is not guaranteed by the API contract and without this guard such a contact would
	// make the sample lookup below read far out of bounds. See OMPE-103060.
	if(triangleIndex == PXC_CONTACT_NO_FACE_INDEX)
		return 0;

	const PxU32 sampleIndex = triangleIndex >> 1;
	const bool isFirstTriangle = (triangleIndex & 0x1) == 0;

	//get sample
	const PxHeightFieldSample* hf = &hfData->samples[sampleIndex];
	return isFirstTriangle ? hf->materialIndex0 : hf->materialIndex1;
}

static void PxcGetMaterialHeightField(const PxsShapeCore* shape, const PxU32 index, const PxContactBuffer& contactBuffer, PxsMaterialInfo* materialInfo)
{
	PX_ASSERT(index == 0 || index == 1);
	const PxHeightFieldGeometryLL& hfGeom = shape->mGeometry.get<const PxHeightFieldGeometryLL>();
	if(hfGeom.materialsLL.numIndices <= 1)
	{
		PxcGetMaterialShape(shape, index, contactBuffer, materialInfo);
	}
	else
	{
		const PxU32 count = contactBuffer.count;
		const PxU16* materialIndices = hfGeom.materialsLL.indices;
			
		const Gu::HeightFieldData* hf = &static_cast<const Gu::HeightField*>(hfGeom.heightField)->getData();
		
		for(PxU32 i=0; i<count; i++)
		{
			const PxContactPoint& contact = contactBuffer.contacts[i];
			const PxU32 localMaterialIndex = getMaterialIndex(hf, contact.internalFaceIndex1);
			(&materialInfo[i].mMaterialIndex0)[index] = materialIndices[localMaterialIndex];
		}
	}
}

static void PxcGetMaterialShapeHeightField(const PxsShapeCore* shape0, const PxsShapeCore* shape1, const PxContactBuffer& contactBuffer, PxsMaterialInfo* materialInfo)
{
	const PxHeightFieldGeometryLL& hfGeom = shape1->mGeometry.get<const PxHeightFieldGeometryLL>();
	if(hfGeom.materialsLL.numIndices <= 1)
	{
		PxcGetMaterialShapeShape(shape0, shape1, contactBuffer, materialInfo);
	}
	else
	{
		const PxU32 count = contactBuffer.count;
		const PxU16* materialIndices = hfGeom.materialsLL.indices;
			
		const Gu::HeightFieldData* hf = &static_cast<const Gu::HeightField*>(hfGeom.heightField)->getData();
		
		for(PxU32 i=0; i<count; i++)
		{
			const PxContactPoint& contact = contactBuffer.contacts[i];
			materialInfo[i].mMaterialIndex0 = shape0->mMaterialIndex;
			//contact.featureIndex0 = shape0->materialIndex;
			const PxU32 localMaterialIndex = getMaterialIndex(hf, contact.internalFaceIndex1);
			//contact.featureIndex1 = materialIndices[localMaterialIndex];
			PX_ASSERT(localMaterialIndex<hfGeom.materialsLL.numIndices);
			materialInfo[i].mMaterialIndex1 = materialIndices[localMaterialIndex];
		}
	}
}

static void PxcGetMaterialSoftBodyHeightField(const PxsShapeCore* shape0, const PxsShapeCore* shape1, const PxContactBuffer& contactBuffer, PxsMaterialInfo* materialInfo)
{
	const PxHeightFieldGeometryLL& hfGeom = shape1->mGeometry.get<const PxHeightFieldGeometryLL>();
	if (hfGeom.materialsLL.numIndices <= 1)
	{
		PxcGetMaterialShapeShape(shape0, shape1, contactBuffer, materialInfo);
	}
	else
	{
		const PxU32 count = contactBuffer.count;
		const PxU16* materialIndices = hfGeom.materialsLL.indices;

		const Gu::HeightFieldData* hf = &static_cast<const Gu::HeightField*>(hfGeom.heightField)->getData();

		for(PxU32 i=0; i<count; i++)
		{
			const PxContactPoint& contact = contactBuffer.contacts[i];
			materialInfo[i].mMaterialIndex0 = shape0->mMaterialIndex;
			//contact.featureIndex0 = shape0->materialIndex;
			const PxU32 localMaterialIndex = getMaterialIndex(hf, contact.internalFaceIndex1);
			//contact.featureIndex1 = materialIndices[localMaterialIndex];
			PX_ASSERT(localMaterialIndex<hfGeom.materialsLL.numIndices);
			materialInfo[i].mMaterialIndex1 = materialIndices[localMaterialIndex];
		}
	}
}

///////////////////////////////////////////////////////////////////////////////

static void PxcGetMaterialSoftBody(const PxsShapeCore* shape, const PxU32 index, const PxContactBuffer& contactBuffer, PxsMaterialInfo* materialInfo)
{
	PX_ASSERT(index == 1);
	PX_UNUSED(index);
	PxcGetMaterialShape(shape, index, contactBuffer, materialInfo);
}

static void PxcGetMaterialShapeSoftBody(const PxsShapeCore* shape0, const PxsShapeCore* shape1, const PxContactBuffer& contactBuffer, PxsMaterialInfo* materialInfo)
{
	PxcGetMaterialShapeShape(shape0, shape1, contactBuffer, materialInfo);
}

static void PxcGetMaterialSoftBodySoftBody(const PxsShapeCore* shape0, const PxsShapeCore* shape1, const PxContactBuffer& contactBuffer, PxsMaterialInfo* materialInfo)
{
	PxcGetMaterialShapeShape(shape0, shape1, contactBuffer, materialInfo);
}

///////////////////////////////////////////////////////////////////////////////

namespace physx
{
PxcGetSingleMaterialMethod g_GetSingleMaterialMethodTable[] = 
{
	PxcGetMaterialShape,		//PxGeometryType::eSPHERE
	PxcGetMaterialShape,		//PxGeometryType::ePLANE
	PxcGetMaterialShape,		//PxGeometryType::eCAPSULE
	PxcGetMaterialShape,		//PxGeometryType::eBOX
	PxcGetMaterialShape,		//PxGeometryType::eCONVEXCORE
	PxcGetMaterialShape,		//PxGeometryType::eCONVEXMESH
	PxcGetMaterialSoftBody,		//PxGeometryType::ePARTICLESYSTEM
	PxcGetMaterialSoftBody,		//PxGeometryType::eTETRAHEDRONMESH
	PxcGetMaterialMesh,			//PxGeometryType::eTRIANGLEMESH	//not used: mesh always uses swept method for midphase.
	PxcGetMaterialHeightField,	//PxGeometryType::eHEIGHTFIELD	//TODO: make HF midphase that will mask this
	PxcGetMaterialShape,		//PxGeometryType::eCUSTOM
};
PX_COMPILE_TIME_ASSERT(sizeof(g_GetSingleMaterialMethodTable) / sizeof(g_GetSingleMaterialMethodTable[0]) == PxGeometryType::eGEOMETRY_COUNT);

//Table of contact methods for different shape-type combinations
PxcGetMaterialMethod g_GetMaterialMethodTable[][PxGeometryType::eGEOMETRY_COUNT] = 
{
	
	//PxGeometryType::eSPHERE
	{
		PxcGetMaterialShapeShape,			//PxGeometryType::eSPHERE
		PxcGetMaterialShapeShape,			//PxGeometryType::ePLANE
		PxcGetMaterialShapeShape,			//PxGeometryType::eCAPSULE
		PxcGetMaterialShapeShape,			//PxGeometryType::eBOX
		PxcGetMaterialShapeShape,			//PxGeometryType::eCONVEXCORE
		PxcGetMaterialShapeShape,			//PxGeometryType::eCONVEXMESH
		PxcGetMaterialShapeSoftBody,		//PxGeometryType::ePARTICLESYSTEM
		PxcGetMaterialShapeSoftBody,		//PxGeometryType::eTETRAHEDRONMESH
		PxcGetMaterialShapeMesh,			//PxGeometryType::eTRIANGLEMESH	//not used: mesh always uses swept method for midphase.
		PxcGetMaterialShapeHeightField,		//PxGeometryType::eHEIGHTFIELD	//TODO: make HF midphase that will mask this
		PxcGetMaterialShapeShape,			//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::ePLANE
	{
		0,									//PxGeometryType::eSPHERE
		0,									//PxGeometryType::ePLANE
		PxcGetMaterialShapeShape,			//PxGeometryType::eCAPSULE
		PxcGetMaterialShapeShape,			//PxGeometryType::eBOX
		PxcGetMaterialShapeShape,			//PxGeometryType::eCONVEXCORE
		PxcGetMaterialShapeShape,			//PxGeometryType::eCONVEXMESH
		PxcGetMaterialShapeSoftBody,		//PxGeometryType::ePARTICLESYSTEM
		PxcGetMaterialShapeSoftBody,		//PxGeometryType::eTETRAHEDRONMESH
		PxcGetMaterialShapeMesh,			//PxGeometryType::eTRIANGLEMESH
		0,									//PxGeometryType::eHEIGHTFIELD
		PxcGetMaterialShapeShape,			//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::eCAPSULE
	{
		0,									//PxGeometryType::eSPHERE
		0,									//PxGeometryType::ePLANE
		PxcGetMaterialShapeShape,			//PxGeometryType::eCAPSULE
		PxcGetMaterialShapeShape,			//PxGeometryType::eBOX
		PxcGetMaterialShapeShape,			//PxGeometryType::eCONVEXCORE
		PxcGetMaterialShapeShape,			//PxGeometryType::eCONVEXMESH
		PxcGetMaterialShapeSoftBody,		//PxGeometryType::ePARTICLESYSTEM
		PxcGetMaterialShapeSoftBody,		//PxGeometryType::eTETRAHEDRONMESH
		PxcGetMaterialShapeMesh,			//PxGeometryType::eTRIANGLEMESH		//not used: mesh always uses swept method for midphase.
		PxcGetMaterialShapeHeightField,		//PxGeometryType::eHEIGHTFIELD		//TODO: make HF midphase that will mask this
		PxcGetMaterialShapeShape,			//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::eBOX
	{
		0,									//PxGeometryType::eSPHERE
		0,									//PxGeometryType::ePLANE
		0,									//PxGeometryType::eCAPSULE
		PxcGetMaterialShapeShape,			//PxGeometryType::eBOX
		PxcGetMaterialShapeShape,			//PxGeometryType::eCONVEXCORE
		PxcGetMaterialShapeShape,			//PxGeometryType::eCONVEXMESH
		PxcGetMaterialShapeSoftBody,		//PxGeometryType::ePARTICLESYSTEM
		PxcGetMaterialShapeSoftBody,		//PxGeometryType::eTETRAHEDRONMESH
		PxcGetMaterialShapeMesh,			//PxGeometryType::eTRIANGLEMESH		//not used: mesh always uses swept method for midphase.
		PxcGetMaterialShapeHeightField,		//PxGeometryType::eHEIGHTFIELD		//TODO: make HF midphase that will mask this
		PxcGetMaterialShapeShape,			//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::eCONVEX
	{
		0,									//PxGeometryType::eSPHERE
		0,									//PxGeometryType::ePLANE
		0,									//PxGeometryType::eCAPSULE
		0,									//PxGeometryType::eBOX
		PxcGetMaterialShapeShape,			//PxGeometryType::eCONVEXCORE
		PxcGetMaterialShapeShape,			//PxGeometryType::eCONVEXMESH
		PxcGetMaterialShapeSoftBody,		//PxGeometryType::ePARTICLESYSTEM
		PxcGetMaterialShapeSoftBody,		//PxGeometryType::eTETRAHEDRONMESH
		PxcGetMaterialShapeMesh,			//PxGeometryType::eTRIANGLEMESH		//not used: mesh always uses swept method for midphase.
		PxcGetMaterialShapeHeightField,		//PxGeometryType::eHEIGHTFIELD		//TODO: make HF midphase that will mask this
		PxcGetMaterialShapeShape,			//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::eCONVEXMESH
	{
		0,									//PxGeometryType::eSPHERE
		0,									//PxGeometryType::ePLANE
		0,									//PxGeometryType::eCAPSULE
		0,									//PxGeometryType::eBOX
		0,									//PxGeometryType::eCONVEXCORE
		PxcGetMaterialShapeShape,			//PxGeometryType::eCONVEXMESH
		PxcGetMaterialShapeSoftBody,		//PxGeometryType::ePARTICLESYSTEM
		PxcGetMaterialShapeSoftBody,		//PxGeometryType::eTETRAHEDRONMESH
		PxcGetMaterialShapeMesh,			//PxGeometryType::eTRIANGLEMESH		//not used: mesh always uses swept method for midphase.
		PxcGetMaterialShapeHeightField,		//PxGeometryType::eHEIGHTFIELD		//TODO: make HF midphase that will mask this
		PxcGetMaterialShapeShape,			//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::ePARTICLESYSTEM
	{
		0,									//PxGeometryType::eSPHERE
		0,									//PxGeometryType::ePLANE
		0,									//PxGeometryType::eCAPSULE
		0,									//PxGeometryType::eBOX
		0,									//PxGeometryType::eCONVEXCORE
		0,									//PxGeometryType::eCONVEXMESH
		PxcGetMaterialSoftBodySoftBody,		//PxGeometryType::ePARTICLESYSTEM
		PxcGetMaterialSoftBodySoftBody,		//PxGeometryType::eTETRAHEDRONMESH
		PxcGetMaterialSoftBodyMesh,			//PxGeometryType::eTRIANGLEMESH		//not used: mesh always uses swept method for midphase.
		PxcGetMaterialSoftBodyHeightField,	//PxGeometryType::eHEIGHTFIELD		//TODO: make HF midphase that will mask this
		PxcGetMaterialShapeShape,			//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::eTETRAHEDRONMESH
	{
		0,									//PxGeometryType::eSPHERE
		0,									//PxGeometryType::ePLANE
		0,									//PxGeometryType::eCAPSULE
		0,									//PxGeometryType::eBOX
		0,									//PxGeometryType::eCONVEXCORE
		0,									//PxGeometryType::eCONVEXMESH
		0,									//PxGeometryType::ePARTICLESYSTEM
		PxcGetMaterialSoftBodySoftBody,		//PxGeometryType::eTETRAHEDRONMESH
		PxcGetMaterialSoftBodyMesh,			//PxGeometryType::eTRIANGLEMESH		//not used: mesh always uses swept method for midphase.
		PxcGetMaterialSoftBodyHeightField,	//PxGeometryType::eHEIGHTFIELD		//TODO: make HF midphase that will mask this
		PxcGetMaterialShapeShape,			//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::eTRIANGLEMESH
	{
		0,								//PxGeometryType::eSPHERE
		0,								//PxGeometryType::ePLANE
		0,								//PxGeometryType::eCAPSULE
		0,								//PxGeometryType::eBOX
		0,								//PxGeometryType::eCONVEXCORE
		0,								//PxGeometryType::eCONVEXMESH
		0,								//PxGeometryType::ePARTICLESYSTEM
		0,								//PxGeometryType::eTETRAHEDRONMESH
		PxcGetMaterialShapeShape,		//PxGeometryType::eTRIANGLEMESH	   // mesh-mesh via SDF (single material)
		0,								//PxGeometryType::eHEIGHTFIELD
		PxcGetMaterialShapeShape,		//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::eHEIGHTFIELD
	{
		0,								//PxGeometryType::eSPHERE
		0,								//PxGeometryType::ePLANE
		0,								//PxGeometryType::eCAPSULE
		0,								//PxGeometryType::eBOX
		0,								//PxGeometryType::eCONVEXCORE
		0,								//PxGeometryType::eCONVEXMESH
		0,								//PxGeometryType::ePARTICLESYSTEM
		0,								//PxGeometryType::eTETRAHEDRONMESH
		0,								//PxGeometryType::eTRIANGLEMESH
		0,								//PxGeometryType::eHEIGHTFIELD
		PxcGetMaterialShapeShape,		//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::eCUSTOM
	{
		0,								//PxGeometryType::eSPHERE
		0,								//PxGeometryType::ePLANE
		0,								//PxGeometryType::eCAPSULE
		0,								//PxGeometryType::eBOX
		0,								//PxGeometryType::eCONVEXCORE
		0,								//PxGeometryType::eCONVEXMESH
		0,								//PxGeometryType::ePARTICLESYSTEM
		0,								//PxGeometryType::eTETRAHEDRONMESH
		0,								//PxGeometryType::eTRIANGLEMESH
		0,								//PxGeometryType::eHEIGHTFIELD
		PxcGetMaterialShapeShape,		//PxGeometryType::eCUSTOM
	},

};
PX_COMPILE_TIME_ASSERT(sizeof(g_GetMaterialMethodTable) / sizeof(g_GetMaterialMethodTable[0]) == PxGeometryType::eGEOMETRY_COUNT);

}
