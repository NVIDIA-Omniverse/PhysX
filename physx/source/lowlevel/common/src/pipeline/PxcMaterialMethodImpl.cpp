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

static void PxcGetMaterialShape(const PxsShapeCore* shape, const PxU32 index, PxcNpThreadContext& context, PxsMaterialInfo* materialInfo)
{
	const PxU16 materialIndex = shape->mMaterialIndex;
	const PxU32 count = context.mContactBuffer.count;
	PX_ASSERT(index==0 || index==1);
	for(PxU32 i=0; i<count; i++)
		(&materialInfo[i].mMaterialIndex0)[index] = materialIndex;
}

static void PxcGetMaterialShapeShape(const PxsShapeCore* shape0, const PxsShapeCore* shape1, PxcNpThreadContext& context, PxsMaterialInfo* materialInfo)
{
	const PxU16 materialIndex0 = shape0->mMaterialIndex;
	const PxU16 materialIndex1 = shape1->mMaterialIndex;
	const PxU32 count = context.mContactBuffer.count;
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

static void PxcGetMaterialMesh(const PxsShapeCore* shape, const PxU32 index, PxcNpThreadContext& context, PxsMaterialInfo* materialInfo)
{
	PX_ASSERT(index == 1);
	PX_UNUSED(index);
	const PxTriangleMeshGeometryLL& shapeMesh = shape->mGeometry.get<const PxTriangleMeshGeometryLL>();
	if(shapeMesh.materialsLL.numIndices <= 1)
	{
		PxcGetMaterialShape(shape, index, context, materialInfo);
	}
	else
	{
		PxContactBuffer& contactBuffer = context.mContactBuffer;
		const PxU32 count = contactBuffer.count;
		const PxU16* eaMaterialIndices = getMaterialIndicesLL(shapeMesh);
		const PxU16* indices = shapeMesh.materialsLL.indices;
		for(PxU32 i=0; i<count; i++)
		{
			PxContactPoint& contact = contactBuffer.contacts[i];
			const PxU32 localMaterialIndex = eaMaterialIndices ? eaMaterialIndices[contact.internalFaceIndex1] : 0;//shapeMesh.triangleMesh->getTriangleMaterialIndex(contact.featureIndex1);
			(&materialInfo[i].mMaterialIndex0)[index] = indices[localMaterialIndex];
		}
	}
}

static void PxcGetMaterialShapeMesh(const PxsShapeCore* shape0, const PxsShapeCore* shape1, PxcNpThreadContext& context, PxsMaterialInfo* materialInfo)
{
	const PxTriangleMeshGeometryLL& shapeMesh = shape1->mGeometry.get<const PxTriangleMeshGeometryLL>();
	if(shapeMesh.materialsLL.numIndices <= 1)
	{
		PxcGetMaterialShapeShape(shape0, shape1, context,  materialInfo);
	}
	else
	{
		PxContactBuffer& contactBuffer = context.mContactBuffer;
		const PxU32 count = contactBuffer.count;
		const PxU16* eaMaterialIndices = getMaterialIndicesLL(shapeMesh);
		const PxU16* indices = shapeMesh.materialsLL.indices;
		const PxU16 materialIndex0 = shape0->mMaterialIndex;
		for(PxU32 i=0; i<count; i++)
		{
			PxContactPoint& contact = contactBuffer.contacts[i];
			materialInfo[i].mMaterialIndex0 = materialIndex0;

			const PxU32 localMaterialIndex = eaMaterialIndices ? eaMaterialIndices[contact.internalFaceIndex1] : 0;//shapeMesh.triangleMesh->getTriangleMaterialIndex(contact.featureIndex1);
			materialInfo[i].mMaterialIndex1 = indices[localMaterialIndex];
		}
	}
}

static void PxcGetMaterialSoftBodyMesh(const PxsShapeCore* shape0, const PxsShapeCore* shape1, PxcNpThreadContext& context, PxsMaterialInfo* materialInfo)
{
	// PT: TODO: check this, it reads shape0 and labels it shapeMesh1? It's otherwise the same code as PxcGetMaterialShapeMesh ?
	const PxTriangleMeshGeometryLL& shapeMesh1 = shape0->mGeometry.get<const PxTriangleMeshGeometryLL>();
	if (shapeMesh1.materialsLL.numIndices <= 1)
	{
		PxcGetMaterialShapeShape(shape0, shape1, context, materialInfo);
	}
	else
	{
		PxContactBuffer& contactBuffer = context.mContactBuffer;
		const PxU32 count = contactBuffer.count;
		const PxU16* eaMaterialIndices = getMaterialIndicesLL(shapeMesh1);
		const PxU16* indices = shapeMesh1.materialsLL.indices;
		const PxU16 materialIndex0 = shape0->mMaterialIndex;
		for (PxU32 i = 0; i<count; i++)
		{
			PxContactPoint& contact = contactBuffer.contacts[i];
			materialInfo[i].mMaterialIndex0 = materialIndex0;

			const PxU32 localMaterialIndex = eaMaterialIndices ? eaMaterialIndices[contact.internalFaceIndex1] : 0;//shapeMesh.triangleMesh->getTriangleMaterialIndex(contact.featureIndex1);
																						   //contact.featureIndex1 = shapeMesh.materials.indices[localMaterialIndex];
			materialInfo[i].mMaterialIndex1 = indices[localMaterialIndex];
		}
	}
}

///////////////////////////////////////////////////////////////////////////////

static PxU32 GetMaterialIndex(const Gu::HeightFieldData* hfData, PxU32 triangleIndex)
{
	const PxU32 sampleIndex = triangleIndex >> 1;
	const bool isFirstTriangle = (triangleIndex & 0x1) == 0;

	//get sample
	const PxHeightFieldSample* hf = &hfData->samples[sampleIndex];
	return isFirstTriangle ? hf->materialIndex0 : hf->materialIndex1;
}

static void PxcGetMaterialHeightField(const PxsShapeCore* shape, const PxU32 index, PxcNpThreadContext& context, PxsMaterialInfo* materialInfo)
{
	PX_ASSERT(index == 1);
	PX_UNUSED(index);
	const PxHeightFieldGeometryLL& hfGeom = shape->mGeometry.get<const PxHeightFieldGeometryLL>();
	if(hfGeom.materialsLL.numIndices <= 1)
	{
		PxcGetMaterialShape(shape, index, context, materialInfo);
	}
	else
	{
		const PxContactBuffer& contactBuffer = context.mContactBuffer;
		const PxU32 count = contactBuffer.count;
		const PxU16* materialIndices = hfGeom.materialsLL.indices;
			
		const Gu::HeightFieldData* hf = &static_cast<const Gu::HeightField*>(hfGeom.heightField)->getData();
		
		for(PxU32 i=0; i<count; i++)
		{
			const PxContactPoint& contact = contactBuffer.contacts[i];
			const PxU32 localMaterialIndex = GetMaterialIndex(hf, contact.internalFaceIndex1);
			(&materialInfo[i].mMaterialIndex0)[index] = materialIndices[localMaterialIndex];
		}
	}
}

static void PxcGetMaterialShapeHeightField(const PxsShapeCore* shape0, const PxsShapeCore* shape1, PxcNpThreadContext& context,  PxsMaterialInfo* materialInfo)
{
	const PxHeightFieldGeometryLL& hfGeom = shape1->mGeometry.get<const PxHeightFieldGeometryLL>();
	if(hfGeom.materialsLL.numIndices <= 1)
	{
		PxcGetMaterialShapeShape(shape0, shape1, context, materialInfo);
	}
	else
	{
		const PxContactBuffer& contactBuffer = context.mContactBuffer;
		const PxU32 count = contactBuffer.count;
		const PxU16* materialIndices = hfGeom.materialsLL.indices;
			
		const Gu::HeightFieldData* hf = &static_cast<const Gu::HeightField*>(hfGeom.heightField)->getData();
		
		for(PxU32 i=0; i<count; i++)
		{
			const PxContactPoint& contact = contactBuffer.contacts[i];
			materialInfo[i].mMaterialIndex0 = shape0->mMaterialIndex;
			//contact.featureIndex0 = shape0->materialIndex;
			const PxU32 localMaterialIndex = GetMaterialIndex(hf, contact.internalFaceIndex1);
			//contact.featureIndex1 = materialIndices[localMaterialIndex];
			PX_ASSERT(localMaterialIndex<hfGeom.materialsLL.numIndices);
			materialInfo[i].mMaterialIndex1 = materialIndices[localMaterialIndex];
		}
	}
}

static void PxcGetMaterialSoftBodyHeightField(const PxsShapeCore* shape0, const PxsShapeCore* shape1, PxcNpThreadContext& context, PxsMaterialInfo* materialInfo)
{
	const PxHeightFieldGeometryLL& hfGeom = shape1->mGeometry.get<const PxHeightFieldGeometryLL>();
	if (hfGeom.materialsLL.numIndices <= 1)
	{
		PxcGetMaterialShapeShape(shape0, shape1, context, materialInfo);
	}
	else
	{
		const PxContactBuffer& contactBuffer = context.mContactBuffer;
		const PxU32 count = contactBuffer.count;
		const PxU16* materialIndices = hfGeom.materialsLL.indices;

		const Gu::HeightFieldData* hf = &static_cast<const Gu::HeightField*>(hfGeom.heightField)->getData();

		for(PxU32 i=0; i<count; i++)
		{
			const PxContactPoint& contact = contactBuffer.contacts[i];
			materialInfo[i].mMaterialIndex0 = shape0->mMaterialIndex;
			//contact.featureIndex0 = shape0->materialIndex;
			const PxU32 localMaterialIndex = GetMaterialIndex(hf, contact.internalFaceIndex1);
			//contact.featureIndex1 = materialIndices[localMaterialIndex];
			PX_ASSERT(localMaterialIndex<hfGeom.materialsLL.numIndices);
			materialInfo[i].mMaterialIndex1 = materialIndices[localMaterialIndex];
		}
	}
}

///////////////////////////////////////////////////////////////////////////////

static void PxcGetMaterialSoftBody(const PxsShapeCore* shape, const PxU32 index, PxcNpThreadContext& context, PxsMaterialInfo* materialInfo)
{
	PX_ASSERT(index == 1);
	PX_UNUSED(index);
	PxcGetMaterialShape(shape, index, context, materialInfo);
}

static void PxcGetMaterialShapeSoftBody(const PxsShapeCore* shape0, const PxsShapeCore* shape1, PxcNpThreadContext& context, PxsMaterialInfo* materialInfo)
{
	PxcGetMaterialShapeShape(shape0, shape1, context, materialInfo);
}

static void PxcGetMaterialSoftBodySoftBody(const PxsShapeCore* shape0, const PxsShapeCore* shape1, PxcNpThreadContext& context, PxsMaterialInfo* materialInfo)
{
	PxcGetMaterialShapeShape(shape0, shape1, context, materialInfo);
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
	PxcGetMaterialShape,		//PxGeometryType::eCONVEXMESH
	PxcGetMaterialSoftBody,		//PxGeometryType::ePARTICLESYSTEM
	PxcGetMaterialSoftBody,		//PxGeometryType::eTETRAHEDRONMESH
	PxcGetMaterialMesh,			//PxGeometryType::eTRIANGLEMESH	//not used: mesh always uses swept method for midphase.
	PxcGetMaterialHeightField,	//PxGeometryType::eHEIGHTFIELD	//TODO: make HF midphase that will mask this
	PxcGetMaterialSoftBody,		//PxGeometryType::eHAIRSYSTEM
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
		PxcGetMaterialShapeShape,			//PxGeometryType::eCONVEXMESH
		PxcGetMaterialShapeSoftBody,		//PxGeometryType::ePARTICLESYSTEM
		PxcGetMaterialShapeSoftBody,		//PxGeometryType::eTETRAHEDRONMESH
		PxcGetMaterialShapeMesh,			//PxGeometryType::eTRIANGLEMESH	//not used: mesh always uses swept method for midphase.
		PxcGetMaterialShapeHeightField,		//PxGeometryType::eHEIGHTFIELD	//TODO: make HF midphase that will mask this
		PxcGetMaterialShapeSoftBody,		//PxGeometryType::eHAIRSYSTEM
		PxcGetMaterialShapeShape,			//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::ePLANE
	{
		0,									//PxGeometryType::eSPHERE
		0,									//PxGeometryType::ePLANE
		PxcGetMaterialShapeShape,			//PxGeometryType::eCAPSULE
		PxcGetMaterialShapeShape,			//PxGeometryType::eBOX
		PxcGetMaterialShapeShape,			//PxGeometryType::eCONVEXMESH
		PxcGetMaterialShapeSoftBody,		//PxGeometryType::ePARTICLESYSTEM
		PxcGetMaterialShapeSoftBody,		//PxGeometryType::eTETRAHEDRONMESH
		0,									//PxGeometryType::eTRIANGLEMESH
		0,									//PxGeometryType::eHEIGHTFIELD
		PxcGetMaterialShapeSoftBody,		//PxGeometryType::eHAIRSYSTEM
		PxcGetMaterialShapeShape,			//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::eCAPSULE
	{
		0,									//PxGeometryType::eSPHERE
		0,									//PxGeometryType::ePLANE
		PxcGetMaterialShapeShape,			//PxGeometryType::eCAPSULE
		PxcGetMaterialShapeShape,			//PxGeometryType::eBOX
		PxcGetMaterialShapeShape,			//PxGeometryType::eCONVEXMESH
		PxcGetMaterialShapeSoftBody,		//PxGeometryType::ePARTICLESYSTEM
		PxcGetMaterialShapeSoftBody,		//PxGeometryType::eTETRAHEDRONMESH
		PxcGetMaterialShapeMesh,			//PxGeometryType::eTRIANGLEMESH		//not used: mesh always uses swept method for midphase.
		PxcGetMaterialShapeHeightField,		//PxGeometryType::eHEIGHTFIELD		//TODO: make HF midphase that will mask this
		PxcGetMaterialShapeSoftBody,		//PxGeometryType::eHAIRSYSTEM
		PxcGetMaterialShapeShape,			//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::eBOX
	{
		0,									//PxGeometryType::eSPHERE
		0,									//PxGeometryType::ePLANE
		0,									//PxGeometryType::eCAPSULE
		PxcGetMaterialShapeShape,			//PxGeometryType::eBOX
		PxcGetMaterialShapeShape,			//PxGeometryType::eCONVEXMESH
		PxcGetMaterialShapeSoftBody,		//PxGeometryType::ePARTICLESYSTEM
		PxcGetMaterialShapeSoftBody,		//PxGeometryType::eTETRAHEDRONMESH
		PxcGetMaterialShapeMesh,			//PxGeometryType::eTRIANGLEMESH		//not used: mesh always uses swept method for midphase.
		PxcGetMaterialShapeHeightField,		//PxGeometryType::eHEIGHTFIELD		//TODO: make HF midphase that will mask this
		PxcGetMaterialShapeSoftBody,		//PxGeometryType::eHAIRSYSTEM
		PxcGetMaterialShapeShape,			//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::eCONVEXMESH
	{
		0,									//PxGeometryType::eSPHERE
		0,									//PxGeometryType::ePLANE
		0,									//PxGeometryType::eCAPSULE
		0,									//PxGeometryType::eBOX
		PxcGetMaterialShapeShape,			//PxGeometryType::eCONVEXMESH
		PxcGetMaterialShapeSoftBody,		//PxGeometryType::ePARTICLESYSTEM
		PxcGetMaterialShapeSoftBody,		//PxGeometryType::eTETRAHEDRONMESH
		PxcGetMaterialShapeMesh,			//PxGeometryType::eTRIANGLEMESH		//not used: mesh always uses swept method for midphase.
		PxcGetMaterialShapeHeightField,		//PxGeometryType::eHEIGHTFIELD		//TODO: make HF midphase that will mask this
		PxcGetMaterialShapeSoftBody,		//PxGeometryType::eHAIRSYSTEM
		PxcGetMaterialShapeShape,			//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::ePARTICLESYSTEM
	{
		0,									//PxGeometryType::eSPHERE
		0,									//PxGeometryType::ePLANE
		0,									//PxGeometryType::eCAPSULE
		0,									//PxGeometryType::eBOX
		0,									//PxGeometryType::eCONVEXMESH
		PxcGetMaterialSoftBodySoftBody,		//PxGeometryType::ePARTICLESYSTEM
		PxcGetMaterialSoftBodySoftBody,		//PxGeometryType::eTETRAHEDRONMESH
		PxcGetMaterialSoftBodyMesh,			//PxGeometryType::eTRIANGLEMESH		//not used: mesh always uses swept method for midphase.
		PxcGetMaterialSoftBodyHeightField,	//PxGeometryType::eHEIGHTFIELD		//TODO: make HF midphase that will mask this
		PxcGetMaterialShapeShape,			//PxGeometryType::eHAIRYSTEM
		PxcGetMaterialShapeShape,			//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::eTETRAHEDRONMESH
	{
		0,									//PxGeometryType::eSPHERE
		0,									//PxGeometryType::ePLANE
		0,									//PxGeometryType::eCAPSULE
		0,									//PxGeometryType::eBOX
		0,									//PxGeometryType::eCONVEXMESH
		0,									//PxGeometryType::ePARTICLESYSTEM
		PxcGetMaterialSoftBodySoftBody,		//PxGeometryType::eTETRAHEDRONMESH
		PxcGetMaterialSoftBodyMesh,			//PxGeometryType::eTRIANGLEMESH		//not used: mesh always uses swept method for midphase.
		PxcGetMaterialSoftBodyHeightField,	//PxGeometryType::eHEIGHTFIELD		//TODO: make HF midphase that will mask this
		PxcGetMaterialShapeShape,			//PxGeometryType::eHAIRYSTEM
		PxcGetMaterialShapeShape,			//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::eTRIANGLEMESH
	{
		0,								//PxGeometryType::eSPHERE
		0,								//PxGeometryType::ePLANE
		0,								//PxGeometryType::eCAPSULE
		0,								//PxGeometryType::eBOX
		0,								//PxGeometryType::eCONVEXMESH
		0,								//PxGeometryType::ePARTICLESYSTEM
		0,								//PxGeometryType::eTETRAHEDRONMESH
		0,								//PxGeometryType::eTRIANGLEMESH
		0,								//PxGeometryType::eHEIGHTFIELD
		PxcGetMaterialShapeShape,		//PxGeometryType::eHAIRYSTEM
		PxcGetMaterialShapeShape,		//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::eHEIGHTFIELD
	{
		0,								//PxGeometryType::eSPHERE
		0,								//PxGeometryType::ePLANE
		0,								//PxGeometryType::eCAPSULE
		0,								//PxGeometryType::eBOX
		0,								//PxGeometryType::eCONVEXMESH
		0,								//PxGeometryType::ePARTICLESYSTEM
		0,								//PxGeometryType::eTETRAHEDRONMESH
		0,								//PxGeometryType::eTRIANGLEMESH
		0,								//PxGeometryType::eHEIGHTFIELD
		PxcGetMaterialShapeShape,		//PxGeometryType::eHAIRYSTEM
		PxcGetMaterialShapeShape,		//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::eHAIRSYSTEM
	{
		0,								//PxGeometryType::eSPHERE
		0,								//PxGeometryType::ePLANE
		0,								//PxGeometryType::eCAPSULE
		0,								//PxGeometryType::eBOX
		0,								//PxGeometryType::eCONVEXMESH
		0,								//PxGeometryType::ePARTICLESYSTEM
		0,								//PxGeometryType::eTETRAHEDRONMESH
		0,								//PxGeometryType::eTRIANGLEMESH
		0,								//PxGeometryType::eHEIGHTFIELD
		0,								//PxGeometryType::eHAIRSYSTEM
		PxcGetMaterialShapeShape,		//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::eCUSTOM
	{
		0,								//PxGeometryType::eSPHERE
		0,								//PxGeometryType::ePLANE
		0,								//PxGeometryType::eCAPSULE
		0,								//PxGeometryType::eBOX
		0,								//PxGeometryType::eCONVEXMESH
		0,								//PxGeometryType::ePARTICLESYSTEM
		0,								//PxGeometryType::eTETRAHEDRONMESH
		0,								//PxGeometryType::eTRIANGLEMESH
		0,								//PxGeometryType::eHEIGHTFIELD
		0,								//PxGeometryType::eHAIRSYSTEM
		PxcGetMaterialShapeShape,		//PxGeometryType::eCUSTOM
	},

};
PX_COMPILE_TIME_ASSERT(sizeof(g_GetMaterialMethodTable) / sizeof(g_GetMaterialMethodTable[0]) == PxGeometryType::eGEOMETRY_COUNT);

}
