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

#include "PxvGlobals.h"
#include "PxsContext.h"
#include "PxcContactMethodImpl.h"
#include "GuContactMethodImpl.h"

#if PX_SUPPORT_GPU_PHYSX
	#include "PxPhysXGpu.h"
	static physx::PxPhysXGpu* gPxPhysXGpu = NULL;
#endif

namespace physx
{

PxvOffsetTable gPxvOffsetTable;

void PxvInit(const PxvOffsetTable& offsetTable)
{
#if PX_SUPPORT_GPU_PHYSX
	gPxPhysXGpu = NULL;
#endif
	gPxvOffsetTable = offsetTable;
}

void PxvTerm()
{
#if PX_SUPPORT_GPU_PHYSX
	PX_RELEASE(gPxPhysXGpu);
#endif
}

}

#if PX_SUPPORT_GPU_PHYSX
namespace physx
{
	//forward declare stuff from PxPhysXGpuModuleLoader.cpp
	void PxLoadPhysxGPUModule(const char* appGUID);
	void PxUnloadPhysxGPUModule();
	typedef physx::PxPhysXGpu* (PxCreatePhysXGpu_FUNC)();
	extern PxCreatePhysXGpu_FUNC* g_PxCreatePhysXGpu_Func;

	PxPhysXGpu* PxvGetPhysXGpu(bool createIfNeeded)
	{
		if (!gPxPhysXGpu && createIfNeeded)
		{
#ifdef PX_PHYSX_GPU_STATIC
			gPxPhysXGpu = PxCreatePhysXGpu();
#else
			PxLoadPhysxGPUModule(NULL);
			if (g_PxCreatePhysXGpu_Func)
			{
				gPxPhysXGpu = g_PxCreatePhysXGpu_Func();
			}
#endif
		}
		
		return gPxPhysXGpu;
	}

	// PT: added for the standalone GPU BP but we may want to revisit this
	void PxvReleasePhysXGpu(PxPhysXGpu* gpu)
	{
		PX_ASSERT(gpu==gPxPhysXGpu);
		PxUnloadPhysxGPUModule();
		PX_RELEASE(gpu);
		gPxPhysXGpu = NULL;
	}
}
#endif

#include "common/PxMetaData.h"
#include "PxsFEMClothMaterialCore.h"
#include "PxsFEMSoftBodyMaterialCore.h"
#include "PxsFLIPMaterialCore.h"
#include "PxsMPMMaterialCore.h"
#include "PxsPBDMaterialCore.h"
#include "PxsMaterialCore.h"

namespace physx
{

template<> void PxsMaterialCore::getBinaryMetaData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_TYPEDEF(stream,	PxCombineMode::Enum, PxU32)
	PX_DEF_BIN_METADATA_TYPEDEF(stream,	PxMaterialFlags, PxU16)

	PX_DEF_BIN_METADATA_CLASS(stream,	PxsMaterialCore)

	// MaterialData
	PX_DEF_BIN_METADATA_ITEM(stream,	PxsMaterialCore, PxReal,			dynamicFriction,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,	PxsMaterialCore, PxReal,			staticFriction,		0)
	PX_DEF_BIN_METADATA_ITEM(stream,	PxsMaterialCore, PxReal,			restitution,		0)
	PX_DEF_BIN_METADATA_ITEM(stream,	PxsMaterialCore, PxReal,			damping,			0)

	PX_DEF_BIN_METADATA_ITEM(stream,	PxsMaterialCore, PxMaterialFlags,	flags,				0)
	PX_DEF_BIN_METADATA_ITEM(stream,	PxsMaterialCore, PxU8,				fricCombineMode,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,	PxsMaterialCore, PxU8,				restCombineMode,	0)

	// MaterialCore
	PX_DEF_BIN_METADATA_ITEM(stream,	PxsMaterialCore, PxMaterial,		mMaterial,			PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream,	PxsMaterialCore, PxU16,				mMaterialIndex,		PxMetaDataFlag::eHANDLE)
}

template<> void PxsFEMSoftBodyMaterialCore::getBinaryMetaData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream, PxsFEMSoftBodyMaterialCore)

	// MaterialData
	PX_DEF_BIN_METADATA_ITEM(stream, PxsFEMSoftBodyMaterialCore, PxReal, youngs,				0)
	PX_DEF_BIN_METADATA_ITEM(stream, PxsFEMSoftBodyMaterialCore, PxReal, poissons,				0)
	PX_DEF_BIN_METADATA_ITEM(stream, PxsFEMSoftBodyMaterialCore, PxReal, dynamicFriction,		0)
	PX_DEF_BIN_METADATA_ITEM(stream, PxsFEMSoftBodyMaterialCore, PxReal, damping,				0)
	PX_DEF_BIN_METADATA_ITEM(stream, PxsFEMSoftBodyMaterialCore, PxU16,  dampingScale,			0)
	PX_DEF_BIN_METADATA_ITEM(stream, PxsFEMSoftBodyMaterialCore, PxU16,  materialModel,         0)
	PX_DEF_BIN_METADATA_ITEM(stream, PxsFEMSoftBodyMaterialCore, PxReal, deformThreshold,		0)
	PX_DEF_BIN_METADATA_ITEM(stream, PxsFEMSoftBodyMaterialCore, PxReal, deformLowLimitRatio,	0)
	PX_DEF_BIN_METADATA_ITEM(stream, PxsFEMSoftBodyMaterialCore, PxReal, deformHighLimitRatio,	0)

	// MaterialCore
	PX_DEF_BIN_METADATA_ITEM(stream, PxsFEMSoftBodyMaterialCore, PxFEMSoftBodyMaterial,	mMaterial,		PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream, PxsFEMSoftBodyMaterialCore, PxU16,					mMaterialIndex,	PxMetaDataFlag::eHANDLE)
}

template<> void PxsFEMClothMaterialCore::getBinaryMetaData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream, PxsFEMClothMaterialCore)

	// MaterialData
	PX_DEF_BIN_METADATA_ITEM(stream, PxsFEMClothMaterialCore, PxReal, youngs,				0)
	PX_DEF_BIN_METADATA_ITEM(stream, PxsFEMClothMaterialCore, PxReal, poissons,				0)
	PX_DEF_BIN_METADATA_ITEM(stream, PxsFEMClothMaterialCore, PxReal, dynamicFriction,		0)
	PX_DEF_BIN_METADATA_ITEM(stream, PxsFEMClothMaterialCore, PxReal, thickness,			0)
	
	// MaterialCore
	PX_DEF_BIN_METADATA_ITEM(stream, PxsFEMClothMaterialCore, PxFEMClothMaterial,	mMaterial,		PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream, PxsFEMClothMaterialCore, PxU16,				mMaterialIndex,	PxMetaDataFlag::eHANDLE)
}

template<> void PxsPBDMaterialCore::getBinaryMetaData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream, PxsPBDMaterialCore)

	// MaterialData
	PX_DEF_BIN_METADATA_ITEM(stream, PxsPBDMaterialCore, PxReal, friction, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, PxsPBDMaterialCore, PxReal, damping, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, PxsPBDMaterialCore, PxReal, adhesion, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, PxsPBDMaterialCore, PxReal, gravityScale, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, PxsPBDMaterialCore, PxReal, adhesionRadiusScale, 0)

	PX_DEF_BIN_METADATA_ITEM(stream, PxsPBDMaterialCore, PxU32, flags, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, PxsPBDMaterialCore, PxReal, viscosity, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, PxsPBDMaterialCore, PxReal, vorticityConfinement, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, PxsPBDMaterialCore, PxReal, surfaceTension, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, PxsPBDMaterialCore, PxReal, cohesion, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, PxsPBDMaterialCore, PxReal, lift, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, PxsPBDMaterialCore, PxReal, drag, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, PxsPBDMaterialCore, PxReal, cflCoefficient, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, PxsPBDMaterialCore, PxReal, particleFrictionScale, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, PxsPBDMaterialCore, PxReal, particleAdhesionScale, 0)
	
	// MaterialCore
	PX_DEF_BIN_METADATA_ITEM(stream, PxsPBDMaterialCore, PxPBDMaterial, mMaterial, PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream, PxsPBDMaterialCore, PxU16, mMaterialIndex, PxMetaDataFlag::eHANDLE)
}

template<> void PxsFLIPMaterialCore::getBinaryMetaData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream, PxsFLIPMaterialCore)

	// MaterialData
	PX_DEF_BIN_METADATA_ITEM(stream, PxsFLIPMaterialCore, PxReal, friction, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, PxsFLIPMaterialCore, PxReal, damping, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, PxsFLIPMaterialCore, PxReal, adhesion, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, PxsFLIPMaterialCore, PxReal, gravityScale, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, PxsFLIPMaterialCore, PxReal, adhesionRadiusScale, 0)

	PX_DEF_BIN_METADATA_ITEM(stream, PxsFLIPMaterialCore, PxReal, viscosity, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, PxsFLIPMaterialCore, PxReal, surfaceTension, 0)
	
	// MaterialCore
	PX_DEF_BIN_METADATA_ITEM(stream, PxsFLIPMaterialCore, PxFLIPMaterial, mMaterial, PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream, PxsFLIPMaterialCore, PxU16, mMaterialIndex, PxMetaDataFlag::eHANDLE)
}

template<> void PxsMPMMaterialCore::getBinaryMetaData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream, PxsMPMMaterialCore)

	PX_DEF_BIN_METADATA_TYPEDEF(stream,	PxIntBool, PxU32)

	// MaterialData
	PX_DEF_BIN_METADATA_ITEM(stream, PxsMPMMaterialCore, PxReal, friction, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, PxsMPMMaterialCore, PxReal, damping, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, PxsMPMMaterialCore, PxReal, adhesion, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, PxsMPMMaterialCore, PxReal, gravityScale, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, PxsMPMMaterialCore, PxReal, adhesionRadiusScale, 0)

	PX_DEF_BIN_METADATA_ITEM(stream, PxsMPMMaterialCore, PxIntBool, isPlastic, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, PxsMPMMaterialCore, PxReal, youngsModulus, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, PxsMPMMaterialCore, PxReal, poissonsRatio, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, PxsMPMMaterialCore, PxReal, hardening, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, PxsMPMMaterialCore, PxReal, criticalCompression, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, PxsMPMMaterialCore, PxReal, criticalStretch, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, PxsMPMMaterialCore, PxReal, tensileDamageSensitivity, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, PxsMPMMaterialCore, PxReal, compressiveDamageSensitivity, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, PxsMPMMaterialCore, PxReal, attractiveForceResidual, 0)
	
	// MaterialCore
	PX_DEF_BIN_METADATA_ITEM(stream, PxsMPMMaterialCore, PxMPMMaterial, mMaterial, PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream, PxsMPMMaterialCore, PxU16, mMaterialIndex, PxMetaDataFlag::eHANDLE)
}

}

