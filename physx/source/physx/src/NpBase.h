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

#ifndef NP_BASE_H
#define NP_BASE_H

#include "foundation/PxUserAllocated.h"
#include "NpScene.h"

#if PX_SUPPORT_PVD
	// PT: updatePvdProperties() is overloaded and the compiler needs to know 'this' type to do the right thing.
	// Thus we can't just move this as an inlined Base function.
	#define UPDATE_PVD_PROPERTY															\
		{																				\
			NpScene* scene = getNpScene();	/* shared shapes also return zero here */	\
			if(scene)																	\
				scene->getScenePvdClientInternal().updatePvdProperties(this);			\
		}
#else
	#define UPDATE_PVD_PROPERTY
#endif

///////////////////////////////////////////////////////////////////////////////
// PT: technically the following macros should all be "NP" macros (they're not exposed to the public API)
// but I only renamed the local ones (used in NpBase.h) as "NP", while the other "PX" are used by clients
// of these macros in other Np files. The PX_CHECK names are very misleading, since these checks seem to
// stay in all builds, contrary to other macros like PX_CHECK_AND_RETURN.
///////////////////////////////////////////////////////////////////////////////

#define NP_API_READ_WRITE_ERROR_MSG(text) PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, text)

// some API read calls are not allowed while the simulation is running since the properties might get
// written to during simulation. Some of those are allowed while collision is running though or in callbacks
// like contact modification, contact reports etc. Furthermore, it is ok to read all of them between fetchCollide
// and advance.
#define NP_API_READ_FORBIDDEN(npScene) (npScene && npScene->isAPIReadForbidden())
#define NP_API_READ_FORBIDDEN_EXCEPT_COLLIDE(npScene) (npScene && npScene->isAPIReadForbidden() && (!npScene->isCollisionPhaseActive()))

///////////////////////////////////////////////////////////////////////////////

#define PX_CHECK_SCENE_API_READ_FORBIDDEN(npScene, text)	\
	if(NP_API_READ_FORBIDDEN(npScene))						\
	{														\
		NP_API_READ_WRITE_ERROR_MSG(text);					\
		return;												\
	}

#define PX_CHECK_SCENE_API_READ_FORBIDDEN_AND_RETURN_VAL(npScene, text, retValue)	\
	if(NP_API_READ_FORBIDDEN(npScene))												\
	{																				\
		NP_API_READ_WRITE_ERROR_MSG(text);											\
		return retValue;															\
	}

#define PX_CHECK_SCENE_API_READ_FORBIDDEN_EXCEPT_COLLIDE(npScene, text)	\
	if(NP_API_READ_FORBIDDEN_EXCEPT_COLLIDE(npScene))					\
	{																	\
		NP_API_READ_WRITE_ERROR_MSG(text);								\
		return;															\
	}

#define PX_CHECK_SCENE_API_READ_FORBIDDEN_EXCEPT_COLLIDE_AND_RETURN_VAL(npScene, text, retValue)	\
	if(NP_API_READ_FORBIDDEN_EXCEPT_COLLIDE(npScene))												\
	{																								\
		NP_API_READ_WRITE_ERROR_MSG(text);															\
		return retValue;																			\
	}

///////////////////////////////////////////////////////////////////////////////

#define NP_API_WRITE_FORBIDDEN(npScene) (npScene && npScene->isAPIWriteForbidden())

// some API write calls are allowed between fetchCollide and advance
#define NP_API_WRITE_FORBIDDEN_EXCEPT_SPLIT_SIM(npScene) (npScene && npScene->isAPIWriteForbidden() && (npScene->getSimulationStage() != Sc::SimulationStage::eFETCHCOLLIDE))

///////////////////////////////////////////////////////////////////////////////

#define PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, text)	\
	if(NP_API_WRITE_FORBIDDEN(npScene))						\
	{														\
		NP_API_READ_WRITE_ERROR_MSG(text);					\
		return;												\
	}

#define PX_CHECK_SCENE_API_WRITE_FORBIDDEN_AND_RETURN_VAL(npScene, text, retValue)	\
	if(NP_API_WRITE_FORBIDDEN(npScene))												\
	{																				\
		NP_API_READ_WRITE_ERROR_MSG(text);											\
		return retValue;															\
	}

#define PX_CHECK_SCENE_API_WRITE_FORBIDDEN_EXCEPT_SPLIT_SIM(npScene, text)	\
	if(NP_API_WRITE_FORBIDDEN_EXCEPT_SPLIT_SIM(npScene))					\
	{																		\
		NP_API_READ_WRITE_ERROR_MSG(text);									\
		return;																\
	}

///////////////////////////////////////////////////////////////////////////////

#define NP_UNUSED_BASE_INDEX	0x07ffffff
#define NP_BASE_INDEX_MASK		0x07ffffff
#define NP_BASE_INDEX_SHIFT		27

namespace physx
{
	struct NpType
	{
		enum Enum
		{
			eUNDEFINED,
			eSHAPE,
			eBODY,
			eBODY_FROM_ARTICULATION_LINK,
			eRIGID_STATIC,
			eCONSTRAINT,
			eARTICULATION,
			eARTICULATION_JOINT,
			eARTICULATION_SPATIAL_TENDON,
			eARTICULATION_ATTACHMENT,
			eARTICULATION_FIXED_TENDON,
			eARTICULATION_TENDON_JOINT,
			eARTICULATION_MIMIC_JOINT,
			eAGGREGATE,
			eDEFORMABLE_SURFACE,
			eDEFORMABLE_VOLUME,
			ePBD_PARTICLESYSTEM,
			eDEFORMABLE_ATTACHMENT,
			eDEFORMABLE_ELEMENT_FILTER,
			eTYPE_COUNT,

			eFORCE_DWORD = 0x7fffffff
		};
	};

	// PT: we're going to store that type on 5 bits, leaving 27 bits for the base index.
	PX_COMPILE_TIME_ASSERT(NpType::eTYPE_COUNT<32);

	class NpBase : public PxUserAllocated
	{
										PX_NOCOPY(NpBase)
		public:
// PX_SERIALIZATION				
										NpBase(const PxEMPTY) :
											mScene		(NULL),
											mFreeSlot	(0)
										{
											// PT: preserve type, reset base index
											setBaseIndex(NP_UNUSED_BASE_INDEX);
										}
		static			void			getBinaryMetaData(PxOutputStream& stream);
//~PX_SERIALIZATION
										NpBase(NpType::Enum type) :
											mScene		(NULL),
											mFreeSlot	(0)
										{
											mBaseIndexAndType = (PxU32(type)<<NP_BASE_INDEX_SHIFT)|NP_UNUSED_BASE_INDEX;
										}

		PX_INLINE		bool			isAPIWriteForbidden()				const	{ return NP_API_WRITE_FORBIDDEN(mScene);						}
		PX_INLINE		bool			isAPIWriteForbiddenExceptSplitSim() const	{ return NP_API_WRITE_FORBIDDEN_EXCEPT_SPLIT_SIM(mScene);		}

		PX_FORCE_INLINE	NpType::Enum	getNpType()							const	{ return NpType::Enum(mBaseIndexAndType>>NP_BASE_INDEX_SHIFT);	}
		PX_FORCE_INLINE void			setNpScene(NpScene* scene)					{ mScene = scene;												}
		PX_FORCE_INLINE	NpScene*		getNpScene()						const	{ return mScene;												}

		PX_FORCE_INLINE	PxU32			getBaseIndex()						const	{ return mBaseIndexAndType & NP_BASE_INDEX_MASK;				}
		PX_FORCE_INLINE	void			setBaseIndex(PxU32 index)
										{
											PX_ASSERT(!(index & ~NP_BASE_INDEX_MASK));
											const PxU32 type = mBaseIndexAndType & ~NP_BASE_INDEX_MASK;
											mBaseIndexAndType = index|type;
										}
		protected:
										~NpBase(){}
		private:
						NpScene*		mScene;
						PxU32			mBaseIndexAndType;
		protected:
						PxU32			mFreeSlot;
	};
}

#endif
