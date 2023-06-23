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

#ifndef DY_HAIR_SYSTEM_H
#define DY_HAIR_SYSTEM_H

#include "DyHairSystemCore.h"
#include "PxvGeometry.h"

namespace physx
{
	namespace Sc
	{
		class HairSystemSim;
	}

	namespace Dy
	{
		typedef size_t HairSystemHandle;
		
		// forward-declarations
		class Context;
		class HairSystem;

		struct HairSystemDirtyFlag
		{
			enum Enum
			{
				// Changes are processed from highest (most fundamental) to lowest to ensure correct dependencies
				eNONE =							0,						//!> default, everything up-to-date
				ePARAMETERS =					1 <<  0,				//!> Parameters were changed
				eGRID_SIZE =					1 <<  1 | ePARAMETERS,	//!> Grid size was changed. sets ePARAMETERS because settings are stored there
				eRIGID_ATTACHMENTS =			1 <<  2,				//!> Rigid attachment was changed
				ePOSITIONS_VELOCITIES_MASS =	1 <<  3, 				//!> Positions, velocities or masses changed
				eREST_POSITION_TRANSFORM =		1 <<  4,				//!> Transform of the rest positions changed
				eLOD_SWITCH =					1 <<  5,				//!> Level of detail was changed, update lodX pos/vel from lod0
				eLOD_DATA =						1 <<  6 | eLOD_SWITCH,	//!> Definition of detail levels changed . Must come after setting any kind of rest positions. Triggers one-off initialization of levels
				eBENDING_REST_ANGLES =			1 <<  7 | eLOD_DATA,	//!> Bending rest angles were changed
				eTWISTING_REST_POSITIONS =		1 <<  8 | eLOD_DATA,	//!> Twisting rest positions were changed
				eREST_POSITIONS =				1 <<  9 | eLOD_DATA,	//!> Rest (=skinned) positions changed
				eSHAPE_MATCHING_SIZES =			1 << 10 | ePARAMETERS | eLOD_DATA,		//!> Shape matching group size or overlap changed. sets ePARAMETERS because settings are stored there
				eSTRAND_LENGTHS  = 				1 << 11 | eLOD_DATA | eSHAPE_MATCHING_SIZES | eBENDING_REST_ANGLES | eTWISTING_REST_POSITIONS, //!> Topology of vertex arrangement was changed
				eNUM_STRANDS_OR_VERTS =			1 << 12 | eSHAPE_MATCHING_SIZES
				| ePOSITIONS_VELOCITIES_MASS | eBENDING_REST_ANGLES | eTWISTING_REST_POSITIONS
				| eREST_POSITIONS | eLOD_DATA | eSTRAND_LENGTHS, //!> Number of strands or vertices changed
				eSOFTBODY_ATTACHMENTS =			1 << 13,				//!> Softbody attachments added or removed

				eALL = (1 << 14) - 1									//!> everything needs updating
			};
		};


		struct HairSystemSolverDesc
		{
			HairSystem*		hairSystem;
		};

		class HairSystem
		{
			PX_NOCOPY(HairSystem)
		public:
			HairSystem(Sc::HairSystemSim* sim, Dy::HairSystemCore& core) :
				mSim(sim)
				, mCore(core)
				, mShapeCore(NULL)
				, mElementId(0xffFFffFF)
				, mGpuRemapId(0xffFFffFF)

			{
				mCore.mDirtyFlags = HairSystemDirtyFlag::eALL;
			}

			~HairSystem() {}

			PX_FORCE_INLINE void setShapeCore(PxsShapeCore* shapeCore)
			{
				mShapeCore = shapeCore;
			}

			PX_FORCE_INLINE PxU32 getGpuRemapId() const { return mGpuRemapId; }
			PX_FORCE_INLINE void setGpuRemapId(PxU32 remapId)
			{
				mGpuRemapId = remapId;
				PxHairSystemGeometryLL& geom = mShapeCore->mGeometry.get<PxHairSystemGeometryLL>();
				geom.gpuRemapId = remapId;
			}

			PX_FORCE_INLINE PxU32 getElementId() const { return mElementId; }
			PX_FORCE_INLINE void setElementId(const PxU32 elementId) { mElementId = elementId; }


			PX_FORCE_INLINE Sc::HairSystemSim* getHairSystemSim() const { return mSim; }

			PX_FORCE_INLINE const HairSystemCore& getCore() const { return mCore; }
			PX_FORCE_INLINE HairSystemCore& getCore() { return mCore; }
			PX_FORCE_INLINE PxsShapeCore& getShapeCore() { return *mShapeCore; }

			PX_FORCE_INLINE PxU16 getIterationCounts() { return mCore.mSolverIterationCounts; }

		private: // variables
			Sc::HairSystemSim*	mSim;
			HairSystemCore&		mCore;
			PxsShapeCore*		mShapeCore;
			PxU32				mElementId; //this is used for the bound array
			PxU32				mGpuRemapId;
		};


		PX_FORCE_INLINE HairSystem* getHairSystem(HairSystemHandle handle)
		{
			return reinterpret_cast<HairSystem*>(handle);
		}

	}
}

#endif
