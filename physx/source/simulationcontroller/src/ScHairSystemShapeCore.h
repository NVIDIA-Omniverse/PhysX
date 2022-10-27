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
// Copyright (c) 2008-2022 NVIDIA Corporation. All rights reserved.

#ifndef SC_HAIRSYSTEM_SHAPECORE_H
#define SC_HAIRSYSTEM_SHAPECORE_H

#include "foundation/PxUserAllocated.h"
#include "DyHairSystemCore.h"
#include "ScShapeCore.h"
#include "PxShape.h"

namespace physx
{
	class PxCudaContextManager;

	namespace Sc
	{

		class HairSystemShapeCore : public Sc::ShapeCore
		{
			//= ATTENTION! =====================================================================================
			// Changing the data layout of this class breaks the binary serialization format.  See comments for 
			// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
			// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
			// accordingly.
			//==================================================================================================
		public:
			// PX_SERIALIZATION
			HairSystemShapeCore(const PxEMPTY);
			//~PX_SERIALIZATION

			HairSystemShapeCore();
			~HairSystemShapeCore();

			PX_FORCE_INLINE const	Dy::HairSystemCore&	getLLCore() const	{ return mLLCore; }
			PX_FORCE_INLINE			Dy::HairSystemCore&	getLLCore()			{ return mLLCore; }

			void createBuffers(PxCudaContextManager* cudaContextManager);
			void releaseBuffers();

			PxU64& getGpuMemStat() { return mGpuMemStat; }

		private:
			Dy::HairSystemCore		mLLCore;
			PxU64					mGpuMemStat;
			PxCudaContextManager*	mCudaContextManager;
		};

	} // namespace Sc
} // namespace physx

#endif
