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

#ifndef NP_PRUNING_STRUCTURE_H
#define NP_PRUNING_STRUCTURE_H

#include "PxPruningStructure.h"

#include "foundation/PxUserAllocated.h"
#include "GuPrunerMergeData.h"

namespace physx
{
	namespace Sq
	{
		class PruningStructure : public PxPruningStructure, public PxUserAllocated
		{
			PX_NOCOPY(PruningStructure)
		public:
			// PX_SERIALIZATION            
													PruningStructure(PxBaseFlags baseFlags);			
			virtual			void					resolveReferences(PxDeserializationContext& );
			static			PruningStructure*		createObject(PxU8*& address, PxDeserializationContext& context);
			static			void					getBinaryMetaData(PxOutputStream& stream);
							void					preExportDataReset() {}
							void					exportExtraData(PxSerializationContext&);
							void					importExtraData(PxDeserializationContext&);
			virtual			void					requiresObjects(PxProcessPxBaseCallback&);
			//~PX_SERIALIZATION

			// PxPruningStructure
			virtual			void					release();
			virtual			PxU32					getRigidActors(PxRigidActor** userBuffer, PxU32 bufferSize, PxU32 startIndex=0) const;
			virtual			PxU32					getNbRigidActors()			const;
			virtual			const void*				getStaticMergeData()		const;
			virtual			const void*				getDynamicMergeData()		const;
			// ~PxPruningStructure
													PruningStructure();
			virtual									~PruningStructure();

							bool					build(PxRigidActor*const* actors, PxU32 nbActors);			

			PX_FORCE_INLINE	PxU32					getNbActors()				const	{ return mNbActors;	}
			PX_FORCE_INLINE	PxActor*const*			getActors()					const	{ return mActors;	}

			PX_FORCE_INLINE	bool					isValid()					const	{ return mValid;	}
							void					invalidate(PxActor* actor);
		private:
							Gu::AABBPrunerMergeData	mData[2];
							PxU32					mNbActors;	// Nb actors from which the pruner structure was build
							PxActor**				mActors;	// actors used for pruner structure build, used later for serialization
							bool					mValid;		// pruning structure validity
		};
	}

}

#endif
