// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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
							void					preExportDataReset() {}
							void					exportExtraData(PxSerializationContext&);
							void					importExtraData(PxDeserializationContext&);
			virtual			void					requiresObjects(PxProcessPxBaseCallback&);
			//~PX_SERIALIZATION

			// PxPruningStructure
			virtual			void					release() PX_OVERRIDE;
			virtual			PxU32					getRigidActors(PxRigidActor** userBuffer, PxU32 bufferSize, PxU32 startIndex=0) const PX_OVERRIDE;
			virtual			PxU32					getNbRigidActors()			const PX_OVERRIDE;
			virtual			const void*				getStaticMergeData()		const PX_OVERRIDE;
			virtual			const void*				getDynamicMergeData()		const PX_OVERRIDE;
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
