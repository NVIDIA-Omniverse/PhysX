// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_VERTEX_REDUCER_H
#define GU_VERTEX_REDUCER_H

#include "foundation/PxVec3.h"
#include "common/PxPhysXCommonConfig.h"

namespace physx
{
namespace Gu
{
	//! Vertex cloud reduction result structure
	struct REDUCEDCLOUD
	{
		// Out
		PxVec3*	RVerts;		//!< Reduced list
		PxU32	NbRVerts;	//!< Reduced number of vertices
		PxU32*	CrossRef;	//!< nb_verts remapped indices
	};

	class ReducedVertexCloud
	{
		public:
											ReducedVertexCloud(const PxVec3* verts, PxU32 nb_verts);
											~ReducedVertexCloud();

						ReducedVertexCloud&	clean();
						bool				reduce(REDUCEDCLOUD* rc=NULL);

		PX_FORCE_INLINE	PxU32				getNbVerts()				const	{ return mNbVerts;		}
		PX_FORCE_INLINE	PxU32				getNbReducedVerts()			const	{ return mNbRVerts;		}
		PX_FORCE_INLINE	const PxVec3*		getReducedVerts()			const	{ return mRVerts;		}
		PX_FORCE_INLINE	const PxVec3&		getReducedVertex(PxU32 i)	const	{ return mRVerts[i];	}
		PX_FORCE_INLINE	const PxU32*		getCrossRefTable()			const	{ return mXRef;			}

		private:
		// Original vertex cloud
						PxU32				mNbVerts;	//!< Number of vertices
						const PxVec3*		mVerts;		//!< List of vertices (pointer copy)

		// Reduced vertex cloud
						PxU32				mNbRVerts;	//!< Reduced number of vertices
						PxVec3*				mRVerts;	//!< Reduced list of vertices
						PxU32*				mXRef;		//!< Cross-reference table (used to remap topologies)
	};
}
}

#endif
	
