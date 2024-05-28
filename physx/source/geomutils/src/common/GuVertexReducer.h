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
	
