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

#ifndef GU_BV4_AABB_AABB_SWEEP_TEST_H
#define GU_BV4_AABB_AABB_SWEEP_TEST_H

#ifndef GU_BV4_USE_SLABS
	PX_FORCE_INLINE PxIntBool BV4_SegmentAABBOverlap(const PxVec3& center, const PxVec3& extents, const PxVec3& extents2, const RayParams* PX_RESTRICT params)
	{
		const Vec4V fdirV = V4LoadA_Safe(&params->mFDir_PaddedAligned.x);
		const Vec4V extentsV = V4Add(V4LoadU(&extents.x), V4LoadU(&extents2.x));
		const Vec4V DV = V4Sub(V4LoadA_Safe(&params->mData2_PaddedAligned.x), V4LoadU(&center.x));
		const Vec4V absDV = V4Abs(DV);
		const BoolV resDV = V4IsGrtr(absDV, V4Add(extentsV, fdirV));
		const PxU32 test = BGetBitMask(resDV);
		if(test&7)
			return 0;

		if(1)
		{
			const Vec4V dataZYX_V = V4LoadA_Safe(&params->mData_PaddedAligned.x);			
			const Vec4V dataXZY_V = V4Perm<1, 2, 0, 3>(dataZYX_V);			
			const Vec4V DXZY_V = V4Perm<1, 2, 0, 3>(DV);
			const Vec4V fV = V4Sub(V4Mul(dataZYX_V, DXZY_V), V4Mul(dataXZY_V, DV));

			const Vec4V fdirZYX_V = V4LoadA_Safe(&params->mFDir_PaddedAligned.x);			
			const Vec4V fdirXZY_V = V4Perm<1, 2, 0, 3>(fdirZYX_V);			
			const Vec4V extentsXZY_V = V4Perm<1, 2, 0, 3>(extentsV);
			// PT: TODO: use V4MulAdd here (TA34704)
			const Vec4V fg = V4Add(V4Mul(extentsV, fdirXZY_V), V4Mul(extentsXZY_V, fdirZYX_V));

			const Vec4V absfV = V4Abs(fV);
			const BoolV resAbsfV = V4IsGrtr(absfV, fg);
			const PxU32 test2 = BGetBitMask(resAbsfV);
			if(test2&7)
				return 0;
			return 1;
		}
	}

#ifdef GU_BV4_QUANTIZED_TREE
	template<class T>
	PX_FORCE_INLINE PxIntBool BV4_SegmentAABBOverlap(const T* PX_RESTRICT node, const PxVec3& extents2, const RayParams* PX_RESTRICT params)
	{
		const VecI32V testV = I4LoadA((PxI32*)node->mAABB.mData);
		const VecI32V qextentsV = VecI32V_And(testV, I4Load(0x0000ffff));
		const VecI32V qcenterV = VecI32V_RightShift(testV, 16);
		const Vec4V centerV0 = V4Mul(Vec4V_From_VecI32V(qcenterV), V4LoadA_Safe(&params->mCenterOrMinCoeff_PaddedAligned.x));
		const Vec4V extentsV0 = V4Mul(Vec4V_From_VecI32V(qextentsV), V4LoadA_Safe(&params->mExtentsOrMaxCoeff_PaddedAligned.x));

		const Vec4V fdirV = V4LoadA_Safe(&params->mFDir_PaddedAligned.x);
		const Vec4V extentsV = V4Add(extentsV0, V4LoadU(&extents2.x));
		const Vec4V DV = V4Sub(V4LoadA_Safe(&params->mData2_PaddedAligned.x), centerV0);
		const Vec4V absDV = V4Abs(DV);
		const BoolV res = V4IsGrtr(absDV, V4Add(extentsV, fdirV));
		const PxU32 test = BGetBitMask(res);
		if(test&7)
			return 0;

		if(1)
		{
			const Vec4V dataZYX_V = V4LoadA_Safe(&params->mData_PaddedAligned.x);
			const Vec4V dataXZY_V = V4Perm<1, 2, 0, 3>(dataZYX_V);			
			const Vec4V DXZY_V = V4Perm<1, 2, 0, 3>(DV);

			const Vec4V fV = V4Sub(V4Mul(dataZYX_V, DXZY_V), V4Mul(dataXZY_V, DV));

			const Vec4V fdirZYX_V = V4LoadA_Safe(&params->mFDir_PaddedAligned.x);
			const Vec4V fdirXZY_V = V4Perm<1, 2, 0, 3>(fdirZYX_V);			
			const Vec4V extentsXZY_V = V4Perm<1, 2, 0, 3>(extentsV);
			// PT: TODO: use V4MulAdd here (TA34704)
			const Vec4V fg = V4Add(V4Mul(extentsV, fdirXZY_V), V4Mul(extentsXZY_V, fdirZYX_V));

			const Vec4V absfV = V4Abs(fV);
			const BoolV res2 = V4IsGrtr(absfV, fg);
			const PxU32 test2 = BGetBitMask(res2);
			if(test2&7)
				return 0;
			return 1;
		}
	}
#endif
#endif

#endif // GU_BV4_AABB_AABB_SWEEP_TEST_H
