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

#ifndef GU_BV4_BOX_BOX_OVERLAP_TEST_H
#define GU_BV4_BOX_BOX_OVERLAP_TEST_H

#ifndef GU_BV4_USE_SLABS		
	PX_FORCE_INLINE PxIntBool BV4_BoxBoxOverlap(const PxVec3& extents, const PxVec3& center, const OBBTestParams* PX_RESTRICT params)
	{
		const Vec4V extentsV = V4LoadU(&extents.x);

		const Vec4V TV = V4Sub(V4LoadA_Safe(&params->mTBoxToModel_PaddedAligned.x), V4LoadU(&center.x));
		{
			const Vec4V absTV = V4Abs(TV);
			const BoolV resTV = V4IsGrtr(absTV, V4Add(extentsV, V4LoadA_Safe(&params->mBB_PaddedAligned.x)));
			const PxU32 test = BGetBitMask(resTV);
			if(test&7)
				return 0;
		}

		Vec4V tV;
		{
			const Vec4V T_YZX_V = V4Perm<1, 2, 0, 3>(TV);
			const Vec4V T_ZXY_V = V4Perm<2, 0, 1, 3>(TV);

			tV = V4Mul(TV, V4LoadA_Safe(&params->mPreca0_PaddedAligned.x));
			tV = V4Add(tV, V4Mul(T_YZX_V, V4LoadA_Safe(&params->mPreca1_PaddedAligned.x)));
			tV = V4Add(tV, V4Mul(T_ZXY_V, V4LoadA_Safe(&params->mPreca2_PaddedAligned.x)));
		}

		Vec4V t2V;
		{
			const Vec4V extents_YZX_V = V4Perm<1, 2, 0, 3>(extentsV);
			const Vec4V extents_ZXY_V = V4Perm<2, 0, 1, 3>(extentsV);

			t2V = V4Mul(extentsV, V4LoadA_Safe(&params->mPreca0b_PaddedAligned.x));
			t2V = V4Add(t2V, V4Mul(extents_YZX_V, V4LoadA_Safe(&params->mPreca1b_PaddedAligned.x)));
			t2V = V4Add(t2V, V4Mul(extents_ZXY_V, V4LoadA_Safe(&params->mPreca2b_PaddedAligned.x)));
			t2V = V4Add(t2V, V4LoadA_Safe(&params->mBoxExtents_PaddedAligned.x));
		}

		{
			const Vec4V abstV = V4Abs(tV);
			const BoolV resB = V4IsGrtr(abstV, t2V);
			const PxU32 test = BGetBitMask(resB);
			if(test&7)
				return 0;
		}
		return 1;
	}

#ifdef GU_BV4_QUANTIZED_TREE	
	template<class T>
	PX_FORCE_INLINE PxIntBool BV4_BoxBoxOverlap(const T* PX_RESTRICT node, const OBBTestParams* PX_RESTRICT params)
	{ 
// A.B. enable new version only for intel non simd path
#if PX_INTEL_FAMILY && !defined(PX_SIMD_DISABLED)
//	#define NEW_VERSION
#endif
#ifdef NEW_VERSION
	SSE_CONST4(maskV,	0x7fffffff);
	SSE_CONST4(maskQV,	0x0000ffff);
#endif

#ifdef NEW_VERSION
		Vec4V centerV = V4LoadA((float*)node->mAABB.mData);
		__m128 extentsV = _mm_castsi128_ps(_mm_and_si128(_mm_castps_si128(centerV), SSE_CONST(maskQV)));
		extentsV = V4Mul(_mm_cvtepi32_ps(_mm_castps_si128(extentsV)), V4LoadA_Safe(&params->mExtentsOrMaxCoeff_PaddedAligned.x));
		centerV = _mm_castsi128_ps(_mm_srai_epi32(_mm_castps_si128(centerV), 16));
		centerV = V4Mul(_mm_cvtepi32_ps(_mm_castps_si128(centerV)), V4LoadA_Safe(&params->mCenterOrMinCoeff_PaddedAligned.x));
#else
		const VecI32V centerVI = I4LoadA((PxI32*)node->mAABB.mData);		
		const VecI32V extentsVI = VecI32V_And(centerVI, I4Load(0x0000ffff));
		const Vec4V extentsV = V4Mul(Vec4V_From_VecI32V(extentsVI), V4LoadA_Safe(&params->mExtentsOrMaxCoeff_PaddedAligned.x));		
		const VecI32V centerVShift = VecI32V_RightShift(centerVI, 16);		
		const Vec4V centerV = V4Mul(Vec4V_From_VecI32V(centerVShift), V4LoadA_Safe(&params->mCenterOrMinCoeff_PaddedAligned.x));
#endif

		const Vec4V TV = V4Sub(V4LoadA_Safe(&params->mTBoxToModel_PaddedAligned.x), centerV);
		{
#ifdef NEW_VERSION
			const __m128 absTV = _mm_and_ps(TV, SSE_CONSTF(maskV));
#else			
			const Vec4V absTV = V4Abs(TV);
#endif
			const BoolV resTV = V4IsGrtr(absTV, V4Add(extentsV, V4LoadA_Safe(&params->mBB_PaddedAligned.x)));
			const PxU32 test = BGetBitMask(resTV);
			if(test&7)
				return 0;
		}

		Vec4V tV;
		{
			const Vec4V T_YZX_V = V4Perm<1, 2, 0, 3>(TV);
			const Vec4V T_ZXY_V = V4Perm<2, 0, 1, 3>(TV);

			tV = V4Mul(TV, V4LoadA_Safe(&params->mPreca0_PaddedAligned.x));
			tV = V4Add(tV, V4Mul(T_YZX_V, V4LoadA_Safe(&params->mPreca1_PaddedAligned.x)));
			tV = V4Add(tV, V4Mul(T_ZXY_V, V4LoadA_Safe(&params->mPreca2_PaddedAligned.x)));
		}

		Vec4V t2V;
		{
			const Vec4V extents_YZX_V = V4Perm<1, 2, 0, 3>(extentsV);
			const Vec4V extents_ZXY_V = V4Perm<2, 0, 1, 3>(extentsV);

			t2V = V4Mul(extentsV, V4LoadA_Safe(&params->mPreca0b_PaddedAligned.x));
			t2V = V4Add(t2V, V4Mul(extents_YZX_V, V4LoadA_Safe(&params->mPreca1b_PaddedAligned.x)));
			t2V = V4Add(t2V, V4Mul(extents_ZXY_V, V4LoadA_Safe(&params->mPreca2b_PaddedAligned.x)));
			t2V = V4Add(t2V, V4LoadA_Safe(&params->mBoxExtents_PaddedAligned.x));
		}

		{
#ifdef NEW_VERSION
			const __m128 abstV = _mm_and_ps(tV, SSE_CONSTF(maskV));
#else
			const Vec4V abstV = V4Abs(tV);
#endif
			const BoolV resB = V4IsGrtr(abstV, t2V);
			const PxU32 test = BGetBitMask(resB);
			if(test&7)
				return 0;
		}
		return 1;
	}
#endif	// GU_BV4_QUANTIZED_TREE
#endif	// GU_BV4_USE_SLABS

#ifdef GU_BV4_USE_SLABS	
	PX_FORCE_INLINE PxIntBool BV4_BoxBoxOverlap(const Vec4V boxCenter, const Vec4V extentsV, const OBBTestParams* PX_RESTRICT params)
	{
		const Vec4V TV = V4Sub(V4LoadA_Safe(&params->mTBoxToModel_PaddedAligned.x), boxCenter);
		{
			const Vec4V absTV = V4Abs(TV);
			const BoolV res = V4IsGrtr(absTV, V4Add(extentsV, V4LoadA_Safe(&params->mBB_PaddedAligned.x)));
			const PxU32 test = BGetBitMask(res);
			if(test&7)
				return 0;
		}

		Vec4V tV;
		{			
			const Vec4V T_YZX_V = V4Perm<1, 2, 0, 3>(TV);
			const Vec4V T_ZXY_V = V4Perm<2, 0, 1, 3>(TV);

			tV = V4Mul(TV, V4LoadA_Safe(&params->mPreca0_PaddedAligned.x));
			tV = V4Add(tV, V4Mul(T_YZX_V, V4LoadA_Safe(&params->mPreca1_PaddedAligned.x)));
			tV = V4Add(tV, V4Mul(T_ZXY_V, V4LoadA_Safe(&params->mPreca2_PaddedAligned.x)));
		}

		Vec4V t2V;
		{
			const Vec4V extents_YZX_V = V4Perm<1, 2, 0, 3>(extentsV);
			const Vec4V extents_ZXY_V = V4Perm<2, 0, 1, 3>(extentsV);

			t2V = V4Mul(extentsV, V4LoadA_Safe(&params->mPreca0b_PaddedAligned.x));
			t2V = V4Add(t2V, V4Mul(extents_YZX_V, V4LoadA_Safe(&params->mPreca1b_PaddedAligned.x)));
			t2V = V4Add(t2V, V4Mul(extents_ZXY_V, V4LoadA_Safe(&params->mPreca2b_PaddedAligned.x)));
			t2V = V4Add(t2V, V4LoadA_Safe(&params->mBoxExtents_PaddedAligned.x));
		}

		{
			const Vec4V abstV = V4Abs(tV);
			const BoolV resB = V4IsGrtr(abstV, t2V);
			const PxU32 test = BGetBitMask(resB);
			if(test&7)
				return 0;
		}
		return 1;
	}
#endif	// GU_BV4_USE_SLABS

#endif	// GU_BV4_BOX_BOX_OVERLAP_TEST_H
