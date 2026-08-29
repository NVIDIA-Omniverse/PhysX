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
// Copyright (c) 2008-2026 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef BP_BOX_PRUNING_KERNELS_H
#define BP_BOX_PRUNING_KERNELS_H

// PT: see here for details: https://github.com/Pierre-Terdiman/BoxPruning/tree/master/Reports

#define	CODEALIGN16		//_asm	align 16

#if PX_INTEL_FAMILY
	#define SIMD_OVERLAP_TEST_14a(box)	_mm_movemask_ps(_mm_cmpngt_ps(b, _mm_load_ps(box)))==15

	#define SIMD_OVERLAP_INIT_9c(box)	\
			__m128 b = _mm_shuffle_ps(_mm_load_ps(&box.mMinY), _mm_load_ps(&box.mMinY), 78);\
			const float Coeff = -1.0f;\
			b = _mm_mul_ps(b, _mm_load1_ps(&Coeff));

	#define SIMD_OVERLAP_TEST_9c(box)					\
			const __m128 a = _mm_load_ps(&box.mMinY);	\
			const __m128 d = _mm_cmpge_ps(a, b);		\
			if(_mm_movemask_ps(d)==15)
#else
	#define SIMD_OVERLAP_TEST_14a(box)	BAllEqFFFF(V4IsGrtr(b, V4LoadA(box)))

	#define SIMD_OVERLAP_INIT_9c(box)				\
			Vec4V b = V4PermZWXY(V4LoadA(&box.mMinY));	\
			b = V4Mul(b, V4Load(-1.0f));

	#define SIMD_OVERLAP_TEST_9c(box)				\
			const Vec4V a = V4LoadA(&box.mMinY);	\
			const Vec4V d = V4IsGrtrOrEq(a, b);		\
			if(BAllEqTTTT(d))
#endif

#ifdef ABP_SIMD_OVERLAP
	#define SIMD_OVERLAP_PRELOAD_BOX0	SIMD_OVERLAP_INIT_9c(box0)
	#define SIMD_OVERLAP_TEST(x)		SIMD_OVERLAP_TEST_9c(x)
	#define ABP_OVERLAP_TEST(x)	SIMD_OVERLAP_TEST(x)
#else
	#define SIMD_OVERLAP_PRELOAD_BOX0
	#define ABP_OVERLAP_TEST(x)	if(intersect2D(box0, x))

	static PX_FORCE_INLINE int intersect2D(const SIMD_AABB_YZ4& a, const SIMD_AABB_YZ4& b)
	{
	/*	if(
			b.mMaxY < a.mMinY || a.mMaxY < b.mMinY
		||
			b.mMaxZ < a.mMinZ || a.mMaxZ < b.mMinZ
		)
			return 0;
		return 1;*/

		const bool b0 = b.mMaxY < a.mMinY;
		const bool b1 = a.mMaxY < b.mMinY;
		const bool b2 = b.mMaxZ < a.mMinZ;
		const bool b3 = a.mMaxZ < b.mMinZ;
	//	const bool b4 = b0 || b1 || b2 || b3;
		const bool b4 = b0 | b1 | b2 | b3;
		return !b4;
	}
#endif

template<const int codepath, class ABP_PairManagerT>
static void boxPruningKernel(	PxU32 nb0, PxU32 nb1,
								const SIMD_AABB_X4* PX_RESTRICT boxes0_X, const SIMD_AABB_X4* PX_RESTRICT boxes1_X,
								const SIMD_AABB_YZ4* PX_RESTRICT boxes0_YZ, const SIMD_AABB_YZ4* PX_RESTRICT boxes1_YZ,
								const PxU32* PX_RESTRICT inToOut0, const PxU32* PX_RESTRICT inToOut1,
								ABP_PairManagerT* PX_RESTRICT pairManager)
{
	pairManager->mInToOut0 = inToOut0;
	pairManager->mInToOut1 = inToOut1;

	PxU32 index0 = 0;
	PxU32 runningIndex1 = 0;

	while(runningIndex1<nb1 && index0<nb0)
	{
		const SIMD_AABB_X4& box0_X = boxes0_X[index0];
		const PosXType2 maxLimit = box0_X.mMaxX;

		const PosXType2 minLimit = box0_X.mMinX;
		if(!codepath)
		{
			while(boxes1_X[runningIndex1].mMinX<minLimit)
				runningIndex1++;
		}
		else
		{
			while(boxes1_X[runningIndex1].mMinX<=minLimit)
				runningIndex1++;
		}

		const SIMD_AABB_YZ4& box0 = boxes0_YZ[index0];
		SIMD_OVERLAP_PRELOAD_BOX0

		if(gUseRegularBPKernel)
		{
			PxU32 index1 = runningIndex1;

			while(boxes1_X[index1].mMinX<=maxLimit)
			{
				ABP_OVERLAP_TEST(boxes1_YZ[index1])
				{
					outputPair(*pairManager, index0, index1);
				}
				index1++;
			}
		}
		else
		{
			PxU32 Offset = 0;
			const char* const CurrentBoxListYZ = reinterpret_cast<const char*>(&boxes1_YZ[runningIndex1]);
			const char* const CurrentBoxListX = reinterpret_cast<const char*>(&boxes1_X[runningIndex1]);

			if(!gUnrollLoop)
			{
				while(*reinterpret_cast<const PosXType2*>(CurrentBoxListX + Offset)<=maxLimit)
				{
					const float* box = reinterpret_cast<const float*>(CurrentBoxListYZ + Offset*2);
#ifdef ABP_SIMD_OVERLAP
					if(SIMD_OVERLAP_TEST_14a(box))
#else
					if(intersect2D(box0, *reinterpret_cast<const SIMD_AABB_YZ4*>(box)))
#endif
					{
						const PxU32 Index1 = PxU32(CurrentBoxListX + Offset - reinterpret_cast<const char*>(boxes1_X))>>3;
						outputPair(*pairManager, index0, Index1);
					}
					Offset += 8;
				}
			}
			else
			{
#define BIP_VERSION4
#ifdef BIP_VERSION4
#ifdef ABP_SIMD_OVERLAP
	#define BLOCK4(x, label)	{const float* box = reinterpret_cast<const float*>(CurrentBoxListYZ + Offset*2 + x*2);	\
								if(SIMD_OVERLAP_TEST_14a(box))															\
								goto label;	}
#else
	#define BLOCK4(x, label)	{const float* box = reinterpret_cast<const float*>(CurrentBoxListYZ + Offset*2 + x*2);	\
								if(intersect2D(box0, *reinterpret_cast<const SIMD_AABB_YZ4*>(box)))						\
								goto label;	}
#endif
		goto StartLoop4;
		CODEALIGN16
FoundOverlap3:
		Offset += 8;
		CODEALIGN16
FoundOverlap2:
		Offset += 8;
		CODEALIGN16
FoundOverlap1:
		Offset += 8;
		CODEALIGN16
FoundOverlap0:
		Offset += 8;
		CODEALIGN16
FoundOverlap:
		{
			const PxU32 Index1 = PxU32(CurrentBoxListX + Offset - 8 - reinterpret_cast<const char*>(boxes1_X))>>3;
			outputPair(*pairManager, index0, Index1);
		}
		CODEALIGN16
StartLoop4:
		while(*reinterpret_cast<const PosXType2*>(CurrentBoxListX + Offset + 8*5)<=maxLimit)
		{
			BLOCK4(0, FoundOverlap0)
			BLOCK4(8, FoundOverlap1)
			BLOCK4(16, FoundOverlap2)
			BLOCK4(24, FoundOverlap3)
			Offset += 40;
			BLOCK4(-8, FoundOverlap)
		}
#undef BLOCK4
#endif

#ifdef ABP_SIMD_OVERLAP
	#define BLOCK	if(*reinterpret_cast<const PosXType2*>(CurrentBoxListX + Offset)<=maxLimit)				\
					{if(SIMD_OVERLAP_TEST_14a(reinterpret_cast<const float*>(CurrentBoxListYZ + Offset*2)))	\
						goto OverlapFound;																	\
					Offset += 8;
#else
	#define BLOCK	if(*reinterpret_cast<const PosXType2*>(CurrentBoxListX + Offset)<=maxLimit)						\
					{if(intersect2D(box0, *reinterpret_cast<const SIMD_AABB_YZ4*>(CurrentBoxListYZ + Offset*2)))	\
						goto OverlapFound;																			\
					Offset += 8;
#endif

		goto LoopStart;
		CODEALIGN16
OverlapFound:
		{
			const PxU32 Index1 = PxU32(CurrentBoxListX + Offset - reinterpret_cast<const char*>(boxes1_X))>>3;
			outputPair(*pairManager, index0, Index1);
		}
		Offset += 8;
		CODEALIGN16
LoopStart:
		BLOCK
			BLOCK
				BLOCK
				}
			}
			goto LoopStart;
		}
#undef BLOCK
			}
		}

		index0++;
	}
}

template<class ABP_PairManagerT>
static void doCompleteBoxPruning_Leaf(	ABP_PairManagerT* PX_RESTRICT pairManager, PxU32 nb,
										const SIMD_AABB_X4* PX_RESTRICT boxes_X,
										const SIMD_AABB_YZ4* PX_RESTRICT boxes_YZ,
										const PxU32* PX_RESTRICT remap)
{
	pairManager->mInToOut0 = remap;
	pairManager->mInToOut1 = remap;

	PxU32 index0 = 0;
	PxU32 runningIndex = 0;
	while(runningIndex<nb && index0<nb)
	{
		const SIMD_AABB_X4& box0_X = boxes_X[index0];
		const PosXType2 maxLimit = box0_X.mMaxX;

		const PosXType2 minLimit = box0_X.mMinX;
		while(boxes_X[runningIndex++].mMinX<minLimit);

		const SIMD_AABB_YZ4& box0 = boxes_YZ[index0];
		SIMD_OVERLAP_PRELOAD_BOX0

		if(gUseRegularBPKernel)
		{
			PxU32 index1 = runningIndex;
			while(boxes_X[index1].mMinX<=maxLimit)
			{
				ABP_OVERLAP_TEST(boxes_YZ[index1])
				{
					outputPair(*pairManager, index0, index1);
				}
				index1++;
			}
		}
		else
		{
			PxU32 Offset = 0;
			const char* const CurrentBoxListYZ = reinterpret_cast<const char*>(&boxes_YZ[runningIndex]);
			const char* const CurrentBoxListX = reinterpret_cast<const char*>(&boxes_X[runningIndex]);

			if(!gUnrollLoop)
			{
				while(*reinterpret_cast<const PosXType2*>(CurrentBoxListX + Offset)<=maxLimit)
				{
					const float* box = reinterpret_cast<const float*>(CurrentBoxListYZ + Offset*2);
#ifdef ABP_SIMD_OVERLAP
					if(SIMD_OVERLAP_TEST_14a(box))
#else
					if(intersect2D(box0, *reinterpret_cast<const SIMD_AABB_YZ4*>(box)))
#endif
					{
						const PxU32 Index = PxU32(CurrentBoxListX + Offset - reinterpret_cast<const char*>(boxes_X))>>3;
						outputPair(*pairManager, index0, Index);
					}
					Offset += 8;
				}
			}
			else
			{
#define VERSION4c
#ifdef VERSION4c
#define VERSION3	// Enable this as our safe loop
#ifdef ABP_SIMD_OVERLAP
	#define BLOCK4(x, label)	{const float* box = reinterpret_cast<const float*>(CurrentBoxListYZ + Offset*2 + x*2);	\
								if(SIMD_OVERLAP_TEST_14a(box))															\
									goto label;	}
#else
	#define BLOCK4(x, label)	{const SIMD_AABB_YZ4* box = reinterpret_cast<const SIMD_AABB_YZ4*>(CurrentBoxListYZ + Offset*2 + x*2);	\
								if(intersect2D(box0, *box))																				\
									goto label;	}
#endif
		goto StartLoop4;
		CODEALIGN16
FoundOverlap3:
		Offset += 8;
		CODEALIGN16
FoundOverlap2:
		Offset += 8;
		CODEALIGN16
FoundOverlap1:
		Offset += 8;
		CODEALIGN16
FoundOverlap0:
		Offset += 8;
		CODEALIGN16
FoundOverlap:
		{
			const PxU32 Index = PxU32(CurrentBoxListX + Offset - 8 - reinterpret_cast<const char*>(boxes_X))>>3;
			outputPair(*pairManager, index0, Index);
		}
		CODEALIGN16
StartLoop4:
		while(*reinterpret_cast<const PosXType2*>(CurrentBoxListX + Offset + 8*5)<=maxLimit)
		{
			BLOCK4(0, FoundOverlap0)
			BLOCK4(8, FoundOverlap1)
			BLOCK4(16, FoundOverlap2)
			BLOCK4(24, FoundOverlap3)
			Offset += 40;
			BLOCK4(-8, FoundOverlap)
		}
#endif

#define VERSION3
#ifdef VERSION3
#ifdef ABP_SIMD_OVERLAP
	#define BLOCK	if(*reinterpret_cast<const PosXType2*>(CurrentBoxListX + Offset)<=maxLimit)				\
					{if(SIMD_OVERLAP_TEST_14a(reinterpret_cast<const float*>(CurrentBoxListYZ + Offset*2)))	\
						goto BeforeLoop;																	\
					Offset += 8;
#else
	#define BLOCK	if(*reinterpret_cast<const PosXType2*>(CurrentBoxListX + Offset)<=maxLimit)						\
					{if(intersect2D(box0, *reinterpret_cast<const SIMD_AABB_YZ4*>(CurrentBoxListYZ + Offset*2)))	\
						goto BeforeLoop;																			\
					Offset += 8;
#endif

		goto StartLoop;
		CODEALIGN16
BeforeLoop:
		{
			const PxU32 Index = PxU32(CurrentBoxListX + Offset - reinterpret_cast<const char*>(boxes_X))>>3;
			outputPair(*pairManager, index0, Index);
			Offset += 8;
		}
		CODEALIGN16
StartLoop:
		BLOCK
			BLOCK
				BLOCK
					BLOCK
						BLOCK
						}
					}
				}
			}
			goto StartLoop;
		}
#endif
			}
		}

		index0++;
	}
}

#endif
