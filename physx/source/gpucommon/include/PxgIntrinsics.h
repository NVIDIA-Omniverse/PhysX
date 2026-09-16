// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_INTRINSICS_H
#define PXG_INTRINSICS_H

#include "foundation/PxSimpleTypes.h"
#include "cuda.h"

#if (defined(_MSC_VER) && defined(_WIN64)) || defined(__LP64__) || defined(__CUDACC_RTC__)
	#define __STG_PTR   "l"
#else
	#define __STG_PTR   "r"
#endif

namespace physx
{
#if __CUDA_ARCH__ >= 350
	template <typename T>
	PX_FORCE_INLINE PX_CUDA_CALLABLE T Pxldca(const T& t) { return __ldca(&t); } //Cache all levels
	template <typename T>
	PX_FORCE_INLINE PX_CUDA_CALLABLE T Pxldcg(const T& t) { return __ldcg(&t); } //Cache at global level (not L1)
	template <typename T>
	PX_FORCE_INLINE PX_CUDA_CALLABLE T Pxldg(const T& t) { return __ldg(&t); } //Load global
	template <typename T>
	PX_FORCE_INLINE PX_CUDA_CALLABLE T Pxldcs(const T& t) { return __ldcs(&t); } //Cache streaming, likely to touch once
	template <typename T>
	PX_FORCE_INLINE PX_CUDA_CALLABLE T Pxldlu(const T& t) { return __ldlu(&t); } // Last use
	template <typename T>
	PX_FORCE_INLINE PX_CUDA_CALLABLE T Pxldcv(const T& t) { return __ldcv(&t); } //Don't catch and fetch again
	
#if __CUDACC_VER_MAJOR__ >= 11
	template <typename T>
	PX_FORCE_INLINE PX_CUDA_CALLABLE void Pxstwb(T* dst, const T& src) { __stwb(dst, src); } //Cache write-back all levels
	template <typename T>
	PX_FORCE_INLINE PX_CUDA_CALLABLE void Pxstcg(T* dst, const T& src) { __stcg(dst, src); } //Cache at global (not L1)
	template <typename T>
	PX_FORCE_INLINE PX_CUDA_CALLABLE void Pxstcs(T* dst, const T& src) { __stcs(dst, src); } // Cache streaming (accessed once)
	template <typename T>
	PX_FORCE_INLINE PX_CUDA_CALLABLE void Pxstwt(T* dst, const T& src) { __stwt(dst, src); } // Cache write through (no caching)
#else
	template <typename T>
	PX_FORCE_INLINE PX_CUDA_CALLABLE void Pxstwb(T* dst, const T& src) { *dst = src; } //Cache write-back all levels
	template <typename T>
	PX_FORCE_INLINE PX_CUDA_CALLABLE void Pxstcg(T* dst, const T& src) { *dst = src; } //Cache at global (not L1)
	template <typename T>
	PX_FORCE_INLINE PX_CUDA_CALLABLE void Pxstcs(T* dst, const T& src) { *dst = src; } // Cache streaming (accessed once)
	template <typename T>
	PX_FORCE_INLINE PX_CUDA_CALLABLE void Pxstwt(T* dst, const T& src) { *dst = src; } // Cache write through (no caching)
#endif
#else
	template <typename T>
	PX_FORCE_INLINE PX_CUDA_CALLABLE T Pxldca(const T& t) { return t; } //Cache all levels
	template <typename T>
	PX_FORCE_INLINE PX_CUDA_CALLABLE T Pxldcg(const T& t) { return t; } //Cache at global level (not L1)
	template <typename T>
	PX_FORCE_INLINE PX_CUDA_CALLABLE T Pxldg(const T& t) { return t; } //Load global
	template <typename T>
	PX_FORCE_INLINE PX_CUDA_CALLABLE T Pxldcs(const T& t) { return t; } //Cache streaming, likely to touch once
	template <typename T>
	PX_FORCE_INLINE PX_CUDA_CALLABLE T Pxldlu(const T& t) { return t; } // Last use
	template <typename T>
	PX_FORCE_INLINE PX_CUDA_CALLABLE T Pxldcv(const T& t) { return t; } //Don't catch and fetch again

	template <typename T>
	PX_FORCE_INLINE PX_CUDA_CALLABLE void Pxstwb(T* dst, const T& src) { *dst = src; } //Cache write-back all levels
	template <typename T>
	PX_FORCE_INLINE PX_CUDA_CALLABLE void Pxstcg(T* dst, const T& src) { *dst = src; } //Cache at global (not L1)
	template <typename T>
	PX_FORCE_INLINE PX_CUDA_CALLABLE void Pxstcs(T* dst, const T& src) { *dst = src; } // Cache streaming (accessed once)
	template <typename T>
	PX_FORCE_INLINE PX_CUDA_CALLABLE void Pxstwt(T* dst, const T& src) { *dst = src; } // Cache write through (no caching)

#endif
}

#endif


