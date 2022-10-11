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
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef PX_KERNEL_MANAGER_H
#define PX_KERNEL_MANAGER_H
/** \addtogroup extensions
  @{
*/


#include "cudamanager/PxCudaContext.h"
#include "cudamanager/PxCudaContextManager.h"

#include "foundation/PxUserAllocated.h"
#include "foundation/PxArray.h"
#include "cudamanager/PxCudaTypes.h"

#include "foundation/PxString.h"



#define ENABLE_KERNEL_LAUNCH_ERROR_CHECK 0


#if !PX_DOXYGEN
namespace physx
{
#endif

	namespace ExtGpu
	{
		const PxU32 CUDA_SUCCESS = 0;

		class PxKernelWrangler : public PxUserAllocated
		{
			PX_NOCOPY(PxKernelWrangler)
		public:
			PxKernelWrangler(PxCudaContextManager& cudaContextManager, PxErrorCallback& errorCallback, const char** funcNames, uint16_t numFuncs);
			~PxKernelWrangler();

			PX_FORCE_INLINE	CUfunction getCuFunction(uint16_t funcIndex) const
			{
				return mCuFunctions[funcIndex];
			}

			const char* getCuFunctionName(uint16_t funcIndex) const;

			static void const* const* getImages();
			static int getNumImages();

			PX_FORCE_INLINE	bool hadError() const { return mError; }

		protected:
			bool					mError;
			PxArray<CUfunction>		mCuFunctions;
			PxArray<CUmodule>	    mCuModules;
			PxCudaContextManager&	mCudaContextManager;
			PxCudaContext*			mCudaContext;
			PxErrorCallback&		mErrorCallback;
		};

		 


		// PT: do not inline this
		static PX_NOINLINE void outputKernelLaunchDebugInfo(PxKernelWrangler* kernelWrangler, PxU16 id, const char* file, PxU32 line)
		{
			char errorMsg[4096];
			physx::Pxsnprintf(errorMsg, 4096, "Launching GPU kernel %s...\n", kernelWrangler->getCuFunctionName(id));
			PxGetFoundation().error(PxErrorCode::eDEBUG_INFO, file, line, errorMsg);
		}

		// PT: do not inline this
		static PX_NOINLINE void outputKernelLaunchError(PxKernelWrangler* kernelWrangler, PxU16 id, const char* file, PxU32 line)
		{
			char errorMsg[4096];
			physx::Pxsnprintf(errorMsg, 4096, "GPU kernel '%s' failed to launch!!\n", kernelWrangler->getCuFunctionName(id));
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, file, line, errorMsg);
		}

		template<const bool debugExecution>
		static PX_FORCE_INLINE physx::PxCUresult _testLaunch(physx::PxCUresult resultR, PxKernelWrangler* kernelWrangler, PxCudaContext* ctx, PxU16 id, CUstream hStream, const char* file, PxU32 line)
		{
			// PT: don't assert immediately, we want to print the failing kernel's name first
			if (resultR != CUDA_SUCCESS)
				outputKernelLaunchError(kernelWrangler, id, file, line);
			else if (debugExecution)
			{
				// PT: launching the kernel might have worked but executing the kernel is a different thing that can also fail.
				// This is only for debugging so code bloat won't matter here, and we can inline the whole thing.
				// This assumes the compiler removes all of it, which should work because we use a template for debugExecution.
				// Note that the launchKernel function itself has a LAUNCH_SYNCHRONOUS mode, so this seems quite redundant.
				// But that's how the existing code was when I found it, and I assume there must be a reason.
				const physx::PxCUresult syncErr = ctx->streamSynchronize(hStream);
				// PT: here again, don't assert immediately
				if (syncErr != CUDA_SUCCESS)
				{
					char buffer[4096];
					physx::Pxsnprintf(buffer, 4096, "GPU kernel '%s' execution failed!\n", kernelWrangler->getCuFunctionName(id));
					PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, file, line, buffer);
				}
				PX_ASSERT(syncErr == CUDA_SUCCESS);
			}
			PX_ASSERT(resultR == CUDA_SUCCESS);
			return resultR;
		}

		// PT: this one is for the regular launchKernel function
		template<const bool debugExecution>
		static PX_FORCE_INLINE physx::PxCUresult _launch(PxKernelWrangler* kernelWrangler, PxCudaContext* ctx, PxU16 id,
			PxU32 gridDimX, PxU32 gridDimY, PxU32 gridDimZ,
			PxU32 blockDimX, PxU32 blockDimY, PxU32 blockDimZ,
			PxU32 sharedMemBytes, CUstream hStream,
			PxCudaKernelParam* kernelParams, size_t kernelParamsSizeInBytes,
			const char* file, PxU32 line)
		{
			if (0)
				outputKernelLaunchDebugInfo(kernelWrangler, id, file, line);

			const physx::PxCUresult resultR = ctx->launchKernel(kernelWrangler->getCuFunction(id),
				gridDimX, gridDimY, gridDimZ,
				blockDimX, blockDimY, blockDimZ,
				sharedMemBytes, hStream,
				kernelParams, kernelParamsSizeInBytes);
			return _testLaunch<debugExecution>(resultR, kernelWrangler, ctx, id, hStream, file, line);
		}

		// PT: this one is for the new 'optimized' launchKernel function
		template<const bool debugExecution>
		static PX_FORCE_INLINE physx::PxCUresult _launch(PxKernelWrangler* kernelWrangler, PxCudaContext* ctx, PxU16 id,
			PxU32 gridDimX, PxU32 gridDimY, PxU32 gridDimZ,
			PxU32 blockDimX, PxU32 blockDimY, PxU32 blockDimZ,
			PxU32 sharedMemBytes, CUstream hStream,
			void** kernelParams,
			const char* file, PxU32 line)
		{
			if (0)
				outputKernelLaunchDebugInfo(kernelWrangler, id, file, line);

			const physx::PxCUresult resultR = ctx->launchKernel(kernelWrangler->getCuFunction(id),
				gridDimX, gridDimY, gridDimZ,
				blockDimX, blockDimY, blockDimZ,
				sharedMemBytes, hStream,
				kernelParams);
			return _testLaunch<debugExecution>(resultR, kernelWrangler, ctx, id, hStream, file, line);
		}

		struct PxKernelArgumentPacker
		{
			//https://riptutorial.com/cplusplus/example/3208/iterating-over-a-parameter-pack
			static void packArguments(PxCudaKernelParam* /*buffer*/, const PxU32 /*index*/)
			{
				// base case
			}

			template <class T, class... Ts>
			static void packArguments(PxCudaKernelParam* buffer, const PxU32 index, T const* first, Ts const*... rest)
			{
				buffer[index].data = const_cast<void*>(static_cast<void const*>(first));
				buffer[index].size = sizeof(*first);
				packArguments(buffer, index + 1, rest...);
			}
		};

		struct PxKernelIds
		{
			enum Enum
			{
				#define KERNEL_DEF(id, name) id,
				#include "PxKernelRegister.h"
				#undef KERNEL_DEF

				KERNEL_COUNT
			};
		};

		class PxStatelessKernelLauncher
		{
		protected:
			static PX_FORCE_INLINE PxU32 divideRoundUp(PxU32 a, PxU32 b)
			{
				return (a + b - 1) / b;
			}

			PxStatelessKernelLauncher() {}

		public:
			static PX_FORCE_INLINE void launchKernel(PxCudaContext* cudaContext, PxKernelWrangler* gpuKernelWranglerManager,
				PxU16 kernelFunctionId, PxU32 gridDimX, PxU32 gridDimY, PxU32 gridDimZ,
				PxU32 blockDimX, PxU32 blockDimY, PxU32 blockDimZ, PxU32 sharedMemBytes, CUstream stream, PxCudaKernelParam* kernelParams, PxU32 kernelParamsSize)
			{
				if (gridDimX == 0 || gridDimY == 0 || gridDimZ == 0 ||
					blockDimX == 0 || blockDimY == 0 || blockDimZ == 0)
					return;

				_launch<ENABLE_KERNEL_LAUNCH_ERROR_CHECK>(gpuKernelWranglerManager, cudaContext, kernelFunctionId,
					gridDimX, gridDimY, gridDimZ, blockDimX, blockDimY, blockDimZ,
					sharedMemBytes, stream, kernelParams, kernelParamsSize, PX_FL);
			}

			static PX_FORCE_INLINE void launchKernelBlocks(PxCudaContext* cudaContext, PxKernelWrangler* gpuKernelWranglerManager,
				PxU16 kernelFunctionId, PxU32 numBlocks, PxU32 numThreadsPerBlock, PxU32 sharedMemBytes, CUstream stream, PxCudaKernelParam* kernelParams, PxU32 kernelParamsSize)
			{
				launchKernel(cudaContext, gpuKernelWranglerManager, kernelFunctionId, numBlocks, 1, 1, numThreadsPerBlock, 1, 1, sharedMemBytes, stream, kernelParams, kernelParamsSize);
			}

			template <class ...Args>
			static PX_FORCE_INLINE void launchKernelAutoPackParameters(PxCudaContext* cudaContext, PxKernelWrangler* gpuKernelWranglerManager,
				PxU16 kernelFunctionId, PxU32 gridDimX, PxU32 gridDimY, PxU32 gridDimZ,
				PxU32 blockDimX, PxU32 blockDimY, PxU32 blockDimZ, PxU32 sharedMemBytes, CUstream stream, const Args&... args)
			{
				PxCudaKernelParam p[sizeof...(args)];
				PxKernelArgumentPacker::packArguments(p, 0, &args...);
				launchKernel(cudaContext, gpuKernelWranglerManager, kernelFunctionId, gridDimX, gridDimY, gridDimZ,
					blockDimX, blockDimY, blockDimZ, sharedMemBytes, stream, p, sizeof(p));
			}

			template <class ...Args>
			static PX_FORCE_INLINE void launchKernelBlocksAutoPackParameters(PxCudaContext* cudaContext, PxKernelWrangler* gpuKernelWranglerManager,
				PxU16 kernelFunctionId, PxU32 numBlocks, PxU32 numThreadsPerBlock, PxU32 sharedMemBytes, CUstream stream, const Args&... args)
			{
				PxCudaKernelParam p[sizeof...(args)];
				PxKernelArgumentPacker::packArguments(p, 0, &args...);
				launchKernelBlocks(cudaContext, gpuKernelWranglerManager, kernelFunctionId, numBlocks,
					numThreadsPerBlock, sharedMemBytes, stream, p, sizeof(p));
			}

			static PX_FORCE_INLINE void launchKernelThreads(PxCudaContext* cudaContext, PxKernelWrangler* gpuKernelWranglerManager,
				PxU16 kernelFunctionId, PxU32 totalNumThreads, PxU32 numThreadsPerBlock, PxU32 sharedMemBytes, CUstream stream, PxCudaKernelParam* kernelParams, PxU32 kernelParamsSize)
			{
				launchKernelBlocks(cudaContext, gpuKernelWranglerManager, kernelFunctionId, divideRoundUp(totalNumThreads, numThreadsPerBlock),
					numThreadsPerBlock, sharedMemBytes, stream, kernelParams, kernelParamsSize);
			}

			template <class ...Args>
			static PX_FORCE_INLINE void launchKernelThreadsAutoPackParameters(PxCudaContext* cudaContext, PxKernelWrangler* gpuKernelWranglerManager,
				PxU16 kernelFunctionId, PxU32 totalNumThreads, PxU32 numThreadsPerBlock, PxU32 sharedMemBytes, CUstream stream, const Args&... args)
			{
				PxCudaKernelParam p[sizeof...(args)];
				PxKernelArgumentPacker::packArguments(p, 0, &args...);
				launchKernelThreads(cudaContext, gpuKernelWranglerManager, kernelFunctionId, totalNumThreads,
					numThreadsPerBlock, sharedMemBytes, stream, p, sizeof(p));
			}
		};		

		class PxKernelLauncher
		{
		protected:			
			PxKernelLauncher() {}

		public:
			static void initialize(PxCudaContextManager& cudaContextManager);
			static void release();

			static void setCudaContextManager(PxCudaContextManager* cudaContextManager);
			static PxCudaContextManager* getCudaContextManager();
			static PxKernelWrangler* getKernelWrangler();

			template <class ...Args>
			static void launchKernel(PxKernelIds::Enum kernelId, PxU32 numBlocks, PxU32 numThreadsPerBlock, PxU32 sharedMemorySize, CUstream stream, const Args&... args)
			{
				PxCudaKernelParam p[sizeof...(args)];
				PxKernelArgumentPacker::packArguments(p, 0, &args...);
				PxStatelessKernelLauncher::launchKernel(getCudaContextManager()->getCudaContext(), getKernelWrangler(),
					PxU16(kernelId), numBlocks, 1, 1, numThreadsPerBlock, 1, 1, sharedMemorySize, stream, p, sizeof(p));
			}
		};

	}

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
