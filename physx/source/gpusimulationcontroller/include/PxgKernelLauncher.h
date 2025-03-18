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
// Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef PXG_KERNEL_LAUNCHER_H
#define PXG_KERNEL_LAUNCHER_H

#include "cudamanager/PxCudaContextManager.h"
#include "CudaKernelWrangler.h"
#include "PxgKernelWrangler.h"
#include "cudamanager/PxCudaContext.h"
#include "foundation/PxString.h"

#define KERNEL_LAUNCH_ERROR_CHECK 0

namespace physx
{
	// PT: do not inline this
	static PX_NOINLINE void outputKernelLaunchDebugInfo(KernelWrangler* kernelWrangler, PxU16 id, const char* file, PxU32 line)
	{
		char errorMsg[4096];
		physx::Pxsnprintf(errorMsg, 4096, "Launching GPU kernel %s...\n", kernelWrangler->getCuFunctionName(id));
		PxGetFoundation().error(PxErrorCode::eDEBUG_INFO, file, line, errorMsg);
	}

	// PT: do not inline this
	static PX_NOINLINE void outputKernelLaunchError(KernelWrangler* kernelWrangler, PxU16 kernelId, PxCUresult result, const char* file, PxU32 line)
	{
		char errorMsg[4096];
		physx::Pxsnprintf(errorMsg, 4096, "GPU kernel '%s' failed to launch with error %u!!\n",
			kernelWrangler->getCuFunctionName(kernelId), result.value);
		PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, file, line, errorMsg);
	}

	//do not inline this
	static PX_NOINLINE void outputKernelLaunchSyncError(KernelWrangler* kernelWrangler, PxU16 kernelId, PxCUresult result, const char* file, PxU32 line)
	{
		char buffer[4096];
		physx::Pxsnprintf(buffer, 4096, "GPU kernel '%s' execution failed with error %u!\n",
			kernelWrangler->getCuFunctionName(kernelId), result.value);
		PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, file, line, buffer);
	}

	template<const bool debugExecution>
	static PX_FORCE_INLINE PxCUresult _testLaunch(PxCUresult resultR, KernelWrangler* kernelWrangler, PxCudaContext* ctx, PxU16 kernelId, CUstream hStream, const char* file, PxU32 line)
	{
		// PT: don't assert immediately, we want to print the failing kernel's name first
		if(resultR != CUDA_SUCCESS)
			outputKernelLaunchError(kernelWrangler, kernelId, resultR, file, line);
		else if(debugExecution)
		{
			// PT: launching the kernel might have worked but executing the kernel is a different thing that can also fail.
			// This is only for debugging so code bloat won't matter here, and we can inline the whole thing.
			// This assumes the compiler removes all of it, which should work because we use a template for debugExecution.
			const PxCUresult syncErr = ctx->streamSynchronize(hStream);
			// PT: here again, don't assert immediately
			if(syncErr != CUDA_SUCCESS)
			{
				outputKernelLaunchSyncError(kernelWrangler, kernelId, syncErr, file, line);
				/*char buffer[4096];
				physx::Pxsnprintf(buffer, 4096, "GPU kernel '%s' execution failed with error %u!\n",
					kernelWrangler->getCuFunctionName(kernelId), syncErr.value);
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, file, line, buffer);*/
			}
			PX_ASSERT(syncErr == CUDA_SUCCESS);
		}
		PX_ASSERT(resultR == CUDA_SUCCESS);
		return resultR;
	}

	// PT: this one is for the regular launchKernel function
	template<const bool debugExecution>
	static PX_FORCE_INLINE PxCUresult _launch(
		KernelWrangler* kernelWrangler, PxCudaContext* ctx, PxU16 kernelId,
		PxU32 gridDimX, PxU32 gridDimY, PxU32 gridDimZ,
		PxU32 blockDimX, PxU32 blockDimY, PxU32 blockDimZ,
		PxU32 sharedMemBytes, CUstream hStream,
		PxCudaKernelParam* kernelParams, size_t kernelParamsSizeInBytes,
		const char* file, PxU32 line)
	{
		if (0)
			outputKernelLaunchDebugInfo(kernelWrangler, kernelId, file, line);

		const PxCUresult resultR = ctx->launchKernel(kernelWrangler->getCuFunction(kernelId),
			gridDimX, gridDimY, gridDimZ,
			blockDimX, blockDimY, blockDimZ,
			sharedMemBytes, hStream,
			kernelParams, kernelParamsSizeInBytes,
			NULL,
			file, line);

		return _testLaunch<debugExecution>(resultR, kernelWrangler, ctx, kernelId, hStream, file, line);
	}

	// PT: this one is for the new 'optimized' launchKernel function
	template<const bool debugExecution>
	static PX_FORCE_INLINE PxCUresult _launch(
		KernelWrangler* kernelWrangler, PxCudaContext* ctx, PxU16 kernelId,
		PxU32 gridDimX, PxU32 gridDimY, PxU32 gridDimZ,
		PxU32 blockDimX, PxU32 blockDimY, PxU32 blockDimZ,
		PxU32 sharedMemBytes, CUstream hStream,
		void** kernelParams,
		const char* file, PxU32 line)
	{
		if (0)
			outputKernelLaunchDebugInfo(kernelWrangler, kernelId, file, line);

		const PxCUresult resultR = ctx->launchKernel(kernelWrangler->getCuFunction(kernelId),
			gridDimX, gridDimY, gridDimZ,
			blockDimX, blockDimY, blockDimZ,
			sharedMemBytes, hStream,
			kernelParams,
			NULL,
			file, line);

		return _testLaunch<debugExecution>(resultR, kernelWrangler, ctx, kernelId, hStream, file, line);
	}

	class PxgKernelLauncher
	{
	private:
		static PX_FORCE_INLINE PxU32 divideRoundUp(PxU32 a, PxU32 b)
		{
			return (a + b - 1) / b;
		}

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

		//https://riptutorial.com/cplusplus/example/3208/iterating-over-a-parameter-pack
		static void packArgumentsPointerOnly(void** /*buffer*/, const PxU32 /*index*/)
		{
			// base case
		}

		template <class T, class... Ts>
		static void packArgumentsPointerOnly(void** buffer, const PxU32 index, T const* first, Ts const*... rest)
		{
			buffer[index] = const_cast<void*>(static_cast<void const*>(first));
			packArgumentsPointerOnly(buffer, index + 1, rest...);
		}


	public:	
		static PX_FORCE_INLINE PxCUresult launchKernel(PxCudaContext* cudaContext, PxgCudaKernelWranglerManager* gpuKernelWranglerManager,
			PxU16 kernelFunctionId, PxU32 gridDimX, PxU32 gridDimY, PxU32 gridDimZ,
			PxU32 blockDimX, PxU32 blockDimY, PxU32 blockDimZ, PxU32 sharedMemBytes, CUstream stream, PxCudaKernelParam* kernelParams, PxU32 kernelParamsSize)
		{
			return _launch<KERNEL_LAUNCH_ERROR_CHECK>(gpuKernelWranglerManager->mKernelWrangler, cudaContext, kernelFunctionId,
													gridDimX, gridDimY, gridDimZ, blockDimX, blockDimY, blockDimZ,
													sharedMemBytes, stream, kernelParams, kernelParamsSize, PX_FL);
		}

		static PX_FORCE_INLINE PxCUresult launchKernel(PxCudaContext* cudaContext, PxgCudaKernelWranglerManager* gpuKernelWranglerManager,
			PxU16 kernelFunctionId, PxU32 gridDimX, PxU32 gridDimY, PxU32 gridDimZ,
			PxU32 blockDimX, PxU32 blockDimY, PxU32 blockDimZ, PxU32 sharedMemBytes, CUstream stream, void** kernelParams)
		{
			return _launch<KERNEL_LAUNCH_ERROR_CHECK>(gpuKernelWranglerManager->mKernelWrangler, cudaContext, kernelFunctionId,
				gridDimX, gridDimY, gridDimZ, blockDimX, blockDimY, blockDimZ,
				sharedMemBytes, stream, kernelParams, PX_FL);
		}

		static PX_FORCE_INLINE PxCUresult launchKernelBlocks(PxCudaContext* cudaContext, PxgCudaKernelWranglerManager* gpuKernelWranglerManager,
			PxU16 kernelFunctionId, PxU32 numBlocks, PxU32 numThreadsPerBlock, PxU32 sharedMemBytes, CUstream stream, PxCudaKernelParam* kernelParams, PxU32 kernelParamsSize)
		{
			return launchKernel(cudaContext, gpuKernelWranglerManager, kernelFunctionId, numBlocks, 1, 1, numThreadsPerBlock, 1, 1, sharedMemBytes, stream, kernelParams, kernelParamsSize);
		}

		template <class ...Args>
		static PX_FORCE_INLINE PxCUresult launchKernelAutoPackParameters(PxCudaContext* cudaContext, PxgCudaKernelWranglerManager* gpuKernelWranglerManager,
			PxU16 kernelFunctionId, PxU32 gridDimX, PxU32 gridDimY, PxU32 gridDimZ,
			PxU32 blockDimX, PxU32 blockDimY, PxU32 blockDimZ, PxU32 sharedMemBytes, CUstream stream, const Args&... args)
		{
			PxCudaKernelParam p[sizeof...(args)];
			packArguments(p, 0, &args...);
			return launchKernel(cudaContext, gpuKernelWranglerManager, kernelFunctionId, gridDimX, gridDimY, gridDimZ,
				blockDimX, blockDimY, blockDimZ, sharedMemBytes, stream, p, sizeof(p));
		}

		template <class ...Args>
		static PX_FORCE_INLINE PxCUresult launchKernelBlocksAutoPackParameters(PxCudaContext* cudaContext, PxgCudaKernelWranglerManager* gpuKernelWranglerManager,
			PxU16 kernelFunctionId, PxU32 numBlocks, PxU32 numThreadsPerBlock, PxU32 sharedMemBytes, CUstream stream, const Args&... args)
		{
			PxCudaKernelParam p[sizeof...(args)];
			packArguments(p, 0, &args...);
			return launchKernelBlocks(cudaContext, gpuKernelWranglerManager, kernelFunctionId, numBlocks,
				numThreadsPerBlock, sharedMemBytes, stream, p, sizeof(p));
		}

		static PX_FORCE_INLINE PxCUresult launchKernelThreads(PxCudaContext* cudaContext, PxgCudaKernelWranglerManager* gpuKernelWranglerManager,
			PxU16 kernelFunctionId, PxU32 totalNumThreads, PxU32 numThreadsPerBlock, PxU32 sharedMemBytes, CUstream stream, PxCudaKernelParam* kernelParams, PxU32 kernelParamsSize)
		{
			return launchKernelBlocks(cudaContext, gpuKernelWranglerManager, kernelFunctionId, divideRoundUp(totalNumThreads, numThreadsPerBlock),
				numThreadsPerBlock, sharedMemBytes, stream, kernelParams, kernelParamsSize);
		}

		template <class ...Args>
		static PX_FORCE_INLINE PxCUresult launchKernelThreadsAutoPackParameters(PxCudaContext* cudaContext, PxgCudaKernelWranglerManager* gpuKernelWranglerManager,
			PxU16 kernelFunctionId, PxU32 totalNumThreads, PxU32 numThreadsPerBlock, PxU32 sharedMemBytes, CUstream stream, const Args&... args)
		{
			PxCudaKernelParam p[sizeof...(args)];
			packArguments(p, 0, &args...);
			return launchKernelThreads(cudaContext, gpuKernelWranglerManager, kernelFunctionId, totalNumThreads,
				numThreadsPerBlock, sharedMemBytes, stream, p, sizeof(p));
		}


		PxgKernelLauncher()
			: mCudaContextManager(NULL), mGpuKernelWranglerManager(NULL) {}

		PxgKernelLauncher(PxCudaContextManager* cudaContextManager, PxgCudaKernelWranglerManager* gpuKernelWranglerManager) 
			 : mCudaContextManager(cudaContextManager), mGpuKernelWranglerManager(gpuKernelWranglerManager) {}

		PX_FORCE_INLINE PxCudaContextManager* getCudaContextManager() { return mCudaContextManager; }
		PX_FORCE_INLINE const PxCudaContextManager* getCudaContextManager() const { return mCudaContextManager; }
		PX_FORCE_INLINE PxgCudaKernelWranglerManager* getKernelWrangler() { return mGpuKernelWranglerManager; }
		PX_FORCE_INLINE const PxgCudaKernelWranglerManager* getKernelWrangler() const { return mGpuKernelWranglerManager; }

		template <class ...Args>
		PxCUresult launchKernel(PxU16 kernelId, PxU32 numBlocks, PxU32 numThreadsPerBlock, PxU32 sharedMemorySize, CUstream stream, const Args&... args)
		{
			void* p[sizeof...(args)];
			packArgumentsPointerOnly(p, 0, &args...);
			return PxgKernelLauncher::launchKernel(getCudaContextManager()->getCudaContext(), getKernelWrangler(),
				kernelId, numBlocks, 1, 1, numThreadsPerBlock, 1, 1, sharedMemorySize, stream, p);
		}

		template <class ...Args>
		PxCUresult launchKernelXYZ(PxU16 kernelId, PxU32 numBlocksX, PxU32 numBlocksY, PxU32 numBlocksZ,
			PxU32 numThreadsPerBlockX, PxU32 numThreadsPerBlockY, PxU32 numThreadsPerBlockZ, PxU32 sharedMemorySize, CUstream stream, const Args&... args)
		{
			void* p[sizeof...(args)];
			packArgumentsPointerOnly(p, 0, &args...);
			return PxgKernelLauncher::launchKernel(getCudaContextManager()->getCudaContext(), getKernelWrangler(),
				kernelId, numBlocksX, numBlocksY, numBlocksZ, numThreadsPerBlockX, numThreadsPerBlockY, numThreadsPerBlockZ, sharedMemorySize, stream, p);
		}

		template <class ...Args>
		PxCUresult launchKernelPtr(PxU16 kernelId, PxU32 numBlocks, PxU32 numThreadsPerBlock, PxU32 sharedMemorySize, CUstream stream, const Args*... args)
		{
			void* p[sizeof...(args)];
			packArgumentsPointerOnly(p, 0, args...);
			return PxgKernelLauncher::launchKernel(getCudaContextManager()->getCudaContext(), getKernelWrangler(),
				kernelId, numBlocks, 1, 1, numThreadsPerBlock, 1, 1, sharedMemorySize, stream, p);
		}

		template <class ...Args>
		PxCUresult launchKernelXYZPtr(PxU16 kernelId, PxU32 numBlocksX, PxU32 numBlocksY, PxU32 numBlocksZ,
			PxU32 numThreadsPerBlockX, PxU32 numThreadsPerBlockY, PxU32 numThreadsPerBlockZ, PxU32 sharedMemorySize, CUstream stream, const Args*... args)
		{
			void* p[sizeof...(args)];
			packArgumentsPointerOnly(p, 0, args...);
			return PxgKernelLauncher::launchKernel(getCudaContextManager()->getCudaContext(), getKernelWrangler(),
				kernelId, numBlocksX, numBlocksY, numBlocksZ, numThreadsPerBlockX, numThreadsPerBlockY, numThreadsPerBlockZ, sharedMemorySize, stream, p);
		}

	private:
		PxCudaContextManager* mCudaContextManager;
		PxgCudaKernelWranglerManager* mGpuKernelWranglerManager;
	};
}

#endif
