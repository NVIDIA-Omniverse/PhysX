// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//


// Runtime-loaded CUDA driver shim for IOptionalCuda.
//
// Build-time requirements:
// - This translation unit requires <cuda.h> (CUDA driver headers) to compile, for
//   driver API types/constants (CUcontext, CUevent, CUDA_MEMCPY2D, etc.).
//
// Runtime behavior:
// - Does NOT link libcuda (no DT_NEEDED on libcuda.so.1 / nvcuda.dll).
// - Dynamically loads the CUDA driver library via dlopen/LoadLibrary and resolves
//   the required cu* symbols (using cuGetProcAddress when available, with a dlsym
//   fallback).
// - CUDA is treated as available only if the driver library loads, all required
//   symbols resolve, cuInit succeeds, and at least one device is present.
// - Every exported operation is guarded by ensureInit(); on failure it returns
//   false and sets outStatus to CUDA_ERROR_NOT_INITIALIZED.

#include "CudaShim.h"

#include <cuda.h>

#include <carb/logging/Log.h>

#include <atomic>
#include <mutex>
#include <unordered_map>

#if defined(_WIN32)
#    include <windows.h>
#    include <setupapi.h>
#    include <devguid.h>
#else
#    include <dlfcn.h>
#endif

namespace omni
{
namespace physx
{
namespace cudaShim
{
namespace
{

// -- Function pointer table -----------------------------------------------

struct CudaFns
{
    // Loader -- 5-arg signature (cuGetProcAddress_v2, CUDA 12+).
    // On CUDA 11.x drivers only the 4-arg cuGetProcAddress exists; we must NOT
    // call that through this 5-arg pointer.  initOnce() attempts to resolve the
    // _v2 symbol and leaves this nullptr when unavailable, causing
    // getProcAddress() to fall back to raw dlsym for all lookups.
    CUresult (*cuGetProcAddress)(const char*, void**, int, cuuint64_t, CUdriverProcAddressQueryResult*) = nullptr;

    // Availability probe
    CUresult (*cuInit)(unsigned int) = nullptr;
    CUresult (*cuDeviceGetCount)(int*) = nullptr;
    CUresult (*cuDeviceGet)(CUdevice*, int) = nullptr;

    // Context
    CUresult (*cuCtxGetCurrent)(CUcontext*) = nullptr;
    CUresult (*cuCtxSetCurrent)(CUcontext) = nullptr;
    CUresult (*cuCtxPushCurrent_v2)(CUcontext) = nullptr;
    CUresult (*cuCtxPopCurrent_v2)(CUcontext*) = nullptr;
    CUresult (*cuCtxGetDevice)(CUdevice*) = nullptr;
    CUresult (*cuCtxSynchronize)() = nullptr;
    CUresult (*cuDevicePrimaryCtxRetain)(CUcontext*, CUdevice) = nullptr;
    CUresult (*cuDevicePrimaryCtxRelease)(CUdevice) = nullptr;

    // Memory
    CUresult (*cuMemAlloc_v2)(CUdeviceptr*, size_t) = nullptr;
    CUresult (*cuMemFree_v2)(CUdeviceptr) = nullptr;
    CUresult (*cuMemcpyHtoD_v2)(CUdeviceptr, const void*, size_t) = nullptr;
    CUresult (*cuMemcpyDtoH_v2)(void*, CUdeviceptr, size_t) = nullptr;
    CUresult (*cuMemcpy2D_v2)(const CUDA_MEMCPY2D*) = nullptr;
    CUresult (*cuMemsetD8_v2)(CUdeviceptr, unsigned char, size_t) = nullptr;
    CUresult (*cuMemsetD32_v2)(CUdeviceptr, unsigned int, size_t) = nullptr;

    // Events
    CUresult (*cuEventCreate)(CUevent*, unsigned int) = nullptr;
    CUresult (*cuEventDestroy_v2)(CUevent) = nullptr;
    CUresult (*cuEventRecord)(CUevent, CUstream) = nullptr;
    CUresult (*cuEventSynchronize)(CUevent) = nullptr;

    // Streams
    CUresult (*cuStreamCreate)(CUstream*, unsigned int) = nullptr;
    CUresult (*cuStreamDestroy_v2)(CUstream) = nullptr;
    CUresult (*cuStreamSynchronize)(CUstream) = nullptr;
    CUresult (*cuStreamWaitEvent)(CUstream, CUevent, unsigned int) = nullptr;

    // Device info — used by selectBestPhysicsDevice; not required for IOptionalCuda
    CUresult (*cuDeviceTotalMem_v2)(size_t*, CUdevice) = nullptr;
    CUresult (*cuDeviceGetAttribute)(int*, CUdevice_attribute, CUdevice) = nullptr;
};

static std::once_flag g_initOnce;
static std::atomic<int> g_available{-1}; // -1 unknown, 0 no, 1 yes
static CudaFns g_fns;

#if defined(_WIN32)
static HMODULE g_cudaModule = nullptr;
#else
static void* g_cudaModule = nullptr;
#endif

// -- Library loading -------------------------------------------------------

#if defined(_WIN32)
// Check for physically present NVIDIA GPU hardware via Windows SetupAPI.
// This prevents loading nvcuda.dll on machines where the driver DLL exists in
// System32 but no GPU is installed -- loading the driver in that state crashes
// inside DllMain before any of our higher-level guards can intervene.
static bool hasNvidiaGpuDevice()
{
    constexpr wchar_t kNvidiaVendorId[] = L"VEN_10DE";

    HDEVINFO devs = ::SetupDiGetClassDevsW(&GUID_DEVCLASS_DISPLAY, nullptr, nullptr, DIGCF_PRESENT);
    if (devs == INVALID_HANDLE_VALUE)
        return false; // Safe default: if SetupAPI is unavailable (e.g. containers), skip CUDA
                      // rather than risk crashing in LoadLibraryA("nvcuda.dll").

    SP_DEVINFO_DATA info{};
    info.cbSize = sizeof(info);
    bool found = false;

    for (DWORD i = 0; ::SetupDiEnumDeviceInfo(devs, i, &info); ++i)
    {
        wchar_t hwId[512]{};
        if (::SetupDiGetDeviceRegistryPropertyW(devs, &info, SPDRP_HARDWAREID,
                nullptr, reinterpret_cast<BYTE*>(hwId), sizeof(hwId), nullptr))
        {
            if (::wcsstr(hwId, kNvidiaVendorId))
            {
                found = true;
                break;
            }
        }
    }

    ::SetupDiDestroyDeviceInfoList(devs);
    return found;
}

#endif // _WIN32

// Intentionally no dlclose/FreeLibrary: the CUDA driver library is kept loaded
// for the lifetime of the process. Unloading it would invalidate resolved
// function pointers and risk use-after-free.
// NOTE: Must only be called under g_initOnce (via initOnce -> ensureInit).
static bool loadDriverLibrary()
{
#if defined(_WIN32)
    if (g_cudaModule) return true;

    if (!hasNvidiaGpuDevice())
    {
        CARB_LOG_INFO("IOptionalCuda: No NVIDIA display device present -- skipping nvcuda.dll load.");
        return false;
    }

    __try
    {
        g_cudaModule = ::LoadLibraryA("nvcuda.dll");
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        CARB_LOG_WARN("IOptionalCuda: Structured exception 0x%08lX while loading nvcuda.dll -- treating CUDA as unavailable.",
                      static_cast<unsigned long>(GetExceptionCode()));
        g_cudaModule = nullptr;
    }

    return g_cudaModule != nullptr;
#else
    if (g_cudaModule) return true;
    g_cudaModule = ::dlopen("libcuda.so.1", RTLD_LAZY | RTLD_LOCAL);
    if (!g_cudaModule)
        g_cudaModule = ::dlopen("libcuda.so", RTLD_LAZY | RTLD_LOCAL);
    return g_cudaModule != nullptr;
#endif
}

static void* getRawSymbol(const char* name)
{
#if defined(_WIN32)
    return g_cudaModule ? reinterpret_cast<void*>(::GetProcAddress(g_cudaModule, name)) : nullptr;
#else
    return g_cudaModule ? ::dlsym(g_cudaModule, name) : nullptr;
#endif
}

static bool getProcAddress(const char* symbol, void** out)
{
    if (!out) return false;
    *out = nullptr;
    if (g_fns.cuGetProcAddress)
    {
        CUdriverProcAddressQueryResult symbolStatus{};
        CUresult res = g_fns.cuGetProcAddress(symbol, out, CUDA_VERSION, CU_GET_PROC_ADDRESS_DEFAULT, &symbolStatus);
        if (res == CUDA_SUCCESS && *out) return true;
        // Fallback: some driver/toolkit combinations can have cuGetProcAddress present
        // but fail lookups for a particular CUDA_VERSION. In that case, still try the
        // raw symbol export to avoid disabling CUDA entirely.
        *out = getRawSymbol(symbol);
        return *out != nullptr;
    }
    *out = getRawSymbol(symbol);
    return *out != nullptr;
}

#define RESOLVE(name) getProcAddress(#name, reinterpret_cast<void**>(&g_fns.name))

static void initOnce()
{
    if (!loadDriverLibrary())
    {
        CARB_LOG_INFO("IOptionalCuda: CUDA driver library not found (no NVIDIA driver installed?).");
        g_available.store(0, std::memory_order_release);
        return;
    }

    // Resolve cuGetProcAddress_v2 (5-arg, CUDA 12+); left null on older drivers so
    // getProcAddress() falls back to raw symbol lookup -- calling the 4-arg variant
    // through this 5-arg pointer would be UB.
    g_fns.cuGetProcAddress = reinterpret_cast<decltype(g_fns.cuGetProcAddress)>(getRawSymbol("cuGetProcAddress_v2"));

    bool ok = true;

    // Availability probe
    ok &= RESOLVE(cuInit);
    ok &= RESOLVE(cuDeviceGetCount);
    ok &= RESOLVE(cuDeviceGet);

    // Context
    ok &= RESOLVE(cuCtxGetCurrent);
    ok &= RESOLVE(cuCtxSetCurrent);
    ok &= RESOLVE(cuCtxPushCurrent_v2);
    ok &= RESOLVE(cuCtxPopCurrent_v2);
    ok &= RESOLVE(cuCtxGetDevice);
    ok &= RESOLVE(cuCtxSynchronize);
    ok &= RESOLVE(cuDevicePrimaryCtxRetain);
    ok &= RESOLVE(cuDevicePrimaryCtxRelease);

    // Memory
    ok &= RESOLVE(cuMemAlloc_v2);
    ok &= RESOLVE(cuMemFree_v2);
    ok &= RESOLVE(cuMemcpyHtoD_v2);
    ok &= RESOLVE(cuMemcpyDtoH_v2);
    ok &= RESOLVE(cuMemcpy2D_v2);
    ok &= RESOLVE(cuMemsetD8_v2);
    ok &= RESOLVE(cuMemsetD32_v2);

    // Events
    ok &= RESOLVE(cuEventCreate);
    ok &= RESOLVE(cuEventDestroy_v2);
    ok &= RESOLVE(cuEventRecord);
    ok &= RESOLVE(cuEventSynchronize);

    // Streams
    ok &= RESOLVE(cuStreamCreate);
    ok &= RESOLVE(cuStreamDestroy_v2);
    ok &= RESOLVE(cuStreamSynchronize);
    ok &= RESOLVE(cuStreamWaitEvent);

    if (!ok)
    {
        CARB_LOG_INFO("IOptionalCuda: Failed to resolve one or more required CUDA driver symbols.");
        g_available.store(0, std::memory_order_release);
        return;
    }

    // Probe CUDA state before calling cuInit: CUDA_ERROR_NOT_INITIALIZED means we own init;
    // CUDA_SUCCESS means CUDA is already live (e.g. GPU rendering stack ran first), skip cuInit.
    // Any other result means CUDA is not usable. The memory probe below is the stub guard.
    {
        int preInitCount = 0;
        CUresult preInitResult = g_fns.cuDeviceGetCount(&preInitCount);
        if (preInitResult == CUDA_ERROR_NOT_INITIALIZED)
        {
            // Normal path: we are the first to initialize CUDA in this process.
            CUresult initResult = g_fns.cuInit(0);
            if (initResult != CUDA_SUCCESS)
            {
                CARB_LOG_WARN("IOptionalCuda: cuInit failed (CUresult %d) -- treating as CPU-only.", static_cast<int>(initResult));
                g_available.store(0, std::memory_order_release);
                return;
            }
        }
        else if (preInitResult == CUDA_SUCCESS)
        {
            // CUDA already initialized by another component (e.g. GPU rendering stack).
            // Skip cuInit -- the driver is already live. The memory probe below confirms
            // that a real GPU is present.
            CARB_LOG_INFO("IOptionalCuda: CUDA already initialized by another component "
                         "(%d device(s) visible pre-init); skipping cuInit.", preInitCount);
        }
        else
        {
            CARB_LOG_WARN("IOptionalCuda: cuDeviceGetCount (pre-init) returned %d -- no CUDA, treating as CPU-only.",
                          static_cast<int>(preInitResult));
            g_available.store(0, std::memory_order_release);
            return;
        }
    }

    int count = 0;
    CUresult countResult = g_fns.cuDeviceGetCount(&count);
    if (countResult != CUDA_SUCCESS || count <= 0)
    {
        CARB_LOG_INFO("IOptionalCuda: No CUDA devices found (count=%d, CUresult %d).", count, static_cast<int>(countResult));
        g_available.store(0, std::memory_order_release);
        return;
    }

    // Pre-context memory probe: cuDeviceTotalMem requires only a device handle, no context.
    // Serves as a second gate. Stub DLLs report 0 bytes; real GPUs report >= 256 MiB.
    // Also used by selectBestPhysicsDevice for multi-GPU ranking — resolve into the struct.

    // Try versioned name (CUDA 5+) first; fall back to the original unversioned symbol.
    if (!getProcAddress("cuDeviceTotalMem_v2", reinterpret_cast<void**>(&g_fns.cuDeviceTotalMem_v2)) || !g_fns.cuDeviceTotalMem_v2)
        getProcAddress("cuDeviceTotalMem", reinterpret_cast<void**>(&g_fns.cuDeviceTotalMem_v2));
    getProcAddress("cuDeviceGetAttribute", reinterpret_cast<void**>(&g_fns.cuDeviceGetAttribute));

    constexpr size_t kMinRealGpuMemBytes = 256ULL * 1024ULL * 1024ULL;
    size_t totalMem = 0;
    bool memConfirmed = false;

    if (g_fns.cuDeviceTotalMem_v2 && g_fns.cuDeviceGet)
    {
        CUdevice device0 = 0;
        if (g_fns.cuDeviceGet(&device0, 0) == CUDA_SUCCESS)
        {
            CUresult memRes = g_fns.cuDeviceTotalMem_v2(&totalMem, device0);
            memConfirmed = (memRes == CUDA_SUCCESS && totalMem >= kMinRealGpuMemBytes);
        }
    }

    if (!memConfirmed)
    {
        CARB_LOG_INFO("IOptionalCuda: cuDeviceTotalMem returned %zu bytes (confirmed=%d) -- "
                      "stub DLL or unresolvable query, treating as CPU-only.",
                      totalMem, static_cast<int>(memConfirmed));
        g_available.store(0, std::memory_order_release);
        return;
    }

    CARB_LOG_INFO("IOptionalCuda: CUDA available (%d device%s, %.0f MiB device memory).",
                  count, count == 1 ? "" : "s", static_cast<double>(totalMem) / (1024.0 * 1024.0));
    g_available.store(1, std::memory_order_release);
}

#undef RESOLVE

static bool ensureInit()
{
    int avail = g_available.load(std::memory_order_acquire);
    if (avail >= 0) return avail == 1;
    std::call_once(g_initOnce, initOnce);
    return g_available.load(std::memory_order_acquire) == 1;
}

// -- IOptionalCuda function implementations -------------------------------

static bool CARB_ABI impl_cudaAvailable()
{
    return ensureInit();
}

static bool failNotInitialized(int* outStatus)
{
    if (outStatus)
        *outStatus = CUDA_ERROR_NOT_INITIALIZED;
    return false;
}

static bool CARB_ABI impl_ctxGetCurrent(uintptr_t* outCtx, int* outStatus)
{
    if (outCtx) *outCtx = 0;
    if (!ensureInit())
        return failNotInitialized(outStatus);
    CUcontext ctx = nullptr;
    CUresult r = g_fns.cuCtxGetCurrent(&ctx);
    if (outCtx) *outCtx = reinterpret_cast<uintptr_t>(ctx);
    if (outStatus) *outStatus = static_cast<int>(r);
    return r == CUDA_SUCCESS;
}

static bool CARB_ABI impl_ctxPushCurrent(uintptr_t ctx, int* outStatus)
{
    if (!ensureInit())
        return failNotInitialized(outStatus);
    CUresult r = g_fns.cuCtxPushCurrent_v2(reinterpret_cast<CUcontext>(ctx));
    if (outStatus) *outStatus = static_cast<int>(r);
    return r == CUDA_SUCCESS;
}

static bool CARB_ABI impl_ctxPopCurrent(uintptr_t* outCtx, int* outStatus)
{
    if (outCtx) *outCtx = 0;
    if (!ensureInit())
        return failNotInitialized(outStatus);
    CUcontext ctx = nullptr;
    CUresult r = g_fns.cuCtxPopCurrent_v2(&ctx);
    if (outCtx) *outCtx = reinterpret_cast<uintptr_t>(ctx);
    if (outStatus) *outStatus = static_cast<int>(r);
    return r == CUDA_SUCCESS;
}

static bool CARB_ABI impl_ctxGetDevice(int* outDevice, int* outStatus)
{
    if (outDevice) *outDevice = -1;
    if (!ensureInit())
        return failNotInitialized(outStatus);
    CUdevice dev = -1;
    CUresult r = g_fns.cuCtxGetDevice(&dev);
    if (outDevice) *outDevice = static_cast<int>(dev);
    if (outStatus) *outStatus = static_cast<int>(r);
    return r == CUDA_SUCCESS;
}

static bool CARB_ABI impl_ctxSynchronize(int* outStatus)
{
    if (!ensureInit())
        return failNotInitialized(outStatus);
    CUresult r = g_fns.cuCtxSynchronize();
    if (outStatus) *outStatus = static_cast<int>(r);
    return r == CUDA_SUCCESS;
}

static bool CARB_ABI impl_deviceGetCount(int* outCount, int* outStatus)
{
    if (outCount) *outCount = 0;
    if (!ensureInit())
        return failNotInitialized(outStatus);
    int count = 0;
    CUresult r = g_fns.cuDeviceGetCount(&count);
    if (outCount) *outCount = count;
    if (outStatus) *outStatus = static_cast<int>(r);
    return r == CUDA_SUCCESS;
}

static bool CARB_ABI impl_deviceGet(int* outDevice, int ordinal, int* outStatus)
{
    if (outDevice) *outDevice = -1;
    if (!ensureInit())
        return failNotInitialized(outStatus);
    CUdevice dev = -1;
    CUresult r = g_fns.cuDeviceGet(&dev, ordinal);
    if (outDevice) *outDevice = static_cast<int>(dev);
    if (outStatus) *outStatus = static_cast<int>(r);
    return r == CUDA_SUCCESS;
}

static bool CARB_ABI impl_memAlloc(uintptr_t* outDevicePtr, size_t bytes, int* outStatus)
{
    if (outDevicePtr) *outDevicePtr = 0;
    if (!ensureInit())
        return failNotInitialized(outStatus);
    CUdeviceptr dptr = 0;
    CUresult r = g_fns.cuMemAlloc_v2(&dptr, bytes);
    if (outDevicePtr) *outDevicePtr = static_cast<uintptr_t>(dptr);
    if (outStatus) *outStatus = static_cast<int>(r);
    return r == CUDA_SUCCESS;
}

static bool CARB_ABI impl_memFree(uintptr_t devicePtr, int* outStatus)
{
    if (!devicePtr) { if (outStatus) *outStatus = CUDA_SUCCESS; return true; }
    if (!ensureInit())
        return failNotInitialized(outStatus);
    CUresult r = g_fns.cuMemFree_v2(static_cast<CUdeviceptr>(devicePtr));
    if (outStatus) *outStatus = static_cast<int>(r);
    return r == CUDA_SUCCESS;
}

static bool CARB_ABI impl_memcpyHtoD(uintptr_t dst, const void* src, size_t bytes, int* outStatus)
{
    if (!ensureInit())
        return failNotInitialized(outStatus);
    CUresult r = g_fns.cuMemcpyHtoD_v2(static_cast<CUdeviceptr>(dst), src, bytes);
    if (outStatus) *outStatus = static_cast<int>(r);
    return r == CUDA_SUCCESS;
}

static bool CARB_ABI impl_memcpyDtoH(void* dst, uintptr_t src, size_t bytes, int* outStatus)
{
    if (!ensureInit())
        return failNotInitialized(outStatus);
    CUresult r = g_fns.cuMemcpyDtoH_v2(dst, static_cast<CUdeviceptr>(src), bytes);
    if (outStatus) *outStatus = static_cast<int>(r);
    return r == CUDA_SUCCESS;
}

static bool CARB_ABI impl_memcpy2DDeviceToDevice(
    uintptr_t dstDevice,
    size_t dstPitch,
    uintptr_t srcDevice,
    size_t srcPitch,
    size_t widthInBytes,
    size_t height,
    int* outStatus)
{
    if (widthInBytes == 0 || height == 0)
    {
        if (outStatus) *outStatus = CUDA_SUCCESS;
        return true;
    }
    if (!ensureInit())
        return failNotInitialized(outStatus);
    CUDA_MEMCPY2D p{};
    p.srcMemoryType = CU_MEMORYTYPE_DEVICE;
    p.srcDevice = static_cast<CUdeviceptr>(srcDevice);
    p.srcPitch = srcPitch;
    p.dstMemoryType = CU_MEMORYTYPE_DEVICE;
    p.dstDevice = static_cast<CUdeviceptr>(dstDevice);
    p.dstPitch = dstPitch;
    p.WidthInBytes = widthInBytes;
    p.Height = height;
    CUresult r = g_fns.cuMemcpy2D_v2(&p);
    if (outStatus) *outStatus = static_cast<int>(r);
    return r == CUDA_SUCCESS;
}

static bool CARB_ABI impl_memsetD8(uintptr_t dst, uint8_t value, size_t count, int* outStatus)
{
    if (!ensureInit())
        return failNotInitialized(outStatus);
    CUresult r = g_fns.cuMemsetD8_v2(static_cast<CUdeviceptr>(dst), value, count);
    if (outStatus) *outStatus = static_cast<int>(r);
    return r == CUDA_SUCCESS;
}

static bool CARB_ABI impl_memsetD32(uintptr_t dst, uint32_t value, size_t count, int* outStatus)
{
    if (!ensureInit())
        return failNotInitialized(outStatus);
    CUresult r = g_fns.cuMemsetD32_v2(static_cast<CUdeviceptr>(dst), value, count);
    if (outStatus) *outStatus = static_cast<int>(r);
    return r == CUDA_SUCCESS;
}

static bool CARB_ABI impl_eventCreate(uintptr_t* outEvent, unsigned int flags, int* outStatus)
{
    if (outEvent) *outEvent = 0;
    if (!ensureInit())
        return failNotInitialized(outStatus);
    CUevent ev = nullptr;
    CUresult r = g_fns.cuEventCreate(&ev, flags);
    if (outEvent) *outEvent = reinterpret_cast<uintptr_t>(ev);
    if (outStatus) *outStatus = static_cast<int>(r);
    return r == CUDA_SUCCESS;
}

static bool CARB_ABI impl_eventDestroy(uintptr_t event, int* outStatus)
{
    if (!event) { if (outStatus) *outStatus = CUDA_SUCCESS; return true; }
    if (!ensureInit())
        return failNotInitialized(outStatus);
    CUresult r = g_fns.cuEventDestroy_v2(reinterpret_cast<CUevent>(event));
    if (outStatus) *outStatus = static_cast<int>(r);
    return r == CUDA_SUCCESS;
}

static bool CARB_ABI impl_eventRecord(uintptr_t event, uintptr_t stream, int* outStatus)
{
    if (!ensureInit())
        return failNotInitialized(outStatus);
    CUresult r = g_fns.cuEventRecord(reinterpret_cast<CUevent>(event), reinterpret_cast<CUstream>(stream));
    if (outStatus) *outStatus = static_cast<int>(r);
    return r == CUDA_SUCCESS;
}

static bool CARB_ABI impl_eventSynchronize(uintptr_t event, int* outStatus)
{
    if (!ensureInit())
        return failNotInitialized(outStatus);
    CUresult r = g_fns.cuEventSynchronize(reinterpret_cast<CUevent>(event));
    if (outStatus) *outStatus = static_cast<int>(r);
    return r == CUDA_SUCCESS;
}

static bool CARB_ABI impl_streamCreate(uintptr_t* outStream, unsigned int flags, int* outStatus)
{
    if (outStream) *outStream = 0;
    if (!ensureInit())
        return failNotInitialized(outStatus);
    CUstream s = nullptr;
    CUresult r = g_fns.cuStreamCreate(&s, flags);
    if (outStream) *outStream = reinterpret_cast<uintptr_t>(s);
    if (outStatus) *outStatus = static_cast<int>(r);
    return r == CUDA_SUCCESS;
}

static bool CARB_ABI impl_streamDestroy(uintptr_t stream, int* outStatus)
{
    if (!stream) { if (outStatus) *outStatus = CUDA_SUCCESS; return true; }
    if (!ensureInit())
        return failNotInitialized(outStatus);
    CUresult r = g_fns.cuStreamDestroy_v2(reinterpret_cast<CUstream>(stream));
    if (outStatus) *outStatus = static_cast<int>(r);
    return r == CUDA_SUCCESS;
}

static bool CARB_ABI impl_streamSynchronize(uintptr_t stream, int* outStatus)
{
    if (!ensureInit())
        return failNotInitialized(outStatus);
    CUresult r = g_fns.cuStreamSynchronize(reinterpret_cast<CUstream>(stream));
    if (outStatus) *outStatus = static_cast<int>(r);
    return r == CUDA_SUCCESS;
}

static bool CARB_ABI impl_streamWaitEvent(uintptr_t stream, uintptr_t event, unsigned int flags, int* outStatus)
{
    if (!ensureInit())
        return failNotInitialized(outStatus);
    CUresult r = g_fns.cuStreamWaitEvent(reinterpret_cast<CUstream>(stream), reinterpret_cast<CUevent>(event), flags);
    if (outStatus) *outStatus = static_cast<int>(r);
    return r == CUDA_SUCCESS;
}

} // anonymous namespace

// -- Public entry point ----------------------------------------------------

void fill(IOptionalCuda& iface)
{
    iface.cudaAvailable     = impl_cudaAvailable;

    iface.ctxGetCurrent     = impl_ctxGetCurrent;
    iface.ctxPushCurrent    = impl_ctxPushCurrent;
    iface.ctxPopCurrent     = impl_ctxPopCurrent;
    iface.ctxGetDevice      = impl_ctxGetDevice;
    iface.ctxSynchronize    = impl_ctxSynchronize;

    iface.deviceGetCount    = impl_deviceGetCount;
    iface.deviceGet         = impl_deviceGet;

    iface.memAlloc          = impl_memAlloc;
    iface.memFree           = impl_memFree;
    iface.memcpyHtoD        = impl_memcpyHtoD;
    iface.memcpyDtoH        = impl_memcpyDtoH;
    iface.memsetD8          = impl_memsetD8;
    iface.memsetD32         = impl_memsetD32;

    iface.eventCreate       = impl_eventCreate;
    iface.eventDestroy      = impl_eventDestroy;
    iface.eventRecord       = impl_eventRecord;
    iface.eventSynchronize  = impl_eventSynchronize;

    iface.streamCreate      = impl_streamCreate;
    iface.streamDestroy     = impl_streamDestroy;
    iface.streamSynchronize = impl_streamSynchronize;
    iface.streamWaitEvent   = impl_streamWaitEvent;

    iface.memcpy2DDeviceToDevice = impl_memcpy2DDeviceToDevice;
}

bool isCudaAvailable()
{
    return ensureInit();
}

int selectBestPhysicsDevice()
{
    int count = 0;
    if (!g_fns.cuDeviceGetCount || g_fns.cuDeviceGetCount(&count) != CUDA_SUCCESS || count <= 0)
        return -1;

    // If another component (e.g. the renderer) already made a context current on
    // this thread, colocate physics on the same device to avoid cross-device copies.
    {
        CUcontext ctx = nullptr;
        if (g_fns.cuCtxGetCurrent(&ctx) == CUDA_SUCCESS && ctx)
        {
            CUdevice dev = 0;
            if (g_fns.cuCtxGetDevice(&dev) == CUDA_SUCCESS)
            {
                CARB_LOG_INFO("PhysXFoundation: %d CUDA device(s) available, selecting device %d (active context).",
                              count, static_cast<int>(dev));
                return static_cast<int>(dev);
            }
        }
    }

    // No active context: rank discrete GPUs by memory and pick the largest,
    // matching the heuristic in CudaHelpers::selectBestPhysicsDevice.
    if (g_fns.cuDeviceGet && g_fns.cuDeviceTotalMem_v2 && g_fns.cuDeviceGetAttribute)
    {
        int bestOrdinal = -1;
        size_t bestMem = 0;

        for (int i = 0; i < count; ++i)
        {
            CUdevice dev;
            if (g_fns.cuDeviceGet(&dev, i) != CUDA_SUCCESS)
                continue;

            int integrated = 0;
            g_fns.cuDeviceGetAttribute(&integrated, CU_DEVICE_ATTRIBUTE_INTEGRATED, dev);
            if (integrated)
                continue;

            size_t mem = 0;
            if (g_fns.cuDeviceTotalMem_v2(&mem, dev) == CUDA_SUCCESS && mem > bestMem)
            {
                bestMem = mem;
                bestOrdinal = i;
            }
        }

        if (bestOrdinal >= 0)
        {
            CARB_LOG_INFO("PhysXFoundation: %d CUDA device(s) available, selecting device %d (%.0f MiB, discrete).",
                          count, bestOrdinal, static_cast<double>(bestMem) / (1024.0 * 1024.0));
            return bestOrdinal;
        }

        CARB_LOG_INFO("PhysXFoundation: %d CUDA device(s) available, selecting device 0 (only integrated GPUs present).", count);
        return 0;
    }

    CARB_LOG_INFO("PhysXFoundation: %d CUDA device(s) available, selecting device 0 (fallback).", count);
    return 0;
}

CUresult cuCtxGetCurrent_(CUcontext* ctx)
{
    ensureInit();
    if (!g_fns.cuCtxGetCurrent)
        return CUDA_ERROR_NOT_INITIALIZED;
    return g_fns.cuCtxGetCurrent(ctx);
}

CUresult cuCtxSetCurrent_(CUcontext ctx)
{
    ensureInit();
    if (!g_fns.cuCtxSetCurrent)
        return CUDA_ERROR_NOT_INITIALIZED;
    return g_fns.cuCtxSetCurrent(ctx);
}

CUresult cuDeviceGetCount_(int* count)
{
    ensureInit();
    if (!g_fns.cuDeviceGetCount)
        return CUDA_ERROR_NOT_INITIALIZED;
    return g_fns.cuDeviceGetCount(count);
}

bool setDevice(int ordinal)
{
    // Equivalent to cudaSetDevice for our needs (activates the primary context for
    // ordinal on the current thread).  Note: the primary context is retained exactly
    // once per ordinal per process and we intentionally do not release — the context
    // is expected to live for the process lifetime.  Subsequent calls for the same
    // ordinal reuse the cached handle and only call cuCtxSetCurrent.
    if (!ensureInit()) return false;

    static std::mutex s_retainMutex;
    static std::unordered_map<int, CUcontext> s_retainedCtx;

    CUcontext ctx = nullptr;
    CUdevice dev = 0;
    bool freshRetain = false;

    {
        std::lock_guard<std::mutex> lock(s_retainMutex);
        auto it = s_retainedCtx.find(ordinal);
        if (it != s_retainedCtx.end())
        {
            ctx = it->second;
        }
        else
        {
            if (g_fns.cuDeviceGet(&dev, ordinal) != CUDA_SUCCESS)
                return false;
            if (g_fns.cuDevicePrimaryCtxRetain(&ctx, dev) == CUDA_SUCCESS && ctx)
            {
                s_retainedCtx[ordinal] = ctx;
                freshRetain = true;
                CARB_LOG_INFO("PhysXFoundation: Retained primary CUDA context %p for device ordinal %d.", ctx, ordinal);
            }
        }
    }

    if (!ctx)
        return false;

    CUresult setResult = g_fns.cuCtxSetCurrent(ctx);
    if (setResult != CUDA_SUCCESS)
    {
        CARB_LOG_ERROR("PhysXFoundation: cuCtxSetCurrent(%p) failed for device ordinal %d (CUresult %d).",
                       ctx, ordinal, static_cast<int>(setResult));
        if (freshRetain)
        {
            // Undo the fresh retain so we don't leave a dangling refcount and the
            // next call can retry.
            std::lock_guard<std::mutex> lock(s_retainMutex);
            s_retainedCtx.erase(ordinal);
            g_fns.cuDevicePrimaryCtxRelease(dev);
        }
        return false;
    }

    CARB_LOG_VERBOSE("PhysXFoundation: Activated primary CUDA context %p for device ordinal %d.", ctx, ordinal);
    return true;
}

} // namespace cudaShim
} // namespace physx
} // namespace omni
