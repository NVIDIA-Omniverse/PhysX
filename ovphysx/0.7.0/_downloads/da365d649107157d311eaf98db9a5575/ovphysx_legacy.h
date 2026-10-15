// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @cond OVPHYSX_PLC_TRACE */
/**
 * @implements REQ-CAPI-LEGACY-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5
 * @implements REQ-CAPI-OMNIPVD-001
 * @covers AC-2
 * @implements REQ-CAPI-SCENEQUERY-001
 * @covers AC-1 AC-2 AC-3 AC-4
 * @implements REQ-CAPI-CONTACT-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7
 * @implements REQ-CAPI-CONTACT-003
 * @covers AC-1 AC-2 AC-3 AC-4
 * @implements REQ-CAPI-CONTACT-002
 * @covers AC-1 AC-2 AC-3 AC-4
 * @implements REQ-CAPI-BINDING-DEVICE-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5
 * @implements REQ-CAPI-OBJECTTYPE-001
 * @covers AC-2 AC-3 AC-4
 * @implements REQ-CAPI-SDFVIEW-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5
 * @implements REQ-CAPI-BINDING-STALE-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6
 * @implements REQ-CAPI-BINDING-SELECTION-001
 * @covers AC-2 AC-3
 * @implements REQ-CAPI-PATTERN-002
 * @covers AC-8
 * @implements REQ-CAPI-SDF-001
 * @covers AC-3
 */
/** @endcond */

#ifndef OVPHYSX_LEGACY_H
#define OVPHYSX_LEGACY_H

#include "ovphysx/ovphysx_export.h"
#include "ovphysx/ovphysx_types.h"
#include "ovphysx/ovphysx_legacy_types.h"

#ifdef __cplusplus
extern "C" {
#endif

    /**
     * @defgroup ovphysx_tensor_binding Tensor bindings (deprecated)
     * Bulk tensor access APIs for simulation data.
     *
     * @deprecated Superseded by the session read/write API (@ref ovphysx_read /
     *   @ref ovphysx_write, Python `PhysX.read` / `PhysX.write`). Retained for the
     *   deprecation window. Write new code against the session API.
     *   Include this header explicitly from new code. `ovphysx.h` still includes
     *   it for existing consumers.
     * @ingroup ovphysx_deprecated
     */

    /**
     * @defgroup ovphysx_deprecated Deprecated APIs
     * Legacy API surface maintained for compatibility.
     */

    /*--------------------------------------------------*/
    /* Legacy creation and configuration               */
    /*--------------------------------------------------*/

    /**
     * @deprecated active_cuda_gpus uses ovphysx_create_instance() then
     * ovphysx_set_config(). Other scoped keys use ovphysx_set_global_config()
     * before create, including OVPHYSX_GLOBAL_CONFIG_OVD_RECORDING_CAPABLE.
     * Carbonite tasking/TBB pool sizes stay on this create-args path.
     * Retired OVD configuration entries return OVPHYSX_API_INVALID_ARGUMENT.
     */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG(
        "active_cuda_gpus: create_instance then set_config; scoped keys: set_global_config; tasking/TBB pool sizes stay on create args")
    ovphysx_result_t ovphysx_create_instance_with_args(
        const ovphysx_create_args* create_args,
        ovphysx_instance_handle_t* out_handle);

    /**
     * @deprecated Keys that have a scoped replacement use ovphysx_set_global_config().
     * OVD recording capability uses OVPHYSX_GLOBAL_CONFIG_OVD_RECORDING_CAPABLE.
     * Carbonite tasking/TBB pool sizes stay on this entry.
     * Retired OVD configuration entries return OVPHYSX_API_INVALID_ARGUMENT.
     */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG(
        "use ovphysx_set_global_config for scoped keys; tasking/TBB pool sizes stay here")
    ovphysx_result_t ovphysx_set_global_config_entry(ovphysx_config_entry_t entry);

    /**
     * @deprecated Use ovphysx_get_global_config().
     * Retired OVD slots return OVPHYSX_API_INVALID_ARGUMENT with migration guidance.
     */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG("use ovphysx_get_global_config")
    ovphysx_result_t ovphysx_get_global_config_bool(ovphysx_config_bool_t key, bool* out_value);

    /**
     * @deprecated Use ovphysx_get_global_config().
     * Retired OVD slots return OVPHYSX_API_INVALID_ARGUMENT with migration guidance.
     */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG("use ovphysx_get_global_config")
    ovphysx_result_t ovphysx_get_global_config_int32(ovphysx_config_int32_t key, int32_t* out_value);

    /** @deprecated Use ovphysx_get_global_config(). */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG("use ovphysx_get_global_config")
    ovphysx_result_t ovphysx_get_global_config_float(ovphysx_config_float_t key, float* out_value);

    /**
     * @deprecated Use ovphysx_get_global_config().
     * Retired OVD slots return OVPHYSX_API_INVALID_ARGUMENT with migration guidance.
     */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG("use ovphysx_get_global_config")
    ovphysx_result_t ovphysx_get_global_config_string(
        ovphysx_config_string_t key,
        ovphysx_string_t* value_out,
        size_t* out_required_size);

    /** @deprecated Use OVPHYSX_GLOBAL_CONFIG_CPU_ONLY with ovphysx_set_global_config(). */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG("use ovphysx_set_global_config")
    ovphysx_result_t ovphysx_set_cpu_mode(bool cpu_only);

    /** @deprecated Use OVPHYSX_GLOBAL_CONFIG_CPU_ONLY with ovphysx_get_global_config(). */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG("use ovphysx_get_global_config")
    ovphysx_result_t ovphysx_get_cpu_mode(bool* out_cpu_only);

    /** @deprecated Use OVPHYSX_GLOBAL_CONFIG_LOG_LEVEL with ovphysx_set_global_config(). */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG("use ovphysx_set_global_config")
    ovphysx_result_t ovphysx_set_log_level(uint32_t level);

    /** @deprecated Use OVPHYSX_GLOBAL_CONFIG_LOG_LEVEL with ovphysx_get_global_config(). */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG("use ovphysx_get_global_config")
    uint32_t ovphysx_get_log_level(void);

    /** @deprecated Use OVPHYSX_GLOBAL_CONFIG_DEFAULT_LOG_OUTPUT with ovphysx_set_global_config(). */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG("use ovphysx_set_global_config")
    ovphysx_result_t ovphysx_enable_default_log_output(bool enable);

    /*--------------------------------------------------*/
    /* Tensor Binding API                                */
    /*--------------------------------------------------*/

    /** @addtogroup ovphysx_tensor_binding */
    /** @{ */

    /*
     * DEPRECATED (ovphysx 0.6): the Tensor Binding API is superseded by the session
     * read/write API (ovphysx_read / ovphysx_write, Python PhysX.read / PhysX.write). It
     * remains available through the deprecation window. Write new code against the session
     * API. The description below documents the legacy behavior for maintaining existing
     * tensor-binding code.
     *
     * The Tensor Binding API provides efficient bulk access to physics simulation data.
     * It maps physics-object path patterns to typed tensor views, including authored USD objects
     * and runtime-only clones. This enables same-device zero-copy data exchange and, for binding
     * types whose storage follows the simulation device, transparent cross-device staging with
     * tensors (PyTorch, NumPy, etc.) via DLPack.
     *
     * DLPack compatibility:
     *   The official DLPack header is vendored (see dlpack/dlpack.h from github.com/dmlc/dlpack).
     *   When constructing DLTensor structs for read/write calls:
     *   - strides may be NULL to indicate C-contiguous layout, OR
     *     explicit C-contiguous strides may be set. Both are accepted.
     *   - Non-contiguous tensors (e.g. transposed views, sliced views with gaps) are
     *     NOT supported and are rejected. Calling .contiguous() on a PyTorch tensor
     *     first produces an accepted layout.
     *   - Supported dtypes: float32, int32, uint32, uint8, bool (kDLBool bits=8).
     *   - Supported devices: kDLCPU, kDLCUDAHost, kDLCUDA, and kDLCUDAManaged, subject to the
     *     binding type and process policy. When CUDA is available, CPU/CUDA mismatches are staged
     *     only for binding types whose storage follows the simulation device.
     *   - CPU-only property bindings cover standalone rigid-body mass/inertia/COM values,
     *     articulation DOF/body properties, rigid-body/articulation shape properties,
     *     deformable-material properties, and disable-simulation/gravity flags. Fixed and spatial
     *     tendon property bindings are not CPU-only.
     *   - CPU-only property bindings require host-resident kDLCPU or kDLCUDAHost destination,
     *     source, index, and mask tensors even when the simulation runs on GPU. kDLCUDA and
     *     kDLCUDAManaged tensors are rejected rather than staged.
     *
     * Works in both GPU mode and CPU mode.
     * Process-wide CPU-only mode (ovphysx_set_cpu_mode(true) or OVPHYSX_DISABLE_GPU is set)
     * rejects kDLCUDA and kDLCUDAManaged tensors with OVPHYSX_API_DEVICE_MISMATCH
     * before accessing CUDA. kDLCPU and kDLCUDAHost remain host-accessible.
     *
     * GPU MODE NOTE: GPU tensor reads require at least one simulation step() after loading USD.
     * This is a PhysX DirectGPU API requirement. Without a warmup step, reads may fail
     * or return stale data. CPU mode does not have this requirement.
     *
     * Typical workflow (GPU mode):
     *   1. ovphysx_create_instance() (GPU is the default. Call ovphysx_set_cpu_mode(true)
     *      before creating any instance for process-wide CPU-only mode.)
     *   2. ovphysx_attach_ovstage() at the initial ordinal, then wait for its task.
     *      Use ovphysx_update_from_ovstage() only for later authored ordinals
     *      and wait for its task before dependent operations.
     *   3. ovphysx_create_tensor_binding() for each tensor type needed.
     *   4. Optional: ovphysx_warmup() to control when warmup happens
     *      (otherwise it happens automatically on the first tensor read).
     *   5. Main loop: read/write tensors, step, wait.
     *
     * Pattern matching:
     *   Patterns select realized physics objects by path, one path level per '/'-separated
     *   component. The full rules are on the "Path patterns" page of the ovphysx
     *   documentation (docs/read_write/path_patterns.md). In short:
     *   - Exact path: "/World/robot" selects that object when it exists. When it does not,
     *     the text is matched as a pattern, under the last-component rule below.
     *   - Within one name: '*' matches any run of characters, '?' one character, "[0-9]" one
     *     character from a class, and "a|b" either whole name. '.' is a literal dot.
     *   - "**" as a whole component matches zero or more levels.
     *   - A named or glob LAST component is searched at any depth below what the earlier
     *     components matched; the earlier components match one level each, and a bare '*'
     *     last component matches direct children only.
     *   - "[N]" is not an index placeholder: it matches the letter N. Use "[0-9]" or '*'.
     *   A single path component (the text between two slashes; a parenthesized group
     *   counts as one component even if it contains a slash) may be at most 4096
     *   characters long; a longer component is rejected with OVPHYSX_API_INVALID_ARGUMENT.
     */

    /**
     * Create a tensor binding for bulk data access (synchronous).
     *
     * A tensor binding connects a physics-object path pattern (e.g., "/World/robot*") to a
     * tensor type (e.g., OVPHYSX_TENSOR_RIGID_BODY_POSE_F32), enabling efficient
     * bulk read/write of physics data for all matching objects. Runtime-only clone
     * paths are eligible even when no USD prim is authored at the path.
     * Each call returns an independent handle. Destroying one binding does not
     * invalidate another binding for the same objects. A new binding selects the
     * currently eligible realized objects even while older bindings remain alive.
     *
     * If the pattern matches zero physics objects, the binding is still created successfully
     * with element_count = 0. This lets callers treat optional scene content as
     * an empty current result instead of an error. Empty bindings do not update
     * when matching physics objects are added or recreated. Destroy the old binding and
     * create a new one after topology changes.
     *
     * Binding lifetime is tied to the currently realized physics objects. The
     * application owns the stage lifecycle: if it will call ovphysx_detach_ovstage(),
     * remove USD data containing bound objects, or otherwise replace/reparse the
     * stage so those objects are destroyed and recreated, cached bindings should
     * be destroyed before the lifecycle operation when practical. If a stale
     * binding survives, only destroy it. Do not read or write through it. Create
     * replacement bindings after the operation completes. ovphysx_step() and
     * ovphysx_step_sync() do not invalidate bindings.
     *
     * @param handle Instance handle
     * @param desc Binding descriptor with pattern and tensor_type
     * @param out_binding_handle [out] Binding handle on success
     * @return ovphysx_result_t (synchronous - completes before returning)
     *
     * @pre handle, desc, and out_binding_handle must be valid.
     * @post Binding handle retains native resources until explicitly destroyed via
     *       ovphysx_destroy_tensor_binding(), or until the parent instance is
     *       destroyed. Stage detach or bound-object removal invalidates the
     *       underlying TensorAPI view. Destroy stale bindings and create
     *       replacements after the lifecycle operation completes.
     *
     * @par Diagnostics
     * Pattern bindings quiet expected TensorAPI no-match diagnostics on the
     * simulation view used to create that binding. Explicit prim_paths keep the
     * default error-level no-match diagnostics for typo detection. For
     * programmatic partial-miss checks with explicit prim_paths, compare the
     * requested paths with ovphysx_tensor_binding_get_prim_paths().
     *
     * @par Threading
     * Do not create bindings concurrently with stage mutation.
     *
     * @par Side Effects
     * Allocates internal binding resources.
     *
     * @par Errors
     * - OVPHYSX_API_INVALID_ARGUMENT for invalid inputs
     * - OVPHYSX_API_ERROR for internal failures
     *
     * @code
     * #include <ovphysx/ovphysx_legacy.h>
     *
     * static ovphysx_result_t create_pose_binding(
     *     ovphysx_instance_handle_t handle,
     *     ovphysx_tensor_binding_handle_t* out_binding)
     * {
     *     ovphysx_tensor_binding_desc_t desc = {
     *         .pattern = OVPHYSX_LITERAL("/World/robot*"),
     *         .tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32,
     *     };
     *     return ovphysx_create_tensor_binding(handle, &desc, out_binding);
     * }
     * @endcode
     *
     * @deprecated The tensor-binding API is deprecated. Use @ref ovphysx_read
     *   (reads) and @ref ovphysx_write (writes) instead.
     */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG(
        "ovphysx tensor bindings are deprecated; use ovphysx_read (reads) / ovphysx_write (writes)")
    ovphysx_result_t ovphysx_create_tensor_binding(
        ovphysx_instance_handle_t handle,
        const ovphysx_tensor_binding_desc_t* desc,
        ovphysx_tensor_binding_handle_t* out_binding_handle);

    /**
     * Destroy a tensor binding and release associated resources (synchronous).
     *
     * @param handle Instance handle
     * @param binding_handle Binding to destroy
     * @return ovphysx_result_t
     *
     * @pre handle and binding_handle must be valid.
     * @post Binding handle is invalid after call.
     *
     * @par Side Effects
     * Releases internal resources.
     *
     * @par Errors
     * - OVPHYSX_API_NOT_FOUND if binding handle is unknown
     * - OVPHYSX_API_ERROR for internal failures
     *
     * @code{.c}
     * #include <ovphysx/ovphysx_legacy.h>
     *
     * static ovphysx_result_t destroy_binding(
     *     ovphysx_instance_handle_t handle,
     *     ovphysx_tensor_binding_handle_t binding)
     * {
     *     return ovphysx_destroy_tensor_binding(handle, binding);
     * }
     * @endcode
     *
     * @deprecated The tensor-binding API is deprecated. Use @ref ovphysx_read
     *   (reads) and @ref ovphysx_write (writes) instead.
     */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG(
        "ovphysx tensor bindings are deprecated; use ovphysx_read (reads) / ovphysx_write (writes)")
    ovphysx_result_t ovphysx_destroy_tensor_binding(
        ovphysx_instance_handle_t handle,
        ovphysx_tensor_binding_handle_t binding_handle);

    /**
     * Get the tensor layout specification for a binding.
     *
     * Returns dtype, ndim, and shape needed to allocate a compatible DLTensor.
     * This is the preferred API for constructing DLTensors correctly.
     *
     * NOTE: ovphysx_tensor_spec_t stores shape in a fixed-size int64[4] for a stable C ABI.
     * Only the first ndim entries are meaningful. The remaining entries are always set to 0.
     *
     * See ovphysx_tensor_type_t documentation for shapes, dtype, and layouts per tensor type.
     * Layout is always row-major contiguous (C-order). Most bindings are float32.
     * OVPHYSX_TENSOR_DEFORMABLE_SIM_ELEMENT_INDICES_S32 is int32.
     *
     * @note The returned specification does not include memory residency. Call
     * ovphysx_get_tensor_binding_native_device() to query the binding's native device.
     *
     * @param handle Instance handle
     * @param binding_handle Tensor binding
     * @param out_spec [out] Tensor dtype, rank, and shape
     * @return ovphysx_result_t
     *
     * @pre handle, binding_handle, and out_spec must be valid.
     * @post out_spec is populated with dtype/shape for the binding.
     *
     * @par Side Effects
     * None.
     *
     * @par Errors
     * - OVPHYSX_API_NOT_FOUND if binding handle is unknown
     * - OVPHYSX_API_ERROR for internal failures
     *
     * @code{.c}
     * #include <ovphysx/ovphysx_legacy.h>
     *
     * static ovphysx_result_t query_binding_spec(
     *     ovphysx_instance_handle_t handle,
     *     ovphysx_tensor_binding_handle_t binding,
     *     ovphysx_tensor_spec_t* out_spec)
     * {
     *     return ovphysx_get_tensor_binding_spec(handle, binding, out_spec);
     * }
     * @endcode
     *
     * @deprecated The tensor-binding API is deprecated. Use @ref ovphysx_read
     *   (reads) and @ref ovphysx_write (writes) instead.
     */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG(
        "ovphysx tensor bindings are deprecated; use ovphysx_read (reads) / ovphysx_write (writes)")
    ovphysx_result_t ovphysx_get_tensor_binding_spec(
        ovphysx_instance_handle_t handle,
        ovphysx_tensor_binding_handle_t binding_handle,
        ovphysx_tensor_spec_t* out_spec);

    /**
     * Get the native device used by a tensor binding.
     *
     * Returns the device used by the binding's native TensorAPI read/write path:
     * `DLDevice{kDLCPU, 0}` for host-resident bindings, or
     * `DLDevice{kDLCUDA, device_ordinal}` for CUDA-resident bindings. CPU-only
     * property tensors report CPU even when the simulation runs on CUDA.
     * For kDLCUDA, device_id is the process-visible CUDA runtime ordinal used
     * with cudaSetDevice() or a framework device such as `cuda:N`, not a physical
     * PCI bus index.
     *
     * This reports native residency, not every device accepted by read/write.
     * Using the native device avoids staging when another accepted placement
     * would require it. The value remains stable for the lifetime of a live
     * binding. Unlike the layout-only spec getter, this is a live mapping query
     * and rejects an invalidated simulation view.
     *
     * @param handle Instance handle
     * @param binding_handle Tensor binding
     * @param out_device [out] Native DLPack device for the binding
     * @return ovphysx_result_t
     *
     * @pre handle, binding_handle, and out_device must be valid.
     * @post out_device is populated with the binding's native device.
     *
     * @par Side Effects
     * None.
     *
     * @par Synchronization
     * Waits for pending operations on the instance before inspecting the binding.
     *
     * @par Errors
     * - OVPHYSX_API_INVALID_ARGUMENT if out_device is NULL
     * - OVPHYSX_API_NOT_FOUND if binding_handle is unknown or stale
     * - OVPHYSX_API_ERROR if handle is invalid
     *
     * @code{.c}
     * #include <ovphysx/ovphysx_legacy.h>
     *
     * static ovphysx_result_t query_binding_device(
     *     ovphysx_instance_handle_t handle,
     *     ovphysx_tensor_binding_handle_t binding,
     *     DLDevice* out_device)
     * {
     *     return ovphysx_get_tensor_binding_native_device(handle, binding, out_device);
     * }
     * @endcode
     *
     * @deprecated The tensor-binding API is deprecated. Use @ref ovphysx_read / @ref ovphysx_write instead.
     */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG(
        "ovphysx tensor bindings are deprecated; use ovphysx_read (reads) / ovphysx_write (writes)")
    ovphysx_result_t ovphysx_get_tensor_binding_native_device(
        ovphysx_instance_handle_t handle,
        ovphysx_tensor_binding_handle_t binding_handle,
        DLDevice* out_device);

    /**
     * @anchor ovphysx_tensor_auto_warmup_note
     * @par Tensor auto-warmup note
     * The first tensor read or write after loading USD may perform an automatic
     * warmup simulation step to initialize PhysX lazy structures. In GPU mode
     * this also initializes DirectGPU buffers.
     *
     * The warmup is a real physics step that advances simulation time by a minimal
     * timestep (~1ns). Physics state may change infinitesimally. This is not a dry run.
     *
     * For deterministic behavior, explicitly control warmup timing by loading USD,
     * waiting for completion, then calling ovphysx_warmup() before the first
     * tensor read or write. To have the first observed state change happen under
     * a chosen timestep instead, call ovphysx_step() explicitly with that dt.
     *
     * Because warmup is a real simulation step, a true "pre-warmup" tensor state
     * cannot be observed. Calling ovphysx_warmup() explicitly only makes the timing
     * of that unavoidable step predictable.
     */

    /**
     * Read data from simulation into a user-provided DLTensor (synchronous).
     *
     * NOTE: On the first tensor read after loading USD, an automatic warmup simulation
     * step may be performed. See @ref ovphysx_tensor_auto_warmup_note "tensor auto-warmup note".
     *
     * DLTensor requirements:
     *   - MUST be pre-allocated with correct shape (use ovphysx_get_tensor_binding_spec())
     *   - dtype must match ovphysx_get_tensor_binding_spec()
     *   - when CUDA is available, CPU/CUDA mismatches are staged for binding types whose storage
     *     follows the simulation device
     *   - CPU-only property bindings require a host-resident kDLCPU or kDLCUDAHost destination.
     *     kDLCUDA and kDLCUDAManaged destinations return OVPHYSX_API_DEVICE_MISMATCH and are not
     *     staged
     *   - cross-GPU ordinal mismatches and CUDA tensors in process-wide CPU-only mode return
     *     OVPHYSX_API_DEVICE_MISMATCH
     *   - layout must be contiguous row-major (C-order)
     *
     * This is a blocking call that completes before returning.
     *
     * @param handle Instance handle
     * @param binding_handle Tensor binding
     * @param dst_tensor Pre-allocated DLTensor with shape from ovphysx_get_tensor_binding_spec()
     * @return ovphysx_result_t
     *
     * @pre handle and binding_handle must be valid.
     * @pre dst_tensor must be pre-allocated, match the spec's dtype/shape, and use a supported device.
     * @post dst_tensor is filled with simulation data on success.
     *
     * @par Side Effects
     * May trigger warmup on first read.
     *
     * @par Ownership
     * Caller owns dst_tensor memory.
     *
     * @par Errors
     * - OVPHYSX_API_DEVICE_MISMATCH if tensor device is incompatible
     * - OVPHYSX_API_INVALID_ARGUMENT for invalid inputs
     * - OVPHYSX_API_NOT_FOUND if the binding is unknown or was invalidated by a stage change
     * - OVPHYSX_API_ERROR for internal failures
     *
     * @code{.c}
     * #include <ovphysx/ovphysx_legacy.h>
     *
     * static ovphysx_result_t read_binding(
     *     ovphysx_instance_handle_t handle,
     *     ovphysx_tensor_binding_handle_t binding,
     *     DLTensor* destination)
     * {
     *     return ovphysx_read_tensor_binding(handle, binding, destination);
     * }
     * @endcode
     *
     * @deprecated The tensor-binding API is deprecated. Use @ref ovphysx_read instead.
     */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG(
        "ovphysx tensor bindings are deprecated; use ovphysx_read")
    ovphysx_result_t ovphysx_read_tensor_binding(
        ovphysx_instance_handle_t handle,
        ovphysx_tensor_binding_handle_t binding_handle,
        DLTensor* dst_tensor);

    /** @} */

    /** @addtogroup ovphysx_tensor_binding */
    /** @{ */

    /**
     * Write data from a user-provided DLTensor into the simulation (synchronous).
     *
     * Not all tensor types are writable:
     * - RIGID_BODY_FORCE_F32, RIGID_BODY_WRENCH_F32, ARTICULATION_LINK_WRENCH_F32 are WRITE-ONLY
     *   (external control inputs applied each step, so reading them returns an error).
     * - RIGID_BODY_ACCELERATION_F32, RIGID_BODY_INV_MASS_F32, RIGID_BODY_INV_INERTIA_F32
     *   are READ-ONLY.
     * - ARTICULATION_LINK_POSE_F32, ARTICULATION_LINK_VELOCITY_F32, ARTICULATION_LINK_ACCELERATION_F32
     *   are READ-ONLY (no setter for individual link state).
     * - Inverse dynamics query tensors (JACOBIAN, MASS_MATRIX, CORIOLIS_AND_CENTRIFUGAL_FORCE, GRAVITY_FORCE,
     *   LINK_INCOMING_JOINT_FORCE, DOF_PROJECTED_JOINT_FORCE, BODY_INV_MASS, BODY_INV_INERTIA)
     *   are READ-ONLY.
     * - DEFORMABLE_REST_NODAL_POSITION_F32 and DEFORMABLE_SIM_ELEMENT_INDICES_S32
     *   are READ-ONLY.
     * - DOF_ACTUATION_FORCE_F32 is read-write (not write-only).
     * See ovphysx_tensor_type_t documentation for shapes, layouts, and read/write semantics.
     *
     * NOTE: On the first tensor write after loading USD, an automatic warmup
     * simulation step may be performed to initialize PhysX structures.
     * See @ref ovphysx_tensor_auto_warmup_note "tensor auto-warmup note".
     *
     * This is a blocking call that completes before returning.
     *
     * @param handle Instance handle
     * @param binding_handle Tensor binding
     * @param src_tensor User tensor with data to write (must match ovphysx_get_tensor_binding_spec())
     * @param index_tensor Optional int32[K] indices for subset write. NULL = write all.
     *   - When index_tensor != NULL: src_tensor must still have full shape [N, ...] matching the binding spec.
     *     Only the rows specified by index_tensor are written. Other rows in src_tensor are ignored.
     *   - Indices are 0-based into the first dimension N of the binding, and must satisfy 0 <= idx < N.
     *   - K (index count) must satisfy K <= N.
     * @return ovphysx_result_t
     *
     * @pre handle and binding_handle must be valid.
     * @pre src_tensor must match the spec's dtype/shape and use a supported device. When CUDA is
     *      available, CPU/CUDA mismatches are staged for binding types whose storage follows the
     *      simulation device.
     * @pre For CPU-only property bindings, src_tensor and any index_tensor must use kDLCPU or
     *      kDLCUDAHost. Otherwise-valid kDLCUDA and kDLCUDAManaged tensors return
     *      OVPHYSX_API_DEVICE_MISMATCH and are not staged.
     * @pre Cross-GPU ordinal mismatches and CUDA tensors in process-wide CPU-only mode return
     *      OVPHYSX_API_DEVICE_MISMATCH.
     * @post Simulation state is updated with new values.
     *
     * @par Side Effects
     * Writes control or state data into the simulation.
     *
     * @par Ownership
     * Caller owns src_tensor and index_tensor memory.
     *
     * @par Errors
     * - OVPHYSX_API_DEVICE_MISMATCH if tensor device is incompatible
     * - OVPHYSX_API_INVALID_ARGUMENT for invalid inputs
     * - OVPHYSX_API_NOT_FOUND if the binding is unknown or was invalidated by a stage change
     * - OVPHYSX_API_ERROR for internal failures
     *
     * @code{.c}
     * #include <ovphysx/ovphysx_legacy.h>
     *
     * static ovphysx_result_t write_binding(
     *     ovphysx_instance_handle_t handle,
     *     ovphysx_tensor_binding_handle_t binding,
     *     const DLTensor* source)
     * {
     *     return ovphysx_write_tensor_binding(handle, binding, source, NULL);
     * }
     * @endcode
     *
     * @deprecated The tensor-binding API is deprecated. Use @ref ovphysx_write instead.
     */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG(
        "ovphysx tensor bindings are deprecated; use ovphysx_write")
    ovphysx_result_t ovphysx_write_tensor_binding(
        ovphysx_instance_handle_t handle,
        ovphysx_tensor_binding_handle_t binding_handle,
        const DLTensor* src_tensor,
        const DLTensor* index_tensor);

    /**
     * Write data from a user-provided DLTensor into the simulation using a binary mask (synchronous).
     *
     * Only elements where mask[i] != 0 are written. Other elements are left unchanged.
     * This is the mask-based alternative to indexed writes via ovphysx_write_tensor_binding.
     *
     * NOTE: On the first tensor write after loading USD, an automatic warmup
     * simulation step may be performed to initialize PhysX structures.
     * See @ref ovphysx_tensor_auto_warmup_note "tensor auto-warmup note".
     *
     * @note There is intentionally no corresponding read_masked function. Reads always return
     *   the full [N,...] tensor via ovphysx_read_tensor_binding(). Callers that need a subset
     *   can index the result on the host/device side. This write-only mask design matches
     *   other reinforcement-learning physics APIs (e.g. Newton's selectionAPI) where masks
     *   are used to selectively apply actions but observations are always returned in full.
     *
     * This is a blocking call that completes before returning.
     *
     * @param handle Instance handle
     * @param binding_handle Tensor binding
     * @param src_tensor User tensor with data to write. Must be full shape [N, ...] matching
     *   the dtype and shape from ovphysx_get_tensor_binding_spec().
     * @param mask_tensor Binary mask selecting which elements to update. Must be 1D with shape [N]
     *   where N matches the binding's first dimension. Dtype must be bool (kDLBool, bits=8) or
     *   uint8 (kDLUInt, bits=8).
     * @return ovphysx_result_t
     *
     * @pre handle and binding_handle must be valid.
     * @pre src_tensor must match the dtype/shape of the binding spec and use a supported device.
     * @pre mask_tensor must be 1D uint8/bool with length N on a supported device. When CUDA is
     *      available, CPU/CUDA mismatches are staged for binding types whose storage follows the
     *      simulation device.
     * @pre For CPU-only property bindings, src_tensor and mask_tensor must use kDLCPU or
     *      kDLCUDAHost. Otherwise-valid kDLCUDA and kDLCUDAManaged tensors return
     *      OVPHYSX_API_DEVICE_MISMATCH and are not staged.
     * @pre Cross-GPU ordinal mismatches and CUDA tensors in process-wide CPU-only mode return
     *      OVPHYSX_API_DEVICE_MISMATCH.
     * @post Simulation state is updated for masked elements only.
     *
     * @par Side Effects
     * Writes control or state data into the simulation for selected elements.
     *
     * @par Ownership
     * Caller owns src_tensor and mask_tensor memory.
     *
     * @par Errors
     * - OVPHYSX_API_DEVICE_MISMATCH if tensor device is incompatible
     * - OVPHYSX_API_INVALID_ARGUMENT for invalid inputs
     * - OVPHYSX_API_NOT_FOUND if the binding is unknown or was invalidated by a stage change
     * - OVPHYSX_API_ERROR for internal failures
     *
     * @code{.c}
     * #include <ovphysx/ovphysx_legacy.h>
     *
     * static ovphysx_result_t write_masked_binding(
     *     ovphysx_instance_handle_t handle,
     *     ovphysx_tensor_binding_handle_t binding,
     *     const DLTensor* source,
     *     const DLTensor* mask)
     * {
     *     return ovphysx_write_tensor_binding_masked(
     *         handle, binding, source, mask);
     * }
     * @endcode
     *
     * @deprecated The tensor-binding API is deprecated. Use @ref ovphysx_write instead.
     */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG(
        "ovphysx tensor bindings are deprecated; use ovphysx_write")
    ovphysx_result_t ovphysx_write_tensor_binding_masked(
        ovphysx_instance_handle_t handle,
        ovphysx_tensor_binding_handle_t binding_handle,
        const DLTensor* src_tensor,
        const DLTensor* mask_tensor);

    /** @} */

    /*--------------------------------------------------*/
    /* Articulation metadata queries                     */
    /*--------------------------------------------------*/

    /** @addtogroup ovphysx_tensor_binding */
    /** @{ */

    /**
     * @brief Get all scalar topology metadata for an articulation binding in one call.
     *
     * Fills out_metadata with dof_count, body_count, joint_count, fixed_tendon_count,
     * spatial_tendon_count, and is_fixed_base. All values are stable for the binding
     * lifetime, so the result can be cached.
     *
     * **Homogeneous topology requirement**: all articulations covered by this binding
     * must have **identical topology** -- not merely matching counts. Two articulations
     * share a topology only when they agree on base type (fixed vs floating), dof_count,
     * body_count, joint_count, joint types AND joint names, DOF parentage, and link leaf
     * names. In particular, two otherwise-identical articulations whose joints differ only
     * in name (including a difference only in length) are distinct topologies and will not
     * bind together.
     * This is a constraint of the native tensor backend, whose tensor shapes are fixed at
     * binding creation time. Articulations of different topology (e.g. a 7-DOF arm and a
     * 30-DOF humanoid, or two arms whose joints are named differently) need a separate
     * binding each.
     * A binding spanning heterogeneous articulations returns a non-SUCCESS status here with
     * all-zero counts -- the same signal as an empty binding -- while
     * ovphysx_get_tensor_binding_spec still reports the correct shape.
     *
     * For name arrays (DOF names, body names, joint names) use the corresponding
     * ovphysx_articulation_get_*_names functions.
     *
     * @param handle          Instance handle
     * @param binding_handle  Tensor binding (must be an articulation binding)
     * @param out_metadata    [out] Caller-allocated struct to fill
     * @return ovphysx_result_t
     *
     * @code{.c}
     * #include <ovphysx/ovphysx_legacy.h>
     * #include <stdio.h>
     *
     * static ovphysx_result_t print_articulation_size(
     *     ovphysx_instance_handle_t handle,
     *     ovphysx_tensor_binding_handle_t binding)
     * {
     *     ovphysx_articulation_metadata_t metadata = {0};
     *     ovphysx_result_t result =
     *         ovphysx_get_articulation_metadata(handle, binding, &metadata);
     *     if (result.status == OVPHYSX_API_SUCCESS)
     *         printf("DOFs: %d  Links: %d\n",
     *                metadata.dof_count, metadata.body_count);
     *     return result;
     * }
     * @endcode
     *
     * @deprecated Part of the tensor-binding surface. It requires a binding handle and is
     *   removed with the binding. Its successor is the non-binding read API: read the six
     *   whole-articulation topology attributes in ovphysx_attributes.h (OVPHYSX_ATTR_DOF_COUNT, OVPHYSX_ATTR_LINK_COUNT,
     *   OVPHYSX_ATTR_JOINT_COUNT, OVPHYSX_ATTR_FIXED_TENDON_COUNT, OVPHYSX_ATTR_SPATIAL_TENDON_COUNT,
     *   OVPHYSX_ATTR_IS_FIXED_BASE) via ovphysx_read() on OVPHYSX_QUERY_TARGET_ARTICULATION, which cover
     *   this struct field-for-field. The DOF/body/joint name getters below now also have non-binding
     *   successors -- see each one's note.
     */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG(
        "ovphysx tensor bindings are deprecated; read the whole-articulation topology attributes (dofCount, linkCount, jointCount, fixedTendonCount, spatialTendonCount, isFixedBase) via ovphysx_read() instead")
    ovphysx_result_t ovphysx_get_articulation_metadata(
        ovphysx_instance_handle_t handle,
        ovphysx_tensor_binding_handle_t binding_handle,
        ovphysx_articulation_metadata_t* out_metadata);

    /**
     * @brief Get DOF names for the articulation.
     *
     * String pointers remain valid until the binding is destroyed.
     *
     * @param handle Instance handle
     * @param binding_handle Tensor binding (must be an articulation binding)
     * @param out_names [out] Array of ovphysx_string_t to fill
     * @param max_names Capacity of out_names array. Set to metadata.dof_count
     *   (from ovphysx_get_articulation_metadata()) to receive all names.
     * @param out_count [out] Actual number of names written
     * @return ovphysx_result_t
     *
     * @deprecated Binding-coupled (requires a tensor-binding handle) and removed with the tensor
     *   binding. Per-DOF identity is now served without a binding by the
     *   `OVPHYSX_ATTR_DOF_JOINT_PATH` macro in ovphysx_attributes.h
     *   and `OVPHYSX_ATTR_DOF_AXIS_INDEX` read attributes on @ref ovphysx_read (one entry per DOF, in
     *   inverse-dynamics matrix column order): a DOF is identified by (owning joint prim, axis index)
     *   rather than a string, so an inverse-dynamics matrix column is joinable back to its joint.
     */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG(
        "ovphysx tensor bindings are deprecated; per-DOF identity is served by the dofJointPath/dofAxisIndex read attributes on ovphysx_read (a DOF is (joint prim, axis))")
    ovphysx_result_t ovphysx_articulation_get_dof_names(
        ovphysx_instance_handle_t handle,
        ovphysx_tensor_binding_handle_t binding_handle,
        ovphysx_string_t* out_names,
        uint32_t max_names,
        uint32_t* out_count);

    /**
     * @brief Get body (link) names for the articulation.
     *
     * String pointers remain valid until the binding is destroyed.
     *
     * @param handle Instance handle
     * @param binding_handle Tensor binding (must be an articulation binding)
     * @param out_names [out] Array of ovphysx_string_t to fill
     * @param max_names Capacity of out_names array. Set to metadata.body_count
     *   (from ovphysx_get_articulation_metadata()) to receive all names.
     * @param out_count [out] Actual number of names written
     * @return ovphysx_result_t
     *
     * @deprecated Binding-coupled (requires a tensor-binding handle) and removed with the tensor
     *   binding. Link names are now served without a binding as the leaf name of each
     *   `OVPHYSX_QUERY_TARGET_ARTICULATION_LINK` prim read by @ref ovphysx_read -- resolve the prim path
     *   through the query's path dictionary and take its leaf.
     */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG(
        "ovphysx tensor bindings are deprecated; link names are the leaf name of the articulation-link prims served by ovphysx_read")
    ovphysx_result_t ovphysx_articulation_get_body_names(
        ovphysx_instance_handle_t handle,
        ovphysx_tensor_binding_handle_t binding_handle,
        ovphysx_string_t* out_names,
        uint32_t max_names,
        uint32_t* out_count);

    /**
     * @brief Get joint names for the articulation.
     *
     * String pointers remain valid until the binding is destroyed.
     *
     * @param handle Instance handle
     * @param binding_handle Tensor binding (must be an articulation binding)
     * @param out_names [out] Array of ovphysx_string_t to fill
     * @param max_names Capacity of out_names array. Set to metadata.joint_count
     *   (from ovphysx_get_articulation_metadata()) to receive all names.
     * @param out_count [out] Actual number of names written
     * @return ovphysx_result_t
     *
     * @deprecated Binding-coupled (requires a tensor-binding handle) and removed with the tensor
     *   binding. For DOF-bearing joints, names are now served without a binding as the leaf name of each
     *   `OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT` prim read by @ref ovphysx_read -- resolve the prim path
     *   through the query's path dictionary and take its leaf. The
     *   `OVPHYSX_ATTR_DOF_JOINT_PATH` descriptor in ovphysx_attributes.h names
     *   the same joint prim per DOF column. A fully-locked (zero-DOF) joint has no read
     *   group, so its path is not among them; the deprecated getter listed those fixed joints too.
     */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG(
        "ovphysx tensor bindings are deprecated; joint names are the leaf name of the articulation-joint prims served by ovphysx_read")
    ovphysx_result_t ovphysx_articulation_get_joint_names(
        ovphysx_instance_handle_t handle,
        ovphysx_tensor_binding_handle_t binding_handle,
        ovphysx_string_t* out_names,
        uint32_t max_names,
        uint32_t* out_count);

    /**
     * @brief Get resolved physics-object paths for a tensor binding.
     *
     * The returned array order matches row order for every `RIGID_BODY_*`
     * tensor read/write on the same binding. For `ARTICULATION_*` tensor
     * bindings, the returned paths are articulation root object paths in the
     * binding's first-dimension row order. ovphysx owns the returned string
     * storage. String pointers remain valid until the binding is destroyed.
     *
     * @param handle Instance handle
     * @param binding_handle Tensor binding
     * @param out_paths [out] Array of ovphysx_string_t to fill
     * @param max_paths Capacity of out_paths array. Must be at least the binding
     *   count to receive all paths.
     * @param out_count [out] Actual number of paths written
     * @return ovphysx_result_t
     *
     * @deprecated The tensor-binding API is deprecated. Use @ref ovphysx_read
     *   (reads) and @ref ovphysx_write (writes) instead.
     */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG(
        "ovphysx tensor bindings are deprecated; use ovphysx_read (reads) / ovphysx_write (writes)")
    ovphysx_result_t ovphysx_tensor_binding_get_prim_paths(
        ovphysx_instance_handle_t handle,
        ovphysx_tensor_binding_handle_t binding_handle,
        ovphysx_string_t* out_paths,
        uint32_t max_paths,
        uint32_t* out_count);

    /** @} */

    /*--------------------------------------------------*/
    /* Contact binding API                               */
    /*--------------------------------------------------*/

    /**
     * @defgroup ovphysx_contact_binding Contact bindings
     *
     * Read-only APIs for querying **aggregate contact force tensors** between
     * sensor and filter bodies. Returns DLPack tensors shaped `[S, 3]` (net
     * forces) or `[S, F, 3]` (force matrix), suitable for RL rewards, safety
     * limits, or force monitoring. GPU-compatible.
     *
     * For tensorized **per-contact-point geometry** (position, normal,
     * separation, force, and friction), use ovphysx_read_normal_contact_data() and
     * ovphysx_read_friction_contact_data().
     *
     * A **sensor** is a set of rigid bodies identified by a physics-object path
     * pattern passed to ovphysx_create_contact_binding(). A **filter** is a
     * second set of bodies whose contacts with each sensor are measured. Both
     * patterns can resolve authored USD objects and runtime-only clones.
     *
     * Sensor selection is opt-in: every authored USD prim matched by a sensor
     * pattern must have `PhysxContactReportAPI` applied, on the prim named as the
     * sensor itself (not a parent body or child collider). A matched prim without
     * the schema is dropped from the binding, and if that leaves no sensors the
     * call fails. Filter prims need no extra schema. Runtime-only clones use the
     * schema metadata copied from the corresponding source object: a cloned
     * collider whose source collider lacks the schema is rejected even if its
     * parent body has it.
     *
     * On CPU simulation and GPU dynamics with CPU readback, a separate child collider's schema enables
     * sensor selection, not reporting for its owning body. Also enable reporting on the body, owning
     * articulation link, or articulation root to avoid depending on reports from the other actor in each
     * pair. If neither actor reports matching contacts, the child sensor reads zero. Configure reporting
     * settings on the reporting actor or articulation root; a standalone static collider is its own actor.
     *
     * In these modes, enable owner reporting but select only the child as a binding sensor. If the owner
     * is also selected as a sensor in this or another live contact binding on the same attachment, the
     * owner takes precedence: contacts are credited to the owner and the child reads zero. Check wildcard
     * patterns for unintended owner matches, and establish bindings before stepping. Session reads support
     * selecting both the owner and child in these modes.
     *
     * On DirectGPU, a separate child collider carrying PhysxContactReportAPI is accepted by the deprecated
     * contact binding, but the runtime logs an unsupported collider warning and the child reads zero force.
     * DirectGPU session queries omit that child row; select the owning body or articulation link instead.
     *
     * **Lifecycle and read timing**
     *
     * A contact binding must be created *before* the first simulation step whose
     * contacts are to be observed. No contact data exists until at least
     * one `ovphysx_step()` or `ovphysx_step_sync()`
     * call has completed.
     *
     * - Call contact binding read functions *after* a stepping call completes.
     * - Reads reflect the contacts accumulated during the **last completed
     *   simulation step**.
     * - On CPU and GPU dynamics with CPU readback, sleeping pairs stop producing
     *   persistent reports: forces can be zero and contacts empty while bodies
     *   remain touching. This is independent of environment IDs. For continuous
     *   reporting, disable sleeping before attachment, or wake the bodies and
     *   step again before reading. DirectGPU disables sleeping.
     * - Calling a read function before any simulation step returns all-zero
     *   output tensors (no contacts have been reported yet).
     * - The `dt` for impulse-to-force conversion (`force = impulse / dt`) is
     *   automatically taken from the last successful `ovphysx_step()`,
     *   `ovphysx_step_sync()` call that simulated. It is not
     *   passed explicitly.
     * - A step with `step_dt == 0` simulates nothing: it neither produces nor
     *   clears contacts, and it does not change the conversion `dt`. Reads after
     *   it return exactly what they returned before it -- the contacts and forces
     *   of the last step with a positive `dt`.
     *
     * **Why a separate binding type (not tensor binding)**
     *
     * Contact data has a fundamentally different shape and semantics from
     * articulation/rigid-body tensors:
     * - Shape is `[S, F, 3]` (sensor x filter x xyz), determined at binding
     *   creation time for aggregate force matrices, plus flat detailed contact
     *   buffers indexed by `[S, F]` count/start-index tensors.
     * - Contact binding is read-only. There is no write path.
     * - Internally it uses the PhysX contact-report callback on CPU/readback
     *   scenes and GPU contact buffers on DirectGPU scenes.
     */

    /** @addtogroup ovphysx_contact_binding */
    /** @{ */

    /**
     * @brief Create a contact binding for reading net contact forces and force matrices.
     *
     * @param handle Instance handle
     * @param sensor_patterns Array of physics-object path patterns matching sensor bodies.
     *   A single path component longer than 4096 characters (here or in filter_patterns)
     *   is rejected with OVPHYSX_API_INVALID_ARGUMENT.
     * @param sensor_patterns_count Number of sensor patterns
     * @param filter_patterns Flat array of filter object-path patterns. All sensors must
     *   have the same number of filters. Total length = sensor_patterns_count * filters_per_sensor.
     *   Pass NULL with filters_per_sensor=0 for unfiltered contacts.
     * @param filters_per_sensor Number of filter patterns per sensor (same for all sensors)
     * @param max_contact_data_count Max detailed contact/friction entries to track.
     *   Set this to a positive value before using ovphysx_read_normal_contact_data()
     *   or ovphysx_read_friction_contact_data(). Detailed reads also require
     *   filters_per_sensor > 0. Aggregate net-force reads do not need detailed
     *   contact capacity or filters.
     * @param out_handle [out] Contact binding handle
     * @return ovphysx_result_t
     *
     * @post Binding handle is valid until explicitly destroyed via ovphysx_destroy_contact_binding(),
     *       or until the parent instance is destroyed.
     *
     * @code
     * #include <ovphysx/ovphysx_legacy.h>
     *
     * static ovphysx_result_t create_filtered_contact_binding(
     *     ovphysx_instance_handle_t handle,
     *     ovphysx_contact_binding_handle_t* out_binding)
     * {
     *     ovphysx_string_t sensors[] = {
     *         ovphysx_cstr("/World/robot_0/ee"),
     *     };
     *     ovphysx_string_t filters[] = {
     *         ovphysx_cstr("/World/obstacles/box"),
     *     };
     *     return ovphysx_create_contact_binding(
     *         handle, sensors, 1, filters, 1, 256, out_binding);
     * }
     * @endcode
     */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG(
        "ovphysx contact bindings are deprecated; include <ovphysx/ovphysx_legacy.h>")
    ovphysx_result_t ovphysx_create_contact_binding(
        ovphysx_instance_handle_t handle,
        const ovphysx_string_t* sensor_patterns,
        uint32_t sensor_patterns_count,
        const ovphysx_string_t* filter_patterns,
        uint32_t filters_per_sensor,
        uint32_t max_contact_data_count,
        ovphysx_contact_binding_handle_t* out_handle);

    /**
     * @brief Destroy a contact binding.
     *
     * @param handle Instance handle
     * @param contact_handle Contact binding to destroy
     * @return ovphysx_result_t
     */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG(
        "ovphysx contact bindings are deprecated; include <ovphysx/ovphysx_legacy.h>")
    ovphysx_result_t ovphysx_destroy_contact_binding(
        ovphysx_instance_handle_t handle,
        ovphysx_contact_binding_handle_t contact_handle);

    /**
     * @brief Query contact view dimensions.
     *
     * @param handle Instance handle
     * @param contact_handle Contact binding
     * @param out_sensor_count [out] Number of sensor bodies matched
     * @param out_filter_count [out] Number of filter bodies per sensor
     * @return ovphysx_result_t
     */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG(
        "ovphysx contact bindings are deprecated; include <ovphysx/ovphysx_legacy.h>")
    ovphysx_result_t ovphysx_get_contact_binding_spec(
        ovphysx_instance_handle_t handle,
        ovphysx_contact_binding_handle_t contact_handle,
        int32_t* out_sensor_count,
        int32_t* out_filter_count);

    /**
     * @brief Get resolved sensor physics-object paths for a contact binding.
     *
     * The returned array order matches row order for contact binding reads.
     * ovphysx owns the returned string storage. String pointers remain valid
     * until the binding is destroyed.
     *
     * Callers must index `min(*out_count, max_paths)`. A short `out_paths`
     * still receives a valid prefix.
     *
     * @param handle Instance handle
     * @param contact_handle Contact binding
     * @param out_paths [out] Array of ovphysx_string_t to fill
     * @param max_paths Capacity of out_paths array
     * @param out_count [out] Total sensor-path demand, even when that exceeds
     *   max_paths
     * @return OVPHYSX_API_SUCCESS when every sensor path was written, or
     *   OVPHYSX_API_BUFFER_TOO_SMALL when demand exceeds max_paths
     */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG(
        "ovphysx contact bindings are deprecated; include <ovphysx/ovphysx_legacy.h>")
    ovphysx_result_t ovphysx_contact_binding_get_sensor_paths(
        ovphysx_instance_handle_t handle,
        ovphysx_contact_binding_handle_t contact_handle,
        ovphysx_string_t* out_paths,
        uint32_t max_paths,
        uint32_t* out_count);

    /**
     * @brief Get resolved filter physics-object paths for a contact binding.
     *
     * Paths are returned in row-major `[sensor, filter]` order with total
     * count `sensor_count * filter_count`. ovphysx owns the returned string
     * storage. String pointers remain valid until the binding is destroyed.
     *
     * Callers must index `min(*out_count, max_paths)`. A short `out_paths`
     * still receives a valid prefix.
     *
     * @param handle Instance handle
     * @param contact_handle Contact binding
     * @param out_paths [out] Array of ovphysx_string_t to fill
     * @param max_paths Capacity of out_paths array
     * @param out_count [out] Total filter-path demand
     *   (`sensor_count * filter_count`), even when that exceeds max_paths
     * @return OVPHYSX_API_SUCCESS when every filter path was written, or
     *   OVPHYSX_API_BUFFER_TOO_SMALL when demand exceeds max_paths
     */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG(
        "ovphysx contact bindings are deprecated; include <ovphysx/ovphysx_legacy.h>")
    ovphysx_result_t ovphysx_contact_binding_get_filter_paths(
        ovphysx_instance_handle_t handle,
        ovphysx_contact_binding_handle_t contact_handle,
        ovphysx_string_t* out_paths,
        uint32_t max_paths,
        uint32_t* out_count);

    /**
     * @brief Query detailed contact/friction flat-buffer capacity.
     *
     * This is the C dimension for `ovphysx_read_normal_contact_data()` and
     * `ovphysx_read_friction_contact_data()` flat buffers. Allocate force/separation
     * buffers as `[C, 1]`, point/normal/friction buffers as `[C, 3]`, and
     * count/start-index buffers as `[S, F]`, where `C` is this value and
     * `S`, `F` come from `ovphysx_get_contact_binding_spec()`.
     *
     * @param handle Instance handle
     * @param contact_handle Contact binding
     * @param out_max_contact_data_count [out] Max detailed contact/friction entries
     * @return ovphysx_result_t
     */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG(
        "ovphysx contact bindings are deprecated; include <ovphysx/ovphysx_legacy.h>")
    ovphysx_result_t ovphysx_get_contact_binding_capacity(
        ovphysx_instance_handle_t handle,
        ovphysx_contact_binding_handle_t contact_handle,
        uint32_t* out_max_contact_data_count);

    /**
     * @brief Read net normal contact forces. dst shape: [S, 3].
     *
     * Returns world-space normal forces over all reported contacts of each
     * sensor, regardless of filters or detailed-contact capacity. Adding the result
     * of ovphysx_read_contact_net_friction_forces() from the same simulation step
     * yields total contact forces.
     *
     * Impulse-to-force conversion uses dt from the last successful ovphysx_step() or
     * ovphysx_step_sync() call that simulated; a zero-dt step is skipped and leaves
     * both the contacts and the dt of the preceding positive-dt step in place. This
     * function waits for pending SDK work before retrieving the contact forces.
     */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG(
        "ovphysx contact bindings are deprecated; include <ovphysx/ovphysx_legacy.h>")
    ovphysx_result_t ovphysx_read_contact_net_normal_forces(
        ovphysx_instance_handle_t handle,
        ovphysx_contact_binding_handle_t contact_handle,
        DLTensor* dst_tensor);

    /**
     * @brief Read net friction forces. dst shape: [S, 3].
     *
     * Sums world-space friction-anchor forces over all reported contacts of each
     * sensor, including contacts with bodies not matched by the binding's
     * filter patterns. Neither filters nor detailed-contact capacity are required.
     * Opposing anchor forces can cancel while producing torque.
     * This vector sum does not report friction torque.
     *
     * Impulse-to-force conversion uses dt from the last successful ovphysx_step() or
     * ovphysx_step_sync() call that simulated; a zero-dt step is skipped and leaves
     * both the contacts and the dt of the preceding positive-dt step in place. This
     * function waits for pending SDK work before retrieving the contact forces.
     */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG(
        "ovphysx contact bindings are deprecated; include <ovphysx/ovphysx_legacy.h>")
    ovphysx_result_t ovphysx_read_contact_net_friction_forces(
        ovphysx_instance_handle_t handle,
        ovphysx_contact_binding_handle_t contact_handle,
        DLTensor* dst_tensor);

    /**
     * @brief Read normal contact force matrix. dst shape: [S, F, 3].
     *
     * Each entry sums world-space normal forces for its sensor/filter pair,
     * independently of detailed-contact capacity. Adding the result of
     * ovphysx_read_contact_friction_force_matrix() from the same step yields
     * total contact forces for the configured pairs.
     *
     * Impulse-to-force conversion uses dt from the last successful ovphysx_step() or
     * ovphysx_step_sync() call that simulated; a zero-dt step is skipped and leaves
     * both the contacts and the dt of the preceding positive-dt step in place. This
     * function waits for pending SDK work before retrieving the contact forces.
     */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG(
        "ovphysx contact bindings are deprecated; include <ovphysx/ovphysx_legacy.h>")
    ovphysx_result_t ovphysx_read_contact_normal_force_matrix(
        ovphysx_instance_handle_t handle,
        ovphysx_contact_binding_handle_t contact_handle,
        DLTensor* dst_tensor);

    /**
     * @brief Read friction force matrix. dst shape: [S, F, 3].
     *
     * Each entry sums world-space friction forces for its sensor/filter pair,
     * independently of detailed-contact capacity. Only contacts with configured
     * filter bodies contribute. Adding the normal force matrix from the same
     * step yields total contact forces for those pairs. Individual friction
     * forces and anchor points are available through ovphysx_read_friction_contact_data().
     *
     * Impulse-to-force conversion uses dt from the last successful ovphysx_step() or
     * ovphysx_step_sync() call that simulated; a zero-dt step is skipped and leaves
     * both the contacts and the dt of the preceding positive-dt step in place. This
     * function waits for pending SDK work before retrieving the contact forces.
     */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG(
        "ovphysx contact bindings are deprecated; include <ovphysx/ovphysx_legacy.h>")
    ovphysx_result_t ovphysx_read_contact_friction_force_matrix(
        ovphysx_instance_handle_t handle,
        ovphysx_contact_binding_handle_t contact_handle,
        DLTensor* dst_tensor);

    /** @deprecated Use ovphysx_read_contact_net_normal_forces(). Still returns normal forces only. */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG("use ovphysx_read_contact_net_normal_forces; returns normal forces only")
    ovphysx_result_t ovphysx_read_contact_net_forces(
        ovphysx_instance_handle_t handle,
        ovphysx_contact_binding_handle_t contact_handle,
        DLTensor* dst_tensor);

    /** @deprecated Use ovphysx_read_contact_normal_force_matrix(). Still returns normal forces only. */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG("use ovphysx_read_contact_normal_force_matrix; returns normal forces only")
    ovphysx_result_t ovphysx_read_contact_force_matrix(
        ovphysx_instance_handle_t handle,
        ovphysx_contact_binding_handle_t contact_handle,
        DLTensor* dst_tensor);

    /**
     * @brief Read detailed normal contact data into flat buffers.
     *
     * contact_force_tensor contains scalar normal forces. Multiply each scalar
     * by the corresponding contact_normal_tensor vector to obtain the world-space
     * normal force vector. Friction forces are returned separately by
     * ovphysx_read_friction_contact_data().
     *
     * Required shapes:
     * - contact_force_tensor: `[C, 1]` float32
     * - contact_point_tensor: `[C, 3]` float32
     * - contact_normal_tensor: `[C, 3]` float32
     * - contact_separation_tensor: `[C, 1]` float32
     * - contact_count_tensor: `[S, F]` int32 or uint32
     * - contact_start_indices_tensor: `[S, F]` int32 or uint32
     *
     * `C` is `ovphysx_get_contact_binding_capacity()`, `S` is sensor_count,
     * and `F` is filter_count. For each `(sensor, filter)` pair, the valid
     * detailed contact slice is:
     * `start = start_indices[s, f]`, `count = counts[s, f]`,
     * `data[start : start + count]`.
     * `C` and `F` must be positive. Pass a positive max_contact_data_count
     * and filters_per_sensor > 0 when creating the binding. Count and
     * start-index tensors may be int32 or uint32.
     * Contact force magnitudes use the timestep from the last successful
     * ovphysx_step() or ovphysx_step_sync() call that simulated; a zero-dt step is
     * skipped and leaves the preceding step's contacts and timestep in place.
     *
     * **Truncation**: `out_required_contact_count` receives the total number of
     * contacts produced for the step, before truncation. When that value exceeds
     * max_contact_data_count, the function returns
     * OVPHYSX_API_BUFFER_TOO_SMALL while still filling the flat buffers with as
     * many contacts as fit. A pair's count reports only contacts actually
     * written, and its start index is clamped to max_contact_data_count, so
     * `[start, start + count)` is always an in-range (possibly empty) slice.
     * Recreate the binding with max_contact_data_count at least
     * `out_required_contact_count`, step again, and retry to obtain a complete
     * result.
     *
     * @param handle Instance handle
     * @param contact_handle Contact binding
     * @param contact_force_tensor Pre-allocated contact normal force magnitudes
     * @param contact_point_tensor Pre-allocated world-frame contact points
     * @param contact_normal_tensor Pre-allocated world-frame contact normals
     * @param contact_separation_tensor Pre-allocated contact separations
     * @param contact_count_tensor Pre-allocated count matrix
     * @param contact_start_indices_tensor Pre-allocated start-index matrix
     * @param out_required_contact_count [out] Total contacts produced for the
     *   step before truncation; the capacity required for a complete read.
     * @return OVPHYSX_API_SUCCESS when the complete result fits,
     *   OVPHYSX_API_BUFFER_TOO_SMALL when only a valid prefix was written, or
     *   another error status on failure.
     */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG(
        "ovphysx contact bindings are deprecated; include <ovphysx/ovphysx_legacy.h>")
    ovphysx_result_t ovphysx_read_normal_contact_data(
        ovphysx_instance_handle_t handle,
        ovphysx_contact_binding_handle_t contact_handle,
        DLTensor* contact_force_tensor,
        DLTensor* contact_point_tensor,
        DLTensor* contact_normal_tensor,
        DLTensor* contact_separation_tensor,
        DLTensor* contact_count_tensor,
        DLTensor* contact_start_indices_tensor,
        uint32_t* out_required_contact_count);

    /** @deprecated Use ovphysx_read_normal_contact_data(). Forces remain normal components only. */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG("use ovphysx_read_normal_contact_data; forces are normal components only")
    ovphysx_result_t ovphysx_read_contact_data(
        ovphysx_instance_handle_t handle,
        ovphysx_contact_binding_handle_t contact_handle,
        DLTensor* contact_force_tensor,
        DLTensor* contact_point_tensor,
        DLTensor* contact_normal_tensor,
        DLTensor* contact_separation_tensor,
        DLTensor* contact_count_tensor,
        DLTensor* contact_start_indices_tensor,
        uint32_t* out_required_contact_count);

    /**
     * @brief Read detailed friction contact data into flat buffers.
     *
     * Returns world-space friction force vectors at friction anchors. These
     * anchors need not correspond one-to-one with the contact points from
     * ovphysx_read_normal_contact_data(); use this read's own counts and start indices.
     *
     * Required shapes:
     * - friction_force_tensor: `[C, 3]` float32
     * - friction_point_tensor: `[C, 3]` float32
     * - contact_count_tensor: `[S, F]` int32 or uint32
     * - contact_start_indices_tensor: `[S, F]` int32 or uint32
     *
     * `C`, `S`, and `F` have the same meanings as in
     * `ovphysx_read_normal_contact_data()`. For each `(sensor, filter)` pair, use the
     * count/start-index tensors to index valid entries in the flat friction
     * buffers. `C` and `F` must be positive. Pass a positive
     * max_contact_data_count and filters_per_sensor > 0 when creating the
     * binding. Count and start-index tensors may be int32 or uint32.
     * Friction forces use the timestep from the last successful
     * ovphysx_step() or ovphysx_step_sync() call that simulated; a zero-dt step is
     * skipped and leaves the preceding step's contacts and timestep in place.
     *
     * **Truncation**: `out_required_friction_count` receives the total number of
     * friction anchors produced for the step, before truncation. When that value
     * exceeds max_contact_data_count, the function returns
     * OVPHYSX_API_BUFFER_TOO_SMALL while still filling the flat buffers with as
     * many anchors as fit. A pair's count reports only anchors actually written,
     * and its start index is clamped to max_contact_data_count, so
     * `[start, start + count)` is always an in-range (possibly empty) slice.
     * Recreate the binding with max_contact_data_count at least
     * `out_required_friction_count`, step again, and retry to obtain a complete
     * result.
     *
     * @param handle Instance handle
     * @param contact_handle Contact binding
     * @param friction_force_tensor Pre-allocated world-frame friction forces
     * @param friction_point_tensor Pre-allocated world-frame friction points
     * @param contact_count_tensor Pre-allocated count matrix
     * @param contact_start_indices_tensor Pre-allocated start-index matrix
     * @param out_required_friction_count [out] Total friction anchors produced
     *   for the step before truncation; the capacity required for a complete read.
     * @return OVPHYSX_API_SUCCESS when the complete result fits,
     *   OVPHYSX_API_BUFFER_TOO_SMALL when only a valid prefix was written, or
     *   another error status on failure.
     */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG(
        "ovphysx contact bindings are deprecated; include <ovphysx/ovphysx_legacy.h>")
    ovphysx_result_t ovphysx_read_friction_contact_data(
        ovphysx_instance_handle_t handle,
        ovphysx_contact_binding_handle_t contact_handle,
        DLTensor* friction_force_tensor,
        DLTensor* friction_point_tensor,
        DLTensor* contact_count_tensor,
        DLTensor* contact_start_indices_tensor,
        uint32_t* out_required_friction_count);

    /** @deprecated Use ovphysx_read_friction_contact_data(). */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG("use ovphysx_read_friction_contact_data")
    ovphysx_result_t ovphysx_read_friction_data(
        ovphysx_instance_handle_t handle,
        ovphysx_contact_binding_handle_t contact_handle,
        DLTensor* friction_force_tensor,
        DLTensor* friction_point_tensor,
        DLTensor* contact_count_tensor,
        DLTensor* contact_start_indices_tensor,
        uint32_t* out_required_friction_count);

    /**
     * @brief Read raw (unfiltered) contact data for a contact binding.
     *
     * Filter-less variant of @ref ovphysx_read_normal_contact_data : returns every
     * contact involving each sensor body regardless of which other actor it
     * collided with, plus per-contact actor-identity tensors so callers can
     * identify both the sensor and the contacting body via
     * @ref ovphysx_contact_binding_get_other_actor_paths_from_ids.
     *
     * Required shapes (C = max_contact_data_count, S = sensor_count):
     * - contact_force_tensor:        `[C, 1]` float32, contact normal force magnitude
     * - contact_point_tensor:        `[C, 3]` float32, contact point in world frame
     * - contact_normal_tensor:       `[C, 3]` float32, contact normal in world frame
     * - contact_separation_tensor:   `[C, 1]` float32, signed separation
     * - sensor_layout_tensor:        `[S, 2]` int32/uint32, per sensor, column 0 is the
     *   contact count and column 1 is its start index into the flat buffers
     * - actor_ids_tensor:            `[C, 2]` int64/uint64, per contact, column 0 is the
     *   reporting sensor's own actor id and column 1 is the actor it contacted
     *
     * The two pairs that are only meaningful together are single tensors rather than
     * separate buffers the caller has to keep in step. The four per-contact value
     * tensors stay separate, matching ovphysx_read_normal_contact_data().
     *
     * The contact binding must be created with `max_contact_data_count > 0`.
     * No filter dimension is required, so `filters_per_sensor` may be zero. The
     * dt for impulse-to-force conversion is taken automatically from the last
     * successful ovphysx_step() or ovphysx_step_sync() call that simulated; a
     * zero-dt step is skipped and leaves the preceding step's contacts and
     * timestep in place.
     *
     * **Truncation**: `out_required_contact_count` receives the total number of
     * contacts produced for the step, before truncation. When that value exceeds
     * max_contact_data_count, the function returns
     * OVPHYSX_API_BUFFER_TOO_SMALL while still filling the flat buffers with as
     * many contacts as fit. A sensor's count reports only contacts actually
     * written, and its start index is clamped to max_contact_data_count, so
     * `[start, start + count)` is always an in-range (possibly empty) slice.
     * Recreate the binding with max_contact_data_count at least
     * `out_required_contact_count`, step again, and retry to obtain a complete
     * result.
     *
     * **Token lifetime**: the uint64 tokens in actor_ids_tensor are opaque runtime
     * actor handles, not encoded paths. Do not decode one. Resolve it with
     * @ref ovphysx_contact_binding_get_other_actor_paths_from_ids. They are stable
     * for as long as the corresponding actor is alive on the attached stage, and
     * become stale once it is removed or the stage is detached or replaced.
     *
     * @param handle Instance handle
     * @param contact_handle Contact binding
     * @param out_required_contact_count [out] Total contacts produced for the
     *   step before truncation; the capacity required for a complete read.
     * @return OVPHYSX_API_SUCCESS when the complete result fits,
     *   OVPHYSX_API_BUFFER_TOO_SMALL when only a valid prefix was written, or
     *   another error status on failure.
     */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG(
        "ovphysx contact bindings are deprecated; include <ovphysx/ovphysx_legacy.h>")
    ovphysx_result_t ovphysx_read_raw_contact_data(
        ovphysx_instance_handle_t handle,
        ovphysx_contact_binding_handle_t contact_handle,
        DLTensor* contact_force_tensor,
        DLTensor* contact_point_tensor,
        DLTensor* contact_normal_tensor,
        DLTensor* contact_separation_tensor,
        DLTensor* sensor_layout_tensor,
        DLTensor* actor_ids_tensor,
        uint32_t* out_required_contact_count);

    /**
     * @brief Resolve actor IDs from @ref ovphysx_read_raw_contact_data to physics-object paths.
     *
     * Given a tensor of opaque actor IDs (a column of the `actor_ids_tensor`
     * `[C, 2]` returned by @ref ovphysx_read_raw_contact_data, either column,
     * since both use the same namespace), fills `out_paths` with the corresponding
     * physics-object paths in the same order. An ID of `0` yields an empty path,
     * as does an ID that is not known to the attached stage at all.
     *
     * `ids_tensor` must be **CPU-resident** (path resolution is a host operation)
     * and **C-contiguous**: a column of `actor_ids_tensor` is a strided view, so
     * make the slice contiguous (e.g. `np.ascontiguousarray(actor_ids[:, 1])`)
     * before passing it. This is the diagnostic path. The per-contact read itself
     * takes the whole contiguous tensor and pays no copy.
     *
     * **Stale IDs report as empty, not as their old path.** Every non-zero ID is
     * checked against the attached stage before it is resolved, so an actor that
     * has since been removed comes back as an empty path rather than the path it
     * used to name. Because the caller holds the IDs, this makes the failure
     * explicit: for a non-zero ID an empty path means "not resolvable now", and
     * only ID `0` yields an empty path for a live read.
     *
     * **Truncation**: `out_count` receives the length of `ids_tensor`. When that
     * exceeds `max_paths`, the function returns OVPHYSX_API_BUFFER_TOO_SMALL
     * while writing a valid prefix of `min(*out_count, max_paths)` strings.
     *
     * The check is as precise as the attached backend's notion of existence. On an
     * ovstage attach a removed prim reports stale. On a USD stage a merely
     * *deactivated* prim still resolves, because existence there follows prim
     * validity and USD hands back a valid prim for an inactive one.
     *
     * @param handle Instance handle
     * @param contact_handle Contact binding
     * @param ids_tensor Input tensor of actor IDs (`[N]` int64/uint64)
     * @param out_paths [out] Array of @ref ovphysx_string_t to fill. ovphysx
     *   owns the returned string storage. Pointers remain valid until the
     *   next call to this function on the same binding (which replaces the
     *   cache) or until the binding is destroyed.
     * @param max_paths Capacity of `out_paths` array.
     * @param out_count [out] Length of `ids_tensor` (total demand). Only
     *   `min(*out_count, max_paths)` entries are written to `out_paths`.
     * @return OVPHYSX_API_SUCCESS when the complete result fits,
     *   OVPHYSX_API_BUFFER_TOO_SMALL when only a valid prefix was written, or
     *   another error status on failure.
     */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG(
        "ovphysx contact bindings are deprecated; include <ovphysx/ovphysx_legacy.h>")
    ovphysx_result_t ovphysx_contact_binding_get_other_actor_paths_from_ids(
        ovphysx_instance_handle_t handle,
        ovphysx_contact_binding_handle_t contact_handle,
        DLTensor* ids_tensor,
        ovphysx_string_t* out_paths,
        uint32_t max_paths,
        uint32_t* out_count);

    /** @} */

    /**
     * @brief Classify an authored USD prim by its high-level TensorAPI object type.
     *
     * Returns the umbrella's view of what kind of simulation object lives at
     * `prim_path`. See @ref ovphysx_object_type_t for the taxonomy, which
     * distinguishes standalone (`OVPHYSX_OBJECT_TYPE_JOINT`), custom
     * (`OVPHYSX_OBJECT_TYPE_CUSTOM_JOINT`), and articulation
     * (`OVPHYSX_OBJECT_TYPE_ARTICULATION_JOINT`) joints.
     *
     * Native addresses for those kinds are the `physxPointer` column:
     * `OVPHYSX_QUERY_TARGET_JOINT`, `OVPHYSX_QUERY_TARGET_CUSTOM_JOINT`, and
     * `OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT`. Object-change callbacks still
     * report `ovphysx_physx_type_t`.
     *
     * Paths with no classified simulation object yield
     * `OVPHYSX_OBJECT_TYPE_INVALID` with `OVPHYSX_API_SUCCESS` (the call
     * itself did not fail, for example because the prim is absent or has no
     * matching TensorAPI object type). Live standalone, custom, and articulation joints
     * at their authored prim paths must not report INVALID.
     *
     * @param handle Instance handle (must have a stage attached)
     * @param prim_path Authored USD prim path (embedded NUL bytes are rejected)
     * @param[out] out_type Receives the object type
     * @return OVPHYSX_API_SUCCESS on success (including the INVALID
     *         classification above), OVPHYSX_API_INVALID_ARGUMENT if
     *         `out_type` is NULL or `prim_path` is NULL, empty, or contains an
     *         embedded NUL byte, or OVPHYSX_API_ERROR if no stage is attached
     *         or the TensorAPI simulation view cannot be created.
     */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG(
        "ovphysx_get_object_type is deprecated; include <ovphysx/ovphysx_legacy.h>")
    ovphysx_result_t ovphysx_get_object_type(
        ovphysx_instance_handle_t handle,
        ovphysx_string_t prim_path,
        ovphysx_object_type_t* out_type);

    /**
     * Wake rigid bodies in a binding.
     *
     * Mirrors PhysX SDK's `physx::PxRigidDynamic::wakeUp`. Bodies that
     * still have `physx::PxActorFlag::eDISABLE_SIMULATION` set are
     * silently skipped (the engine refuses to wake disabled actors).
     *
     * Typical pair: clear the actor's `eDISABLE_SIMULATION` flag through the
     * ovstage `disableSimulation` write (re-add to simulation), then call this
     * to bring the actor active for the next simulate -- otherwise it sits in
     * the sleep state PhysX placed it in when the flag was first set.
     *
     * @param handle Instance handle
     * @param binding_handle Rigid body tensor binding identifying the set
     *        of bodies in scope.
     * @param indices Optional int32 DLTensor of indices into the binding,
     *        or NULL to wake every body in the binding.
     * @return OVPHYSX_API_SUCCESS, OVPHYSX_API_INVALID_ARGUMENT if the
     *         binding is not a rigid-body binding, OVPHYSX_API_NOT_FOUND
     *         if the binding has been invalidated by a stage change,
     *         OVPHYSX_API_ERROR on engine failure.
     *
     * @deprecated Binding-coupled (requires a tensor-binding handle) and removed with the tensor
     *   binding. Successor: the `wake` write attribute on `ovphysx_write` -- a per-prim rigid-body
     *   sleep control (nonzero wakes, zero sleeps), supported on CPU scenes and GPU scenes with CPU
     *   readback, refused on DirectGPU. A prim-set-selected query narrows the command to those bodies.
     */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG(
        "ovphysx tensor bindings are deprecated; this binding-coupled wake/sleep API is removed with them (successor: the 'wake' write attribute on ovphysx_write)")
    ovphysx_result_t ovphysx_rigid_body_view_wake_up(
        ovphysx_instance_handle_t handle,
        ovphysx_tensor_binding_handle_t binding_handle,
        const DLTensor* indices);

    /**
     * Force rigid bodies in a binding to sleep.
     *
     * Mirrors PhysX SDK's `physx::PxRigidDynamic::putToSleep`. Sets the
     * sleep state on each body so it is excluded from the next solve unless
     * woken by a contact or an explicit @ref ovphysx_rigid_body_view_wake_up
     * call. Bodies that have `physx::PxActorFlag::eDISABLE_SIMULATION` set
     * are silently skipped.
     *
     * @param handle Instance handle
     * @param binding_handle Rigid body tensor binding identifying the set
     *        of bodies in scope.
     * @param indices Optional int32 DLTensor of indices into the binding,
     *        or NULL to put every body in the binding to sleep.
     * @return OVPHYSX_API_SUCCESS, OVPHYSX_API_INVALID_ARGUMENT if the
     *         binding is not a rigid-body binding, OVPHYSX_API_NOT_FOUND
     *         if the binding has been invalidated by a stage change,
     *         OVPHYSX_API_ERROR on engine failure.
     *
     * @deprecated Binding-coupled (requires a tensor-binding handle) and removed with the tensor
     *   binding. Successor: the `wake` write attribute on `ovphysx_write` -- a per-prim rigid-body
     *   sleep control (nonzero wakes, zero sleeps), supported on CPU scenes and GPU scenes with CPU
     *   readback, refused on DirectGPU. A prim-set-selected query narrows the command to those bodies.
     */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG(
        "ovphysx tensor bindings are deprecated; this binding-coupled wake/sleep API is removed with them (successor: the 'wake' write attribute on ovphysx_write)")
    ovphysx_result_t ovphysx_rigid_body_view_sleep(
        ovphysx_instance_handle_t handle,
        ovphysx_tensor_binding_handle_t binding_handle,
        const DLTensor* indices);

    /*--------------------------------------------------*/
    /* Scene queries                                    */
    /*--------------------------------------------------*/

    /**
     * @defgroup ovphysx_scene_query Scene queries
     *
     * Raycast, sweep, and overlap queries against the physics scene.
     *
     * All three functions follow the same output pattern: hit results are
     * written to an **internal buffer** owned by the ovphysx instance. The
     * returned pointer is valid until the next scene query call on the same
     * instance (any of the three functions). This avoids two-call patterns
     * and is simple for language bindings.
     *
     * Path fields in the hit struct (collision, rigid_body, material) hold an
     * opaque omni::physics::parse::ObjectKey.handle, not a uint64-encoded
     * SdfPath. There is no client-side bit-cast that reproduces or compares
     * against it. Resolve a hit's identity fields to a prim path with
     * @ref ovphysx_scene_query_get_paths_from_ids instead of comparing raw
     * values.
     *
     * @pre A USD stage must be loaded and at least one simulation step
     *      completed for the scene to contain queryable objects.
     *
     * **Thread safety:** Scene query functions must only be called between
     * simulation steps, after `ovphysx_wait_task()` / `ovphysx_wait_all()`
     * complete and before the next `ovphysx_step()`.
     */

    /** @addtogroup ovphysx_scene_query */
    /** @{ */

    /**
     * @brief Cast a ray and return hits.
     *
     * @param handle     Instance handle.
     * @param origin     Ray origin (world space, 3 floats).
     * @param direction  Normalized ray direction (3 floats).
     * @param distance   Maximum ray distance. Must be >= 0.
     * @param both_sides If true, test both sides of mesh triangles.
     * @param mode       CLOSEST (0 or 1 hit), ANY (0 or 1), or ALL.
     * @param[out] out_hits   Receives pointer to internal hit array.
     * @param[out] out_count  Number of hits in the array.
     * @return ovphysx_result_t
     *
     * @code{.c}
     * #include <ovphysx/ovphysx_legacy.h>
     * #include <stdio.h>
     *
     * static ovphysx_result_t print_closest_hit(ovphysx_instance_handle_t handle)
     * {
     *     const ovphysx_scene_query_hit_t* hits = NULL;
     *     uint32_t count = 0;
     *     const float origin[3] = {0.0f, 10.0f, 0.0f};
     *     const float direction[3] = {0.0f, -1.0f, 0.0f};
     *     ovphysx_result_t result = ovphysx_raycast(
     *         handle, origin, direction, 100.0f, false,
     *         OVPHYSX_SCENE_QUERY_MODE_CLOSEST, &hits, &count);
     *     if (result.status == OVPHYSX_API_SUCCESS && count > 0)
     *         printf("Hit at distance %f\n", hits[0].distance);
     *     return result;
     * }
     * @endcode
     */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG(
        "ovphysx scene queries are deprecated; include <ovphysx/ovphysx_legacy.h>")
    ovphysx_result_t ovphysx_raycast(
        ovphysx_instance_handle_t handle,
        const float origin[3],
        const float direction[3],
        float distance,
        bool both_sides,
        ovphysx_scene_query_mode_t mode,
        const ovphysx_scene_query_hit_t** out_hits,
        uint32_t* out_count);

    /**
     * @brief Sweep a geometry shape along a direction and return hits.
     *
     * @param handle     Instance handle.
     * @param geometry   Geometry descriptor (sphere, box, or arbitrary shape).
     * @param direction  Normalized sweep direction (3 floats).
     * @param distance   Maximum sweep distance. Must be >= 0.
     * @param both_sides If true, test both sides of mesh triangles.
     * @param mode       CLOSEST (0 or 1 hit), ANY (0 or 1), or ALL.
     * @param[out] out_hits   Receives pointer to internal hit array.
     * @param[out] out_count  Number of hits in the array.
     * @return ovphysx_result_t
     */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG(
        "ovphysx scene queries are deprecated; include <ovphysx/ovphysx_legacy.h>")
    ovphysx_result_t ovphysx_sweep(
        ovphysx_instance_handle_t handle,
        const ovphysx_scene_query_geometry_desc_t* geometry,
        const float direction[3],
        float distance,
        bool both_sides,
        ovphysx_scene_query_mode_t mode,
        const ovphysx_scene_query_hit_t** out_hits,
        uint32_t* out_count);

    /**
     * @brief Test geometry overlap against objects in the scene.
     *
     * Overlap queries do not have a direction or distance. Location fields
     * in the hit struct (normal, position, distance, face_index, material)
     * are zeroed. Only object identity (collision, rigid_body, proto_index)
     * is populated.
     *
     * @param handle     Instance handle.
     * @param geometry   Geometry descriptor (sphere, box, or arbitrary shape).
     * @param mode       ANY (0 or 1 result) or ALL. CLOSEST is treated as ALL.
     * @param[out] out_hits   Receives pointer to internal hit array.
     * @param[out] out_count  Number of hits (or overlaps) in the array.
     * @return ovphysx_result_t
     */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG(
        "ovphysx scene queries are deprecated; include <ovphysx/ovphysx_legacy.h>")
    ovphysx_result_t ovphysx_overlap(
        ovphysx_instance_handle_t handle,
        const ovphysx_scene_query_geometry_desc_t* geometry,
        ovphysx_scene_query_mode_t mode,
        const ovphysx_scene_query_hit_t** out_hits,
        uint32_t* out_count);

    /**
     * @brief Resolve object-identity handles to physics-object paths.
     *
     * Given an array of opaque object-identity handles produced by this API,
     * fills `out_paths` with the corresponding physics-object paths in the same
     * order. Despite the `scene_query` name the resolution is generic: any
     * identity field carries the same opaque handle, so scene-query hits
     * (`collision`, `rigid_body`, `material` of an
     * @ref ovphysx_scene_query_hit_t from @ref ovphysx_raycast,
     * @ref ovphysx_sweep or @ref ovphysx_overlap) are accepted. Contact-sensor
     * read identities are prim-path handles; resolve those through the attached
     * Stage's path dictionary. IDs
     * that cannot be resolved (a zero/sentinel id, an id from an object removed
     * since it was reported, or a call with no active attach) yield empty paths
     * rather than an error.
     *
     * @param handle Instance handle.
     * @param ids Input array of opaque identity handles (see fields above).
     * @param id_count Number of entries in `ids`.
     * @param[out] out_paths Array of @ref ovphysx_string_t to fill. ovphysx
     *   owns the returned string storage. Unlike
     *   @ref ovphysx_contact_binding_get_other_actor_paths_from_ids, these
     *   pointers are not a per-call cache. Each is owned by the currently
     *   attached physics source and stays valid only until the next detach or
     *   re-attach. A later call to this function (against the same attach)
     *   does not invalidate a pointer returned by an earlier one. Callers
     *   that need a path to outlive a detach/re-attach must copy it.
     * @param max_paths Capacity of `out_paths` array.
     * @param[out] out_count Total number of ids in `ids` (== id_count). Only
     *   `min(id_count, max_paths)` entries are written to `out_paths`.
     * @return OVPHYSX_API_SUCCESS when the complete result fits,
     *   OVPHYSX_API_BUFFER_TOO_SMALL when only a valid prefix was written.
     *   Unresolvable ids still yield empty strings; BUFFER_TOO_SMALL is only
     *   for a short `out_paths`, not for empty path entries.
     */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG(
        "ovphysx scene queries are deprecated; include <ovphysx/ovphysx_legacy.h>")
    ovphysx_result_t ovphysx_scene_query_get_paths_from_ids(
        ovphysx_instance_handle_t handle,
        const uint64_t* ids,
        uint32_t id_count,
        ovphysx_string_t* out_paths,
        uint32_t max_paths,
        uint32_t* out_count);

    /** @} */

    /*=========================================================================
     * SDF shape evaluation
     *=========================================================================*/

    /**
     * Create an SDF shape view for shapes matching the given pattern.
     *
     * Deprecated: use ovphysx_evaluate_sdf_shapes (ovphysx_geometry_query.h) over an
     * OVPHYSX_QUERY_TARGET_SDF_SHAPE query, which evaluates the same shapes and takes Q from the
     * tensors on every call.
     *
     * The view evaluates the signed distance field of PhysX collision shapes
     * at caller-supplied query points. Requires a GPU instance. CPU SDF
     * evaluation is not implemented and create fails for CPU instances.
     *
     * The returned handle must be released with ovphysx_destroy_sdf_view when
     * no longer needed. The handle is invalidated when the attached USD stage
     * changes (ovphysx_detach_ovstage or loading a new stage) or when the instance is destroyed (via ovphysx_destroy_instance) even if
     * ovphysx_destroy_sdf_view is not called explicitly. After invalidation,
     * evaluate/get calls return OVPHYSX_API_NOT_FOUND. Recreate the view after
     * re-attaching a stage.
     *
     * Thread safety: safe to call concurrently with other ovphysx API functions
     * on the same handle, but not concurrently with ovphysx_destroy_instance on the
     * same handle.
     *
     * @param handle            Instance handle.
     * @param pattern           USD-style path glob selecting SDF-enabled collision
     *                          shapes, including runtime-only clones (e.g. "/World/Mesh*").
     *                          Must match at least one shape or the call returns an error.
     *                          A single path component longer than 4096 characters is
     *                          rejected with OVPHYSX_API_INVALID_ARGUMENT.
     * @param max_query_points  Number of query points per shape per evaluate
     *                          call. Query tensors passed to ovphysx_evaluate_sdf
     *                          must have Q == this value (second dimension).
     *                          Must be > 0.
     * @param out_handle        Receives the new SDF view handle on success.
     *                          Set to 0 on failure.
     * @return OVPHYSX_API_SUCCESS or an error code.
     */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG(
        "ovphysx SDF views are deprecated; include <ovphysx/ovphysx_legacy.h>")
    ovphysx_result_t ovphysx_create_sdf_view(
        ovphysx_instance_handle_t handle,
        ovphysx_string_t pattern,
        uint32_t max_query_points,
        ovphysx_sdf_view_handle_t* out_handle);

    /**
     * Return the number of shapes in the SDF view (N, first dimension of query tensors).
     *
     * @param handle        Instance handle.
     * @param sdf_handle    SDF view handle from ovphysx_create_sdf_view.
     * @param out_count     Receives the shape count on success.
     * @return OVPHYSX_API_SUCCESS, OVPHYSX_API_INVALID_ARGUMENT if out_count is NULL,
     *         OVPHYSX_API_NOT_FOUND if sdf_handle is not valid.
     */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG(
        "ovphysx SDF views are deprecated; include <ovphysx/ovphysx_legacy.h>")
    ovphysx_result_t ovphysx_sdf_view_get_count(
        ovphysx_instance_handle_t handle,
        ovphysx_sdf_view_handle_t sdf_handle,
        uint32_t* out_count);

    /**
     * Return the max_query_points value this view was created with (Q, second dimension).
     *
     * @param handle               Instance handle.
     * @param sdf_handle           SDF view handle from ovphysx_create_sdf_view.
     * @param out_max_query_points Receives the max query points on success.
     * @return OVPHYSX_API_SUCCESS, OVPHYSX_API_INVALID_ARGUMENT if out_max_query_points is NULL,
     *         OVPHYSX_API_NOT_FOUND if sdf_handle is not valid.
     */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG(
        "ovphysx SDF views are deprecated; include <ovphysx/ovphysx_legacy.h>")
    ovphysx_result_t ovphysx_sdf_view_get_max_query_points(
        ovphysx_instance_handle_t handle,
        ovphysx_sdf_view_handle_t sdf_handle,
        uint32_t* out_max_query_points);

    /**
     * Evaluate the SDF at query points and write distances + gradients.
     *
     * DLTensor requirements:
     *   query_points:
     *     - shape: [N, Q, 3] where N = shape count, Q == max_query_points
     *     - dtype: float32
     *     - device: GPU (kDLCUDA)
     *   out_distances_and_gradients:
     *     - shape: [N, Q, 4] with component layout (grad.x, grad.y, grad.z, distance)
     *     - dtype: float32
     *     - device: same as query_points
     *     - must be pre-allocated with the correct shape
     *
     * @param handle                        Instance handle.
     * @param sdf_handle                    SDF view handle from ovphysx_create_sdf_view.
     * @param query_points                  Query point tensor [N, Q, 3].
     * @param out_distances_and_gradients   Output tensor [N, Q, 4].
     * @return OVPHYSX_API_SUCCESS or an error code.
     */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG(
        "ovphysx SDF views are deprecated; include <ovphysx/ovphysx_legacy.h>")
    ovphysx_result_t ovphysx_evaluate_sdf(
        ovphysx_instance_handle_t handle,
        ovphysx_sdf_view_handle_t sdf_handle,
        const DLTensor* query_points,
        DLTensor* out_distances_and_gradients);

    /**
     * Destroy an SDF view and release its resources.
     *
     * Idempotent: returns OVPHYSX_API_SUCCESS if the view was already
     * destroyed or removed by stage detach cleanup. After this call
     * the handle is invalid and must not be used for evaluate/read paths.
     *
     * @param handle        Instance handle.
     * @param sdf_handle    SDF view handle from ovphysx_create_sdf_view.
     * @return OVPHYSX_API_SUCCESS, or OVPHYSX_API_ERROR on invalid instance handle.
     */
    OVPHYSX_API OVPHYSX_DEPRECATED_MSG(
        "ovphysx SDF views are deprecated; include <ovphysx/ovphysx_legacy.h>")
    ovphysx_result_t ovphysx_destroy_sdf_view(
        ovphysx_instance_handle_t handle,
        ovphysx_sdf_view_handle_t sdf_handle);


#ifdef __cplusplus
}
#endif

#endif // OVPHYSX_LEGACY_H
