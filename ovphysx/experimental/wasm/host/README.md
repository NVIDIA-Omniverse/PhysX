<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Static browser host prerequisite

This experiment prepares an isolated copy of the existing OVStage WASM Carbonite
headers and host. It does not replace OvPhysX or implement a second physics engine.
The dispatcher test includes the original `PhysXCarbCPUDispatcher.h` from this checkout.

The host has synchronization support but no Carbonite scheduler worker pool.
The copied host's `ITasking.getDesc` reports that zero-worker configuration. The
original PhysX dispatcher then executes `task.run()` followed by `task.release()`
on the caller. Other unsupported host operations retain their existing failures.
The test checks that behavior for requested worker counts 0, 1 and 8, including
call order and caller-thread identity.

The separate [solver worker experiment](../integration/SOLVER_THREADS.md) uses
the native runtime's existing SDK dispatcher option for 2 or 4 additional solver
workers. It does not add Carbonite scheduling support. The bootstrap accepts
nonzero solver counts only for that SDK dispatcher; the package owner must check
preloaded Emscripten pool capacity before attachment. Zero remains the default.

The copied Carbonite headers gain explicit Emscripten branches for environment
configuration paths and the diagnostic platform name. XDG paths refer to the
module's Emscripten filesystem; they provide no native host directory access or
persistent browser storage. The script checks the exact input hashes, records
all edits and output hashes, and never modifies its dependency sources. Relative
Carbonite includes require copying the include tree as a unit. Preparation checks
planned destinations before creating output, including existing symlink and
hard-link aliases into either dependency or the recipe's source files. Unwritten
entries in a reused output are left alone; separate output files may live under
the recipe directory. Pure input-preservation tests require no SDK or native build:

```sh
python3 ovphysx/experimental/wasm/host/test-prepare.py
```

Run with the already prepared OVStage browser dependencies:

```bash
python3 ovphysx/experimental/wasm/host/prepare.py \
  --carb-include /tmp/ovstage-fabric-build/carb-port/include \
  --fabric-host /home/rilei/dev/kit_2/rendering/ovstage/experimental/wasm/fabric/host \
  --output /tmp/ovphysx-wasm-canonical/host \
  --test \
  --emxx /tmp/ovstage-wasm-emsdk/upstream/emscripten/em++ \
  --node /tmp/ovstage-wasm-emsdk/node/24.19.0_64bit/bin/node
```

The selected compiler must support memory64 and the Node runtime must support
WebAssembly memory64 (the tested Emscripten 5.0.7 output requires Node 23+).
`dispatcher-command.json`, `settings-command.json`, `events-command.json`, build/test logs,
`overlay-receipt.json`, and the corresponding test receipts are written to the output directory. The test uses C++20 for
the existing host; genuine OvPhysX source objects use their upstream C++17 mode.
All linked objects must agree on memory64, threads, exceptions, SDK compile-time
feature macros, and the shared runtime ABI.

`TaskStorage.inl` provides bounded, unique live task-storage keys required by
original runtime callback initialization. With no scheduler task context,
get/set return their documented null/false outcomes; the runtime retains its
original native thread-local callback state. Freeing a key never invokes its
destructor callback. Allocation, exhaustion, free and reuse are tested.

`SettingsArrays.inl` stores actual string arrays as indexed children in the
existing settings tree. It supports empty defaults, nonempty replacement,
default preservation, indexed reads, and array-shape checks. The original
`PhysXSettings::setDefaults()` calls these APIs during startup.

`Events.cpp` registers an actual IEvents 1.4 provider in the borrowed stage host.
It supports FIFO queues, blocking push/pop, ordered and typed push/pop listeners,
consumption on pop/dispatch, subscription mutation during callbacks, and sender
IDs. External unsubscribe waits for running callbacks; callback-side unsubscribe
marks the subscription inactive without waiting, avoiding crossed callback
waits. Each running callback retains its listener. Stream operations retain the
stream while calling user callbacks. The suite tests these lifetime contracts,
including simultaneous self-unsubscribe and releasing the stream's final owner
from a push callback. Object-name enumeration reports an explicit unsupported
error when attached objects exist; it does not claim to supply the RString
service.

`DictionaryPayload.inl` adds independently owned dictionary roots and descendants
for event payloads to the copied host. Integer, floating-point, boolean, string,
and nested payload writes use the same real IDictionary interface that physics
calls. It retains the existing flat settings store and its notification behavior.
The overlay's scoped locks delegate to the stage host's lock-depth tracking, so
settings callbacks run after the outermost scope releases. Retained settings items
and tree roots preserve callback lifetime through deletion and recreation. The
`--test` command compiles the canonical stage `population/native/settings-test.cpp`
against this generated host, catching overlay changes that would bypass the stage
host's callback rules. Callbacks must not throw; destruction callbacks may hold
internal locks. Unsupported settings-tree enumeration still fails explicitly. All dictionary
operations share the host recursive mutex. Payload roots must be destroyed
before the owner shuts the host down.

`Log.cpp` provides the synchronous Omni `ILog` consumed by original runtime
startup: global and channel enable/threshold overrides, live channel slots,
copy-owned descriptions, retained message/update consumers, enumeration,
formatting and thread-safe delivery. It writes diagnostics to stderr and invokes
registered consumers. `setAsync(true)` fails explicitly. `StaticServices.cpp`
replaces only the original population `UnavailableServices.cpp` object: logging
is supplied, optional structured logging stays absent, and dynamic type creation
and remote blob-key hashing still fail explicitly. `log-test.cpp` exercises the
real provider under WASM64, including concurrent delivery and slot removal.

`StaticBootstrap.cpp` replaces the native OS/plugin-directory loader in this
experimental target. It borrows the existing framework, registers events, checks
public CPU_ONLY=true and ASYNC_EXECUTION=false configuration, disables UJITSO,
and calls the original `omni::physx::runtime::startup()`. `StaticSidecar.cpp`
retains the original interface/version assertions and binds original sidecar
exports directly to the original runtime read/write/accessor functions. The
three original sidecar implementation units remain linked. Exclude only the
native loader, dynamic sidecar loader, and peer-DSO global-bootstrap unit.
The main owns shared services. Each linked module initializes its deliberately
local Carbonite globals from the same borrowed framework; the split bootstrap
also initializes allocator/error/weak-object callbacks from that framework.

These are compiled bootstrap components and executed host prerequisites. They
do not prove complete OvPhysX initialization, attachment, stepping, GPU support,
or browser packaging. The shared runtime has passed the exercised CPU attach/step and browser
fixtures; broader services and native workflow parity remain separate gates. The
synchronous instance mode (`OVPHYSX_GLOBAL_CONFIG_ASYNC_EXECUTION=false`) avoids
the additional instance worker pool; it does not remove those service needs.
