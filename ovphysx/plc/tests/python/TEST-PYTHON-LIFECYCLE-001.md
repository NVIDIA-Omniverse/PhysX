<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-PYTHON-LIFECYCLE-001
maps_to: REQ-PYTHON-LIFECYCLE-001
type: integration
---

## Scenario

The public Python lifecycle is exercised against a real native runtime for the
success, child-invalidation, multi-instance, and idempotence paths. Focused fake
native-library and process-lifecycle hooks make both otherwise-unreachable
failure phases deterministic: an exception before native status return, a
returned native error status, and a process-shutdown exception after native
destruction.

The main-suite test is discovered automatically by pytest in
`scripts/test_python_runtime.cmake`. Each real-runtime lifecycle test file is
discovered by the same script and run in its own subprocess because Carbonite
cannot be reinitialized safely within one Python process.

## Given

- Two simultaneous real `PhysX` instances and a tensor binding owned by the
  first instance.
- Two real C handles and process-global asynchronous state whose first-handle
  destruction succeeds before repeated destruction exercises the
  already-absent native status.
- A fake native library whose destroy operation can return success, return a
  known error status, or raise a Python exception.
- A replaceable process-lifecycle release hook that can succeed or raise.
- A constructor that fails before lifecycle or native-instance ownership, a
  partially constructed `PhysX` object with no initialized attributes, and an
  object whose handle contains the invalid sentinel.
- A native log callback invoked while process initialization or final shutdown
  has published its lifecycle transition and is running native callback work
  without holding the process-lifecycle lock.

## When

- `destroy()` is called on the first real instance, its child binding is used,
  the second instance is used, and destruction is repeated on both instances.
- Native destruction returns an error status.
- Native destruction raises before returning a status and `destroy()` is then
  retried.
- The first real C handle is destroyed twice while the peer handle and one of
  its queued operations remain live.
- Native destruction succeeds but process shutdown raises, followed by another
  `destroy()` and finalizer cleanup.
- Native destruction returns an error and process shutdown also raises.
- The runtime class and public type stub are inspected for the removed
  `release()` and main-object context-manager protocols.
- `__del__` cleans up an undestroyed instance, including a first-call native
  invocation exception followed by either success or the terminal
  already-absent status, plus persistent invocation exceptions on both cleanup
  attempts, partial-construction objects, and callback-time finalization with
  either the sole process token or one of multiple tokens.
- Construction fails during the version check before acquiring lifecycle
  ownership.
- The callback attempts `PhysX` construction and destruction. During process
  initialization it starts but does not wait on an ordinary constructor; during
  final shutdown it waits on a constructor that must reject immediately.
- Independent threads race lifecycle acquisition with initialization,
  including native failure and injected Python interruption paths.
- Python is interrupted when native initialization may already have committed
  but before the returned status can be published into Python lifecycle ownership,
  including an interruption while that ambiguous result is handed to shutdown.
- Python is interrupted while final release publishes its process-shutdown
  handoff and while the completed shutdown clears `SHUTTING_DOWN`.

## Then

- Successful destruction clears the parent handle, invalidates the child
  binding, decrements the process refcount once, leaves the simultaneous
  instance usable, and makes repeated destruction a no-op (REQ AC-1, AC-4).
- A returned native error commits terminal state, releases lifecycle ownership
  once, includes the status name and value without a last-error string, and is
  not retried by a later call (REQ AC-2).
- A Python/native-invocation exception preserves the handle, attached stage,
  destroyed marker, and lifecycle ownership; a later call retries and reaches
  a terminal state (REQ AC-3).
- Repeated native destruction returns `OVPHYSX_API_ERROR` before teardown,
  leaves the peer handle and its queued operation intact, and allows both to be
  cleaned up normally. This pins the side-effect-free native
  already-absent behavior that Python AC-3 uses to converge after an ambiguous
  invocation exception.
- A shutdown exception reports that the instance is destroyed but process
  shutdown failed. The state stays terminal, and neither a later call nor
  `__del__` decrements the refcount again. If native status and process shutdown
  both fail, the shutdown failure is reported and its message includes the
  native status name and value (REQ AC-2, AC-4).
- `PhysX.release`, `PhysX.__enter__`, and `PhysX.__exit__` are absent at runtime
  and from the public stub. The finalizer emits `ResourceWarning`, retries only
  when the first failure leaves ownership live, and suppresses persistent
  invocation exceptions after its single retry while abandoning the Python
  handle, emitting `RuntimeWarning` for possibly registered native state, and
  discharging process-lifecycle ownership. Callback-time finalization retains
  the object, returns without waiting for cleanup, and completes the same
  cleanup policy off the callback thread for both sole- and multi-instance
  refcounts (REQ AC-5).
- Missing and invalid-sentinel handles do not call the native function and
  finish in coherent terminal Python state (REQ AC-6).
- A constructor failure before ownership emits no spurious `ResourceWarning`
  and does not initialize the native process (REQ AC-6).
- Callback-time construction and non-terminal destruction reject without
  waiting on the process lock or changing ownership, while terminal destruction
  remains a no-op. Ordinary concurrent acquisition waits through initialization,
  then shares a successful initialization or retries a known failure. An
  ambiguous result transitions atomically into conservative shutdown, where
  racing acquisition fails fast instead of observing a synthetic token.
  Acquisition racing ordinary final shutdown also fails fast before it can
  deadlock callback draining (REQ AC-7).
- An ambiguous post-initialize interruption is propagated only after the
  conservative native outcome is balanced through final shutdown. An
  interruption during that handoff cannot strand an initialization/shutdown
  transition or Python refcount (REQ AC-7).
- An interruption during final-release handoff or final transition clear is
  propagated only after native shutdown completes and `SHUTTING_DOWN` and the
  process refcount are coherent (REQ AC-7).
