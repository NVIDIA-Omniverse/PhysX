<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-PYTHON-LIFECYCLE-001
title: Checked PhysX Instance Destruction
status: implemented
owner: ovphysx
---

## Description

The main Python `PhysX` object exposes `destroy()` as its canonical explicit
lifecycle operation, matching the destruction vocabulary used by its child
objects. Destruction is checked, deterministic, and idempotent. It invalidates
the Python handle and attached-stage keepalive, lets the native instance clean up
all child bindings, and discharges process-lifecycle ownership exactly once.

The native `omni_sdk_physx_destroy()` implementation has an important status
invariant: its only non-success return occurs when the handle is already absent
from the instance registry. Every path for a live handle erases it before
returning success. Python therefore commits terminal state after any returned
native status and then reports a non-success status. It retains ownership only
when the Python or ctypes invocation itself raises before returning a status, so
the caller can retry. This commit-then-report rule prevents a stale native handle
from leaking the process-lifecycle token.

Process shutdown is a second cleanup phase. It runs only after native destruction
and cannot make the destroyed instance live again. Python clears lifecycle
ownership before invoking shutdown; a shutdown exception reports that the
instance was destroyed but process shutdown failed, and a later `destroy()` is a
no-op rather than a second refcount decrement.

ovphysx 0.6 intentionally removes the former `release()` spelling and the
main-object context-manager protocol. Callers must use `destroy()` explicitly,
normally from a `finally` block. This is a deliberate pre-1.0 Python source
break that leaves one deterministic lifecycle operation. Because destruction is
checked, a cleanup failure raised from `finally` becomes the active exception;
Python preserves an in-flight exception as chained context. `__del__` remains
only as a non-throwing safety net for missed explicit cleanup.

## Acceptance Criteria

- AC-1: `PhysX.destroy()` is the canonical public operation, returns `None` on
  success, destroys the native instance, clears the handle and attached-stage
  keepalive, invalidates child bindings, marks the instance destroyed, and is an
  idempotent no-op on every later call.

- AC-2: After native destruction returns any status, Python commits terminal
  state and discharges process-lifecycle ownership. A non-success status raises
  `RuntimeError` containing the numeric status and a known status name, states
  that the instance is no longer registered, and directs the caller to the log.
  It must not report the thread-local last-error string because this native call
  deliberately does not set one. If process shutdown also fails, the
  process-global shutdown failure is reported and its message also includes the
  returned native status name and numeric value.

- AC-3: A Python or ctypes exception raised while invoking native destruction,
  before a status is returned, leaves the native handle, attached-stage
  keepalive, destroyed marker, and lifecycle ownership unchanged so a later
  `destroy()` can retry. If the native call completed before conversion raised,
  the retry converges through AC-2's terminal already-absent status. The native
  already-absent path performs no teardown and leaves live peers and
  process-global asynchronous state unchanged.

- AC-4: Process-lifecycle ownership is discharged exactly once per instance.
  Destroying one of several simultaneous instances leaves the other instances
  usable. A process-shutdown exception raises a distinct `RuntimeError` saying
  that the instance was destroyed but process shutdown failed; the instance
  remains terminal and later destruction does not retry shutdown.

- AC-5: `PhysX` exposes neither `release()` nor `__enter__` / `__exit__`; the
  runtime class and public type stub agree on that surface. `__del__` applies
  the `destroy()` cleanup policy, preserves the existing `ResourceWarning` for
  missing deterministic cleanup, and suppresses cleanup exceptions. If its first
  native invocation raises while lifecycle ownership remains live, `__del__`
  makes one immediate best-effort retry so transient or post-call conversion
  failures can converge without a caller. Returned-status and shutdown failures
  have already committed terminal state and are not retried. If both native
  invocations raise before returning a status, the finalizer suppresses them,
  emits a `RuntimeWarning` that the native instance may remain registered,
  abandons the unknowable native handle state, and discharges process-lifecycle
  ownership so the Python token cannot outlive the Python object. The token is
  process-lifecycle accounting rather than native-instance ownership; retaining
  it cannot make an otherwise unreachable native handle recoverable. If the
  finalizer runs from native log-callback delivery, it retains the object while
  deferring the same cleanup policy off the callback thread; callback delivery
  does not wait for that cleanup.

- AC-6: Destruction is safe during partial construction. A missing, null, or
  invalid-sentinel native handle causes no native destroy call, establishes
  terminal Python state, and discharges lifecycle ownership only if that
  ownership was acquired.

- AC-7: Constructing `PhysX` or destroying a live/partially constructed
  instance from a native log callback raises `RuntimeError` before waiting on
  the process-lifecycle lock or mutating instance/lifecycle ownership. The
  caller can retry after callback delivery returns. Concurrent construction
  racing process initialization waits on a condition while native work runs
  without the condition lock. Success wakes waiters to share the initialized
  process; a known failure wakes one to retry initialization. An ambiguous
  result atomically hands ownership to conservative final shutdown, and racing
  construction fails fast until that rollback finishes. Construction racing
  ordinary final shutdown also fails fast so callback dependencies cannot
  deadlock the callback drain. A callback must not synchronously wait for work
  that may emit into the same serialized callback registration. Native
  initialization must not invoke callbacks or wait for callback delivery;
  changes to that invariant require revisiting the wait policy. If Python is
  interrupted after native initialization has an ambiguous outcome, it
  conservatively balances that outcome through final shutdown before
  propagating the original exception. Destroying an
  already terminal instance remains an idempotent no-op in callback context.
  Missed explicit cleanup that reaches `__del__` in callback context follows
  AC-5's deferred finalizer path instead of stranding its lifecycle token.

- AC-8: After the first successful native instance construction, Python
  registers one `atexit` callback and tracks every live `PhysX` instance in
  an identity-keyed weak registry that does not hash or compare the object,
  so an `__eq__`-only or equal-hashable subclass cannot raise, coalesce, or
  extend ordinary garbage-collection lifetime. Terminal `destroy()` untracks
  the instance. At normal interpreter exit the callback snapshots all
  remaining instances and applies AC-5's non-throwing finalizer policy to
  each before `sys.is_finalizing()` becomes true. After those instances are
  destroyed, any process-lifecycle tokens still held by exit-reachable
  `read()` Warp arrays are collapsed and take the existing
  `ovphysx_shutdown()` path while ctypes is still valid, rather than leaving
  direct-runtime teardown to C++ static destruction. The library does not
  replace application signal handlers; an uncaught `KeyboardInterrupt`
  follows normal interpreter exit, while termination that bypasses Python
  `atexit` is outside this guarantee. Forking after constructing `PhysX` is
  also outside this guarantee: the library does not install
  `os.register_at_fork` handlers, and a child must not run the inherited
  `atexit` teardown against native state it does not own.

## Test References

- TEST-PYTHON-LIFECYCLE-001
- TEST-PYTHON-LIFECYCLE-002

## Code References

- ovphysx/python/ovphysx/api.py (`PhysX.destroy`, `PhysX.__del__`)
- ovphysx/python/ovphysx/api.pyi
- ovphysx/src/ovphysx/ovphysx.cpp (`omni_sdk_physx_destroy` returned-status invariant)
- ovphysx/tests/python_tests/test_lifecycle_failure_injection.py
- ovphysx/tests/python_tests/lifecycle_tests/test_destroy.py
- ovphysx/tests/python_tests/lifecycle_tests/test_destroy_no_warning.py
- ovphysx/tests/python_tests/lifecycle_tests/test_resource_warning.py
- ovphysx/tests/python_tests/lifecycle_tests/test_process_instance_registry.py
- ovphysx/tests/python_tests/test_type_stubs.py
- ovphysx/tests/python_tests/cpu_tests/test_log_level.py
- ovphysx/tests/python_tests/test_ujitso_cooking_cache.py
- ovphysx/tests/c_unittests/test_global_lifecycle.cpp

## Dependencies

- None
