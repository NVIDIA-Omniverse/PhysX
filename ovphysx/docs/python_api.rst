.. SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
.. SPDX-License-Identifier: Apache-2.0

Python API Reference
====================

.. automodule:: ovphysx.api
   :no-members:

Core Classes
------------

.. autoclass:: ovphysx.api.PhysX
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: ovphysx.api.TensorBinding
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: ovphysx.api.ContactBinding
   :members:
   :undoc-members:
   :show-inheritance:

Task Completion
---------------

``PhysX.step()``, ``PhysX.clone()``, ``PhysX.attach_ovstage()``,
``PhysX.update_from_ovstage()`` and ``PhysX.detach_ovstage()`` return an
immutable :class:`~ovphysx.api.Task` receipt. Collect one task's outcome with
:meth:`ovphysx.api.PhysX.wait_task` or every uncollected task with
:meth:`ovphysx.api.PhysX.wait_all`; both return ``None`` on success and raise
the exceptions below on failure or timeout.

Stage lifecycle tasks currently execute inline, but a returned receipt reports
admission only. Keep the Stage alive until successful detach or instance
destruction, and wait before consuming the result of attach or update.
``PhysX.is_stage_attached()`` is a synchronous ownership snapshot, not a
completion or readiness check. ``PhysX.reset_stage()`` has been removed; use
``PhysX.detach_ovstage()`` for teardown. Neither operation restores initial poses.

The session API (``query()``, ``read()``, ``write()``, ``fetch_next()``,
``commit()`` and ``close()``) returns an :class:`~ovphysx.api.Operation` without
waiting. ``Operation.wait()`` collects its native tasks and returns the typed
result; ``Operation.result`` never waits. ``wait_all()`` also collects
outstanding operations and caches their outcomes. ``read_sync()``,
``read_tokens_sync()``, ``write_sync()``, ``commit_sync()``, ``close_sync()``,
``update_articulations_kinematic()`` and ``evaluate_sdf_shapes()`` wait for their
native tasks internally. Waiting raises ``TaskError`` for accepted execution
failures. ctypes exposes the complete native receipts. CUDA producer-event and consumer-stream dependencies
remain necessary; a completed host task does not imply all GPU work is finished.

Run custom application code directly after the relevant task wait succeeds.
``wait_task(task)`` collects only that task; ``wait_all()`` also collects and
acknowledges earlier uncollected outcomes, including failures.

.. autoclass:: ovphysx.api.Task
   :members:
   :show-inheritance:

.. autoexception:: ovphysx.api.TaskError
   :members:
   :show-inheritance:

.. autoexception:: ovphysx.api.TaskTimeoutError
   :members:
   :show-inheritance:

Session Read/Write
------------------

:meth:`ovphysx.api.PhysX.query`, :meth:`ovphysx.api.PhysX.read` and
:meth:`ovphysx.api.PhysX.write` return operations whose results are the types
below. The :doc:`read_write/index` pages describe the data contract.

.. autoclass:: ovphysx.api.Operation
   :members:

.. autoclass:: ovphysx.api.Query
   :members:

.. autoclass:: ovphysx.api.ReadResult
   :members:

.. autoclass:: ovphysx.api.ReadGroup

.. autoclass:: ovphysx.api.WriteSession
   :members:

.. autoclass:: ovphysx.api.WriteGroup
   :members:

.. autoclass:: ovphysx.api.QueryMetadata
   :members:

.. autoclass:: ovphysx.api.ReadMetadata
   :members:

.. autoclass:: ovphysx.api.ReadAttributeResult
   :members:

.. autoclass:: ovphysx.api.ReadOmission
   :members:

.. autoclass:: ovphysx.api.WriteMetadata
   :members:

.. autoclass:: ovphysx.api.WriteAttributeResult
   :members:

Schema Providers
----------------

Applications install ``physx-usd-schemas`` independently and register
its two resource directories before population. See :doc:`physics_schemas`
for metadata-based discovery and native deployment recipes.

The Newton USD schema (``pip install newton-usd-schemas``) is a separate package
whose ``newton:*`` attributes ovphysx reads as fallbacks for the PhysX spellings;
this helper locates the installed package so it can be registered in the same call.

.. autofunction:: ovphysx.newton_schema_root

Logging
-------

.. autofunction:: ovphysx.api.set_log_level
.. autofunction:: ovphysx.api.get_log_level
.. autofunction:: ovphysx.api.enable_default_log_output
.. autofunction:: ovphysx.api.flush_log
.. autofunction:: ovphysx.api.enable_python_logging
.. autofunction:: ovphysx.api.disable_python_logging

Configuration
-------------

Process-global keys use :func:`ovphysx.api.set_global_config` /
:func:`ovphysx.api.get_global_config`; startup-only keys must be set before the
first ``PhysX()``. Configure every setting with a ``GlobalConfigKey`` through
``set_global_config``, and create the instance with no config args for those
settings. Set
:attr:`ovphysx.types.ConfigKey.ACTIVE_CUDA_GPUS`
with :meth:`ovphysx.api.PhysX.set_config` after create and before attach.
Environment variables remain authoritative over the matching global setter.

``PhysXConfig``, ``config=`` on ``PhysX()``, and the typed ``ConfigBool`` /
``ConfigInt32`` / ``ConfigString`` enums are the legacy path.
Carbonite-only settings, such as DirectGPU and tasking/TBB pool sizes, still
use ``PhysXConfig.carbonite_overrides``. Passing ``config=`` emits a
``DeprecationWarning`` for this legacy path.

.. autofunction:: ovphysx.api.set_global_config
.. autofunction:: ovphysx.api.get_global_config

.. automodule:: ovphysx.config
   :members:
   :undoc-members:
   :show-inheritance:

OVD recording
~~~~~~~~~~~~~

Set recording capability with
``set_global_config(GlobalConfigKey.OVD_RECORDING_CAPABLE, True)`` before the
first ``PhysX()``, attach a scene, then use ``start_recording()`` with an
explicit FILE or TCP destination. Start before the first simulation step to
capture it. Creation and attachment leave output dormant. Startup config fields,
including ``PhysXConfig.omnipvd_recording_capable``, and the legacy OVD config
module are removed. See :doc:`ovd_api_migration`.

Types and Enums
---------------

.. automodule:: ovphysx.types
   :members:
   :undoc-members:
   :show-inheritance:

DLPack Tensor Structures
------------------------

.. automodule:: ovphysx.dlpack
   :members:
   :undoc-members:
   :show-inheritance:

Physics Utilities
-----------------

.. automodule:: ovphysx.utils
   :no-members:

.. Keep ``:no-members:`` so the submodule directives below control the
   documented public surface.

Utilities are grouped by submodule. Except for the codeless schema helpers,
public names are also available directly from :py:mod:`ovphysx.utils`.

Codeless schema access
~~~~~~~~~~~~~~~~~~~~~~

Not flat re-exported. Import it as ``from ovphysx.utils import codeless``.

.. automodule:: ovphysx.utils.codeless
   :members:
   :show-inheritance:

Shape constructors
~~~~~~~~~~~~~~~~~~

.. automodule:: ovphysx.utils.shapes
   :members:

Ground and quad planes
~~~~~~~~~~~~~~~~~~~~~~

.. automodule:: ovphysx.utils.planes
   :members:

Joints
~~~~~~

.. automodule:: ovphysx.utils.joints
   :members:

Physics materials
~~~~~~~~~~~~~~~~~

.. automodule:: ovphysx.utils.materials
   :members:

Collision filtering
~~~~~~~~~~~~~~~~~~~

.. automodule:: ovphysx.utils.filtering
   :members:

Rigid bodies and colliders on existing prims
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. automodule:: ovphysx.utils.authoring
   :members:

Stage paths
~~~~~~~~~~~

.. automodule:: ovphysx.utils.paths
   :members:

Transforms
~~~~~~~~~~

.. automodule:: ovphysx.utils.transform
   :members:

Mesh construction and tetrahedral meshes
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. automodule:: ovphysx.utils.mesh
   :members:

Particles
~~~~~~~~~

.. automodule:: ovphysx.utils.particles
   :members:

Deformables
~~~~~~~~~~~

.. automodule:: ovphysx.utils.deformable
   :members:

Schema introspection
~~~~~~~~~~~~~~~~~~~~

.. automodule:: ovphysx.utils.schema
   :members:

Simulation
~~~~~~~~~~

.. automodule:: ovphysx.utils.simulation
   :members:

Metadata keys and naming constants
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. automodule:: ovphysx.utils.constants
   :members:
