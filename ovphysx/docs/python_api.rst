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

Codeless Schema Discovery
-------------------------

ovphysx ships its PhysX USD schemas as codeless resources and never registers
them itself; these helpers tell the application where they are so it can
register them with the USD runtime it owns (see :doc:`physics_schemas`).

.. autofunction:: ovphysx.codeless_schema_root
.. autofunction:: ovphysx.codeless_schema_paths

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

.. automodule:: ovphysx.config
   :members:
   :undoc-members:
   :show-inheritance:

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

Contact Report Structures
-------------------------

ctypes mirrors of the C ABI structs returned by
:py:meth:`ovphysx.api.PhysX.get_contact_report`; see the :doc:`C API Reference <api>`
for full field semantics.

.. automodule:: ovphysx.contact_types
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

Constants
---------

.. currentmodule:: ovphysx

.. py:data:: OP_INDEX_ALL

   Sentinel value (``0xFFFFFFFFFFFFFFFF``) passed to ``wait_op()`` to wait for
   all outstanding operations. Equivalent to ``OVPHYSX_OP_INDEX_ALL`` in the C API.
