.. SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
.. SPDX-License-Identifier: BSD-3-Clause

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

Schema Path Registration
------------------------

.. autofunction:: ovphysx.register_schema_paths

Logging
-------

.. autofunction:: ovphysx.api.set_log_level
.. autofunction:: ovphysx.api.get_log_level
.. autofunction:: ovphysx.api.enable_default_log_output
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

Constants
---------

.. currentmodule:: ovphysx

.. py:data:: OP_INDEX_ALL

   Sentinel value (``0xFFFFFFFFFFFFFFFF``) passed to ``wait_op()`` to wait for
   all outstanding operations. Equivalent to ``OVPHYSX_OP_INDEX_ALL`` in the C API.
