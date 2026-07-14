# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
"""Silence USD's TfScriptModuleLoader warnings for pxr.<Name> Python peers
that the ovphysx wheel deliberately does not ship.

Background
----------
USD's ``TfScriptModuleLoader`` (``pxr/base/tf/scriptModuleLoader.cpp``) walks
every registered library at USD bootstrap and tries to ``PyImport_ImportModule``
each library's declared Python peer. The registrations happen at static-init
time inside the libraries themselves, via ``TF_REGISTRY_FUNCTION``:

  - ``libphysxSchema.so``           registers ``pxr.PhysxSchema``
  - ``libphysicsSchemaTools.so``    registers ``pxr.PhysicsSchemaTools``
  - ``libov_25.11usd_ms.so``        (monolithic USD) registers ``pxr.CameraUtil``,
                                    ``pxr.PxOsd``, and other bundled-USD names

In environments where the matching Python peer is installed (for example a
full USD/Kit-style runtime or an editable schema dev install that includes the
relevant Python wrappers), the auto-import succeeds and USD wires the
library's C++ side to its Python wrapper.

In environments where the Python peer is NOT installed (the released ovphysx
wheel ships C++ libraries plus a tensor-API Python surface, but no ``pxr/``
Python tree -- see ``deps_manifest.toml`` exclusions of ``_physxSchema.*`` and
``_physicsSchemaTools.*``), the auto-import fails and USD prints one
``Error loading lib 'X's module 'pxr.Y':\\n... No module named 'pxr.Y'``
warning per registered library. The warnings are functionally harmless --
nothing in ovphysx's surface (the tensor API, ``ovphysx.PhysX``,
``ovphysx.ContactBinding``, etc.) needs ``pxr.<Name>`` Python wrappers -- but
they spam stderr at startup and confuse users who think something failed.

What this module does
---------------------
For each ``pxr.<Name>`` Python peer that the ovphysx wheel does not ship, we
first check whether the real module is already in ``sys.modules`` or
findable on the active import path (via ``importlib.util.find_spec``). For a
dotted name, ``find_spec`` imports parent packages as needed to inspect their
``__path__``, but it does NOT execute the target leaf module's body. If a real
leaf module is reachable, we leave the name alone. If it is not, we install a
small stub module under that exact child-module name in ``sys.modules``. If no
real top-level ``pxr`` package exists, we also synthesize the minimal parent
package required by CPython's ``PyImport_ImportModule("pxr.X")`` path, using a
dynamic ``__path__`` so real ``pxr.*`` children added to ``sys.path`` later are
still discoverable. The stub:

  - makes ``PyImport_ImportModule("pxr.X")`` succeed silently, killing the
    TfScriptModuleLoader warning
  - raises an informative ``AttributeError`` on any non-dunder attribute
    access, so user code that genuinely needs the missing bindings still
    fails with a useful message instead of a silent no-op

Limitations and planned removal
-------------------------------
This is a runtime workaround at the ovphysx wheel boundary. It does NOT
modify the underlying schema source (``omni/schema/source/*/moduleDeps.cpp``)
or the monolithic USD bundle (``libov_25.11usd_ms.so``), which is built
outside this repository.

This file is **intended to be deleted** once the underlying fixes land.
There are two parts:

  - **Stop building the PhysX schema C++ plugins.** Defining the PhysX
    schemas codelessly removes
    the ``libphysxSchema.so`` and ``libphysicsSchemaTools.so`` C++ plugins
    entirely, which removes the ``RegisterLibrary(..., \"pxr.PhysxSchema\", ...)``
    / ``RegisterLibrary(..., \"pxr.PhysicsSchemaTools\", ...)`` calls at the
    source. No registration, no auto-import attempt, no warning. That kills
    two of the four ``pxr.<Name>`` warnings this module is designed to
    silence (``pxr.PhysxSchema`` and ``pxr.PhysicsSchemaTools``).

  - **Bundled USD runtime cleanup.** The remaining two warnings
    (``pxr.CameraUtil`` and ``pxr.PxOsd``) come from upstream USD libraries
    bundled inside the monolithic ``libov_25.11usd_ms.so``. They are not
    in this repo. The follow-up is either to rebuild the bundle with
    those registrations stripped (or with ``TfScriptModuleLoader``
    Python-peer registration disabled wholesale) or to ship the upstream
    ``pxr.CameraUtil`` / ``pxr.PxOsd`` Python bindings in the wheel. To be
    coordinated with the owners of that bundled USD runtime once the
    schema-side noise is gone.

Once the schema plugins stop being built and the bundled-USD warnings are
dealt with, delete this module, drop the ``_pxr_stubs`` import block from
``ovphysx/__init__.py``, and remove the matching test file. The warnings
will no longer be produced at all.

Must be invoked BEFORE any code that touches USD natively. Today that means
before ovphysx's lazy native bootstrap (gated behind ``__getattr__`` for
``PhysX``, ``ContactBinding``, etc.). The call site is the top of
``ovphysx/__init__.py``.
"""

from __future__ import annotations

import importlib.util as _importlib_util
import importlib.machinery as _importlib_machinery
import os as _os
import sys as _sys
import types as _types


# pxr.<Name> Python peer modules the ovphysx wheel does not ship.
# Update this list if a new library that registers a pxr.X peer is bundled
# without its Python wrapper.
_MISSING_PXR_PEERS: tuple[str, ...] = (
    "pxr.CameraUtil",
    "pxr.PhysicsSchemaTools",
    "pxr.PhysxSchema",
    "pxr.PxOsd",
)


class _DynamicPxrPath(list):
    """Package search path for a synthetic ``pxr`` parent.

    The synthetic parent is only installed when no real ``pxr`` package is
    importable, but some embedding applications may add a real USD Python tree
    to ``sys.path`` later. A plain ``__path__ = []`` would block subsequent
    ``import pxr.Usd`` style imports through the already-loaded parent. This
    list refreshes from the live ``sys.path`` whenever importlib iterates it.
    """

    def _refresh(self) -> None:
        paths = []
        seen = set()
        for entry in _sys.path:
            root = entry or _os.getcwd()
            candidate = _os.path.join(root, "pxr")
            if candidate in seen or not _os.path.isdir(candidate):
                continue
            seen.add(candidate)
            paths.append(candidate)
        self[:] = paths

    def __iter__(self):
        self._refresh()
        return super().__iter__()

    def __len__(self) -> int:
        self._refresh()
        return super().__len__()

    def __getitem__(self, index):
        self._refresh()
        return super().__getitem__(index)


def _make_parent_package_stub(name: str) -> _types.ModuleType:
    parent = _types.ModuleType(name)
    parent.__package__ = name
    parent.__loader__ = None
    parent.__ovphysx_synthetic_parent__ = True
    path = _DynamicPxrPath()
    parent.__path__ = path
    spec = _importlib_machinery.ModuleSpec(name, loader=None, is_package=True)
    spec.submodule_search_locations = path
    parent.__spec__ = spec
    return parent


class _OvphysxMissingPxrStub(_types.ModuleType):
    """Empty stub for a ``pxr.<Name>`` module that the ovphysx wheel does not ship.

    Importing the module name succeeds (which is what we want -- it silences
    USD's auto-import warning), but any actual attribute access raises an
    informative ``AttributeError`` so user code that genuinely needs the
    missing bindings does not silently no-op.
    """

    def __getattr__(self, attr: str):
        if attr.startswith("_"):
            raise AttributeError(attr)
        # Be precise about where the real bindings actually come from:
        # pxr.PhysxSchema and pxr.PhysicsSchemaTools are NVIDIA PhysX schema
        # extensions. pxr.CameraUtil and pxr.PxOsd are upstream OpenUSD imaging
        # modules. None of the four are shipped by PyPI usd-core (verified for
        # usd-core 25.11 and 26.5).
        is_nvidia_only = self.__name__ in (
            "pxr.PhysxSchema",
            "pxr.PhysicsSchemaTools",
        )
        if is_nvidia_only:
            where = (
                "The real module is an NVIDIA PhysX schema Python binding. "
                "It is not provided by PyPI usd-core; it only exists in "
                "NVIDIA builds that include the PhysX schema Python wrappers."
            )
        else:
            where = (
                "The real module is an upstream OpenUSD imaging Python binding. "
                "It is not provided by PyPI usd-core; it requires a fuller "
                "USD/Kit-style runtime that includes the imaging Python wrappers."
            )
        raise AttributeError(
            f"module {self.__name__!r} is an empty stub provided by the ovphysx "
            f"wheel; ovphysx does not ship Python bindings for {self.__name__}. "
            f"{where} Note: ovphysx runs a namespaced USD runtime that is "
            f"intentionally isolated from host USD Python bindings. A pxr.* "
            f"binding from a host USD runtime is not namespace-compatible with "
            f"ovphysx's internal USD stage, and ovphysx does not expose that "
            f"stage as a pxr.UsdStage Python object. Use ovphysx's own Python "
            f"API to interact with ovphysx state."
        )


def _real_module_reachable(fullname: str) -> bool:
    """Return True if ``fullname`` is already loaded or findable on the active
    import path without executing the target leaf module's body.

    ``importlib.util.find_spec`` consults the configured finders for the
    given name. For a dotted name it imports the parent package(s) along the
    way (necessary to consult their ``__path__``), but it never executes the
    target submodule's body. That makes it safe against the side effects we
    care about here: it does not run the leaf module code that could trigger
    USD plugin discovery or Kit/native bootstrap just because ``ovphysx`` was
    imported.
    """
    if fullname in _sys.modules:
        return True
    try:
        spec = _importlib_util.find_spec(fullname)
    except Exception:  # noqa: BLE001 -- treat parent-package errors as "unreachable"
        return False
    return spec is not None


def _install_pxr_stub(fullname: str) -> None:
    """Install a no-op stub for ``fullname`` under sys.modules, if and only if
    the real Python module is not already loaded and not findable on the
    active import path. Uses a leaf-non-executing probe (``find_spec``) so that
    plain ``import ovphysx`` never executes the real ``pxr.<Name>`` leaf module
    body in environments where it happens to be available.
    """
    if _real_module_reachable(fullname):
        return

    parent_name, _, leaf = fullname.rpartition(".")
    if parent_name and parent_name not in _sys.modules:
        # PyImport_ImportModule("pxr.X") still imports the parent package first
        # even when sys.modules already contains "pxr.X". A minimal parent is
        # therefore necessary in environments with no real pxr package.
        try:
            parent_spec = _importlib_util.find_spec(parent_name)
        except Exception:  # noqa: BLE001
            parent_spec = None
        if parent_spec is None and parent_name not in _sys.modules:
            _sys.modules[parent_name] = _make_parent_package_stub(parent_name)

    stub = _OvphysxMissingPxrStub(fullname)
    _sys.modules[fullname] = stub
    if parent_name:
        parent = _sys.modules.get(parent_name)
        if parent is not None:
            try:
                setattr(parent, leaf, stub)
            except (AttributeError, TypeError):
                # Tolerate read-only or non-attribute-bearing parent modules.
                pass


def install_pxr_stubs() -> None:
    """Idempotently install no-op stubs for every pxr.<Name> peer this wheel
    does not ship. Safe to call multiple times; safe to call before or after
    ``register_schema_paths()``.
    """
    for fullname in _MISSING_PXR_PEERS:
        _install_pxr_stub(fullname)
