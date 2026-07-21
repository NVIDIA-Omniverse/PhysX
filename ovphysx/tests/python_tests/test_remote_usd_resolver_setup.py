# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
"""Regression tests for the pre-collection remote USD resolver setup."""

import os
import sys
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

import conftest as test_conftest
import ovphysx


def test_remote_resolver_setup_selects_plugin_before_first_resolver_access(tmp_path, monkeypatch):
    install_dir = tmp_path / "install"
    bin_dir = install_dir / "bin"
    plugins_dir = install_dir / "plugins"
    resources_dir = plugins_dir / "usd" / "omni_usd_resolver" / "resources"
    bin_dir.mkdir(parents=True)
    resources_dir.mkdir(parents=True)

    dll_handles = []
    schema_registered = False
    resolver_type = SimpleNamespace(isUnknown=False)
    plugin = SimpleNamespace(
        path=str(plugins_dir / "omni_usd_resolver.dll"),
        isLoaded=False,
        Load=Mock(side_effect=lambda: pytest.fail("resolver activation must remain deferred")),
    )
    register_plugins = Mock(return_value=[plugin])
    get_plugin_for_type = Mock(return_value=plugin)
    registry = SimpleNamespace(RegisterPlugins=register_plugins, GetPluginForType=get_plugin_for_type)
    find_by_name = Mock(return_value=resolver_type)
    set_preferred_resolver = Mock()

    def register_schema_paths():
        nonlocal schema_registered
        schema_registered = True

    def registry_factory():
        assert schema_registered, "PhysX schemas must be registered before constructing Plug.Registry"
        return registry

    fake_ar = SimpleNamespace(
        SetPreferredResolver=set_preferred_resolver,
        GetUnderlyingResolver=lambda: pytest.fail("resolver activation must remain deferred"),
    )
    fake_pxr = SimpleNamespace(
        Ar=fake_ar,
        Plug=SimpleNamespace(Registry=registry_factory),
        Tf=SimpleNamespace(Type=SimpleNamespace(FindByName=find_by_name)),
    )

    monkeypatch.setattr(sys, "platform", "win32")
    monkeypatch.setattr(
        os,
        "add_dll_directory",
        lambda path: dll_handles.append(object()) or dll_handles[-1],
        raising=False,
    )
    monkeypatch.setattr(ovphysx, "register_schema_paths", register_schema_paths)
    monkeypatch.setitem(sys.modules, "pxr", fake_pxr)
    monkeypatch.setattr(test_conftest, "_remote_resolver_dll_directory_handles", [])
    monkeypatch.delenv("OVSTAGE_LIBRARY_PATH_HINT", raising=False)

    test_conftest._prepare_remote_usd_resolver(str(install_dir))

    assert schema_registered
    register_plugins.assert_called_once_with(str(resources_dir))
    find_by_name.assert_called_once_with("OmniUsdResolver")
    get_plugin_for_type.assert_called_once_with(resolver_type)
    set_preferred_resolver.assert_called_once_with("OmniUsdResolver")
    plugin.Load.assert_not_called()
    assert len(test_conftest._remote_resolver_dll_directory_handles) == 2
    assert len(dll_handles) == 2
    assert os.environ["OVSTAGE_LIBRARY_PATH_HINT"] == str(bin_dir)


def test_remote_resolver_activation_loads_selected_plugin(monkeypatch):
    default_resolver_type = type("DefaultResolver", (), {})
    selected_resolver = object()
    resolver_type = SimpleNamespace(isUnknown=False)
    plugin = SimpleNamespace(path="omni_usd_resolver", isLoaded=False)

    def load_plugin():
        plugin.isLoaded = True
        return True

    plugin.Load = Mock(side_effect=load_plugin)
    get_plugin_for_type = Mock(return_value=plugin)
    get_underlying_resolver = Mock(return_value=selected_resolver)
    fake_pxr = SimpleNamespace(
        Ar=SimpleNamespace(
            DefaultResolver=default_resolver_type,
            GetUnderlyingResolver=get_underlying_resolver,
        ),
        Plug=SimpleNamespace(
            Registry=lambda: SimpleNamespace(GetPluginForType=get_plugin_for_type),
        ),
        Tf=SimpleNamespace(Type=SimpleNamespace(FindByName=Mock(return_value=resolver_type))),
    )
    monkeypatch.setitem(sys.modules, "pxr", fake_pxr)

    test_conftest._activate_remote_usd_resolver()

    plugin.Load.assert_called_once_with()
    get_plugin_for_type.assert_called_once_with(resolver_type)
    get_underlying_resolver.assert_called_once_with()
