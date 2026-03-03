# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

"""
USD Version Detection and Compatibility Checking

This module provides runtime validation of USD version compatibility,
schema availability, and library preloading.
It uses Python's import system to detect USD and validate version requirements.

Dependencies:
    - packaging: Standard Python packaging tooling for PEP 440 version checking
"""

import importlib.util
import logging
import os
import sys
from pathlib import Path
from typing import List, Optional, Tuple

_logger = logging.getLogger(__name__)
_PXR_PLUGINS_RELOADED = False


def get_usd_version() -> str:
    """
    Get USD version from loaded or available USD installation.

    Returns:
        Version string in format "XX.YY" or similar

    Raises:
        ImportError: If USD cannot be imported
    """
    _ensure_pxr_plugin_path()
    # Importing pxr is intentional here; callers are expected to gate on _pxr_available().
    # If USD is not importable, this should raise ImportError rather than silently masking it.
    from pxr import Usd

    # Try standard USD API
    try:
        version = Usd.GetVersion()
        # Normalize tuple returns like (0, 25, 11) to "0.25.11"
        if isinstance(version, tuple):
            return ".".join(str(part) for part in version)
        return str(version)
    except AttributeError:
        # GetVersion() method doesn't exist in this USD version
        pass
    except Exception as e:  # noqa: BLE001
        # Intentional broad catch: USD API may raise unexpected errors (RuntimeError, TypeError, etc.)
        # We log and continue to next detection method rather than failing entirely
        _logger.debug("Unexpected error calling Usd.GetVersion(): %s", e, exc_info=True)

    # Try pxr.__version__ attribute
    try:
        import pxr

        if hasattr(pxr, "__version__"):
            return str(pxr.__version__)
    except (ImportError, ModuleNotFoundError, AttributeError):
        # pxr module not available or __version__ attribute missing/invalid
        pass
    except Exception as e:  # noqa: BLE001
        # Intentional broad catch: version attribute access may raise unexpected errors
        # We log and continue to next detection method rather than failing entirely
        _logger.debug("Unexpected error accessing pxr.__version__: %s", e, exc_info=True)

    # Fallback: parse from library path
    try:
        import pxr.Tf

        version = _parse_version_from_path(pxr.Tf.__file__)
        if version:
            return version
    except (ImportError, ModuleNotFoundError, AttributeError, OSError):
        # pxr.Tf module not available, __file__ missing, or path access error
        pass
    except Exception as e:  # noqa: BLE001
        # Intentional broad catch: path parsing may raise unexpected errors (e.g., from regex)
        # We log and continue rather than failing (returns "unknown" as final fallback)
        _logger.debug("Unexpected error parsing version from pxr.Tf path: %s", e, exc_info=True)

    return "unknown"


def _pxr_available() -> bool:
    """Return True if pxr can be imported in this environment."""
    return importlib.util.find_spec("pxr") is not None


def _ensure_pxr_plugin_path() -> None:
    """Ensure PXR_PLUGINPATH_NAME is set so schema plugins can be discovered."""
    candidates = []
    plugins_dir = os.environ.get("OVPHYSX_PLUGINS_DIR")
    if plugins_dir:
        candidates.append(Path(plugins_dir) / "usd")
    lib_path = os.environ.get("OVPHYSX_LIB")
    if lib_path:
        try:
            candidates.append(Path(lib_path).resolve().parent.parent / "plugins" / "usd")
        except OSError:
            pass
    module_dir = Path(__file__).parent
    candidates.append(module_dir / "plugins" / "usd")
    existing = os.environ.get("PXR_PLUGINPATH_NAME", "")
    paths = [p for p in existing.split(os.pathsep) if p]
    appended = False
    for candidate in candidates:
        if not candidate.exists():
            continue
        candidate_str = str(candidate)
        if candidate_str not in paths:
            paths.append(candidate_str)
            appended = True
    if appended or (paths and existing == ""):
        os.environ["PXR_PLUGINPATH_NAME"] = os.pathsep.join(paths)


def _reload_pxr_plugins() -> None:
    """Reload USD plugin registry after setting PXR_PLUGINPATH_NAME."""
    global _PXR_PLUGINS_RELOADED
    if _PXR_PLUGINS_RELOADED:
        return
    try:
        from pxr import Plug
    except Exception as e:  # noqa: BLE001
        _logger.debug("USD plugin reload failed: %s", e, exc_info=True)
        return
    registry = Plug.Registry()
    if hasattr(registry, "ReloadPlugins"):
        try:
            registry.ReloadPlugins()
            _PXR_PLUGINS_RELOADED = True
        except Exception as e:  # noqa: BLE001
            _logger.debug("USD plugin reload failed: %s", e, exc_info=True)
            return


def _parse_version_from_path(path: str) -> Optional[str]:
    """
    Parse USD version from file path as fallback.

    Looks for patterns like /path/to/usd-23.11/lib/python/...
    """
    import re

    patterns = [
        r"usd[-_](\d+)[._](\d+)(?:[._](\d+))?",
        r"USD[-_](\d+)[._](\d+)(?:[._](\d+))?",
    ]

    for pattern in patterns:
        match = re.search(pattern, path, re.IGNORECASE)
        if match:
            # Convert to int to normalize (removes leading zeros)
            major = int(match.group(1))
            minor = int(match.group(2))
            patch = int(match.group(3)) if match.group(3) else None
            return f"{major}.{minor}.{patch}" if patch else f"{major}.{minor}"

    return None


def _normalize_usd_version(version: str) -> str:
    """Normalize USD version string for compatibility comparisons.

    Converts OpenUSD-style "0.xx.yy" to "xx.yy" when appropriate.
    """
    if not version or version == "unknown":
        return version
    parts = version.split(".")
    if len(parts) >= 2 and parts[0] == "0":
        try:
            minor = int(parts[1])
        except ValueError:
            return version
        if minor >= 20:
            normalized = [parts[1], parts[2] if len(parts) > 2 else "0"]
            if len(parts) > 3:
                normalized.extend(parts[3:])
            return ".".join(normalized)
    return version


def check_compatibility(loaded_version: str, requirement_spec: str) -> bool:
    """
    Check if USD version satisfies requirement specification.

    Uses PEP 440 version specifier format for compatibility checking.
    Requires the 'packaging' module (standard Python packaging tooling).

    Args:
        loaded_version: Version string of loaded USD (e.g., "23.11")
        requirement_spec: PEP 440 version spec (e.g., "==25.11")

    Returns:
        True if version is compatible, False otherwise

    Raises:
        ImportError: If packaging module is not available (indicates broken environment)
    """
    if loaded_version == "unknown":
        _logger.warning("Cannot verify compatibility with unknown USD version")
        return True  # Conservative: allow unknown versions with warning

    from packaging.specifiers import SpecifierSet
    from packaging.version import Version

    spec = SpecifierSet(requirement_spec)
    normalized_version = _normalize_usd_version(loaded_version)
    version = Version(normalized_version)
    return version in spec


def format_compatibility_error(loaded_version: str, loaded_path: Optional[str], config: dict) -> str:
    """Format detailed error message for USD version incompatibility."""
    lib_name = config["library"]["name"]
    lib_version = config["library"]["version"]
    required_spec = config["usd"]["version_spec"]

    error_msg = f"""
{'═' * 70}
USD VERSION INCOMPATIBILITY DETECTED
{'═' * 70}

Library: {lib_name} v{lib_version}
  Required USD: {required_spec}
  
Current Process:
  USD Version: {loaded_version} (INCOMPATIBLE)"""

    if loaded_path:
        error_msg += f"\n  Loaded From: {loaded_path}"

    error_msg += f"""
  
USD can only be loaded once per process. To fix this:
  1. Install compatible USD version: {required_spec}
  2. Update your PYTHONPATH to use compatible USD
  3. Check other libraries' USD requirements for conflicts

For more information, see documentation on USD version management.
{'═' * 70}
"""

    return error_msg


def check_schema_available(schema_name: str) -> Tuple[bool, Optional[str]]:
    """
    Check if a USD schema is available and get its version if possible.

    Args:
        schema_name: Schema type name (e.g., "UsdPhysics", "PhysxSchema")

    Returns:
        Tuple of (is_available, version_string)
    """
    try:
        module_available = _preload_schema_module(schema_name)
        _reload_pxr_plugins()

        # Some schema "names" map to USD modules rather than concrete Tf types.
        if schema_name in {"UsdPhysics", "PhysxSchema"}:
            return (module_available, None)

        from pxr import Tf

        # Check if schema type is registered with USD's type system
        schema_type = Tf.Type.FindByName(schema_name)
        if not schema_type or schema_type.isUnknown:
            return (False, None)

        # Schema is available - try to get version if exposed
        # Most schemas don't expose runtime version info, so this often returns None
        version = _get_schema_version(schema_name)
        return (True, version)

    except ImportError:
        return (False, None)
    except Exception as e:
        _logger.warning("Error checking schema %s: %s", schema_name, e)
        return (False, None)


def _get_schema_version(schema_name: str) -> Optional[str]:
    """
    Attempt to get schema version.

    Note: Most USD schemas don't expose runtime version info.
    This is best-effort only.
    """
    # Schema versioning is not standardized in USD
    # Most schemas don't expose version at runtime
    # Future enhancement: check plugInfo.json for version metadata
    return None


def _preload_schema_module(schema_name: str) -> bool:
    """Best-effort import to register schema types before lookup."""
    module_map = {
        "UsdPhysics": "pxr.UsdPhysics",
        "PhysxSchema": "pxr.PhysxSchema",
    }
    module_name = module_map.get(schema_name)
    if not module_name:
        return False
    try:
        import importlib

        importlib.import_module(module_name)
        return True
    except Exception as e:  # noqa: BLE001
        _logger.debug("Schema preload failed for %s (%s): %s", schema_name, module_name, e, exc_info=True)
        return False


def validate_schemas(config: dict, search_paths: List[str]) -> None:
    """
    Validate that required schemas are available.

    Args:
        config: Library configuration dictionary
        search_paths: Paths to search for schema plugins if loading needed

    Raises:
        RuntimeError: If required schema is unavailable
    """
    schema_config = config.get("schemas", {})
    required_schemas = schema_config.get("required", [])

    if not required_schemas:
        return  # No schema requirements

    for schema_req in required_schemas:
        schema_name = schema_req.get("name")
        is_required = schema_req.get("required", True)
        version_spec = schema_req.get("version_spec")
        source = schema_req.get("source", "unknown")

        # Check if schema is available
        is_available, schema_version = check_schema_available(schema_name)

        if not is_available:
            message = (
                f"Schema '{schema_name}' not found.\n"
                f"  Source: {source}\n"
                f"  Required: {is_required}\n"
                "Ensure the schema plugin is loaded before using this library."
            )

            if is_required:
                raise RuntimeError(f"REQUIRED SCHEMA MISSING:\n{message}")
            else:
                _logger.warning("Optional schema missing:\n%s", message)
                continue

        # Check version if available and specified
        if schema_version and version_spec:
            if not check_compatibility(schema_version, version_spec):
                message = (
                    "Incompatible schema version.\n"
                    f"  Schema: {schema_name}\n"
                    f"  Found: {schema_version}\n"
                    f"  Required: {version_spec}"
                )

                if is_required:
                    raise RuntimeError(message)
                else:
                    _logger.warning("Incompatible optional schema version: %s", message)
        elif version_spec and not schema_version:
            # Version specified but couldn't determine it
            _logger.info(
                "Cannot verify version for %s (version info not exposed at runtime)",
                schema_name,
            )


def check_usd_compatibility():
    """
    Main entry point for USD compatibility checking (opportunistic).

    Only validates USD if it's already loaded in Python (sys.modules).
    If USD is not yet loaded, defers to C++ library which will preload USD
    from SDK-shipped libraries. This allows tests and wheel users to work
    without requiring a separate pip install of usd-core.

    Raises:
        RuntimeError: If USD is loaded but incompatible
    """
    # Only validate when USD is already loaded in Python.
    # IMPORTANT: Do NOT call _ensure_pxr_plugin_path() before this guard.
    # Setting PXR_PLUGINPATH_NAME unconditionally at import time causes other
    # USD-hosting libraries (e.g., OVRTX) to discover and load ovphysx's bundled
    # schema plugins, transitively pulling in a second copy of USD and triggering
    # a fatal TF_DEBUG duplicate symbol abort.
    if "pxr" not in sys.modules:
        return

    _ensure_pxr_plugin_path()

    # Load library configuration
    module_dir = Path(__file__).parent

    # Try lib/ subdirectory first (wheel structure matches _install/)
    config_path = module_dir / "lib" / "config.toml"

    if not config_path.exists():
        # Try module root (legacy)
        config_path = module_dir / "config.toml"

    if not config_path.exists():
        # Try repo root: python/ovphysx/ -> python/ -> omni/ovphysx/
        sdk_root = module_dir.parent.parent
        config_path = sdk_root / "config.toml"

    if not config_path.exists():
        _logger.warning("config.toml not found, skipping version check")
        return

    # Parse TOML config file
    try:
        import tomllib  # Python 3.11+
    except ImportError:
        try:
            import tomli as tomllib  # Fallback for Python <3.11
        except ImportError:
            _logger.warning("TOML parser not available, skipping version check")
            return

    with open(config_path, "rb") as f:
        config = tomllib.load(f)

    if not config.get("usd", {}).get("required", False):
        return  # USD not required

    lib_name = config["library"]["name"]
    required_spec = config["usd"]["version_spec"]

    # Validate already-loaded USD version
    try:
        version = get_usd_version()
        _reload_pxr_plugins()

        # Validate compatibility
        if not check_compatibility(version, required_spec):
            # Get path for error message
            loaded_path = None
            try:
                import pxr.Tf

                loaded_path = str(Path(pxr.Tf.__file__).parent)
            except (ImportError, AttributeError, TypeError):
                # pxr.Tf not available, __file__ missing, or path conversion failed
                pass

            error_msg = format_compatibility_error(version, loaded_path, config)
            raise RuntimeError(error_msg)

        _logger.info("[%s] Using USD %s", lib_name, version)

    except ImportError:
        # USD was in sys.modules but can't be imported - unusual but non-fatal
        # Let C++ handle loading
        pass

    # Validate required schemas are available
    # Note: Library preloading happens in ovphysx_load_tensor_plugins() (C++)
    # which is called later when tensor functionality is first used
    try:
        _ensure_pxr_plugin_path()
        validate_schemas(config, [])
    except RuntimeError:
        # Required schema missing - this is fatal
        raise
    except Exception as e:
        # Unexpected error during schema validation - warn but continue
        _logger.warning("[%s] Schema validation error: %s", lib_name, e)
