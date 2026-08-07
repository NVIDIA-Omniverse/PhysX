import json
import os

SCHEMA_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def load_usd_version():
    """Load USD version from build_info.json"""
    build_info_path = os.path.join(SCHEMA_ROOT, "_build", "target-deps", "usd", "release", "BUILD_INFO", "build_info.json")

    try:
        with open(build_info_path, 'r') as f:
            build_info = json.load(f)
            return build_info.get("usd_ver", None)
    except (FileNotFoundError, json.JSONDecodeError) as e:
        print(f"Error loading USD version from build_info.json: {e}")
        return None


def load_usd_version_from_pxr_h():
    """Load USD version from pxr/pxr.h by parsing PXR_*_VERSION defines"""
    import re
    pxr_h_path = os.path.join(SCHEMA_ROOT, "_build", "target-deps", "usd", "release", "include", "pxr", "pxr.h")

    try:
        with open(pxr_h_path, 'r') as f:
            content = f.read()
        major = re.search(r'#define\s+PXR_MAJOR_VERSION\s+(\d+)', content)
        minor = re.search(r'#define\s+PXR_MINOR_VERSION\s+(\d+)', content)
        patch = re.search(r'#define\s+PXR_PATCH_VERSION\s+(\d+)', content)
        if major and minor and patch:
            return f"{major.group(1)}.{minor.group(1)}.{patch.group(1)}"
    except FileNotFoundError as e:
        print(f"Error loading USD version from pxr.h: {e}")
    return None


def load_version_file():
    """Load version from VERSION file"""
    try:
        with open(os.path.join(SCHEMA_ROOT, "VERSION"), 'r') as f:
            return f.read().strip()
    except FileNotFoundError:
        print("Error: VERSION file not found")
        return None


usd_ver = load_usd_version_from_pxr_h()
if usd_ver is None:
    usd_ver = load_usd_version()
version = load_version_file()

if usd_ver is None:
    print("Error: Could not determine OpenUSD version (build_info.json and pxr.h not found). Skipping version check.")
    exit(1)
elif version is None:
    print("Error: Could not determine schema version (VERSION file not found). Skipping version check.")
    exit(1)
else:
    print(f"OpenUSD version: {usd_ver}")
    print(f"VERSION file: {version}")
    # VERSION is <usd_minor>.<usd_patch>.<schema_patch>; only the first two track OpenUSD.
    usd_tracking = ".".join(version.split(".")[:2])
    if usd_ver != "0." + usd_tracking:
        print(f"Error: OpenUSD version ({usd_ver}) does not match version in VERSION file ({version}). "
              "The first two components of VERSION must match OpenUSD without the leading '0.' "
              "(e.g. OpenUSD 0.25.11 -> VERSION 25.11.<patch>).")
        exit(1)
    else:
        print("OpenUSD version matches version in VERSION file")
