# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import os
import argparse
import logging
import re
from typing import Dict, Callable
import fnmatch


logger = logging.getLogger("omni.repo.version_reset")


def matches(ext_id: str, include, exclude):
    return any(fnmatch.fnmatch(ext_id, p) for p in include) and not any(
        fnmatch.fnmatch(ext_id, p) for p in exclude
    )


def update_version_in_ext_config(path, new_version):
    # Read it
    content = None
    with open(path, "r") as f:
        content = f.read()

    # Replace actual setting with regexp:
    p = re.compile(r'(version\s*=\s*"[^"]*")')
    if p.search(content):
        content = p.sub(f'version = "{new_version}"', content, 1) # Only replace the first occurrence (in [package])
        with open(path, "w") as f:
            f.write(content)


def lock_version_in_ext_config(path, new_version, include, exclude):
    # Read it
    with open(path, "r") as f:
        lines = f.read().splitlines()
        dependencies_lines = False
        pattern = re.compile('\"(?P<extname>\S+)\"\s*=\s*{\s*(?:\s*,?\s*(version\s*=\s*"[^"]*")|\s*,?\s*(exact\s*=\s*true)|\s*,?\s*(?P<optional>optional\s*=\s*true))*\s*}')

        for index, line in enumerate(lines):
            if not dependencies_lines:
                if "[dependencies]" in line:
                    dependencies_lines = True
            else:
                if "[" in line:
                    break

                match = pattern.search(line)
                if match:
                    extname = match.group("extname")
                    if matches(extname, include, exclude):
                        optional = match.group("optional")
                        optional = f", {optional}" if optional else ""
                        lines[index] = f"\"{extname}\" = {{ version = \"{new_version}\", exact = true{optional} }}"
                elif line != "" and line != " " and not line.startswith("#"):
                    print(f"Error: did not match `{line}` as an extension dependency line!")

    with open(path, "w") as f:
        for line in lines:
            f.write(f"{line}\n")


def update_version_file(path, new_version):
    with open(path, "w") as f:
        f.write(new_version)


def get_all_extensions(root):
    ext_folders = [
        "extensions/runtime/source",
        "extensions/ux/source",
    ]
    for folder in ext_folders:
        extensions_root = os.path.join(root, folder)
        extensions_full = os.listdir(extensions_root)
        for ext in extensions_full:
            ext_path = f"{extensions_root}/{ext}"
            if os.path.isdir(ext_path):
                yield ext, ext_path


def run(tool_config, options, root, publish_config):
    print("Running repo version reset")
    group = "physx"
    root_exts = tool_config.get("exts")
    if group not in root_exts:
        print(f"Error: group \"{group}\" has not been defined in the [repo_version_reset] section of the repo.toml file")
        return
    process_group(tool_config, options, root, root_exts, group)


def process_group(tool_config, options, root, root_exts, group):
    exts = root_exts.get(group)
    print("Resetting with extensions filter: " + str(exts))
    print("Resetting to version: " + str(options.version))
    print("Resetting group: " + str(group))

    version = options.version

    extensions_to_bump = []
    extensions_to_lock = []
    for ext, ext_path in get_all_extensions(root):
        if matches(ext, exts["include"], exts.get("exclude", [])):
            extensions_to_bump.append(ext_path)
        if exts.get("use_inclexcl_lists_as_lock_list", False):
            if matches(ext, exts["include"], exts.get("exclude", [])):
                extensions_to_lock.append(ext_path)
        else:
            if ext in exts.get("lock_list", []):
                extensions_to_lock.append(ext_path)

    print("Resetting extensions: " + str(extensions_to_bump))
    for ext_path in extensions_to_bump:
        config_file_path = os.path.join(ext_path, "config/extension.toml")
        if os.path.exists(config_file_path):
            update_version_in_ext_config(config_file_path, version)

    print("Locking extension version in these: " + str(extensions_to_lock))
    for ext_path in extensions_to_lock:
        config_file_path = os.path.join(ext_path, "config/extension.toml")
        if os.path.exists(config_file_path): 
            lock_version_in_ext_config(config_file_path, version, exts.get("include", []), exts.get("exclude", []))

    return extensions_to_bump


def setup_repo_tool(parser: argparse.ArgumentParser, config: Dict) -> Callable:
    tool_config = config.get("repo_version_reset", {})
    publish_config = config.get("repo_publish_exts", {})
    enabled = tool_config.get("enabled", False)
    if not enabled:
        return None

    parser.description = "Tool to update version for omni physx extensions."
    parser.add_argument("-v", "--info", dest="log_info", required=False, action="store_true")
    parser.add_argument(
        "--version",
        dest="version",
        nargs="?",
        const="",
        required=True,
        help="Version to bump to.",
    )

    def run_repo_tool(options: Dict, config: Dict):
        repo_folders = config["repo"]["folders"]

        if options.log_info:
            logger.setLevel(logging.INFO)

        run(tool_config, options, repo_folders["root"], publish_config)

    return run_repo_tool
