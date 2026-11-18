#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Prepare copied Carbonite browser headers and test the real zero-worker dispatcher."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

HERE = Path(__file__).resolve().parent
OVPHYSX = HERE.parents[2]
HOST_SHA256 = "e2c8a674c4f9c89b15f57165663695b37bc87073f3858a09682f4f9e8468f40e"


def digest(data):
    return hashlib.sha256(data).hexdigest()


def _validate_output_isolation(carb_include, fabric_host, output):
    """Reject planned writes into dependency or recipe inputs, including output aliases.

    The output directory may be a common ancestor of the inputs. Only its copied
    include subtree and files emitted by prepare() must be separate. Existing
    links are resolved before any mkdir/copy/write; concurrent filesystem changes
    during preparation are outside this local build tool's contract.
    """
    inputs = [carb_include, fabric_host]
    input_files = set()
    pending = list(inputs)
    visited = set()
    while pending:
        path = pending.pop()
        resolved = path.resolve()
        if resolved in visited:
            continue
        visited.add(resolved)
        if path.is_dir():
            # copytree follows input directory links, so their targets are inputs too.
            if all(resolved != source and source not in resolved.parents for source in inputs):
                inputs.append(resolved)
            pending.extend(path.iterdir())
        else:
            info = path.stat()
            input_files.add((info.st_dev, info.st_ino))

    # Protect the recipe's actual inputs without reserving its whole directory.
    recipe_inputs = {(HERE / name).resolve() for name in
                     ("platform-patches.json", "DictionaryPayload.inl",
                      "SettingsArrays.inl", "TaskStorage.inl")}
    recipe_inputs.add(Path(__file__).resolve())
    for path in recipe_inputs:
        info = path.stat()
        input_files.add((info.st_dev, info.st_ino))

    def check_destination(path, tree=False, expect_file=False):
        resolved = path.resolve()
        if resolved in recipe_inputs:
            raise ValueError(f"Generated output overlaps a recipe input file: {path}")
        if any(resolved == source or source in resolved.parents or
               (tree and resolved in source.parents) for source in inputs):
            raise ValueError(f"Generated output overlaps a dependency input tree: {path}")
        # copy2 treats a directory as a container and appends the source basename.
        # Reject that alternate target before copytree can bypass the checked path.
        if expect_file and path.is_dir():
            raise ValueError(f"Generated file destination is a directory: {path}")
        if path.exists() and not path.is_dir():
            info = path.stat()
            if (info.st_dev, info.st_ino) in input_files:
                raise ValueError(f"Generated output aliases a source input file: {path}")

    def check_copied_tree(source, destination, ancestors=()):
        check_destination(destination, tree=True)
        resolved = source.resolve()
        if resolved in ancestors:
            raise ValueError(f"Dependency input contains a recursive directory link: {source}")
        for child in source.iterdir():
            target = destination / child.name
            if child.is_dir():
                check_copied_tree(child, target, (*ancestors, resolved))
            else:
                check_destination(target, expect_file=True)

    check_destination(output)
    check_copied_tree(carb_include, output / "carb" / "include")
    for name in ("host.cpp", "DictionaryPayload.inl", "SettingsArrays.inl",
                 "TaskStorage.inl", "overlay-receipt.json"):
        check_destination(output / name, expect_file=True)


def prepare(carb_include, fabric_host, output):
    """Return copied header/host paths; never modify either dependency source."""
    carb_include = Path(carb_include).resolve()
    fabric_host = Path(fabric_host).resolve()
    output = Path(output).resolve()
    include = output / "carb" / "include"
    _validate_output_isolation(carb_include, fabric_host, output)
    output.mkdir(parents=True, exist_ok=True)
    patches = json.loads((HERE / "platform-patches.json").read_text())
    for patch in patches:
        source = carb_include / patch["path"]
        if digest(source.read_bytes()) != patch["source_sha256"]:
            raise ValueError(f"Carbonite source changed: {source}")
    source = fabric_host / "host.cpp"
    host_bytes = source.read_bytes()
    if digest(host_bytes) != HOST_SHA256:
        raise ValueError(f"OVStage host source changed: {source}")
    shutil.copytree(carb_include, include, dirs_exist_ok=True)
    receipt = []
    for patch in patches:
        destination = include / patch["path"]
        text = destination.read_text()
        for edit in patch["edits"]:
            if text.count(edit["old"]) != 1:
                raise ValueError(f"Expected exactly one patch location: {destination}")
            text = text.replace(edit["old"], edit["new"])
        destination.write_text(text)
        receipt.append({**patch, "output_sha256": digest(text.encode())})
    text = host_bytes.decode()
    old = "        initializeUnsupportedTasking(result);"
    new = old + """
        result.getDesc = []() -> const carb::tasking::TaskingDesc& {
            static const carb::tasking::TaskingDesc descriptor{};
            return descriptor;
        };"""
    if text.count(old) != 1:
        raise ValueError("Expected exactly one host task table initializer")
    text = text.replace(old, new)
    text = text.replace('#include <shared_mutex>', '#include <shared_mutex>\n#include <stdexcept>')
    marker = 'carb::dictionary::IDictionary& dictionary()'
    if text.count(marker) != 1:
        raise ValueError("Expected dictionary table definition")
    text = text.replace(marker, '#include "DictionaryPayload.inl"\n' + marker)
    marker = '        initializeUnsupportedDictionary(result);'
    if text.count(marker) != 1:
        raise ValueError("Expected dictionary table initializer")
    begin = text.index('carb::dictionary::IDictionary& dictionary()')
    end = text.index('carb::tasking::ITasking& tasking()', begin)
    dictionary_table = text[begin:end]
    if dictionary_table.count('        return result;') != 1:
        raise ValueError("Expected dictionary table return")
    dictionary_table = dictionary_table.replace('        return result;',
        '        installPayloadDictionary(result);\n        return result;')
    text = text[:begin] + dictionary_table + text[end:]
    shutil.copyfile(HERE / "DictionaryPayload.inl", output / "DictionaryPayload.inl")
    receipt.append({"path": str(HERE / "DictionaryPayload.inl"),
                    "source_sha256": digest((HERE / "DictionaryPayload.inl").read_bytes()),
                    "output_sha256": digest((output / "DictionaryPayload.inl").read_bytes())})
    marker = 'carb::settings::ISettings& settings()'
    if text.count(marker) != 1:
        raise ValueError("Expected settings table definition")
    text = text.replace(marker, '#include "SettingsArrays.inl"\n' + marker)
    begin = text.index(marker)
    end = text.index('carb::dictionary::IDictionary& dictionary()', begin)
    settings_table = text[begin:end]
    if settings_table.count('        return result;') != 1:
        raise ValueError("Expected settings table return")
    settings_table = settings_table.replace('        return result;',
        '        installSettingsArrays(result);\n        return result;')
    text = text[:begin] + settings_table + text[end:]
    shutil.copyfile(HERE / "SettingsArrays.inl", output / "SettingsArrays.inl")
    receipt.append({"path": str(HERE / "SettingsArrays.inl"),
                    "source_sha256": digest((HERE / "SettingsArrays.inl").read_bytes()),
                    "output_sha256": digest((output / "SettingsArrays.inl").read_bytes())})
    marker = 'carb::tasking::ITasking& tasking()'
    if text.count(marker) != 1:
        raise ValueError("Expected tasking table definition")
    text = text.replace(marker, '#include "TaskStorage.inl"\n' + marker)
    text = text.replace('        initializeUnsupportedTasking(result);',
        '        initializeUnsupportedTasking(result);\n        installTaskStorage(result);')
    shutil.copyfile(HERE / "TaskStorage.inl", output / "TaskStorage.inl")
    receipt.append({"path": str(HERE / "TaskStorage.inl"),
                    "source_sha256": digest((HERE / "TaskStorage.inl").read_bytes()),
                    "output_sha256": digest((output / "TaskStorage.inl").read_bytes())})
    host_output = output / "host.cpp"
    host_output.write_text(text)
    receipt.append({"path": str(source), "source_sha256": HOST_SHA256,
                    "output_sha256": digest(text.encode()),
                    "change": "getDesc reports zero scheduler workers; all other host behavior is preserved"})
    (output / "overlay-receipt.json").write_text(json.dumps(receipt, indent=2) + "\n")
    return {"include": include, "host": host_output, "output": output}


def run_test(paths, fabric_host, compiler, node):
    output = paths["output"]
    command = [str(compiler), "-std=c++20", "-sMEMORY64=1", "-pthread", "-fexceptions", "-frtti", "-DNDEBUG",
               "-DCARB_STATIC_PLUGIN_BUILD=1", "-DCARB_DISABLE_DEFER_LOAD=1", "-DCARB_REQUIRE_LINKED=1",
               "-Dcarb_logging_ILogging=CARB_HEXVERSION(1,6)", "-Dcarb_filesystem_IFileSystem=CARB_HEXVERSION(2,2)",
               "-Dcarb_Framework=carb_Framework_latest", "-DPX_PHYSX_STATIC_LIB", "-Wno-character-conversion",
               "-I" + str(paths["include"]), "-I" + str(fabric_host),
               "-I" + str(OVPHYSX.parent / "physx" / "include"),
               "-I" + str(OVPHYSX / "ovruntime/source/omni.physx/plugins"),
               str(paths["host"]), str(HERE / "dispatcher-test.cpp"),
               "-sENVIRONMENT=node", "-sASSERTIONS=1", "-sINITIAL_MEMORY=33554432", "-sSTACK_SIZE=1048576",
               "-sPTHREAD_POOL_SIZE=0", "-o", str(output / "dispatcher-test.js")]
    (output / "dispatcher-command.json").write_text(json.dumps(command, indent=2) + "\n")
    build = subprocess.run(command, capture_output=True, text=True)
    (output / "dispatcher-build.log").write_text(build.stdout + build.stderr)
    if build.returncode:
        raise RuntimeError(f"Dispatcher compile failed; see {output / 'dispatcher-build.log'}")
    execution = [str(node), str(output / "dispatcher-test.js")]
    result = subprocess.run(execution, capture_output=True, text=True)
    (output / "dispatcher-test.log").write_text(result.stdout + result.stderr)
    (output / "test-receipt.json").write_text(json.dumps({"command": execution, "exit": result.returncode,
        "source": str(OVPHYSX / "ovruntime/source/omni.physx/plugins/PhysXCarbCPUDispatcher.h"),
        "compiled": True, "scope": "original dispatcher and static host prerequisite, not complete OvPhysX runtime"}, indent=2) + "\n")
    print(result.stdout, end="")
    if result.returncode:
        raise RuntimeError(f"Dispatcher test failed; see {output / 'dispatcher-test.log'}")
    event_command = [part.replace("dispatcher-test.cpp", "events-test.cpp")
                    .replace("dispatcher-test.js", "events-test.js")
                    .replace("-sPTHREAD_POOL_SIZE=0", "-sPTHREAD_POOL_SIZE=2") for part in command]
    event_command.insert(event_command.index(str(paths["host"])), str(HERE / "Events.cpp"))
    (output / "events-command.json").write_text(json.dumps(event_command, indent=2) + "\n")
    event_build = subprocess.run(event_command, capture_output=True, text=True)
    (output / "events-build.log").write_text(event_build.stdout + event_build.stderr)
    if event_build.returncode:
        raise RuntimeError(f"Event provider compile failed; see {output / 'events-build.log'}")
    event_execution = [str(node), str(output / "events-test.js")]
    event_result = subprocess.run(event_execution, capture_output=True, text=True, timeout=30)
    (output / "events-test.log").write_text(event_result.stdout + event_result.stderr)
    (output / "events-test-receipt.json").write_text(json.dumps({"command": event_execution,
        "exit": event_result.returncode, "scope": "real event host contracts, not a full OvPhysX runtime test"}, indent=2) + "\n")
    print(event_result.stdout, end="")
    if event_result.returncode:
        raise RuntimeError(f"Event provider test failed; see {output / 'events-test.log'}")

    log_command = [part.replace("events-test.cpp", "log-test.cpp")
                   .replace("events-test.js", "log-test.js") for part in event_command
                   if part not in (str(paths["host"]), str(HERE / "Events.cpp"))]
    rendering = fabric_host.parents[4]
    log_command[1:1] = ["-I" + str(rendering / "include"), str(HERE / "Log.cpp"),
                       str(HERE / "StaticServices.cpp")]
    (output / "log-command.json").write_text(json.dumps(log_command, indent=2) + "\n")
    log_build = subprocess.run(log_command, capture_output=True, text=True)
    (output / "log-build.log").write_text(log_build.stdout + log_build.stderr)
    if log_build.returncode:
        raise RuntimeError(f"Log provider compile failed; see {output / 'log-build.log'}")
    log_execution = [str(node), str(output / "log-test.js")]
    log_result = subprocess.run(log_execution, capture_output=True, text=True, timeout=30)
    (output / "log-run.log").write_text(log_result.stdout + log_result.stderr)
    (output / "log-test-receipt.json").write_text(json.dumps({"command": log_execution,
        "exit": log_result.returncode, "scope": "real synchronous ILog contracts"}, indent=2) + "\n")
    print(log_result.stdout, end="")
    if log_result.returncode:
        raise RuntimeError(f"Log provider test failed; see {output / 'log-run.log'}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--carb-include", type=Path, required=True)
    parser.add_argument("--fabric-host", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--test", action="store_true")
    parser.add_argument("--emxx", type=Path)
    parser.add_argument("--node", type=Path)
    args = parser.parse_args()
    if args.test and (args.emxx is None or args.node is None):
        parser.error("--test requires --emxx and --node (Node 23 or newer for memory64)")
    paths = prepare(args.carb_include, args.fabric_host, args.output)
    print(paths["include"])
    if args.test:
        run_test(paths, args.fabric_host.resolve(), args.emxx.resolve(), args.node.resolve())


if __name__ == "__main__":
    main()
