#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Build a separate qualification image with refresh/update/uncached-publication hooks."""
import argparse
import ast
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    candidate, output = args.candidate.resolve(), args.out.resolve()
    if candidate == output or output.exists():
        parser.error("Use a new qualification directory, separate from the timed candidate")
    output.mkdir(parents=True)
    commands = json.loads((candidate / "commands.json").read_text())
    shutil.copy2(candidate / "bridge.cpp", output / "bridge.cpp")
    shutil.copytree(candidate / "schemas", output / "schemas")
    for archive in candidate.glob("*.a"):
        shutil.copy2(archive, output / archive.name)
    extra_exports = ["_ovp_test_refresh_output_cache", "_ovp_test_update_from_stage", "_ovp_test_publish_uncached"]
    source = Path(__file__).resolve().parent / "output-query-test-bindings.cpp"
    actual = []
    for entry in commands:
        if entry["name"] in ("shared-host-archive", "side-bootstrap-archive"):
            continue
        command = [value.replace(str(candidate), str(output)) for value in entry["argv"]]
        if entry["name"] == "physics-bindings":
            inputs = [i for i, value in enumerate(command) if value.endswith("/bindings.cpp")]
            if len(inputs) != 1:
                raise ValueError("Expected one canonical physics binding source")
            command[inputs[0]] = str(source)
        if entry["name"] in ("link-physics", "link"):
            indices = [i for i, value in enumerate(command) if value.startswith("-sEXPORTED_FUNCTIONS=")]
            if len(indices) != 1:
                raise ValueError("Expected one explicit export list per shared-module link")
            index = indices[0]
            exports = ast.literal_eval(command[index].split("=", 1)[1])
            command[index] = "-sEXPORTED_FUNCTIONS=" + json.dumps(sorted(set(exports + extra_exports)))
            if entry["name"] == "link":
                indices = [i for i, value in enumerate(command) if value.startswith("-sDEFAULT_LIBRARY_FUNCS_TO_INCLUDE=")]
                if len(indices) != 1:
                    raise ValueError("Expected the canonical side-module JavaScript forwarding list")
                index = indices[0]
                functions = ast.literal_eval(command[index].split("=", 1)[1])
                command[index] = "-sDEFAULT_LIBRARY_FUNCS_TO_INCLUDE=" + json.dumps(sorted(set(functions + [name[1:] for name in extra_exports])))
        actual.append({"name": entry["name"], "cwd": entry["cwd"], "argv": command})
        (output / "commands.json").write_text(json.dumps(actual, indent=2) + "\n")
        with (output / (entry["name"] + ".log")).open("w") as log:
            subprocess.run(command, cwd=entry["cwd"], stdout=log, stderr=subprocess.STDOUT,
                           env=dict(os.environ, EMCC_CORES="4", BINARYEN_CORES="4"), check=True)
    sha = lambda file: hashlib.sha256(file.read_bytes()).hexdigest()
    receipt = {"qualificationOnly": True, "candidate": str(candidate),
               "candidateCommandsSha256": sha(candidate / "commands.json"),
               "testSource": str(source), "testSourceSha256": sha(source),
               "artifactSha256": {name: sha(output / name) for name in ("ovstage.mjs", "ovstage.wasm", "ovphysx.wasm")}}
    (output / "qualification-receipt.json").write_text(json.dumps(receipt, indent=2) + "\n")
    print(output)


if __name__ == "__main__":
    main()
