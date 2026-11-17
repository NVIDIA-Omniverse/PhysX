# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Set hidden visibility on an explicit set of defined WASM functions only.

This operates on the standard linking-v2 symbol metadata. Code, data, names,
binding strength, undefined symbols, and ar member offsets are unchanged.
Unknown object formats fail closed. Input files are never modified.
"""
from __future__ import annotations

import ast
import hashlib
import json
import os
import subprocess
import sys
from collections.abc import Callable, Sequence
from pathlib import Path


class Reader:
    def __init__(self, data: bytearray, start: int, end: int):
        self.data, self.pos, self.end = data, start, end

    def byte(self) -> int:
        if self.pos >= self.end:
            raise ValueError("Truncated WASM metadata")
        result = self.data[self.pos]
        self.pos += 1
        return result

    def uint(self) -> int:
        result = 0
        for shift in range(0, 70, 7):
            value = self.byte()
            result |= (value & 0x7f) << shift
            if not value & 0x80:
                return result
        raise ValueError("Invalid WASM unsigned LEB")

    def string(self) -> str:
        size = self.uint()
        end = self.pos + size
        if end > self.end:
            raise ValueError("Truncated WASM string")
        result = bytes(self.data[self.pos:end]).decode("utf-8")
        self.pos = end
        return result


def hide_object(data: bytearray, start: int, end: int, names: set[str], label: str) -> list[dict]:
    if bytes(data[start:start + 8]) != b"\0asm\x01\0\0\0":
        raise ValueError(f"Expected a WASM object: {label}")
    reader = Reader(data, start + 8, end)
    changes = []
    linking_sections = 0
    while reader.pos < end:
        section = reader.byte()
        size = reader.uint()
        section_end = reader.pos + size
        if section_end > end:
            raise ValueError(f"Section exceeds object: {label}")
        if section == 0:
            body = Reader(data, reader.pos, section_end)
            if body.string() == "linking":
                linking_sections += 1
                if body.uint() != 2:
                    raise ValueError("Only WASM linking-v2 is supported")
                while body.pos < body.end:
                    subsection = body.byte()
                    sub_size = body.uint()
                    sub_end = body.pos + sub_size
                    if sub_end > body.end:
                        raise ValueError("Linking subsection exceeds section")
                    if subsection == 8:
                        symbols = Reader(data, body.pos, sub_end)
                        for _ in range(symbols.uint()):
                            kind = symbols.byte()
                            flags_offset = symbols.pos
                            flags = symbols.uint()
                            name = None
                            if kind in (0, 2, 4, 5):
                                symbols.uint()  # Function/global/tag/table index.
                                if not flags & 0x10 or flags & 0x40:
                                    name = symbols.string()
                            elif kind == 1:
                                name = symbols.string()
                                if not flags & 0x10:
                                    symbols.uint()  # Segment.
                                    symbols.uint()  # Offset.
                                    symbols.uint()  # Size.
                            elif kind == 3:
                                symbols.uint()  # Section index.
                            else:
                                raise ValueError(f"Unknown symbol kind {kind}")
                            if name in names and not flags & 0x10:
                                if kind != 0:
                                    raise ValueError(f"Requested symbol is not a function: {name}")
                                if flags & 0x20:
                                    raise ValueError(f"Explicitly exported symbol cannot be hidden: {name}")
                                if not flags & 0x4:
                                    before = data[flags_offset]
                                    data[flags_offset] |= 0x4
                                    changes.append({"member": label, "symbol": name,
                                                    "offset": flags_offset,
                                                    "before": before, "after": data[flags_offset]})
                        if symbols.pos != symbols.end:
                            raise ValueError("Unexpected trailing symbol-table bytes")
                    body.pos = sub_end
        reader.pos = section_end
    if linking_sections != 1:
        raise ValueError(f"Expected exactly one linking section: {label}")
    return changes


def transform_relocatable(original: bytes, names: set[str], protected: set[str]) -> tuple[bytes, list[dict]]:
    if names & protected:
        raise ValueError("Cannot hide protected shared/public symbols")
    data = bytearray(original)
    if data.startswith(b"\0asm"):
        changes = hide_object(data, 0, len(data), names, "object")
    elif data.startswith(b"!<arch>\n"):
        changes = []
        position = 8
        while position < len(data):
            if position + 60 > len(data) or data[position + 58:position + 60] != b"`\n":
                raise ValueError("Malformed ar member header")
            header = bytes(data[position:position + 60])
            name = header[:16].decode("ascii").strip()
            encoded_size = header[48:58].strip()
            if not encoded_size or not encoded_size.isdigit():
                raise ValueError("Invalid ar member size")
            size = int(encoded_size)
            begin, end = position + 60, position + 60 + size
            if end > len(data):
                raise ValueError("Truncated ar member")
            if name not in ("/", "//", "/SYM64/"):
                if name.startswith("#1/"):
                    if not name[3:].isdigit() or int(name[3:]) > size:
                        raise ValueError("Invalid BSD ar member name length")
                    begin += int(name[3:])
                changes.extend(hide_object(data, begin, end, names, name))
            position = end + (size & 1)
        if position != len(data):
            raise ValueError("Invalid ar padding")
    else:
        raise ValueError("Only ordinary ar archives and WASM objects are supported")
    expected = {entry["offset"] for entry in changes}
    actual = {index for index, (old, new) in enumerate(zip(original, data)) if old != new}
    if len(original) != len(data) or actual != expected:
        raise AssertionError("Only recorded visibility flag bytes may change")
    return bytes(data), changes



# Use the engine's binary metadata API, without constructing a native instance.
_INSPECT_MODULES = r'''
const fs = require('node:fs');
const files = process.argv.slice(1);
const result = files.map(file => {
  const module = new WebAssembly.Module(fs.readFileSync(file));
  return {imports: WebAssembly.Module.imports(module), exports: WebAssembly.Module.exports(module)};
});
process.stdout.write(JSON.stringify(result));
'''


def _digest(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _identity(path: Path) -> dict:
    data = path.read_bytes()
    return {"path": str(path), "bytes": len(data), "sha256": _digest(data)}


def _write_json(path: Path, value: object) -> None:
    with path.open("x") as stream:
        json.dump(value, stream, indent=2)
        stream.write("\n")


def resolve_emscripten_node(compiler: Path, *, env: dict[str, str] | None = None) -> list[str]:
    """Use Emscripten's configured engine, never an implicit system Node fallback."""
    config = Path(compiler).resolve().parent / "em-config.py"
    if not config.is_file():
        raise ValueError(f"Missing Emscripten configuration query: {config}")
    result = subprocess.run([sys.executable, str(config), "NODE_JS"], env=env,
                            capture_output=True, text=True, check=False)
    if result.returncode:
        raise ValueError("Cannot read Emscripten NODE_JS: " + result.stderr.strip())
    text = result.stdout.strip()
    try:
        value = ast.literal_eval(text)
    except (ValueError, SyntaxError):
        value = text if text.startswith("/") and "\n" not in text else None
    command = [value] if isinstance(value, str) else list(value) if isinstance(value, (list, tuple)) else []
    if not command or any(not isinstance(item, str) or not item for item in command):
        raise ValueError("Emscripten NODE_JS must specify a nonempty executable/argument list")
    executable = Path(command[0])
    if not executable.is_absolute() or not executable.is_file() or not os.access(executable, os.X_OK):
        raise ValueError("Emscripten NODE_JS must name an explicit existing executable; no PATH fallback")
    command[0] = str(executable.resolve())
    # Shared memory64 and table64 declarations are compiled, never instantiated.
    probe = ("new WebAssembly.Module(new Uint8Array([0,97,115,109,1,0,0,0,"
             "4,4,1,112,4,0,5,4,1,7,0,1]));console.log('ovstage-memory64-probe-ok')")
    result = subprocess.run([*command, "-e", probe], env=env, capture_output=True, text=True, check=False)
    if result.returncode or result.stdout.strip() != "ovstage-memory64-probe-ok":
        raise ValueError("Configured Emscripten Node cannot compile shared memory64/table64: " + result.stderr.strip())
    return command


def _inspect_modules(node: Sequence[str], files: Sequence[Path]) -> list[dict]:
    if not node:
        raise ValueError("A configured WASM64-capable Node command is required")
    command = [*map(str, node), "-e", _INSPECT_MODULES, *map(str, files)]
    result = subprocess.run(command, capture_output=True, text=True, check=False)
    if result.returncode:
        raise RuntimeError("Cannot inspect compiled WASM with the configured Node: " + result.stderr.strip())
    modules = json.loads(result.stdout)
    if not isinstance(modules, list) or len(modules) != len(files):
        raise ValueError("Unexpected compiled-module metadata result")
    for module in modules:
        if not isinstance(module, dict) or set(module) != {"imports", "exports"}:
            raise ValueError("Invalid compiled-module metadata")
        for key in ("imports", "exports"):
            if not isinstance(module[key], list) or any(
                not isinstance(item, dict) or not isinstance(item.get("name"), str)
                or item.get("kind") not in {"function", "global", "memory", "table", "tag"}
                or (key == "imports" and not isinstance(item.get("module"), str))
                for item in module[key]
            ):
                raise ValueError("Invalid compiled-module symbol record")
    return modules


def _link_inputs(command: Sequence[str], cwd: Path) -> list[Path]:
    inputs = []
    for argument in command:
        if argument.startswith("@"):
            raise ValueError("Response-file inputs are unsupported by the visibility audit")
        if argument.startswith("-") or not argument.endswith((".a", ".o")):
            continue
        path = Path(argument)
        path = (path if path.is_absolute() else cwd / path).resolve()
        if not path.is_file():
            raise ValueError(f"Missing link input: {path}")
        if path not in inputs:
            inputs.append(path)
    if not inputs:
        raise ValueError("No explicit object/archive inputs to audit")
    return inputs


def _output_path(command: Sequence[str], cwd: Path) -> Path:
    if command.count("-o") != 1 or command.index("-o") + 1 == len(command):
        raise ValueError("Expected exactly one explicit link output")
    path = Path(command[command.index("-o") + 1])
    return (path if path.is_absolute() else cwd / path).resolve()


def _public_functions(command: Sequence[str]) -> set[str]:
    flags = [value for value in command if value.startswith("-sEXPORTED_FUNCTIONS=")]
    if len(flags) != 1:
        raise ValueError("Expected one explicit JSON side-module export list")
    names = json.loads(flags[0].split("=", 1)[1])
    if not isinstance(names, list) or not names or any(not isinstance(name, str) or not name.startswith("_") for name in names):
        raise ValueError("Invalid side-module JavaScript export list")
    return {name[1:] for name in names}


def _derive_policy(main: dict, side: dict, public: set[str], definitions: set[str]) -> dict:
    protected = {row["name"] for row in main["exports"]} | public | definitions
    protected.update(row["name"] for row in side["exports"] if row["kind"] != "function")
    exported = {row["name"] for row in side["exports"] if row["kind"] == "function"}
    imported = {row["name"] for row in side["imports"] if row["kind"] == "function"}
    if not public <= exported:
        raise ValueError("The linked side module is missing declared public functions")
    return {"functions": sorted((imported & exported) - protected), "protected": sorted(protected),
            "publicFunctions": sorted(public)}


def _records(rows: list[dict]) -> set[str]:
    encoded = [json.dumps(row, sort_keys=True, separators=(",", ":")) for row in rows]
    if len(encoded) != len(set(encoded)):
        raise ValueError("Duplicate module import/export descriptors are unsupported")
    return set(encoded)


def audit_modules(before: dict, after: dict, policy: dict) -> dict:
    """Reject any change beyond the selected private function bindings."""
    selected, protected = set(policy["functions"]), set(policy["protected"])
    public = set(policy["publicFunctions"])
    for name, predicate in [
        ("public functions", lambda row: row["name"] in public or row["name"].startswith("ovp_wasm_")),
        ("non-function exports", lambda row: row["kind"] != "function"),
        ("shared exports", lambda row: row["name"] in protected),
    ]:
        if _records([row for row in before["exports"] if predicate(row)]) != _records(
            [row for row in after["exports"] if predicate(row)]
        ):
            raise ValueError(f"Visibility relink changed {name}")
    old_imports, new_imports = _records(before["imports"]), _records(after["imports"])
    if new_imports - old_imports:
        raise ValueError("Visibility relink introduced imports")
    removed = [row for row in before["imports"] if json.dumps(row, sort_keys=True, separators=(",", ":")) not in new_imports]
    for row in removed:
        if row["name"] not in selected or row["name"] in protected or not (
            row["kind"] == "function" or row["kind"] == "global" and row["module"] == "GOT.func"
        ):
            raise ValueError(f"Visibility relink removed a protected/non-function import: {row['name']}")
    if any(row["kind"] == "function" and row["name"] in selected for row in after["imports"]):
        raise ValueError("Selected functions still require external imports")
    old_count = sum(row["kind"] == "function" for row in before["imports"])
    new_count = sum(row["kind"] == "function" for row in after["imports"])
    if new_count >= old_count:
        raise ValueError("Visibility relink did not reduce function imports")
    return {"functionImportsBefore": old_count, "functionImportsAfter": new_count,
            "removedImports": removed, "publicExportsUnchanged": True,
            "nonFunctionExportsUnchanged": True, "sharedExportsUnchanged": True,
            "protectedImportsUnchanged": True}


def optimize_side_functions(
    *, main_wasm: Path, main_loader: Path, side_wasm: Path,
    main_link: Sequence[str], side_link: Sequence[str], cwd: Path, work_dir: Path,
    node: Sequence[str], llvm_nm: Path, run: Callable[[list[str], str], None],
) -> dict:
    """Relink private functions for one main + optional physics side graph.

    Call only after the normal side and main links and the runtime object/archive
    ABI guards. The callback runs/logs the derived command with the same cwd as
    the normal links. The original side is replaced only after all audits pass.
    No source archive, object, main module, memory, table, or native owner is edited.
    Future additional side modules require a newly qualified binding policy.
    """
    cwd = Path(cwd).resolve()
    main_wasm, main_loader, side_wasm = (Path(path).resolve() for path in (main_wasm, main_loader, side_wasm))
    main_link, side_link = list(map(str, main_link)), list(map(str, side_link))
    if "-sMAIN_MODULE=2" not in main_link or "-sAUTOLOAD_DYLIBS=0" not in main_link or "-sSIDE_MODULE=2" not in side_link:
        raise ValueError("Visibility optimization requires the qualified optional MAIN_MODULE=2/SIDE_MODULE=2 graph")
    if _output_path(main_link, cwd) != main_loader or _output_path(side_link, cwd) != side_wasm:
        raise ValueError("Link commands do not describe the supplied generated artifacts")
    if len({main_wasm, main_loader, side_wasm}) != 3:
        raise ValueError("Main and side artifacts must be distinct")
    side_inputs_in_main = [Path(value) for value in main_link if not value.startswith("-") and value.endswith(".wasm")]
    side_inputs_in_main = [(path if path.is_absolute() else cwd / path).resolve() for path in side_inputs_in_main]
    if side_inputs_in_main != [side_wasm]:
        raise ValueError("Only one optional physics side module is supported")
    for path in (main_wasm, main_loader, side_wasm):
        if not path.is_file():
            raise ValueError(f"Missing generated artifact: {path}")
    if any(os.path.samefile(a, b) for a, b in [(main_wasm, main_loader), (main_wasm, side_wasm), (main_loader, side_wasm)]):
        raise ValueError("Generated artifacts alias each other")
    main_inputs, side_inputs = _link_inputs(main_link, cwd), _link_inputs(side_link, cwd)
    identities = {path: _identity(path) for path in dict.fromkeys([*main_inputs, *side_inputs, main_wasm, main_loader, side_wasm])}
    main, side = _inspect_modules(node, [main_wasm, side_wasm])
    definitions, main_inventory = set(), []
    for path in main_inputs:
        result = subprocess.run([str(llvm_nm), "--defined-only", "--just-symbol-name", str(path)],
                                capture_output=True, text=True, check=True)
        names = set(result.stdout.splitlines())
        definitions.update(names)
        main_inventory.append({**identities[path], "definedSymbols": len(names)})
    policy = _derive_policy(main, side, _public_functions(side_link), definitions)
    work_dir = Path(work_dir).resolve()
    if work_dir.exists():
        raise ValueError("Visibility work directory must be fresh")
    work_dir.mkdir(parents=True)
    _write_json(work_dir / "policy.json", policy)
    receipt = {"protocol": 1, "scope": "one main and one optional physics side; not an arbitrary dlopen ABI",
               "applied": False, "mainArtifacts": [identities[main_loader], identities[main_wasm]],
               "originalSide": identities[side_wasm], "mainInputDefinitions": main_inventory,
               "policy": _identity(work_dir / "policy.json"), "helper": _identity(Path(__file__).resolve()),
               "selectedFunctions": len(policy["functions"]), "inspectionNode": list(map(str, node))}
    if not policy["functions"]:
        for path, expected in identities.items():
            if _identity(path) != expected:
                raise ValueError(f"Original link input or artifact changed during visibility inspection: {path}")
        receipt["originalInputsUnchanged"] = True
        receipt["mainArtifactsUnchanged"] = True
        receipt["reason"] = "No eligible side-private function imports"
        _write_json(work_dir / "receipt.json", receipt)
        return receipt
    (work_dir / "inputs").mkdir()
    (work_dir / "candidate").mkdir()
    (work_dir / "control").mkdir()
    with (work_dir / "control" / side_wasm.name).open("xb") as stream:
        stream.write(side_wasm.read_bytes())
    replacements, derived, changed = {}, [], set()
    for index, path in enumerate(side_inputs):
        before = path.read_bytes()
        if _digest(before) != identities[path]["sha256"]:
            raise ValueError(f"Input changed during visibility preparation: {path}")
        after, changes = transform_relocatable(before, set(policy["functions"]), set(policy["protected"]))
        destination = work_dir / "inputs" / f"{index:03d}-{path.name}"
        with destination.open("xb") as stream:
            stream.write(after)
        replacements[path] = destination
        changed.update(change["symbol"] for change in changes)
        derived.append({"original": identities[path], "derived": _identity(destination), "changes": changes})
    if changed != set(policy["functions"]):
        raise ValueError("Not every selected function was found in defined side input symbols")
    command = []
    for argument in side_link:
        path = Path(argument)
        resolved = (path if path.is_absolute() else cwd / path).resolve() if not argument.startswith("-") else None
        command.append(str(replacements[resolved]) if resolved in replacements else argument)
    candidate = work_dir / "candidate" / side_wasm.name
    command[command.index("-o") + 1] = str(candidate)
    receipt.update({"derivedInputs": derived, "command": command, "cwd": str(cwd),
                    "modifiedDefinitions": sum(len(row["changes"]) for row in derived)})
    _write_json(work_dir / "prelink.json", receipt)
    run(command, "link-physics-private-functions")
    after = _inspect_modules(node, [candidate])[0]
    receipt["audit"] = audit_modules(side, after, policy)
    for path, expected in identities.items():
        if _identity(path) != expected:
            raise ValueError(f"Original link input or artifact changed during visibility relink: {path}")
    for row in derived:
        if _identity(Path(row["derived"]["path"])) != row["derived"]:
            raise ValueError("Derived input changed after its audited metadata transformation")
    receipt["candidate"] = _identity(candidate)
    receipt["applied"] = True
    receipt["originalInputsUnchanged"] = True
    receipt["mainArtifactsUnchanged"] = True
    _write_json(work_dir / "audit.json", receipt)
    os.replace(candidate, side_wasm)
    receipt["installedSide"] = _identity(side_wasm)
    _write_json(work_dir / "receipt.json", receipt)
    return receipt
