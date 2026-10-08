# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""get_benchmark_summary - regenerate the ovphysx C++ benchmark catalogue (BENCHMARK_SUMMARY.md).

Sibling tool to ovruntime's ``tools/repoman/get_benchmark_summary.py``, adapted for ovphysx's
cmake-script build (no repo_man here). For ``tests/benchmarks/benchmarks/`` this tool:
  1. Extracts every registered benchmark row name from the .cpp sources: plain
     ``Register<Class[, true]> var("Group.name");`` calls, the fixed-suffix expansions produced by
     the file-local ``DEFINE_LAB_ANYMAL`` and ``DEFINE_TENSOR_IO`` macros, and every
     ``DEFINE_OUTPUT_READ_VARIANT``-based wrapper macro in ``OutputRead.cpp``. The wrapper op token
     (e.g. ``queryread_rb``, ``tensorbinding_arti_link``) is read from each wrapper's own
     ``#define`` body rather than hardcoded here, so a new wrapper (or a renamed op token on an
     existing one) is picked up automatically instead of silently going undocumented or being
     reported under a stale name.
  2. Looks up a per-file analysis (category, tags, description, per-row notes) from the side-car
     database ``scripts/benchmark_summary_analysis.json`` (keyed by ovphysx-relative path).
  3. Writes ``BENCHMARK_SUMMARY.md``, grouped by subsystem category, with a leading "Heavy / do not
     loop" section listing every row flagged hidden.

The generated catalogue is NOT checked in. It is a build artifact, regenerated into
``_build/generated/benchmark-summaries/ovphysx/BENCHMARK_SUMMARY.md`` whenever the opt-in benchmark
suite is built (CMake target ``generate_benchmark_summary``, gated the same as
``OVPHYSX_BUILD_BENCHMARKS``). Only the analysis side-car is source-controlled.

Unlike ovruntime, "heavy" here IS read straight from the source: every existing hidden row already
uses the ``Register<T, true>`` template flag and the binary's ``--hidden`` / ``BENCHMARK_HIDDEN=1``
CLI already gates them at runtime (see ``tests/benchmarks/README.md``). The side-car's optional
``"heavy"`` per-row override exists only for the rare case a non-hidden row still deserves the
warning (e.g. a large but not-yet-hidden fixture).

Rows also carry a device requirement (some do not self-document it via a `_gpu`/`_cpu` name
suffix, e.g. `Clone.envs_64`, `Lab.anymal_*`). ``resolve_device()`` reads the `_gpu`/`_cpu` suffix
when present. Otherwise it falls back to scanning the registered class's (or its macro family's
shared base class's) `isValid()` body for `BmGlobals::forceGpu()`/`directGpu()` checks, see
`find_class_bodies()` / `resolve_macro_family_device()`. The catalogue annotates rows this fallback
resolves as GPU/DirectGPU-only so a caller (or `measure-benchmark-change`) has something to pin the
device pass to even without a name suffix.

Usage:
    python scripts/get_benchmark_summary.py            # regenerate into _build/generated
    python scripts/get_benchmark_summary.py --check    # exit non-zero if any row lacks an analysis note
"""
import argparse
import json
import logging
import os
import re

logger = logging.getLogger(os.path.basename(__file__))

# Paths are relative to the ovphysx root (the directory holding this repo's top-level CMakeLists.txt).
ANALYSIS_REL = os.path.join("scripts", "benchmark_summary_analysis.json")

# Generated catalogue lands here (relative to the ovphysx root). This is under _build/, which is
# gitignored. The file is a build artifact, regenerated on demand, never committed.
GENERATED_REL = os.path.join("_build", "generated", "benchmark-summaries")

# (benchmark tree relative to ovphysx root, page title, intro paragraph)
TREES = [
    (
        os.path.join("tests", "benchmarks", "benchmarks"),
        "ovphysx -- C++ benchmark catalogue",
        "Catalogue of every registered row in the ovphysx C++ benchmark suite (`ovphysx_benchmarks`), "
        "grouped by the subsystem each file predominantly exercises and cross-tagged for "
        "discoverability. Rows flagged heavy are hidden by default (`Register<T, true>` / "
        "`--hidden`) -- large-scale, diagnostic, or DirectGPU-only scenarios that should be run "
        "once, deliberately, never in a repeated local iteration loop.",
    ),
]

# Plain `Register<Class[, true]> var("Literal.Name");`. Captures the leading class identifier
# (used by `resolve_device()` for rows whose name carries no _gpu/_cpu suffix), the optional
# hidden flag, and the literal row name. Matches across newlines because some registrations
# wrap the name onto its own line.
REGISTER_RE = re.compile(
    r"Register\s*<\s*(?P<cls>[\w:]+)(?:\s*<[^<>]*>)?\s*(?P<hidden>,\s*true\s*)?>"
    r"\s*\w+\s*\(\s*(?P<name>\"(?:[^\"\\]|\\.)*\")\s*\)",
    re.DOTALL,
)
STRING_RE = re.compile(r'"((?:[^"\\]|\\.)*)"')

# Balanced-brace class-body extraction, used only to resolve device requirements for rows whose
# registered name has no _gpu/_cpu suffix (e.g. Clone.envs_64, Lab.anymal_1024_step): their
# isValid() gates on BmGlobals::forceGpu()/directGpu() instead of the caller picking the pass by
# name. `CLASS_DEF_RE` finds "class Name [: public Base] {", `_class_body_from` walks the source to
# the matching close brace.
CLASS_DEF_RE = re.compile(r"\bclass\s+(\w+)\s*(?::\s*public\s+(\w+))?[^{;]*\{")
MACRO_BASE_RE = re.compile(r"#define\s+{macro}\b.*?:\s*public\s+(\w+)", re.DOTALL)


def _class_body_from(text, brace_pos):
    depth = 0
    i = brace_pos
    while i < len(text):
        if text[i] == "{":
            depth += 1
        elif text[i] == "}":
            depth -= 1
            if depth == 0:
                return text[brace_pos:i + 1]
        i += 1
    return text[brace_pos:]


def find_class_bodies(text):
    """Return {class_name: (base_class_or_None, body_text)} for every `class Name [: public
    Base] { ... }` in ``text``, one level of brace nesting resolved (nested classes/lambdas inside
    are included in the parent's body text, which is acceptable because device-requirement
    detection only greps for forceGpu()/directGpu() textually)."""
    # setdefault, not overwrite: a class name can appear twice under an #if/#else (e.g.
    # LabCartpole.cpp's CUDAToolkit-gated implementation and its stub). The first occurrence
    # is the real implementation whose isValid() gates on forceGpu()/directGpu(), and the
    # stub's trivial body must not overwrite it.
    bodies = {}
    for m in CLASS_DEF_RE.finditer(text):
        name, base = m.group(1), m.group(2)
        body = _class_body_from(text, m.end() - 1)
        bodies.setdefault(name, (base, body))
    return bodies


def _class_requires(name, bodies, depth=0):
    if depth > 3 or name not in bodies:
        return False, False
    base, body = bodies[name]
    requires_gpu = "forceGpu()" in body
    requires_direct_gpu = "directGpu()" in body
    if not requires_gpu and not requires_direct_gpu and base:
        b_gpu, b_direct = _class_requires(base, bodies, depth + 1)
        requires_gpu, requires_direct_gpu = requires_gpu or b_gpu, requires_direct_gpu or b_direct
    return requires_gpu, requires_direct_gpu


def resolve_device(name, cls, bodies, text):
    """Return (requires_gpu, requires_direct_gpu) for a registered row. Rows whose name ends in
    `_gpu`/`_cpu` self-document their device pass. That suffix is authoritative and is not
    cross-checked against source. Rows with no such suffix fall back to scanning the registered
    class's own body (and, if it declares no isValid() of its own, one level of its base class)
    for BmGlobals::forceGpu()/directGpu() checks. ``cls`` may be None (macro-expanded row names
    with no literal `class Foo` in source, e.g. Lab.anymal_*)."""
    if name.endswith("_gpu"):
        return True, False
    if name.endswith("_cpu"):
        return False, False
    if cls is None:
        return False, False
    return _class_requires(cls, bodies, depth=0)


def resolve_macro_family_device(text, macro_name, bodies):
    """Device requirement shared by every row a file-local token-pasting macro (e.g.
    DEFINE_LAB_ANYMAL) expands to, resolved from the macro's *own* `#define` body. It always
    inherits from one shared base class, so that base's isValid() is read once rather than per-row."""
    m = re.search(MACRO_BASE_RE.pattern.format(macro=re.escape(macro_name)), text, re.DOTALL)
    if not m:
        return False, False
    return _class_requires(m.group(1), bodies, depth=0)

# File-local token-pasting macros whose Register<> call is not literally regex-matchable (the row
# name and class name are built with ## token pasting). Each is special-cased the same way
# ovruntime's REGISTER_USD_SCENARIO is: regex the macro *invocation*, not its *definition*, and
# rebuild the row name from the same template the macro body uses.
LAB_ANYMAL_RE = re.compile(r"DEFINE_LAB_ANYMAL\s*\(\s*(\w+)\s*,\s*[\w:]+\s*,\s*(\d+)\s*\)")
TENSOR_IO_RE = re.compile(r"DEFINE_TENSOR_IO\s*\(\s*(\w+)\s*,\s*[\w:]+\s*,\s*(\d+)\s*\)")

# OutputRead.cpp defines thin wrapper macros, one per (base class, row family) pair, that all
# bottom out in DEFINE_OUTPUT_READ_VARIANT(base, cls, op, N, device, gpu, hide). A hardcoded
# wrapper-name -> op-token mapping would go stale whenever a wrapper is added or renamed, so every
# "#define DEFINE_<X>(N, device, gpu, hide) ..." whose body calls DEFINE_OUTPUT_READ_VARIANT is
# found and the op token (3rd argument) is read from that call.
OUTPUT_READ_WRAPPER_DEF_RE = re.compile(
    r"#define\s+(DEFINE_\w+)\s*\(\s*N\s*,\s*device\s*,\s*gpu\s*,\s*hide\s*\)"
    r"(?P<body>(?:[^\n]*\\\n)*[^\n]*)",
)
OUTPUT_READ_VARIANT_CALL_RE = re.compile(
    r"DEFINE_OUTPUT_READ_VARIANT\(\s*[\w:]+\s*,\s*[\w:]+\s*,\s*([\w:]+)\s*,\s*N\s*,\s*device\s*,\s*gpu\s*,\s*hide\s*\)"
)


def find_output_read_wrappers(text):
    """Return {macro_name: op_token} for every DEFINE_OUTPUT_READ_VARIANT-based wrapper macro
    defined in ``text``, discovered from the #define bodies rather than a hardcoded list."""
    wrappers = {}
    for m in OUTPUT_READ_WRAPPER_DEF_RE.finditer(text):
        # Strip line-continuation backslashes so the call regex's \s* can span the (former) line
        # break without needing to special-case the backslash character itself.
        body = re.sub(r"\\\s*\n", "\n", m.group("body"))
        call = OUTPUT_READ_VARIANT_CALL_RE.search(body)
        if call:
            wrappers[m.group(1)] = call.group(1)
    return wrappers

# Fallback filename keyword -> category used only when a file is absent from the analysis.
FALLBACK_RULES = [
    ("Authoring", "Authoring"), ("WriteScaling", "Authoring"), ("InstancingAttach", "Loading & Attach"),
    ("UsdLoad", "Loading & Attach"), ("Clone", "Cloning"), ("Step", "Simulation"),
    ("OutputRead", "Output Read"), ("TensorBindings", "Tensor I/O"), ("LabCartpole", "Lab Probes"),
    ("LabAnymal", "Lab Probes"), ("LowLoad", "Low Load"), ("Smoke", "Smoke"),
]


def _is_commented(text, start):
    line_start = text.rfind("\n", 0, start) + 1
    prefix = text[line_start:start]
    return "//" in prefix or "#define" in prefix


def extract_rows(text):
    """Return list of (name, hidden, requires_gpu, requires_direct_gpu) for each registered
    benchmark row in a .cpp file. ``requires_gpu``/``requires_direct_gpu`` come from the row's own
    `_gpu`/`_cpu` name suffix when present. For rows with no such suffix they are resolved from the
    registered class's (or its macro family's shared base class's) isValid() body, see
    `resolve_device()` / `resolve_macro_family_device()`. Always False/False for rows that are
    provably neither (e.g. a plain CPU-only row that never calls forceGpu()/directGpu())."""
    out = []
    bodies = find_class_bodies(text)

    for m in REGISTER_RE.finditer(text):
        if _is_commented(text, m.start()):
            continue
        name = STRING_RE.match(m.group("name")).group(1)
        gpu, direct_gpu = resolve_device(name, m.group("cls"), bodies, text)
        out.append((name, bool(m.group("hidden")), gpu, direct_gpu))

    if LAB_ANYMAL_RE.search(text):
        lab_gpu, lab_direct_gpu = resolve_macro_family_device(text, "DEFINE_LAB_ANYMAL", bodies)
        for m in LAB_ANYMAL_RE.finditer(text):
            if _is_commented(text, m.start()):
                continue
            op_name, n = m.group(1), m.group(2)
            out.append((f"Lab.anymal_{n}_{op_name}", False, lab_gpu, lab_direct_gpu))

    for m in TENSOR_IO_RE.finditer(text):
        if _is_commented(text, m.start()):
            continue
        op_name, n = m.group(1), m.group(2)
        out.append((f"TensorIo.pose_{op_name}_{n}_cpu", False, False, False))

    wrappers = find_output_read_wrappers(text)
    if wrappers:
        invocation_re = re.compile(
            r"(" + "|".join(re.escape(name) for name in wrappers) + r")"
            r"\s*\(\s*(\d+)\s*,\s*(\w+)\s*,\s*\w+\s*,\s*(true|false)\s*\)"
        )
        for m in invocation_re.finditer(text):
            if _is_commented(text, m.start()):
                continue
            macro, n, device, hide = m.group(1), m.group(2), m.group(3), m.group(4)
            name = f"OutputRead.{wrappers[macro]}_{n}_{device}"
            gpu, direct_gpu = resolve_device(name, None, bodies, text)
            out.append((name, hide == "true", gpu, direct_gpu))

    return out


def normalize_rel(root, path):
    """Relative path from ``root`` to ``path``, always POSIX-separated. ``os.path.relpath`` uses
    the native separator (``\\`` on Windows), and the analysis side-car is keyed with ``/``.
    Without this normalization a Windows run would never look up an existing side-car entry."""
    return os.path.relpath(path, root).replace(os.sep, "/")


def fallback_category(fname):
    stem = fname[:-4] if fname.endswith(".cpp") else fname
    for kw, cat in FALLBACK_RULES:
        if kw.lower() in stem.lower():
            return cat
    return "Miscellaneous"


def anchor(cat):
    a = cat.lower().replace(" & ", "-").replace("/", "-").replace(",", "")
    return re.sub(r"-+", "-", re.sub(r"\s+", "-", a.strip()))


def build_page(root, tree_rel, title, intro, analysis, missing_counter, out_dir, matched_keys):
    tree_abs = os.path.join(root, tree_rel)
    files = []
    for dirpath, _, names in os.walk(tree_abs):
        for n in sorted(names):
            if n.endswith(".cpp"):
                p = os.path.join(dirpath, n)
                with open(p, encoding="utf-8", errors="replace") as fh:
                    text = fh.read()
                rows = extract_rows(text)
                if rows:
                    files.append((p, rows))
    files.sort(key=lambda x: os.path.basename(x[0]))

    groups = {}
    heavy_rows = []
    for p, rows in files:
        rel = normalize_rel(root, p)
        matched_keys.add(rel)
        an = analysis.get(rel, {})
        cat = an.get("category") or fallback_category(os.path.basename(p))
        groups.setdefault(cat, []).append((p, rows, an))
        acases = an.get("cases", {})
        row_names = {name for name, *_ in rows}
        for name, src_hidden, *_device in rows:
            if src_hidden or acases.get(name, {}).get("heavy"):
                heavy_rows.append(name)
        stale = sorted(set(acases) - row_names)
        if stale:
            missing_counter[0] += len(stale)
            logger.warning("%s: analysis entries no longer match a registered row (stale?): %s",
                            rel, ", ".join(stale))

    total = sum(len(r) for _, r in files)
    L = [f"# {title}", "", intro, "",
         f"- **Benchmark files:** {len(files)}",
         f"- **Total registered rows:** {total}",
         f"- **Categories:** {len(groups)}",
         f"- **Heavy rows:** {len(heavy_rows)}", "",
         "Descriptions, tags, and per-row notes below are derived from reading the actual benchmark "
         "bodies (what each row's `step()` actually times) and `tests/benchmarks/README.md`, not "
         "just the row names. Use this to find which benchmark rows cover a subsystem you are "
         "changing.", "",
         "> Maintenance: regenerate with `python scripts/get_benchmark_summary.py` (or the "
         "`generate_benchmark_summary` cmake target) after adding/removing/renaming a registered "
         "row. See _Regenerating_ at the bottom.", ""]

    if heavy_rows:
        L += ["## Heavy / do not loop", "",
              "These rows are hidden by default (`Register<T, true>`). Run each once, deliberately, "
              "with `--hidden` -- never as part of a repeated local iteration loop.", ""]
        for name in sorted(heavy_rows):
            L.append(f"- `{name}`")
        L.append("")

    L.append("## Category index")
    L.append("")
    for cat in sorted(groups):
        n = sum(len(r) for _, r, _ in groups[cat])
        L.append(f"- [{cat}](#{anchor(cat)}) -- {len(groups[cat])} file(s), {n} row(s)")
    L.append("")

    for cat in sorted(groups):
        L += [f"## {cat}", ""]
        for p, rows, an in sorted(groups[cat], key=lambda x: os.path.basename(x[0])):
            fname = os.path.basename(p)
            tags = an.get("tags") or [cat]
            desc = an.get("description", "")
            acases = an.get("cases", {})
            L += [f"### `{fname}` ({len(rows)})", ""]
            if desc:
                L += [desc, ""]
            L += ["Tags: " + ", ".join(f"`{t}`" for t in tags), ""]
            for name, src_hidden, requires_gpu, requires_direct_gpu in rows:
                case_an = acases.get(name, {})
                note = case_an.get("note", "")
                heavy = src_hidden or case_an.get("heavy", False)
                if name not in acases:
                    missing_counter[0] += 1
                nm = name.replace("|", "\\|")
                marker = " *(heavy)*" if heavy else ""
                # Rows whose name has no _gpu/_cpu suffix but whose isValid() gates on
                # BmGlobals::forceGpu()/directGpu() (e.g. Clone.envs_64, Lab.anymal_*) get an
                # explicit device note. Otherwise nothing in the row name or this catalogue would
                # tell a caller which single device pass to pin.
                if not name.endswith("_gpu") and not name.endswith("_cpu"):
                    if requires_direct_gpu:
                        marker += " *(requires --forceGpu --directGpu)*"
                    elif requires_gpu:
                        marker += " *(requires --forceGpu)*"
                L.append(f"- `{nm}`{marker}" + (f" -- {note}" if note else ""))
            L.append("")

    L += ["---", "", "## Regenerating", "",
          "This file is a build artifact -- it is regenerated on demand into "
          "`_build/generated/benchmark-summaries/` (CMake target `generate_benchmark_summary`, "
          "built when `OVPHYSX_BUILD_BENCHMARKS=ON`) and is NOT checked in. It is produced by "
          "`python scripts/get_benchmark_summary.py`, which extracts every registered row from the "
          "sources and merges per-file analysis from the source-controlled side-car "
          "`scripts/benchmark_summary_analysis.json`. To refresh it without a build, run that "
          "script directly; add or refresh the file's entry in the analysis side-car to keep this "
          "catalogue useful.", ""]
    out_path = os.path.join(out_dir, "ovphysx", "BENCHMARK_SUMMARY.md")
    return out_path, "\n".join(L) + "\n"


def generate(root, check=False, out_dir=None):
    """Core entry point. ``root`` is the ovphysx root. ``out_dir`` is where the generated catalogue
    is written (defaults to ``root/_build/generated/benchmark-summaries``). Returns the number of
    missing analysis notes."""
    analysis_path = os.path.join(root, ANALYSIS_REL)
    analysis = {}
    if os.path.exists(analysis_path):
        with open(analysis_path, encoding="utf-8") as fh:
            analysis = json.load(fh)
    else:
        logger.warning("%s not found; using filename heuristic only", analysis_path)

    if out_dir is None:
        out_dir = os.path.join(root, GENERATED_REL)

    missing = [0]
    matched_keys = set()
    for tree_rel, title, intro in TREES:
        out_path, md = build_page(root, tree_rel, title, intro, analysis, missing, out_dir, matched_keys)
        os.makedirs(os.path.dirname(out_path), exist_ok=True)
        with open(out_path, "w", encoding="utf-8") as fh:
            fh.write(md)
        logger.info("wrote %s (%d lines)", out_path, md.count("\n"))
        print(f"wrote {out_path} ({md.count(chr(10))} lines)")

    # Whole-file orphans: a side-car key whose .cpp is gone (deleted/renamed) or now registers no
    # recognized rows. Per-case staleness (build_page above) only catches drift within a file that
    # still matched and still has rows. It never visits a key outside `matched_keys`, so an
    # orphaned key would otherwise pass --check silently.
    orphan_keys = sorted(set(analysis) - matched_keys)
    if orphan_keys:
        missing[0] += len(orphan_keys)
        for key in orphan_keys:
            logger.warning("%s: analysis side-car key has no matching source file with registered "
                            "rows (deleted/renamed/no recognized rows?)", key)

    if missing[0]:
        print(f"{missing[0]} row(s) have no analysis note (shown name-only)")
    return missing[0]


if __name__ == "__main__":
    logging.basicConfig(level=logging.WARNING, format="%(message)s")
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--check", action="store_true", help="exit non-zero if any row lacks an analysis note")
    ap.add_argument("--out-dir", default=None,
                    help="output directory (default: <root>/_build/generated/benchmark-summaries)")
    args = ap.parse_args()
    # ovphysx root = one level up from scripts/
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    n = generate(repo_root, check=args.check, out_dir=args.out_dir)
    if args.check and n:
        raise SystemExit(1)
