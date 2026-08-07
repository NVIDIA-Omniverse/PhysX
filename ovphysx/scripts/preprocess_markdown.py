# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

"""Preprocess MyST Markdown for shipping as portable plain Markdown.

Transforms source .md files (authored for Sphinx/MyST) into clean Markdown
that renders correctly in GitHub, SDK README, LLM contexts, and any standard
Markdown viewer. Sphinx reads the source files directly and is unaffected.

Handled directives (fenced blocks):
  {literalinclude}  -> inlined code block (reads referenced source file)
  {toctree}         -> stripped entirely
  {eval-rst}        -> stripped entirely
  anything else     -> hard error (must be converted in source first)

Inline roles (e.g., {doc}`path`, {ref}`label`):
  any occurrence    -> hard error (must be converted to Markdown links)
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path


def _resolve_literalinclude(
    source_md_dir: Path,
    fence_info: str,
    body_lines: list[str],
    project_root: Path | None = None,
) -> list[str]:
    """Resolve a {literalinclude} block into an inline fenced code block."""
    rel_path = fence_info.strip()
    if not rel_path:
        raise ValueError("{literalinclude} directive has no file path")

    target = (source_md_dir / rel_path).resolve()
    if project_root is not None and not target.is_relative_to(project_root):
        raise ValueError(
            f"{{literalinclude}} target {target} escapes project root "
            f"({project_root})"
        )
    if not target.is_file():
        raise FileNotFoundError(
            f"{{literalinclude}} target not found: {rel_path} "
            f"(resolved to {target})"
        )

    opts: dict[str, str] = {}
    for line in body_lines:
        stripped = line.strip()
        if stripped.startswith(":") and ":" in stripped[1:]:
            key, _, value = stripped[1:].partition(":")
            opts[key.strip()] = value.strip()

    supported = {"language", "lines", "start-after", "end-before"}
    unsupported = set(opts) - supported
    if unsupported:
        raise ValueError(
            f"{{literalinclude}} uses unsupported options {unsupported} "
            f"in reference to {rel_path}. Add support in preprocess_markdown.py "
            f"before proceeding."
        )
    if "lines" in opts and ("start-after" in opts or "end-before" in opts):
        raise ValueError(
            f"{{literalinclude}} in reference to {rel_path} specifies both "
            f":lines: and marker-based options (:start-after:/:end-before:). "
            f"Use one or the other."
        )

    language = opts.get("language", "")
    all_lines = target.read_text(encoding="utf-8").splitlines()

    if "start-after" in opts or "end-before" in opts:
        start = 0
        end = len(all_lines)
        if "start-after" in opts:
            marker = opts["start-after"]
            for i, line in enumerate(all_lines):
                if marker in line:
                    start = i + 1
                    break
            else:
                raise ValueError(
                    f"start-after marker {marker!r} not found in {rel_path}"
                )
        if "end-before" in opts:
            marker = opts["end-before"]
            for i, line in enumerate(all_lines[start:], start):
                if marker in line:
                    end = i
                    break
            else:
                raise ValueError(
                    f"end-before marker {marker!r} not found in {rel_path}"
                )
        selected = all_lines[start:end]
    elif "lines" in opts:
        parts = opts["lines"].split("-", maxsplit=1)
        start = int(parts[0]) - 1
        if len(parts) == 1:
            end = start + 1
        elif parts[1]:
            end = int(parts[1])
        else:
            end = len(all_lines)
        start = max(0, min(start, len(all_lines)))
        end = max(start, min(end, len(all_lines)))
        selected = all_lines[start:end]
    else:
        selected = all_lines

    result = [f"```{language}"]
    result.extend(selected)
    result.append("```")
    return result



_MYST_FENCE_RE = re.compile(r"^(`{3,})\{(\w[\w-]*)\}\s*(.*)?$")
_STD_FENCE_RE = re.compile(r"^(`{3,})")
_MYST_INLINE_ROLE_RE = re.compile(r"\{(\w[\w-]*)\}`[^`]*`")
STRIP_DIRECTIVES = {"toctree", "eval-rst"}


def preprocess(
    source_path: Path, output_path: Path, project_root: Path | None = None
) -> None:
    if project_root is not None:
        project_root = project_root.resolve()
    source_md_dir = source_path.parent.resolve()
    text = source_path.read_text(encoding="utf-8")
    lines = text.splitlines()

    result: list[str] = []
    i = 0
    while i < len(lines):
        line = lines[i]
        m = _MYST_FENCE_RE.match(line)

        if m:
            fence_ticks = m.group(1)
            directive = m.group(2)
            fence_info = m.group(3) or ""
            min_ticks = len(fence_ticks)

            body_lines: list[str] = []
            i += 1
            while i < len(lines):
                close = _STD_FENCE_RE.match(lines[i])
                if close and len(close.group(1)) >= min_ticks:
                    i += 1
                    break
                body_lines.append(lines[i])
                i += 1

            if directive in STRIP_DIRECTIVES:
                continue
            elif directive == "literalinclude":
                result.extend(
                    _resolve_literalinclude(
                        source_md_dir, fence_info, body_lines, project_root
                    )
                )
            else:
                raise ValueError(
                    f"Unhandled MyST directive '{{{directive}}}' in "
                    f"{source_path}. Convert to standard Markdown in source "
                    f"or add handling in preprocess_markdown.py."
                )
        else:
            result.append(line)
            i += 1

    for line_num, line in enumerate(result, 1):
        m_role = _MYST_INLINE_ROLE_RE.search(line)
        if m_role:
            raise ValueError(
                f"Unhandled MyST inline role '{{{m_role.group(1)}}}' at "
                f"{source_path}:{line_num}. Convert to standard Markdown "
                f"link in source (e.g., {{doc}}`path` -> [title](path.md))."
            )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(result) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Preprocess MyST Markdown for portable shipping."
    )
    parser.add_argument("input", type=Path, help="Source .md file")
    parser.add_argument("output", type=Path, help="Destination .md file")
    parser.add_argument(
        "--project-root",
        type=Path,
        default=None,
        help="Project root boundary for {literalinclude} path validation",
    )
    args = parser.parse_args()

    try:
        preprocess(args.input, args.output, args.project_root)
    except (FileNotFoundError, ValueError) as e:
        print(f"ERROR: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
