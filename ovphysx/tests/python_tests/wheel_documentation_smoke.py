# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PACKAGING-DOCS-001
# @covers AC-1 AC-2 AC-3

"""Validate the portable documentation bundled in an ovphysx artifact."""

from __future__ import annotations

import argparse
import importlib.util
import re
from pathlib import Path
from urllib.parse import unquote, urlsplit

_PUBLIC_DOC_SUFFIXES = {".jpg", ".md", ".png", ".rst"}
_MARKDOWN_LINK_RE = re.compile(r"!?\[[^\]]*\]\(([^)]+)\)")

# Sphinx publishes the C headers as download artifacts under hashed directories,
# so they are checked by name rather than by path.
_RENDERED_DOC_HEADERS = ("ovphysx.h", "ovphysx_types.h", "ovphysx_config.h")


def _installed_package_root() -> Path:
    package_spec = importlib.util.find_spec("ovphysx")
    if package_spec is None or package_spec.origin is None:
        raise RuntimeError("Installed ovphysx package was not found")
    return Path(package_spec.origin).parent


def _public_source_paths(source_docs: Path) -> set[Path]:
    return {
        path.relative_to(source_docs)
        for path in source_docs.rglob("*")
        if path.is_file()
        and path.suffix.lower() in _PUBLIC_DOC_SUFFIXES
        and "internal" not in path.relative_to(source_docs).parts
    }


def _markdown_destination(raw_destination: str) -> str:
    destination = raw_destination.strip()
    if destination.startswith("<"):
        closing_angle = destination.find(">")
        if closing_angle < 0:
            return destination
        return destination[1:closing_angle]

    escaped = False
    target_characters: list[str] = []
    for character in destination:
        if escaped:
            target_characters.append(character)
            escaped = False
        elif character == "\\":
            escaped = True
        elif character.isspace():
            break
        else:
            target_characters.append(character)
    if escaped:
        target_characters.append("\\")
    return "".join(target_characters)


def _broken_relative_links(package_root: Path, *, allow_unbundled_header_links: bool = False) -> list[str]:
    findings: list[str] = []
    package_include = (package_root / "include").resolve()
    for markdown_path in sorted(package_root.rglob("*.md")):
        relative_markdown = markdown_path.relative_to(package_root)
        for line_number, line in enumerate(markdown_path.read_text(encoding="utf-8").splitlines(), 1):
            for match in _MARKDOWN_LINK_RE.finditer(line):
                target = _markdown_destination(match.group(1))
                parsed_target = urlsplit(target)
                if parsed_target.scheme or target.startswith(("//", "#")):
                    continue

                target_path = unquote(parsed_target.path)
                resolved = (markdown_path.parent / target_path).resolve()
                if not resolved.is_relative_to(package_root):
                    findings.append(f"{relative_markdown.as_posix()}:{line_number}: {target}")
                    continue

                is_sdk_header = (
                    relative_markdown.parts[0] == "docs"
                    and target_path.startswith("../include/")
                    and resolved.is_relative_to(package_include)
                )
                if is_sdk_header and allow_unbundled_header_links:
                    # The C/C++ headers ship only in the SDK and are outside this
                    # wheel-documentation bug's scope (NVBug 6560076).
                    continue

                if not resolved.exists():
                    findings.append(f"{relative_markdown.as_posix()}:{line_number}: {target}")
    return findings


def _rendered_docs_findings(package_docs: Path) -> list[str]:
    """Report what the rendered documentation tree is missing, if anything.

    The rendered tree is built once and reused by every platform, so its absence
    means a platform silently shipped a smaller artifact than its siblings --
    which is exactly how the aarch64 wheel lost all 118 rendered files for two
    releases (NVBug 6717253).
    """
    rendered_root = package_docs / "html"
    if not rendered_root.is_dir():
        return [f"the rendered documentation tree is absent: {rendered_root}"]

    findings: list[str] = []
    if not (rendered_root / "index.html").is_file():
        findings.append(f"no index.html under {rendered_root}")

    downloadable = {path.name for path in rendered_root.rglob("*") if path.is_file()}
    missing_headers = [name for name in _RENDERED_DOC_HEADERS if name not in downloadable]
    if missing_headers:
        findings.append(f"downloadable C headers are absent: {', '.join(missing_headers)}")
    return findings


def validate(
    package_root: Path,
    source_docs: Path,
    *,
    allow_unbundled_header_links: bool = False,
    require_rendered_docs: bool = False,
) -> None:
    package_docs = package_root / "docs"
    if not package_docs.is_dir():
        raise RuntimeError(f"Bundled documentation directory was not found: {package_docs}")

    if require_rendered_docs:
        rendered_findings = _rendered_docs_findings(package_docs)
        if rendered_findings:
            rendered_lines = "\n".join(f"  {finding}" for finding in rendered_findings)
            raise RuntimeError(f"Rendered documentation is missing from the artifact:\n{rendered_lines}")

    internal_paths = sorted(
        path.relative_to(package_docs)
        for path in package_docs.rglob("*")
        if "internal" in path.relative_to(package_docs).parts
    )
    if internal_paths:
        internal_lines = "\n".join(f"  {path.as_posix()}" for path in internal_paths)
        raise RuntimeError(f"Internal documentation leaked into the artifact:\n{internal_lines}")

    expected = _public_source_paths(source_docs)
    actual = {
        path.relative_to(package_docs)
        for path in package_docs.rglob("*")
        if path.is_file()
        and path.suffix.lower() in _PUBLIC_DOC_SUFFIXES
        and path.relative_to(package_docs).parts[0] != "html"
    }

    missing = sorted(expected - actual)
    if missing:
        missing_lines = "\n".join(f"  {path.as_posix()}" for path in missing)
        raise RuntimeError(f"Public documentation is missing from the artifact:\n{missing_lines}")

    unexpected = sorted(actual - expected)
    if unexpected:
        unexpected_lines = "\n".join(f"  {path.as_posix()}" for path in unexpected)
        raise RuntimeError(f"Unexpected portable documentation is bundled in the artifact:\n{unexpected_lines}")

    mismatched_raw = sorted(
        relative_path
        for relative_path in expected
        if relative_path.suffix.lower() != ".md"
        and (source_docs / relative_path).read_bytes() != (package_docs / relative_path).read_bytes()
    )
    if mismatched_raw:
        mismatch_lines = "\n".join(f"  {path.as_posix()}" for path in mismatched_raw)
        raise RuntimeError(f"RST or image content differs from its public source:\n{mismatch_lines}")

    broken = _broken_relative_links(package_root, allow_unbundled_header_links=allow_unbundled_header_links)
    if broken:
        broken_lines = "\n".join(f"  {finding}" for finding in broken)
        raise RuntimeError(f"Bundled Markdown has broken relative links:\n{broken_lines}")


def main() -> None:
    default_source_docs = Path(__file__).resolve().parents[2] / "docs"
    parser = argparse.ArgumentParser()
    parser.add_argument("--package-root", type=Path, default=None)
    parser.add_argument("--source-docs", type=Path, default=default_source_docs)
    parser.add_argument("--allow-unbundled-header-links", action="store_true")
    parser.add_argument("--require-rendered-docs", action="store_true")
    args = parser.parse_args()

    package_root = args.package_root or _installed_package_root()
    validate(
        package_root.resolve(),
        args.source_docs.resolve(),
        allow_unbundled_header_links=args.allow_unbundled_header_links,
        require_rendered_docs=args.require_rendered_docs,
    )


if __name__ == "__main__":
    main()
