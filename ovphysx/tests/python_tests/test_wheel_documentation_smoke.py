# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PACKAGING-DOCS-001
# @covers AC-1 AC-2

from pathlib import Path

import pytest
from wheel_documentation_smoke import _broken_relative_links, validate


def test_supported_markdown_destinations_with_whitespace(tmp_path: Path) -> None:
    package_root = tmp_path / "package"
    docs = package_root / "docs"
    docs.mkdir(parents=True)
    target = docs / "space file.md"
    target.write_text("# Target\n", encoding="utf-8")
    (docs / "index.md").write_text(
        "[angle](<space file.md>)\n" "[escaped](space\\ file.md)\n" '[title](space%20file.md "Title")\n',
        encoding="utf-8",
    )

    assert _broken_relative_links(package_root) == []

    target.unlink()
    assert len(_broken_relative_links(package_root)) == 3


def test_relative_markdown_destination_cannot_escape_package(tmp_path: Path) -> None:
    package_root = tmp_path / "package"
    docs = package_root / "docs"
    docs.mkdir(parents=True)
    (tmp_path / "outside.md").write_text("# Outside\n", encoding="utf-8")
    (docs / "index.md").write_text("[outside](../../outside.md)\n", encoding="utf-8")

    assert _broken_relative_links(package_root) == ["docs/index.md:1: ../../outside.md"]


def test_header_exception_cannot_escape_package(tmp_path: Path) -> None:
    package_root = tmp_path / "package"
    docs = package_root / "docs"
    docs.mkdir(parents=True)
    (tmp_path / "outside.md").write_text("# Outside\n", encoding="utf-8")
    (docs / "index.md").write_text("[outside](../include/../../outside.md)\n", encoding="utf-8")

    assert _broken_relative_links(package_root, allow_unbundled_header_links=True) == [
        "docs/index.md:1: ../include/../../outside.md"
    ]


def test_header_exception_applies_only_to_wheel(tmp_path: Path) -> None:
    package_root = tmp_path / "package"
    docs = package_root / "docs"
    include = package_root / "include"
    docs.mkdir(parents=True)
    include.mkdir()
    (docs / "index.md").write_text("[header](../include/header.h)\n", encoding="utf-8")

    assert _broken_relative_links(package_root) == ["docs/index.md:1: ../include/header.h"]
    assert _broken_relative_links(package_root, allow_unbundled_header_links=True) == []

    (include / "header.h").write_text("/* header */\n", encoding="utf-8")
    assert _broken_relative_links(package_root) == []


def test_validate_rejects_nested_internal_docs(tmp_path: Path) -> None:
    source_docs = tmp_path / "source-docs"
    package_root = tmp_path / "package"
    package_docs = package_root / "docs"
    source_docs.mkdir()
    package_docs.mkdir(parents=True)
    (source_docs / "index.md").write_text("# Index\n", encoding="utf-8")
    (package_docs / "index.md").write_text("# Index\n", encoding="utf-8")
    internal = package_docs / "guides" / "internal"
    internal.mkdir(parents=True)
    (internal / "secret.md").write_text("# Secret\n", encoding="utf-8")

    with pytest.raises(RuntimeError, match="Internal documentation leaked"):
        validate(package_root, source_docs)


def test_validate_rejects_stale_portable_docs(tmp_path: Path) -> None:
    source_docs = tmp_path / "source-docs"
    package_root = tmp_path / "package"
    package_docs = package_root / "docs"
    source_docs.mkdir()
    package_docs.mkdir(parents=True)
    (source_docs / "index.md").write_text("# Index\n", encoding="utf-8")
    (package_docs / "index.md").write_text("# Index\n", encoding="utf-8")
    (package_docs / "stale.md").write_text("# Stale\n", encoding="utf-8")

    with pytest.raises(RuntimeError, match="Unexpected portable documentation"):
        validate(package_root, source_docs)


def test_validate_compares_raw_documentation_bytes(tmp_path: Path) -> None:
    source_docs = tmp_path / "source-docs"
    package_root = tmp_path / "package"
    package_docs = package_root / "docs"
    source_docs.mkdir()
    package_docs.mkdir(parents=True)
    (source_docs / "diagram.png").write_bytes(b"source")
    (package_docs / "diagram.png").write_bytes(b"artifact")

    with pytest.raises(RuntimeError, match="RST or image content differs"):
        validate(package_root, source_docs)
