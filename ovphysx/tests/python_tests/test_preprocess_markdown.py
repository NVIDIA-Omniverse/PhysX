# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

"""Tests for scripts/preprocess_markdown.py.

These are pure-Python tests that do not require the ovphysx library.
They validate that MyST directives and inline roles are properly handled
during Markdown preprocessing for portable shipping.
"""

import sys
from pathlib import Path

import pytest

SCRIPTS_DIR = Path(__file__).resolve().parents[2] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from preprocess_markdown import preprocess  # noqa: E402


@pytest.fixture
def tmp_paths(tmp_path):
    """Return (input_path, output_path) in a temporary directory."""
    return tmp_path / "input.md", tmp_path / "output.md"


# ============================================================================
# Fenced directive tests
# ============================================================================


class TestFencedDirectives:
    def test_toctree_stripped(self, tmp_paths):
        src, dst = tmp_paths
        src.write_text(
            "# Title\n"
            "\n"
            "Some text.\n"
            "\n"
            "```{toctree}\n"
            ":maxdepth: 2\n"
            ":caption: Overview\n"
            "\n"
            "page_a\n"
            "page_b\n"
            "```\n"
            "\n"
            "More text.\n"
        )
        preprocess(src, dst)
        result = dst.read_text()
        assert "toctree" not in result
        assert "page_a" not in result
        assert "# Title" in result
        assert "Some text." in result
        assert "More text." in result

    def test_eval_rst_stripped(self, tmp_paths):
        src, dst = tmp_paths
        src.write_text("# Title\n" "\n" "```{eval-rst}\n" ".. automodule:: foo\n" "```\n" "\n" "After.\n")
        preprocess(src, dst)
        result = dst.read_text()
        assert "eval-rst" not in result
        assert "automodule" not in result
        assert "After." in result

    def test_literalinclude_resolved(self, tmp_paths):
        src, dst = tmp_paths
        code_file = src.parent / "example.py"
        code_file.write_text("print('hello')\nprint('world')\n")
        src.write_text("# Demo\n" "\n" "```{literalinclude} example.py\n" ":language: python\n" "```\n")
        preprocess(src, dst, project_root=src.parent)
        result = dst.read_text()
        assert "literalinclude" not in result
        assert "```python" in result
        assert "print('hello')" in result
        assert "print('world')" in result

    def test_unknown_fenced_directive_errors(self, tmp_paths):
        src, dst = tmp_paths
        src.write_text("# Title\n" "\n" "```{note}\n" "This is a note.\n" "```\n")
        with pytest.raises(ValueError, match="Unhandled MyST directive.*note"):
            preprocess(src, dst)

    def test_multiple_toctrees_all_stripped(self, tmp_paths):
        src, dst = tmp_paths
        src.write_text(
            "```{toctree}\n"
            ":caption: A\n"
            "\n"
            "page_a\n"
            "```\n"
            "\n"
            "Middle.\n"
            "\n"
            "```{toctree}\n"
            ":caption: B\n"
            "\n"
            "page_b\n"
            "```\n"
        )
        preprocess(src, dst)
        result = dst.read_text()
        assert "toctree" not in result
        assert "page_a" not in result
        assert "page_b" not in result
        assert "Middle." in result


# ============================================================================
# Inline role tests
# ============================================================================


class TestInlineRoles:
    def test_doc_role_errors(self, tmp_paths):
        src, dst = tmp_paths
        src.write_text("See {doc}`tutorials/quickstart` for details.\n")
        with pytest.raises(ValueError, match="Unhandled MyST inline role.*doc"):
            preprocess(src, dst)

    def test_ref_role_errors(self, tmp_paths):
        src, dst = tmp_paths
        src.write_text("See {ref}`my-label` for details.\n")
        with pytest.raises(ValueError, match="Unhandled MyST inline role.*ref"):
            preprocess(src, dst)

    def test_func_role_errors(self, tmp_paths):
        src, dst = tmp_paths
        src.write_text("Call {func}`mymodule.do_thing` to start.\n")
        with pytest.raises(ValueError, match="Unhandled MyST inline role.*func"):
            preprocess(src, dst)

    def test_role_with_explicit_title_errors(self, tmp_paths):
        src, dst = tmp_paths
        src.write_text("See {doc}`Quickstart Guide <tutorials/quickstart>` here.\n")
        with pytest.raises(ValueError, match="Unhandled MyST inline role.*doc"):
            preprocess(src, dst)

    def test_role_inside_stripped_block_not_checked(self, tmp_paths):
        """Inline roles inside stripped fenced blocks should not trigger errors."""
        src, dst = tmp_paths
        src.write_text("```{toctree}\n" ":caption: {doc}`Overview`\n" "```\n" "\n" "Clean text.\n")
        preprocess(src, dst)
        result = dst.read_text()
        assert "Clean text." in result


# ============================================================================
# Passthrough tests
# ============================================================================


class TestPassthrough:
    def test_standard_markdown_unchanged(self, tmp_paths):
        src, dst = tmp_paths
        content = "# Heading\n" "\n" "A [link](page.md) and **bold** text.\n" "\n" "- item 1\n" "- item 2\n"
        src.write_text(content)
        preprocess(src, dst)
        result = dst.read_text()
        assert "# Heading" in result
        assert "[link](page.md)" in result
        assert "**bold**" in result
        assert "- item 1" in result

    def test_standard_code_fence_not_treated_as_directive(self, tmp_paths):
        src, dst = tmp_paths
        src.write_text("```python\n" "print('hello')\n" "```\n")
        preprocess(src, dst)
        result = dst.read_text()
        assert "```python" in result
        assert "print('hello')" in result

    def test_inline_code_with_braces_not_flagged(self, tmp_paths):
        """Backtick-quoted code like `{something}` should not trigger role detection."""
        src, dst = tmp_paths
        src.write_text("Use `{toctree}` directive for navigation.\n")
        preprocess(src, dst)
        result = dst.read_text()
        assert "`{toctree}`" in result
