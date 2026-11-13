# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PACKAGING-DOCS-001
# @covers AC-1 AC-2 AC-3

"""Exercise the real CMake documentation staging and its standalone CI gate."""

from pathlib import Path
import shutil
import subprocess
import sys

import pytest
from PIL import Image

from wheel_documentation_smoke import _RENDERED_DOC_HEADERS


PROJECT = Path(__file__).resolve().parents[2]
CHECK = PROJECT / "scripts/test_documentation_package.py"


def write(path, content="fixture\n"):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")


@pytest.fixture
def project(tmp_path):
    root = tmp_path / "project with spaces"
    for relative in (
        "scripts/stage_documentation.cmake", "scripts/preprocess_docs.cmake",
        "scripts/preprocess_markdown.py", "scripts/crossplatform_helpers.cmake",
        "cmake/public_headers.cmake", "tests/python_tests/wheel_documentation_smoke.py",
    ):
        destination = root / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(PROJECT / relative, destination)
    write(root / "docs/guide.md", "# Guide\n[Header](../include/ovphysx/api.h)\n")
    write(root / "docs/internal/hidden.md")
    write(root / "SKILLS.md", "[Skill](skills/example/SKILL.md)\n")
    write(root / "skills/example/SKILL.md", "# Skill\n[Guide](../../docs/guide.md)\n")
    write(root / "SDK_README.md", "# SDK\n[Guide](docs/guide.md)\n")
    write(root / "tests/c_samples/example/main.c")
    write(root / "tests/c_samples/example/_build/local.cache")
    write(root / "tests/python_samples/example.py")
    write(root / "tests/python_samples/.venv/local.cache")
    write(root / "tests/python_samples/uv.lock")
    write(root / "tests/data/scene.usda")
    write(root / "include/ovphysx/api.h")
    write(root / "include/ovphysx/not_a_header.txt")
    write(root / "companion/include/companion/api.hpp")
    write(root / "companion/include/companion/not_a_header.txt")
    return root


@pytest.fixture
def html(tmp_path):
    root = tmp_path / "rendered docs"
    write(root / "index.html", "<html>Documentation</html>\n")
    for header in _RENDERED_DOC_HEADERS:
        write(root / "_downloads/fixture" / header)
    return root


def run_check(project, *args):
    return subprocess.run(
        [sys.executable, str(CHECK), "--project-root", str(project), *map(str, args)],
        text=True, capture_output=True,
    )


def test_shared_staging_preserves_artifact_specific_layouts(project, tmp_path):
    for artifact in ("sdk", "wheel"):
        destination = tmp_path / artifact
        # The real SDK installer must clear Python samples from an older install.
        if artifact == "sdk":
            write(destination / "samples/python_samples/stale.py")
        subprocess.run([
            "cmake", f"-DPYTHON={sys.executable}", f"-DPROJECT_ROOT={project}",
            f"-DPACKAGE_ROOT={destination}", f"-DARTIFACT={artifact}", "-DREQUIRE_HTML=OFF",
            "-P", str(project / "scripts/stage_documentation.cmake"),
        ], check=True, capture_output=True, text=True)
        assert (destination / "docs/guide.md").is_file()
        assert not (destination / "docs/internal").exists()
        assert (destination / "SKILLS.md").is_file()
        assert (destination / "skills/example/SKILL.md").is_file()
        assert (destination / "samples/data/scene.usda").is_file()
        if artifact == "sdk":
            assert (destination / "README.md").is_file()
            assert (destination / "samples/c_samples/example/main.c").is_file()
            assert not (destination / "samples/c_samples/example/_build").exists()
            assert not (destination / "samples/python_samples").exists()
            assert (destination / "include/ovphysx/api.h").is_file()
            assert (destination / "include/companion/api.hpp").is_file()
            assert not list((destination / "include").rglob("*.txt"))
        else:
            assert not (destination / "include").exists()
            assert not (destination / "samples/c_samples").exists()
            assert (destination / "samples/python_samples/example.py").is_file()
            assert not (destination / "samples/python_samples/.venv").exists()
            assert not (destination / "samples/python_samples/uv.lock").exists()


def test_complete_check_accepts_real_staging(project, html):
    result = run_check(project, "--html-docs", html)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "sdk: packaged documentation validated" in result.stdout
    assert "wheel: packaged documentation validated" in result.stdout


@pytest.mark.parametrize("mutation, diagnostic", [
    ("myst", "Unhandled MyST directive"),
    ("skill_link", "Bundled Markdown has broken relative links"),
    ("header_download", "downloadable C headers are absent"),
    ("unshipped_sdk_header", "Bundled Markdown has broken relative links"),
])
def test_complete_check_rejects_documentation_regressions(project, html, mutation, diagnostic):
    if mutation == "myst":
        write(project / "docs/guide.md", "```{note}\nA valid Sphinx note.\n```\n")
    elif mutation == "skill_link":
        write(project / "SKILLS.md", "[Missing](skills/example/MISSING.md)\n")
    elif mutation == "header_download":
        (html / "_downloads/fixture" / _RENDERED_DOC_HEADERS[0]).unlink()
    else:
        write(project / "docs/guide.md", "[Not shipped](../include/ovphysx/not_a_header.txt)\n")
    result = run_check(project, "--html-docs", html)
    assert result.returncode != 0
    assert diagnostic in result.stdout + result.stderr


def test_rendered_validation_can_only_be_omitted_explicitly(project, tmp_path):
    assert run_check(project).returncode != 0
    assert run_check(project, "--html-docs", tmp_path / "missing").returncode != 0
    result = run_check(project, "--portable-only")
    assert result.returncode == 0, result.stdout + result.stderr
    assert "Portable-only validation" in result.stdout


@pytest.mark.parametrize("location", ["portable", "rendered"])
def test_complete_check_rejects_pointer_images(project, html, location):
    source_image = project / "docs/diagram.png"
    rendered_image = html / "_images/diagram.png"
    rendered_image.parent.mkdir()
    Image.new("RGB", (8, 8)).save(source_image)
    shutil.copy2(source_image, rendered_image)
    target = source_image if location == "portable" else rendered_image
    target.write_text("version https://git-lfs.github.com/spec/v1\n"
                      + "oid sha256:" + "0" * 64 + "\nsize 1000\n", encoding="utf-8")
    result = run_check(project, "--html-docs", html)
    assert result.returncode != 0
    assert "Git LFS pointer" in result.stdout + result.stderr


def test_complete_check_decodes_real_images_in_both_layouts(project, html):
    for name, format in (("diagram.png", "PNG"), ("photo.jpg", "JPEG")):
        source = project / "docs" / name
        Image.new("RGB", (8, 8), (20, 100, 50)).save(source, format=format)
        destination = html / "_images" / name
        destination.parent.mkdir(exist_ok=True)
        shutil.copy2(source, destination)
    result = run_check(project, "--html-docs", html)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "sdk: packaged documentation validated" in result.stdout
    assert "wheel: packaged documentation validated" in result.stdout
