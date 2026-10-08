#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Run Skill Evaluator over the ovphysx product skills in skills/.

Two blocking phases per skill, matching what CI enforces:

  Phase A  skillevaluator validate --external --no-llm --no-dedup
           --external is the public/outbound profile: it keeps org-style authors
           (e.g. "NVIDIA Omniverse Physics") as advisories instead of failing
           author_format, which fits skills shipped in the wheel and SDK.

  Phase B  skillevaluator tier3 validate. A skill with no evals/evals.json fails
           unless it is named in _ALLOW_MISSING_EVALS below. Dataset contract
           only. This is NOT the agent-driven Tier 3 live evaluation. That is
           `skillevaluator evaluate`, which needs a Harbor sandbox plus an
           inference key and is not run here.

Usage:
    python3 scripts/validate_skills.py                  # all skills
    python3 scripts/validate_skills.py --skill basic-workflow
    python3 scripts/validate_skills.py --skills-root path/to/skills

Requires skillevaluator on PATH (CI pins it, see ci/skills/run_skill_evaluator.sh).
Exit codes: 0 all passed, 1 a skill failed, 2 setup/tooling problem.
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

_SCRIPT_DIR = Path(__file__).resolve().parent
_PROJECT_ROOT = _SCRIPT_DIR.parent

# Reporters for Phase A. json and markdown are what CI keeps as job artifacts.
_REPORTERS = ("cli", "json", "markdown")

# Skills permitted to ship without a Tier 3 dataset, by directory name.
#
# Empty on purpose: every skill under skills/ has evals/evals.json, so a missing
# dataset is a regression rather than a known gap. Treating it as a failure keeps
# a deleted evals.json from turning a blocking check into a green skip. Add a name
# here (with a comment saying why) only for a deliberate, temporary gap.
_ALLOW_MISSING_EVALS: frozenset[str] = frozenset()


def _find_skillevaluator() -> str | None:
    found = shutil.which("skillevaluator")
    if found:
        return found
    # uv tool install puts it here, which is not always on a non-login PATH.
    for hint in (Path.home() / ".local" / "bin",):
        for name in ("skillevaluator", "skillevaluator.exe"):
            candidate = hint / name
            if candidate.is_file():
                return str(candidate)
    return None


def _child_env() -> dict[str, str]:
    """Environment for skillevaluator subprocesses.

    Its console reporter draws box-drawing characters and check marks through
    rich. On a Windows console defaulting to cp1252 that raises
    UnicodeEncodeError *after* the checks have already run, turning a passing
    validate into a nonzero exit. Force UTF-8 so the exit code reflects the
    checks rather than the terminal encoding.
    """
    env = dict(os.environ)
    env.setdefault("PYTHONUTF8", "1")
    env.setdefault("PYTHONIOENCODING", "utf-8")
    env.setdefault("TERM", "xterm-256color")
    return env


def _discover_skills(skills_root: Path, only: str | None) -> list[Path]:
    skills = sorted(
        entry for entry in skills_root.iterdir()
        if entry.is_dir() and (entry / "SKILL.md").is_file()
    )
    if only:
        skills = [s for s in skills if s.name == only]
    return skills


def _run(cmd: list[str], env: dict[str, str]) -> int:
    print(f"$ {' '.join(cmd)}", flush=True)
    return subprocess.run(cmd, env=env).returncode


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run Skill Evaluator Tier 1 and the Tier 3 dataset contract over ovphysx skills.")
    parser.add_argument(
        "--skills-root", type=Path, default=_PROJECT_ROOT / "skills",
        help="directory holding one subdirectory per skill (default: ovphysx/skills)")
    parser.add_argument(
        "--report-dir", type=Path, default=None,
        help="where to write reports (default: <repo>/_skill_evaluator_reports)")
    parser.add_argument(
        "--skill", default=None, metavar="NAME",
        help="validate only this skill instead of every skill found")
    args = parser.parse_args()

    skillevaluator = _find_skillevaluator()
    if skillevaluator is None:
        print(
            "ERROR: skillevaluator not found on PATH.\n"
            "  Install it with:  uv tool install --python 3.12 skillevaluator==1.0.0\n"
            "  CI installs the pinned version in ci/skills/run_skill_evaluator.sh.",
            file=sys.stderr)
        return 2

    skills_root = args.skills_root.resolve()
    if not skills_root.is_dir():
        print(f"ERROR: skills root not found: {skills_root}", file=sys.stderr)
        return 2

    report_dir = (args.report_dir or _PROJECT_ROOT.parent / "_skill_evaluator_reports").resolve()

    skills = _discover_skills(skills_root, args.skill)
    if not skills:
        if args.skill:
            print(f"ERROR: no skill named '{args.skill}' under {skills_root}", file=sys.stderr)
        else:
            print(f"ERROR: no SKILL.md directories under {skills_root}", file=sys.stderr)
        return 2

    env = _child_env()
    print(f"Evaluating {len(skills)} skill(s) under {skills_root}")
    print(f"Reports: {report_dir}")
    failed: list[str] = []

    for skill_dir in skills:
        name = skill_dir.name
        print("=" * 60)
        print(f"Skill: {name} ({skill_dir})")
        print("=" * 60, flush=True)

        out_dir = report_dir / name
        out_dir.mkdir(parents=True, exist_ok=True)

        phase_a = [skillevaluator, "validate", str(skill_dir),
                   "--external", "--no-llm", "--no-dedup"]
        for reporter in _REPORTERS:
            phase_a += ["-r", reporter]
        phase_a += ["-o", str(out_dir / "full")]

        if _run(phase_a, env) != 0:
            print(f"FAIL: Tier 1 validate --external failed for {name}", file=sys.stderr)
            failed.append(f"{name} (tier 1)")
        else:
            print(f"PASS: Tier 1 validate --external ({name})")

        if (skill_dir / "evals" / "evals.json").is_file():
            if _run([skillevaluator, "tier3", "validate", str(skill_dir)], env) != 0:
                print(f"FAIL: Tier 3 dataset validate failed for {name}", file=sys.stderr)
                failed.append(f"{name} (tier 3 dataset)")
            else:
                print(f"PASS: Tier 3 dataset validate ({name})")
        elif name in _ALLOW_MISSING_EVALS:
            print(f"SKIP: no evals/evals.json for {name} (allowlisted)")
        else:
            print(f"FAIL: {name} has no evals/evals.json. Add a Tier 3 dataset, or "
                  f"allowlist it in _ALLOW_MISSING_EVALS with a reason.", file=sys.stderr)
            failed.append(f"{name} (missing evals dataset)")

    print("=" * 60)
    if failed:
        print(f"Skill Evaluator reported {len(failed)} blocking failure(s):", file=sys.stderr)
        for item in failed:
            print(f"  - {item}", file=sys.stderr)
        print(f"Reports under: {report_dir}", file=sys.stderr)
        return 1

    print(f"All blocking Skill Evaluator checks passed. Reports under: {report_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
