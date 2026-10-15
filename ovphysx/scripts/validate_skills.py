#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Run Skill Evaluator over the ovphysx product skills in skills/.

Phases A and B are blocking and are what the per-MR CI gate enforces. Phase C is
opt-in:

  Phase A  skillevaluator validate --external --no-llm --no-dedup
           --external is the public/outbound profile: it keeps org-style authors
           (e.g. "NVIDIA Omniverse Physics") as advisories instead of failing
           author_format, which fits skills shipped in the wheel and SDK.

  Phase B  skillevaluator tier3 validate. A skill with no evals/evals.json fails
           unless it is named in _ALLOW_MISSING_EVALS below. Dataset contract
           only. It checks that the eval cases parse, not that an agent can act
           on them.

  Phase C  skillevaluator evaluate, the agent-driven Tier 3 live evaluation
           (Skill Lift), opt-in via --live / --live-only. Unlike A and B this
           spends real agent time and needs an inference credential plus a
           Harbor backend, so it is off by default and in CI runs only under the
           "Skills Live Agent Eval" pipeline type. An environment without those
           prerequisites is reported as a skip, not a failure.

           Two models are involved, set in two places: the agent under test
           (--live-model, forwarded as `evaluate --model`) and the Harbor
           verifier / judge, which the pinned skillevaluator (1.0.0) reads from
           LLM_JUDGE_MODEL (alias SKILL_EVAL_JUDGE_MODEL). They want different
           answers. A modest agent is the interesting subject, while an
           inconsistent grader just adds noise to the lift number, so neither
           is derived from the other.

Usage:
    python3 scripts/validate_skills.py                  # all skills, phases A+B
    python3 scripts/validate_skills.py --skill basic-workflow
    python3 scripts/validate_skills.py --skills-root path/to/skills
    python3 scripts/validate_skills.py --live-only      # phase C alone

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

# Live-evaluation backends skillevaluator can drive. "local" runs the agent
# unsandboxed on the host, so it belongs only on a disposable machine such as a
# CI pod; "k8s-sandbox" is the managed internal backend; "docker" is the tool's
# own default and needs a daemon the CI runners do not have.
_LIVE_ENV_MODES = ("docker", "k8s-sandbox", "local")

# Skills permitted to ship without a Tier 3 dataset, by directory name.
#
# Empty on purpose: every skill under skills/ has evals/evals.json, so a missing
# dataset is a regression rather than a known gap. Treating it as a failure keeps
# a deleted evals.json from turning a blocking check into a green skip. Add a name
# here (with a comment saying why) only for a deliberate, temporary gap.
_ALLOW_MISSING_EVALS: frozenset[str] = frozenset()


def _judge_model(env: dict[str, str]) -> str | None:
    """Harbor verifier / judge model that skillevaluator 1.0.0 actually uses.

    The pinned tool forwards LLM_JUDGE_MODEL (alias SKILL_EVAL_JUDGE_MODEL) to
    the Harbor verifier. SKILLSPECTOR_MODEL is not that setting and must not be
    logged or documented as if it were: a sentinel in SKILLSPECTOR_MODEL still
    leaves the verifier on its own default.
    """
    for name in ("LLM_JUDGE_MODEL", "SKILL_EVAL_JUDGE_MODEL"):
        value = env.get(name, "").strip()
        if value:
            return value
    return None


def _env_int(name: str) -> int | None:
    """Integer from the environment, or None when unset or empty.

    A malformed value is a configuration mistake and is reported as one rather
    than silently falling back to "no limit", which would quietly uncap a
    budget the CI job meant to bound.
    """
    raw = os.environ.get(name, "").strip()
    if not raw:
        return None
    try:
        return int(raw)
    except ValueError:
        print(f"ERROR: {name} must be an integer, got: {raw!r}", file=sys.stderr)
        sys.exit(2)


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


def _static_phases(skillevaluator: str, skill_dir: Path, out_dir: Path,
                   env: dict[str, str]) -> list[str]:
    """Run phases A and B for one skill and return its failure labels."""
    name = skill_dir.name
    failed: list[str] = []

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

    return failed


def _live_skip_reason(skillevaluator: str, agents: str, env_mode: str,
                      env: dict[str, str]) -> str | None:
    """Why Phase C cannot run here, or None when it can.

    Live evaluation needs an inference credential and an agent backend. Neither
    exists on a plain dev box, and in CI the credential is a protected variable
    that unprotected refs never see. Callers therefore report a reason from here
    as a skip: an unconfigured environment says nothing about the skills.
    """
    if not env.get("NVIDIA_INFERENCE_KEY"):
        return ("NVIDIA_INFERENCE_KEY is not set (a protected CI variable; "
                "locally, export your own key)")
    if _run([skillevaluator, "doctor", "-a", agents, "--env-mode", env_mode], env) != 0:
        return f"skillevaluator doctor reports the {env_mode} backend is not ready"
    return None


def _live_command(skillevaluator: str, skill_dir: Path, results_dir: Path,
                  args: argparse.Namespace) -> list[str]:
    """Phase C command line.

    --results-dir is always passed: left to itself the tool writes trial output
    into the skill's own evals/results/, i.e. into the checkout. --progress plain
    keeps the log readable when there is no TTY, as in CI.
    """
    cmd = [skillevaluator, "evaluate", str(skill_dir),
           "-a", args.live_agents,
           "--env-mode", args.live_env_mode,
           "--results-dir", str(results_dir),
           "--progress", "plain"]
    if args.live_model:
        cmd += ["--model", args.live_model]
    if args.live_max_cases is not None:
        cmd += ["--max-cases", str(args.live_max_cases)]
    if args.live_attempts is not None:
        cmd += ["--n-attempts", str(args.live_attempts)]
    return cmd


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run Skill Evaluator Tier 1, the Tier 3 dataset contract, and "
                    "optionally the Tier 3 live evaluation over ovphysx skills.")
    parser.add_argument(
        "--skills-root", type=Path, default=_PROJECT_ROOT / "skills",
        help="directory holding one subdirectory per skill (default: ovphysx/skills)")
    parser.add_argument(
        "--report-dir", type=Path, default=None,
        help="where to write reports (default: <repo>/_skill_evaluator_reports)")
    parser.add_argument(
        "--skill", default=None, metavar="NAME",
        help="validate only this skill instead of every skill found")

    # Phase C. Defaults come from the environment so the nightly CI job can set
    # its budget in ci/skills/skill_evaluator.yml instead of here.
    live = parser.add_argument_group("live evaluation (Phase C)")
    mode = live.add_mutually_exclusive_group()
    mode.add_argument(
        "--live", action="store_true",
        help="also run the Tier 3 live evaluation after the static phases")
    mode.add_argument(
        "--live-only", action="store_true",
        help="run only the Tier 3 live evaluation, skipping phases A and B")
    live.add_argument(
        "--live-agents", default=os.environ.get("SKILL_EVAL_LIVE_AGENTS", "codex"),
        metavar="LIST", help="comma-separated Harbor agents (default: codex)")
    live.add_argument(
        "--live-env-mode", choices=_LIVE_ENV_MODES,
        default=os.environ.get("SKILL_EVAL_LIVE_ENV_MODE", "docker"),
        help="live-evaluation backend (default: docker, the tool's own default)")
    live.add_argument(
        "--live-model", metavar="ID",
        default=os.environ.get("SKILL_EVAL_LIVE_MODEL", "").strip() or None,
        help="model for the agent under test. The grading model is separate and "
             "comes from LLM_JUDGE_MODEL (alias SKILL_EVAL_JUDGE_MODEL; "
             "default: each side's own default)")
    live.add_argument(
        "--live-max-cases", type=int, default=_env_int("SKILL_EVAL_LIVE_MAX_CASES"),
        metavar="N", help="evaluate at most N cases per skill")
    live.add_argument(
        "--live-attempts", type=int, default=_env_int("SKILL_EVAL_LIVE_ATTEMPTS"),
        metavar="N", help="attempts per case")
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
    run_static = not args.live_only
    run_live = args.live or args.live_only

    print(f"Evaluating {len(skills)} skill(s) under {skills_root}")
    print(f"Reports: {report_dir}")

    live_skip: str | None = None
    if run_live:
        live_skip = _live_skip_reason(skillevaluator, args.live_agents,
                                      args.live_env_mode, env)
        if live_skip:
            print(f"SKIP: live evaluation unavailable -- {live_skip}")
            if not run_static:
                # Nothing left to do, and an unconfigured environment is not a
                # finding. Exit clean so the nightly job does not cry wolf.
                return 0
        else:
            # Both models are logged because a lift number is only comparable
            # against another run that graded the same way with the same agent.
            print(f"Live evaluation: agents={args.live_agents} "
                  f"env-mode={args.live_env_mode} "
                  f"agent-model={args.live_model or 'agent default'} "
                  f"grading-model={_judge_model(env) or 'provider default'}")

    failed: list[str] = []

    for skill_dir in skills:
        name = skill_dir.name
        print("=" * 60)
        print(f"Skill: {name} ({skill_dir})")
        print("=" * 60, flush=True)

        out_dir = report_dir / name
        out_dir.mkdir(parents=True, exist_ok=True)

        if run_static:
            failed += _static_phases(skillevaluator, skill_dir, out_dir, env)

        if run_live and not live_skip:
            if _run(_live_command(skillevaluator, skill_dir, out_dir / "live", args),
                    env) != 0:
                print(f"FAIL: Tier 3 live evaluation failed for {name}", file=sys.stderr)
                failed.append(f"{name} (tier 3 live)")
            else:
                print(f"PASS: Tier 3 live evaluation ({name})")

    print("=" * 60)
    if failed:
        print(f"Skill Evaluator reported {len(failed)} failure(s):", file=sys.stderr)
        for item in failed:
            print(f"  - {item}", file=sys.stderr)
        print(f"Reports under: {report_dir}", file=sys.stderr)
        return 1

    print(f"All Skill Evaluator checks passed. Reports under: {report_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
