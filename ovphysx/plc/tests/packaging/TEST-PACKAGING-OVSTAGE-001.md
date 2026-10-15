<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-PACKAGING-OVSTAGE-001
maps_to: REQ-PACKAGING-OVSTAGE-001
type: unit
---

## Scenario

Unit coverage for the OVStage acquisition path: archive-name and URL derivation,
the mandatory digest check, the extraction contract, and the release tooling that
switches mirrors and regenerates digests. The tests build stand-in archives rather
than downloading, so they run offline in the standard python-test job.

## Given

- `scripts/fetch_ovstage_release.py` and `scripts/sync_ovstage_version.py`
  imported directly by the python-test suite.
- Synthetic native archives with the published root layout (`include/`, `bin/`,
  `lib/cmake/ovstage/`, `python/ovstage`, `THIRD-PARTY-NOTICES.txt`), plus
  malformed variants.

## When

- `_zip_name` / `_zip_url` derive the archive name and URL for a pinned build
  against both mirror templates.
- `_digest` is asked for a platform that is unpinned or only partly pinned.
- `_download` verifies a payload whose digest matches and one whose does not.
- `_extract_zip` extracts a well-formed archive and each malformed variant.
- `apply_zip_mirror` switches channels; `compute_digests` runs against a build
  whose archives are only partly published.
- `verify_digests` checks a target channel serving different native archive bytes.
- `validate_python_test_uv_lock` inspects the checked-in lockfiles, a lock using
  the retired local path registry, and a lock whose wheels carry no hash.

- The actual staging command exports the tracked source subset without PhysX.
- Both internal-reference scanners inspect the canonical OVStage assignment,
  other endpoints, and references elsewhere, with and without private staging mode.

## Then

- The archive name is `ovstage@<numeric>.<githash>.<platform>.zip`, using the
  archive's own platform tokens; a pin without a git hash is rejected. The
  internal URL keys off the build number and the public one off the release tag
  (REQ AC-1, AC-3).
- A digest mismatch reports the expected and actual hashes; an unpinned
  platform names `sync_ovstage_version` (REQ AC-2).
- Extraction lands the package prefix at the OVStage root with no `ovstage/`
  prefix, replaces any previous tree, and rejects zip-slip entries, a missing
  `include/`/`bin/`, and missing `THIRD-PARTY-NOTICES.txt` (REQ AC-1).
- `apply_zip_mirror` rewrites the archive URL without writing to disk, so a later
  abort cannot leave the channel half-switched; `compute_digests` aborts on a
  build with an unpublished platform archive instead of emitting a partial table
  (REQ AC-3).
- A source-only switch rejects native archives that differ from the committed
  digests (REQ AC-3).
- The checked-in lockfiles resolve ovstage from an index URL with a sha256 per
  wheel; a relative path registry and an unhashed wheel are both rejected
  (REQ AC-6).

- Opt-in retries recover from transient transfer failures within the configured
  window. Authentication, certificate, disk, and payload failures stop immediately.
  Downloads do not replace a neighboring archive with the same filename.

- Staging preserves the native fetcher and Python project/lock contents. Public
  validation still rejects the internal native URL by default; the private staging
  exception permits only that assignment in its expected file (REQ AC-7).

## Not Covered

- Cold SDK bootstrap and package-not-found reporting lack a committed integration
  test. `scripts/test_cpp_samples.cmake` covers supplied-package discovery and
  launches Windows samples through the generated runtime-path helpers (AC-4, AC-5).
  A test using the published SDK archive remains follow-up work in ADR-0033.

## Code References

- ovphysx/tests/python_tests/test_public_build_flags.py
- ovphysx/tests/python_tests/test_oss_release_packaging.py
- ovphysx/tests/python_tests/test_fetch_ovstage_release.py
- ovphysx/tests/python_tests/test_ovstage_version_sync.py
- ovphysx/scripts/validate_python_test_uv_lock.py
