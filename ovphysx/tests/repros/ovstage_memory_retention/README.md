<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# OVStage / ovphysx import memory retention repro

Repeated successful `ovphysx_update_from_ovstage()` calls retain increasing
allocations until the ovstage instance is destroyed. This standalone diagnostic
uses public SDK APIs and four disabled rigid bodies. It performs no simulation
steps, rendering, or physics output reads during the measured loop.

This branch supplies a bug-report reproducer, not a fix. It is not registered
with the normal ovphysx test suite. No SDK binaries or generated logs are committed.

## Build and reproduce

Validated on Linux x86_64, glibc 2.39, GCC 13.3, Python 3.12.3, on 2026-10-09.
Physics uses CPU-only mode. The host has an RTX 5090, driver 595.91.07;
a GPU-free host and other operating systems have not been tested. Allocation
measurement requires Linux/glibc (`mallinfo2`, `malloc_trim`, and `/proc`).

From this directory, obtain the official ovphysx 0.6.4.72118723 SDK and its
required ovstage 0.2.1.385922 package:

```bash
mkdir -p output/sdk-0.6.4
gh release download ovphysx-0.6.4 --repo NVIDIA-Omniverse/PhysX \
  --pattern ovphysx-linux-x86_64-0.6.4.72118723.tar.gz \
  --dir output/sdk-0.6.4
tar -xzf output/sdk-0.6.4/ovphysx-linux-x86_64-0.6.4.72118723.tar.gz \
  -C output/sdk-0.6.4
python3 ../../../scripts/fetch_ovstage_release.py \
  --platform manylinux_2_35_x86_64 --dest output/ovstage-0.2.1
cmake -S . -B output/build-0.6.4 -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$PWD/output/sdk-0.6.4/ovphysx;$PWD/output/ovstage-0.2.1"
cmake --build output/build-0.6.4 -j4
ctest --test-dir output/build-0.6.4 --output-on-failure
```

The test is expected to **fail with a memory-retention diagnostic** on this
SDK pair. It runs two fresh processes, each performing 50,000 pose writes and
seals. One also imports each sealed ordinal into physics. The assertion
compares allocation growth between iterations 10,000 and 50,000, excluding
early settling. It fails when the import case grows more than 4 MiB beyond the
write-only control. This allowance distinguishes the observed growth; it is
not an SDK memory contract or proof of bounded memory when the test passes.

Logs, raw samples, and `result.json` are under
`output/build-0.6.4/memory-test/`. Allocations are
`mallinfo2().uordblks + mallinfo2().hblkhd`; RSS is recorded separately to avoid
mistaking allocator-held free pages for live allocations. The Python test
returns 1 for retention and 2 for an invalid run (API failure, timeout, or
missing samples). CTest reports either as failure; inspect its message.
API waits have 30-second deadlines, each process has a 300-second timeout,
and the complete CTest case has a 620-second timeout.

Run the full control matrix, including no writes, constant poses, one body,
mass edits, and no warm-up:

```bash
python3 run_matrix.py --binary output/build-0.6.4/control_drain_repro \
  --output output/matrix-0.6.4
```

For the earlier SDK pair, configure a separate build against complete ovphysx
0.6.3 and ovstage 0.2.0.377349.70d78229 installations:

```bash
cmake -S . -B output/build-0.6.3 -DCMAKE_BUILD_TYPE=Release \
  -DREPRO_OVPHYSX_VERSION=0.6.3 -DREPRO_OVSTAGE_VERSION=0.2.0 \
  -DCMAKE_PREFIX_PATH="/path/to/ovphysx-0.6.3;/path/to/ovstage-0.2.0"
cmake --build output/build-0.6.3 -j4
ctest --test-dir output/build-0.6.3 --output-on-failure
```

CMake checks exact major/minor/patch versions. The download identifiers above
pin the full builds. Keep the SDK plugin directories intact. This directory
can also be copied out of the repository and built against preinstalled SDKs;
only the optional dependency-download command uses a repository script.

## Results

Source base: PhysX `6dc53585d7db07346f53d0598a5fd3bfdce47d8d`.
Both SDK pairs were tested with the same repro source and scene. All API calls
completed successfully. Growth is allocated memory from post-attach/warm-up
baseline to end of the loop, in MiB:

| SDK pair | 10,000 imports | 50,000 imports | 50,000 writes without imports |
| --- | ---: | ---: | ---: |
| ovphysx 0.6.3 / ovstage 0.2.0.377349.70d78229 (earlier investigation) | 16.481 | 91.273 | -0.036 |
| ovphysx 0.6.4.72118723 / ovstage 0.2.1.385922 | 3.461 | 16.551 | -0.035 |

The newer pair substantially reduces growth but does not eliminate it.
The first 10,000-update diagnostic passed its 4 MiB allowance despite 2.711 MiB
of additional growth after iteration 2,000. Extending the measurement window,
without lowering the allowance, exposes the slower retention: the final
0.6.4 CTest case fails with 13.088 MiB excess growth from 10,000 to 50,000
(18.59 seconds). The same final test on 0.6.3 fails with 74.732 MiB excess
growth (20.54 seconds). Both controls show 0.000 MiB growth in that window.

In the 0.6.4 matrix's 50,000-update run, allocated memory reaches 152.019 MiB,
remains at 151.188 MiB after physics destruction, and falls to 134.112 MiB
after stage destruction. An explicit ovstage initialization reference remains
live across that checkpoint. This indicates stage-lifetime retention, not a
process-exit leak. No plateau is observed through 50,000 updates; mathematical
unboundedness is not established. Both SDK versions change between the pairs,
so these results do not attribute the improvement to either library alone.

The 0.6.4 no-write control also grows by 3.468 MiB over 10,000 imports. Its
constant-pose and no-warm-up cases grow similarly. The remaining issue is not
specific to alternating translations. Earlier allocation profiling placed
increasing stacks in libovstage beneath the physics update call; allocation
location alone does not identify which component owns the defect.

Binary SHA-256 identities for the current release:

| File | SHA-256 |
| --- | --- |
| libovphysx.so.0.6.4 | b9d15f82842dc192b86fb25d993a079ca94811c656b35b14612dccf4d8834a13 |
| libovstage.so | 99154fa3e3dd497311ecc383c0d2d6ab9d83c342d8d09a148c17ab7b29d82b2c |

## API sequence

1. Initialize CPU-only physics and ovstage. Register the physics schemas before
   populating the four-body scene at ordinal 1; wait and seal it.
2. Attach physics, optionally warm up, and create one query over the bodies.
3. At ordinal `i + 2`, write translations, wait/release the operation, seal the
   ordinal, wait/release, then import the closed range `[ordinal, ordinal]`.
4. Release the query, detach/destroy physics, destroy the stage, and shut down
   both SDKs. Sample memory between teardown boundaries.

The C++ program returns zero when the API sequence succeeds, even if memory
grows. Use the Python/CTest assertion to classify retention. `PROBE_`
environment variables select iterations, body count, writes, imports,
alternating values, masses, and warm-up; the test runners reset them explicitly.
