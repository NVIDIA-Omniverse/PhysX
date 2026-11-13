<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Third-party and separately licensed source notices

This index describes code included in the ovphysx source tree. The original
copyright and license notices in those files remain applicable. NVIDIA's
ovphysx code is covered by [LICENSE.txt](LICENSE.txt), except where a file
declares another license.

## DLPack

- Source: [include/ovphysx/dlpack/dlpack.h](include/ovphysx/dlpack/dlpack.h).
- Upstream: [DLPack v1.3](https://github.com/dmlc/dlpack/tree/v1.3).
- Copyright (c) 2017 - by Contributors, as retained in the upstream header.
- License: Apache License 2.0; full text in
  [tools/internal-licenses/dlpack-LICENSE.txt](tools/internal-licenses/dlpack-LICENSE.txt).

The Python DLPack wrappers are NVIDIA implementations rather than copies of
the upstream C header.

## OpenUSD math routines

- Source: the attributed OpenUSD-derived routines in
  [MatrixTools.h](ovruntime/source/common/include/common/foundation/MatrixTools.h),
  [MatrixTools.cpp](ovruntime/source/common/source/foundation/MatrixTools.cpp), and
  [TransformedExtent.h](ovruntime/source/common/include/common/foundation/TransformedExtent.h).
- Upstream: [OpenUSD v25.11](https://github.com/PixarAnimationStudios/OpenUSD/tree/v25.11).
- Copyright 2016 Pixar.
- License: Tomorrow Open Source Technology License 1.0; full text in
  [tools/internal-licenses/OpenUSD-LICENSE.txt](tools/internal-licenses/OpenUSD-LICENSE.txt).

The source comments describe the original routines and NVIDIA's modifications.
These source transcriptions are present even though libovphysx does not link
an OpenUSD library.

## V-HACD convex-decomposition code

- Source:
  [Vector.h](ovruntime/source/omni.convexdecomposition/plugins/Vector.h),
  [Volume.h](ovruntime/source/omni.convexdecomposition/plugins/Volume.h), and
  [Volume.cpp](ovruntime/source/omni.convexdecomposition/plugins/Volume.cpp).
  Vector.h includes
  [Vector.inl](ovruntime/source/omni.convexdecomposition/plugins/Vector.inl)
  for its template implementation.
- Copyright (c) 2011 Khaled Mamou.
- License: BSD 3-Clause; full text in
  [tools/internal-licenses/vhacd-LICENSE.txt](tools/internal-licenses/vhacd-LICENSE.txt).

The original notices are retained in the source files. Volume.cpp also
retains the attribution to Tomas Akenine-Moller for its AABB-triangle overlap
test.

## Repoman bootstrap tooling

- Source: [repo.sh](repo.sh), [repo.bat](repo.bat),
  [tools/repoman/repoman.py](tools/repoman/repoman.py), and
  [tools/repoman/repoman_bootstrapper.py](tools/repoman/repoman_bootstrapper.py).
- Copyright NVIDIA CORPORATION, with the original years retained in each
  source file and the upstream license.
- License: MIT; full text in
  [tools/internal-licenses/repoman-LICENSE.txt](tools/internal-licenses/repoman-LICENSE.txt).

The license text is preserved from the repo_man 2.9.2 distribution; the
repo_man 2.10.1 distribution carries the same text. These bootstrap files
retain their upstream MIT identifiers and notices. They are managed by the
Repoman update system; local compliance changes belong in this index and the
separate full-license copy, not in the update-managed files.

## Packman bootstrap tooling

- Source: [tools/packman/](tools/packman/) and
  [ovruntime/tools/packman/](ovruntime/tools/packman/).
- Copyright NVIDIA CORPORATION, with the original years retained in each
  source file.
- License: Apache License 2.0; full text in [LICENSE.txt](LICENSE.txt).

These files are managed by the Packman update system and retain their upstream
headers. This notice records their license without modifying those files;
upstream header-format changes must come through the Packman update.

## Binary-distribution license material

The following notices describe pre-built products and dependencies:

- [licenses/LICENSE-binary.txt](licenses/LICENSE-binary.txt): NVIDIA Omniverse
  terms for ovphysx SDK packages and Python wheels.
- [tools/internal-licenses/Omniverse-Extensions-LICENSE.txt](tools/internal-licenses/Omniverse-Extensions-LICENSE.txt):
  NVIDIA Omniverse terms for the Omniverse Extensions dependency.

Pre-built distributions provide their bundled dependency licenses in
ovphysx-LICENSES.zip and applicable dependency-specific notice files. This
source inventory does not replace those binary-distribution notices.
