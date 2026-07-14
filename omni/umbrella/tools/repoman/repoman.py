# SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: MIT
#

import contextlib
import io
import logging
import os
import sys

import packmanapi
from repoman_bootstrapper import repoman_bootstrap

logger = logging.getLogger(__name__)
REPO_ROOT = os.path.join(os.path.dirname(os.path.normpath(__file__)), "../..")
REPO_DEPS_FILE = os.path.join(REPO_ROOT, "deps/repo-deps.packman.xml")


def pull_deps():
    """
    Bootstrap all omni.repo modules.

    Pull with packman from repo.packman.xml and add them all to python sys.path to enable importing.
    """
    with contextlib.redirect_stdout(io.StringIO()):
        deps = packmanapi.pull(REPO_DEPS_FILE)
    for dep_path in deps.values():
        if dep_path not in sys.path:
            sys.path.append(dep_path)


if __name__ == "__main__":
    repoman_bootstrap()
    pull_deps()

    import omni.repo.man

    omni.repo.man.main(REPO_ROOT)
