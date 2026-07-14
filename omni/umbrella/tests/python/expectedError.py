# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary
#

import carb

_preface_printed = False


# --------------------------------------------------------------------------------------------------------------
class ExpectedError:
    """
    Helper class used to prefix any pending error messages with [Expected Error]

    Usage:
        with ExpectedError():
            function_that_produced_error_output()

    """

    def __enter__(self):
        global _preface_printed
        if not _preface_printed:
            carb.log_warn("Test(s) are running that expect errors and/or warnings")
            _preface_printed = True

        # Preflush any output, otherwise it may be appended to the next statement.
        print("", flush=True)

        # Output the prefix string without a newline so that the error to be ignored will appear on
        # the same line.
        print("[Ignore this error/warning] ", end="", flush=False)

    def __exit__(self, exit_type, value, traceback):
        # Print a newline, to avoid actual errors being ignored.
        print("", flush=True)
