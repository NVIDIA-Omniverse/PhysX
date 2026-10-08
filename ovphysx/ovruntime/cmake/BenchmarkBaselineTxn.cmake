# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# Sibling copy of ovphysx's scripts/benchmark_baseline_txn.cmake (same protocol, ovruntime-local
# so the measure-benchmark-change skill can protect ovruntime's own developer baseline without
# reaching across the project boundary). See that file's history/PLC coverage for the design
# rationale; this copy carries no independent @implements annotation.
#
# A crash-recoverable snapshot/restore transaction for one file.
#
# The measure-benchmark-change skill (and, for ovphysx, test_benchmark_contract.cmake) runs the
# benchmark binary with --regenerate, which makes BmOutput's destructor rewrite the shared
# developer baseline at <app dir>/../data/benchmarkData/_baseline.txt with only the rows that
# narrow run selected. Callers that borrow that file for a filtered/temporary run have to put the
# developer's original content back afterward.
#
# Doing that with an in-memory "did it exist?" flag is not enough. If the
# driving cmake process dies after the benchmark child has already replaced the
# baseline, the flag dies with it -- and the next invocation would snapshot the
# narrow leftover on top of the only surviving copy of the developer file.
#
# So the transaction state lives on disk:
#
#   <marker>  one line, either "ABSENT" or "EXISTED <sha256-of-target>"
#   <backup>  byte copy of the target, present only in the EXISTED case
#
# Ordering is what makes it recoverable, not cleanup handlers -- a SIGKILL runs
# nothing:
#
#   begin()   writes and verifies the backup FIRST, then the marker. Dying in
#             between leaves no marker, and nothing destructive has run yet, so
#             the next begin() simply re-snapshots the untouched target.
#   commit()  restores, machine-checks the result against the recorded hash,
#             then removes the marker BEFORE the backup. Dying in between
#             leaves a marker whose backup is still intact, so recovery repeats
#             harmlessly; the reverse order could leave a marker pointing at a
#             backup that no longer exists.
#   recover() is therefore idempotent, and is the only thing allowed to run
#             before begin().
#
# The guarantee is not "cleanup always runs". It is that an interrupted
# transaction stays recoverable and that the next invocation recovers it before
# it does anything destructive.
#
# Callers set these before each call:
#   BASELINE_TXN_TARGET  file to protect
#   BASELINE_TXN_BACKUP  snapshot path
#   BASELINE_TXN_MARKER  marker path
#
# cmake/test_benchmark_baseline_txn.cmake exercises all of this against a
# scratch directory, so the protocol is covered without risking a real
# developer baseline.

cmake_minimum_required(VERSION 3.16)


function(_baseline_txn_require_paths who)
    foreach(_var BASELINE_TXN_TARGET BASELINE_TXN_BACKUP BASELINE_TXN_MARKER)
        if(NOT ${_var})
            message(FATAL_ERROR "${who}: ${_var} is not set")
        endif()
    endforeach()
endfunction()


# Parses the marker into OUT_STATE ("EXISTED"/"ABSENT"/"" when there is no
# marker) and OUT_HASH (the recorded target hash, EXISTED only).
function(_baseline_txn_read_marker OUT_STATE OUT_HASH)
    set(${OUT_STATE} "" PARENT_SCOPE)
    set(${OUT_HASH} "" PARENT_SCOPE)
    if(NOT EXISTS "${BASELINE_TXN_MARKER}")
        return()
    endif()

    file(READ "${BASELINE_TXN_MARKER}" _raw)
    string(STRIP "${_raw}" _raw)
    if(_raw STREQUAL "ABSENT")
        set(${OUT_STATE} "ABSENT" PARENT_SCOPE)
        return()
    endif()
    if(_raw MATCHES "^EXISTED ([0-9a-f]+)$")
        set(${OUT_STATE} "EXISTED" PARENT_SCOPE)
        set(${OUT_HASH} "${CMAKE_MATCH_1}" PARENT_SCOPE)
        return()
    endif()

    message(FATAL_ERROR
        "Unreadable baseline transaction marker '${BASELINE_TXN_MARKER}': '${_raw}'.\n"
        "Refusing to guess. Inspect '${BASELINE_TXN_BACKUP}', put the correct file back at\n"
        "'${BASELINE_TXN_TARGET}' by hand, then delete the marker.")
endfunction()


# Completes an interrupted prior transaction. No-op when there is no marker.
function(baseline_txn_recover)
    _baseline_txn_require_paths("baseline_txn_recover")
    _baseline_txn_read_marker(_state _hash)
    if(NOT _state)
        return()
    endif()

    if(_state STREQUAL "EXISTED")
        if(NOT EXISTS "${BASELINE_TXN_BACKUP}")
            message(FATAL_ERROR
                "Baseline transaction marker claims a snapshot exists, but '${BASELINE_TXN_BACKUP}' is missing.\n"
                "'${BASELINE_TXN_TARGET}' may hold a narrow regenerated baseline. Restore it by hand,\n"
                "then delete '${BASELINE_TXN_MARKER}'.")
        endif()
        configure_file("${BASELINE_TXN_BACKUP}" "${BASELINE_TXN_TARGET}" COPYONLY)
        file(SHA256 "${BASELINE_TXN_TARGET}" _restored)
        if(NOT _restored STREQUAL _hash)
            message(FATAL_ERROR
                "Recovered baseline does not match the hash recorded when it was snapshotted.\n"
                "expected ${_hash}, got ${_restored}. Snapshot: '${BASELINE_TXN_BACKUP}'.")
        endif()
        message(STATUS
            "Recovered an interrupted baseline transaction: restored '${BASELINE_TXN_TARGET}' from its snapshot")
    else()
        file(REMOVE "${BASELINE_TXN_TARGET}")
        if(EXISTS "${BASELINE_TXN_TARGET}")
            message(FATAL_ERROR
                "Could not remove the baseline left by the interrupted run at '${BASELINE_TXN_TARGET}'")
        endif()
        message(STATUS
            "Recovered an interrupted baseline transaction: '${BASELINE_TXN_TARGET}' had not existed, removed it")
    endif()

    file(REMOVE "${BASELINE_TXN_MARKER}")
    file(REMOVE "${BASELINE_TXN_BACKUP}")
endfunction()


# Snapshots the target. Absence is recorded explicitly, so a run that created a
# baseline where none existed can still be undone.
function(baseline_txn_begin)
    _baseline_txn_require_paths("baseline_txn_begin")
    if(EXISTS "${BASELINE_TXN_MARKER}")
        message(FATAL_ERROR
            "An unrecovered baseline transaction is still open at '${BASELINE_TXN_MARKER}'.\n"
            "baseline_txn_recover() must run before any snapshot; overwriting the backup now\n"
            "would destroy the only surviving copy of the developer baseline.")
    endif()

    file(REMOVE "${BASELINE_TXN_BACKUP}")
    if(NOT EXISTS "${BASELINE_TXN_TARGET}")
        file(WRITE "${BASELINE_TXN_MARKER}" "ABSENT\n")
        return()
    endif()

    file(SHA256 "${BASELINE_TXN_TARGET}" _hash)
    configure_file("${BASELINE_TXN_TARGET}" "${BASELINE_TXN_BACKUP}" COPYONLY)
    file(SHA256 "${BASELINE_TXN_BACKUP}" _backupHash)
    if(NOT _backupHash STREQUAL _hash)
        file(REMOVE "${BASELINE_TXN_BACKUP}")
        message(FATAL_ERROR "Baseline snapshot did not reproduce '${BASELINE_TXN_TARGET}'; refusing to continue.")
    endif()
    # Only now is the marker allowed to claim the backup is good.
    file(WRITE "${BASELINE_TXN_MARKER}" "EXISTED ${_hash}\n")
endfunction()


# Puts the target back and proves it byte for byte, then clears all state.
function(baseline_txn_commit)
    _baseline_txn_require_paths("baseline_txn_commit")
    _baseline_txn_read_marker(_state _hash)
    if(NOT _state)
        message(FATAL_ERROR "baseline_txn_commit() called with no open transaction")
    endif()

    if(_state STREQUAL "EXISTED")
        configure_file("${BASELINE_TXN_BACKUP}" "${BASELINE_TXN_TARGET}" COPYONLY)
        file(SHA256 "${BASELINE_TXN_TARGET}" _after)
        if(NOT _after STREQUAL _hash)
            message(FATAL_ERROR
                "Restored baseline is not byte-identical to the snapshot: expected ${_hash}, got ${_after}.\n"
                "The snapshot is still at '${BASELINE_TXN_BACKUP}'.")
        endif()
    else()
        # A narrow baseline left where none existed would make the next full
        # comparison run report 'No baseline' for every row it does not name.
        file(REMOVE "${BASELINE_TXN_TARGET}")
        if(EXISTS "${BASELINE_TXN_TARGET}")
            message(FATAL_ERROR "Could not remove the baseline this run created at '${BASELINE_TXN_TARGET}'")
        endif()
    endif()

    file(REMOVE "${BASELINE_TXN_MARKER}")
    file(REMOVE "${BASELINE_TXN_BACKUP}")
endfunction()
