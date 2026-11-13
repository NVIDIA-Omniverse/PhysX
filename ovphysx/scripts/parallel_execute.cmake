# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# Bounded parallel execution for cmake -P test drivers.
# Each job is a self-contained shell/batch script. The runner starts at most
# MAX_JOBS at a time and waits for completion before launching more.
# Per-job wall time defaults to 120s (override: -DOVPHYSX_PARALLEL_JOB_TIMEOUT=N).
#
# The runner reports every job start and end on its own stdout, so a phase that
# has stopped making progress is visible in the driver log, and a job that
# overruns its budget is ended as a whole process tree: on Windows the job's
# process tree is listed, each python.exe in it gets a minidump next to the job
# log, then taskkill /T /F ends the tree; on Linux `timeout` sends TERM to the
# job's process group and the reaper KILLs that group afterwards. Ending only
# the top process (the earlier Process.Kill() of the cmd.exe wrapper) left
# `cmake -E env` -> `pytest.exe` -> `python.exe` alive with inherited copies of
# the pipes CMake's execute_process waits on, so the phase never returned:
# GitHub job 118663756922 (2026-10-20) wrote 27 of its 28 lifecycle junit files
# by 14:05:39 (`test_multi_instance_output` most likely hung inside the test)
# and then stayed silent until the 300 min job timeout. With this runner that
# job would have ended at the 120 s budget with a minidump of the stuck test.

include_guard(GLOBAL)

function(_ovphysx_shell_single_quote OUT_VAR VALUE)
    string(REPLACE "'" "'\\''" _escaped "${VALUE}")
    set(${OUT_VAR} "'${_escaped}'" PARENT_SCOPE)
endfunction()

function(_ovphysx_cmd_double_quote OUT_VAR VALUE)
    string(REPLACE "\"" "\"\"" _escaped "${VALUE}")
    set(${OUT_VAR} "\"${_escaped}\"" PARENT_SCOPE)
endfunction()

function(_ovphysx_join_shell_args OUT_VAR)
    set(_parts "")
    foreach(_arg IN LISTS ARGN)
        _ovphysx_shell_single_quote(_quoted "${_arg}")
        list(APPEND _parts "${_quoted}")
    endforeach()
    string(JOIN " " _joined ${_parts})
    set(${OUT_VAR} "${_joined}" PARENT_SCOPE)
endfunction()

# A PowerShell single-quoted literal: the only character that needs care is the quote itself.
function(_ovphysx_ps_single_quote OUT_VAR VALUE)
    string(REPLACE "'" "''" _escaped "${VALUE}")
    set(${OUT_VAR} "'${_escaped}'" PARENT_SCOPE)
endfunction()

function(_ovphysx_join_cmd_args OUT_VAR)
    set(_parts "")
    foreach(_arg IN LISTS ARGN)
        _ovphysx_cmd_double_quote(_quoted "${_arg}")
        list(APPEND _parts "${_quoted}")
    endforeach()
    string(JOIN " " _joined ${_parts})
    set(${OUT_VAR} "${_joined}" PARENT_SCOPE)
endfunction()

# Write a job script that runs one COMMAND (argv list) in WORKING_DIR.
function(ovphysx_write_job_script SCRIPT_PATH WORKING_DIR)
    if(WIN32)
        _ovphysx_cmd_double_quote(_work_dir_quoted "${WORKING_DIR}")
        _ovphysx_join_cmd_args(_cmd_line ${ARGN})
        file(WRITE "${SCRIPT_PATH}" "@echo off\r\n")
        file(APPEND "${SCRIPT_PATH}" "cd /d ${_work_dir_quoted}\r\n")
        file(APPEND "${SCRIPT_PATH}" "${_cmd_line}\r\n")
        file(APPEND "${SCRIPT_PATH}" "if errorlevel 1 exit /b 1\r\n")
        file(APPEND "${SCRIPT_PATH}" "exit /b 0\r\n")
    else()
        _ovphysx_shell_single_quote(_work_dir_quoted "${WORKING_DIR}")
        _ovphysx_join_shell_args(_cmd_line ${ARGN})
        file(WRITE "${SCRIPT_PATH}" "#!/usr/bin/env bash\n")
        file(APPEND "${SCRIPT_PATH}" "set -euo pipefail\n")
        file(APPEND "${SCRIPT_PATH}" "cd ${_work_dir_quoted}\n")
        file(APPEND "${SCRIPT_PATH}" "exec ${_cmd_line}\n")
    endif()
endfunction()

# The Windows runner body. A bracket argument, so CMake leaves PowerShell's
# `${name}` alone; the paths spliced in above it are single-quoted with any `'`
# doubled. Every job is tracked through a process handle pinned right after the
# start ($proc.Handle): a finished job's PID can be reused within the 100 ms
# poll by the next job's processes, and without a cached handle .NET cannot
# read ExitCode at all once the process is gone, which is how the previous
# runner flagged all 28 jobs of a round as failed (probe run 39480901381).
set(_OVPHYSX_PARALLEL_PS1_BODY [==[
$ErrorActionPreference = 'Stop'
$failed = @()
$queue = [System.Collections.Queue]::new()
foreach ($s in $scripts) { [void]$queue.Enqueue($s) }
$running = @()
$clock = [System.Diagnostics.Stopwatch]::StartNew()

function Write-JobLine([string]$text) {
  $line = ('[jobs {0,7:N1}s] {1}' -f $clock.Elapsed.TotalSeconds, $text)
  [Console]::Out.WriteLine($line)
  [Console]::Out.Flush()
}

function Get-JobName([string]$path) { return [System.IO.Path]::GetFileNameWithoutExtension($path) }

function Get-ProcessTree([int]$rootPid) {
  $all = @(Get-CimInstance Win32_Process -OperationTimeoutSec 30 | Select-Object ProcessId, ParentProcessId, Name, CommandLine, CreationDate)
  $byPid = @{}
  foreach ($p in $all) { $byPid[[int]$p.ProcessId] = $p }
  $found = @()
  $stack = [System.Collections.Stack]::new()
  $stack.Push($rootPid)
  while ($stack.Count -gt 0) {
    $parent = $stack.Pop()
    $parentStart = $null
    if ($byPid.ContainsKey($parent)) { $parentStart = $byPid[$parent].CreationDate }
    foreach ($candidate in $all) {
      if ($candidate.ParentProcessId -ne $parent -or $candidate.ProcessId -eq $parent) { continue }
      # Windows reuses PIDs: an older process whose real parent had this PID still names it as parent.
      # A real child is never older than its parent.
      if ($parentStart -and $candidate.CreationDate -and $candidate.CreationDate -lt $parentStart) { continue }
      $found += $candidate
      $stack.Push([int]$candidate.ProcessId)
    }
  }
  return $found
}

# Compiled and run inside a background job (its own powershell.exe), so a dump that blocks on a
# process stuck in a driver call cannot hold the runner: the job gets 60 s in total.
$minidumpSource = @'
using System;
using System.IO;
using System.Runtime.InteropServices;
public static class OvphysxMiniDump {
  [DllImport("dbghelp.dll", SetLastError = true)]
  static extern bool MiniDumpWriteDump(IntPtr process, uint pid, IntPtr file, int type, IntPtr exception, IntPtr user, IntPtr callback);
  [DllImport("kernel32.dll", SetLastError = true)]
  static extern IntPtr OpenProcess(uint access, bool inherit, uint pid);
  [DllImport("kernel32.dll", SetLastError = true)]
  static extern bool CloseHandle(IntPtr handle);
  // PROCESS_QUERY_INFORMATION | PROCESS_VM_READ | PROCESS_DUP_HANDLE
  const uint Access = 0x0400 | 0x0010 | 0x0040;
  // MiniDumpWithHandleData | WithUnloadedModules | WithThreadInfo: stacks and module lists for every
  // thread, no heap. Not WithProcessThreadData: it copies the environment block, and with it the CI
  // job's credentials, into a dump that is uploaded as an artifact.
  const int Type = 0x0004 | 0x0020 | 0x1000;
  public static string Write(uint pid, string path) {
    IntPtr process = OpenProcess(Access, false, pid);
    if (process == IntPtr.Zero) return "OpenProcess failed: " + Marshal.GetLastWin32Error();
    try {
      using (FileStream stream = new FileStream(path, FileMode.Create, FileAccess.ReadWrite, FileShare.None)) {
        bool ok = MiniDumpWriteDump(process, pid, stream.SafeFileHandle.DangerousGetHandle(), Type, IntPtr.Zero, IntPtr.Zero, IntPtr.Zero);
        return ok ? ("written, " + stream.Length + " bytes") : ("MiniDumpWriteDump failed: " + Marshal.GetLastWin32Error());
      }
    } finally { CloseHandle(process); }
  }
}
'@

function Stop-JobTree($entry, [double]$elapsedSeconds) {
  # Native stderr under 'Stop' becomes a terminating error in Windows PowerShell; diagnostics must not.
  $ErrorActionPreference = 'Continue'
  $name = Get-JobName $entry.Script
  Write-JobLine ('TIMEOUT {0} after {1:N0}s (budget {2}s); process tree of pid {3}:' -f $name, $elapsedSeconds, ($jobTimeoutMs / 1000), $entry.Process.Id)
  try {
    # Diagnostics first, each with its own time limit (CIM 30 s, dumps 60 s in total); the kill in
    # finally runs whatever they do.
    $tree = @()
    try { $tree = @(Get-ProcessTree $entry.Process.Id) } catch { Write-JobLine ('  process tree unavailable: {0}' -f $_) }
    foreach ($p in $tree) {
      Write-JobLine ('  pid {0} ppid {1} {2} started {3}: {4}' -f $p.ProcessId, $p.ParentProcessId, $p.Name, $p.CreationDate, $p.CommandLine)
    }
    $targets = @()
    foreach ($p in $tree) {
      if ($p.Name -match '^python') { $targets += @{ Pid = [uint32]$p.ProcessId; Path = ('{0}.{1}.dmp' -f $entry.Script, $p.ProcessId) } }
    }
    if ($targets.Count -gt 0) {
      $dumpJob = $null
      try {
        $dumpJob = Start-Job -ScriptBlock {
          param($source, $targets)
          Add-Type -TypeDefinition $source
          foreach ($t in $targets) {
            try { $note = [OvphysxMiniDump]::Write([uint32]$t.Pid, [string]$t.Path) } catch { $note = 'failed: ' + $_ }
            '{0}: {1}' -f $t.Path, $note
          }
        } -ArgumentList $minidumpSource, $targets
        if (Wait-Job $dumpJob -Timeout 60) {
          Receive-Job $dumpJob | ForEach-Object { Write-JobLine ('  minidump {0}' -f $_) }
        } else {
          Write-JobLine '  minidump: gave up after 60 s'
          Stop-Job $dumpJob
        }
      } catch { Write-JobLine ('  minidump job failed: {0}' -f $_) }
      finally { if ($dumpJob) { Remove-Job $dumpJob -Force -ErrorAction SilentlyContinue } }
    }
  } finally {
    try {
      $output = (& taskkill.exe /T /F /PID $entry.Process.Id 2>&1 | Out-String).Trim()
      Write-JobLine ('  taskkill /T /F: {0}' -f ($output -replace '\s+', ' '))
    } catch { Write-JobLine ('  taskkill failed: {0}' -f $_) }
  }
}

Write-JobLine ('starting {0} job(s), {1} at a time, {2}s each' -f $scripts.Count, $maxJobs, ($jobTimeoutMs / 1000))
while ($queue.Count -gt 0 -or $running.Count -gt 0) {
  while ($running.Count -lt $maxJobs -and $queue.Count -gt 0) {
    $scriptPath = [string]$queue.Dequeue()
    $logPath = "${scriptPath}.log"
    $errPath = "${scriptPath}.err.log"
    $proc = Start-Process -FilePath 'cmd.exe' -ArgumentList @('/d', '/c', $scriptPath) `
      -RedirectStandardOutput $logPath -RedirectStandardError $errPath `
      -PassThru -NoNewWindow -Wait:$false
    # Pin the process handle so later HasExited/ExitCode reads follow this process, not its PID.
    try { $null = $proc.Handle } catch { }
    Write-JobLine ('start {0} (pid {1})' -f (Get-JobName $scriptPath), $proc.Id)
    $running += [PSCustomObject]@{ Script = $scriptPath; Process = $proc; Started = $clock.Elapsed }
  }
  $stillRunning = @()
  foreach ($entry in $running) {
    $elapsed = ($clock.Elapsed - $entry.Started).TotalSeconds
    $exited = $false
    try { $exited = $entry.Process.HasExited } catch { $exited = $true }
    if ($exited) {
      $code = -1
      try { $null = $entry.Process.WaitForExit(5000); $code = $entry.Process.ExitCode } catch { }
      Write-JobLine ('done {0} rc={1} in {2:N1}s' -f (Get-JobName $entry.Script), $code, $elapsed)
      if ($code -ne 0) { $failed += $entry.Script }
    } elseif (($elapsed * 1000) -ge $jobTimeoutMs) {
      try { Stop-JobTree $entry $elapsed } catch { Write-JobLine ('  stop failed: {0}' -f $_) }
      $failed += $entry.Script
    } else {
      $stillRunning += $entry
    }
  }
  $running = $stillRunning
  if ($running.Count -gt 0) { Start-Sleep -Milliseconds 100 }
}
Write-JobLine ('finished: {0} failed of {1}' -f $failed.Count, $scripts.Count)
if ($failed.Count -gt 0) {
  # Always an array (a one-element array piped into ConvertTo-Json would arrive as a bare string), UTF-8 without BOM.
  [System.IO.File]::WriteAllText($failedJson, (ConvertTo-Json -InputObject @($failed) -Compress), (New-Object System.Text.UTF8Encoding($false)))
  exit 1
}
exit 0
]==])

# Run JOB_SCRIPT_PATHS with at most MAX_JOBS concurrent processes.
function(ovphysx_run_job_scripts_parallel MAX_JOBS JOB_SCRIPT_PATHS
         OUT_FAILED_SCRIPT_PATHS OUT_COMBINED_OUTPUT)
    list(LENGTH JOB_SCRIPT_PATHS _job_count)
    if(_job_count EQUAL 0)
        set(${OUT_FAILED_SCRIPT_PATHS} "" PARENT_SCOPE)
        set(${OUT_COMBINED_OUTPUT} "" PARENT_SCOPE)
        return()
    endif()

    if(MAX_JOBS LESS 1)
        set(MAX_JOBS 1)
    endif()

    set(_job_timeout_sec 120)
    if(DEFINED OVPHYSX_PARALLEL_JOB_TIMEOUT AND NOT "${OVPHYSX_PARALLEL_JOB_TIMEOUT}" STREQUAL "")
        set(_job_timeout_sec "${OVPHYSX_PARALLEL_JOB_TIMEOUT}")
    endif()
    if(_job_timeout_sec LESS 1)
        set(_job_timeout_sec 1)
    endif()
    math(EXPR _job_timeout_ms "${_job_timeout_sec} * 1000")

    list(GET JOB_SCRIPT_PATHS 0 _first_script)
    get_filename_component(_runner_dir "${_first_script}" DIRECTORY)

    if(WIN32)
        set(_runner "${_runner_dir}/_ovphysx_parallel_jobs.ps1")
        set(_failed_json "${_runner_dir}/_ovphysx_parallel_failed.json")
        file(REMOVE "${_failed_json}")
        set(_lines "$maxJobs = ${MAX_JOBS}\n")
        string(APPEND _lines "$jobTimeoutMs = ${_job_timeout_ms}\n")
        _ovphysx_ps_single_quote(_failed_json_quoted "${_failed_json}")
        string(APPEND _lines "$failedJson = ${_failed_json_quoted}\n")
        string(APPEND _lines "$scripts = @(\n")
        set(_script_idx 0)
        foreach(_script IN LISTS JOB_SCRIPT_PATHS)
            math(EXPR _script_idx "${_script_idx} + 1")
            _ovphysx_ps_single_quote(_script_quoted "${_script}")
            if(_script_idx EQUAL _job_count)
                string(APPEND _lines "  ${_script_quoted}\n")
            else()
                string(APPEND _lines "  ${_script_quoted},\n")
            endif()
        endforeach()
        string(APPEND _lines ")\n")
        string(APPEND _lines "${_OVPHYSX_PARALLEL_PS1_BODY}")
        file(WRITE "${_runner}" "${_lines}")
        execute_process(
            COMMAND powershell.exe -NoProfile -ExecutionPolicy Bypass -File "${_runner}"
            RESULT_VARIABLE _runner_result
        )
    else()
        set(_runner "${_runner_dir}/_ovphysx_parallel_jobs.sh")
        set(_failed_list "${_runner_dir}/_ovphysx_parallel_failed.list")
        file(WRITE "${_runner}" "#!/usr/bin/env bash\n")
        file(APPEND "${_runner}" "set -euo pipefail\n")
        file(APPEND "${_runner}" "MAX_JOBS=${MAX_JOBS}\n")
        file(APPEND "${_runner}" "JOB_TIMEOUT_SEC=${_job_timeout_sec}\n")
        file(APPEND "${_runner}" "FAILED_LIST='${_failed_list}'\n")
        file(APPEND "${_runner}" ": > \"\${FAILED_LIST}\"\n")
        file(APPEND "${_runner}" "START_TS=\$(date +%s)\n")
        file(APPEND "${_runner}" "job_line() { printf '[jobs %5ss] %s\\n' \"\$(( \$(date +%s) - START_TS ))\" \"\$*\"; }\n")
        file(APPEND "${_runner}" "running_pids=()\n")
        file(APPEND "${_runner}" "running_scripts=()\n")
        file(APPEND "${_runner}" "running_starts=()\n")
        file(APPEND "${_runner}" "reap_finished_jobs() {\n")
        file(APPEND "${_runner}" "  local still_pids=()\n")
        file(APPEND "${_runner}" "  local still_scripts=()\n")
        file(APPEND "${_runner}" "  local still_starts=()\n")
        file(APPEND "${_runner}" "  local idx\n")
        file(APPEND "${_runner}" "  for idx in \"\${!running_pids[@]}\"; do\n")
        file(APPEND "${_runner}" "    local pid=\"\${running_pids[\${idx}]}\"\n")
        file(APPEND "${_runner}" "    local script=\"\${running_scripts[\${idx}]}\"\n")
        file(APPEND "${_runner}" "    local started=\"\${running_starts[\${idx}]}\"\n")
        file(APPEND "${_runner}" "    if kill -0 \"\${pid}\" 2>/dev/null; then\n")
        file(APPEND "${_runner}" "      still_pids+=(\"\${pid}\")\n")
        file(APPEND "${_runner}" "      still_scripts+=(\"\${script}\")\n")
        file(APPEND "${_runner}" "      still_starts+=(\"\${started}\")\n")
        file(APPEND "${_runner}" "    else\n")
        file(APPEND "${_runner}" "      local rc=0\n")
        file(APPEND "${_runner}" "      wait \"\${pid}\" || rc=\$?\n")
        file(APPEND "${_runner}" "      local elapsed=\$(( \$(date +%s) - started ))\n")
        # A budget overrun is a 124 (timeout's own code) or a 137 (its KILL) at or past the budget; a
        # 137 earlier than that is the OOM killer or someone else's kill, and is reported as such.
        file(APPEND "${_runner}" "      if { [ \"\${rc}\" -eq 124 ] || [ \"\${rc}\" -eq 137 ]; } && [ \"\${elapsed}\" -ge \"\${JOB_TIMEOUT_SEC}\" ]; then\n")
        file(APPEND "${_runner}" "        job_line \"TIMEOUT \$(basename \"\${script}\" .sh) after \${elapsed}s (budget \${JOB_TIMEOUT_SEC}s), rc=\${rc}; killing its process group\"\n")
        # timeout leads its own process group (the job's pid). Its TERM reached the group, but a
        # `cmake -E env` from CMake 4.4 exits on TERM at once, so timeout returns before its -k
        # fires and the test underneath can survive. The group KILL ends whatever ignored TERM.
        file(APPEND "${_runner}" "        kill -KILL -- \"-\${pid}\" 2>/dev/null || true\n")
        file(APPEND "${_runner}" "      else\n")
        file(APPEND "${_runner}" "        job_line \"done \$(basename \"\${script}\" .sh) rc=\${rc} in \${elapsed}s\"\n")
        file(APPEND "${_runner}" "      fi\n")
        file(APPEND "${_runner}" "      if [ \"\${rc}\" -ne 0 ]; then\n")
        file(APPEND "${_runner}" "        echo \"\${script}\" >> \"\${FAILED_LIST}\"\n")
        file(APPEND "${_runner}" "      fi\n")
        file(APPEND "${_runner}" "    fi\n")
        file(APPEND "${_runner}" "  done\n")
        file(APPEND "${_runner}" "  running_pids=(\${still_pids[@]+\"\${still_pids[@]}\"})\n")
        file(APPEND "${_runner}" "  running_scripts=(\${still_scripts[@]+\"\${still_scripts[@]}\"})\n")
        file(APPEND "${_runner}" "  running_starts=(\${still_starts[@]+\"\${still_starts[@]}\"})\n")
        file(APPEND "${_runner}" "}\n")
        # Poll rather than `wait -n`: bash 3.2 (macOS) has no `wait -n`, so the previous loop
        # already fell back to polling there; one path for every bash, and each job's status
        # is read exactly once, by the reaper's `wait "$pid"`.
        file(APPEND "${_runner}" "wait_for_slot() {\n")
        file(APPEND "${_runner}" "  while [ \"\${#running_pids[@]}\" -ge \"\${MAX_JOBS}\" ]; do\n")
        file(APPEND "${_runner}" "    sleep 0.05\n")
        file(APPEND "${_runner}" "    reap_finished_jobs\n")
        file(APPEND "${_runner}" "  done\n")
        file(APPEND "${_runner}" "}\n")
        file(APPEND "${_runner}" "job_line \"starting ${_job_count} job(s), \${MAX_JOBS} at a time, \${JOB_TIMEOUT_SEC}s each\"\n")
        foreach(_script IN LISTS JOB_SCRIPT_PATHS)
            _ovphysx_shell_single_quote(_script_quoted "${_script}")
            get_filename_component(_script_name "${_script}" NAME_WE)
            file(APPEND "${_runner}" "wait_for_slot\n")
            # timeout sends TERM to the job's process group at the budget and KILL 10 s later to its
            # direct child; the reaper above KILLs the whole group once the job is reaped.
            file(APPEND "${_runner}" "timeout -k 10 \"\${JOB_TIMEOUT_SEC}\" bash ${_script_quoted} >${_script_quoted}.log 2>${_script_quoted}.err.log &\n")
            file(APPEND "${_runner}" "running_pids+=(\$!)\n")
            file(APPEND "${_runner}" "running_scripts+=(${_script_quoted})\n")
            file(APPEND "${_runner}" "running_starts+=(\$(date +%s))\n")
            file(APPEND "${_runner}" "job_line \"start ${_script_name} (pid \$!)\"\n")
        endforeach()
        file(APPEND "${_runner}" "while [ \"\${#running_pids[@]}\" -gt 0 ]; do\n")
        file(APPEND "${_runner}" "  sleep 0.05\n")
        file(APPEND "${_runner}" "  reap_finished_jobs\n")
        file(APPEND "${_runner}" "done\n")
        file(APPEND "${_runner}" "job_line \"finished: \$(grep -c . \"\${FAILED_LIST}\" || true) failed of ${_job_count}\"\n")
        file(APPEND "${_runner}" "if [ -s \"\${FAILED_LIST}\" ]; then\n")
        file(APPEND "${_runner}" "  exit 1\n")
        file(APPEND "${_runner}" "fi\n")
        file(APPEND "${_runner}" "exit 0\n")
        execute_process(
            COMMAND bash "${_runner}"
            RESULT_VARIABLE _runner_result
        )
    endif()

    set(_failed "")
    if(WIN32)
        if(EXISTS "${_failed_json}")
            file(READ "${_failed_json}" _failed_json_content)
            # Element by element: string(JSON ... GET) without an index returns the whole array as one
            # string, which is how the previous runner's failed list never matched a single job.
            string(JSON _failed_count ERROR_VARIABLE _json_err LENGTH "${_failed_json_content}")
            if(_json_err)
                set(_failed "${JOB_SCRIPT_PATHS}")
            elseif(_failed_count GREATER 0)
                math(EXPR _failed_last "${_failed_count} - 1")
                foreach(_index RANGE 0 ${_failed_last})
                    string(JSON _failed_item GET "${_failed_json_content}" ${_index})
                    list(APPEND _failed "${_failed_item}")
                endforeach()
            endif()
        endif()
    else()
        if(EXISTS "${_failed_list}")
            file(READ "${_failed_list}" _failed_list_content)
            string(STRIP "${_failed_list_content}" _failed_list_content)
            string(REPLACE "\n" ";" _failed "${_failed_list_content}")
        endif()
    endif()

    if(NOT _runner_result EQUAL 0 AND _failed STREQUAL "")
        set(_failed "${JOB_SCRIPT_PATHS}")
    endif()

    set(_combined "")
    foreach(_script IN LISTS JOB_SCRIPT_PATHS)
        set(_log "${_script}.log")
        set(_err "${_script}.err.log")
        if(EXISTS "${_log}")
            file(READ "${_log}" _log_content)
            string(APPEND _combined "${_log_content}")
        endif()
        if(EXISTS "${_err}")
            file(READ "${_err}" _err_content)
            string(APPEND _combined "${_err_content}")
        endif()
    endforeach()

    set(${OUT_FAILED_SCRIPT_PATHS} "${_failed}" PARENT_SCOPE)
    set(${OUT_COMBINED_OUTPUT} "${_combined}" PARENT_SCOPE)
endfunction()

function(ovphysx_compute_lifecycle_test_jobs OUT_VAR)
    if(DEFINED OVPHYSX_LIFECYCLE_JOBS AND NOT "${OVPHYSX_LIFECYCLE_JOBS}" STREQUAL "")
        set(${OUT_VAR} "${OVPHYSX_LIFECYCLE_JOBS}" PARENT_SCOPE)
        return()
    endif()
    ovphysx_compute_build_jobs(_build_jobs)
    if(_build_jobs GREATER 4)
        set(_jobs 4)
    else()
        set(_jobs ${_build_jobs})
    endif()
    if(_jobs LESS 1)
        set(_jobs 1)
    endif()
    set(${OUT_VAR} ${_jobs} PARENT_SCOPE)
endfunction()
