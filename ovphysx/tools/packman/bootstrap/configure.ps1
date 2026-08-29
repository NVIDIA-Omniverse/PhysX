<#
Copyright 2019-2026 NVIDIA CORPORATION

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
#>

[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$PackmanVersion,
    [Parameter(Mandatory = $true)]
    [string]$PythonVersion,
    [string]$PackmanCommonSha256 = ""
)

Set-StrictMode -Version 2.0
$ErrorActionPreference = "Stop"

$PM_PACKMAN_VERSION = $PackmanVersion
$PM_PYTHON_VERSION = $PythonVersion
$PM_PACKMAN_COMMON_SHA256 = $PackmanCommonSha256
$BOOTSTRAP_ROOT = $PSScriptRoot
$PM_INSTALL_PATH = [System.IO.Path]::GetFullPath((Join-Path $BOOTSTRAP_ROOT ".."))
$SENTINEL_FILE_NAME = ".packman.rdy"
$LOCK_WAIT_LOG_INTERVAL_MS = 5000

Add-Type -AssemblyName System.IO.Compression
Add-Type -AssemblyName System.IO.Compression.FileSystem

[System.Net.ServicePointManager]::SecurityProtocol = `
    [System.Net.ServicePointManager]::SecurityProtocol -bor `
    [System.Net.SecurityProtocolType]::Tls12

function Write-Log {
    param([string]$Message)
    [Console]::Error.WriteLine($Message)
}

function Write-Failure {
    param([string]$Message)
    [Console]::Error.WriteLine($Message)
}

function Get-DefaultPackagesRoot {
    $drive = [System.IO.Path]::GetPathRoot($PM_INSTALL_PATH)
    if ([string]::IsNullOrEmpty($drive)) {
        throw "Unable to resolve install drive from '$PM_INSTALL_PATH'."
    }

    return [System.IO.Path]::Combine($drive, "packman-repo")
}

function Set-UserEnvironmentVariable {
    param(
        [string]$Name,
        [string]$Value
    )

    Write-Log "Setting user environment variable $Name to $Value"
    $null = & setx $Name $Value
    if ($LASTEXITCODE -ne 0) {
        throw "Failed to persist user environment variable $Name."
    }
}

function Write-VisualStudioWarning {
    if ($env:PM_DISABLE_VS_WARNING) {
        return
    }

    if (-not $env:VSLANG) {
        return
    }

    Write-Log "The above is a once-per-computer operation. Unfortunately VisualStudio cannot pick up environment change"
    Write-Log "unless *VisualStudio is RELAUNCHED*."
    Write-Log "If you are launching VisualStudio from command line or command line utility make sure"
    Write-Log "you have a fresh launch environment (relaunch the command line or utility)."
    Write-Log "If you are using 'linkPath' and referring to packages via local folder links you can safely ignore this warning."
    Write-Log "You can disable this warning by setting the environment variable PM_DISABLE_VS_WARNING."
    Write-Log ""
}

function Ensure-DirectoryExists {
    param([string]$Path)

    if (-not (Test-Path -LiteralPath $Path)) {
        New-Item -ItemType Directory -Path $Path -Force | Out-Null
    }
}

function Resolve-NormalizedPath {
    param([string]$Path)

    return [System.IO.Path]::GetFullPath($Path)
}

function Remove-DirectoryIfPresent {
    param([string]$Path)

    if (Test-Path -LiteralPath $Path) {
        Remove-Item -LiteralPath $Path -Recurse -Force
    }
}

function Get-SentinelPath {
    param([string]$DestinationDir)

    return [System.IO.Path]::Combine($DestinationDir, $SENTINEL_FILE_NAME)
}

function Test-Ready {
    param([string]$DestinationDir)

    return Test-Path -LiteralPath (Get-SentinelPath -DestinationDir $DestinationDir)
}

function Get-PathHash {
    param([string]$Path)

    $normalizedPath = [System.IO.Path]::GetFullPath($Path).ToLowerInvariant()
    $sha256 = [System.Security.Cryptography.SHA256]::Create()
    try {
        $bytes = [System.Text.Encoding]::UTF8.GetBytes($normalizedPath)
        $hash = $sha256.ComputeHash($bytes)
    }
    finally {
        $sha256.Dispose()
    }

    return ([System.BitConverter]::ToString($hash).Replace("-", "")).ToLowerInvariant()
}

function Get-BytesSha256Hex {
    param([byte[]]$Bytes)

    $sha256 = [System.Security.Cryptography.SHA256]::Create()
    try {
        $hash = $sha256.ComputeHash($Bytes)
    }
    finally {
        $sha256.Dispose()
    }

    return ([System.BitConverter]::ToString($hash).Replace("-", "")).ToLowerInvariant()
}

function Get-LockFilePath {
    param(
        [string]$PackagesRoot,
        [string]$DestinationDir
    )

    $lockRoot = Join-Path $PackagesRoot ".locks"
    Ensure-DirectoryExists -Path $lockRoot
    return Join-Path $lockRoot ((Get-PathHash -Path $DestinationDir) + ".txt")
}

function Write-LockMetadata {
    param(
        [System.IO.FileStream]$LockStream,
        [string]$DestinationDir
    )

    $metadata = @(
        "target=" + [System.IO.Path]::GetFullPath($DestinationDir)
        "pid=" + $PID
        "machine=" + $env:COMPUTERNAME
        "acquired_utc=" + [DateTime]::UtcNow.ToString("o")
    ) -join [Environment]::NewLine

    $LockStream.SetLength(0)
    $LockStream.Position = 0
    $writer = New-Object System.IO.StreamWriter($LockStream, [System.Text.Encoding]::UTF8, 1024, $true)
    try {
        $writer.WriteLine($metadata)
        $writer.Flush()
        $LockStream.Flush()
    }
    finally {
        $writer.Dispose()
    }
}

function Invoke-WithDestinationFileLock {
    param(
        [string]$PackagesRoot,
        [string]$DestinationDir,
        [string]$Label,
        [scriptblock]$Body
    )

    $lockPath = Get-LockFilePath -PackagesRoot $PackagesRoot -DestinationDir $DestinationDir
    $lockStream = $null

    try {
        while ($null -eq $lockStream) {
            try {
                $lockStream = [System.IO.File]::Open(
                    $lockPath,
                    [System.IO.FileMode]::OpenOrCreate,
                    [System.IO.FileAccess]::ReadWrite,
                    [System.IO.FileShare]::None
                )
                Write-LockMetadata -LockStream $lockStream -DestinationDir $DestinationDir
            }
            catch [System.IO.IOException] {
                if ($null -ne $lockStream) {
                    $lockStream.Dispose()
                    $lockStream = $null
                }

                Write-Log "Waiting for $Label install lock at $DestinationDir ..."
                Start-Sleep -Milliseconds $LOCK_WAIT_LOG_INTERVAL_MS
            }
        }

        & $Body
    }
    finally {
        if ($null -ne $lockStream) {
            $lockStream.Dispose()
        }
    }
}

function Invoke-DownloadBytes {
    param(
        [string]$PackageName,
        [string]$DestinationLabel
    )

    $triesLeft = 4
    $delaySeconds = 2
    $sourceUrl = "https://bootstrap.packman.nvidia.com/$PackageName"

    while ($triesLeft -gt 0) {
        $triesLeft -= 1
        try {
            Write-Log "Fetching $PackageName for $DestinationLabel ..."
            $webClient = New-Object System.Net.WebClient
            try {
                return $webClient.DownloadData($sourceUrl)
            }
            finally {
                $webClient.Dispose()
            }
        }
        catch {
            Write-Log "Error downloading $sourceUrl"
            Write-Log $_.Exception.ToString()
            if ($triesLeft -le 0) {
                break
            }

            Write-Log "Retrying in $delaySeconds seconds ..."
            Start-Sleep -Seconds $delaySeconds
            $delaySeconds *= $delaySeconds
        }
    }

    throw "Failed to download $PackageName."
}

function Expand-ZipBytes {
    param(
        [byte[]]$ZipBytes,
        [string]$DestinationDir
    )

    $destinationRoot = [System.IO.Path]::GetFullPath($DestinationDir)
    $destinationRootWithSeparator = $destinationRoot.TrimEnd([System.IO.Path]::DirectorySeparatorChar) + [System.IO.Path]::DirectorySeparatorChar
    $stream = New-Object System.IO.MemoryStream(,$ZipBytes)
    try {
        $archive = New-Object System.IO.Compression.ZipArchive(
            $stream,
            [System.IO.Compression.ZipArchiveMode]::Read,
            $false
        )
        try {
            foreach ($entry in $archive.Entries) {
                if ([string]::IsNullOrEmpty($entry.FullName)) {
                    continue
                }

                $entryPath = $entry.FullName.Replace("/", [System.IO.Path]::DirectorySeparatorChar)
                $targetPath = [System.IO.Path]::GetFullPath([System.IO.Path]::Combine($destinationRoot, $entryPath))
                if (($targetPath -ne $destinationRoot) -and (-not $targetPath.StartsWith($destinationRootWithSeparator, [System.StringComparison]::OrdinalIgnoreCase))) {
                    throw "Archive entry '$($entry.FullName)' escapes destination '$destinationRoot'."
                }

                if ($entry.FullName.EndsWith("/")) {
                    Ensure-DirectoryExists -Path $targetPath
                    continue
                }

                $targetDir = Split-Path -Parent $targetPath
                if (-not [string]::IsNullOrEmpty($targetDir)) {
                    Ensure-DirectoryExists -Path $targetDir
                }

                $entryStream = $entry.Open()
                try {
                    $fileStream = [System.IO.File]::Open($targetPath, [System.IO.FileMode]::Create, [System.IO.FileAccess]::Write, [System.IO.FileShare]::None)
                    try {
                        $entryStream.CopyTo($fileStream)
                    }
                    finally {
                        $fileStream.Dispose()
                    }
                }
                finally {
                    $entryStream.Dispose()
                }
            }
        }
        finally {
            $archive.Dispose()
        }
    }
    finally {
        $stream.Dispose()
    }
}

function Write-Sentinel {
    param(
        [string]$DestinationDir,
        [string]$PackageName
    )

    $sentinelPath = Get-SentinelPath -DestinationDir $DestinationDir
    $content = @(
        "package=$PackageName"
        "ready_utc=$([DateTime]::UtcNow.ToString("o"))"
    )
    [System.IO.File]::WriteAllLines($sentinelPath, $content)
}

function Ensure-ZipPackageInstall {
    param(
        [string]$PackagesRoot,
        [string]$DestinationDir,
        [string]$PackageName,
        [string]$Label,
        [string]$ExpectedSha256 = ""
    )

    if (Test-Ready -DestinationDir $DestinationDir) {
        return
    }

    Invoke-WithDestinationFileLock -PackagesRoot $PackagesRoot -DestinationDir $DestinationDir -Label $Label -Body {
        if (Test-Ready -DestinationDir $DestinationDir) {
            return
        }

        Remove-DirectoryIfPresent -Path $DestinationDir
        Ensure-DirectoryExists -Path $DestinationDir

        $zipBytes = Invoke-DownloadBytes -PackageName $PackageName -DestinationLabel $Label
        if (-not [string]::IsNullOrEmpty($ExpectedSha256)) {
            $actualSha256 = Get-BytesSha256Hex -Bytes $zipBytes
            if ($actualSha256 -ne $ExpectedSha256.ToLowerInvariant()) {
                throw "Package '$PackageName' must have a sha256 of '$ExpectedSha256' but was found to have '$actualSha256'."
            }
        }
        Write-Log "Unpacking $Label ..."
        Expand-ZipBytes -ZipBytes $zipBytes -DestinationDir $DestinationDir
        Write-Sentinel -DestinationDir $DestinationDir -PackageName $PackageName
    }
}

function Write-Export {
    param(
        [string]$Name,
        [string]$Value
    )

    [Console]::Out.WriteLine($Name + "=" + $Value)
}

try {
    $pmPackagesRootWasProvided = -not [string]::IsNullOrEmpty($env:PM_PACKAGES_ROOT)
    if ($pmPackagesRootWasProvided) {
        $pmPackagesRoot = Resolve-NormalizedPath -Path $env:PM_PACKAGES_ROOT
    }
    else {
        $pmPackagesRoot = Resolve-NormalizedPath -Path (Get-DefaultPackagesRoot)
        Set-UserEnvironmentVariable -Name "PM_PACKAGES_ROOT" -Value $pmPackagesRoot
        Write-VisualStudioWarning
    }

    if (-not (Test-Path -LiteralPath $pmPackagesRoot)) {
        Write-Log "Creating packman packages cache at $pmPackagesRoot"
        Ensure-DirectoryExists -Path $pmPackagesRoot
    }

    if ($env:PM_PYTHON_EXT) {
        $pmPython = Resolve-NormalizedPath -Path $env:PM_PYTHON_EXT
    }
    else {
        $pmPythonBaseDir = Join-Path $pmPackagesRoot "python"
        $pmPythonDir = Join-Path $pmPythonBaseDir $PM_PYTHON_VERSION
        $pmPython = Resolve-NormalizedPath -Path (Join-Path $pmPythonDir "python.exe")
        Ensure-DirectoryExists -Path $pmPythonBaseDir
        Ensure-ZipPackageInstall -PackagesRoot $pmPackagesRoot -DestinationDir $pmPythonDir -PackageName ("python@" + $PM_PYTHON_VERSION + ".zip") -Label "Python interpreter"
    }

    if ($env:PM_MODULE_DIR_EXT) {
        $pmModuleDir = Resolve-NormalizedPath -Path $env:PM_MODULE_DIR_EXT
    }
    else {
        $pmModuleDir = Resolve-NormalizedPath -Path (Join-Path (Join-Path $pmPackagesRoot "packman-common") $PM_PACKMAN_VERSION)
        Ensure-ZipPackageInstall -PackagesRoot $pmPackagesRoot -DestinationDir $pmModuleDir -PackageName ("packman-common@" + $PM_PACKMAN_VERSION + ".zip") -Label "packman" -ExpectedSha256 $PM_PACKMAN_COMMON_SHA256
    }

    $pmModule = Resolve-NormalizedPath -Path (Join-Path $pmModuleDir "run.py")

    Write-Export -Name "PM_INSTALL_PATH" -Value $PM_INSTALL_PATH
    Write-Export -Name "PM_PACKAGES_ROOT" -Value $pmPackagesRoot
    Write-Export -Name "PM_PYTHON" -Value $pmPython
    Write-Export -Name "PM_MODULE_DIR" -Value $pmModuleDir
    Write-Export -Name "PM_MODULE" -Value $pmModule
    exit 0
}
catch {
    Write-Failure "!!! Failure while configuring local machine :( !!!"
    Write-Failure $_.Exception.Message
    exit 1
}

# SIG # Begin signature block
# MIIofwYJKoZIhvcNAQcCoIIocDCCKGwCAQExDzANBglghkgBZQMEAgEFADB5Bgor
# BgEEAYI3AgEEoGswaTA0BgorBgEEAYI3AgEeMCYCAwEAAAQQH8w7YFlLCE63JNLG
# KX7zUQIBAAIBAAIBAAIBAAIBADAxMA0GCWCGSAFlAwQCAQUABCAWwKj2dEbYNg/h
# mA+bs0J3T8vxT2+X/FQuZ4d/HB3WAKCCDbUwggawMIIEmKADAgECAhAIrUCyYNKc
# TJ9ezam9k67ZMA0GCSqGSIb3DQEBDAUAMGIxCzAJBgNVBAYTAlVTMRUwEwYDVQQK
# EwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5jb20xITAfBgNV
# BAMTGERpZ2lDZXJ0IFRydXN0ZWQgUm9vdCBHNDAeFw0yMTA0MjkwMDAwMDBaFw0z
# NjA0MjgyMzU5NTlaMGkxCzAJBgNVBAYTAlVTMRcwFQYDVQQKEw5EaWdpQ2VydCwg
# SW5jLjFBMD8GA1UEAxM4RGlnaUNlcnQgVHJ1c3RlZCBHNCBDb2RlIFNpZ25pbmcg
# UlNBNDA5NiBTSEEzODQgMjAyMSBDQTEwggIiMA0GCSqGSIb3DQEBAQUAA4ICDwAw
# ggIKAoICAQDVtC9C0CiteLdd1TlZG7GIQvUzjOs9gZdwxbvEhSYwn6SOaNhc9es0
# JAfhS0/TeEP0F9ce2vnS1WcaUk8OoVf8iJnBkcyBAz5NcCRks43iCH00fUyAVxJr
# Q5qZ8sU7H/Lvy0daE6ZMswEgJfMQ04uy+wjwiuCdCcBlp/qYgEk1hz1RGeiQIXhF
# LqGfLOEYwhrMxe6TSXBCMo/7xuoc82VokaJNTIIRSFJo3hC9FFdd6BgTZcV/sk+F
# LEikVoQ11vkunKoAFdE3/hoGlMJ8yOobMubKwvSnowMOdKWvObarYBLj6Na59zHh
# 3K3kGKDYwSNHR7OhD26jq22YBoMbt2pnLdK9RBqSEIGPsDsJ18ebMlrC/2pgVItJ
# wZPt4bRc4G/rJvmM1bL5OBDm6s6R9b7T+2+TYTRcvJNFKIM2KmYoX7BzzosmJQay
# g9Rc9hUZTO1i4F4z8ujo7AqnsAMrkbI2eb73rQgedaZlzLvjSFDzd5Ea/ttQokbI
# YViY9XwCFjyDKK05huzUtw1T0PhH5nUwjewwk3YUpltLXXRhTT8SkXbev1jLchAp
# QfDVxW0mdmgRQRNYmtwmKwH0iU1Z23jPgUo+QEdfyYFQc4UQIyFZYIpkVMHMIRro
# OBl8ZhzNeDhFMJlP/2NPTLuqDQhTQXxYPUez+rbsjDIJAsxsPAxWEQIDAQABo4IB
# WTCCAVUwEgYDVR0TAQH/BAgwBgEB/wIBADAdBgNVHQ4EFgQUaDfg67Y7+F8Rhvv+
# YXsIiGX0TkIwHwYDVR0jBBgwFoAU7NfjgtJxXWRM3y5nP+e6mK4cD08wDgYDVR0P
# AQH/BAQDAgGGMBMGA1UdJQQMMAoGCCsGAQUFBwMDMHcGCCsGAQUFBwEBBGswaTAk
# BggrBgEFBQcwAYYYaHR0cDovL29jc3AuZGlnaWNlcnQuY29tMEEGCCsGAQUFBzAC
# hjVodHRwOi8vY2FjZXJ0cy5kaWdpY2VydC5jb20vRGlnaUNlcnRUcnVzdGVkUm9v
# dEc0LmNydDBDBgNVHR8EPDA6MDigNqA0hjJodHRwOi8vY3JsMy5kaWdpY2VydC5j
# b20vRGlnaUNlcnRUcnVzdGVkUm9vdEc0LmNybDAcBgNVHSAEFTATMAcGBWeBDAED
# MAgGBmeBDAEEATANBgkqhkiG9w0BAQwFAAOCAgEAOiNEPY0Idu6PvDqZ01bgAhql
# +Eg08yy25nRm95RysQDKr2wwJxMSnpBEn0v9nqN8JtU3vDpdSG2V1T9J9Ce7FoFF
# UP2cvbaF4HZ+N3HLIvdaqpDP9ZNq4+sg0dVQeYiaiorBtr2hSBh+3NiAGhEZGM1h
# mYFW9snjdufE5BtfQ/g+lP92OT2e1JnPSt0o618moZVYSNUa/tcnP/2Q0XaG3Ryw
# YFzzDaju4ImhvTnhOE7abrs2nfvlIVNaw8rpavGiPttDuDPITzgUkpn13c5Ubdld
# AhQfQDN8A+KVssIhdXNSy0bYxDQcoqVLjc1vdjcshT8azibpGL6QB7BDf5WIIIJw
# 8MzK7/0pNVwfiThV9zeKiwmhywvpMRr/LhlcOXHhvpynCgbWJme3kuZOX956rEnP
# LqR0kq3bPKSchh/jwVYbKyP/j7XqiHtwa+aguv06P0WmxOgWkVKLQcBIhEuWTatE
# QOON8BUozu3xGFYHKi8QxAwIZDwzj64ojDzLj4gLDb879M4ee47vtevLt/B3E+bn
# KD+sEq6lLyJsQfmCXBVmzGwOysWGw/YmMwwHS6DTBwJqakAwSEs0qFEgu60bhQji
# WQ1tygVQK+pKHJ6l/aCnHwZ05/LWUpD9r4VIIflXO7ScA+2GRfS0YW6/aOImYIbq
# yK+p/pQd52MbOoZWeE4wggb9MIIE5aADAgECAhAJrnXkyTwXU23cf6lU5rhZMA0G
# CSqGSIb3DQEBCwUAMGkxCzAJBgNVBAYTAlVTMRcwFQYDVQQKEw5EaWdpQ2VydCwg
# SW5jLjFBMD8GA1UEAxM4RGlnaUNlcnQgVHJ1c3RlZCBHNCBDb2RlIFNpZ25pbmcg
# UlNBNDA5NiBTSEEzODQgMjAyMSBDQTEwHhcNMjUwNzMwMDAwMDAwWhcNMjgwNzI5
# MjM1OTU5WjCBhDELMAkGA1UEBhMCVVMxEzARBgNVBAgTCkNhbGlmb3JuaWExFDAS
# BgNVBAcTC1NhbnRhIENsYXJhMRswGQYDVQQKExJOVklESUEgQ29ycG9yYXRpb24x
# EDAOBgNVBAsTBzIwMDhCOUYxGzAZBgNVBAMTEk5WSURJQSBDb3Jwb3JhdGlvbjCC
# AaIwDQYJKoZIhvcNAQEBBQADggGPADCCAYoCggGBAKa2dMEsbSxk5eQlpZLnH3gh
# gkCcxlFDR/+as8oUqOztXFPrpGWMZkyOu06MNiPycUbVMh6pb8nJjr0ULpqPwd+7
# lPzZ1RBDiUizSvLW7QkcVrtcpaTzLV5N0aezqj1lXwFWU1MVGan0tXmcrSAaAFJG
# F6ChQbmh5ltEeJd+9ZQenKaH+eYZIPCS4Fk9sQefuuH060+oN4aME1iGyPL/l/cL
# wXnfGFkfy0TdmZDO+IHERhcjqrAFKfYgsDlDsUnOc6smhyRv1RFgsx2W73Mztt7U
# sZ81qORdZDTKztGykoQIC/YYo03iI7BfotTNY9c+81iN8qIsHBhdpRrerEKA/rm7
# dF8HrGjg7Nn+p75fRrIKZ7v86dOxPJm7s6HjDL/Ww37XwK+yK1XH+o+376bx0mFE
# OhmGczyn8YMUwz8frLHDb+Hi0Z0qLerYaU4Io1hxk6QCciCNToGwSzj+G+Cy1TH4
# DTtgl4A+GRDFMG9dY745HfRVlxdMlYpIMMfoO1kGIQIDAQABo4ICAzCCAf8wHwYD
# VR0jBBgwFoAUaDfg67Y7+F8Rhvv+YXsIiGX0TkIwHQYDVR0OBBYEFM8QW3t1WsAl
# sCyglf8KFZDQkFOgMD4GA1UdIAQ3MDUwMwYGZ4EMAQQBMCkwJwYIKwYBBQUHAgEW
# G2h0dHA6Ly93d3cuZGlnaWNlcnQuY29tL0NQUzAOBgNVHQ8BAf8EBAMCB4AwEwYD
# VR0lBAwwCgYIKwYBBQUHAwMwgbUGA1UdHwSBrTCBqjBToFGgT4ZNaHR0cDovL2Ny
# bDMuZGlnaWNlcnQuY29tL0RpZ2lDZXJ0VHJ1c3RlZEc0Q29kZVNpZ25pbmdSU0E0
# MDk2U0hBMzg0MjAyMUNBMS5jcmwwU6BRoE+GTWh0dHA6Ly9jcmw0LmRpZ2ljZXJ0
# LmNvbS9EaWdpQ2VydFRydXN0ZWRHNENvZGVTaWduaW5nUlNBNDA5NlNIQTM4NDIw
# MjFDQTEuY3JsMIGUBggrBgEFBQcBAQSBhzCBhDAkBggrBgEFBQcwAYYYaHR0cDov
# L29jc3AuZGlnaWNlcnQuY29tMFwGCCsGAQUFBzAChlBodHRwOi8vY2FjZXJ0cy5k
# aWdpY2VydC5jb20vRGlnaUNlcnRUcnVzdGVkRzRDb2RlU2lnbmluZ1JTQTQwOTZT
# SEEzODQyMDIxQ0ExLmNydDAJBgNVHRMEAjAAMA0GCSqGSIb3DQEBCwUAA4ICAQAb
# hocVBslPLkweNoXnzDyHjgUHVsdaBSxnKjHDTdOzXpo/a6VkK1VXK1fIhWYy4CcZ
# /wfyeb80+99KnfWWQzgL7nIElm4SkJRIMK8dODX3my4CQR6oSEsOimM1QUr8Gfio
# H98oQe8fhIQUOjnQsWiqbPzukx7ehCYnm2Xbu6nnSuvdzwFjiPykA91IFkVyP4Ex
# kf7JzWrFko4nceMXwtfGLB0jH1L+fmFUlAzXKNVIV/2GlXElSCMGHlUDy18D2hk+
# nij2DT1Gp+PmSSBCmQVIr+6HJMVXdz3jCtE3nQGKfAT+M8dvvEIq//0+cpkZdJxQ
# ctnHK5qukn8InimLh7fK8B+gu08wPHAhdEAc2eNM5Mmw67iCKN2/9hGvoVQlrtYm
# Ta1YglDOm4G0uSwtGIoa8O4S2uZES75HpAXRtc/hzSCOdeHR52wCMCx91OXrJ1kz
# NxIWIJZCE3NlzwKFObOZKONRiHPpC8oEToY/cxhTIn3Gg/70emY8JMvJoVfyokbu
# NfteFha3Z/5wIoxkcW971JaP2Z2TO1StjGcKEZvUF2OW7PQ7Toh55AowUx2UuSZQ
# MioaOVUrhqxcYWSBobA1Y3mwvuRfAgMqrfQ+eIwWYX+t7qg9fNjTPtxlQViE1H3+
# cRW9p8uV5rqJXhNzXmKYH8k4AZoUUOMnY//9ZE8TtjGCGiAwghocAgEBMH0waTEL
# MAkGA1UEBhMCVVMxFzAVBgNVBAoTDkRpZ2lDZXJ0LCBJbmMuMUEwPwYDVQQDEzhE
# aWdpQ2VydCBUcnVzdGVkIEc0IENvZGUgU2lnbmluZyBSU0E0MDk2IFNIQTM4NCAy
# MDIxIENBMQIQCa515Mk8F1Nt3H+pVOa4WTANBglghkgBZQMEAgEFAKB8MBAGCisG
# AQQBgjcCAQwxAjAAMBkGCSqGSIb3DQEJAzEMBgorBgEEAYI3AgEEMBwGCisGAQQB
# gjcCAQsxDjAMBgorBgEEAYI3AgEVMC8GCSqGSIb3DQEJBDEiBCAHZUPKZVXcGpNc
# UB7Q/h1O9rGaDATEYdRrcxwGZovEnjANBgkqhkiG9w0BAQEFAASCAYBOP3Ho+po5
# XLUDHTLze5ljcJMxgRepN06jPeufsK8S9WVM1gHurmvAfzY8dz2ov68M0rUWfKxF
# 1FdBUHLiFAbdVIozGMViF2noVCjSBFEcz6V7SO+nM2rN24mmeTMY8MUQQbxEQQIr
# A38NtTHpQs0Ab6qp74lMPhpdtO7W9goqYRdDhYqhhUXCrTxMwOYXaa8TmHdfLQXk
# BkvZH52Vf7hrFj907zlTYz7k7pc67BM6VDKG/rXsa+z57yKJBGsh0AIEFqpG3Vzi
# C83j2XIDDi/Gu80kCRjlbzlnvq1aurV1XAMr1kgYEV101L4vI6+BZ/kLCZKlheUc
# wXkdaPlVlY1zrVYgO4qCcubwevXpQdFAqeB12nswn6i5y0uBuOOQhJxzJLmQ2FrE
# qWPB6HDrp08upi+DsIl7xz8qcRHrZLeBMYvdLw7tMSOgIeLfFS/rKj7EdqCcs1h6
# /aBa3MVPZHnfGzs9rOvtP2uBgrOcAep806q6CqD95bHFHZN+ZsDlRnqhghd2MIIX
# cgYKKwYBBAGCNwMDATGCF2IwghdeBgkqhkiG9w0BBwKgghdPMIIXSwIBAzEPMA0G
# CWCGSAFlAwQCAQUAMHcGCyqGSIb3DQEJEAEEoGgEZjBkAgEBBglghkgBhv1sBwEw
# MTANBglghkgBZQMEAgEFAAQgi8XXHRMZUXNXuWbDRcl5gt8rK6pYDUTm1YrsyNFB
# kqYCEDoW9ExivtX6n5EQMMpBEBUYDzIwMjYwNDIwMTgyOTMyWqCCEzowggbtMIIE
# 1aADAgECAhAKgO8YS43xBYLRxHanlXRoMA0GCSqGSIb3DQEBCwUAMGkxCzAJBgNV
# BAYTAlVTMRcwFQYDVQQKEw5EaWdpQ2VydCwgSW5jLjFBMD8GA1UEAxM4RGlnaUNl
# cnQgVHJ1c3RlZCBHNCBUaW1lU3RhbXBpbmcgUlNBNDA5NiBTSEEyNTYgMjAyNSBD
# QTEwHhcNMjUwNjA0MDAwMDAwWhcNMzYwOTAzMjM1OTU5WjBjMQswCQYDVQQGEwJV
# UzEXMBUGA1UEChMORGlnaUNlcnQsIEluYy4xOzA5BgNVBAMTMkRpZ2lDZXJ0IFNI
# QTI1NiBSU0E0MDk2IFRpbWVzdGFtcCBSZXNwb25kZXIgMjAyNSAxMIICIjANBgkq
# hkiG9w0BAQEFAAOCAg8AMIICCgKCAgEA0EasLRLGntDqrmBWsytXum9R/4ZwCgHf
# yjfMGUIwYzKomd8U1nH7C8Dr0cVMF3BsfAFI54um8+dnxk36+jx0Tb+k+87H9WPx
# NyFPJIDZHhAqlUPt281mHrBbZHqRK71Em3/hCGC5KyyneqiZ7syvFXJ9A72wzHpk
# BaMUNg7MOLxI6E9RaUueHTQKWXymOtRwJXcrcTTPPT2V1D/+cFllESviH8YjoPFv
# ZSjKs3SKO1QNUdFd2adw44wDcKgH+JRJE5Qg0NP3yiSyi5MxgU6cehGHr7zou1zn
# OM8odbkqoK+lJ25LCHBSai25CFyD23DZgPfDrJJJK77epTwMP6eKA0kWa3osAe8f
# cpK40uhktzUd/Yk0xUvhDU6lvJukx7jphx40DQt82yepyekl4i0r8OEps/FNO4ah
# fvAk12hE5FVs9HVVWcO5J4dVmVzix4A77p3awLbr89A90/nWGjXMGn7FQhmSlIUD
# y9Z2hSgctaepZTd0ILIUbWuhKuAeNIeWrzHKYueMJtItnj2Q+aTyLLKLM0MheP/9
# w6CtjuuVHJOVoIJ/DtpJRE7Ce7vMRHoRon4CWIvuiNN1Lk9Y+xZ66lazs2kKFSTn
# nkrT3pXWETTJkhd76CIDBbTRofOsNyEhzZtCGmnQigpFHti58CSmvEyJcAlDVcKa
# cJ+A9/z7eacCAwEAAaOCAZUwggGRMAwGA1UdEwEB/wQCMAAwHQYDVR0OBBYEFOQ7
# /PIx7f391/ORcWMZUEPPYYzoMB8GA1UdIwQYMBaAFO9vU0rp5AZ8esrikFb2L9RJ
# 7MtOMA4GA1UdDwEB/wQEAwIHgDAWBgNVHSUBAf8EDDAKBggrBgEFBQcDCDCBlQYI
# KwYBBQUHAQEEgYgwgYUwJAYIKwYBBQUHMAGGGGh0dHA6Ly9vY3NwLmRpZ2ljZXJ0
# LmNvbTBdBggrBgEFBQcwAoZRaHR0cDovL2NhY2VydHMuZGlnaWNlcnQuY29tL0Rp
# Z2lDZXJ0VHJ1c3RlZEc0VGltZVN0YW1waW5nUlNBNDA5NlNIQTI1NjIwMjVDQTEu
# Y3J0MF8GA1UdHwRYMFYwVKBSoFCGTmh0dHA6Ly9jcmwzLmRpZ2ljZXJ0LmNvbS9E
# aWdpQ2VydFRydXN0ZWRHNFRpbWVTdGFtcGluZ1JTQTQwOTZTSEEyNTYyMDI1Q0Ex
# LmNybDAgBgNVHSAEGTAXMAgGBmeBDAEEAjALBglghkgBhv1sBwEwDQYJKoZIhvcN
# AQELBQADggIBAGUqrfEcJwS5rmBB7NEIRJ5jQHIh+OT2Ik/bNYulCrVvhREafBYF
# 0RkP2AGr181o2YWPoSHz9iZEN/FPsLSTwVQWo2H62yGBvg7ouCODwrx6ULj6hYKq
# dT8wv2UV+Kbz/3ImZlJ7YXwBD9R0oU62PtgxOao872bOySCILdBghQ/ZLcdC8cbU
# UO75ZSpbh1oipOhcUT8lD8QAGB9lctZTTOJM3pHfKBAEcxQFoHlt2s9sXoxFizTe
# HihsQyfFg5fxUFEp7W42fNBVN4ueLaceRf9Cq9ec1v5iQMWTFQa0xNqItH3CPFTG
# 7aEQJmmrJTV3Qhtfparz+BW60OiMEgV5GWoBy4RVPRwqxv7Mk0Sy4QHs7v9y69NB
# qycz0BZwhB9WOfOu/CIJnzkQTwtSSpGGhLdjnQ4eBpjtP+XB3pQCtv4E5UCSDag6
# +iX8MmB10nfldPF9SVD7weCC3yXZi/uuhqdwkgVxuiMFzGVFwYbQsiGnoa9F5AaA
# yBjFBtXVLcKtapnMG3VH3EmAp/jsJ3FVF3+d1SVDTmjFjLbNFZUWMXuZyvgLfgyP
# ehwJVxwC+UpX2MSey2ueIu9THFVkT+um1vshETaWyQo8gmBto/m3acaP9QsuLj3F
# NwFlTxq25+T4QwX9xa6ILs84ZPvmpovq90K8eWyG2N01c4IhSOxqt81nMIIGtDCC
# BJygAwIBAgIQDcesVwX/IZkuQEMiDDpJhjANBgkqhkiG9w0BAQsFADBiMQswCQYD
# VQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3d3cuZGln
# aWNlcnQuY29tMSEwHwYDVQQDExhEaWdpQ2VydCBUcnVzdGVkIFJvb3QgRzQwHhcN
# MjUwNTA3MDAwMDAwWhcNMzgwMTE0MjM1OTU5WjBpMQswCQYDVQQGEwJVUzEXMBUG
# A1UEChMORGlnaUNlcnQsIEluYy4xQTA/BgNVBAMTOERpZ2lDZXJ0IFRydXN0ZWQg
# RzQgVGltZVN0YW1waW5nIFJTQTQwOTYgU0hBMjU2IDIwMjUgQ0ExMIICIjANBgkq
# hkiG9w0BAQEFAAOCAg8AMIICCgKCAgEAtHgx0wqYQXK+PEbAHKx126NGaHS0URed
# Ta2NDZS1mZaDLFTtQ2oRjzUXMmxCqvkbsDpz4aH+qbxeLho8I6jY3xL1IusLopuW
# 2qftJYJaDNs1+JH7Z+QdSKWM06qchUP+AbdJgMQB3h2DZ0Mal5kYp77jYMVQXSZH
# ++0trj6Ao+xh/AS7sQRuQL37QXbDhAktVJMQbzIBHYJBYgzWIjk8eDrYhXDEpKk7
# RdoX0M980EpLtlrNyHw0Xm+nt5pnYJU3Gmq6bNMI1I7Gb5IBZK4ivbVCiZv7PNBY
# qHEpNVWC2ZQ8BbfnFRQVESYOszFI2Wv82wnJRfN20VRS3hpLgIR4hjzL0hpoYGk8
# 1coWJ+KdPvMvaB0WkE/2qHxJ0ucS638ZxqU14lDnki7CcoKCz6eum5A19WZQHkqU
# JfdkDjHkccpL6uoG8pbF0LJAQQZxst7VvwDDjAmSFTUms+wV/FbWBqi7fTJnjq3h
# j0XbQcd8hjj/q8d6ylgxCZSKi17yVp2NL+cnT6Toy+rN+nM8M7LnLqCrO2JP3oW/
# /1sfuZDKiDEb1AQ8es9Xr/u6bDTnYCTKIsDq1BtmXUqEG1NqzJKS4kOmxkYp2WyO
# Di7vQTCBZtVFJfVZ3j7OgWmnhFr4yUozZtqgPrHRVHhGNKlYzyjlroPxul+bgIsp
# zOwbtmsgY1MCAwEAAaOCAV0wggFZMBIGA1UdEwEB/wQIMAYBAf8CAQAwHQYDVR0O
# BBYEFO9vU0rp5AZ8esrikFb2L9RJ7MtOMB8GA1UdIwQYMBaAFOzX44LScV1kTN8u
# Zz/nupiuHA9PMA4GA1UdDwEB/wQEAwIBhjATBgNVHSUEDDAKBggrBgEFBQcDCDB3
# BggrBgEFBQcBAQRrMGkwJAYIKwYBBQUHMAGGGGh0dHA6Ly9vY3NwLmRpZ2ljZXJ0
# LmNvbTBBBggrBgEFBQcwAoY1aHR0cDovL2NhY2VydHMuZGlnaWNlcnQuY29tL0Rp
# Z2lDZXJ0VHJ1c3RlZFJvb3RHNC5jcnQwQwYDVR0fBDwwOjA4oDagNIYyaHR0cDov
# L2NybDMuZGlnaWNlcnQuY29tL0RpZ2lDZXJ0VHJ1c3RlZFJvb3RHNC5jcmwwIAYD
# VR0gBBkwFzAIBgZngQwBBAIwCwYJYIZIAYb9bAcBMA0GCSqGSIb3DQEBCwUAA4IC
# AQAXzvsWgBz+Bz0RdnEwvb4LyLU0pn/N0IfFiBowf0/Dm1wGc/Do7oVMY2mhXZXj
# DNJQa8j00DNqhCT3t+s8G0iP5kvN2n7Jd2E4/iEIUBO41P5F448rSYJ59Ib61eoa
# lhnd6ywFLerycvZTAz40y8S4F3/a+Z1jEMK/DMm/axFSgoR8n6c3nuZB9BfBwAQY
# K9FHaoq2e26MHvVY9gCDA/JYsq7pGdogP8HRtrYfctSLANEBfHU16r3J05qX3kId
# +ZOczgj5kjatVB+NdADVZKON/gnZruMvNYY2o1f4MXRJDMdTSlOLh0HCn2cQLwQC
# qjFbqrXuvTPSegOOzr4EWj7PtspIHBldNE2K9i697cvaiIo2p61Ed2p8xMJb82Yo
# sn0z4y25xUbI7GIN/TpVfHIqQ6Ku/qjTY6hc3hsXMrS+U0yy+GWqAXam4ToWd2UQ
# 1KYT70kZjE4YtL8Pbzg0c1ugMZyZZd/BdHLiRu7hAWE6bTEm4XYRkA6Tl4KSFLFk
# 43esaUeqGkH/wyW4N7OigizwJWeukcyIPbAvjSabnf7+Pu0VrFgoiovRDiyx3zEd
# mcif/sYQsfch28bZeUz2rtY/9TCA6TD8dC3JE3rYkrhLULy7Dc90G6e8BlqmyIjl
# gp2+VqsS9/wQD7yFylIz0scmbKvFoW2jNrbM1pD2T7m3XDCCBY0wggR1oAMCAQIC
# EA6bGI750C3n79tQ4ghAGFowDQYJKoZIhvcNAQEMBQAwZTELMAkGA1UEBhMCVVMx
# FTATBgNVBAoTDERpZ2lDZXJ0IEluYzEZMBcGA1UECxMQd3d3LmRpZ2ljZXJ0LmNv
# bTEkMCIGA1UEAxMbRGlnaUNlcnQgQXNzdXJlZCBJRCBSb290IENBMB4XDTIyMDgw
# MTAwMDAwMFoXDTMxMTEwOTIzNTk1OVowYjELMAkGA1UEBhMCVVMxFTATBgNVBAoT
# DERpZ2lDZXJ0IEluYzEZMBcGA1UECxMQd3d3LmRpZ2ljZXJ0LmNvbTEhMB8GA1UE
# AxMYRGlnaUNlcnQgVHJ1c3RlZCBSb290IEc0MIICIjANBgkqhkiG9w0BAQEFAAOC
# Ag8AMIICCgKCAgEAv+aQc2jeu+RdSjwwIjBpM+zCpyUuySE98orYWcLhKac9WKt2
# ms2uexuEDcQwH/MbpDgW61bGl20dq7J58soR0uRf1gU8Ug9SH8aeFaV+vp+pVxZZ
# VXKvaJNwwrK6dZlqczKU0RBEEC7fgvMHhOZ0O21x4i0MG+4g1ckgHWMpLc7sXk7I
# k/ghYZs06wXGXuxbGrzryc/NrDRAX7F6Zu53yEioZldXn1RYjgwrt0+nMNlW7sp7
# XeOtyU9e5TXnMcvak17cjo+A2raRmECQecN4x7axxLVqGDgDEI3Y1DekLgV9iPWC
# PhCRcKtVgkEy19sEcypukQF8IUzUvK4bA3VdeGbZOjFEmjNAvwjXWkmkwuapoGfd
# pCe8oU85tRFYF/ckXEaPZPfBaYh2mHY9WV1CdoeJl2l6SPDgohIbZpp0yt5LHucO
# Y67m1O+SkjqePdwA5EUlibaaRBkrfsCUtNJhbesz2cXfSwQAzH0clcOP9yGyshG3
# u3/y1YxwLEFgqrFjGESVGnZifvaAsPvoZKYz0YkH4b235kOkGLimdwHhD5QMIR2y
# VCkliWzlDlJRR3S+Jqy2QXXeeqxfjT/JvNNBERJb5RBQ6zHFynIWIgnffEx1P2Ps
# IV/EIFFrb7GrhotPwtZFX50g/KEexcCPorF+CiaZ9eRpL5gdLfXZqbId5RsCAwEA
# AaOCATowggE2MA8GA1UdEwEB/wQFMAMBAf8wHQYDVR0OBBYEFOzX44LScV1kTN8u
# Zz/nupiuHA9PMB8GA1UdIwQYMBaAFEXroq/0ksuCMS1Ri6enIZ3zbcgPMA4GA1Ud
# DwEB/wQEAwIBhjB5BggrBgEFBQcBAQRtMGswJAYIKwYBBQUHMAGGGGh0dHA6Ly9v
# Y3NwLmRpZ2ljZXJ0LmNvbTBDBggrBgEFBQcwAoY3aHR0cDovL2NhY2VydHMuZGln
# aWNlcnQuY29tL0RpZ2lDZXJ0QXNzdXJlZElEUm9vdENBLmNydDBFBgNVHR8EPjA8
# MDqgOKA2hjRodHRwOi8vY3JsMy5kaWdpY2VydC5jb20vRGlnaUNlcnRBc3N1cmVk
# SURSb290Q0EuY3JsMBEGA1UdIAQKMAgwBgYEVR0gADANBgkqhkiG9w0BAQwFAAOC
# AQEAcKC/Q1xV5zhfoKN0Gz22Ftf3v1cHvZqsoYcs7IVeqRq7IviHGmlUIu2kiHdt
# vRoU9BNKei8ttzjv9P+Aufih9/Jy3iS8UgPITtAq3votVs/59PesMHqai7Je1M/R
# Q0SbQyHrlnKhSLSZy51PpwYDE3cnRNTnf+hZqPC/Lwum6fI0POz3A8eHqNJMQBk1
# RmppVLC4oVaO7KTVPeix3P0c2PR3WlxUjG/voVA9/HYJaISfb8rbII01YBwCA8sg
# sKxYoA5AY8WYIsGyWfVVa88nq2x2zm8jLfR+cWojayL/ErhULSd+2DrZ8LaHlv1b
# 0VysGMNNn3O3AamfV6peKOK5lDGCA3wwggN4AgEBMH0waTELMAkGA1UEBhMCVVMx
# FzAVBgNVBAoTDkRpZ2lDZXJ0LCBJbmMuMUEwPwYDVQQDEzhEaWdpQ2VydCBUcnVz
# dGVkIEc0IFRpbWVTdGFtcGluZyBSU0E0MDk2IFNIQTI1NiAyMDI1IENBMQIQCoDv
# GEuN8QWC0cR2p5V0aDANBglghkgBZQMEAgEFAKCB0TAaBgkqhkiG9w0BCQMxDQYL
# KoZIhvcNAQkQAQQwHAYJKoZIhvcNAQkFMQ8XDTI2MDQyMDE4MjkzMlowKwYLKoZI
# hvcNAQkQAgwxHDAaMBgwFgQU3WIwrIYKLTBr2jixaHlSMAf7QX4wLwYJKoZIhvcN
# AQkEMSIEICaY4QeE6Fmy2B4rhjFiAat3DCIilw2l/8ajeP/XH0wMMDcGCyqGSIb3
# DQEJEAIvMSgwJjAkMCIEIEqgP6Is11yExVyTj4KOZ2ucrsqzP+NtJpqjNPFGEQoz
# MA0GCSqGSIb3DQEBAQUABIICAHC6pfwLarK8w2Q3D9tKscJgzURlt1TIeGT4eg9L
# Q5RcoCADVfnDY77cFdbzuuR/l0f8iEtBnLlFXcMKNhsn2c3QljZLZoCSG01HADAl
# YdIGO4vnKzC+EfIsButXKZWlBDbHGk45yAF4CpAHX3K27fyjKqD9Jee/owNdeXNE
# HKAqibVWtpbDeu5GoGa/b3lQj8SxIaDZ9Gy8RfNjOFtMaYf+M7deuiyJ657Q6o59
# Vy99hzxvxoUI6LbyfKi1kL8Uvd+zSpj7H5N9+eCwDThPJqEB0vP9cNYEgnsRGBEh
# GytRuRJvffxiuYTEQkdR03Lr2NcmI93DcliLSPWFhyrR95+b+Z6mipKeNA4rx9fz
# I/IwW4eHEf6Ce99w2ZuDgT7c48BOgzYFQ3wRRMiav9IuRc1BJ3ujXrL3vuZxMQEG
# vNoy5+ivmvf+AwInTd2Qlncqi1XoWiSIwUN9esiRcvWrkSwM04wPd5q0QwXSBG2d
# AHFzKbBeyCnpcnMJz2STXP5uTEeauwpv9seAmF0RXGd7952M+mhdBmft7fILqKZk
# 6DrgTWCm9L5mB6JMXx9tfk7qBu/8AEFRItP/v4Q9x/MQ3OX18s3DIEYlBSIiUWkV
# fEpcEgXJEeFQmANUNx7BjeB2zH5XU5I1MiTGEiXo5uRLl2SYHds5N+q9cQ1AQUnA
# UoAe
# SIG # End signature block
