[CmdletBinding()]
param(
    [string]$Distro = 'Ubuntu-24.04',
    # Insta360 X4 UVC 模式的 VID:PID。已实测（2026-08-29）：2e1a:0003（USB 摄像头模式）。
    # 若固件升级后 PID 变化，按 docs/insta/insta360-x4-integration-guide.md Phase 0 重新实测替换。
    [string]$VidPid = '2e1a:0003',
    [switch]$Detach
)

$ErrorActionPreference = 'Stop'
$Usbipd = 'C:\Program Files\usbipd-win\usbipd.exe'

$principal = New-Object Security.Principal.WindowsPrincipal([Security.Principal.WindowsIdentity]::GetCurrent())
if (-not $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
    $argList = @('-ExecutionPolicy', 'Bypass', '-File', $PSCommandPath)
    if ($Distro) { $argList += @('-Distro', $Distro) }
    if ($VidPid) { $argList += @('-VidPid', $VidPid) }
    if ($Detach) { $argList += '-Detach' }
    Start-Process powershell.exe -Verb RunAs -ArgumentList $argList -Wait
    exit $LASTEXITCODE
}

if (-not (Test-Path -LiteralPath $Usbipd)) {
    throw 'usbipd-win was not found. Install usbipd-win first.'
}

if ($VidPid -eq '2e1a:0000') {
    Write-Warning 'Using placeholder VID:PID 2e1a:0000. Pass the real value via -VidPid (measured default for X4 UVC: 2e1a:0003).'
}

$lines = @(& $Usbipd list)
$device = $lines | Where-Object { $_ -match "^\s*\S+\s+$([regex]::Escape($VidPid))\s+" } | Select-Object -First 1
if (-not $device) {
    throw "USB device $VidPid was not found. Ensure the X4 is on, connected by USB data cable, and 'USB camera' mode is selected on the camera screen. Also close any Windows app occupying the camera (Camera app / OBS / meeting software)."
}

$busId = ($device.Trim() -split '\s+')[0]
Write-Host "Found Insta360 UVC device: BUSID $busId"

if ($Detach) {
    & $Usbipd detach --busid $busId
    Write-Host "Detached $busId from WSL. The camera is back to Windows."
    exit 0
}

& $Usbipd bind --busid $busId
& $Usbipd attach --wsl --busid $busId
Write-Host "Attached BUSID $busId to WSL distro $Distro."

Start-Sleep -Seconds 3
$devices = wsl.exe -d $Distro -- bash -lc 'ls -1 /dev/video* 2>/dev/null || true'
if ($devices) {
    Write-Host 'WSL video devices:'
    $devices | Write-Host
    Write-Host 'Verify formats: v4l2-ctl -d /dev/video0 --list-formats-ext (expect 2880x1440@30).'
    Write-Host 'If access is denied, run in WSL: sudo usermod -aG video $USER, then restart WSL.'
} else {
    Write-Warning 'No /dev/video* device is visible in WSL yet. Re-run the script after a few seconds, or check that no Windows app is still holding the camera.'
}
