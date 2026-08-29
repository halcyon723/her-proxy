[CmdletBinding()]
param(
    [string]$Distro = 'Ubuntu-24.04',
    [string]$VidPid = 'caf1:ffff',
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

$lines = @(& $Usbipd list)
$device = $lines | Where-Object { $_ -match "^\s*\S+\s+$([regex]::Escape($VidPid))\s+" } | Select-Object -First 1
if (-not $device) {
    throw "USB device $VidPid was not found. Check that COM10-COM16 are connected."
}

$busId = ($device.Trim() -split '\s+')[0]
Write-Host "Found USB serial composite device: BUSID $busId (COM10-COM16)"

if ($Detach) {
    & $Usbipd detach --busid $busId
    Write-Host "Detached $busId from WSL."
    exit 0
}

& $Usbipd bind --busid $busId
& $Usbipd attach --wsl --busid $busId
Write-Host "Attached BUSID $busId to WSL distro $Distro."

Start-Sleep -Seconds 2
$devices = wsl.exe -d $Distro -- bash -lc 'ls -1 /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || true'
if ($devices) {
    Write-Host 'WSL serial devices:'
    $devices | Write-Host
    Write-Host 'Note: COM10-COM16 appear in WSL as ttyACM0-ttyACM6, not as COM10-COM16.'
    $check = wsl.exe -d $Distro -- bash -lc 'python3 -c "import glob,os; ds=glob.glob(\"/dev/ttyACM*\")+glob.glob(\"/dev/ttyUSB*\"); ok=0; [os.close(os.open(d,os.O_RDWR|os.O_NOCTTY|os.O_NONBLOCK)) or (print(d),globals().__setitem__(\"ok\",ok+1)) for d in ds]; print(\"SERIAL_RW_COUNT=\"+str(ok))"'
    $check | Write-Host
    Write-Host 'If access is denied, run in WSL: sudo usermod -aG dialout $USER, then restart WSL.'
} else {
    Write-Warning 'No ttyACM/ttyUSB device is visible in WSL yet. Re-run the script after a few seconds.'
}
