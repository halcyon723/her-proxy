# Reset port 9000, start fresh receiver, run demo test
$ErrorActionPreference = "Continue"

Write-Output "=== 1. kill all listeners on port 9000 ==="
$seen = @{}
for ($i = 0; $i -lt 10; $i++) {
    $conns = Get-NetTCPConnection -LocalPort 9000 -State Listen -ErrorAction SilentlyContinue
    if (-not $conns) { break }
    foreach ($c in $conns) {
        $pidToKill = $c.OwningProcess
        if (-not $seen.ContainsKey($pidToKill)) {
            $seen[$pidToKill] = $true
            Write-Output "  killing PID $pidToKill"
            Stop-Process -Id $pidToKill -Force -ErrorAction SilentlyContinue
            & taskkill.exe /F /PID $pidToKill 2>&1 | Out-Null
        }
    }
    Start-Sleep -Milliseconds 500
}
if (Get-NetTCPConnection -LocalPort 9000 -State Listen -ErrorAction SilentlyContinue) {
    Write-Output "!! port still busy, abort"; exit 1
}
Write-Output "  port is free"

Write-Output "=== 2. start fresh receiver ==="
$recv = Start-Process python -ArgumentList (Join-Path $PSScriptRoot "receiver.py") -RedirectStandardOutput "$env:TEMP\recvfinal_out.txt" -RedirectStandardError "$env:TEMP\recvfinal_err.txt" -PassThru -NoNewWindow
Write-Output ("  receiver PID=" + $recv.Id)
Start-Sleep -Seconds 3

Write-Output "=== 3. verify listener ==="
foreach ($c in Get-NetTCPConnection -LocalPort 9000 -State Listen -ErrorAction SilentlyContinue) {
    Write-Output ("  listener PID=" + $c.OwningProcess + " (expected " + $recv.Id + ")")
}

Write-Output "=== 4. send demo commands ==="
python (Join-Path $PSScriptRoot "sender.py") --host 127.0.0.1 --demo 2>&1 | ForEach-Object { Write-Output ("  " + $_) }

Write-Output "=== 5. stop receiver ==="
Stop-Process -Id $recv.Id -Force -ErrorAction SilentlyContinue
Write-Output "DONE"
