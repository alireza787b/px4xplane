# Optional Windows-only QGC diagnostic. Run as Administrator with QGC listening.
# This adds a local UDP rule scoped to the current default WSL distribution IP.
$ErrorActionPreference = 'Stop'
$identity = [Security.Principal.WindowsIdentity]::GetCurrent()
$principal = New-Object Security.Principal.WindowsPrincipal($identity)
if (-not $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
    throw 'Run this script in an Administrator PowerShell, or allow QGC through the firewall manually.'
}
$raw = (& wsl.exe -- ip -4 -o addr show eth0 | Out-String)
if ($LASTEXITCODE -ne 0 -or $raw -notmatch 'inet\s+(\d+\.\d+\.\d+\.\d+)/') {
    throw 'Cannot detect WSL eth0 IPv4. No firewall changes made. Check networking mode manually.'
}
$wslIP = $Matches[1]
$rule = 'PX4-Ref6-QGC-WSL-UDP14550'
if (-not (Get-NetUDPEndpoint -LocalPort 14550 -ErrorAction SilentlyContinue)) {
    throw 'QGC is not listening on UDP 14550. Open/enable the UDP link first.'
}
Write-Host "Allow inbound UDP 14550 only from WSL $wslIP on this PC. No WAN routing."
if ((Read-Host 'Type YES to add or update this experimental rule') -ne 'YES') { exit 1 }
if (Get-NetFirewallRule -Name $rule -ErrorAction SilentlyContinue) {
    Remove-NetFirewallRule -Name $rule
}
New-NetFirewallRule -Name $rule -DisplayName 'PX4 Ref6 QGC from WSL only' `
    -Direction Inbound -Action Allow -Protocol UDP -LocalPort 14550 `
    -RemoteAddress $wslIP -Profile Any | Out-Null
Write-Host "Rule added for $wslIP. Refresh after WSL IP changes."
Write-Host "Remove when finished: Remove-NetFirewallRule -Name '$rule'"
