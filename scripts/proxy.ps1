# Sets $proxyUrl from PROXY_URL in the repo's .env (empty when there is no .env or no value).
# Dot-source it: . "$PSScriptRoot\proxy.ps1"
$proxyUrl = ""
$envFile = Join-Path $PSScriptRoot "..\.env"
if (Test-Path $envFile) {
  $line = Get-Content $envFile | Where-Object { $_ -match '^\s*PROXY_URL\s*=\s*(.*)$' } | Select-Object -Last 1
  if ($line) { $proxyUrl = $Matches[1].Trim().Trim('"') }
}
