# Build crane-arm:humble from the repo root, passing the proxy from .env when one is set.
. "$PSScriptRoot\proxy.ps1"
$buildArgs = @()
if ($proxyUrl) { $buildArgs = "--build-arg", "HTTP_PROXY=$proxyUrl", "--build-arg", "HTTPS_PROXY=$proxyUrl" }

docker build -t crane-arm:humble -f "$PSScriptRoot\..\docker\Dockerfile" @buildArgs "$PSScriptRoot\.."
