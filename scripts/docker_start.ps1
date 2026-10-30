# The proxy comes from PROXY_URL in .env (see .env.example); without it the container gets no proxy.
. "$PSScriptRoot\proxy.ps1"
$proxyArgs = @()
if ($proxyUrl) {
  foreach ($name in "HTTP_PROXY", "HTTPS_PROXY", "http_proxy", "https_proxy") {
    $proxyArgs += "-e", "$name=$proxyUrl"
  }
  $proxyArgs += "-e", "NO_PROXY=localhost,127.0.0.1,::1", "-e", "no_proxy=localhost,127.0.0.1,::1"
}

New-Item -ItemType Directory -Force "C:\crane-arm\src" | Out-Null

docker run `
  --gpus all `
  -e RESOLUTION=1920x1080 `
  @proxyArgs `
  --name custom-crane `
  -p 15900:5900 `
  -p 13389:3389 `
  -p 9090:9090 `
  -p 9876:9876 `
  -p 6080:80 `
  -v "C:\Users\pongs\Documents\GitHub\Arm-Robot-Picking:/home/ubuntu/airobot_ws/src/user_code" `
  --shm-size=512m `
  --privileged `
  crane-arm:humble
