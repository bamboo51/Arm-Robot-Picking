
# Change 7890 to your actual proxy port.
# Use this address if the proxy runs on your Windows PC.
$proxyUrl = "http://wproxy.net.sendai-nct.ac.jp:8080"

# Create your project folder if needed
New-Item -ItemType Directory -Force "C:\crane-arm\src" | Out-Null

docker run `
  --gpus all `
  -e RESOLUTION=1920x1080 `
  -e "HTTP_PROXY=$proxyUrl" `
  -e "HTTPS_PROXY=$proxyUrl" `
  -e "http_proxy=$proxyUrl" `
  -e "https_proxy=$proxyUrl" `
  -e "NO_PROXY=localhost,127.0.0.1,::1" `
  -e "no_proxy=localhost,127.0.0.1,::1" `
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
