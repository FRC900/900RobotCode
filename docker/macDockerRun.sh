ip=$(ifconfig en0 | grep inet | awk '$1=="inet" {print $2}')
docker run -it --privileged -v /tmp/.X11-unix:/tmp/.X11-unix --net=host \
-v $(pwd)/../../other:/home/ubuntu/other -v $(pwd)/..:/home/ubuntu/900RobotCode -e DISPLAY=$ip:0 --user ubuntu frc900/zebros-2025-dev:latest /bin/bash
