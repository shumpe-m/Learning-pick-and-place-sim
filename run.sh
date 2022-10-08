docker run --rm -it --gpus all --privileged --net=host --ipc=host \
    -e DOCKER_USER_NAME=$(id -un) \
    -e DOCKER_USER_ID=$(id -u) \
    -e DOCKER_USER_GROUP_NAME=$(id -gn) \
    -e DOCKER_USER_GROUP_ID=$(id -g) \
    -v $HOME/.Xauthority:/home/$(id -un)/.Xauthority -e XAUTHORITY=/home/$(id -un)/.Xauthority \
    -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY \
    -v /dev/snd:/dev/snd -e AUDIODEV="hw:Device, 0" \
    -v $HOME/learning-pick-and-place:$HOME/learning-pick-and-place \
    learning-pick-and-place-docker   
