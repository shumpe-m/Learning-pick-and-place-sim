nvidia/cuda:11.0.3-devel-ubuntu16.04

# for avoidance of 'tzdata' configuring
RUN DEBIAN_FRONTEND=noninteractive apt install postfix -y
# for GUI
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics


RUN apt-get update && apt-get install -y --no-install-recommends qt5*

# Ros kinetic
RUN apt install ros-kinetic-libfranka ros-kinetic-franka-ros

# python3.6 & pytorch & tensorflow keras
RUN apt-get install -y python3.6 python3-pip
RUN pip3 install torch torchvision
RUN pip3 install tensorflow-gpu && pip3 install keras

# EnsensoSDK
RUN wget https://download.optonic.com/s/ensensosdk/download?files=ensenso-sdk-3.3.1417-x64.deb
RUN apt install ./download?files=ensenso-sdk-3.3.1417-x64.deb

# For uid, gid
ENV UNAME user
RUN useradd -m $UNAME
RUN apt-get update -qq && apt-get -y install gosu
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

RUN pip install numpy-quaternion

RUN git clone https://github.com/pantor/learning-pick-and-place.git && cd learning-pick-and-place &&\
    python3.6 -m pip install -r requirements.txt
RUN export PYTHONPATH=$PYTHONPATH:$./learning-pick-and-place/Documents/bin_picking/scripts

WORKDIR /opt/project

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
