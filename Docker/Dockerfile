FROM nvidia/opengl:1.0-glvnd-runtime-ubuntu20.04

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

########################
### INSTALL PACKAGES ###
########################
RUN apt-get update && apt-get install -y --no-install-recommends \
    wget \
    curl \
    git \
    vim \
    python3-pyqt5 \
    python3-pip \
    lsb-release \
    gnupg

# install ros noetic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get update && apt-get install -y \
    ros-noetic-desktop-full

RUN apt-get update && apt-get install -y \
    ros-noetic-joy \
    ros-noetic-gazebo* \
    ros-noetic-moveit* \
    ros-noetic-image-view* \
    ros-noetic-cv-camera* \
    ros-noetic-joint* \
    ros-noetic-robot-state* \
    ros-noetic-graph* \
    ros-noetic-ros-controller* \
    ros-noetic-joy-teleop* \
    ros-noetic-eigen* \
    ros-noetic-rosbridge-server* \
    ros-noetic-geometric* \
    ros-noetic-object-recognition* \
    ros-noetic-map-server* \
    ros-noetic-warehouse-ros* \
    ros-noetic-geodesy \
    ros-noetic-controller-interface \
    ros-noetic-effort-controllers \
	ros-noetic-vision-opencv

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-rosinstall \
    python3-rosdep \
    python3-catkin-tools \
    python

RUN mkdir -p /root/catkin_ws/src && \
    cd /root/catkin_ws && \
    rosdep init

ENV USERNAME=morita
RUN useradd --create-home --shell /bin/bash ${USERNAME}

RUN echo 'export ROS_IP=`hostname -I | cut -d' ' -f1`'  >> /root/.bashrc
RUN echo 'export ROS_MASTER_URI=http://localhost:11311'  >> /root/.bashrc

#################################################################################################################################################################

#####################
### INSTALL CMAKE ###
#####################
# Reference: https://apt.kitware.com/

RUN git clone https://gitlab.kitware.com/cmake/cmake.git \
	&& cd cmake \
	&& git checkout tags/v3.16.3 \
	&& ./bootstrap --parallel=8 \
	&& make -j8 \
	&& make install \
	&& cd .. \
	&& rm -rf cmake

##################	
### EXTRA DEPS ###
##################
RUN pip3 install catkin_pkg \
	&& pip3 install "pybind11[global]" \
	&& pip3 install pyyaml \
	&& pip3 install empy \
	&& wget -q --show-progress --progress=bar:force https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.tar.gz \
	&& tar -xzf eigen-3.3.7.tar.gz \
	&& cd eigen-3.3.7 \
	&& mkdir build && cd build \
	&& cmake .. \
	&& make install \
	&& cd ../.. \
	&& rm eigen-3.3.7.tar.gz

#########################
### INSTALL LIBFRANKA ###
#########################
RUN git clone --recursive https://github.com/frankaemika/libfranka.git \
	&& cd libfranka \
	&& git submodule update \
	&& mkdir build && cd build \
	&& cmake -DCMAKE_BUILD_TYPE=Release .. \
	&& cmake --build .

######################################
### INSTALL LIBFRANKA ROS PACKAGES ###
######################################
RUN apt install -y ros-noetic-libfranka ros-noetic-franka-ros
RUN mkdir -p /root/catkin_ws/src && cd /root/catkin_ws/src \
	&& git clone --recursive https://github.com/frankaemika/franka_ros \
	&& cd /root/catkin_ws/ \
	&& apt-get update \
	&& rosdep update \
	&& rosdep install --from-paths src --ignore-src --rosdistro noetic -y --skip-keys libfranka \
	&& /bin/bash -c ". /opt/ros/noetic/setup.sh; catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/home/Workspace/libfranka/build" \
	&& echo "source /home/Workspace/catkin_ws/devel/setup.sh" >> ~/.bashrc \
	&& /bin/bash -c "source ~/.bashrc"

#########################
### INSTALL REALSENSE ###
#########################
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

RUN apt-get update \
	&& apt-get install -y software-properties-common

RUN apt-get update \
	# && apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE \
	&& apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE \
	&& add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u \
	&& apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 6F3EFCDE \
	&& apt-get update \
	# Install the librealsense, Development packages & and other needed packages
	&& apt-get install -y \
	librealsense2-dkms \
	librealsense2-utils \
	librealsense2-dev  \
	librealsense2-dbg \
	# Upgrade the local packages 
	&& apt-get update && apt-get --only-upgrade install -y librealsense2-utils librealsense2-dkms

# Create catkin workspace and clone realsense git repository
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc \
	&& /bin/bash -c "source ~/.bashrc" \
	&& cd /root/catkin_ws/src \
	&& git clone https://github.com/IntelRealSense/realsense-ros.git \
	&& git clone https://github.com/pal-robotics/ddynamic_reconfigure.git \
	&& cd realsense-ros

# Build the workspace from the repository
RUN /bin/bash -c "source ~/.bashrc" \
	&& cd /root/catkin_ws \
	&& rosdep update \
	&& rosdep install -r --from-paths src --ignore-src --rosdistro=noetic -y --skip-keys libfranka \
	&& /bin/bash -c ". /opt/ros/noetic/setup.sh; catkin_make clean" \
	&& /bin/bash -c ". /opt/ros/noetic/setup.sh; catkin_make --cmake-args -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release" \
	&& /bin/bash -c ". /opt/ros/noetic/setup.sh; catkin_make install" 
	
RUN echo "source /home/Workspace/catkin_ws/devel/setup.bash" >> ~/.bashrc \
	&& /bin/bash -c "source ~/.bashrc"

##############################
### CLONE THE MAIN PROJECT ###
##############################
#RUN mkdir -p catkin_ws/src/learning-pick-and-place
#ENV ROS_MASTER_URI=http://10.224.60.100:11311
#ENV ROS_IP=10.224.60.100

RUN pip3 install rospkg

###################
### ENSENSO SDK ###
###################
RUN cd .. && mkdir Downloads && cd Downloads \
	&& wget -q --show-progress --progress=bar:force https://download.ensenso.com/s/ensensosdk/download?files=ensenso-sdk-2.3.1536-x64.deb \
	&& mv download?files=ensenso-sdk-2.3.1536-x64.deb ensenso-sdk-2.3.1536-x64.deb \
	&& dpkg -i ensenso-sdk-2.3.1536-x64.deb
RUN echo 'export ENSENSO_INSTALL=/opt/ensenso' >> /root/.bashrc

# RUN wget https://raw.githubusercontent.com/dti-research/panda_moveit_config/noetic-devel/config/panda_arm.xacro

RUN apt-get update && apt-get install -y \
  	libglib2.0-0  \
  	libgl1-mesa-glx \
  	xcb \
  	libx11-xcb-dev \
  	libglu1-mesa-dev \
  	libxrender-dev \
  	libxi6 \
  	libdbus-1-3 \
  	libfontconfig1 \
  	xvfb \
  	&& rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends qt5*
RUN mkdir -p /root/workspace/CoppeliaSim_Edu_V4_4_0_rev0_Ubuntu20_04
COPY ./CoppeliaSim_Edu_V4_4_0_rev0_Ubuntu20_04 /root/workspace/CoppeliaSim_Edu_V4_4_0_rev0_Ubuntu20_04
RUN mkdir /run/user/0
ENV XDG_RUNTIME_DIR=/run/user/0
ENV COPPELIASIM_ROOT=/root/workspace/CoppeliaSim_Edu_V4_4_0_rev0_Ubuntu20_04
ENV LD_LIBRARY_PATH=$COPPELIASIM_ROOT:$LD_LIBRARY_PATH
ENV QT_QPA_PLATFORM_PLUGIN_PATH=$COPPELIASIM_ROOT
RUN echo 'export COPPELIASIM_ROOT=/root/workspace/CoppeliaSim_Edu_V4_4_0_rev0_Ubuntu20_04' >> /root/.bashrc
RUN echo 'export LD_LIBRARY_PATH=$COPPELIASIM_ROOT:$LD_LIBRARY_PATH' >> /root/.bashrc
RUN echo 'export QT_QPA_PLATFORM_PLUGIN_PATH=$COPPELIASIM_ROOT' >> /root/.bashrc
RUN echo "alias coppeliasim='/root/workspace/CoppeliaSim_Edu_V4_4_0_rev0_Ubuntu20_04/coppeliaSim.sh'" >> /root/.bashrc


RUN echo 'export COPPELIASIM_ROOT_DIR=/root/workspace/CoppeliaSim_Edu_V4_4_0_rev0_Ubuntu20_04' >> /root/.bashrc
RUN pip3 install xmlschema
RUN cd /root/catkin_ws/src \
	&& git clone -b coppeliasim-v4.4.0-rev0  https://github.com/CoppeliaRobotics/simExtROS.git sim_ros_interface
#COPY /root/catkin_ws/devel/lib/libsimExtROS.so /root/workspace/CoppeliaSim_Edu_V4_4_0_rev0_Ubuntu20_04/

RUN git clone https://github.com/stepjam/PyRep.git && cd PyRep &&\
    pip install -r requirements.txt && pip install .
RUN pip install numpy-quaternion


