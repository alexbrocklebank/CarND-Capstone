FROM nvidia/cuda:8.0-cudnn6-runtime-ubuntu16.04

# setup keys
RUN apt-key adv --keyserver ha.pool.sks-keyservers.net --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list

# install bootstrap tools
# RUN apt-get update && \
#    apt-get install --no-install-recommends -y \
#    python-rosdep \
#    python-rosinstall \
#    python-vcstools && \
#    rm -rf /var/lib/apt/lists/*

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list' && \
  apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116 && \
  apt-get update && \
  apt-get install -y \
    python-pip=8.1.1-2ubuntu0.4 \
    python-vcstools \
    rviz mesa-utils \
    ros-kinetic-desktop-full && \

# cleanup
  apt-get clean && \
  rm -rf \
          /tmp/* \
          /var/lib/apt/lists/* \
          /var/tmp/*

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
#$ export LANGUAGE=en_US.UTF-8
#$ export LANG=en_US.UTF-8
#$ export LC_ALL=en_US.UTF-8
#$ locale-gen en_US.UTF-8
#$ dpkg-reconfigure locales

# bootstrap rosdep
RUN rosdep init && \
    rosdep update

# install ros packages
# ENV ROS_DISTRO kinetic
# RUN apt-get update && \
#    apt-get install -y ros-kinetic-ros-core=1.3.1-0* && \
#    # Including kinetic perception
#    apt-get install -y ros-kinetic-perception && \
#    rm -rf /var/lib/apt/lists/*

# SDC Capstone project related
RUN apt-get -y update && \
    apt-get -y install wget sudo && \
    apt-get -y -o Dpkg::Options::="--force-confmiss" install --reinstall netbase && \
#    pip install --upgrade pip && \
    pip install  pylint "Flask>=0.11.1" "attrdict>=2.0.0" "eventlet>=0.19.0" \
                 "python-socketio>=1.6.1" "numpy>=1.13.1" "Pillow>=2.2.1" scipy \
                 tensorflow-gpu keras h5py && \
    rm -rf /var/cache/apt/archives/.*deb

# SDC Capstone project related
RUN wget -q  https://bitbucket.org/DataspeedInc/dbw_mkz_ros/raw/default/dbw_mkz/scripts/sdk_install.bash && \
    bash < sdk_install.bash && \
    rm sdk_install.bash && \
    rm -rf /var/cache/apt/archives/.*deb

RUN export LD_LIBRARY_PATH="/usr/local/cuda/lib64:$LD_LIBRARY_PATH" && \
    ln -s /usr/local/cuda/lib64/libcudnn.so.6 /usr/local/cuda/lib64/libcudnn.so

# install required ros dependencies
# RUN apt-get install -y ros-$ROS_DISTRO-cv-bridge
# RUN apt-get install -y ros-$ROS_DISTRO-pcl-ros
# RUN apt-get install -y ros-$ROS_DISTRO-image-proc

# socket io
RUN apt-get install -y netbase

# nvidia-docker hooks
LABEL com.nvidia.volumes.needed="nvidia_driver"
ENV PATH /usr/local/nvidia/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}

RUN mkdir /capstone
VOLUME ["/capstone"]
VOLUME ["/root/.ros/log/"]
WORKDIR /capstone/ros
