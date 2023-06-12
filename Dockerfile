FROM ros:noetic

RUN apt-get update && apt-get install -y python3-pip\
    ros-noetic-rqt-image-view \
    ros-noetic-tf-conversions \
    ros-noetic-cv-bridge\
    openssh-server\
    ffmpeg\
    libsm6\
    libxext6\
    sudo

RUN useradd -rm -d /localization -s /bin/bash -g root -G sudo -u 1000 localization-system

RUN  echo 'localization-system:autolab' | chpasswd

RUN service ssh start

WORKDIR /localization

COPY requirements.txt ./

RUN python3 -m pip install --upgrade pip && python3 -m pip install -r requirements.txt

COPY src /localization/src

RUN . /opt/ros/noetic/setup.sh && catkin_make

COPY launch ./launch

EXPOSE 22

RUN chmod +x launch/docker_launch.sh

CMD ["bash", "-c", "/localization/launch/docker_launch.sh"]





