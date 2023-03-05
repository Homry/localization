FROM ros:noetic

RUN apt-get update && apt-get install -y python3-pip ros-noetic-tf-conversions

WORKDIR /localization

COPY requirements.txt ./

RUN python3 -m pip install --upgrade pip && python3 -m pip install -r requirements.txt

COPY src /localization/src

RUN . /opt/ros/noetic/setup.sh && catkin_make

COPY launch ./launch

EXPOSE 25468

RUN chmod +x launch/docker_launch.sh

ENTRYPOINT ["bash", "-c", "launch/docker_launch.sh"]





