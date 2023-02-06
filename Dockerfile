FROM ros:noetic

WORKDIR /localization

COPY requirements.txt ./
COPY src /localization/src

RUN apt-get update && apt-get install -y python3-pip

RUN python3 -m pip install -r requirements.txt

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && catkin_make
RUN ./devel/setup.sh
CMD ["roslaunch", "localization", "all.launch config_path:=./src/localization/config/cameraConfig.py map_path:=./src/map/assets/autolab.jpeg"]
