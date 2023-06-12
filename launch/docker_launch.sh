#!/bin/bash

bash -c /usr/sbin/sshd -D

source /localization/devel/setup.bash

roslaunch extend_localization localization_docker.launch