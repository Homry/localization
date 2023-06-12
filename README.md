# Preparation
1. All cameras need to be calibrated
2. ```The data after calibration```, ```path to the camera```, ```markup markers coordinates``` and ```markers ids``` need to be placed into [config file](https://github.com/OSLL/autolab-extended-localization/blob/master/src/localization/config/cameraConfig.py) 

```
conf = {
    "$camera name$": {
        "camera_matrix": np.float32(),
        "dist_coefs": np.float32(),
        "videoPath": "path",
        "rectification_matrix": np.float32(),
        "projection_matrix": np.float32(),
        "markers_id": np.array(),
        "markers": np.array(),
    }
}
```

# Run system
## Run in Docker
To start the system in docker, you need to run the following command in the home foldes page of the repo:
```
docker-compose up
```

## local run
To start the system in local computer, you need to update [launch file](https://github.com/OSLL/autolab-extended-localization/blob/master/src/extend_localization/launch/localization_local.launch)
1. [Install ros-noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
2. update path to the map and config
3. In the src folder ```autolab_extend_localization/src``` need to run the following command
```
catkin_make
source devel/setup.bash
roslaunc extend_localization/launch/localozation_local.launch
```
