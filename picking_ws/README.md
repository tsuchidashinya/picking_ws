# denso_simulator
**NOTE: This branch is created for Sugihara's graduation research. This branch is just remained for an asset and not to be mermged to the master branch.**

## Overview
ROS simulation related packages for Denso robots

## Install
See [denso_apps](https://github.com/Nishida-Lab/denso_apps)

## Usage

### Gazebo simulation

#### Topic, service, parameter
![rqt_graph](.img/denso_gazebo.png)

For more info, see [Gazebo Tutorial](http://gazebosim.org/tutorials/?tut=ros_comm#Tutorial:ROSCommunication)

#### VS087
```bash
roslaunch denso_gazebo vs087_gazebo.launch
```

#### VS087 with hand
For example,
```bash
roslaunch denso_gazebo vs087_with_mhand2_gazebo.launch
```

## CI
See [here](https://github.com/Nishida-Lab/denso_docs/tree/master/ci) for detail decumentation.

Replace the repository specific keywords in the above link as follows:
- `<your_repo>` -> `denso_simulator`
- `<your_pkg>` -> `denso_gazebo`
- `<your_rosinstall_dir>` -> `.`
