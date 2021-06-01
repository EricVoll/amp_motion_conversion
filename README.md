# AMP Motion Conversion

This repository hosts a ros package that takes AMP motion data, converts it to a ROS-urdf-compatable format (spherical joints to 3 revolute joints, handling the quaternions etc.) and then stores the trajectory as jsons.

It also contains a ROS-valid urdf of the humanoid used in the original AMP paper (inertias are not properly calculated yet).

To use it you will have to have ros installed and an existing catkin_ws:

```
$ cd catkin_ws/src
$ git clone git@github.com:EricVoll/map_motion_conversion
$ source ../devel/setup.bash
$ roslaunch amp_motion_conversion show.launch
```

The default launch file will open rviz, play one of the provided motions and then store the motion as a json.
The xacro and urdf of the humanoid are in `/amp_motion_conversion/urdf`