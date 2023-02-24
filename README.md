# AMP Motion Conversion

This repository hosts a ros package that takes AMP motion data from the [AMP paper](https://github.com/xbpeng/DeepMimic), converts it to a ROS-urdf-compatable format (spherical joints to 3 revolute joints, handling the quaternions etc.) and then stores the trajectory as jsons.

All files within `cfg/motions/` that are not in a `converted` folder are not my work and are copied from [the DeepMimic repo](https://github.com/xbpeng/DeepMimic). 

It also contains a ROS-valid urdf of the humanoid used in the original AMP paper (inertias are not properly calculated yet).

To use it you will have to have ros installed and an existing catkin_ws:

```
$ cd catkin_ws/src
$ git clone git@github.com:EricVoll/map_motion_conversion
$ catkin build or catkin_make (whatever you use)
$ source ../devel/setup.bash
$ roslaunch amp_motion_conversion show.launch
```

The default launch file will open rviz, play one of the provided motions and then store the motion as a json.
The xacro and urdf of the humanoid are in `/amp_motion_conversion/urdf`

AMP motion data in Rviz
![image](https://user-images.githubusercontent.com/28082576/120357868-3026c780-c306-11eb-83e3-12628830cafe.png)
