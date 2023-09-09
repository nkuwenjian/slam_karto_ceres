# slam_karto_ceres
A ROS package for 2-D pose graph SLAM using open karto package for the front-end and Google's Ceres solver for the back-end. This package has been tested on Ubuntu 16.04 and 18.04. Please install Ceres solver first following the official [installation tutorial](http://ceres-solver.org/installation.html#linux).

After that, please create and initialize a ROS workspace. We assume that your workspace is named catkin_ws. Then, run the following commands to clone and build open karto package:
```
$ cd ~/catkin_ws/src/
$ git clone https://github.com/ros-perception/open_karto.git
$ cd ../
$ catkin_make -DCMAKE_BUILD_TYPE=Release
```

After the above preparation, clone and build this package:
```
$ cd ~/catkin_ws/src/
$ git clone https://github.com/nkuwenjian/slam_karto_ceres.git
$ cd ../
$ catkin_make -DCMAKE_BUILD_TYPE=Release
```

Finally, run the following commands to launch Karto SLAM:
```
$ source ~/catkin_ws/devel/setup.bash
$ roslaunch slam_karto_ceres slam_karto_ceres.launch
```

Open a new terminal and play your rosbag:
```
$ rosbag play <rosbagfile> --clock
```

## Remarks
The source code of the SLAM back-end in this package refers to the [pose_graph_2d](https://ceres-solver.googlesource.com/ceres-solver/+/master/examples/slam/pose_graph_2d) in Ceres tutorials. It is recommended to read this tutorial to learn more about the use of Ceres.
