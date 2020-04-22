## Rviz Tools for python

Some useful tools for using ROS Rviz with python.  
Currently this package contains some functions that make it easy to publish Rviz Markers in a pythonic way.

Python code by David Butterworth.  
Based on Dave Coleman's C++ library [RvizVisualTools](https://github.com/davetcoleman/rviz_visual_tools)

This package been tested on Ubuntu 14.04 with ROS Indigo.


**Usage examples:**

The normal way to publish an Rviz Marker is by setting its properties using ROS message types:
```
P = Pose(Point(0,0,0),Quaternion(0,0,0,1))
scale = Vector3(1.1,0.2,0.8)
markers.publishCube(P, 'red', scale)
```

Alternatively you can publish a Marker using a numpy matrix, with parameters specified as floats, tuples and lists:
```
T = transformations.translation_matrix((1,1,1))
rgb_color = [0,1,0]
cube_width = 0.5
markers.publishCube(T, rgb_color, cube_width)
```


**Demo:**

To run the example code:  
```
$ rosrun rviz_tools_py demo.py
```

![Demo markers](https://raw.github.com/DavidB-CMU/rviz_tools_py/master/demo_markers1.png)
