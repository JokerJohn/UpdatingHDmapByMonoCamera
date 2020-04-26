# 可视化模块

## rviz_project_line_to_hdmap

### 功能

把车道线检测结果动态投影到HDMap上

### 效果

蓝色为HDMap车道线, 白色点云为车道线检测结果

![](figure/res.gif)

### 运行

中间忽略一些的基本操作

1. 把本功能包放到工作空间中编译

2. `rosrun rviz_project_line_to_hdmap demo.py`
3. 打开rviz, 注意需要调整下点云大小, 从而达到最好的可视化效果

## tftree_veihcle2world

### 功能

读取车辆GPS信息, 用tf树, 实现车辆与世界坐标系的动态变化

### 效果

![](figure/tree.gif)

### 运行

编译这个package

rosrun gps_folw auto_follow.py

打开rviz

## test_pcl

pcl可视化案例
