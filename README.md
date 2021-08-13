## Updating HDmap by Mono Camera and GPS

![1566396892077](http://xchu.net/2019/08/21/Deecamp%E9%A1%B9%E7%9B%AE%E6%A6%82%E8%BF%B0/1566396892077.png)

## 项目概述

​		此项目源于Deecamp 2019北京训练营，目前整理完代码并开源出来。此项目是一种利用单目相机+GPS RTK更新高精地图的低成本方案，可以拓展至视觉车道线重建。由于训练营时间以及分组限制，目前开源的代码是二组的，只完成了车道线更新部分，地标以及道路标志牌红绿灯的更新等都未完成。感谢滴滴和创新工场提供的平台！

​		目前的代码都是直接传上来的，可能无法直接run，后面会慢慢整理各模块，并给出详细的使用说明。项目里面用到的一些算法大部分都比较粗略，大佬请略过。开源本项目的目的只是为了提供一种思路，感兴趣的可提交代码。

![image-20210813103307246](visualization/README/res.gif)

## 项目思路

[博客 : 《基于车载摄像头的高精地图创建与更新》](http://www.xchu.net/2019/08/21/Deecamp%E9%A1%B9%E7%9B%AE%E6%A6%82%E8%BF%B0/)

![1566396918685](http://xchu.net/2019/08/21/Deecamp%E9%A1%B9%E7%9B%AE%E6%A6%82%E8%BF%B0/1566396918685.png)

![1566397109906](http://xchu.net/2019/08/21/Deecamp%E9%A1%B9%E7%9B%AE%E6%A6%82%E8%BF%B0/1566397109906.png)

![1566397187944](http://xchu.net/2019/08/21/Deecamp%E9%A1%B9%E7%9B%AE%E6%A6%82%E8%BF%B0/1566397187944.png)

![1566397284210](http://xchu.net/2019/08/21/Deecamp%E9%A1%B9%E7%9B%AE%E6%A6%82%E8%BF%B0/1566397284210.png)

## 项目成员

| 高校                   | 成员                                                         |
| :--------------------- | :----------------------------------------------------------- |
| 北京大学               | [林中亚](https://github.com/daniallin)                       |
| 杜克大学               | [张景](https://github.com/zhangjing1997)                     |
| 北京航空航天大学       | [王友辰](https://github.com/yohoochen)、[xchu](https://github.com/JokerJohn) |
| 北京理工大学           | [刘丰刚](https://github.com/LiuFG)、[林瀚熙]()               |
| 中国科学技术大学       | [柯延杰](https://github.com/USTC-Keyanjie)                   |
| 哈尔滨工业大学         | [刘欢锂]()                                                   |
| 中科院上海微电子研究所 | [姜昊辰](https://github.com/jhch1995)                        |
| 中国科学院大学         | [陈卓]()                                                     |
| 山东大学               | [程大海](https://github.com/DaHaiHuha)                       |

## 工程结构

```
-hdmap  主工程目录
-dataset  数据目录
-detection 视觉检测模块
-geos_parse geos空间计算
-transform  坐标转换和深度估计
-visualization  rviz以及pcl可视化
```



