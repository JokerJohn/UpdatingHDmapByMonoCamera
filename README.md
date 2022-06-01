## Updating HDMap by Mono Camera and RTK-GPS

![1566396892077](http://xchu.net/2019/08/21/Deecamp%E9%A1%B9%E7%9B%AE%E6%A6%82%E8%BF%B0/1566396892077.png)

# Introduction

 This project originated from the Beijing Deecamp 2019. At present, the code of some modules has been sorted out and open sourced. This project involves some dataset that are too large and confidential, so the whole project is difficult to run through, but the ideas can be referenced.
This project is a low-cost solution to update high-definition maps using a monocular camera  and RTK GPS. Due to the training camp time and group restrictions, the current open source code is  from group 2.  Only the update module of the lane line has been completed, and the update of landmarks and road signs and traffic lights has not been completed. 

![image-20210813103307246](visualization/README/res.gif)

# Ideas

will be added later！

[博客 : 《基于车载摄像头的高精地图创建与更新》](http://www.xchu.net/2019/08/21/Deecamp%E9%A1%B9%E7%9B%AE%E6%A6%82%E8%BF%B0/)

![1566396918685](http://xchu.net/2019/08/21/Deecamp%E9%A1%B9%E7%9B%AE%E6%A6%82%E8%BF%B0/1566396918685.png)

![1566397109906](http://xchu.net/2019/08/21/Deecamp%E9%A1%B9%E7%9B%AE%E6%A6%82%E8%BF%B0/1566397109906.png)

![1566397187944](http://xchu.net/2019/08/21/Deecamp%E9%A1%B9%E7%9B%AE%E6%A6%82%E8%BF%B0/1566397187944.png)

![1566397284210](http://xchu.net/2019/08/21/Deecamp%E9%A1%B9%E7%9B%AE%E6%A6%82%E8%BF%B0/1566397284210.png)

## Project Members

| University                                                   | Name                                                         |
| :----------------------------------------------------------- | :----------------------------------------------------------- |
| Peking University                                            | [Zhongya Lin](https://github.com/daniallin)                       |
| Duke University                                              | [Jing Zhang](https://github.com/zhangjing1997)                     |
| Beihang University                                           | [YOuchen Wang](https://github.com/yohoochen)、[Xiangcheng Hu](https://github.com/JokerJohn) |
| Beijing Institute of Technology                              | [Fenggang Liu](https://github.com/LiuFG)、[Hanxi Lin]()               |
| University of Science and Technology of China                | [Yanjie Ke](https://github.com/USTC-Keyanjie)                   |
| Harbin Institute of Technology                               | [Huanli Liu]()                                                   |
| Shanghai Institute of Microelectronics, Chinese Academy of Sciences | [Haocheng Jiang](https://github.com/jhch1995)                        |
| Chinese Academy of Sciences University                       | [Zhuo Chen]()                                                     |
| Shan Dong University                                         | [Dahai Cheng](https://github.com/DaHaiHuha)                       |

# Structure

- `hdmap` : main project directory
- `dataset` : data directory
- `detection`: visual detection module
- `geos_parse` : geos spatial calculation
- `transform` : coordinate transformation and depth estimation
- `visualization`: rviz and pcl visualization

# Dependences

- [libgeos](https://github.com/libgeos/geos.git)

- [protobuf](https://github.com/protocolbuffers/protobuf.git)

# Acknowledgments

Thanks to the  [Sinovation Ventures)](https://www.chuangxin.com/)!

Thanks to all team members for their hard work！
