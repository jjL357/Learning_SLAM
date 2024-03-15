# LOAM代码及运行配置
## 一、代码阅读
这部分我们学习scr中的四个代码

LOAM的源码主要分为scanRegistration提取特征点、laserOdometry 10HZ估计位姿、laserMapping 1HZ构建三维地图、transforMaintenance 位姿优化等四个模块
### 1. laser_mapping_node.cpp
### 2. laser_odometry_node.cpp
### 3.multi_scan_registration_node.cpp

### 4.transform_maintenance_node.cpp

## 七、实验验证
一开始使用了ubuntu22.04来配置环境运行，一开始没发现由于编译pcl时make导致swap空间爆了而倒是虚拟机差点出问题，后面增加swap空间成功编译pcl。

然后接着尝试了几个星期，发现最后到catkin_make的时候都会出现问题，得出结论ubuntu22.04采用的ros版本似乎和catkin_make所要用到的依赖包有冲突，所以放弃了ubuntu22.04

重新装了ubuntu18.04

按照参考链接:
https://blog.csdn.net/weixin_48924581/article/details/122461138

成功配置好了环境
(中间要注意虚拟机的磁盘和swap空间要够，不然好不容易快编译好了会出问题闪退，导致要重新来，我使用了make -j2来编译，花费的时间长了一点，但是不会出现swap空间不足的 问题)

配置完环境之后，运行:
![](./result_0.png)
可以看到只显示了路径，没有点云

后来查阅资料并和同学对比，成功的同学是装了双系统，可能是因为自己在虚拟机上做的，导致可能是其他问题，例如说rviz问题

然后参考链接:
https://blog.csdn.net/Pichai/article/details/123648573

输入命令:
```
export LIBGL_ALWAYS_SOFTWARE=1
```
关闭硬件加速

重新实验，得到正确结果:
![](./result.png)