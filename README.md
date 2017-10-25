# PTAM-GS
PTAM source code using OpenCV,Eigen,Sophus,OpenGL,Pangolin libraries.

---

# 相关算法

## 颜色空间转换
* RGB 转 Gray : CV_BGR2GRAY

## 金字塔构建
* 高斯金字塔
* ４层
* 卷积核：５＊５

## FAST(Features from Accelerated Segment Test)
We saw several feature detectors and many of them are really good. But when looking from a real-time application point of view, they are not fast enough. One best example would be SLAM (Simultaneous Localization and Mapping) mobile robot which have limited computational resources.  
As a solution to this, FAST (Features from Accelerated Segment Test) algorithm was proposed by Edward Rosten and Tom Drummond in their paper “Machine learning for high-speed corner detection” in 2006 (Later revised it in 2010).   
检测**局部像素灰度**变化明显的地方。  
1. 在图像中选取像素p，假设它的亮度为Ip
2. 设置一个阈值Ｔ；
3. 以像素ｐ为中心，选取半径为３的圆上的１６个像素；
4. 假设选取的圆上有连续的Ｎ个点的亮度大于Ip+T或Ip-T，则该点ｐ可被认为是特征点（Ｎ通常取12，即为FAST-12，其他常用的Ｎ取值有９和１１，分别被成为FAST-9和FAST-11）；
5. 循环以上四步；  
![](https://docs.opencv.org/3.0-beta/_images/fast_speedtest.jpg)

## 非极大值抑制
non-maximal suppression
* FAST角点经常出现“扎堆”的情况，通过非极大值抑制，在一定区域内仅保留响应极大值的角点，避免角点集中的问题

## SSD(Sum of Squared Distance)

## Shi-Tomas

## 块匹配
1. 假设图像I1和图像I2，分别对应的角点为p1i和p2j，在图像I2角点中找到与图像I1对应的角点；
2.  以角点p1i为中心，在图像I1中提取9*9的像素块作为模板图像T1i；
3. 在图像I2中p1i点周围(以角点p1i为中心20*20的像素 范围)查找所有的角点p2jm(m<=n,n为该范围内角点数)；
4. 遍历所有的角点p2jm，以角点p2jm为中心，在图像I2中提取9*9的像素块，计算该像素块与T1i的SSD；
5. SSD最小对应的角点p2jm，即为图像I2中与图像I1中角点p1i对应的匹配角点；
6. 循环执行以上５步，查找图像I2中与图像I1对应的所有匹配角点；
