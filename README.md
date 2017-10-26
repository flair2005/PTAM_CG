<!DOCTYPE html>  
<html>
<script type="text/javascript" src="http://cdn.mathjax.org/mathjax/latest/MathJax.js?config=default"></script>  

# PTAM-GS
PTAM source code using OpenCV,Eigen,Sophus,OpenGL,Pangolin libraries.

---

# 数字图像处理

## 颜色空间转换
1. RGB to Gray (CV_BGR2GRAY)  
$$
Y = 0.299*R + 0.587*G + 0.114*B
$$

## 图像金字塔
* 均值金字塔：2*2邻域均值滤波
* 高斯金字塔：向下降采样图像(4层)，高斯核5*5
* 拉普拉斯金字塔

## FAST Algorithm for Corner Detection
* [FAST Algorithm for Corner Detection](https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_feature2d/py_fast/py_fast.html)    

### Features from Accelerated Segment Test   
* 检测**局部像素灰度**变化明显的地方    
1. 在图像中选取像素p，假设它的亮度为Ip
2. 设置一个阈值Ｔ；
3. 以像素p为中心，选取半径为3的圆上的16个像素；
4. 假设选取的圆上有连续的Ｎ个点的亮度大于Ip+T或Ip-T，则该点ｐ可被认为是特征点（Ｎ通常取12，即为FAST-12，其他常用的Ｎ取值有9和11，分别被成为FAST-9和FAST-11）；
5. 循环以上四步；  
![](https://docs.opencv.org/3.0-beta/_images/fast_speedtest.jpg)

### Non-maximal Suppression
* FAST角点经常出现“扎堆”的情况，通过非极大值抑制，在一定区域内仅保留响应极大值的角点，避免角点集中的问题

## Shi-Tomas
* [Harris corner detector](https://docs.opencv.org/2.4/doc/tutorials/features2d/trackingmotion/harris_detector/harris_detector.html)
* [Shi-Tomasi Corner Detector & Good Features to Track](https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_feature2d/py_shi_tomasi/py_shi_tomasi.html)
1. 根据Harris角点检测方法，构建梯度协方差矩阵M
$$
\begin{aligned}
M =  
\left[\begin{array}{cc}
{I_x}^2&I_xI_y\\I_xI_y&{I_y}^2
\end{array}\right]  
\end{aligned}
$$
2. 求矩阵M的特征值
$$
det(\lambda E - M) = 0
$$
3. Shi-Tomas分数即为最小特征值
$$
R=min(\lambda_1,\lambda_2)
$$

## SSD
Sum of Squared Distance
$$ {D(I_A,I_B)}_{SSD} = \sum_{x,y}[{I_A}_{(x,y)}-{I_B}_{(x,y)}]^2  $$  

## 块匹配
1. 假设图像I1和图像I2，分别对应的角点为p1i和p2j，在图像I2角点中找到与图像I1对应的角点；
2.  以角点p1i为中心，在图像I1中提取9*9的像素块作为模板图像T1i；
3. 在图像I2中p1i点周围(以角点p1i为中心20*20的像素 范围)查找所有的角点p2jm(m<=n,n为该范围内角点数)；
4. 遍历所有的角点p2jm，以角点p2jm为中心，在图像I2中提取9*9的像素块，计算该像素块与T1i的SSD；
5. SSD最小对应的角点p2jm，即为图像I2中与图像I1中角点p1i对应的匹配角点；
6. 循环执行以上5步，查找图像I2中与图像I1对应的所有匹配角点；

# Camera
## Parameters
* $f_x,f_y$ : focal legth
* $c_x,c_y$ : principal point
* $\omega$ : radial distortion, the **FOV-model**

## Project
$$ r = \sqrt{{x_c}^2+{y_c}^2} $$
$$ r'= \frac{1}{\omega}arctan(2r tan(\frac{\omega}{2})) $$
$$
\begin{aligned}
\left[\begin{array}{c}u\\v\end{array}\right] =  
\left[\begin{array}{c}c_x\\c_y\end{array}\right] +
\left[\begin{array}{cc}f_x&0\\0&f_y\end{array}\right]  
\frac{r'}{r}
\left[\begin{array}{c}x_c\\y_c\end{array}\right]
\end{aligned}
$$

## UnProject
$$
\begin{aligned}
\left[\begin{array}{c}{x_c}'\\{y_c}'\end{array}\right] =
\left[\begin{array}{cc}f_x&0\\0&f_y\end{array}\right]^{-1}
\lgroup
\left[\begin{array}{c}u\\v\end{array}\right] -  
\left[\begin{array}{c}c_x\\c_y\end{array}\right]
\rgroup
\end{aligned}
$$
$$ r' = \sqrt{ {{x_c}'}^2+{{y_c}'}^2 } $$
$$ r = \frac{tan(\omega r')}{2tan\frac{\omega}{2}} $$
$$
\begin{aligned}
\left[\begin{array}{c}x_c\\y_c\end{array}\right] =
\frac{r}{r'}
\left[\begin{array}{c}{x_c}'\\{y_c}'\end{array}\right]
\end{aligned}
$$

## Derivative of Projection
the derivative of image frame wrt camera z=1 frame at the last computed projection
$$
\begin{aligned}
J_{projection} =
\left[\begin{array}{cc}\frac{du}{dx_c}&\frac{du}{dy_c}\\\frac{dv}{dx_c}&\frac{dv}{dy_c}\end{array}\right]  
\end{aligned}
$$
</html> 
