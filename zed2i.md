# zed2i相机使用

[yaml参数设置](https://blog.csdn.net/qq_41839222/article/details/86564879)
## 相机内参

这个内参在`/usr/local/zed/settings`中查看相机的内参，仅限于sdk

ros输出图像:内参在`rostopic echo /zed2i/zed_node/left/camera_info`中读取

==vins_mono需要的是T_imu_cam0==

不确定内参的准确程度？
需要标定？

1. zed相机的内参？找到对应的内参(在camera_info中)
2. 标定相机的内参？找到对应的内参和imu 的内参、联合标定

### 系统值

imu与相机：T_imu_cam0

```
 0.999945   0.0101402 -0.00272787      -0.002
 -0.0101334    0.999946  0.00249161      -0.023
 0.00275299 -0.00246383    0.999993     0.00022
          0           0           0           1
```
2k:
1104*621

vga: 640*360

左相机去畸变：
```
distortion_model: "plumb_bob"
D: [0.0, 0.0, 0.0, 0.0, 0.0]
K: [493.12091064453125, 0.0, 319.18212890625, 0.0, 493.12091064453125, 187.33084106445312, 0.0, 0.0, 1.0]
R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
P: [493.12091064453125, 0.0, 319.18212890625, 0.0, 0.0, 493.12091064453125, 187.33084106445312, 0.0, 0.0, 0.0, 1.0, 0.0]
```

左相机未去畸变：
```
D: [-0.059671301394701004, -0.06355620175600052, 0.0, 4.919200000585988e-05, -0.0002835890045389533]
K: [476.12249755859375, 0.0, 318.6650085449219, 0.0, 476.3175048828125, 179.2834930419922, 0.0, 0.0, 1.0]
```

右相机去畸变：
```
D: [0.0, 0.0, 0.0, 0.0, 0.0]
K: [493.12091064453125, 0.0, 319.18212890625, 0.0, 493.12091064453125, 187.33084106445312, 0.0, 0.0, 1.0]
```

右相机未去畸变：
```
D: [-0.05872559919953346, -0.06531070172786713, 0.0, 0.00012821599375456572, 1.0293600098520983e-05]
K: [476.2550048828125, 0.0, 321.7300109863281, 0.0, 476.5274963378906, 193.312255859375, 0.0, 0.0, 1.0]
```

left与imu:	`/zed2i/zed_node/left_cam_imu_transform `

```
translation: 
  x: -0.0020000000949949026
  y: -0.023000003769993782
  z: 0.0002200000308221206
rotation: 
  x: -0.0012388783507049084
  y: -0.001370235811918974
  z: -0.005068491213023663
  w: 0.9999854564666748
```



## 标定相机

### [camera_calibration相机标定](https://blog.csdn.net/TengYun_zhang/article/details/123072847)

```
rosrun camera_calibration cameracalibrator.py --approximate 0.1 --size 11x8 --square 0.01 right:=/zed2i/zed_node/right_raw/image_raw_gray left:=/zed2i/zed_node/left_raw/image_raw_gray right_camera:=/zed2i/zed_node/right_raw left_camera:=/zed2i/zed_node/left_raw --no-service-check

```



### KALIAR

[标定板](https://blog.csdn.net/xiaojinger_123/article/details/121232948)

降频率：

```
rosrun topic_tools throttle messages /zed2i/zed_node/left/image_rect_color 4.0 /zed2i/zed_node/left/image_rect_color2
rosrun topic_tools throttle messages /zed2i/zed_node/right/image_rect_color 4.0 /zed2i/zed_node/right/image_rect_color2
```



```
rosbag record -O Kalib_data_HD720_3.bag /zed2i/zed_node/imu/data_raw /zed2i/zed_node/left/image_rect_color2 /zed2i/zed_node/right/image_rect_color2
```



```
rosrun kalibr kalibr_calibrate_cameras --bag Kalib_data_HD720_3.bag --topics /zed2i/zed_node/left/image_rect_color2 /zed2i/zed_node/right/image_rect_color2 --models pinhole-radtan pinhole-radtan --target april.yaml 
```

```
rosrun kalibr kalibr_calibrate_imu_camera --bag Kalib_data_HD720_3.bag --cam Kalib_data_HD720_3-camchain.yaml --imu imu.yaml --target april.yaml --perform-synchronization
```





### 结果

960 x 540 `camera_calibration`:

```
Left:
D = [-0.046569611618025385, -0.09080741647257272, 0.004768983436670957, -0.0005447580108221106, 0.0]
K = [1069.1830134580994, 0.0, 419.0621954607083, 0.0, 1076.747229163823, 260.2587668067614, 0.0, 0.0, 1.0]
R = [0.960306779104331, -0.018603885714873522, 0.2783249637430562, 0.023012878517091738, 0.99965598879118, -0.012582189645915133, -0.2779951392176187, 0.01848782059184153, 0.9604045517703156]
P = [1147.6040903490864, 0.0, 34.80210876464844, 0.0, 0.0, 1147.6040903490864, 288.00843048095703, 0.0, 0.0, 0.0, 1.0, 0.0]

Right:
D = [-0.10305751320323894, 0.0758357733781336, 0.008429125835285328, -0.008029636212553648, 0.0]
K = [980.1283191521529, 0.0, 468.76620559518955, 0.0, 978.5849132664596, 313.4447584040264, 0.0, 0.0, 1.0]
R = [0.9515440904224765, -0.025742845425226004, 0.30643294517965075, 0.020871877324497114, 0.9995985025096898, 0.01916242462051106, -0.30680320845619113, -0.011838061065839454, 0.9516993493699513]
P = [1147.6040903490864, 0.0, 34.80210876464844, -153.86795478823035, 0.0, 1147.6040903490864, 288.00843048095703, 0.0, 0.0, 0.0, 1.0, 0.0]
self.T = [-0.12758070863933282, 0.003451537868607802, -0.041085781194962694]
self.R = [0.9995443632738519, -0.00250984302107413, -0.030079337351071104, 0.0015735333629075528, 0.9995146864256329, -0.031111342179736202, 0.030142824025392718, 0.03104983586878329, 0.9990632201479023]
None
# oST version 5.0 parameters


[image]

width
960

height
540

[narrow_stereo/left]

camera matrix
1069.183013 0.000000 419.062195
0.000000 1076.747229 260.258767
0.000000 0.000000 1.000000

distortion
-0.046570 -0.090807 0.004769 -0.000545 0.000000

rectification
0.960307 -0.018604 0.278325
0.023013 0.999656 -0.012582
-0.277995 0.018488 0.960405

projection
1147.604090 0.000000 34.802109 0.000000
0.000000 1147.604090 288.008430 0.000000
0.000000 0.000000 1.000000 0.000000
# oST version 5.0 parameters

```

**系统自带**960 x 540 

```
height: 540
width: 960
distortion_model: "plumb_bob"
D: [-0.059671301394701004, -0.06355620175600052, 0.0, 4.919200000585988e-05, -0.0002835890045389533]
K: [952.2449951171875, 0.0, 478.8299865722656, 0.0, 952.635009765625, 270.0669860839844, 0.0, 0.0, 1.0]
R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
P: [952.2449951171875, 0.0, 478.8299865722656, 0.0, 0.0, 952.635009765625, 270.0669860839844, 0.0, 0.0, 0.0, 1.0, 0.0]

```



## imu标定

### 运行

在其中的`launch`创建`zed_calibration.launch`

```xaml
<launch>
    <node pkg="imu_utils" type="imu_an" name="imu_an" output="screen">
        <param name="imu_topic" type="string" value= "/zed2i/zed_node/imu/data_raw"/> #自己的话题名
        <param name="imu_name" type="string" value= "ZED2"/> 
        <param name="data_save_path" type="string" value= "$(find imu_utils)/zed2i_data_raw/"/> #保存路径
        <param name="max_time_min" type="int" value= "120"/>
        <param name="max_cluster" type="int" value= "200"/>
    </node>
</launch>
```



在`imu_ws`中运行

```
source devel/setup.bash 
roslaunch imu_utils zed_calibration.launch 
```

在终端运行录制好的rosbag

```
rosbag play imu_calibration_2.bag -r 200
```



[allan_variance_ros标定](https://blog.csdn.net/kanhao100/article/details/130073156)

### 结果

sdk内置的[获取](https://blog.csdn.net/leesanity/article/details/121301317)

```
Sensor Type: Accelerometer
Max Rate: 400Hz
Range: [-78.48 78.48] m/s²
Resolution: 0.00239502 m/s²
Noise Density: 0.0016 m/s²/√Hz
Random Walk: 0.0002509 m/s²/s/√Hz
*****************************
Sensor Type: Gyroscope
Max Rate: 400Hz
Range: [-1000 1000] °/s
Resolution: 0.0305176 °/s
Noise Density: 0.007 °/s/√Hz
Random Walk: 0.0019474 °/s/s/√Hz
```

角度转弧度：2π / 360  = π / 180 ≈ 0.0174rad, 即: **度数 \* (π / 180） = 弧度**

```
Sensor Type: Gyroscope
Noise Density:1.2217e-4 rad/s/√Hz
Random Walk: 3.3989e-5 rad/s/s/√Hz
```





150hz `zed2i/zed_node/imu/data_raw`

```
Gyr:
   unit: " rad/s"
   avg-axis:
      gyr_n: 1.3711290412992481e-03
      gyr_w: 2.9574346929954821e-06
Acc:
   unit: " m/s^2"
   avg-axis:
      acc_n: 1.2186617835411724e-02
      acc_w: 3.5823993886502608e-04
```

150hz `zed2i/zed_node/imu/data`

```
Gyr:
   unit: " rad/s"
   avg-axis:
      gyr_n: 1.3450919746321472e-03
      gyr_w: 3.7309709869607166e-06
Acc:
   unit: " m/s^2"
   avg-axis:
      acc_n: 1.2184457833156670e-02
      acc_w: 3.5810686043004662e-04
```

结论:两个的噪声测量一样，网上说imu_util测量不准？

chatgpt:

在ZED相机的ROS节点中，imu和imu_raw是两个不同的话题。

imu话题提供了一个已经经过陀螺仪和加速度计校准后的惯性测量单元（IMU）的输出数据。这些数据包括：角速度、线性加速度以及使用矩阵旋转方式表示的姿态估计信息。该话题发布的消息类型为“sensor_msgs/Imu”。

而imu_raw话题则提供了未经校准的原始IMU数据。这些数据包括三个轴上的陀螺仪和加速度计输出，以及传感器的温度。这些原始数据可以被用于进行自定义的IMU校准和处理。该话题发布的消息类型为“sensor_msgs/Imu”。

因此，imu和imu_raw的区别在于它们提供的IMU数据是否经过校准处理。如果需要直接使用IMU数据进行姿态估计等操作，则应该使用imu话题；如果需要进行自定义的IMU校准和处理，则应该使用imu_raw话题。



150hz,`allan_variance_ros`测量

```
#Accelerometer
accelerometer_noise_density: 0.0011874331491394511 
accelerometer_random_walk: 0.0002024129386619518 

#Gyroscope
gyroscope_noise_density: 0.00012655300240696358 
gyroscope_random_walk: 6.94234736045084e-07 
```





## imu与相机标定

离散值：200hz
```
gyr_n: 1.0219614587222572821604537989924e-4
gyr_w: 7.6635418027197093851094806893575e-7
acc_n: 7.9441311864484361289875703915164e-4
acc_w: 4.3264320844772316716792844448505e-5
```
150hz(有噪声)
```
连续值
  gyr_n: 1.3592758921806569e-03
  gyr_w: 5.6174156884021636e-06
  acc_n: 1.2141858141114856e-02
  acc_w: 3.6250666406610778e-04
间断：
  gyr_n: 1.1098e-4
  gyr_w: 4.586600e-7
  acc_n: 9.91378e-4
  acc_w: 2.95985e-5
```

重力加速度：

 x: -1.2223907709121704
 y: 0.07208067923784256
 z: 9.688394546508789

### kalibr标定

[使用](https://blog.csdn.net/i_robots/article/details/114747281)

[ZED 2i 双目-IMU标定](https://blog.csdn.net/weixin_40599145/article/details/126899201)

命令行：相机标定

```
rosrun camera_calibration cameracalibrator.py --approximate 0.1 --size 8x6 --square 0.02 right:=/zed2i/zed_node/right/image_rect_color left:=/zed2i/zed_node/left/image_rect_color right_camera:=/zed2i/zed2_node/right left_camera:=/zed2i/zed_node/left --no-service-check
```

640*360：标定出的T_cam0_imu

```
 data: 
 [0.0034775113040986516, -0.999992310758429, 0.0018125504538369364, 
  -0.013748583270139614, -0.0018602011593337464, -0.9999037534231539,
  0.9998994366301015, 0.003452256604693782, -0.01375494642452113]
t:
[0.02368744604971605, -0.00068476918544764, -0.018235916091044788]
```

==imu与相机的坐标系不对，需要乘以个变换矩阵==





kalib标定出来的结果

```
  [0.0034775113040986516, -0.999992310758429, 0.0018125504538369364, 0.02368744604971605]
  [-0.013748583270139614, -0.0018602011593337464, -0.9999037534231539, -0.00068476918544764]
  [0.9998994366301015, 0.003452256604693782, -0.01375494642452113, -0.018235916091044788]
  [0.0, 0.0, 0.0, 1.0]
```






重新标定的结果：T_cam_imu
```
[0.0036704711759193986, -0.9999879305720811, 0.0032659380753689016, 
-0.013688950414021683, -0.003315899025666813, -0.9999008038051638,
0.9998995650953689, 0.003625399814776553, -0.013700956108017415]
t:
0.02158166123076754
0.0017499993714911969
-0.024531144782777767

T_imu_cam:
0.00367047  -0.999988 0.00326594 
 -0.013689 -0.0033159  -0.999901    
    0.9999  0.0036254  -0.013701 
    
 0.0215817,0.00175,-0.0245311
```

2023.4.18

相机的标定不太准确，需要重新标定

ros:info的旋转参数不对

出来的是T_imu_cam的参数，需要求逆，参数不太对，坐标系不不太对

去了解sdk的标定

去了解相机的联合标定



[参考链接](https://blog.csdn.net/heyijia0327/article/details/83583360)

```
两种都可以，但是要注意使用方式。通常大家都是用没有去畸变的图像和 imu 一起标定外参数，这时候标定的外参数是不能用来和rectify后的图像一起做vio的。因为rectify的图像是在畸变图像上还会加一个微小的旋转，即畸变图像和imu之间的外参数 和 rectify 图像跟imu之间的外参数是不一样的。
```





## kalibra

```
rosbag record -O Kalib_data_HD720.bag /zed2i/zed_node/imu/data_raw /zed2i/zed_node/left/image_rect_color /zed2i/zed_node/right/image_rect_color
```



## vins_mono

运行命令：
```
roslaunch zed_wrapper zed2i.launch
roslaunch vins_estimator realsense_color.launch 
roslaunch vins_estimator vins_rviz.launch
```
出现漂移，估计是imu的参数问题



## 计划

20230415： 

相机标定弄好，imu标定



20230420:  

小结：问题：配置文件：现在相机：15hz imu:150hz

`imu/data`的效果好于`imu/data_raw`,原因可能是去噪了



