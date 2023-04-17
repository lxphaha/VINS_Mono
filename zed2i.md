# zed2i相机使用

## 相机内参

这个内参在`/usr/local/zed/settings`中查看相机的内参，仅限于sdk

ros输出图像:内参在`rostopic echo /zed2i/zed_node/left/camera_info`中读取



不确定内参的准确程度？
需要标定？
1. zed相机的内参？找到对应的内参
2. 标定相机的内参？找到对应的内参和imu 的内参、联合标定

[相机标定](https://blog.csdn.net/TengYun_zhang/article/details/123072847)

命令行：相机标定

```
rosrun camera_calibration cameracalibrator.py --approximate 0.1 --size 8x6 --square 0.02 right:=/zed2i/zed_node/right/image_rect_color left:=/zed2i/zed_node/left/image_rect_color right_camera:=/zed2i/zed2_node/right left_camera:=/zed2i/zed_node/left --no-service-check
```
imu与相机：
0.999945 0.010142 -0.002728 -0.002000
-0.010135 0.999946 0.002477 -0.023000
0.002753 -0.002450 0.999993 0.000220
0.000000 0.000000 0.000000 1.000000

2k:
1104*621

vga: 640*360
distortion_model: "plumb_bob"
D: [0.0, 0.0, 0.0, 0.0, 0.0]
K: [493.12091064453125, 0.0, 319.18212890625, 0.0, 493.12091064453125, 187.33084106445312, 0.0, 0.0, 1.0]
R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
P: [493.12091064453125, 0.0, 319.18212890625, 0.0, 0.0, 493.12091064453125, 187.33084106445312, 0.0, 0.0, 0.0, 1.0, 0.0]



## vins_mono

运行命令：
```
roslaunch zed_wrapper zed2i.launch
roslaunch vins_estimator realsense_color.launch 
roslaunch vins_estimator vins_rviz.launch
```
出现漂移，估计是imu的参数问题



## 计划

20230415： 相机标定弄好，imu标定