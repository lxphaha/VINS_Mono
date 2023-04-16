# zed2i相机使用

## 相机内参

文件可以在`/usr/local/zed/settings`中查看相机的内参

不确定内参的准确程度？
需要标定？
1. zed相机的内参？找到对应的内参
2. 标定相机的内参？找到对应的内参和imu 的内参、联合标定

[相机标定](https://blog.csdn.net/TengYun_zhang/article/details/123072847)

命令行：相机标定

```
rosrun camera_calibration cameracalibrator.py --approximate 0.1 --size 8x6 --square 0.02 right:=/zed2i/zed_node/right/image_rect_color left:=/zed2i/zed_node/left/image_rect_color right_camera:=/zed2i/zed2_node/right left_camera:=/zed2i/zed_node/left --no-service-check
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

20230415： 相机标定弄好，imu标定