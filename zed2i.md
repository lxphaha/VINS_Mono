# zed2i相机使用

## 相机内参

文件可以在`/usr/local/zed/settings`中查看相机的内参

不确定内参的准确程度？
需要标定？

## vins_mono

运行命令：
```
roslaunch zed_wrapper zed2i.launch
roslaunch vins_estimator realsense_color.launch 
roslaunch vins_estimator vins_rviz.launch
```
出现