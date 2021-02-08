# pcl_tools
rosでpclを使うときのツールパッケージ

```
#依存パッケージのインストール
sudo apt-get install ros-melodic-pcl-ros
```

# サンプル  

唐揚げ点群がrealsenseと同じtopicで出力される.  
topic名:/camera/depth/color/points
frame:camera_depth_optical_frame
```
roslaunch pcl_tools pcd_to_realsese_pointcloud.launch
```
