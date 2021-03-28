# pcl_tools
rosでpclを使うときのツール

```
#依存パッケージのインストール
sudo apt-get install ros-melodic-pcl-ros
```

# サンプル  
##点群の出力
唐揚げ点群がrealsenseと同じtopicで出力される.    
topic名:/camera/depth/color/points  
frame:camera_depth_optical_frame  
```
roslaunch pcl_tools pcd_to_realsese_pointcloud.launch
```

##差分検出
PointCloud2から入力したPCDデータ部分を除去してPointCloud2として出力

