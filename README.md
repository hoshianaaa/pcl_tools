# pcl_tools

ros pcl tool package

# install

```
sudo apt-get install ros-melodic-pcl-ros
```

# サンプル
## 点群の出力
唐揚げ点群がrealsenseと同じtopicで出力される.    
topic名:/camera/depth/color/points  
frame:camera_depth_optical_frame  
```
roslaunch pcl_tools pcd_to_realsese_pointcloud.launch
```

## 差分検出
PointCloud2から入力したPCDデータ部分を除去してPointCloud2として出力

## pca把持推定

```
roslaunch pcl_tools pca.launch 
```

x,y方向はpcaで中心位置推定,zは点群の最大高さ

![Screenshot from 2021-05-01 15-47-30](https://user-images.githubusercontent.com/40942409/116774055-cb711680-aa94-11eb-8163-e70297023efb.png)





