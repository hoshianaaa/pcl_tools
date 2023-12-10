#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <vector>
#include <cmath>

// PointCloud2メッセージを扱うためのヘルパー関数
void convertToPointCloud2(const cv::Mat &image, sensor_msgs::PointCloud2 &pointcloud, float scale, float offsetX, float offsetY, float rotationAngle) {
    // 点群の初期設定
    pointcloud.height = 1;
    pointcloud.width = image.rows * image.cols;
    pointcloud.is_dense = false;
    pointcloud.is_bigendian = false;

    // フィールドの定義
    sensor_msgs::PointField x_field, y_field, z_field, rgb_field;
    x_field.name = "x";
    x_field.offset = 0;
    x_field.datatype = sensor_msgs::PointField::FLOAT32;
    x_field.count = 1;

    y_field.name = "y";
    y_field.offset = 4;
    y_field.datatype = sensor_msgs::PointField::FLOAT32;
    y_field.count = 1;

    z_field.name = "z";
    z_field.offset = 8;
    z_field.datatype = sensor_msgs::PointField::FLOAT32;
    z_field.count = 1;

    rgb_field.name = "rgb";
    rgb_field.offset = 12;
    rgb_field.datatype = sensor_msgs::PointField::UINT32;
    rgb_field.count = 1;

    pointcloud.fields.push_back(x_field);
    pointcloud.fields.push_back(y_field);
    pointcloud.fields.push_back(z_field);
    pointcloud.fields.push_back(rgb_field);

    // 点群データのサイズ設定
    pointcloud.point_step = 16; // x, y, z の各4バイト + RGBの4バイト
    pointcloud.row_step = pointcloud.point_step * pointcloud.width;
    pointcloud.data.resize(pointcloud.row_step * pointcloud.height);

    // 回転のためのパラメータ
    float cosAngle = cos(rotationAngle);
    float sinAngle = sin(rotationAngle);
    float centerX = image.cols / 2.0f;
    float centerY = image.rows / 2.0f;

    // 画像データを点群データに変換
    int idx = 0;
    for (int i = 0; i < image.rows; ++i) {
        for (int j = 0; j < image.cols; ++j) {
            // 座標の計算
            float x = j * scale + offsetX;
            float y = (image.rows - 1 - i) * scale + offsetY;

            // 中心点を軸にした回転
            float translatedX = x - centerX;
            float translatedY = y - centerY;
            float rotatedX = translatedX * cosAngle - translatedY * sinAngle;
            float rotatedY = translatedX * sinAngle + translatedY * cosAngle;
            x = rotatedX + centerX;
            y = rotatedY + centerY;

            // 座標を設定
            *(reinterpret_cast<float*>(&pointcloud.data[idx + 0])) = x;
            *(reinterpret_cast<float*>(&pointcloud.data[idx + 4])) = y;
            *(reinterpret_cast<float*>(&pointcloud.data[idx + 8])) = 0.0; // Z座標 (常に0)

            // RGB値を設定
            cv::Vec3b color = image.at<cv::Vec3b>(i, j);
            uint32_t rgb = (static_cast<uint32_t>(color[2]) << 16 | static_cast<uint32_t>(color[1]) << 8 | static_cast<uint32_t>(color[0]));
            *(reinterpret_cast<uint32_t*>(&pointcloud.data[idx + 12])) = rgb;

            idx += pointcloud.point_step;
        }
    }
}

int main(int argc, char **argv) {
    // ROSノードの初期化
    ros::init(argc, argv, "image_to_pointcloud");
    ros::NodeHandle nh;

    // Publisherの作成
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pointcloud", 1);

    // OpenCVで画像を読み込む
    cv::Mat image = cv::imread("/home/dev/office_map2.png", cv::IMREAD_COLOR);
    if (image.empty()) {
        std::cerr << "Failed to load image." << std::endl;
        return -1;
    }

    // 変換パラメータ
    float scale = 0.01f; // 拡大縮小率
    float offsetX = 0.0f; // Xオフセット
    float offsetY = 0.0f; // Yオフセット
    float rotationAngle = 0.0f; // 回転角度（ラジアン）

    // 画像をPointCloud2に変換
    sensor_msgs::PointCloud2 pointcloud;
    convertToPointCloud2(image, pointcloud, scale, offsetX, offsetY, rotationAngle);
    pointcloud.header.frame_id = "map";

    // ループしてPointCloudをpublishする
    ros::Rate loop_rate(1);
    while (ros::ok()) {
        pcl_pub.publish(pointcloud);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

