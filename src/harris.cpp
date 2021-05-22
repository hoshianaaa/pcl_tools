#include <fstream>
#include <string>
#include <sstream>
#include <vector>

#include <ros/ros.h>
#include <ros/package.h>

#include <iostream>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/geometry.h>
#include <pcl/registration/transformation_estimation_svd.h>

#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/keypoints/harris_3d.h>

#include <tf/transform_listener.h>

using namespace std;


class Harris
{
public:
  Harris()
  {
    ros::NodeHandle private_nh("~");
    private_nh.param("input", input_cloud_topic_, std::string("/camera/depth/color/points"));
    private_nh.param("output", output_cloud_topic_, std::string("/harris_points"));
    output_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_cloud_topic_,1, false);
    input_cloud_sub_ = nh_.subscribe(input_cloud_topic_, 1, &Harris::cloudCb, this);
  }
private:
  ros::NodeHandle nh_;
  ros::Publisher output_cloud_pub_;
  ros::Subscriber input_cloud_sub_;
  std::string input_cloud_topic_;
  std::string output_cloud_topic_;

  void cloudCb(const sensor_msgs::PointCloud2ConstPtr& msgs)
  {
    std::cout << "callback" << std::endl;

    pcl::PointCloud<pcl::PointXYZ> input_cloud, tf_cloud;
    pcl::fromROSMsg (*msgs, input_cloud);

    std::cout << input_cloud.size() << std::endl;

    pcl::HarrisKeypoint3D <pcl::PointXYZ, pcl::PointXYZI> detector;
    pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints (new pcl::PointCloud<pcl::PointXYZI>);
    detector.setNonMaxSupression (true);
    detector.setInputCloud (input_cloud.makeShared());

    detector.setThreshold (1e-6);

    pcl::StopWatch watch;
    detector.compute (*keypoints);

    pcl::PointIndicesConstPtr keypoints_indices = detector.getKeypointsIndices ();

    sensor_msgs::PointCloud2 output_cloud_ros;

    toROSMsg(*keypoints, output_cloud_ros);

    output_cloud_ros.header.frame_id = msgs->header.frame_id;

    output_cloud_pub_.publish(output_cloud_ros);
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_tf");

  Harris hr;

  ros::spin();
}
