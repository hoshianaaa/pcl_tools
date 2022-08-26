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

#include <tf/transform_listener.h>

using namespace std;

double m11, m12, m13, m14;
double m21, m22, m23, m24;
double m31, m32, m33, m34;


std::string input_topic_ = "input_cloud";
std::string output_topic_ = "output_cloud";
std::string output_frame_ = "base_link";

class CloudTF
{
public:
  CloudTF()
  {
//    ros::NodeHandle private_nh("~");
//    private_nh.param("flip_y", flip_y_, false);
//    private_nh.param("robot_frame", robot_frame_, std::string("base_link"));
//    private_nh.param("input", input_cloud_topic_, std::string("/camera/depth/color/points"));
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_,1, false);
    cloud_sub_ = nh_.subscribe(input_topic_, 1, &CloudTF::cloudCallback, this);

  }
private:
  ros::NodeHandle nh_;
  ros::Publisher cloud_pub_;
  ros::Subscriber cloud_sub_;

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {

    //sensor_msgs::PointCloud2 ro

    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;

    pcl::fromROSMsg (*msg, pcl_cloud);

    for(int i=0;i<pcl_cloud.size();i++)
    {
      pcl::PointXYZ p;

      double x = pcl_cloud[i].x;
      double y = pcl_cloud[i].y;
      double z = pcl_cloud[i].z;

      p.x = m11 * x + m12 * y + m13 * z + m14;
      p.y = m21 * x + m22 * y + m23 * z + m24;
      p.z = m31 * x + m32 * y + m33 * z + m34;

      pcl_cloud[i] = p;
    }

    sensor_msgs::PointCloud2 ros_cloud;

    toROSMsg(pcl_cloud, ros_cloud);

    ros_cloud.header.frame_id = output_frame_;

    cloud_pub_.publish(ros_cloud);
  }
};

/*
vector<string> split(string& input, char delimiter)
{
  istringstream stream(input);
  string field;
  vector<string> result;
  while (getline(stream, field, delimiter)) {
    result.push_back(field);
  }
  return result;
}
*/


int main(int argc, char** argv)
{

  ros::init(argc, argv, "camera_tf");

  m11 = 1;
  m12 = 0;
  m13 = 0;
  m14 = 0;

  m21 = 0;
  m22 = 1;
  m23 = 0;
  m24 = 0;

  m31 = 0;
  m32 = 0;
  m33 = 1;
  m34 = 0;


/*
  std::string fname(argv[1]);

  ifstream ifs(fname);

  std::cout << "file name:" << fname << std::endl;

  string line;
  vector<string> strvec = split(line, ',');


  getline(ifs, line);
  strvec = split(line, ',');
  m11 = std::stof(strvec.at(0));
  m12 = std::stof(strvec.at(1));
  m13 = std::stof(strvec.at(2));
  m14 = std::stof(strvec.at(3));

  getline(ifs, line);
  strvec = split(line, ',');
  m21 = std::stof(strvec.at(0));
  m22 = std::stof(strvec.at(1));
  m23 = std::stof(strvec.at(2));
  m24 = std::stof(strvec.at(3));

  getline(ifs, line);
  strvec = split(line, ',');
  m31 = std::stof(strvec.at(0));
  m32 = std::stof(strvec.at(1));
  m33 = std::stof(strvec.at(2));
  m34 = std::stof(strvec.at(3));

  std::cout << m11 << " " << m12 << " " << m13 << " " << m14 << std::endl;
  std::cout << m21 << " " << m22 << " " << m23 << " " << m24 << std::endl;
  std::cout << m31 << " " << m32 << " " << m33 << " " << m34 << std::endl;
  */

  CloudTF ct;

  ros::spin();
}
