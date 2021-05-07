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


std::string input_cloud_topic_ = "/camera/depth/color/points";
std::string output_cloud_topic_ = "tf_cloud";
std::string robot_frame_ = "base_link";
std::string target_frame_ = "camera_color_optical_frame";

class CameraTF
{
public:
  CameraTF()
  {


    ros::NodeHandle private_nh("~");
    private_nh.param("flip_y", flip_y_, false);
    output_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_cloud_topic_,1, false);
    input_cloud_sub_ = nh_.subscribe(input_cloud_topic_, 1, &CameraTF::cloudCb, this);
  }
private:
  ros::NodeHandle nh_;
  ros::Publisher output_cloud_pub_;
  ros::Subscriber input_cloud_sub_;
  tf::TransformListener tf_listener_;
  bool flip_y_;

  void cloudCb(const sensor_msgs::PointCloud2ConstPtr& msgs)
  {
    std::cout << "callback" << std::endl;

    auto input_frame = std::string(msgs->header.frame_id);

    std::cout << "input_frame:" <<  input_frame << std::endl;

    sensor_msgs::PointCloud2 ros_cloud;

    if(input_frame != target_frame_)
    {
       try
      {
          tf::StampedTransform trans;
          tf_listener_.waitForTransform(target_frame_, input_frame, ros::Time(0), ros::Duration(0.5));
          tf_listener_.lookupTransform(target_frame_, input_frame, ros::Time(0), trans);
          pcl_ros::transformPointCloud(target_frame_, trans, *msgs, ros_cloud);
      }
      catch(tf::TransformException &e)
      {
          ROS_WARN("%s", e.what());
      }
    }

    pcl::PointCloud<pcl::PointXYZ> input_cloud, tf_cloud;
    pcl::fromROSMsg (ros_cloud, input_cloud);

    for(int i=0;i<input_cloud.size();i++)
    {
      pcl::PointXYZ p;
      double x = input_cloud[i].x;
      double y = input_cloud[i].y;
      double z = input_cloud[i].z;
      p.x = m11 * x + m12 * y + m13 * z + m14;
      p.y = m21 * x + m22 * y + m23 * z + m24;
      p.z = m31 * x + m32 * y + m33 * z + m34;

      if(flip_y_ == true)p.y = -p.y;

      tf_cloud.push_back(p);
    }

    sensor_msgs::PointCloud2 output_cloud_ros;
    toROSMsg(tf_cloud, output_cloud_ros);
    output_cloud_ros.header.frame_id = robot_frame_;
    output_cloud_pub_.publish(output_cloud_ros);
  }
};

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


int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_tf");

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

  CameraTF ct;

  ros::spin();
}
