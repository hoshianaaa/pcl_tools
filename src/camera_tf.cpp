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

using namespace std;

std::string input_cloud_topic_ = "/camera/depth/color/points";
std::string output_cloud_topic_ = "tf_depth_cloud";
std::string robot_frame_ = "base_link";

class CameraTF
{
public:
  CameraTF(const std::string& fname)
  {

    ifstream ifs(fname);

    string line;
    vector<string> strvec = split(line, ',');

    getline(ifs, line);
    strvec = split(line, ',');
    a_1_1 = std::stof(strvec.at(0));
    a_1_2 = std::stof(strvec.at(1));
    a_1_3 = std::stof(strvec.at(2));
    c_1 = std::stof(strvec.at(3));

    getline(ifs, line);
    strvec = split(line, ',');
    a_2_1 = std::stof(strvec.at(0));
    a_2_2 = std::stof(strvec.at(1));
    a_2_3 = std::stof(strvec.at(2));
    c_2 = std::stof(strvec.at(3));

    getline(ifs, line);
    strvec = split(line, ',');
    a_3_1 = std::stof(strvec.at(0));
    a_3_2 = std::stof(strvec.at(1));
    a_3_3 = std::stof(strvec.at(2));
    c_3 = std::stof(strvec.at(3));


    output_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_cloud_topic_,1, false);
    input_cloud_sub_ = nh_.subscribe(input_cloud_topic_, 1, &CameraTF::cloudCb, this);
  }
private:
  ros::NodeHandle nh_;
  ros::Publisher output_cloud_pub_;
  ros::Subscriber input_cloud_sub_;

  double a_1_1, a_1_2, a_1_3, c_1;
  double a_2_1, a_2_2, a_2_3, c_2;
  double a_3_1, a_3_2, a_3_3, c_3;

  void cloudCb(const sensor_msgs::PointCloud2ConstPtr& msgs)
  {
    std::cout << "callback" << std::endl;
    pcl::PointCloud<pcl::PointXYZ> input_cloud, tf_cloud;
    pcl::fromROSMsg (*msgs, input_cloud);

    for(int i=0;i<input_cloud.size();i++)
    {
      pcl::PointXYZ p;
      double x = input_cloud[i].x;
      double y = input_cloud[i].y;
      double z = input_cloud[i].z;
      p.x = a_1_1 * x + a_1_2 * y + a_1_3 * z + c_1;
      p.y = a_2_1 * x + a_2_2 * y + a_2_3 * z + c_2;
      p.z = a_3_1 * x + a_3_2 * y + a_3_3 * z + c_3;

      p.x = p.x / 1000;
      p.y = p.y / 1000;
      p.z = p.z / 1000;

      tf_cloud.push_back(p);
    }

    sensor_msgs::PointCloud2 output_cloud_ros;
    toROSMsg(tf_cloud, output_cloud_ros);
    output_cloud_ros.header.frame_id = robot_frame_;
    output_cloud_pub_.publish(output_cloud_ros);
  }

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
};

int main(int argc, char** argv)
{
  std::cout << "test" << std::endl;
  ros::init(argc, argv, "camera_tf");

  if(argc==2 || argc==3)
  {
    std::string fname(argv[1]);
    CameraTF ct(fname);
  }
  ros::spin();
}
