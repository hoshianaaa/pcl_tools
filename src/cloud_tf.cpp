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

#include <std_msgs/Float32MultiArray.h>

using namespace std;

std::vector<double> m(12);

std::string name_ = "cloud_tf";
std::string input_topic_ = name_ + "/input_cloud";
std::string output_topic_ = name_ + "/output_cloud";

std::string output_frame_ = "base_link";

std::string f_name = "~/.ros/matrix.csv";

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
    matrix_sub_ = nh_.subscribe( name_ + "/matrix", 1, &CloudTF::matrixCallback, this);

  }
private:
  ros::NodeHandle nh_;
  ros::Publisher cloud_pub_;
  ros::Subscriber cloud_sub_, matrix_sub_;

  void matrixCallback(const std_msgs::Float32MultiArray& msg)
  {

    int num = msg.data.size();
    ROS_INFO("I susclibed [%i]", num);

    if (num != m.size())
    {
      ROS_ERROR("size error");
      return;
    }

    for (int i = 0; i < num; i++)
    {
      ROS_INFO("[%i]:%f", i, msg.data[i]);
    }

    auto list = msg.data;

    for (int i=0;i<m.size();i++)
    {
      m[i] = list[i];
    }

  }

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

      p.x = m[0] * x + m[1] * y + m[2] * z + m[3];
      p.y = m[4] * x + m[5] * y + m[6] * z + m[7];
      p.z = m[8] * x + m[9] * y + m[10] * z + m[11];

      pcl_cloud[i] = p;
    }

    sensor_msgs::PointCloud2 ros_cloud;

    toROSMsg(pcl_cloud, ros_cloud);

    ros_cloud.header.frame_id = output_frame_;

    cloud_pub_.publish(ros_cloud);
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

std::string home_dir_check(std::string str)
{

      std::string f_name = str;

      if (f_name[0] == '~')
      {
        f_name = f_name.erase(0, 1);
        
        std::string home_dir = std::getenv("HOME");

        //std::cout << "home dir:" << home_dir << std::endl;

        f_name = home_dir + f_name;
      }

      return f_name;
}

bool read_file(std::string f_name, std::vector<double>& data)
{
  std::ifstream ifs(f_name);
  if (!ifs)return false;
  
  data.clear();
  std::string line;
  std::vector<std::string> strvec;
  getline(ifs, line);
  strvec = split(line, ',');
  for (int i=0;i<strvec.size();i++)
    data.push_back(std::stof(strvec.at(0)));

  return true;
}

void write_file(std::string f_name, std::vector<double> data)
{
  ofstream outputfile(f_name);
  for (int i=0;i<data.size();i++)
  {
    outputfile << data[i];
    if (i<data.size()-1)outputfile << ",";
  }
  outputfile.close();
}


int main(int argc, char** argv)
{

  ros::init(argc, argv, "cloud_tf");

  m[0] = 1;
  m[5] = 1;
  m[10] = 1;

  // file_name
  if (argc > 1)
  {
    f_name = argv[1];
  }

  f_name = home_dir_check(f_name);

  std::cout << "file name :" << f_name << std::endl;

  // read_file or write_file
  std::vector<double> data;
  auto result = read_file(f_name, data);

  if (!result || data.size()!=12)
    write_file(f_name, m);    
  else
    m = data;

/*
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
