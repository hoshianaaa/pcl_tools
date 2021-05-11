//ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_srvs/Empty.h>

//pcl
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>

#include <pcl/io/io.h>


class ChangeDetection
{
public:
  ChangeDetection(ros::NodeHandle* nodehandle, std::string fname);

private:
  ros::NodeHandle nh_;

  ros::Publisher diff_pub_;
  ros::Subscriber cloud_sub_;
  ros::ServiceServer srv_;

  void cloudCallback(const sensor_msgs::PointCloud2 &pc);
  bool serviceCallBack(std_srvs::Empty::Request &req,std_srvs::Empty::Response &resp);
  void publishcloud();
  std::string sensor_frame_;
  std::string input_topic_name_;
  std::string output_topic_name_;

  float resolution_,noise_filter_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr first_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud;

  std::string fname_;
  bool first_;

};

ChangeDetection::ChangeDetection(ros::NodeHandle* nodehandle, std::string fname):nh_(*nodehandle)
{

  fname_ = fname;

  std::cout << "start change detection" << std::endl;

  input_topic_name_ = "/points";
  output_topic_name_ = "/change_cloud";

  resolution_ = 0.01;
  noise_filter_ = 2;

  first_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
  filtered_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::io::loadPCDFile (fname_, *first_cloud);

  diff_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_name_,1, false);
  cloud_sub_ = nh_.subscribe(input_topic_name_, 1, &ChangeDetection::cloudCallback, this);
  srv_ = nh_.advertiseService("savePCD", &ChangeDetection::serviceCallBack, this);

  first_ = false;
}

void ChangeDetection::cloudCallback(const sensor_msgs::PointCloud2  &pc)
{

  if(pc.fields.size() <= 0){
    return;
  }

  std::cout << "cloud callback" << std::endl;
  sensor_frame_ = pc.header.frame_id;
  std::cout << "frame id" << sensor_frame_ << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

 // std::vector<int> indices;

  pcl::fromROSMsg (pc, *cloud);

  if (first_ == true)
  {
    first_cloud = cloud;
    pcl::io::savePCDFileASCII (fname_, *first_cloud);
    first_ = false;
  }

  pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> *octree_  = new pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ>(resolution_);

  octree_->setInputCloud (first_cloud);
  octree_->addPointsFromInputCloud ();

  octree_->switchBuffers ();

  octree_->setInputCloud (cloud);
  octree_->addPointsFromInputCloud ();

  boost::shared_ptr<std::vector<int> > newPointIdxVector (new std::vector<int>);
  octree_->getPointIndicesFromNewVoxels (*newPointIdxVector, noise_filter_);

  filtered_cloud.reset (new pcl::PointCloud<pcl::PointXYZ>);
  filtered_cloud->points.reserve(newPointIdxVector->size());

  for (std::vector<int>::iterator it = newPointIdxVector->begin (); it != newPointIdxVector->end (); it++)
    filtered_cloud->points.push_back(cloud->points[*it]);

  sensor_msgs::PointCloud2 octree_change_pointcloud2;
  pcl::toROSMsg(*filtered_cloud, octree_change_pointcloud2);
  
  octree_change_pointcloud2.header = pc.header;
  octree_change_pointcloud2.is_dense = false;
  diff_pub_.publish(octree_change_pointcloud2);

  delete octree_;
}

bool ChangeDetection::serviceCallBack(std_srvs::Empty::Request &req,
std_srvs::Empty::Response &resp) {        
  ROS_INFO_STREAM("save now point cloud!");       
  first_ = true;
  return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "change_detection");

  ros::NodeHandle nh;

  std::string fname(argv[1]);
	ChangeDetection cd(&nh,fname);

  ros::spin();

	return 0;
}
  
