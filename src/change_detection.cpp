//ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>

//pcl
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>

class ChangeDetection
{
public:
  ChangeDetection();

private:
  ros::NodeHandle nh_;
  ros::Publisher diff_pub_;
  ros::Subscriber cloud_sub_;
  void cloudCallback(const sensor_msgs::PointCloud2 &pc);
  void publishcloud();
  std::string sensor_frame_;
  std::string input_topic_name_;
  std::string output_topic_name_;

  float resolution_,noise_filter_;


  pcl::PointCloud<pcl::PointXYZ>::Ptr first_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud;

  bool first_ = true;

};

ChangeDetection::ChangeDetection()
{
  std::cout << "start change detection" << std::endl;
  input_topic_name_ = "/points";
  output_topic_name_ = "/change_cloud";

  resolution_ = 0.01;
  noise_filter_ = 2;

  first_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
  filtered_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

  diff_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_name_,1, false);
  cloud_sub_ = nh_.subscribe(input_topic_name_, 1, &ChangeDetection::cloudCallback, this);
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

  if (first_){
    first_cloud = cloud;
    first_ = false;
  }
  else
  {


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
  }
}

int main(int argc, char **argv)
{
  std::cout << "start_change detection" << std::endl;
	ros::init(argc, argv, "change_detection");
	ChangeDetection cd;
	ros::Rate r(10);
	while(ros::ok()){
		ros::spinOnce();
		r.sleep();
	}
	return 0;
}
  
