//ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>

//pcl
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>

#include <iostream>
#include <vector>
#include <ctime>

double inner_product(std::vector<double> vec1, std::vector<double> vec2) 
{
    int i;
    double s = 0.0;

    for ( i = 0; i < vec1.size(); i++ ) {
        s += vec1[i] * vec2[i];
    }
    return s;
}

double vector_distance(std::vector<double> vec)
{
    double s = 0.0;

    for (int i = 0; i < vec.size(); i++ ) {
        s += vec[i] * vec[i];
    }
 
    return std::sqrt(s);
}

double vector_angle(std::vector<double> vec1, std::vector<double> vec2)
{
    return std::acos(inner_product(vec1,vec2)/vector_distance(vec1)/vector_distance(vec2));
}

class TowelDetection
{
public:
TowelDetection(ros::NodeHandle* nodehandle);
private:
ros::NodeHandle nh_;
ros::Publisher weights_pub_;
ros::Subscriber cloud_sub_;
void cloudCallback(const sensor_msgs::PointCloud2 &pc);
std::string frame_;
std::string input_topic_name_;
std::string weights_topic_name_;
};

TowelDetection::TowelDetection(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{
ros::NodeHandle private_nh("~");
private_nh.param("sensor_frame", frame_, std::string("/base_link"));
private_nh.param("input_topic_name", input_topic_name_, std::string("input_cloud"));

private_nh.param("weights_topic_name", weights_topic_name_, std::string("weights_cloud"));

weights_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(weights_topic_name_,1, false);
cloud_sub_ = nh_.subscribe(input_topic_name_, 1, &TowelDetection::cloudCallback, this);
}

void TowelDetection::cloudCallback(const sensor_msgs::PointCloud2 &pc)
{

  std::cout << "callback" << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromROSMsg (pc, *cloud);

  // ---------- graphのエッジ作成 --------------

  float radius = 0.007;

  std::vector<std::pair<int, int>> edges;

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

  kdtree.setInputCloud (cloud);

  pcl::PointXYZ searchPoint;

  for (size_t i = 0; i < cloud->points.size (); ++i)
  {

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    std::pair<int, int> edge;
    edge.first = i;

    //std::cout << "index[" << i << "]" << std::endl;
    if ( kdtree.radiusSearch (cloud->points[i], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
    {
      for (size_t j = 0; j < pointIdxRadiusSearch.size (); ++j)
      {
        edge.second = pointIdxRadiusSearch[j];
        if (edge.first == edge.second)continue;
        auto p = std::make_pair(edge.second, edge.first);
        /*
        if (std::find(edges.begin(), edges.end(), p) != edges.end())
        {
          //std::cout << "edge1:" << edge.first << " edge2:" << edge.second << " found!" << std::endl;
          continue;
        }
        */
        edges.push_back(edge);
        //std::cout << "edge1:" << edge.first << " edge2:" << edge.second << std::endl;
      }
    }
  }

  // ----------- 法線推定 --------------

  int k_ = 10;
  int search_radius_ = 0;
  int num_of_threads_ = 0;

  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> impl(num_of_threads_);
  impl.setInputCloud(cloud);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr
    tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  impl.setSearchMethod (tree);
  impl.setKSearch(k_);
  impl.setRadiusSearch(search_radius_);
  pcl::PointCloud<pcl::Normal>::Ptr
    normal_cloud(new pcl::PointCloud<pcl::Normal>);
  impl.compute(*normal_cloud);

  //normal_cloud->points[0].normal_x

  // --------- edge 重み計算 ----------
  std::vector<double> weights;
  double beta_th_ = 15;

  for (size_t i = 0; i < edges.size (); ++i)
  {

    auto index1 = edges[i].first;
    auto index2 = edges[i].second;

    std::vector<double> p1 = { cloud->points[index1].x, cloud->points[index1].y, cloud->points[index1].z };
    std::vector<double> p2 = { cloud->points[index2].x, cloud->points[index2].y, cloud->points[index2].z };

    std::vector<double> n1 = { normal_cloud->points[index1].normal_x, normal_cloud->points[index1].normal_y, normal_cloud->points[index1].normal_z };
    std::vector<double> n2 = { normal_cloud->points[index2].normal_x, normal_cloud->points[index2].normal_y, normal_cloud->points[index2].normal_z };

    std::vector<double> p12 = { p1[0] - p2[0], p1[1] - p2[1], p1[2] - p2[2]};
    std::vector<double> n12 = { n1[0] - n2[0], n1[1] - n2[1], n1[2] - n2[2]};

    double alpha1 = std::abs(vector_angle(p12,n1));
    double alpha2 = std::abs(vector_angle(p12,n2));

    double beta = std::abs(alpha1 - alpha2);
    
    double w;

    if ( beta < beta_th_ or inner_product(n12,p12) < 0)
    {
      w = inner_product(n1,n2);
    }
    else
    {
      w = -inner_product(n1,n2);
    }

    //std::cout << "weight:" << w << std::endl;
    weights.push_back(w);
  }

  pcl::PointCloud<pcl::PointXYZI> pcl_weights_cloud;
  pcl::PointXYZI p;

  for (size_t i = 0; i < edges.size (); ++i)
  {
    int index1 = edges[i].first;
    int index2 = edges[i].second;

    p.x = (cloud->points[index1].x + cloud->points[index2].x) / 2;
    p.y = (cloud->points[index1].y + cloud->points[index2].y) / 2;
    p.z = (cloud->points[index1].z + cloud->points[index2].z) / 2;
    p.intensity = weights[i]; 

    pcl_weights_cloud.push_back(p);
  }

  sensor_msgs::PointCloud2 weights_cloud_ros;
  pcl::toROSMsg(pcl_weights_cloud, weights_cloud_ros);

  weights_cloud_ros.header = pc.header;
  weights_cloud_ros.is_dense = false;
  weights_pub_.publish(weights_cloud_ros);
}

int
main (int argc, char** argv)
{
  ros::init(argc, argv, "towel_detection");
  ros::NodeHandle nh;
  TowelDetection td(&nh);
  ros::spin();

  return 0;
}
