#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

#include "pcl_conversions/pcl_conversions.h"

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/filters/voxel_grid.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/impl/search.hpp>

#include <vision_msgs/msg/detection3_d.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include <pcl/features/moment_of_inertia_estimation.h>

#include <pcl/common/transforms.h>

#include <math.h> 
#define PI 3.14159265

class Clustering : public rclcpp::Node
{
public:
  Clustering() : Node("clustering")
  {
    this->declare_parameter("input_topic", "/filteredPcl");
    this->declare_parameter("output_topic", "/detections_det");
    this->declare_parameter("cluster_tolerance", 0.2);
    this->declare_parameter("min_cluster_size", 20);
    this->declare_parameter("max_cluster_size", 10000);
    this->declare_parameter("OBB", false);
    this->declare_parameter("step", 15);
    this->declare_parameter("debug", false);

    input_topic = this->get_parameter("input_topic").as_string();
    output_topic = this->get_parameter("output_topic").as_string();
    cluster_tolerance = this->get_parameter("cluster_tolerance").as_double();
    min_cluster_size = this->get_parameter("min_cluster_size").as_int();
    max_cluster_size = this->get_parameter("max_cluster_size").as_int();
    get_parameter("OBB", OBB);
    get_parameter("step", step);
    debug = this->get_parameter("debug").as_bool();

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(input_topic, 10, std::bind(&Clustering::topic_callback, this, std::placeholders::_1));
    detections_publisher_ = this->create_publisher<vision_msgs::msg::Detection3DArray>(output_topic, 10);
    RCLCPP_INFO(this->get_logger(), "Clustering node has started.");
  }

private:
  void Get_AABB_(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr, pcl::Indices *indices, pcl::PointXYZ *min_point, pcl::PointXYZ *max_point) const
	{// https://pointclouds.org/documentation/moment__of__inertia__estimation_8hpp_source.html    
    min_point->x = std::numeric_limits<float>::max();
    min_point->y = std::numeric_limits<float>::max();
    min_point->z = std::numeric_limits<float>::max();
    
    max_point->x = -std::numeric_limits<float>::max();
    max_point->y = -std::numeric_limits<float>::max();
    max_point->z = -std::numeric_limits<float>::max();
    
    unsigned int number_of_points = static_cast <unsigned int>(indices->size());
    for (unsigned int i_point = 0; i_point < number_of_points; i_point++)
    {    
      if ((*cloud_ptr)[(*indices)[i_point]].x <= min_point->x) min_point->x = (*cloud_ptr)[(*indices)[i_point]].x;
      if ((*cloud_ptr)[(*indices)[i_point]].y <= min_point->y) min_point->y = (*cloud_ptr)[(*indices)[i_point]].y;
      if ((*cloud_ptr)[(*indices)[i_point]].z <= min_point->z) min_point->z = (*cloud_ptr)[(*indices)[i_point]].z;
    
      if ((*cloud_ptr)[(*indices)[i_point]].x >= max_point->x) max_point->x = (*cloud_ptr)[(*indices)[i_point]].x;
      if ((*cloud_ptr)[(*indices)[i_point]].y >= max_point->y) max_point->y = (*cloud_ptr)[(*indices)[i_point]].y;
      if ((*cloud_ptr)[(*indices)[i_point]].z >= max_point->z) max_point->z = (*cloud_ptr)[(*indices)[i_point]].z;
    }
  }
  void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    std::stringstream log;

    vision_msgs::msg::Detection3DArray output_det;
    output_det.header = msg->header;
    output_det.detections.clear();
  
    // Dont have anything to process -> return
    if (!(*cloud).size())
    {
      detections_publisher_->publish(output_det);
      return;
    }
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud->makeShared());

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance);
    ec.setMinClusterSize(min_cluster_size);
    ec.setMaxClusterSize(max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud->makeShared());
    ec.extract(cluster_indices);

    if (debug)
    {
      log << "PointCloud representing: " << cluster_indices.size() << " Clusters." << std::endl;
      RCLCPP_INFO(this->get_logger(), log.str());
      log.clear();
    }
    // Dont have anything to publish -> return
    if (!cluster_indices.size())
    {
      detections_publisher_->publish(output_det);
      return;
    }
    std::vector<double> cx, cy, cz, dx, dy, dz, ox, oy, oz, ow;
    pcl::PointXYZ minPoint, maxPoint, center_point, min_point, max_point;
  
    if (!OBB)
    {
      for (auto &it : cluster_indices)
      {
        Get_AABB_(cloud, &it.indices, &minPoint, &maxPoint);
        cx.emplace_back((maxPoint.x + minPoint.x) / 2.0f);
        cy.emplace_back((maxPoint.y + minPoint.y) / 2.0f);
        cz.emplace_back((maxPoint.z + minPoint.z) / 2.0f);
        dx.emplace_back(maxPoint.x);
        dy.emplace_back(maxPoint.y);
        dz.emplace_back(maxPoint.z);
        ox.emplace_back(0.0);
        oy.emplace_back(0.0);
        oz.emplace_back(0.0);
        ow.emplace_back(1.0);
      }
    }
    else
    {
      if (debug)
      {
        log << "Computing clusters OBB." << std::endl;
        RCLCPP_INFO(this->get_logger(), log.str());
        log.clear();
      }      
      for (pcl::PointIndices &it : cluster_indices)
      {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::Indices cluster_idx;
        for (const auto &idx : it.indices)
          cloud_cluster->push_back((*cloud)[idx]);
        for (size_t i=0; i<cloud_cluster->size(); i++)
          cluster_idx.push_back(i);

        Clustering::Get_AABB_(cloud_cluster, &cluster_idx, &minPoint, &maxPoint);

        double angle = 0;
        double surface_area = (maxPoint.x - minPoint.x) * (maxPoint.y - minPoint.y);
        min_point = minPoint;
        max_point = maxPoint;
        center_point.x = (maxPoint.x + minPoint.x) / 2.0f;
        center_point.y = (maxPoint.y + minPoint.y) / 2.0f;
        center_point.z = (maxPoint.z + minPoint.z) / 2.0f;

        // subract mean value for correct rotation
        for (const auto &idx : cluster_idx)
        {
          (*cloud_cluster)[idx].x -= center_point.x;
          (*cloud_cluster)[idx].y -= center_point.y;
          (*cloud_cluster)[idx].z -= center_point.z;
        }

        for(size_t angle_step=1; angle_step*step < 180; angle_step++){
          Eigen::Matrix4f transform_rot = Eigen::Matrix4f::Identity();
          transform_rot.topLeftCorner(3, 3) = Eigen::Matrix3f(Eigen::Quaternionf(cos(step/2/180.0f*PI),0,0,sin(step/2/180.0f*PI)));
				  pcl::transformPointCloud(*cloud_cluster, *cloud_cluster, transform_rot);

          Clustering::Get_AABB_(cloud_cluster, &cluster_idx, &minPoint, &maxPoint);
          if ((maxPoint.x - minPoint.x) * (maxPoint.y - minPoint.y) < surface_area){
            surface_area = (maxPoint.x - minPoint.x) * (maxPoint.y - minPoint.y);
            angle = angle_step*step;
            min_point = minPoint;
            max_point = maxPoint;
          }          
        }

        cx.emplace_back(center_point.x);
        cy.emplace_back(center_point.y); 
        cz.emplace_back(center_point.z); 
        dx.emplace_back(max_point.x - min_point.x);
        dy.emplace_back(max_point.y - min_point.y); 
        dz.emplace_back(max_point.z - min_point.z); 
        ox.emplace_back(0.0);
        oy.emplace_back(0.0);
        oz.emplace_back(sin(-angle/2.0f/180.0f*PI));
        ow.emplace_back(cos(-angle/2.0f/180.0f*PI));
      }
    }

    if (debug)
    {
      log << "Creating Detection3DArray..." << std::endl;
      RCLCPP_INFO(this->get_logger(), log.str());
      log.clear();
    }

    size_t cnt = 0;
    for (size_t i = 0; i < cx.size(); i++)
    {
      vision_msgs::msg::Detection3D det;
      det.header = msg->header;
      det.tracking_id = std::to_string(cnt++);

      det.bbox.center.position.x = cx.at(i);
      det.bbox.center.position.y = cy.at(i);
      det.bbox.center.position.z = cz.at(i);

      det.bbox.center.orientation.x = ox.at(i);
      det.bbox.center.orientation.y = oy.at(i);
      det.bbox.center.orientation.z = oz.at(i);
      det.bbox.center.orientation.w = ow.at(i);

      det.bbox.size.x = dx.at(i);
      det.bbox.size.y = dy.at(i);
      det.bbox.size.z = dz.at(i);

      output_det.detections.push_back(det);
    }
    detections_publisher_->publish(output_det);

    if (debug)
    {
      log << "Publishing: " << cx.size() << " detections." << std::endl;
      RCLCPP_INFO(this->get_logger(), log.str());
      log.clear();
    }
  }
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_{nullptr};
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr detections_publisher_{nullptr};

  std::string input_topic, output_topic, output_topic_vizualization;
  double cluster_tolerance, min_cluster_size, max_cluster_size;
  bool OBB;
  int step;
  bool debug;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Clustering>());
  rclcpp::shutdown();
  return 0;
}