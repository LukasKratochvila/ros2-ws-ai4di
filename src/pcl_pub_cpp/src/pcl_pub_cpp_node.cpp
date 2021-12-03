#include <cstdio>
#include <string>
#include <iostream>
#include "boost/filesystem.hpp"

#include "rclcpp/rclcpp.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "cv_bridge/cv_bridge.h"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "pcl_conversions/pcl_conversions.h"

class PclPublisher : public rclcpp::Node
{
  public:
    PclPublisher(void)
    : Node("pcl_pub_cpp"), pcl_counter(0)
    {
      pclPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud",10);
      period_ = std::chrono::milliseconds(500);
      timer_ = this->create_wall_timer(period_, std::bind(&PclPublisher::lidar_callback, this));

      dir_ = "/home/kratochvila/Desktop/Data/tovarna/vyber/mer1/pcl";
      for (const auto & entry : boost::filesystem::directory_iterator(dir_))
        files_.push_back(entry.path().string());
      std::sort(files_.begin(), files_.end());

      RCLCPP_INFO(get_logger(), "Pcl_pub_cpp has started.");
    }

  private:    
    void lidar_callback(void)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
      sensor_msgs::msg::PointCloud2::SharedPtr msg (new sensor_msgs::msg::PointCloud2());

      if (pcl_counter > files_.max_size()-1) pcl_counter = 0;
      
      if (pcl::io::loadPCDFile<pcl::PointXYZ> (files_.at(pcl_counter), *cloud) == -1)
        PCL_ERROR("Couldn't read file %s!", files_.at(pcl_counter).c_str());
      
      pcl::toROSMsg(*cloud, *msg);
      msg->header.frame_id="cloud";
      pclPub_->publish(*msg);
      //RCLCPP_INFO(get_logger(), "Publishing a point cloud from file %s", files_.at(pcl_counter).c_str());	    
	    ++pcl_counter;
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pclPub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::milliseconds period_;
    size_t pcl_counter;
    std::vector<std::string> files_;

    std::string dir_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PclPublisher>());
  rclcpp::shutdown();
  return 0;
}
