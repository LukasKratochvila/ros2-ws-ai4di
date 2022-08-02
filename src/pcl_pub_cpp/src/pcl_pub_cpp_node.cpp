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
    : Node("pcl_pub_cpp")
    {
      this->declare_parameter("input_dir", "/home/kratochvila/Desktop/Data/tovarna/vyber/mer1/pcl");
      this->declare_parameter("frequency", 2.0);
      this->declare_parameter("output_topic", "/cloud");
      this->declare_parameter("frame_id", "cloud");
      this->declare_parameter("start_counter", 0);
      this->declare_parameter("file_ext", ".pcd");
      this->declare_parameter("debug", false);
      
      pcl_counter_ = this->get_parameter("start_counter").as_int();
      file_ext_ = this->get_parameter("file_ext").as_string();
      debug_ = this->get_parameter("debug").as_bool();
      frame_id_ = this->get_parameter("frame_id").as_string();
      freq_ = this->get_parameter("frequency").as_double();
      period_ = std::chrono::milliseconds(static_cast<int>(1000.0 / freq_));
      pclPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(this->get_parameter("output_topic").as_string(),10);
      timer_ = this->create_wall_timer(period_, std::bind(&PclPublisher::lidar_callback, this));

      dir_ = this->get_parameter("input_dir").as_string();
      if (!boost::filesystem::is_directory(dir_)){
        RCLCPP_ERROR(get_logger(), "Input dir: %s is not directory or doesn't exist!", dir_.c_str());
        return;
      }
      for (const auto & entry : boost::filesystem::directory_iterator(dir_))
        if (entry.path().extension() == file_ext_)
          files_.push_back(entry.path().string());
        else if (debug_)
          RCLCPP_INFO(get_logger(), "Unsuported extension: %s", entry.path().extension().c_str());
      if (debug_)
        RCLCPP_INFO(get_logger(), "Read directory %s with %d files.", dir_.c_str(), files_.size());
      std::sort(files_.begin(), files_.end());

      RCLCPP_INFO(get_logger(), "Pcl_pub_cpp has started.");
    }

  private:    
    void lidar_callback(void)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
      sensor_msgs::msg::PointCloud2::SharedPtr msg (new sensor_msgs::msg::PointCloud2());

      if (pcl_counter_ > files_.size()-1) pcl_counter_ = 0;
      
      if (pcl::io::loadPCDFile<pcl::PointXYZ> (files_.at(pcl_counter_), *cloud) == -1)
        PCL_ERROR("Couldn't read file %s!", files_.at(pcl_counter_).c_str());
      
      pcl::toROSMsg(*cloud, *msg);
      msg->header.frame_id=frame_id_;
      msg->header.stamp=this->now();
      pclPub_->publish(*msg);
      if(debug_)
        RCLCPP_INFO(get_logger(), "Publishing a point cloud from file %s", files_.at(pcl_counter_).c_str());	    
	    ++pcl_counter_;
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pclPub_{nullptr};
    
    std::string frame_id_;
    std::string dir_;
    std::string file_ext_;
    std::vector<std::string> files_;
    
    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    std::chrono::milliseconds period_;
    double freq_;
    size_t pcl_counter_;
    
    bool debug_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PclPublisher>());
  rclcpp::shutdown();
  return 0;
}
