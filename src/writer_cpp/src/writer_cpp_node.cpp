#include <cstdio>
#include <string>
#include <iostream>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>

#include "rclcpp/rclcpp.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class ImagePclSaver : public rclcpp::Node
{
  public:
    ImagePclSaver(char *argv[])
    : Node("image_pcl_saver"), img_counter(0), pcl_counter(0)
    {
      mode = (std::string)argv[1];
      synchronised = false;
      
      if (mode == (std::string)"img")
      {
	img_timestamp_file.open(this->savepath + this->img_dir + "timestamp.csv");
	imgSub_ = this->create_subscription<sensor_msgs::msg::Image>(
      					(std::string)"image",
      					10,
      					std::bind(&ImagePclSaver::image_callback, this, std::placeholders::_1));
      }
      else if (mode == (std::string)"pcl")
      {
	pcl_timestamp_file.open(this->savepath + this->pcl_dir + "timestamps.csv");
	pclSub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      					(std::string)"livox/lidar",
      					10,
      					std::bind(&ImagePclSaver::lidar_callback, this, std::placeholders::_1));
      }
      else if (mode == (std::string)"all")
      {
	img_timestamp_file.open(this->savepath + this->img_dir + "timestamps.csv");
	imgSub_ = this->create_subscription<sensor_msgs::msg::Image>(
      					(std::string)"image",
      					10,
      					std::bind(&ImagePclSaver::image_callback, this, std::placeholders::_1));

	pcl_timestamp_file.open(this->savepath + this->pcl_dir + "timestamps.csv");
	pclSub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      					(std::string)"livox/lidar",
      					10,
      					std::bind(&ImagePclSaver::lidar_callback, this, std::placeholders::_1));
      }
      else
      {
	RCLCPP_ERROR(get_logger(), "Wrong mode: " + mode + ".");
	return;
      }
	  
      RCLCPP_INFO(get_logger(), "Image_pcl_saver has started in the " + mode + " mode.");
    }

  private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
      std::stringstream pathStream;
      pathStream << this->savepath << this->img_dir << "frame" << std::setw(6) << std::setfill('0') << this->img_counter << "." << this->img_ext;
      if (cv::imwrite(pathStream.str(), cv_bridge::toCvShare(msg, (std::string)sensor_msgs::image_encodings::BGR8)->image))
		    {
		      img_timestamp_file << msg->header.stamp.sec << "." << msg->header.stamp.nanosec << ", " << img_counter << std::endl;
		      RCLCPP_INFO(get_logger(), "Saving: " + pathStream.str());
		    }
	    else 
		    RCLCPP_WARN(get_logger(), "FAILED in saving: " + pathStream.str());
		  ++img_counter;
    }
    
    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
      std::stringstream pathStream;
      pathStream << this->savepath << this->pcl_dir << "pcl" << std::setw(6) << std::setfill('0') << this->pcl_counter << "." << this->pcl_ext;
      int ret = pcl::io::savePCDFile(pathStream.str(), *msg);
	    if (not(ret))
	    {
	      pcl_timestamp_file << msg->header.stamp.sec << "." << msg->header.stamp.nanosec << ", " << pcl_counter << std::endl;
	      RCLCPP_INFO(get_logger(), "Saving: " + pathStream.str());
	    }
	    else
	  	  RCLCPP_WARN(get_logger(), "FAILED in saving: " + pathStream.str());
	    ++pcl_counter;
    }
    
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imgSub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pclSub_;
    bool synchronised;
    int img_counter;
    int pcl_counter;
    
    std::string mode;

    std::string savepath = "./";
    std::string pcl_dir = "pcl/";
    std::string img_dir = "image/";

    std::string pcl_ext = "pcd";
    std::string img_ext = "jpg";
    
    std::ofstream img_timestamp_file;
    std::ofstream pcl_timestamp_file;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImagePclSaver>(argv));
  rclcpp::shutdown();
  return 0;
}
