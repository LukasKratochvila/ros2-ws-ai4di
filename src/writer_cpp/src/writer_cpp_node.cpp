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
    ImagePclSaver()
    : Node("image_pcl_saver")
    {
      this->declare_parameter("save_dir", "./");
      this->declare_parameter("img_sub_dir", "image/");
      this->declare_parameter("pcl_sub_dir", "pcl/");
      this->declare_parameter("input_img_topic", "/image");
      this->declare_parameter("input_pcl_topic", "/livox/lidar");
      this->declare_parameter("mode", "both");
      this->declare_parameter("synchronized", false);
      this->declare_parameter("time_toll", 0.01);
      this->declare_parameter("start_img_counter", 0);
      this->declare_parameter("start_pcl_counter", 0);
      this->declare_parameter("img_file_ext", "jpg");
      this->declare_parameter("pcl_file_ext", "pcd");
      this->declare_parameter("debug", true);

      mode = this->get_parameter("mode").as_string();
      synchronized = this->get_parameter("synchronized").as_bool();
      time_toll = this->get_parameter("time_toll").as_double();

      savepath = this->get_parameter("save_dir").as_string();
      pcl_dir = this->get_parameter("pcl_sub_dir").as_string();
      img_dir = this->get_parameter("img_sub_dir").as_string();
      pcl_ext = this->get_parameter("pcl_file_ext").as_string();
      img_ext = this->get_parameter("img_file_ext").as_string();
      img_counter = this->get_parameter("start_img_counter").as_int();
      pcl_counter = this->get_parameter("start_pcl_counter").as_int();

      debug = this->get_parameter("debug").as_bool();
      
      if (mode == (std::string)"img"){
	      img_timestamp_file.open(this->savepath + this->img_dir + "timestamp.csv");
	      imgSub_ = this->create_subscription<sensor_msgs::msg::Image>(
      		this->get_parameter("input_img_topic").as_string(), 10, std::bind(&ImagePclSaver::image_callback, this, std::placeholders::_1));
      }
      else if (mode == (std::string)"pcl"){
	      pcl_timestamp_file.open(this->savepath + this->pcl_dir + "timestamps.csv");
	      pclSub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      		this->get_parameter("input_pcl_topic").as_string(), 10, std::bind(&ImagePclSaver::lidar_callback, this, std::placeholders::_1));
      }
      else if (mode == (std::string)"both"){
	      img_timestamp_file.open(this->savepath + this->img_dir + "timestamps.csv");
	      imgSub_ = this->create_subscription<sensor_msgs::msg::Image>(
      		this->get_parameter("input_img_topic").as_string(), 10, std::bind(&ImagePclSaver::image_callback, this, std::placeholders::_1));

	      pcl_timestamp_file.open(this->savepath + this->pcl_dir + "timestamps.csv");
	      pclSub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      		this->get_parameter("input_pcl_topic").as_string(), 10,	std::bind(&ImagePclSaver::lidar_callback, this, std::placeholders::_1));
      }
      else{
	      RCLCPP_ERROR(get_logger(), "Wrong mode: " + mode + ".");
	      return;
      }
      RCLCPP_INFO(get_logger(), "Image_pcl_saver has started in the " + mode + " mode.");
    }

  private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg){
      std::stringstream pathStream;
      pathStream << this->savepath << this->img_dir << "frame" << std::setw(6) << std::setfill('0') << this->img_counter << "." << this->img_ext;
      if (cv::imwrite(pathStream.str(), cv_bridge::toCvShare(msg, (std::string)sensor_msgs::image_encodings::BGR8)->image)){
		      img_timestamp_file << msg->header.stamp.sec << "." << msg->header.stamp.nanosec << ", " << img_counter << std::endl;
		      RCLCPP_INFO(get_logger(), "Saving: " + pathStream.str());
		    }
	    else 
          RCLCPP_WARN(get_logger(), "FAILED in saving: " + pathStream.str());
		  ++img_counter;
    }
    
    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
      std::stringstream pathStream;
      pathStream << this->savepath << this->pcl_dir << "pcl" << std::setw(6) << std::setfill('0') << this->pcl_counter << "." << this->pcl_ext;
      int ret = pcl::io::savePCDFile(pathStream.str(), *msg);
	    if (ret == 0){
	      pcl_timestamp_file << msg->header.stamp.sec << "." << msg->header.stamp.nanosec << ", " << pcl_counter << std::endl;
	      RCLCPP_INFO(get_logger(), "Saving: " + pathStream.str());
	    }
	    else
	  	  RCLCPP_WARN(get_logger(), "FAILED in saving: " + pathStream.str());
	    ++pcl_counter;
    }
    
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imgSub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pclSub_;
    bool synchronized;
    double time_toll;
    int img_counter;
    int pcl_counter;
    
    std::string mode;

    std::string savepath;
    std::string pcl_dir;
    std::string img_dir;

    std::string pcl_ext;
    std::string img_ext;
    
    std::ofstream img_timestamp_file;
    std::ofstream pcl_timestamp_file;

    bool debug;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImagePclSaver>());
  rclcpp::shutdown();
  return 0;
}
