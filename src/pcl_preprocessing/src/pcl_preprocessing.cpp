//General
#include <cstdio>
#include <string>

//CV
//#include <opencv2/highgui/highgui.hpp>

//ROS2
#include "rclcpp/rclcpp.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>

//PCL
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>

class PclPreprocessing : public rclcpp::Node
{
	public:
		PclPreprocessing(char * argv[])
		: Node("pcl_preprocessor")
		{
			this->declare_parameter("crop", true);
			this->declare_parameter("detectFloor", false);
			this->declare_parameter("downSample", true);
			this->declare_parameter("debug", false);
			this->declare_parameter("synchronised", false);
			this->declare_parameter("input_topic", "/livox/lidar");
			this->declare_parameter("output_cloud_topic", "/filteredPcl");
			this->declare_parameter("output_map_topic", "/localMap");

			this->declare_parameter("filter_x_min", 0.1);
			this->declare_parameter("filter_y_min", -10.0);
			this->declare_parameter("filter_z_min", -0.2);
			this->declare_parameter("filter_x_max", 25.0);
			this->declare_parameter("filter_y_max", 10.0);
			this->declare_parameter("filter_z_max", 2.0);

			this->declare_parameter("voxel_grid_x_res", 0.025);
			this->declare_parameter("voxel_grid_y_res", 0.025);
			this->declare_parameter("voxel_grid_z_res", 0.025);
			this->declare_parameter("map_frame", "map");
				
			crop = this->get_parameter("crop").as_bool();
			detectFloor = this->get_parameter("detectFloor").as_bool();
			downSample = this->get_parameter("downSample").as_bool();
			synchronised = this->get_parameter("synchronised").as_bool();
			debug = this->get_parameter("debug").as_bool();

			minX = this->get_parameter("filter_x_min").as_double();
			minY = this->get_parameter("filter_y_min").as_double();
			minZ = this->get_parameter("filter_z_min").as_double();
			maxX = this->get_parameter("filter_x_max").as_double();
			maxY = this->get_parameter("filter_y_max").as_double();
			maxZ = this->get_parameter("filter_z_max").as_double();

			VGxRes = this->get_parameter("voxel_grid_x_res").as_double();
			VGyRes = this->get_parameter("voxel_grid_y_res").as_double();
			VGzRes = this->get_parameter("voxel_grid_z_res").as_double();

			mapFrame = this->get_parameter("map_frame").as_string();
			
			pclSub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
				this->get_parameter("input_topic").as_string(), 10, std::bind(&PclPreprocessing::lidar_callback, this, std::placeholders::_1));
			publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(this->get_parameter("output_cloud_topic").as_string(), 10);
			mapPub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(this->get_parameter("output_map_topic").as_string(), 10);
			tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
			transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
		}
	private:
	    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){

	    	pcl::PCLPointCloud2 cloud_pc2;
	    	//pcl_conversions::toPCL(*msg, cloud_pc2);
	    	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	    	//pcl::fromPCLPointCloud2(cloud_pc2,*cloud);
	    	pcl::fromROSMsg(*msg, *cloud);
	    	

	    	// Cloud cropping
	    	if(crop){
	    		if(debug)
	    			RCLCPP_INFO(get_logger(), "Cropping...");
				//pcl::PointCloud<pcl::PointXYZ>::Ptr crop (new pcl::PointCloud<pcl::PointXYZ>);
				pcl::CropBox<pcl::PointXYZ> boxFilter;
				boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
				boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
				boxFilter.setInputCloud(cloud);
				//boxFilter.setNegative(true);
				//boxFilter.filter(*crop);
				boxFilter.setNegative(false);
				boxFilter.filter(*cloud);
				if(debug)
					RCLCPP_INFO(get_logger(), "Cropped...");
			}
			if(detectFloor){
				// Data processing
				pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
				pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
				// Create the segmentation object
				pcl::SACSegmentation<pcl::PointXYZ> seg;
				// Optional
				seg.setOptimizeCoefficients(true);
				// Mandatory
				seg.setModelType(pcl::SACMODEL_PLANE);
				seg.setMethodType(pcl::SAC_RANSAC);
				seg.setDistanceThreshold (0.1);

				seg.setInputCloud(cloud);
				seg.segment(*inliers, *coefficients);

				// Create the filtering object
				pcl::ExtractIndices<pcl::PointXYZ> extract;
				// Extract the inliers
				extract.setInputCloud(cloud);
				extract.setIndices(inliers);
				//pcl::PointCloud<pcl::PointXYZ>::Ptr ground (new pcl::PointCloud<pcl::PointXYZ>);
				//extract.setNegative (false);
				//extract.filter (*ground);
				extract.setNegative (true);
				extract.filter(*cloud);

				if(debug)
					RCLCPP_INFO(get_logger(),
						"Detected floor parameters: %lf, %lf, %lf, %lf.\n",
						coefficients->values[0], coefficients->values[1], 
						coefficients->values[2], coefficients->values[3]);
			}

			if(downSample){
				if(debug)
					RCLCPP_INFO(get_logger(), "Downsampling...");
				pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>);
				pcl::VoxelGrid<pcl::PointXYZ> VG;
				VG.setInputCloud(cloud);
				VG.setLeafSize(VGxRes, VGyRes, VGzRes);
				VG.filter(*cloud);
				if(debug){
					RCLCPP_INFO(get_logger(), "Downsampled cloud has: %d data points.\n", cloud->size());
					RCLCPP_INFO(get_logger(), "Downsampled...");
				}
			}

			//auto message = sensor_msgs::msg::PointCloud2::SharedPtr();
			sensor_msgs::msg::PointCloud2::SharedPtr message(new sensor_msgs::msg::PointCloud2);
			//toPCLPointCloud2 (*cloud, cloud_pc2);	//void   toPCLPointCloud2 (const pcl::PointCloud<PointT>& cloud, pcl::PCLPointCloud2& msg)
			//pcl_conversions::fromPCL(cloud_pc2, *message);	//void 	fromPCL (const pcl::PCLPointCloud2 &pcl_pc2, sensor_msgs::PointCloud2 &pc2)
			pcl::toROSMsg( *cloud, *message);
			if(debug)
				RCLCPP_INFO(get_logger(), "Publishing cloud of size of %d points", cloud->size());
			publisher_->publish(*message);


			// Occupancy grid
			// Set meta-data
			nav_msgs::msg::OccupancyGrid::SharedPtr localMap(new nav_msgs::msg::OccupancyGrid);
			//localMap->header.seq = 1;
			localMap->header.frame_id = mapFrame;
			localMap->info.origin.position.z = 0;
			localMap->info.origin.orientation.w = 1;
			localMap->info.origin.orientation.x = 0;
			localMap->info.origin.orientation.y = 0;
			localMap->info.origin.orientation.z = 0;
			//localMap->header.stamp.sec = rclcpp::Time::now().sec;
			//localMap->header.stamp.nsec = rclcpp::Time::now().nsec;
			//localMap->info.map_load_time = rclcpp::Time::now();
			localMap->info.resolution = VGxRes;
			localMap->info.height = (maxY - minY) / VGyRes;
			localMap->info.width = (maxX) / VGxRes;
			localMap->info.origin.position.x = 0;
			localMap->info.origin.position.y = minY;
			std::string toFrameRel = mapFrame;
			std::string fromFrameRel = msg->header.frame_id;
			geometry_msgs::msg::TransformStamped transformStamped;
			try {
				transformStamped = tf_buffer_->lookupTransform(
					toFrameRel, fromFrameRel,tf2::TimePointZero);
				}
			catch (tf2::TransformException & ex) {
				RCLCPP_INFO(get_logger(), "Could not transform %s to %s: %s", toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
			}
			
			geometry_msgs::msg::PoseStamped map_pose;
			map_pose.header = msg->header;
			map_pose.pose = localMap->info.origin;
			tf2::doTransform(map_pose,map_pose,transformStamped);
			localMap->info.origin = map_pose.pose;

			// create and fill the map
			int mapRows = (int)(maxX/VGxRes);
			int mapCols = (int)((maxY-minY)/VGyRes);
			std::vector<int8_t> map(mapRows * mapCols, 0);
			if(debug)
				RCLCPP_INFO(get_logger(), "Map has %d rows and %d columns.", mapRows, mapCols);
			for (const auto& point: cloud->points){
				if(debug)
    				RCLCPP_INFO(get_logger(), "Point x = %lf, y = %lf, z = %lf", point.x, point.y, point.z);
		      	//xcoord = (int)((x - minX) / cellResolution);
		      	//ycoord = (int)((y - minY) / cellResolution);
		      	//RCLCPP_INFO(get_logger(), "[] = %d", ((int)((point.y - minY) / VGlatRes)) * mapRows + ((int)((point.x - minX) / VGlatRes)));
		      	map[((int)((point.y - minY) / VGyRes)) * mapRows + ((int)((point.x - minX) / VGxRes))] = 100;
			}
			
			// Assign map and publish
			localMap->data = map;
			if(debug)
				RCLCPP_INFO(get_logger(), "Publishing localMap");
			mapPub_->publish(*localMap);
		}

	    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pclSub_{nullptr};
	    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_{nullptr};
	    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mapPub_{nullptr};
	    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  		std::unique_ptr<tf2_ros::Buffer> tf_buffer_{nullptr};
	    bool synchronised;  
	    bool crop, detectFloor, downSample;
	    bool debug;
		double minX, minY, minZ, maxX, maxY, maxZ;
		double VGxRes, VGyRes, VGzRes;
		std::string mapFrame;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PclPreprocessing>(argv));
  rclcpp::shutdown();
  return 0;
}
