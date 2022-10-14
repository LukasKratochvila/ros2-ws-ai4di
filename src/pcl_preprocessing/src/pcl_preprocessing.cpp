// General
#include <cstdio>
#include <string>
#include <vector>

// CV
//#include <opencv2/highgui/highgui.hpp>

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "cv_bridge/cv_bridge.h"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/srv/get_map.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>

// PCL
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

#include <pcl/common/transforms.h>

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"

class PclPreprocessing : public nav2_util::LifecycleNode //: public rclcpp::Node
{
public:
	PclPreprocessing()
		: nav2_util::LifecycleNode("pcl_preprocessor") // Node("pcl_preprocessor")
	{
		declare_parameter("crop", true);
		declare_parameter("detectFloor", false);
		declare_parameter("downSample", true);
		declare_parameter("mapCrop", true);
		declare_parameter("enableAgg", true);
		declare_parameter("createLocalMap", true);
		declare_parameter("input_topic", "/livox/lidar");
		declare_parameter("output_cloud_topic", "/filteredPcl");
		declare_parameter("output_map_topic", "/localMap");

		declare_parameter("filter_x_min", 0.1);
		declare_parameter("filter_y_min", -10.0);
		declare_parameter("filter_z_min", -0.2);
		declare_parameter("filter_x_max", 25.0);
		declare_parameter("filter_y_max", 10.0);
		declare_parameter("filter_z_max", 2.0);

		declare_parameter("voxel_grid_x_res", 0.025);
		declare_parameter("voxel_grid_y_res", 0.025);
		declare_parameter("voxel_grid_z_res", 0.025);

		declare_parameter("mapTreshold", 200);
		declare_parameter("agg_window", 1e9);
		declare_parameter("map_frame", "map");

		declare_parameter("costmap_node_name", "pcl_preprocessing_costmap");
		declare_parameter("local_costmap_namespace", "pcl_preprocessing_costmap");

		declare_parameter("debug", false);

		// The costmap node is used in the implementation of the node
		costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
			get_parameter("costmap_node_name").as_string(), 
			std::string{get_namespace()}, 
			get_parameter("local_costmap_namespace").as_string());
		// Launch a thread to run the costmap node
		costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);
	}

protected:
	nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override
	{
		auto node = shared_from_this();
		costmap_ros_->on_configure(state);

		get_parameter("crop", crop);
		get_parameter("detectFloor", detectFloor);
		get_parameter("downSample", downSample);
		get_parameter("mapCrop", mapCrop);
		get_parameter("enableAgg", enableAgg);
		get_parameter("createLocalMap", createLocalMap);

		get_parameter("filter_x_min", minX);
		get_parameter("filter_y_min", minY);
		get_parameter("filter_z_min", minZ);
		get_parameter("filter_x_max", maxX);
		get_parameter("filter_y_max", maxY);
		get_parameter("filter_z_max", maxZ);

		get_parameter("voxel_grid_x_res", VGxRes);
		get_parameter("voxel_grid_y_res", VGyRes);
		get_parameter("voxel_grid_z_res", VGzRes);

		get_parameter("mapTreshold", mapTreshold);
		get_parameter("agg_window", window_); // 1e9; //1 sec in nanosecond
		get_parameter("map_frame", mapFrame);

		get_parameter("debug", debug);

		pclSub_ = create_subscription<sensor_msgs::msg::PointCloud2>(get_parameter("input_topic").as_string(), 10, std::bind(&PclPreprocessing::lidar_callback, this, std::placeholders::_1));
		publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(get_parameter("output_cloud_topic").as_string(), 10);
		if (createLocalMap)
			mapPub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(get_parameter("output_map_topic").as_string(), 10);
		tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
		transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

		if (enableAgg){
			cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
			aggregation_started_time_ = now().nanoseconds();
			agg_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(get_parameter("output_cloud_topic").as_string() + "/agg", 10);
		}
		return nav2_util::CallbackReturn::SUCCESS;
	}
	nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override
	{
		costmap_ros_->on_activate(state);
		publisher_->on_activate();
		if (createLocalMap)
			mapPub_->on_activate();
		if (enableAgg)
			agg_publisher_->on_activate();
		return nav2_util::CallbackReturn::SUCCESS;
	}
	nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override
	{
		costmap_ros_->on_deactivate(state);
		publisher_->on_deactivate();
		mapPub_->on_deactivate();
		agg_publisher_->on_deactivate();
		return nav2_util::CallbackReturn::SUCCESS;
	}
	nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override
	{
		costmap_ros_->on_cleanup(state);
		publisher_.reset();
		mapPub_.reset();
		agg_publisher_.reset();
		return nav2_util::CallbackReturn::SUCCESS;
	}
	nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override
	{
		costmap_ros_->on_shutdown(state);
		return nav2_util::CallbackReturn::SUCCESS;
	}

private:
	void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
	{
		// pcl::PCLPointCloud2 cloud_pc2;
		// pcl_conversions::toPCL(*msg, cloud_pc2);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		// pcl::fromPCLPointCloud2(cloud_pc2,*cloud);
		pcl::fromROSMsg(*msg, *cloud);


		// Cloud cropping
		if (crop)
		{
			if (debug)
				RCLCPP_INFO(get_logger(), "Cropping... minX: %f, minY: %f, minZ: %f, maxX: %f, maxY: %f, maxZ: %f", minX, minY, minZ, maxX, maxY, maxZ);
			// pcl::PointCloud<pcl::PointXYZ>::Ptr crop (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::CropBox<pcl::PointXYZ> boxFilter;
			boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
			boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
			boxFilter.setInputCloud(cloud);
			// boxFilter.setNegative(true);
			// boxFilter.filter(*crop);
			boxFilter.setNegative(false);
			boxFilter.filter(*cloud);
			if (debug)
				RCLCPP_INFO(get_logger(), "Cropped...");
		}
		if (detectFloor)
		{
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
			seg.setDistanceThreshold(0.1);

			seg.setInputCloud(cloud);
			seg.segment(*inliers, *coefficients);

			// Create the filtering object
			pcl::ExtractIndices<pcl::PointXYZ> extract;
			// Extract the inliers
			extract.setInputCloud(cloud);
			extract.setIndices(inliers);
			// pcl::PointCloud<pcl::PointXYZ>::Ptr ground (new pcl::PointCloud<pcl::PointXYZ>);
			// extract.setNegative (false);
			// extract.filter (*ground);
			extract.setNegative(true);
			extract.filter(*cloud);

			if (debug)
				RCLCPP_INFO(get_logger(),
							"Detected floor parameters: %lf, %lf, %lf, %lf.\n",
							coefficients->values[0], coefficients->values[1],
							coefficients->values[2], coefficients->values[3]);
		}

		if (downSample)
		{
			if (debug)
				RCLCPP_INFO(get_logger(), "Downsampling...");
			// pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::VoxelGrid<pcl::PointXYZ> VG;
			VG.setInputCloud(cloud);
			VG.setLeafSize(VGxRes, VGyRes, VGzRes);
			VG.filter(*cloud);
			if (debug)
			{
				RCLCPP_INFO(get_logger(), "Downsampled cloud has: %d data points.\n", cloud->size());
				RCLCPP_INFO(get_logger(), "Downsampled...");
			}
		}

		// Get transform from current cloud to map for cloud transform
		std::string fromFrameRel = msg->header.frame_id;
		std::string toFrameRel = mapFrame;
		getMap = true;
		try
		{
			transformStampedToMap = tf_buffer_->lookupTransform(
				toFrameRel, fromFrameRel, tf2_ros::fromMsg(msg->header.stamp));
			if (debug)
				RCLCPP_INFO(get_logger(), "Transforming to %s from %s.", toFrameRel.c_str(), fromFrameRel.c_str());
		}
		catch (tf2::TransformException &ex)
		{
			RCLCPP_INFO(get_logger(), "Could not transform %s to %s: %s", fromFrameRel.c_str(), toFrameRel.c_str(), ex.what());
			getMap = false;
		}
		if (getMap)
		{
			pcl::toROSMsg(*cloud, *msg);
			tf2::doTransform(*msg, *msg, transformStampedToMap); // Transform cloud into the map frame
			pcl::fromROSMsg(*msg, *cloud);
		}

		if (mapCrop)
		{
			auto costmap = costmap_ros_->getCostmap();
			if (costmap == nullptr)
				RCLCPP_ERROR(get_logger(), "Preprocessing mapCrop has no map.");
			else
			{
				if (debug)
					RCLCPP_INFO(get_logger(), "MapCropping...");

				// Set up a point bag and an extractror
				pcl::PointIndices::Ptr knownObstacle(new pcl::PointIndices());
				pcl::ExtractIndices<pcl::PointXYZ> extract;

				unsigned int mx, my;
				auto costmap = costmap_ros_->getCostmap();
				bool inMap;

				// Go through points in cloud, transform and look up in the map
				for (size_t i = 0; i < cloud->size(); i++)
				{
					// Find the bitmap index corresponding to the point in the map
					inMap = costmap->worldToMap(cloud->points[i].x, cloud->points[i].y, mx, my);
					if (!inMap) // if there is a nonzero value in the map the obstacle is known and the point is added to be removed
						knownObstacle->indices.push_back(i);
					else if (costmap->getCost(mx, my) > mapTreshold)
						knownObstacle->indices.push_back(i);
				}
				extract.setInputCloud(cloud);
				extract.setIndices(knownObstacle);
				extract.setNegative(true);
				extract.filter(*cloud);
				if (debug)
					RCLCPP_INFO(get_logger(), "Cropped %d points as known obstacles in the map.", knownObstacle->indices.size());
			}
		}
		if (!(*cloud).size())
		{
			sensor_msgs::msg::PointCloud2::SharedPtr message(new sensor_msgs::msg::PointCloud2);
			pcl::toROSMsg(*cloud, *message);
			if (debug)
				RCLCPP_INFO(get_logger(), "Publishing cloud of size of %d points", cloud->size());
			// if(debug)
			//	RCLCPP_INFO(get_logger(), "Input msg stamp:%d Output msg stamp:%d", msg->header.stamp.sec, message->header.stamp.sec);
			message->header = msg->header;
			publisher_->publish(*message);
		}
		else
		{
			RCLCPP_WARN(get_logger(), "Cloud has %d points", cloud->size());
			return;
		}

		if (enableAgg){
			uint64_t time_nano = msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;
			uint64_t diff = time_nano - aggregation_started_time_;
			if ((diff) > window_ && cloud_->size() != 0)
			{
				aggregation_started_time_ = time_nano;
				sensor_msgs::msg::PointCloud2::SharedPtr m(new sensor_msgs::msg::PointCloud2);
				pcl::toROSMsg(*cloud_, *m);
				m->header = msg->header;
				agg_publisher_->publish(*m);
				if (debug)
					RCLCPP_INFO(get_logger(), "Publishing cloud of size of %d points", cloud_->size());
				cloud_->points.clear();
			}
			else
			{
				geometry_msgs::msg::TransformStamped transformStamped;
				bool getTrans = true;
				try
				{
					transformStamped = tf_buffer_->lookupTransform(msg->header.frame_id,
																msg->header.frame_id,
																tf2_ros::fromMsg(msg->header.stamp),
																tf2::Duration(diff));
				}
				catch (tf2::TransformException &ex)
				{
					getTrans = false;
					RCLCPP_INFO(get_logger(), "Could not transform for agg in time: %s: %s", msg->header.stamp, ex.what());
				}
				if (getTrans)
				{
					Eigen::Matrix4f transform_trans = Eigen::Matrix4f::Identity();
					Eigen::Matrix4f transform_rot = Eigen::Matrix4f::Identity();
					transform_trans(0, 3) = transformStamped.transform.translation.x;
					transform_trans(1, 3) = transformStamped.transform.translation.y;
					transform_trans(2, 3) = transformStamped.transform.translation.z;
					transform_rot.topLeftCorner(3, 3) = Eigen::Matrix3f(Eigen::Quaternionf(transformStamped.transform.rotation.w,
																						transformStamped.transform.rotation.x,
																						transformStamped.transform.rotation.y,
																						transformStamped.transform.rotation.z));

					pcl::transformPointCloud(*cloud, *cloud, transform_trans * transform_rot);
					*cloud_ += *cloud;
				}
			}
		}

		if (createLocalMap){
			// Occupancy grid
			// Set meta-data
			nav_msgs::msg::OccupancyGrid::SharedPtr localMap(new nav_msgs::msg::OccupancyGrid);
			// localMap->header.seq = 1;
			localMap->header.frame_id = mapFrame;
			localMap->info.origin.position.z = 0;
			localMap->info.origin.orientation.w = 1;
			localMap->info.origin.orientation.x = 0;
			localMap->info.origin.orientation.y = 0;
			localMap->info.origin.orientation.z = 0;
			// localMap->header.stamp.sec = rclcpp::Time::now().sec;
			// localMap->header.stamp.nsec = rclcpp::Time::now().nsec;
			// localMap->info.map_load_time = rclcpp::Time::now();
			localMap->info.resolution = VGxRes;
			localMap->info.height = (maxY - minY) / VGyRes;
			localMap->info.width = (maxX) / VGxRes;
			localMap->info.origin.position.x = 0;
			localMap->info.origin.position.y = minY;

			if (getMap)
			{
				geometry_msgs::msg::PoseStamped map_pose;
				map_pose.header = msg->header;
				map_pose.pose = localMap->info.origin;
				tf2::doTransform(map_pose, map_pose, transformStampedToMap);
				localMap->info.origin = map_pose.pose;
			}

			// create and fill the map
			int mapRows = (int)(maxX / VGxRes);
			int mapCols = (int)((maxY - minY) / VGyRes);
			std::vector<int8_t> map(mapRows * mapCols, 0);
			if (debug)
				RCLCPP_INFO(get_logger(), "Map has %d rows and %d columns.", mapRows, mapCols);
			for (const auto &point : cloud->points)
			{
				// if(debug)
				// RCLCPP_INFO(get_logger(), "Point x = %lf, y = %lf, z = %lf", point.x, point.y, point.z);
				// xcoord = (int)((x - minX) / cellResolution);
				// ycoord = (int)((y - minY) / cellResolution);
				// RCLCPP_INFO(get_logger(), "[] = %d", ((int)((point.y - minY) / VGlatRes)) * mapRows + ((int)((point.x - minX) / VGlatRes)));
				int xcoord = (int)((point.x - minX) / VGxRes);
				int ycoord = (int)((point.y - minY) / VGyRes);
				//if(debug)
				//	RCLCPP_INFO(get_logger(), "Point xcoord: %d, ycoord: %d", xcoord, ycoord);
				if(xcoord > 0 && xcoord < mapRows-1 && ycoord > 0 && ycoord < mapCols-1)
					map[ycoord * mapRows + xcoord] = 100;
			}

			// Assign map and publish
			localMap->data = map;
			if (debug)
				RCLCPP_INFO(get_logger(), "Publishing localMap");
			mapPub_->publish(*localMap);
		}
	}

	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pclSub_{nullptr};
	rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_{nullptr};
	rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mapPub_{nullptr};
	std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
	std::unique_ptr<tf2_ros::Buffer> tf_buffer_{nullptr};
	bool crop, detectFloor, downSample, mapCrop, createLocalMap, enableAgg;
	unsigned char mapTreshold;
	double minX, minY, minZ, maxX, maxY, maxZ;
	double VGxRes, VGyRes, VGzRes;
	std::string mapFrame;
	bool getMap;

	bool debug;

	// agg
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_{nullptr};
	double window_;
	rcl_time_point_value_t aggregation_started_time_;
	rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr agg_publisher_{nullptr};
	geometry_msgs::msg::TransformStamped transformStampedToMap;

	// The node needs a costmap node
	std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
	std::unique_ptr<nav2_util::NodeThread> costmap_thread_;
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	std::shared_ptr<PclPreprocessing> node = std::make_shared<PclPreprocessing>();
	rclcpp::spin(node->get_node_base_interface());
	rclcpp::shutdown();
	return 0;
}
