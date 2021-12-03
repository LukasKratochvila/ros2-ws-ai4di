//General
#include <cstdio>
#include <string>
#include <vector>

//CV
//#include <opencv2/highgui/highgui.hpp>

//ROS2
#include "rclcpp/rclcpp.hpp"
//#include "pcl_conversions/pcl_conversions.h"
#include "cv_bridge/cv_bridge.h"
//#include "sensor_msgs/msg/image.hpp"
//#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/srv/get_map.hpp"
//#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
//#include <tf2/exceptions.h>

#include <chrono>

class MapLookup : public rclcpp::Node
{
  public:
    MapLookup()
    : Node("map_lookup")
    {
      synchronised = false;
      boxSub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
      					(std::string)"/detections",
      					10,
      					std::bind(&MapLookup::bbox_callback, this, std::placeholders::_1));
      boxPub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/unknownObstacles", 10);
      mapPub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/ObstacleMap", 10);
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    	transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    	mapClient_ = this->create_client<nav_msgs::srv::GetMap>("/map_server/map");
    	
    	while(!mapClient_->wait_for_service(std::chrono::seconds(3))){
    	if (!rclcpp::ok()) {
    	   RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
    	   return;
        }
	RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }
    	RCLCPP_INFO(this->get_logger(), "Map lookup node has started.");
    }
    void setMap(nav_msgs::msg::OccupancyGrid map){
    	if (map_ == nullptr)
    	   map_ = std::shared_ptr<nav_msgs::msg::OccupancyGrid>(new nav_msgs::msg::OccupancyGrid);
    	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setting map");
    	map_->header=map.header;
    	map_->info=map.info;
    	map_->data=map.data;
    }

  private:
    void bbox_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
    {
    	double cx, cy, dx, dy;
    	
	//RCLCPP_INFO(this->get_logger(), "Recieved %d MarkerArray.", msg->markers.size()/2);
	if (map_ == nullptr){
	    RCLCPP_INFO(this->get_logger(), "No map.");
	    return;
	}
	std::string fromFrameRel = msg->markers[0].header.frame_id;
        std::string toFrameRel = "mapOrigin";
        geometry_msgs::msg::TransformStamped transformStamped;
        try {
          transformStamped = tf_buffer_->lookupTransform(
          toFrameRel, fromFrameRel,
          tf2::TimePointZero);
        }
        catch (tf2::TransformException & ex) {
          RCLCPP_INFO(get_logger(), "Could not transform %s to %s: %s", toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
        }

    	geometry_msgs::msg::PointStamped point_stamped;
    	point_stamped.header = msg->markers[0].header;
    	point_stamped.point.z = 0;
    	std::vector<geometry_msgs::msg::PointStamped> PointVector;
    	nav_msgs::msg::OccupancyGrid ObstacleMap;
    	ObstacleMap.header = map_->header;
    	ObstacleMap.info = map_->info;
    	ObstacleMap.data = map_->data;

    	//nav_msgs::msg::OccupancyGrid map;
    	/*RCLCPP_INFO(this->get_logger(), "Requesting map.");
    	//auto request = rclcpp::Client<nav_msgs::srv::GetMap>::SharedRequest();
    	auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();
    	
//      if(!mapClient_->service_is_ready()){
//            RCLCPP_INFO(this->get_logger(), "service not available, stopping request...");
//           return;
//      }
//    	nav_msgs::srv::GetMap srv;
//    	mapClient_->call(srv);
//    	map = srv.Response.map;

    	auto future = mapClient_->async_send_request(request);
    	
        auto status = future.wait_for(std::chrono::seconds(3));

        if (status == std::future_status::ready) //future.valid())//
    	   RCLCPP_INFO(this->get_logger(), "Map recieved.");
        else {
	   RCLCPP_ERROR(this->get_logger(), "Map unavailable.");
	   //return;
	}
        
        RCLCPP_ERROR(this->get_logger(), "Read Map.");
    	map = future.get()->map;
    	RCLCPP_ERROR(this->get_logger(), "Map readed.");*/
    	
    	visualization_msgs::msg::MarkerArray outmsg;

    	for(size_t i = 0; i < msg->markers.size(); ++(++i)){
    		cx = msg->markers[i].pose.position.x;
    		cy = msg->markers[i].pose.position.y;
    		dx = msg->markers[i].scale.x;
    		dy = msg->markers[i].scale.y;

    		//size_t cnt = 0;
    		PointVector.clear();
    		for(auto x = -dx/2; x <= dx/2; x += dx/4){
    			for(auto y = -dy/2; y <= dy/2; y += dy/4){
    				point_stamped.point.x = cx + x + map_->info.origin.position.x, point_stamped.point.y = cy + y + map_->info.origin.position.y;
    				//RCLCPP_INFO(this->get_logger(), "BT Point: x = %lf, y = %lf",point_stamped.point.x, point_stamped.point.y);
    				tf2::doTransform(point_stamped,point_stamped,transformStamped);
    				//point_stamped = tf_buffer_->transform(point_stamped, "mapOrigin"/*map_->header.frame_id*/);
    				//RCLCPP_INFO(this->get_logger(), "AT Point: x = %lf, y = %lf",point_stamped.point.x, point_stamped.point.y);
    				PointVector.push_back(point_stamped);
    				//RCLCPP_INFO(this->get_logger(), "Pushing point %d [%lf, %lf, %lf]",++cnt, point_stamped.point.x, point_stamped.point.y, point_stamped.point.z);
    			}
    		}

    		int accumulator = 0;
    		size_t index = 0;
    		for (const auto& point: PointVector){
    			//map->info.resolution = VGlatRes;
		  		//map->info.height = (maxY - minY) / VGlatRes;
		  		//map->info.width = (maxX) / VGlatRes;
		  		//RCLCPP_INFO(this->get_logger(), "Map Res:%lf W:%d H:%d S:%d", map_->info.resolution, map_->info.width, map_->info.height, map_->data.size());
		  	index = (size_t)((size_t)((size_t)(point.point.y / map_->info.resolution) * map_->info.width) + (size_t)(point.point.x / map_->info.resolution));
    			accumulator += (int8_t)map_->data[index];
    			ObstacleMap.data[index] = 100;
    			//ObstacleMap.data[1] = 100;
    		//RCLCPP_INFO(this->get_logger(), "Point: x = %lf, y = %lf",point.point.x, point.point.y);
    		//RCLCPP_INFO(this->get_logger(), "MapValue: %d, index %zu, %d, %d, %d",(int8_t)map_->data[index], index, map_->info.width, map_->info.height, map_->data.size());
    		}
    		//RCLCPP_INFO(this->get_logger(), "A: %d, V:%lf", accumulator, (double)0.2 * 100 * PointVector.size());
    		if(accumulator <= 0.2 * 100 * PointVector.size()){
    			outmsg.markers.push_back(msg->markers[i]);
    			outmsg.markers.push_back(msg->markers[i+1]);
    		}

    	}

    	boxPub_->publish(outmsg);
    	mapPub_->publish(ObstacleMap);
      std::stringstream log;
      log << "Publishing: " << outmsg.markers.size()/2 << " detections.";
      RCLCPP_INFO(this->get_logger(), log.str());
		  
    }
    
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr boxSub_{nullptr};
    rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr mapClient_{nullptr};
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr boxPub_{nullptr};
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mapPub_{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_{nullptr};
    std::shared_ptr<nav_msgs::msg::OccupancyGrid> map_{nullptr};

    bool synchronised;

};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<MapLookup> node = std::make_shared<MapLookup>();

  auto client = node->create_client<nav_msgs::srv::GetMap>("/map_server/map");
  auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();
  
  while (!client->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }
  rclcpp::sleep_for(std::chrono::seconds(1));
  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(node->get_logger(), "Map recieved.");
    node->setMap(result.get()->map);
  } else {
    RCLCPP_ERROR(node->get_logger(), "Failed to call service GetMap");
    return 1;
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
