#include <simple_layers/simple_layer.h>
#include <pluginlib/class_list_macros.h>
#include <simple_layers/MPvector.h>
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <tf2_ros/message_filter.h>
#include <costmap_2d/costmap_math.h>
#include <cmath>
 
PLUGINLIB_EXPORT_CLASS(simple_layer_namespace::SimpleLayer, costmap_2d::Layer)
 
using costmap_2d::LETHAL_OBSTACLE;
 
namespace simple_layer_namespace
{
 
SimpleLayer::SimpleLayer() {}
 
void SimpleLayer::onInitialize()
{
	ros::NodeHandle nh("~/" + name_), g_nh;
	//ros::Subscriber sub = g_nh.subscribe("current_mapPoints",1,&SimpleLayer::MPReceived, this);
	//ROS_INFO("Initialize the layer");
	std::string topics_string;
	nh.param("observation_sources", topics_string, std::string("current_mapPoints"));
	ROS_INFO("Subscribed to Topics: %s", topics_string.c_str());
	ros::NodeHandle source_node(nh, topics_string);

    // get the parameters for the specific topic
    double observation_keep_time, expected_update_rate, min_obstacle_height, max_obstacle_height;
    std::string topic, sensor_frame, data_type;
    bool inf_is_valid, clearing, marking;
	global_frame_ = layered_costmap_->getGlobalFrameID();
    source_node.param("topic", topic, topics_string);
    source_node.param("sensor_frame", sensor_frame, std::string(""));
    source_node.param("observation_persistence", observation_keep_time, 5.0);
    source_node.param("expected_update_rate", expected_update_rate, 0.0);
    source_node.param("data_type", data_type, std::string("simple_layers::MPvector"));//
    source_node.param("min_obstacle_height", min_obstacle_height, 0.0);
    source_node.param("max_obstacle_height", max_obstacle_height, 2.0);
    source_node.param("inf_is_valid", inf_is_valid, false);
    source_node.param("clearing", clearing, true);
    source_node.param("marking", marking, true);
    
    boost::shared_ptr < message_filters::Subscriber<simple_layers::MPvector>
          > sub(new message_filters::Subscriber<simple_layers::MPvector>(g_nh, topic, 1));

	boost::shared_ptr<tf2_ros::MessageFilter<simple_layers::MPvector> > filter(
        new tf2_ros::MessageFilter<simple_layers::MPvector>(*sub, *tf_, global_frame_, 1, g_nh));

	filter->registerCallback(boost::bind(&SimpleLayer::MPReceived, this, _1));
	observation_subscribers_.push_back(sub);
    observation_notifiers_.push_back(filter);
    observation_notifiers_.back()->setTolerance(ros::Duration(0.05));

	current_ = true;

	dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
	dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
	  &SimpleLayer::reconfigureCB, this, _1, _2);
	dsrv_->setCallback(cb);
}
 
//Callback functions to save map points
void SimpleLayer::MPReceived(const simple_layers::MPvector::ConstPtr& msg){
	ROS_INFO("Callback once");
	std::vector <geometry_msgs::Point>().swap(MPs);
	MPs = msg->poses;
	update_bound = true;
} 

void SimpleLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void SimpleLayer::updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{//Use map points locations to update bounds

	if(update_bound){
		if (!enabled_)
			return;
		ROS_INFO("Update bounds once.");
		float x, y;
		float fx, fy;
		std::vector <geometry_msgs::Point>().swap(new_MPs);
                for(geometry_msgs::Point MP:MPs){//Coordinate transformation
			x = MP.z;
			y = -MP.x;
                        if(origin_yaw<0){
				fx = (x*cos(-origin_yaw)+y*sin(-origin_yaw));
				fy = (-x*sin(-origin_yaw)+y*cos(-origin_yaw));
                        }else{
				fx = (x*cos(origin_yaw)-y*sin(origin_yaw));
				fy = (x*sin(origin_yaw)+y*cos(origin_yaw));
			}
			if(abs(fx)<3)
				continue;
			
			fx += origin_x;
			fy += origin_y;

			geometry_msgs::Point output;
			output.x = fx;
			output.y = fy;
			new_MPs.push_back(output);
			if(minX > fx)
				minX = fx;
			if(maxX < fx)
				maxX = fx;
			if(minY > fy)
				minY = fy;
			if(maxY < fy)
				maxY = fy;
		}
		*min_x = std::min(*min_x, minX);
		*min_y = std::min(*min_y, minY);
		*max_x = std::max(*max_x, maxX);
		*max_y = std::max(*max_y, maxY);
                update_bound = false;
	}
	
}
 
void SimpleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{	
		if (!enabled_)
			return;
		
		unsigned int mx;
		unsigned int my;
		for(geometry_msgs::Point MP:new_MPs){
			
                        if(master_grid.worldToMap(MP.x, MP.y, mx, my)){
                                master_grid.setCost(mx, my, LETHAL_OBSTACLE);
			}
		}
}
 
} // end namespace
