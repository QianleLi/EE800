#ifndef SIMPLE_LAYER_H_
#define SIMPLE_LAYER_H_

#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <simple_layers/MPvector.h>
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>
 
namespace simple_layer_namespace
{
 
class SimpleLayer : public costmap_2d::Layer
{
public:
  SimpleLayer();
 
  virtual void onInitialize();
  virtual void updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y, double* max_x,
                             double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
private:
  std::vector<boost::shared_ptr<message_filters::SubscriberBase> > observation_subscribers_;  ///< @brief Used for the observation message filters
  std::vector<boost::shared_ptr<tf2_ros::MessageFilterBase> > observation_notifiers_;  ///< @brief Used to make sure that transforms are available for each sensor
  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
  void MPReceived(const simple_layers::MPvector::ConstPtr& msg);
  double minX = 99;
  double maxX = 0;
  double minY = 99;
  double maxY = 0;
  bool update_bound = false;
  std::string global_frame_;
  std::vector<geometry_msgs::Point> MPs;  //Map points in the camera coordinates
  std::vector<geometry_msgs::Point> new_MPs;  //Map points in the gazebo environment
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
};
}
#endif
