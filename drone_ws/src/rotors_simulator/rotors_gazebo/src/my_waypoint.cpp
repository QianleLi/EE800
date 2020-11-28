//This programs shows messages from /turtle1/pose on the screen.
#include <fstream>
#include <iostream>

#include <Eigen/Geometry>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Transform.h>
#include <list>
#include <string>
using namespace std;

geometry_msgs::Transform robot_pose;
//A callback function Excuted each time a new pose message arrives
//要发送的数据与接收到的数据结构略有不同
void poseMessageReceived(const gazebo_msgs::ModelStates& msg){//这里还需要加入对string name 的判断
//需要对string name列表中的模型名称进行判断，获得机器人在列表中的index，然后再把pose列表中对应索引的数据取出来
	vector<string> model_name = msg.name;
	vector<string>::iterator iter;
	iter = find(model_name.begin(),model_name.end(),"mbot");
	int index = iter - model_name.begin();
	//auto index = std::distance(std::begin(msg.name), iter);
	robot_pose.translation.x = msg.pose[index].position.x;
	robot_pose.translation.y = msg.pose[index].position.y;
	robot_pose.translation.z = 3.0;
	//ROS_INFO("Pose: x: %f, y: %f, z: %f", robot_pose.translation.x, robot_pose.translation.y, robot_pose.translation.z);
	robot_pose.rotation = msg.pose[index].orientation;
	//ROS_INFO("Orientation: x: %f, y: %f, z: %f, w: %f", robot_pose.rotation.x, robot_pose.rotation.y, robot_pose.rotation.z, robot_pose.rotation.w);
}

int main(int argc,char **argv){
	//Initialize the ROS system and become a node
	ros::init(argc,argv,"synchronization");
	ros::NodeHandle nh;
	
	//Create a subscriber and a publisher object.
	ros::Subscriber sub = nh.subscribe("/gazebo/model_states",1,&poseMessageReceived);
	ros::Publisher wp_pub =nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(mav_msgs::default_topics::COMMAND_TRAJECTORY, 1);
	
	while (ros::ok()) {
	//创建一个发送的
	trajectory_msgs::MultiDOFJointTrajectoryPtr msg(new trajectory_msgs::MultiDOFJointTrajectory);
    ros::spinOnce();
    msg->header.stamp = ros::Time::now();
	msg->joint_names.push_back("base_link");
	trajectory_msgs::MultiDOFJointTrajectoryPoint current_point;
	current_point.transforms.push_back(robot_pose);
	msg->points.push_back(current_point);
	wp_pub.publish(msg);
    ros::Duration(0.1).sleep();
  }
}
