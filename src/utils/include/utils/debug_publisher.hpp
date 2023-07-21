#ifndef DEBUG_PUBLISHER_HPP
#define DEBUG_PUBLISHER_HPP

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <string>
#include <thread>
namespace debug_publisher
{

void DBSendNew(const std::string& title, const std::string& message);
void DBSendOptiStep(const std::vector<double>& step);
void DBSendLogCost(const std::vector<double>& log_cost);

void init(ros::NodeHandle& nh);

};
#endif