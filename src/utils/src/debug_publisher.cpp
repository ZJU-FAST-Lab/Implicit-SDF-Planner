
#include "utils/debug_publisher.hpp"
#include <thread>
namespace debug_publisher {


    ros::Publisher news_pub;
    ros::Publisher opti_step_pub;
    ros::Publisher log_cost_pub;


    void DBSendNew(const std::string& title, const std::string& message) {
        std_msgs::String msg;
        msg.data = title + "@" + message;
        news_pub.publish(msg);
    }

    void DBSendOptiStep(const std::vector<double>& step) {
        std_msgs::Float64MultiArray msg;
        msg.data = step;
        opti_step_pub.publish(msg);
    }

    void DBSendLogCost(const std::vector<double>& log_cost) {
        std_msgs::Float64MultiArray msg;
        msg.data = log_cost;
        log_cost_pub.publish(msg);
    }

    void init(ros::NodeHandle& nh) {
        news_pub      = nh.advertise<std_msgs::String>("/debug_receive_news", 10);
        opti_step_pub = nh.advertise<std_msgs::Float64MultiArray>("/debug_receive_opti_step", 10);
        log_cost_pub  = nh.advertise<std_msgs::Float64MultiArray>("/debug_receive_log_cost", 10);
    }
}