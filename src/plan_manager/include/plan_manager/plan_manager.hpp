#ifndef PLANNER_MANAGER_H
#define PLANNER_MANAGER_H

#include <planner_algorithm/front_end_Astar.hpp>
#include <planner_algorithm/back_end_optimizer.hpp>
#include <planner_algorithm/mid_end.hpp>
#include <utils/se3_state.hpp>
#include <map_manager/PCSmap_manager.h>
#include <swept_volume/sw_manager.hpp>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8MultiArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <utils/config.hpp>
#include <utils/debug_publisher.hpp>

using namespace Eigen;


class PlannerManager
{
  public:
    PlannerManager(){};
    ~PlannerManager(){
    };

    void clear();
    double bdx;
    double bdy;
    double bdz;
    bool use_lbfgs{true};//
    bool working = false;
    bool write_res = false;

    pcl::PointCloud<pcl::PointXYZ> global_map_pcl_cloud;
    

    void init( ros::NodeHandle& nh, ros::NodeHandle& nh_prev );

    void mapRcvCallBack(const std_msgs::Empty& msg);
    void settingRcvCallBack(const std_msgs::Int8MultiArray& msg);
    void targetRcvCallBack(const geometry_msgs::PoseStamped& msg);
    void viscallback(const ros::TimerEvent & event);
    void resetrandommap(pcl::PointCloud<pcl::PointXYZ>&global_map_pcl_cloud);
    void geneWall(double ori_x , double ori_y , double length, double width,double height,pcl::PointCloud<pcl::PointXYZ>&global_map_pcl_cloud,double tor=1.3);
    void geneWall(double ori_x , double ori_y ,double ori_z, double length, double width,double height,pcl::PointCloud<pcl::PointXYZ>&global_map_pcl_cloud,double tor=1.3);
    
    void reShowTraj( const std_msgs::Empty::Ptr msg);


    /**
     * 调用前端算法生成路径
     * @param start      起点
     * @param end        终点
     * @return 成功则返回True
     */
    bool generatePath( Vector3d start, Vector3d end );

    /**
     * 调用后端算法生成最终轨迹
     * @param path      R3的路径点
     */
    void generateTraj(const vector<Vector3d>& path );

    /**
     * 调用后端算法生成最终轨迹
     * @param path      SE3的路径点
     */
    void generateTraj(const vector<SE3State>& path );


    /**
     * traj_server 可视化进程
     * @attention 可能需要改进
     */
    void process();
    

    // params
    double traj_parlength{1.0};    

    // stream control
    #define STEP_NOPOINT      0
    #define STEP_HAVE_START   1
    #define STEP_HAVE_TARGET  2

    int step_state = STEP_NOPOINT;

    //odom
    nav_msgs::Odometry recent_odom;
    bool has_odom;

    // setting
    int current_front_end;

    // planning
    vector<Vector3d> recent_path;
    vector<SE3State> recent_se3_path;
    vector<double> recent_se3_path_roll;
    vector<double> recent_se3_path_pitch;
    Vector3d start_pos;
    Vector3d end_pos;
    Trajectory<TRAJ_ORDER> recent_traj;

    // ros
    ros::Subscriber target_sub;           
    ros::Subscriber odom_sub;             
    ros::Subscriber rcvmap_signal_sub;    
    ros::Subscriber setting_sub;          
    ros::Subscriber rs;                   
    ros::Timer vis_timer;
    ros::Publisher traj_pub;
    
    // visualization
    Visualization::Ptr visulizer;

    // map
    PCSmapManager::Ptr pcsmap_manager;

    // front_end
    AstarPathSearcher::Ptr    astar_searcher;


    // back_end
    OriTraj::Ptr ori_traj_generator;
    TrajOptimizer::Ptr minco_traj_optimizer;
    SweptVolumeManager::Ptr sv_manager;
    Config config;


  
  public:
    typedef shared_ptr<PlannerManager> Ptr;
};


class DebugAssistant
{
  public:
    DebugAssistant(){};
    ~DebugAssistant(){};
    std::thread callback_thread;
    void init(ros::NodeHandle& nh, PlannerManager::Ptr pm);
    void debugMsgcallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    
  
  private:
    ros::Subscriber debug_sub;
    PlannerManager::Ptr planner_manager;
  
  public:
    typedef std::shared_ptr<DebugAssistant> Ptr;
};


PlannerManager::Ptr planner_manager;
DebugAssistant::Ptr debug_assistant;

#endif