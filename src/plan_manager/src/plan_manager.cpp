#include <plan_manager/plan_manager.hpp>
#include <utils/Visualization.hpp>
#include <thread>

using namespace std;
using namespace ros;
using namespace Eigen;

#define USE_MIASTAR 1
#define USE_SE3_RRT 2
#define USE_DYNASTAR 3

int main(int argc, char **argv)
{

  ros::init(argc, argv, "plan_manager");
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv("~");

  debug_publisher::init(nh_);
  ros::Duration(0.5).sleep();

  debug_publisher::DBSendNew("plan_manager", "Program start");
  planner_manager.reset(new PlannerManager);
  planner_manager->init(nh_, nh_priv);

  debug_assistant.reset(new DebugAssistant);
  debug_assistant->init(nh_, planner_manager);
  debug_publisher::DBSendNew("plan_manager", "Program init done!");

  double test_rate = planner_manager->config.testRate;
  if (test_rate > 0.0)
  {
    ros::Rate lr(test_rate);
    while (ros::ok())
    {
      planner_manager->process();
      ros::spinOnce();
      lr.sleep();
    }
  }
  else
  {
    ros::spin();
  }
  ros::spin();
  debug_assistant->callback_thread.join();
  return 0;
}
void PlannerManager::resetrandommap(pcl::PointCloud<pcl::PointXYZ> &global_map_pcl_cloud)
{
  double x, y, z;
  double len, wid, hig;
  double cloud_resolution = 0.6;
  geneWall(0, 0, 0.2, 0.2, 3.0, global_map_pcl_cloud);
  geneWall(60, 60, 35.0, 0.2, 0.2, 3.0, global_map_pcl_cloud);

  double num = 200;
  srand(time(NULL));
  pcl::PointXYZ s_point;
  for (int i = 0; i < num; i++)
  {
    x = (rand() % 400 + 100) / 10;
    y = (rand() % 400 + 100) / 10;
    z = (rand() % 250 + 50) / 10;

    len = 2.0 * cloud_resolution;
    wid = 2.0 * cloud_resolution;
    hig = 2.0 * cloud_resolution;

    geneWall(x, y, z, len, wid, hig, global_map_pcl_cloud);
  }
}

void PlannerManager::geneWall(double ori_x, double ori_y, double ori_z, double length, double width, double height, pcl::PointCloud<pcl::PointXYZ> &global_map_pcl_cloud, double tor)
{
  pcl::PointXYZ s_point;
  double cloud_resolution = 0.6;
  Eigen::Vector3d pos_eva, gradp_rel;
  double tf = 0.0;
  for (double t_z = ori_z; t_z < height + ori_z; t_z += cloud_resolution)
  {
    for (double t_y = ori_y; t_y < ori_y + width; t_y += cloud_resolution)
    {
      for (double t_x = ori_x; t_x < ori_x + length; t_x += cloud_resolution)
      {
        s_point.x = t_x + (rand() % 10) / 250.0;
        s_point.y = t_y + (rand() % 10) / 250.0;
        s_point.z = t_z + (rand() % 10) / 800.0;
        pos_eva = Eigen::Vector3d{s_point.x, s_point.y, s_point.z};
        if (planner_manager->sv_manager->getSDFofSweptVolume<false>(pos_eva, tf, gradp_rel, false) <= tor)
        {
          std::cout << "continue" << std::endl;
          continue;
        }
        global_map_pcl_cloud.push_back(s_point);
      }
    }
  }
}

void PlannerManager::geneWall(double ori_x, double ori_y, double length, double width, double height, pcl::PointCloud<pcl::PointXYZ> &global_map_pcl_cloud, double tor)
{
  pcl::PointXYZ s_point;
  double cloud_resolution = 0.6;
  Eigen::Vector3d pos_eva, gradp_rel;
  double tf = 0.0;
  for (double t_z = 0.0; t_z < height; t_z += cloud_resolution)
  {
    for (double t_y = ori_y; t_y < ori_y + width; t_y += cloud_resolution)
    {
      for (double t_x = ori_x; t_x < ori_x + length; t_x += cloud_resolution)
      {
        s_point.x = t_x + (rand() % 10) / 250.0;
        s_point.y = t_y + (rand() % 10) / 250.0;
        s_point.z = t_z + (rand() % 10) / 800.0;
        pos_eva = Eigen::Vector3d{s_point.x, s_point.y, s_point.z};
        if (planner_manager->sv_manager->getSDFofSweptVolume<false>(pos_eva, tf, gradp_rel, false) <= tor)
        {
          std::cout << "continue" << std::endl;
          continue;
        }

        global_map_pcl_cloud.push_back(s_point);
      }
    }
  }
}

void PlannerManager::init(ros::NodeHandle &nh, ros::NodeHandle &nh_prev)
{
  debug_publisher::DBSendNew("plan_manager", "planner_manager init start");
  config.loadParameters(nh_prev);

  bdx = config.kernel_size * config.occupancy_resolution; // 自身bounding box的长宽高
  bdy = config.kernel_size * config.occupancy_resolution; // 自身bounding box的长宽高
  bdz = config.kernel_size * config.occupancy_resolution; // 自身bounding box的长宽高

  pcsmap_manager.reset(new PCSmapManager(config));
  pcsmap_manager->init(nh);
  debug_publisher::DBSendNew("plan_manager", "pcsmap_manager init done");

  sv_manager.reset(new SweptVolumeManager(config));
  sv_manager->init(nh, config);
  debug_publisher::DBSendNew("plan_manager", "sv_manager init done");

  astar_searcher.reset(new AstarPathSearcher);
  astar_searcher->init(nh);


  debug_publisher::DBSendNew("plan_manager", "front_end init done");

  traj_parlength = 3.0;
  minco_traj_optimizer.reset(new TrajOptimizer);
  minco_traj_optimizer->setParam(nh, config);
  minco_traj_optimizer->setEnvironment(sv_manager);

  debug_publisher::DBSendNew("plan_manager", "back_end init done");

  ori_traj_generator.reset(new OriTraj);
  ori_traj_generator->setParam(nh, config);

  current_front_end = USE_MIASTAR;

  visulizer.reset(new vis::Visualization(nh));
  target_sub = nh.subscribe("/goal", 1, &PlannerManager::targetRcvCallBack, this);
  rcvmap_signal_sub = nh.subscribe("/rcvmap_signal", 1, &PlannerManager::mapRcvCallBack, this);
  setting_sub = nh.subscribe("/setting_receive", 1, &PlannerManager::settingRcvCallBack, this);
  // vis_timer = nh.createTimer(ros::Duration(1), &PlannerManager::viscallback, this); // 定期清除或者重新可视化
  rs = nh.subscribe("/reshow", 1, &PlannerManager::reShowTraj, this);

  debug_publisher::DBSendNew("plan_manager", "planner_manager init done");
}

void PlannerManager::viscallback(const ros::TimerEvent &event)
{
  // std::cout<<"visualization"<<std::endl;
  // minco_traj_optimizer->renderAABBpoints();
}
bool PlannerManager::generatePath(Vector3d start, Vector3d end)
{
  if (current_front_end == USE_MIASTAR)
  {
    astar_searcher->AstarPathSearch(start, end);
    if (astar_searcher->success_flag)
    {
      recent_path = astar_searcher->getPath();
      visulizer->visR3Path("front_end_path", recent_path);
      recent_se3_path = astar_searcher->getastarSE3Path();
      visulizer->visSE3Path("SE3path", sv_manager->getRobotShapeMesh(), recent_se3_path);
      ROS_WARN("[A*] search success.");
    }
    else
    {
      ROS_WARN("[A*] search failed.");
    }
    astar_searcher->reset();
    return (astar_searcher->success_flag);
  }
}

void PlannerManager::generateTraj(const vector<Vector3d> &path)
{
  int N;
  int path_size = path.size();
  double temp_traj_parlength = traj_parlength;
  int index_gap = ceil(temp_traj_parlength / ((pcsmap_manager->occupancy_map)->grid_resolution));

  while (index_gap >= path_size - 1)
  {
    temp_traj_parlength /= 1.5;
    index_gap = ceil(temp_traj_parlength / ((pcsmap_manager->occupancy_map)->grid_resolution));
  }

#ifdef USE_DYN_A_STAR
  index_gap = 1;
#endif

  bool ret_opt;
  Matrix3d initState = Matrix3d::Zero();
  Matrix3d finalState = Matrix3d::Zero();
  initState.col(0) = path.front();
  finalState.col(0) = path.back();

  vector<Vector3d> Q;
  vector<Vector3d> acc_list;
  Vector3d wp;
  Vector3d tmp_pos{999, 999, 999};
  Vector3d dir;
  Matrix3d rotate;
  vector<Matrix3d> rotatelist;
  minco_traj_optimizer->pcsmap_manager->aabb_points.clear();
  const Eigen::Vector3d offset = Eigen::Vector3d(config.offsetAABBbox[0], config.offsetAABBbox[1], config.offsetAABBbox[2]);
  cout << "\033[32mOffset for AABB box:\033[0m" << offset.transpose() << std::endl;
  for (int ind = index_gap; ind < path_size - 1; ind += index_gap)
  {
    wp = path[ind];
    rotate = recent_se3_path[ind].getRotMatrix();
    dir = rotate * Eigen::Vector3d(0, 0, 1);
    Q.push_back(wp);
    rotatelist.push_back(rotate);
    acc_list.push_back(dir);
    minco_traj_optimizer->pcsmap_manager->getPointsInAABBOutOfLastOne(wp, tmp_pos, bdx / 3.0, bdy / 3.0, bdz / 3.0, offset);
    tmp_pos = wp;
  }
  minco_traj_optimizer->parallel_points.clear();
  for (const auto &pair : minco_traj_optimizer->pcsmap_manager->aabb_points)
  {
    minco_traj_optimizer->parallel_points.push_back(pair.second); // 便于多线程加速,每次可能不一样？
  }
  minco_traj_optimizer->parallel_points_num = minco_traj_optimizer->pcsmap_manager->aabb_points.size();
  cout << "\033[32m=========parallel_points_num:==========\033[0m" << minco_traj_optimizer->parallel_points_num << endl;

  minco_traj_optimizer->lastTstar = vector<double>(minco_traj_optimizer->parallel_points_num, 0.0);

  N = Q.size() + 1;

  Matrix3Xd inPts = MatrixXd::Zero(3, N);
  for (int i = 0; i < N - 1; i++)
  {
    inPts.col(i) = Q[i];
  }
  VectorXd ts = config.inittime * VectorXd::Ones(N);
  bool ret, ret2;
  bool mid_ret;
  ros::Time t1;

  visulizer->visSE3Vec("path_vec", Q, acc_list, 156468);
  Eigen::VectorXd opt_x;
  mid_ret = ori_traj_generator->getOriTraj(initState, finalState, Q, ts, acc_list, rotatelist, N, recent_traj, opt_x);

  sv_manager->updateTraj(recent_traj);
  clear(); 
  std::cout << "[planner manager] Ori trajectory generated successfully!" << std::endl;

 
  if (mid_ret)
  {
    minco_traj_optimizer->integration_sequential_ms = 0.0;
    minco_traj_optimizer->integration_parallel_ms = 0.0;
    minco_traj_optimizer->discrete_sequential_ms = 0.0;
    minco_traj_optimizer->discrete_parallel_ms = 0.0;
    minco_traj_optimizer->cost_iter = 0;
    t1 = ros::Time::now();

    cout << "\033[32m=========使用lmbm优化========\033[0m" << endl;
    cout << "\033[32m=========使用lmbm优化========\033[0m" << endl;
    cout << "\033[32m=========使用lmbm优化========\033[0m" << endl;
    ret = minco_traj_optimizer->optimize_traj_lmbm(initState, finalState, opt_x, N, recent_traj);

    cout << "\033[32m最终parallel Integration total time s:\033[0m" << minco_traj_optimizer->integration_parallel_ms / 1000.0 << endl;
    cout << "\033[32m最终parallel Integration average time ms:\033[0m" << minco_traj_optimizer->integration_parallel_ms / (minco_traj_optimizer->cost_iter) << endl;
    cout << "\033[32m最终parallel Discrete total time s:\033[0m" << minco_traj_optimizer->discrete_parallel_ms / 1000.0 << endl;
    cout << "\033[32m最终parallel Discrete average time ms:\033[0m" << minco_traj_optimizer->discrete_parallel_ms / (minco_traj_optimizer->cost_iter) << endl;
  }

  if (1)
  {
    std::cout << "[planner manager] Trajectory optimization is successful! " << std::endl;
    ros::Time t2 = ros::Time::now();
    std::cout << "Optimization time cost = " << (t2 - t1).toSec() * 1000 << " ms" << std::endl;
    visulizer->visTraj("traj", recent_traj);
    std::cout << " tmax for traj: " << recent_traj.getTotalDuration() << std::endl;
    sv_manager->updateTraj(recent_traj);

    if (sv_manager->isTrajCollide())
    {
      ROS_WARN("traj collide.");
    }
    sv_manager->setTrajStamp(ros::Time::now().toSec());
    sv_manager->process(recent_traj);
  }
}

  // SE3 front_end ver.
  void PlannerManager::generateTraj(const vector<SE3State> &path)
  {
    debug_publisher::DBSendNew("plan_manager", "Try to generate trajectory based on R3 path.");
    int N;
    int path_size = path.size();
    double temp_traj_parlength = traj_parlength;
    int index_gap = ceil(temp_traj_parlength / ((pcsmap_manager->occupancy_map)->grid_resolution));

    while (index_gap >= path_size - 1)
    {
      temp_traj_parlength /= 1.5;
      index_gap = ceil(temp_traj_parlength / ((pcsmap_manager->occupancy_map)->grid_resolution));
    }

    bool ret_opt;
    Matrix3d initState = Matrix3d::Zero();
    Matrix3d finalState = Matrix3d::Zero();
    initState.col(0) = path.front().position;
    finalState.col(0) = path.back().position;

    vector<Vector3d> Q;
    Vector3d wp;
    for (int ind = index_gap; ind < path_size - 1; ind += index_gap)
    {
      wp = path[ind].position;
      Q.push_back(wp);
    }

    N = Q.size() + 1;

    Matrix3Xd inPts = MatrixXd::Zero(3, N);
    for (int i = 0; i < N; i++)
    {
      inPts.col(i) = Q[i];
    }
    VectorXd ts = config.inittime * VectorXd::Ones(N);

    minco::MINCO_S3NU minimal_jerk;
    minimal_jerk.setConditions(initState, finalState, N);
    minimal_jerk.setParameters(inPts, ts);
    minimal_jerk.getTrajectory(recent_traj);

    sv_manager->updateTraj(recent_traj);
    clear();

    std::cout << "[planner manager] Ori trajectory generated successfully!" << std::endl;
    debug_publisher::DBSendNew("plan_manager", "Ori trajectory generated.");

    ros::Time t1 = ros::Time::now();

    bool ret;

    ret = minco_traj_optimizer->optimize_traj_lmbm(initState, finalState, Q, ts, N, recent_traj);

    if (ret)
    {
      debug_publisher::DBSendNew("plan_manager", "Final trajectory optimization success!");
    }
    else
    {
      debug_publisher::DBSendNew("plan_manager", "Final trajectory optimization fail!");
    }
    std::cout << "[planner manager] Trajectory optimization is done! " << std::endl;
    ros::Time t2 = ros::Time::now();
    std::cout << "Optimization time cost = " << (t2 - t1).toSec() * 1000 << " ms" << std::endl;
    visulizer->visTraj("traj", recent_traj);
    sv_manager->updateTraj(recent_traj);
    if (sv_manager->isTrajCollide())
    {
      ROS_WARN("traj collide.");
    }

    sv_manager->setTrajStamp(ros::Time::now().toSec());
    sv_manager->process(recent_traj);
  }

  void PlannerManager::clear()
  {
    planner_manager->sv_manager->swept_cal->sweptvolumeclear();
  }

  void PlannerManager::mapRcvCallBack(const std_msgs::Empty &msg)
  {
    debug_publisher::DBSendNew("plan_manager", "Try to gene byte kernel");
    uint8_t *map_kernel = pcsmap_manager->generateMapKernel(config.kernel_size);
    debug_publisher::DBSendNew("plan_manager", "gene byte kernel done!");

    int sizeX = pcsmap_manager->occupancy_map->X_size;
    int sizeY = pcsmap_manager->occupancy_map->Y_size;
    int sizeZ = pcsmap_manager->occupancy_map->Z_size;
    sv_manager->setMapKernel(map_kernel, sizeX, sizeY, sizeZ);
    debug_publisher::DBSendNew("plan_manager", "set byte kernel done!");
    astar_searcher->initGridMap(pcsmap_manager, sv_manager);
    minco_traj_optimizer->setGridMap(pcsmap_manager);
    cout << "init map A* --" << endl;
  }

  void PlannerManager::settingRcvCallBack(const std_msgs::Int8MultiArray &msg)
  {
#define CLEAR_MAP 1
#define SETTING_FRONT_END 2
#define SETTING_SHAPE 3

    if (msg.data.size() < 2)
    {
      return;
    }
    int head = msg.data[0];

    if (head == CLEAR_MAP)
    {
      pcsmap_manager->clearMap();
      std::cout << "[planner manager] clear map!" << std::endl;
    }
  }
  /**
   * @attention 记得更新可视化clear清除一些变量等等
   */
  void PlannerManager::targetRcvCallBack(const geometry_msgs::PoseStamped &msg)
  {
    clear();
    if (step_state == STEP_NOPOINT)
    {
      start_pos(0) = msg.pose.position.x;
      start_pos(1) = msg.pose.position.y;
      start_pos(2) = msg.pose.position.z;
      step_state = STEP_HAVE_START;
      std::cout << "[plan_manager] Get start position! " << std::endl;
      debug_publisher::DBSendNew("plan_manager", "Get start position!");
    }
    else if (step_state == STEP_HAVE_START)
    {
      end_pos(0) = msg.pose.position.x;
      end_pos(1) = msg.pose.position.y;
      end_pos(2) = msg.pose.position.z;
      step_state = STEP_HAVE_TARGET;
      std::cout << "[plan_manager] Get target position! " << std::endl;

      debug_publisher::DBSendNew("plan_manager", "Get target position!");
    }
    else if (step_state == STEP_HAVE_TARGET)
    {
      start_pos(0) = msg.pose.position.x;
      start_pos(1) = msg.pose.position.y;
      start_pos(2) = msg.pose.position.z;
      step_state = STEP_HAVE_START;
      std::cout << "[plan_manager] Get start position! " << std::endl;
      debug_publisher::DBSendNew("plan_manager", "Get target position!");
    }

    if (step_state == STEP_HAVE_TARGET)
    {
      working = true;
      std::cout << "[plan_manager] Try to generate path. " << std::endl;
      debug_publisher::DBSendNew("plan_manager", "Try to generate path.");
      if (generatePath(start_pos, end_pos))
      {
        std::cout << "[plan_manager] Path generated successfully! " << std::endl;
        debug_publisher::DBSendNew("plan_manager", "Path generated successfully!");

        if (current_front_end == USE_MIASTAR || current_front_end == USE_DYNASTAR)
        {
          generateTraj(recent_path);
        }

        if (current_front_end == USE_SE3_RRT)
        {
          generateTraj(recent_se3_path);
        }
      }
      working = false;
    }
  }

  void PlannerManager::process()
  {
    this->sv_manager->process(recent_traj);
  }

  void PlannerManager::reShowTraj(const std_msgs::Empty::Ptr msg)
  {
    sv_manager->setTrajStamp(ros::Time::now().toSec());
    sv_manager->process(recent_traj);
  }

  const uint8_t bit_sel[8] = {0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01};
  void DebugAssistant::debugMsgcallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
  {

    planner_manager->sv_manager->swept_cal->sweptvolumeclear();

    if (msg->data.size() < 1)
    {
      return;
    }

    // stop optimization
    if (msg->data[0] == 21)
    {
      planner_manager->minco_traj_optimizer->exit = true;
    }

    // pause
    if (msg->data[0] == 22)
    {
      planner_manager->minco_traj_optimizer->pause = msg->data[1];
    }

    // 渲染0状态kernel
    if (msg->data[0] == 101)
    {
      Config conf = planner_manager->config;
      vector<Vector3d> kernel;
      int xkernel_size = floor(2 * conf.kernel_max_roll / conf.kernel_ang_res) + 1;
      int ykernel_size = floor(2 * conf.kernel_max_pitch / conf.kernel_ang_res) + 1;
      int zero_pose_i = (xkernel_size - 1) / 2;
      int zero_pose_j = (ykernel_size - 1) / 2;

      Vector3d offset = Vector3d(-1, -1, -1);
      double res = conf.occupancy_resolution;

      kernel.clear();

      for (size_t x = 0; x < conf.kernel_size; x++)
      {
        for (size_t y = 0; y < conf.kernel_size; y++)
        {
          for (size_t z = 0; z < conf.kernel_size; z++)
          {
            if (planner_manager->sv_manager->current_robot_shape->byte_shape_kernels[zero_pose_i * ykernel_size + zero_pose_j].getOccupied(x, y, z) == true)
            // if (planner_manager -> sv_manager -> current_robot_shape -> byte_shape_kernels[zero_pose_i*ykernel_size + zero_pose_j].getOccupied(x,y,z) == true)
            // if (planner_manager -> sv_manager -> current_robot_shape -> shape_kernels[zero_pose_i*ykernel_size + zero_pose_j].getOccupied(x,y,z) == true)//这部分ok，从shape_kernels到byte_shape_kernels有问题
            {
              kernel.push_back(offset + Vector3d(x, y, z) * res);
            }
          }
        }
      }
      planner_manager->visulizer->visPointcloudByVector(kernel, "zero_kernel_vis");
    }
    // 可视化扫略体积
    if (msg->data[0] == 102)
    {
      if (planner_manager->recent_path.size() == 0)
      {
        std::cout << "Error no trajectory now" << std::endl;
        return;
      }
      planner_manager->sv_manager->swept_cal->sweptvolumeclear();
      Eigen::MatrixXd U_;
      Eigen::MatrixXi G_;
      planner_manager->sv_manager->calculateSwept(U_, G_);
      return;
    }
    // reshow
    if (msg->data[0] == 103)
    {
      if (planner_manager->recent_path.size() == 0)
      {
        std::cout << "Error no trajectory now" << std::endl;
        return;
      }
      planner_manager->sv_manager->setTrajStamp(ros::Time::now().toSec());
      planner_manager->sv_manager->process(planner_manager->recent_traj);
      return;
    }

    // 渲染map kernel
    if (msg->data[0] == 104)
    {

      Config conf = planner_manager->config;
      vector<Vector3d> kernel;
      int side_size = (conf.kernel_size - 1) / 2;
      int X_size = planner_manager->pcsmap_manager->occupancy_map->X_size + 2 * side_size;
      int Y_size = planner_manager->pcsmap_manager->occupancy_map->Y_size + 2 * side_size;
      int Z_size = planner_manager->pcsmap_manager->occupancy_map->Z_size + 2 * side_size;
      int len_of_last_dim = (Z_size + 7) / 8;

      double res = conf.occupancy_resolution;
      Vector3d ofsb = Vector3d::Ones() * side_size;

      kernel.clear();

      for (size_t x = 0; x < X_size; x++)
      {
        for (size_t y = 0; y < Y_size; y++)
        {
          for (size_t z = 0; z < Z_size; z++)
          {
            int byteIDx = x * Y_size * len_of_last_dim + y * len_of_last_dim + (z) / 8;
            int offset = z % 8;
            if (planner_manager->sv_manager->map_kernel[byteIDx] & bit_sel[offset])
            {
              std::cout << byteIDx << std::endl;
              kernel.push_back((Vector3d(x, y, z) - ofsb) * res);
            }
          }
        }
      }
      planner_manager->visulizer->visPointcloudByVector(kernel, "zero_kernel_vis");
    }
    // 重新绘制SE3path
    if (msg->data[0] == 105)
    {
      if (planner_manager->recent_path.size() == 0)
      {
        std::cout << "Error no trajectory now" << std::endl;
        return;
      }
      planner_manager->visulizer->visSE3Path("SE3path", planner_manager->sv_manager->getRobotShapeMesh(), planner_manager->recent_se3_path);
    }

    // 从轨迹生成地图
    if (msg->data[0] == 113)
    {

      std_msgs::Int8MultiArray clear_map;
      clear_map.data.push_back(CLEAR_MAP);
      clear_map.data.push_back(2);
      planner_manager->settingRcvCallBack(clear_map);

      planner_manager->global_map_pcl_cloud.clear();

      planner_manager->geneWall(0, 0, 0.2, 0.2, 3.0, planner_manager->global_map_pcl_cloud);
      planner_manager->geneWall(50, 50, 32.0, 0.2, 0.2, 3.0, planner_manager->global_map_pcl_cloud);
      //===================================================================================================
      double x = (int)msg->data[3];
      double y = (int)msg->data[2];
      double z = (int)msg->data[1];
      double tor = msg->data[4]; // 控制空洞的safe horizon

      std::cout << "=========x===========:" << x << std::endl;
      std::cout << "=========y===========:" << y << std::endl;
      std::cout << "=========z===========:" << z << std::endl;
      planner_manager->geneWall(x, 0.0, 2.0, 50.0, 35.0, planner_manager->global_map_pcl_cloud, tor);
      planner_manager->geneWall(y, 0.0, 2.0, 50.0, 35.0, planner_manager->global_map_pcl_cloud, tor);
      planner_manager->geneWall(z, 0.0, 2.0, 50.0, 35.0, planner_manager->global_map_pcl_cloud, tor);
      //     // planner_manager->geneWall(10.0, 0.0, 2.0, 50.0, 35.0, planner_manager->global_map_pcl_cloud);
      //     // planner_manager->geneWall(25.0, 0.0, 2.0, 50.0, 35.0, planner_manager->global_map_pcl_cloud);
      //     // planner_manager->geneWall(40.0, 0.0, 2.0, 50.0, 35.0, planner_manager->global_map_pcl_cloud);

      // //======================================================后期这部分不可视化======================================================
      planner_manager->geneWall(0.0, 49.0, 50.0, 1.0, 35.0, planner_manager->global_map_pcl_cloud, tor);      // 墙封住，避免前段取巧
      planner_manager->geneWall(0.0, 0.0, 34.0, 50.0, 50.0, 1.0, planner_manager->global_map_pcl_cloud, tor); // 封住天花板,避免前段取巧

      // 这里有double free的bug===============================
      //  planner_manager->geneWall(0.0, 0.0, -1.0,50.0, 50.0, 1.0,planner_manager->global_map_pcl_cloud); //封住地下,避免前段取巧

      sensor_msgs::PointCloud2 global_map_cloud;
      pcl::toROSMsg(planner_manager->global_map_pcl_cloud, global_map_cloud);
      planner_manager->pcsmap_manager->rcvGlobalMapHandler(global_map_cloud);
    }
  }

  void DebugAssistant::init(ros::NodeHandle & nh, PlannerManager::Ptr pm)
  {

    debug_publisher::DBSendNew("plan_manager", "debug_assistant init start");

    planner_manager = pm;
    debug_publisher::DBSendNew("plan_manager", "debug_assistant init done");

    callback_thread = (std::thread([&]()
                                   {
    ros::NodeHandle private_nh("~");  
    ros::Subscriber sub2 = private_nh.subscribe("/debug_cmd", 10, &DebugAssistant::debugMsgcallback,this);
    ros::Rate lr(10);
  
    while (ros::ok())
    {
      ros::spinOnce();  
      lr.sleep();
    } }));
  }