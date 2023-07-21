#include <planner_algorithm/back_end_optimizer.hpp>

int TrajOptimizer::optimize_traj_lmbm(const Eigen::Matrix3d &initS,
                                  const Eigen::Matrix3d &finalS,
                                  const std::vector<Eigen::Vector3d> &Q,
                                  const Eigen::VectorXd &T,
                                  const int N,
                                  Trajectory<TRAJ_ORDER> &traj)
{
  total_opt_time = 0.0;
  total_sdf_time = 0.0;
  total_AABB_time = 0.0;
  ros::Time start_time = ros::Time::now();
  pieceN = N;
  temporalDim = N;
  spatialDim = 3 * (N - 1);
  initState = initS;
  finalState = finalS;
  minco.setConditions(initState, finalState, pieceN);
  const int total_opt_variable_num = temporalDim + spatialDim;

  x_variable = (double *)malloc(sizeof(double) * total_opt_variable_num);

  Eigen::Map<Eigen::VectorXd>(x_variable, temporalDim) = T;
  for (int i = 0; i < N - 1; ++i)
  {
    Eigen::Map<Eigen::VectorXd>(x_variable + temporalDim + 3 * i, 3) = Q[i]; 
  }

  iter = 0;
  std::vector<double> clear_flag;
  clear_flag.push_back(-1);
  debug_publisher::DBSendOptiStep(clear_flag);
  debug_publisher::DBSendLogCost(clear_flag);


  cout << "\033[33m====lmbm====\033[0m" << endl;
  cout << "\033[33m====x =====\033[0m" << endl;
  for (size_t i = 0; i < total_opt_variable_num; i++)
  {
    cout<<x_variable[i]<<" ,";
  }
  cout<<std::endl;

  double final_cost;
  lmbm::lmbm_parameter_t param;
  int ret = lmbm::lmbm_optimize(total_opt_variable_num,
                                x_variable,
                                &final_cost,
                                costFunctionLmbm,
                                this,
                                earlyexitLmbm,
                                &param);
  {
    printf("-----------\n");
    printf("| Output: |\n");
    printf("-----------\n");
    printf("%-16s %f\n", "Final value:", final_cost);
  }

  if (ret >= 0)
  {
    forwardT(x_variable, times, temporalDim); 
    forwardP((x_variable + temporalDim), points, spatialDim / 3);
    minco.setConditions(initState, finalState, pieceN);
    minco.setParameters(points, times);
    minco.getTrajectory(traj);
    std::cout << "[Optimization LMBM] Optimization Success. Final Cost = "
              << final_cost
              << std::endl;
    if (x_variable != NULL)
    {
      free(x_variable);
      x_variable = NULL;
    }
    return ret;
  }
  else
  {
    forwardT(x_variable, times, temporalDim); // tao--->T
    forwardP((x_variable + temporalDim), points, spatialDim / 3);
    minco.setConditions(initState, finalState, pieceN);
    minco.setParameters(points, times);
    minco.getTrajectory(traj);
    std::cout << "\x33"
              << "[Optimization LMBM] Optimization Failed: "
              << " code = " << ret
              << std::endl;
    if (x_variable != NULL)
    {
      free(x_variable);
      x_variable = NULL;
    }
    return ret;
  }

  return ret;
}
int TrajOptimizer::optimize_traj_lmbm(const Eigen::Matrix3d &initS,
                                  const Eigen::Matrix3d &finalS,
                                  const Eigen::VectorXd &opt_x,
                                  const int N,
                                  Trajectory<TRAJ_ORDER> &traj)
{
  total_opt_time = 0.0;
  total_sdf_time = 0.0;
  total_AABB_time = 0.0;
  ros::Time start_time = ros::Time::now();
  pieceN = N;
  temporalDim = N;
  spatialDim = 3 * (N - 1);
  initState = initS;
  finalState = finalS;
  minco.setConditions(initState, finalState, pieceN);
  const int total_opt_variable_num = temporalDim + spatialDim;
  x_variable = (double *)malloc(sizeof(double) * total_opt_variable_num);
  Eigen::Map<Eigen::VectorXd>(x_variable, total_opt_variable_num) = opt_x;
  iter = 0;
  std::vector<double> clear_flag;
  clear_flag.push_back(-1);
  debug_publisher::DBSendOptiStep(clear_flag);
  debug_publisher::DBSendLogCost(clear_flag);

  double final_cost;
  lmbm::lmbm_parameter_t param;
  int ret = lmbm::lmbm_optimize(total_opt_variable_num,
                                x_variable,
                                &final_cost,
                                costFunctionLmbm,
                                this,
                                earlyexitLmbm,
                                &param);
  {
    printf("-----------\n");
    printf("| Output: |\n");
    printf("-----------\n");
    printf("%-16s %f\n", "Final value:", final_cost);
  }

  if (ret >= 0)
  {
    forwardT(x_variable, times, temporalDim); // tao--->T
    forwardP((x_variable + temporalDim), points, spatialDim / 3);
    minco.setConditions(initState, finalState, pieceN);
    minco.setParameters(points, times);
    minco.getTrajectory(traj);
    std::cout << "[Optimization LMBM] Optimization Success. Final Cost = "
              << final_cost
              << std::endl;
    if (x_variable != NULL)
    {
      free(x_variable);
      x_variable = NULL;
    }
    return ret;
  }
  else
  {
    forwardT(x_variable, times, temporalDim); // tao--->T
    forwardP((x_variable + temporalDim), points, spatialDim / 3);
    minco.setConditions(initState, finalState, pieceN);
    minco.setParameters(points, times);
    minco.getTrajectory(traj);
    std::cout << "\x33"
              << "[Optimization LMBM] Optimization Failed: "
              << " code = " << ret
              << std::endl;
    if (x_variable != NULL)
    {
      free(x_variable);
      x_variable = NULL;
    }
    return ret;
  }

  return ret;
}
// #endif

void TrajOptimizer::drawDebug()
{
  vis->visTraj("step_traj", step_traj, 114515);
}

void TrajOptimizer::renderAABBpoints()
{

  vector<Vector3d> aabbpoints;
  for (const auto &b : pcsmap_manager->aabb_points)
  {
    aabbpoints.emplace_back(b.second);
  }
  vis->visPointcloudByVector(aabbpoints, "aabbpoints");

}



void TrajOptimizer::clearDebugVector()
{
  visualization_msgs::Marker clear_sig;
  clear_sig.header.frame_id = "map";
  clear_sig.id = 1540;
  clear_sig.type = visualization_msgs::Marker::SPHERE;
  clear_sig.action = visualization_msgs::Marker::DELETEALL;
  debug_vec_pub.publish(clear_sig);
  debug_wp_pub.publish(clear_sig);
}

void TrajOptimizer::clearvisAABBpoints()
{
  visualization_msgs::MarkerArray mk;
  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::DELETEALL;
  mk.markers.emplace_back(marker);
  debug_wplists_pub.publish(mk);
}

