#include <planner_algorithm/mid_end.hpp>

bool OriTraj::getOriTraj(      const Eigen::Matrix3d initS,
                               const Eigen::Matrix3d finalS,
                               const std::vector<Eigen::Vector3d> &Q,
                               Eigen::VectorXd T,
                               std::vector<Eigen::Vector3d> acc_list,
                               std::vector<Eigen::Matrix3d> rot_list,
                               const int N,
                               Trajectory<TRAJ_ORDER> &traj,
                               Eigen::VectorXd& opt_x)
{

  debug_publisher::DBSendNew("mid_end", "getOriTraj start");
  pieceN      = N;
  temporalDim = N;
  spatialDim  = 3 * (N - 1);
  initState   = initS;
  finalState  = finalS;
  minco.setConditions(initState, finalState, pieceN);
  Eigen::VectorXd x(temporalDim + spatialDim);
  Eigen::Map<Eigen::VectorXd> tau(x.data(), temporalDim);
  Eigen::Map<Eigen::VectorXd> xi(x.data() + temporalDim, spatialDim);

  iter = 0;

  backwardT(T, tau);
  ref_points.resize(3, N - 1);
  accelerations.resize(3, N - 1);
  double tk = T(0);
  att_constraints.clear();
  att_constraints = rot_list;
  for (int i = 0; i < N - 1; ++i)
  {
    xi.segment(3 * i, 3)  = Q[i];
    ref_points.col(i)         = Q[i];
    accelerations.col(i)  = acc_list[i];
    
    
    tk += T(i + 1);
  }

  gradByPoints.resize(3, N - 1);
  gradByPoints.setZero();

  double final_cost = 0.0;
 
  lbfgs::lbfgs_parameter_t lbfgs_params;
  lbfgs_params.mem_size       = conf.mem_size;
  lbfgs_params.past           = conf.past;
  lbfgs_params.min_step       = conf.min_step;
  lbfgs_params.g_epsilon      = conf.g_epsilon;
  lbfgs_params.max_iterations = 100000;
  lbfgs_params.delta          = conf.relCostTolMidEnd; 

  int ret = lbfgs::lbfgs_optimize(x,
                                  final_cost,
                                  &costFunction,
                                  nullptr,
                                  &earlyExit,
                                  this,
                                  lbfgs_params);

  debug_publisher::DBSendNew("mid_end", "getOriTraj finish");
  if (ret >= 0)
  {
    forwardT(tau, times); // tao--->T
    forwardP(xi, points);
    minco.setConditions(initState, finalState, pieceN);
    minco.setParameters(points, times);
    minco.getTrajectory(traj);
    std::cout << "[Optimization] Optimization Success. Final Cost = "
              << final_cost
              << std::endl;
    opt_x = x;
    return true;
  }
  else
  {
    //   traj.clear();
    forwardT(tau, times); // tao--->T
    forwardP(xi, points);
    minco.setConditions(initState, finalState, pieceN);
    minco.setParameters(points, times);
    minco.getTrajectory(traj);
    std::cout << "\x33"
              << "[Optimization] Optimization Failed: "
              << lbfgs::lbfgs_strerror(ret)
              << " code = " << ret
              << std::endl;
    return false;
  }
  return true;
}








void OriTraj::drawDebugTraj()
{
  visualization_msgs::Marker line_strip;
  line_strip.header.frame_id = "map";
  line_strip.header.stamp = ros::Time::now();
  line_strip.type   = visualization_msgs::Marker::LINE_STRIP;
  line_strip.action = visualization_msgs::Marker::ADD;
  line_strip.id = 125 + 8000;

  line_strip.color.r = 1;
  line_strip.color.g = 0;
  line_strip.color.b = 0;
  line_strip.color.a = 1;
  line_strip.scale.x = 0.05 / 2;

  geometry_msgs::Point pt;
  double dur = step_traj.getDurations().sum();
  for (double i = 0; i < dur - 1e-4; i += 0.1)
  {
    Eigen::Vector3d dur_p = step_traj.getPos(i);
    pt.x = dur_p(0);
    pt.y = dur_p(1);
    pt.z = dur_p(2);
    line_strip.points.push_back(pt);
  }
  debug_pub.publish(line_strip);

  Eigen::VectorXd T = step_traj.getDurations();
  double t = T(0);
  const double alpha = 0.0;

}