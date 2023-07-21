#ifndef SW_MANAGER_HPP
#define SW_MANAGER_HPP
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/Eigen>
#include <std_msgs/Float64MultiArray.h>
#include <utils/trajectory.hpp>
#include <utils/Shape.hpp>
#include <utils/Visualization.hpp>
#include <utils/config.hpp>
#include <utils/flatness.hpp>
#include <utils/lbfgs.hpp>
#include <swept_volume/sw_calculate.hpp>
#include <map>
#include <functional>

#define TRAJ_ORDER 5
#define useScale false 


using namespace shape;
using namespace Eigen;

class SweptVolumeManager 
{
public:
    uint8_t *map_kernel;
    int map_Xsize;
    int map_Ysize;
    int map_Zsize;

    Eigen::Vector3d current_pos_eva; 
    double traj_duration;
    Eigen::Vector3d vis_sdf_point{Eigen::Vector3d::Zero()};
    Eigen::Vector3d neighbor_point{Eigen::Vector3d::Zero()};
    ros::Publisher sdf_vis_pub;
    ros::Publisher sdf_vis_clean;


private:

    double cp;
    double veps;
    double grav;
    double dh;
    double dh_over_m;
    double bdx;
    double bdy;
    double bdz;
    Trajectory<TRAJ_ORDER> traj;
    Config config;
    double eps{0.1};
    Eigen::RowVector3d p0;
    const double iso = 0.001; 
    int num_seeds{100};       

    double momentum{0.0}; 
    double trajStamp;     
    double t_min{0.0};
    double t_max{1.0};
    int xkernel_size; 
    int ykernel_size;
    int kernelsize;


    Eigen::MatrixXd X, N, B;
    Eigen::VectorXi I;
    double minx, miny, minz;
    std::vector<double> init_times;             
    std::vector<Eigen::RowVector3d> init_points;
    std::vector<Eigen::RowVector3i> init_voxels;

public:
    std::map<std::string, std::function<Generalshape *(const Config &)>> shapeConstructors = {
        {"CSG", [](const Config &conf)
         { std::cout << "\033[35m=======CSG======= "<< "\033[0m" << std::endl;
        return new CSG(conf); }},
        {"Torus", [](const Config &conf)
         { std::cout << "\033[35m=======Torus======= "<< "\033[0m" << std::endl;
        return new Torus(conf); }},
        {"Torus_big", [](const Config &conf)
         { std::cout << "\033[35m=======Torus_big======= "<< "\033[0m" << std::endl;
        return new Torus_big(conf); }},
        {"Cappedtorus", [](const Config &conf)
         { std::cout << "\033[35m=======Cappedtorus======= "<< "\033[0m" << std::endl;
        return new Cappedtorus(conf); }},
        {"Trefoil", [](const Config &conf)
         { std::cout << "\033[35m=======Trefoil======= "<< "\033[0m" << std::endl;
        return new Trefoil(conf); }},
        {"Table", [](const Config &conf)
         { std::cout << "\033[35m=======Table======= "<< "\033[0m" << std::endl;
        return new Table(conf); }},
        {"CappedCone", [](const Config &conf)
         { std::cout << "\033[35m=======CappedCone======= "<< "\033[0m" << std::endl;
        return new CappedCone(conf); }},
        {"RoundedCone", [](const Config &conf)
         { std::cout << "\033[35m=======RoundedCone======= "<< "\033[0m" << std::endl;
        return new RoundedCone(conf); }},
        {"WireframeBox", [](const Config &conf)
         { std::cout << "\033[35m=======WireframeBox======= "<< "\033[0m" << std::endl;
        return new WireframeBox(conf); }},
        {"BendLinear", [](const Config &conf)
         { std::cout << "\033[35m=======BendLinear======= "<< "\033[0m" << std::endl;
        return new BendLinear(conf); }},
        {"BendLinear_big", [](const Config &conf)
         { std::cout << "\033[35m=======BendLinear_big======= "<< "\033[0m" << std::endl;
        return new BendLinear_big(conf); }},
        {"TwistBox", [](const Config &conf)
         { std::cout << "\033[35m=======TwistBox======= "<< "\033[0m" << std::endl;
        return new TwistBox(conf); }},
        {"BendBox", [](const Config &conf)
         { std::cout << "\033[35m=======BendBox======= "<< "\033[0m" << std::endl;
        return new BendBox(conf); }},
        {"SmoothDifference", [](const Config &conf)
         { std::cout << "\033[35m=======SmoothDifference======= "<< "\033[0m" << std::endl;
        return new SmoothDifference(conf); }},    
        {"SmoothIntersection", [](const Config &conf)
        { std::cout << "\033[35m=======SmoothIntersection======= "<< "\033[0m" << std::endl;
        return new SmoothIntersection(conf); }},
        {"SmoothIntersection_big", [](const Config &conf)
        { std::cout << "\033[35m=======SmoothIntersection_big======= "<< "\033[0m" << std::endl;
        return new SmoothIntersection_big(conf); }}
         };
    sw_calculate::Ptr swept_cal; 
    Generalshape *current_robot_shape{nullptr}; 
    Visualization::Ptr vis;
    flatness::FlatnessMap flatness;

public:
    bool outputdebug{false};
    SweptVolumeManager() = delete;
    SweptVolumeManager(const Config &conf) : config(conf)
    {
        eps = conf.eps;
        xkernel_size = floor(2 * conf.kernel_max_roll / conf.kernel_ang_res) + 1;
        ykernel_size = floor(2 * conf.kernel_max_pitch / conf.kernel_ang_res) + 1;
        kernelsize = config.kernel_size;

        cp = config.parasDrag;
        veps = config.speedEps;
        grav = config.gravAcc;
        dh = config.horizDrag;
        dh_over_m = dh / config.vehicleMass;

        swept_cal.reset(new sw_calculate); 
    }
    ~SweptVolumeManager()
    {
        delete current_robot_shape;
        delete[] map_kernel;
    }
    typedef shared_ptr<SweptVolumeManager> Ptr;

public:
    /**
     * 设置map_kernel
     */
    inline void setMapKernel(uint8_t *mk, int mx, int my, int mz)
    {
        map_kernel = mk;
        map_Xsize = mx;
        map_Ysize = my;
        map_Zsize = mz;
    }

    /**
     * 计算得到init_points与init_voxels便于传入扫略体积的计算类
     * @attention 必须在选取种子之前更新轨迹的时间bounds t_min t_max
     */
    inline void getInitSeedsforSwept()
    {
        srand(100);
        igl::per_face_normals(current_robot_shape->V, current_robot_shape->F, Eigen::Vector3d(0.0, 0.0, -1.0).normalized(), N);
        igl::random_points_on_mesh(num_seeds, current_robot_shape->V, current_robot_shape->F, B, I, X); 
        init_times.push_back(0.0);
        init_times.push_back(0.0);
        init_voxels.push_back(Eigen::RowVector3i(0, 0, 0));
        for (int i = 0; i < X.rows(); i++)
        {                                    
            Eigen::RowVector3d P = X.row(i); 
            Eigen::Matrix3d VRt, Rt;
            Eigen::Vector3d xt, vt;
            Eigen::RowVector3d pos, point_velocity, normal;
            Eigen::Matrix3d St, dSt; 
            Eigen::RowVector3d candidate;
            for (double t = t_min; t <= t_max; t = t + 0.1)
            {
                getStateOnTrajStamp(t, xt, vt, Rt, VRt, St, dSt);
                pos = (St * Rt * P.transpose()).transpose() + xt.transpose();                                                      // 原始的采样点在时间t时刻新的位置
                point_velocity = (St * VRt * P.transpose()).transpose() + vt.transpose() + (dSt * Rt * P.transpose()).transpose(); // 原始的采样点在时间t时刻新的速度
                point_velocity.normalize();
                normal = (St * Rt * N.row(I(i)).transpose()).transpose();
                normal.normalize();
                if ((fabs(normal.dot(point_velocity)) < 0.05) || (normal.dot(point_velocity) < 0.0 && t == t_min) || (normal.dot(point_velocity) > 0.0 && t == t_max))
                {
                    candidate = pos + iso * normal;
                    init_points.push_back(candidate);
                    init_times.push_back(t);
                    minx = std::min(minx, candidate(0));
                    miny = std::min(miny, candidate(1));
                    minz = std::min(minz, candidate(2));
                }
            }
        }
        p0(0) = minx;
        p0(1) = miny;
        p0(2) = minz;
        Eigen::RowVector3d this_point;
        int ix, iy, iz;
        for (int s = 0; s < init_points.size(); s++)
        {
            this_point = init_points[s];                   
            ix = std::floor((this_point[0] - minx) / eps); 
            iy = std::floor((this_point[1] - miny) / eps);
            iz = std::floor((this_point[2] - minz) / eps);
            init_voxels.push_back(Eigen::RowVector3i(ix, iy, iz)); 
        }
        std::cout << "======Starting continuation with====== " << init_voxels.size() << " seeds." << std::endl;
        std::cout << "======Starting continuation with====== " << init_times.size() << " init_times." << std::endl;
    }

    /**
     * 计算扫略体积
     * @attention 保证tmin与tmax以已经更新了
     */
    inline void calculateSwept(Eigen::MatrixXd &U_, Eigen::MatrixXi &G_)
    {
        swept_cal->updatebounds(t_min, t_max);
        getInitSeedsforSwept();
        std::cout << "t_min:" << t_min << "t_max:" << t_max << std::endl;
        std::cout << "======after init ====== " << std::endl;
        std::cout << "=========use analytic method to calculate swept volume==========" << std::endl;
        swept_cal->calculation(p0, init_voxels, init_times, scalarFunc, eps, 1000000);
        std::cout << "======after calculation ====== " << std::endl;
        swept_cal->getmesh(U_, G_);
        vis->visMesh("sweptmesh", "sweptedge", U_, G_, 0.1, vis::Color::blue, 1);
       
    }

    inline void init(ros::NodeHandle &nh, Config &conf)
    {
        vis.reset(new vis::Visualization(nh));
        sdf_vis_pub  = nh.advertise<std_msgs::Float64MultiArray>("/sdf_vis", 10);
        flatness.reset(conf.vehicleMass, conf.gravAcc, conf.horizDrag,
                       conf.vertDrag, conf.parasDrag, conf.speedEps);

        momentum = conf.momentum;
        initShape(conf);
        bdx = bdy = bdz = conf.kernel_size * conf.occupancy_resolution;
    }

    /**
     * 初始化shape
     * @attention 注意需要调用子类的初始化函数initShape
     */
    inline void initShape(Config &conf)
    {
        std::string inputdata = conf.inputdata;

        size_t start = inputdata.find_last_of("/") + 1; 
        size_t end = inputdata.find_last_of(".");       
        std::string shapetype = inputdata.substr(start, end - start);

        if (shapeConstructors.count(shapetype) > 0)
        {
            std::cout << "\033[35m======= Analytic Shape now======= "
                      << "\033[0m" << std::endl;
            current_robot_shape = shapeConstructors[shapetype](conf);
        }
        else
        {
            std::cout << "\033[35m======= Generalshape now======= "
                      << "\033[0m" << std::endl;
            current_robot_shape = new Generalshape(conf);
        }
    }

    inline Eigen::MatrixXd getRobotShapeMesh()
    {

        return current_robot_shape->mesh;
    }

    /**
     * 更新机器人轨迹
     * @param new_traj 新的轨迹
     */
    inline void updateTraj(const Trajectory<TRAJ_ORDER> &new_traj)
    {
        this->traj = new_traj;
        double td = traj.getTotalDuration();
        if (td < 3 * 1e2)
        {
            traj_duration = td;
            t_max = td; 
        }
    }

    /**
     * 获得轨迹上具体某一时刻的状态
     * @param time_stamp 时刻
     * @param xt 引用格式，获得time_stamp时刻的位置
     * @param vt 引用格式，获得time_stamp时刻的速度
     * @param Rt 引用格式，获得time_stamp时刻的姿态
     * @param VRt 引用格式，获得time_stamp时刻的姿态变化
     */
    inline void getStateOnTrajStamp(const double &time_stamp,
                                    Eigen::Vector3d &xt,
                                    Eigen::Vector3d &vt,
                                    Eigen::Matrix3d &Rt,
                                    Eigen::Matrix3d &VRt)
    {
        Eigen::Vector3d at,jt;
        traj.getPos_Vel_Acc_Jerk(time_stamp,xt,vt,at,jt);
        Eigen::Vector4d quat;
        Eigen::Vector3d omg;
        flatness.optimizated_forward(vt, at, jt, quat, omg);
        Rt = Eigen::Quaterniond(quat[0], quat[1], quat[2], quat[3]).toRotationMatrix();
        Eigen::Matrix3d w_hat;
        w_hat << 0, -omg(2), omg(1),
            omg(2), 0, -omg(0),
            -omg(1), omg(0), 0;
        VRt = Rt * w_hat;
    }
    /**
     * 获得轨迹上具体某一时刻的状态
     * @param time_stamp 时刻
     * @param xt 引用格式，获得time_stamp时刻的位置
     * @param vt 引用格式，获得time_stamp时刻的速度
     * @param Rt 引用格式，获得time_stamp时刻的姿态
     * @param VRt 引用格式，获得time_stamp时刻的姿态变化导函数
     * @param St 引用格式,获得time_stamp时刻三轴缩放比例
     * @param dSt 引用格式,获得time_stamp时刻三轴缩放变化导函数
     */
    inline void getStateOnTrajStamp(const double &time_stamp,
                                    Eigen::Vector3d &xt,
                                    Eigen::Vector3d &vt,
                                    Eigen::Matrix3d &Rt,
                                    Eigen::Matrix3d &VRt,
                                    Eigen::Matrix3d &St,
                                    Eigen::Matrix3d &dSt)
    {
        Eigen::Vector3d at,jt;
        traj.getPos_Vel_Acc_Jerk(time_stamp,xt,vt,at,jt);
        Eigen::Vector4d quat; 
        Eigen::Vector3d omg;;
        flatness.optimizated_forward(vt, at, jt, quat, omg);
        Rt = Eigen::Quaterniond(quat[0], quat[1], quat[2], quat[3]).toRotationMatrix();
        Eigen::Matrix3d w_hat;
        w_hat << 0, -omg(2), omg(1),
            omg(2), 0, -omg(0),
            -omg(1), omg(0), 0;
        VRt = Rt * w_hat;
        St = getScale(time_stamp);
        dSt = getDotScale(time_stamp);
    }

    /**
     * 获得轨迹上具体某一时刻的状态
     * @param time_stamp 时刻
     * @param xt 引用格式，获得time_stamp时刻的位置
     * @param Rt 引用格式，获得time_stamp时刻的姿态
     */
    inline void getStateOnTrajStamp(const double &time_stamp,
                                    Eigen::Vector3d &xt,
                                    Eigen::Matrix3d &Rt)
    {
        Eigen::Vector3d vt,at,jt;
        traj.getPos_Vel_Acc_Jerk(time_stamp,xt,vt,at,jt);
        Eigen::Vector4d quat;
        flatness.optimizated_forward(vt, at, jt, quat);
        Rt = Eigen::Quaterniond(quat[0], quat[1], quat[2], quat[3]).toRotationMatrix();
    }
    /**
     * 获得轨迹上具体某一时刻的状态
     * @param time_stamp 时刻
     * @param xt 引用格式，获得time_stamp时刻的位置
     * @param Rt 引用格式，获得time_stamp时刻的姿态
     * @param St 引用格式，获得time_stamp时刻的形变
     */
    inline void getStateOnTrajStamp(const double &time_stamp,
                                    Eigen::Vector3d &xt,
                                    Eigen::Matrix3d &Rt,
                                    Eigen::Matrix3d &St)
    {
        Eigen::Vector3d vt,at,jt;
        traj.getPos_Vel_Acc_Jerk(time_stamp,xt,vt,at,jt);
        Eigen::Vector4d quat; 
        flatness.optimizated_forward(vt, at, jt, quat);
        Rt = Eigen::Quaterniond(quat[0], quat[1], quat[2], quat[3]).toRotationMatrix();
        St = getScale(time_stamp);
    }

    /**
     * 获得轨迹上具体某一时刻的机器人尺寸因子
     * @param time_stamp 时刻
     * @return time_stamp 时刻机器人尺寸因子
     * @attention 当前返回单位矩阵，需要按需修改,注意奇异点问题Scale缩放分母除以0，求inverse问题
     */
    inline Eigen::Matrix3d getScale(const double t)
    {
        return Eigen::Matrix3d::Identity();
    }

    /**
     * 获得轨迹上具体某一时刻的机器人尺寸因子变化导数
     * @param time_stamp 时刻
     * @return time_stamp 时刻机器人尺寸因子
     * @attention 当前返回0矩阵，需要按需修改
     */
    inline Eigen::Matrix3d getDotScale(const double t) // 得到三轴缩放
    {
        Eigen::Matrix3d St;
        St = Matrix3d::Zero();
        return St;//原先这里出shit。导致优化后死循环？
    }

    /**
     * pos_eva 转为 pos_rel
     * @param pos_eva 世界系坐标
     * @param xt 机器人所在的位置
     * @param Rt 机器人的姿态
     * @return 机器人系下的坐标
     * @attention 带St的重载版本考虑优化inverse()计算
     */
    inline Eigen::Vector3d posEva2Rel(const Eigen::Vector3d &pos_eva,
                                      const Eigen::Vector3d &xt,
                                      const Eigen::Matrix3d &Rt)
    {
        return (Rt.transpose() * (pos_eva - xt));
    }
    /**
     * pos_eva 转为 pos_rel
     * @param pos_eva 世界系坐标
     * @param xt 机器人所在的位置
     * @param Rt 机器人的姿态
     * @param St 机器人的缩放因子
     * @return 机器人系下的坐标
     * @attention 带St的重载版本考虑优化inverse()计算
     */
    inline Eigen::Vector3d posEva2Rel(const Eigen::Vector3d &pos_eva,
                                      const Eigen::Vector3d &xt,
                                      const Eigen::Matrix3d &Rt,
                                      const Eigen::Matrix3d &St)
    {
        return (Rt.transpose() * St.inverse() * (pos_eva - xt));
    }

  
       /**
     * 初次选取时间种子的策略
     * 在轨迹上离散采样，初步获取时间种子
     * @param pos_eva 世界系下需要查询的点
     * @param dt 筛选种子点的分辨率
     * @return 找到的时间种子
     * @attention 注意手动制定模板元withscale是否考虑形状
     */
    template <bool withscale = false>
    inline double choiceTInit(const Eigen::Vector3d &pos_eva, const double dt, vector<double>& range_l, vector<double>& range_r,  vector<double>& range_ts)
    {
        double rough_dt = 0.2;

        double mindis = 1e9;
        double range_mindis = 1e9;
        double dis = 1e9;
        double time_seed = 0.0;
        double range_time_seed = 0.0;

        double dis_rough        = 1e9;
        double min_dis_rough    = 1e9;
        double second_dis_rough = 1e9;
        double tou_lb = 0;
        double tou_ub = 0;
        double safty_hor_inf = 2 * config.safety_hor + 0.1;

        range_l.clear();
        range_r.clear();
        range_ts.clear();
        bool flag_in_range = false;

        Eigen::Vector3d xt, vt;
        Eigen::Matrix3d Rt;
        if (!withscale)
        {
            for(double t = 0; t < traj_duration; t += rough_dt)
            {
                getStateOnTrajStamp(t, xt, Rt);
                dis = current_robot_shape->getonlySDF(posEva2Rel(pos_eva, xt, Rt));
                if (dis < mindis)
                {
                    time_seed = t;
                    mindis = dis;
                }

                if(dis < safty_hor_inf){
                    if(flag_in_range == false){
                        flag_in_range = true;
                        range_mindis = dis;
                        tou_lb = std::max(0.0, t - rough_dt);
                        tou_ub = t;
                    }
                    else{
                        tou_ub = std::min(traj_duration, t + rough_dt);
                    }
                }
                else{
                    if(flag_in_range == true){
                        flag_in_range = false;
                        tou_ub = std::min(traj_duration, t + rough_dt);
                        range_l.push_back(tou_lb);
                        range_r.push_back(tou_ub);
                    }
                }
            }

            int N = range_l.size();
            for(size_t i = 0 ; i < N ; i++) {
                tou_lb = range_l[i];
                tou_ub = range_r[i];
                for(double t = tou_lb; t < tou_ub; t += dt)
                {
                    getStateOnTrajStamp(t, xt, Rt);
                    dis = current_robot_shape->getonlySDF(posEva2Rel(pos_eva, xt, Rt));
                    if (dis < range_mindis)
                    {
                        range_time_seed = t;
                        range_mindis = dis;
                    }
                }
                range_ts.push_back( range_time_seed );
            }


            return time_seed;
        }

        else
        {
            Eigen::Matrix3d St;
            for (double t = 0; t < traj_duration; t += dt) 
            {
                getStateOnTrajStamp(t, xt, Rt, St);
                dis = current_robot_shape->getonlySDF(posEva2Rel(pos_eva, xt, Rt, St));
                if (dis < mindis)
                {
                    time_seed = t;
                    mindis = dis;
                }

                if(dis < safty_hor_inf){
                    if(flag_in_range == false){
                        flag_in_range = true;
                        tou_lb = std::max(0.0, t - rough_dt);
                        tou_ub = t;
                    }
                    else{
                        tou_ub = std::min(traj_duration, t + rough_dt);
                    }
                }
                else{
                    if(flag_in_range == true){
                        flag_in_range = false;
                        tou_ub = std::min(traj_duration, t + rough_dt);
                        range_l.push_back(tou_lb);
                        range_r.push_back(tou_ub);
                    }
                }
            }

            int N = range_l.size();
            for(size_t i = 0 ; i < N ; i++) {
                tou_lb = range_l[i];
                tou_ub = range_r[i];
                for(double t = tou_lb; t < tou_ub; t += dt)
                {
                    getStateOnTrajStamp(t, xt, Rt, St);
                    dis = current_robot_shape->getonlySDF(posEva2Rel(pos_eva, xt, Rt, St));
                    if (dis < range_mindis)
                    {
                        range_time_seed = t;
                        range_mindis = dis;
                    }
                }
                range_ts.push_back( range_time_seed );
            }
            return time_seed;
        }
    }



    /**
     * 初次选取时间种子的策略
     * 在轨迹上离散采样，初步获取时间种子
     * @param pos_eva 世界系下需要查询的点
     * @param dt 筛选种子点的分辨率
     * @return 找到的时间种子
     * @attention 注意手动制定模板元withscale是否考虑形状
     */
    template <bool withscale = false>
    inline double choiceTInit(const Eigen::Vector3d &pos_eva, const double dt)
    {
        double rough_dt = 0.15;

        double mindis = 1e9;
        double second_mindis = 1e9;
        double dis = 1e9;
        double time_seed = 0.0;
        double second_time_seed = 0.0;

        double dis_rough        = 1e9;
        double min_dis_rough    = 1e9;
        double second_dis_rough = 1e9;
        double tou_lb = 0;
        double tou_ub = 0;
        double safty_hor_inf = 2 * config.safety_hor + 0.1;

        vector<double> range_l;
        vector<double> range_r;
        bool flag_in_range = false;


        Eigen::Vector3d xt, vt;
        Eigen::Matrix3d Rt;
        if (!withscale)
        {
            for(double t = 0; t < traj_duration; t += rough_dt)
            {
                getStateOnTrajStamp(t, xt, Rt);
                dis = current_robot_shape->getonlySDF(posEva2Rel(pos_eva, xt, Rt));
                if (dis < mindis)
                {
                    time_seed = t;
                    mindis = dis;
                }

                if(dis < safty_hor_inf){
                    if(flag_in_range == false){
                        flag_in_range = true;
                        tou_lb = std::max(0.0, t - rough_dt);
                        tou_ub = t;
                    }
                    else{
                        tou_ub = std::min(traj_duration, t + rough_dt);
                    }
                }
                else{
                    if(flag_in_range == true){
                        flag_in_range = false;
                        tou_ub = std::min(traj_duration, t + rough_dt);
                        range_l.push_back(tou_lb);
                        range_r.push_back(tou_ub);
                    }
                }
            }

            int N = range_l.size();
            for(size_t i = 0 ; i < N ; i++) {
                tou_lb = range_l[i];
                tou_ub = range_r[i];
                for(double t = tou_lb; t < tou_ub; t += dt)
                {
                    getStateOnTrajStamp(t, xt, Rt);
                    dis = current_robot_shape->getonlySDF(posEva2Rel(pos_eva, xt, Rt));
                    if (dis < mindis)
                    {
                        time_seed = t;
                        mindis = dis;
                    }
                }
            }

            return time_seed;
        }
        else
        {
            Eigen::Matrix3d St;
            for (double t = 0; t < traj_duration; t += dt) 
            {
                getStateOnTrajStamp(t, xt, Rt, St);
                dis = current_robot_shape->getonlySDF(posEva2Rel(pos_eva, xt, Rt, St));
                if (dis < mindis)
                {
                    time_seed = t;
                    mindis = dis;
                }
                if(dis < safty_hor_inf){
                    if(flag_in_range == false){
                        flag_in_range = true;
                        tou_lb = std::max(0.0, t - rough_dt);
                        tou_ub = t;
                    }
                    else{
                        tou_ub = std::min(traj_duration, t + rough_dt);
                    }
                }
                else{
                    if(flag_in_range == true){
                        flag_in_range = false;
                        tou_ub = std::min(traj_duration, t + rough_dt);
                        range_l.push_back(tou_lb);
                        range_r.push_back(tou_ub);
                    }
                }
            }

            int N = range_l.size();
            for(size_t i = 0 ; i < N ; i++) {
                tou_lb = range_l[i];
                tou_ub = range_r[i];
                for(double t = tou_lb; t < tou_ub; t += dt)
                {
                    getStateOnTrajStamp(t, xt, Rt,St);
                    dis = current_robot_shape->getonlySDF(posEva2Rel(pos_eva, xt, Rt,St));
                    if (dis < mindis)
                    {
                        time_seed = t;
                        mindis = dis;
                    }
                }
            }

            return time_seed;
        }
    }

   

    /**
     * 得到机器人运动到xt状态，姿态和缩放为Rt和St时，在pos_eva处的SDF和grad_prel
     * @return SDF值
     */
    inline double getSDFWithGradWhenRobotAtState(const Eigen::Vector3d &pos_eva, Eigen::Vector3d &grad_prel, const Eigen::Vector3d &xt,
                                                 const Eigen::Matrix3d &Rt, const Eigen::Matrix3d &St)
    {
        return current_robot_shape->getSDFwithGrad1(posEva2Rel(pos_eva, xt, Rt, St), grad_prel);
    }

    /**
     * 得到机器人运动到xt状态，姿态和缩放为Rt时，在pos_eva处的SDF和grad_prel
     * @return SDF值
     */
    inline double getSDFWithGradWhenRobotAtState(const Eigen::Vector3d &pos_eva, Eigen::Vector3d &grad_prel, const Eigen::Vector3d &xt,
                                                 const Eigen::Matrix3d &Rt)
    {
        return current_robot_shape->getSDFwithGrad1(posEva2Rel(pos_eva, xt, Rt), grad_prel);
    }

    /**
     * 得到机器人运动到xt状态，姿态和缩放为Rt和St时，在pos_eva处的SDF和grad_prel
     * 使用数值方法计算的版本
     * @return SDF值
     */
    inline double getSDFWithGradWhenRobotAtStateNum(const Eigen::Vector3d &pos_eva, Eigen::Vector3d &grad_prel, const Eigen::Vector3d &xt,
                                                    const Eigen::Matrix3d &Rt, const Eigen::Matrix3d &St)
    {
        return current_robot_shape->getSDFwithGrad1Num(posEva2Rel(pos_eva, xt, Rt, St), grad_prel);
    }

    /**
     * 得到机器人运动到xt状态，姿态和缩放为Rt时，在pos_eva处的SDF和grad_prel
     * 使用数值方法计算的版本
     * @return SDF值
     */
    inline double getSDFWithGradWhenRobotAtStateNum(const Eigen::Vector3d &pos_eva, Eigen::Vector3d &grad_prel, const Eigen::Vector3d &xt,
                                                    const Eigen::Matrix3d &Rt)
    {
        return current_robot_shape->getSDFwithGrad1Num(posEva2Rel(pos_eva, xt, Rt), grad_prel);
    }

    /**
     * 解析方法得到机器人运动到t时刻时，在pos_eva处的SDF
     * @param pos_eva 世界系下需要查询的点
     * @param time_stamp 时刻t
     * @return 机器人运动到t时刻时，在pos_eva处的SDF
     */
    template <bool withscale = false>
    inline double getSDFAtTimeStamp(const Eigen::Vector3d &pos_eva, const double &time_stamp)
    {
        Eigen::Vector3d xt;
        Eigen::Matrix3d Rt;
        if (!withscale)
        {
            getStateOnTrajStamp(time_stamp, xt, Rt);
            return current_robot_shape->getonlySDF(posEva2Rel(pos_eva, xt, Rt));
        }
        else
        {
            Eigen::Matrix3d St;
            getStateOnTrajStamp(time_stamp, xt, Rt, St);
            return current_robot_shape->getonlySDF(posEva2Rel(pos_eva, xt, Rt, St));
        }
    }

    /**
     * 数值方法得到机器人运动到t时刻时，在pos_eva处的SDF
     * @param pos_eva 世界系下需要查询的点
     * @param time_stamp 时刻t
     * @return 机器人运动到t时刻时，在pos_eva处的SDF
     */
    template <bool withscale = false>
    inline double getSDFNumAtTimeStamp(const Eigen::Vector3d &pos_eva, const double &time_stamp)
    {
        Eigen::Vector3d xt;
        Eigen::Matrix3d Rt;
        if (!withscale)
        {
            getStateOnTrajStamp(time_stamp, xt, Rt);
            return current_robot_shape->getonlySDFNum(posEva2Rel(pos_eva, xt, Rt));
        }
        else
        {
            Eigen::Matrix3d St;
            getStateOnTrajStamp(time_stamp, xt, Rt, St);
            return current_robot_shape->getonlySDFNum(posEva2Rel(pos_eva, xt, Rt, St));
        }
    }

    /**
     * 解析方法得到机器人运动到t时刻时，在pos_eva处的GradPrel
     * @param pos_eva 世界系下需要查询的点
     * @param time_stamp 时刻t
     * @return 机器人运动到t时刻时，在pos_eva处的GradPrel
     */
    template <bool withscale = false>
    inline Eigen::Vector3d getGradPrelAtTimeStamp(const Eigen::Vector3d &pos_eva, const double &time_stamp)
    {
        Eigen::Vector3d xt;
        Eigen::Matrix3d Rt;
        if (!withscale)
        {
            getStateOnTrajStamp(time_stamp, xt, Rt);
            return current_robot_shape->getonlyGrad1(posEva2Rel(pos_eva, xt, Rt));
        }
        else
        {
            Eigen::Matrix3d St;
            getStateOnTrajStamp(time_stamp, xt, Rt, St);
            return current_robot_shape->getonlyGrad1(posEva2Rel(pos_eva, xt, Rt, St));
        }
    }

    /**
     * 数值方法得到机器人运动到t时刻时，在pos_eva处的GradPrel
     * @param pos_eva 世界系下需要查询的点
     * @param time_stamp 时刻t
     * @return 机器人运动到t时刻时，在pos_eva处的GradPrel
     */
    template <bool withscale = false>
    inline Eigen::Vector3d getGradPrelNumAtTimeStamp(const Eigen::Vector3d &pos_eva, const double &time_stamp)
    {
        Eigen::Vector3d xt;
        Eigen::Matrix3d Rt;
        if (!withscale)
        {
            getStateOnTrajStamp(time_stamp, xt, Rt);
            return current_robot_shape->getonlyGrad1Num(posEva2Rel(pos_eva, xt, Rt));
        }
        else
        {
            Eigen::Matrix3d St;
            getStateOnTrajStamp(time_stamp, xt, Rt, St);
            return current_robot_shape->getonlyGrad1Num(posEva2Rel(pos_eva, xt, Rt, St));
        }
    }

    /**
     * 解析方法得到机器人运动到t时刻时，在pos_eva处的SDF对于轨迹运动时间t的导数
     * @param withscale 模板决定是否考率形变
     * @param pos_eva 世界系下需要查询的点
     * @param time_stamp 时刻t
     * @return 机器人运动到t时刻时，在pos_eva处的SDF对于轨迹运动时间t的导数
     */
    template <bool withscale = false>
    inline double getSDF_DOTAtTimeStamp(const Eigen::Vector3d &pos_eva, const double &time_stamp/**,bool debug=false**/)
   {    

        double t1 = std::max(0.0 ,time_stamp - 0.000001);
        double t2 = std::min(traj_duration, time_stamp + 0.000001);
        double sdf1 = getSDFAtTimeStamp<withscale>( pos_eva, t1);
        double sdf2 = getSDFAtTimeStamp<withscale>( pos_eva, t2); 
        return (sdf2 - sdf1) * 500000; 
    }


    template <bool withscale = false>
    inline double getSDF_DOTAtTimeStampOptimized(const Eigen::Vector3d &pos_eva, const double &time_stamp/**,bool debug=false**/)
    {
        Eigen::Vector3d xt, vt, at, jt;

        traj.getPos_Vel_Acc_Jerk(time_stamp, xt, vt, at, jt); 

        double v0 = vt(0);
        double v1 = vt(1);
        double v2 = vt(2);
        double a0 = at(0);
        double a1 = at(1);
        double a2 = at(2);
        double cp_term = sqrt(v0 * v0 + v1 * v1 + v2 * v2 + veps);
        double w_term = 1.0 + cp * cp_term;
        double w0 = w_term * v0;
        double w1 = w_term * v1;
        double w2 = w_term * v2;
        double zu0 = a0 + dh_over_m * w0;
        double zu1 = a1 + dh_over_m * w1;
        double zu2 = a2 + dh_over_m * w2 + grav;
        double zu_sqr0 = zu0 * zu0;
        double zu_sqr1 = zu1 * zu1;
        double zu_sqr2 = zu2 * zu2;
        double zu_sqr_norm = zu_sqr0 + zu_sqr1 + zu_sqr2;
        double zu_norm = sqrt(zu_sqr_norm);
        double z0 = zu0 / zu_norm;
        double z1 = zu1 / zu_norm;
        double z2 = zu2 / zu_norm;
        double tilt_den = sqrt(2.0 * (1.0 + z2)); 
        Eigen::Vector4d quat;                     
        quat(0) = 0.5 * tilt_den;
        quat(1) = -z1 / tilt_den;
        quat(2) = z0 / tilt_den;
        quat(3) = 0;
        double ng_den = zu_sqr_norm * zu_norm;
        double ng00 = (zu_sqr1 + zu_sqr2) / ng_den;
        double zu01 = zu0 * zu1;
        double zu12 = zu1 * zu2;
        double zu02 = zu0 * zu2;
        double ng01 = -zu01 / ng_den;
        double ng02 = -zu02 / ng_den;
        double ng11 = (zu_sqr0 + zu_sqr2) / ng_den;
        double ng12 = -zu12 / ng_den;
        double ng22 = (zu_sqr0 + zu_sqr1) / ng_den;
        double v_dot_a = v0 * a0 + v1 * a1 + v2 * a2;
        double dw_term = cp * v_dot_a / cp_term;
        double dw0 = w_term * a0 + dw_term * v0;
        double dw1 = w_term * a1 + dw_term * v1;
        double dw2 = w_term * a2 + dw_term * v2;
        double dz_term0 = jt(0) + dh_over_m * dw0;
        double dz_term1 = jt(1) + dh_over_m * dw1;
        double dz_term2 = jt(2) + dh_over_m * dw2;
        double dz0 = ng00 * dz_term0 + ng01 * dz_term1 + ng02 * dz_term2;
        double dz1 = ng01 * dz_term0 + ng11 * dz_term1 + ng12 * dz_term2;
        double dz2 = ng02 * dz_term0 + ng12 * dz_term1 + ng22 * dz_term2;
        double omg_den = z2 + 1.0;
        double omg_term = dz2 / omg_den;
        Eigen::Vector3d omg;
        omg(0) = -dz1 + z1 * omg_term;
        omg(1) = dz0 - z0 * omg_term;
        omg(2) = (z1 * dz0 - z0 * dz1) / omg_den;
        Eigen::Matrix3d Rt = Eigen::Quaterniond(quat[0], quat[1], quat[2], quat[3]).toRotationMatrix();
        Eigen::Matrix3d w_hat;
        w_hat << 0, -omg(2), omg(1),
            omg(2), 0, -omg(0),
            -omg(1), omg(0), 0;
        Eigen::Matrix3d Rt_trans = Rt.transpose();
        if (!withscale)
        {
        Eigen::Vector3d temp = Rt_trans * (pos_eva - xt);
        Eigen::Vector3d sdf_grad1 = current_robot_shape->getonlyGrad1(temp);
        Eigen::Vector3d point_velocity = -(Rt_trans * vt + w_hat * temp);
        return sdf_grad1.dot(point_velocity);
        }
        else
        {
        Eigen::Matrix3d St = getScale(time_stamp);
        Eigen::Vector3d temp = Rt_trans* St.inverse() * (pos_eva - xt) ;
        Eigen::Vector3d sdf_grad1 = current_robot_shape->getonlyGrad1(temp);
        Eigen::Matrix3d dSt = getDotScale(time_stamp);
        Eigen::Vector3d point_velocity = -(Rt_trans * St.inverse() * vt + w_hat * temp + Rt_trans * St.inverse() * dSt * St.inverse() * (pos_eva - xt));
        return sdf_grad1.dot(point_velocity);
        }

    }

    /**
     * 数值方法得到机器人运动到t时刻时，在pos_eva处的SDF对于轨迹运动时间t的导数
     * @param withscale 模板决定是否考率形变
     * @param pos_eva 世界系下需要查询的点
     * @param time_stamp 时刻t
     * @return 机器人运动到t时刻时，在pos_eva处的SDF对于轨迹运动时间t的导数
     */
    template <bool withscale = false>
    inline double getSDF_DOTNumAtTimeStamp(const Eigen::Vector3d &pos_eva, const double &time_stamp)
    {
        Eigen::Vector3d sdf_grad1;
        Eigen::Vector3d point_velocity;
        Eigen::Vector3d xt, vt;
        Eigen::Matrix3d Rt, VRt, Rt_trans;//有shit原先
        
        if (!withscale)
        {
            getStateOnTrajStamp(time_stamp, xt, vt, Rt, VRt);
            Rt_trans = Rt.transpose();
            sdf_grad1 = current_robot_shape->getonlyGrad1Num(posEva2Rel(pos_eva, xt, Rt));
            point_velocity = -(Rt_trans * vt + Rt_trans * VRt * Rt_trans * (pos_eva - xt));
        }
        else
        {
            Eigen::Matrix3d St, dSt;
            getStateOnTrajStamp(time_stamp, xt, vt, Rt, VRt, St, dSt);
            Rt_trans = Rt.transpose();
            sdf_grad1 = current_robot_shape->getonlyGrad1Num(posEva2Rel(pos_eva, xt, Rt, St));
            point_velocity = -(Rt_trans * St.inverse() * vt + Rt_trans * VRt * Rt_trans * St.inverse() * (pos_eva - xt) + Rt_trans * St.inverse() * dSt * St.inverse() * (pos_eva - xt));
        } 

        return sdf_grad1.dot(point_velocity);
    }

    /**
     * 解析方法得到扫略体积的SDF值
     * @param pos_eva 世界系下需要查询的点
     * @param time_seed_f 引用格式，传入轨迹时间点的迭代初值，获得梯度下降后的t*
     * @param grad_prel 引用格式，得到点pos_eva处对t*时刻物体的SDF梯度
     * @param set_ts 是否指定时间初值,如果为false,则迭代初值采用离散策略选取而不是time_seed_f传入
     * @param need_grad_prel 是否需要计算grad_prel
     * @return 单个点对扫略体积的SDF
     * @attention 注意需要制定模板元set_ts与need_grad_prel
     */
    template <bool set_ts = false, bool need_grad_prel = true>
    inline double getSDFofSweptVolume(const Eigen::Vector3d &pos_eva, double &time_seed_f, Eigen::Vector3d &grad_prel)
    {
        Eigen::Vector3d xt, vt;
        double ts = time_seed_f;
        double t_star;
        double sdf_star;
        double dtime = 0.0005;
        if (set_ts == false)
        {
            ts = choiceTInit<useScale>(pos_eva, dtime); 
        }
        double tmin_ = std::max(0.0, ts - 3.4);           
        double tmax_ = std::min(ts + 3.4, traj_duration); 
        gradientDescent(momentum, tmin_, tmax_, ts, sdf_star, t_star, pos_eva);
        if (need_grad_prel)
        {
            grad_prel = getGradPrelAtTimeStamp<useScale>(pos_eva, t_star);
        }
        time_seed_f = t_star;
        return sdf_star;
    }


    /**
     * 解析方法得到扫略体积的SDF值
     * @param pos_eva 世界系下需要查询的点
     * @param time_seed_f 引用格式，传入轨迹时间点的迭代初值，获得梯度下降后的t*
     * @param grad_prel 引用格式，得到点pos_eva处对t*时刻物体的SDF梯度
     * @param set_ts 是否指定时间初值,如果为false,则迭代初值采用离散策略选取而不是time_seed_f传入
     * @param need_grad_prel 是否需要计算grad_prel
     * @return 单个点对扫略体积的SDF
     * @attention 注意需要制定模板元set_ts与need_grad_prel
     */
    template <bool need_grad_prel = true>
    inline double getSDFofSweptVolume(const Eigen::Vector3d &pos_eva, double &time_seed_f, Eigen::Vector3d &grad_prel, bool set_ts)
    {
        Eigen::Vector3d xt, vt;
        double ts = time_seed_f;
        double t_star;
        double sdf_star = 1e1;
        double min_sdf_star = 1e1;
        double dtime = 0.02;

        std::vector<double> range_l;
        std::vector<double> range_r;
        std::vector<double> range_ts;

        if (set_ts == false)
        {
            ts = choiceTInit<useScale>(pos_eva, dtime , range_l, range_r, range_ts);//在ts附近作梯度下降
        }

        int range_count = range_l.size();
        for( int i = 0 ; i < range_count; i++ ){
            double temp_ts = range_ts[i];
            double tmin_ = std::max(0.0, range_l[i]);           
            double tmax_ = std::min(range_r[i], traj_duration); 
            gradientDescent(momentum, tmin_, tmax_, temp_ts, sdf_star, t_star, pos_eva);
            
            if( sdf_star < min_sdf_star){

                min_sdf_star = sdf_star;
                time_seed_f = t_star;
                if (need_grad_prel)
                {
                    grad_prel = getGradPrelAtTimeStamp<useScale>(pos_eva, t_star);
                }
            }
        }
        return min_sdf_star;
    }

   
    /**
     * 设置轨迹的起始时刻
     * @param traj_stamp 时刻
     */
    inline void setTrajStamp(const double &traj_stamp)
    {
        trajStamp = traj_stamp;
    }

    /**
     * 判断轨迹是否无碰
     * @return 轨迹碰撞返回true
     * @attention 目前永远返回false
     */
    inline bool isTrajCollide()
    {
        return false;
    }
    bool isIndexValid(const int ix, const int iy, const int iz) const
    {
    }
    bool isIndexOccupied(int ix, int iy, int iz)
    {
    }

    template <bool useByteKernel = true>
    inline bool kernelConv(int kernel_i, int kernel_j, const Eigen::Vector3i &ind) 
    {
        int kernel_size = config.kernel_size;
        int side_size = (kernel_size - 1) / 2;

        int ind_x = ind(0);
        int ind_y = ind(1);
        int ind_z = ind(2);

        if (useByteKernel == false)
        {
            int a, b, c;
            for (int off_x = -side_size; off_x <= side_size; off_x++)
            {
                for (int off_y = -side_size; off_y <= side_size; off_y++)
                {
                    for (int off_z = -side_size; off_z <= side_size; off_z++)
                    { 
                        if (isIndexValid(off_x + ind_x, off_y + ind_y, off_z + ind_z) == false)
                        {
                            continue;
                        }                                                                                                                                          // 查询的中心点不在地图-->安全？应该直接为false
                        a = off_x + side_size;                                                                                                                     // abc从0～2*side_size对应机体自身地图
                        b = off_y + side_size;                                                                                                                     // abc从0～2*side_size对应机体自身地图
                        c = off_z + side_size;                                                                                                                     // abc从0～2*side_size对应机体自身地图
                        if (current_robot_shape->shape_kernels[kernel_i * ykernel_size + kernel_j].map[a * kernelsize * kernelsize + b * kernelsize + c] == false) // 自身没有碰撞,ok
                        {
                            continue;
                        }
                        if (isIndexOccupied(off_x + ind_x, off_y + ind_y, off_z + ind_z) == false)
                        {
                            continue;
                        }            
                        return false; 
                    }
                }
            }
            return true; 
        }
        else
        {
            int half_size = (kernel_size - 1) / 2;
            int kernel_size_yz = kernel_size * kernel_size;
            int bytes_len_of_last_dim = (kernel_size + 7) / 8;

            int map_Xsize_inflated = map_Xsize + 2 * half_size;
            int map_Ysize_inflated = map_Ysize + 2 * half_size;
            int map_Zsize_inflated = map_Zsize + 2 * half_size;
            int map_last_dim_bytes_len = (map_Zsize_inflated + 7) / 8;

            int box_min_x = ind[0]; 
            int box_min_y = ind[1];
            int box_min_z = ind[2];
            for (int i = 0; i < kernel_size; i++)
            { 
                for (int j = 0; j < kernel_size; j++)
                { 
                    int startByteIdx = (box_min_x + i) * map_Ysize_inflated * map_last_dim_bytes_len + (box_min_y + j) * map_last_dim_bytes_len + box_min_z / 8;
                    int offsetIdx = box_min_z % 8;
                    for (int k = 0; k < bytes_len_of_last_dim; k++)
                    { 
                        int kernelByteIdx = i * kernel_size * bytes_len_of_last_dim + j * bytes_len_of_last_dim + k;
                        uint8_t block = (map_kernel[startByteIdx + k] << offsetIdx) | (map_kernel[startByteIdx + k + 1] >> (8 - offsetIdx));
                        uint8_t res = current_robot_shape->byte_shape_kernels[kernel_i * ykernel_size + kernel_j].map[kernelByteIdx] & block;
                        if (res)
                        {
                            return false;
                        }
                    }
                }
            }
            return true;
        }
    }
    

    inline bool visit_kernels_by_distance(int &returni, int &returnj, int start_x, int start_y, const Eigen::Vector3i &ind, int maxdeepth = 800)
    { 
        int rows = xkernel_size;
        int cols = ykernel_size;
        vector<vector<bool>> visited(rows, vector<bool>(cols, false));
        queue<pair<int, int>> q;
        int zero_pose_i = (xkernel_size - 1) / 2;
        int zero_pose_j = (ykernel_size - 1) / 2;
        int conv_result=false;
        volatile bool result = false;
        conv_result = kernelConv<true>(zero_pose_i, zero_pose_j, ind);
        if (conv_result)
        {
            returni = zero_pose_i;
            returnj = zero_pose_j;
            result = true;
            return result;
        }
        q.push(make_pair(start_x, start_y));
        visited[start_x][start_y] = true;
        vector<pair<int, int>> directions = {{0, 1}, {0, -1}, {1, 0}, {-1, 0}};
        int deep = 0;
        while (!q.empty())
        {
            deep++;
            int x = q.front().first;
            int y = q.front().second;
            q.pop();
            if (x != zero_pose_i || y != zero_pose_j)
            {
                conv_result = kernelConv<true>(x, y, ind);
                if (conv_result)
                {
                    returni = x;
                    returnj = y;
                    result = true;
                    break;
                }
            }
            for (auto &dir : directions)
            {
                int nx = x + dir.first;
                int ny = y + dir.second;
                if (nx < 0 || nx >= rows || ny < 0 || ny >= cols)
                {
                    continue;
                }
                if (visited[nx][ny])
                {
                    continue;
                }
                visited[nx][ny] = true;
                q.push(make_pair(nx, ny));
            }
            if (deep > maxdeepth) 
            {
                result = false;
                break;
            }
        }
        return result;
    }

    inline bool checkKernelValue(double father_roll, double father_pitch, double &child_roll, double &child_pitch, const Eigen::Vector3i &ind)
    {
        auto start = std::chrono::system_clock::now();
        int father_i = (father_roll + config.kernel_max_roll) / config.kernel_ang_res;  
        int father_j = (father_pitch + config.kernel_max_pitch) / config.kernel_ang_res;
        int seed_i = father_i;
        int seed_j;
        int min_j;
        int max_j;
        int min_i;
        int max_i;
        int ret_i;
        int ret_j;
        int dis_2_f = 0;
        bool res = false; 
        ret_i = father_i;
        ret_j = father_j; // BFS
        if (visit_kernels_by_distance(ret_i, ret_j, father_i, father_j, ind))
        {
            child_roll = father_roll + (ret_i - father_i) * (config.kernel_ang_res);
            child_pitch = father_pitch + (ret_j - father_j) * (config.kernel_ang_res);
            auto end = std::chrono::system_clock::now();
            return true;
        }

        auto end = std::chrono::system_clock::now();
        return false;
    }

    /**
     * 执行traj_server可视化轨迹运动的下一帧
     * @param traj    轨迹
     */
    inline void process(const Trajectory<TRAJ_ORDER> &traj)
    {
        static int init = 0;
        if (init < 200)
        {
            vis->visPolytope(current_robot_shape->mesh, "Polymesh", "Polyedge", false,2);                              
            vector<double> bounds = current_robot_shape->getbound();                                                              
            vis->visABoxWithId(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(2 * bounds[3], 2 * bounds[4], 2 * bounds[5]), "selfbox"); 
            vis->visABoxWithId(Eigen::Vector3d(config.offsetAABBbox[0], config.offsetAABBbox[1], config.offsetAABBbox[2]), Eigen::Vector3d(2* bdx / 3.0,2* bdy / 3.0,2* bdz / 3.0), "AABBbox",vis::yellow,0.4,2); 
            init++;
        }
        const double delta = ros::Time::now().toSec() - trajStamp;
        Eigen::Vector3d trans;
        if (delta > 0.0 && delta < traj.getTotalDuration())
        {
            Eigen::Matrix3d St;
            trans = traj.getPos(delta);
            St = getScale(delta);
            double thr;
            Eigen::Vector4d quat; 
            Eigen::Vector3d omg;
            flatness.forward(traj.getVel(delta),
                             traj.getAcc(delta),
                             traj.getJer(delta),
                             0.0, 0.0,
                             thr, quat, omg);
            double speed = traj.getVel(delta).norm();
            double acc = traj.getAcc(delta).norm();
            double bodyratemag = omg.norm();
            double tiltangle = acos(1.0 - 2.0 * (quat(1) * quat(1) + quat(2) * quat(2)));

            Eigen::Matrix3d Rt = Eigen::Quaterniond(quat[0], quat[1], quat[2], quat[3]).toRotationMatrix(); // r_dot=R*skew(w)
            Eigen::Matrix3d w_hat;
            w_hat << 0, -omg(2), omg(1),
                omg(2), 0, -omg(0),
                -omg(1), omg(0), 0;
            Eigen::Matrix3d VRt = Rt * w_hat;
            current_robot_shape->Transform(quat, trans, St);
            vis->visPolytope(current_robot_shape->mesh_var, "Polymesh", "Polyedge", false,2);
            vis->visABall(current_robot_shape->interior_var, 0.1, "interior", vis::Color::blue);
        
        }
    }

    /**
     * 梯度下降函数(SweptVolumeManager类专用)
     * @param momentum 动量
     * @param t_min    搜索范围下界
     * @param t_max    搜索范围上界
     * @param x0       迭代初始值
     * @param fx       引用格式，接收SDF*
     * @param x        引用格式，接收t*
     * @param pos_eva  评价点的坐标
     */
    inline void gradientDescent(double momentum, double t_min, double t_max,
                                const double x0, double &fx, double &x, const Eigen::Vector3d &pos_eva)
    {
        assert((t_max > 0) && (t_max < 100) && " in gradient descent,t_max must > 0 and t_max <100");
        assert((t_min >= 0) && " in gradient descent,t_min must >=0");
        assert((momentum >= 0) && (momentum <= 1) && "momentum must between 0~1");

        int max_iter = 300;
        double alpha = 0.02; 
        double tau = alpha;
        double g = 0.0; 
        double tol = 1e-5;
        double min = t_min;
        double max = t_max;
        double xmin = x0; 
        double xmax = x0;
        x = x0;

        double projection = 0;
        double change = 0;

        double prev_x = 10000000.0;
        int iter = 0;
        bool stop = false;
        double x_candidate;
        double fx_candidate;

        g = 100.0;

        while (iter < max_iter && !stop && abs(x - prev_x) > tol) 
        {
            xmin = std::min(xmin, x);
            xmax = std::max(xmax, x);

            if (iter == 0)
            {
                fx = getSDFAtTimeStamp<useScale>(pos_eva, x);
            }
            g = getSDF_DOTAtTimeStampOptimized<useScale>(pos_eva, x);
            tau = alpha;
            prev_x = x;
            for (int div = 1; div < 10; div++) 
            {
                iter = iter + 1;
                projection = x;
                g = getSDF_DOTAtTimeStampOptimized<useScale>(pos_eva, projection);
                x_candidate = x - tau * ((int)(g > 0) - (int)(g < 0));
                x_candidate = std::max(std::min(x_candidate, t_max), t_min);
                fx_candidate = getSDFAtTimeStamp<useScale>(pos_eva, x_candidate);
                if ((fx_candidate - fx) < 0) 
                {
                    x = x_candidate;
                    fx = fx_candidate;
                    break;
                }
                tau = 0.5 * tau;
                if (div == 9)
                {
                    stop = true; 
                }
            }
        }
    }

   
    /**
     * 梯度下降函数(可视化扫略体积专用)
     * @param momentum 动量
     * @param t_min    搜索范围下界
     * @param t_max    搜索范围上界
     * @param x0       迭代初始值
     * @param fx       引用格式，接收SDF*
     * @param x        引用格式，接收t*
     * @param pos_eva  评价点坐标
     * @param intervals  加速策略
     * @param values  加速策略
     * @param minima  加速策略
     */
    void gradient_descent(double momentum, double t_min, double t_max, const double x0,
                          double &fx, double &x, const Eigen::Vector3d &pos_eva, std::vector<double> &intervals,
                          std::vector<double> &values, std::vector<double> &minima)
    {
        assert((t_max > 0) && (t_max < 100) && " in gradient descent,t_max must > 0 and t_max <100");
        assert((t_min >= 0) && " in gradient descent,t_min must >=0");
        assert((momentum >= 0) && (momentum <= 1) && "momentum must between 0~1"); 
        int max_iter = 1000;
        double alpha = 0.02; 
        double tau = alpha;
        double g = 0.0; 
        double min = t_min;
        double max = t_max;
        double tol = 1e-3;
        double xmin = x0; 
        double xmax = x0;
        x = x0;

        double projection = 0;
        double change = 0;

        double prev_x = 10000000.0;
        int iter = 0;
        bool stop = false;
        double x_candidate, fx_candidate;
        g = 100.0;
        int in_existing_interval = -1;
        while (iter < max_iter && !stop && abs(x - prev_x) > tol) 
        {
            xmin = std::min(xmin, x);
            xmax = std::max(xmax, x);
            for (int mm = 0; mm < (intervals.size() / 2); mm++)
            {
                if ((x >= (intervals[2 * mm] - 1e-6)) && (x <= (intervals[2 * mm + 1] + 1e-6)))
                {
                    fx = values[mm];
                    x = minima[mm];
                    in_existing_interval = mm; 
                    break;
                }
            }
            if (in_existing_interval > -1)
            {
                break; 
            }

            if (iter == 0)
            {
                fx = getSDFAtTimeStamp<useScale>(pos_eva, x);
            }
            g = getSDF_DOTAtTimeStampOptimized<useScale>(pos_eva, x);
            tau = alpha;
            prev_x = x;
            for (int div = 1; div < 10; div++)
            {
                iter = iter + 1;
                assert(iter < max_iter && "不满足iter < max_iter");
                projection = x + momentum * change;
                g = getSDF_DOTAtTimeStampOptimized<useScale>(pos_eva, projection);
                change = momentum * change - tau * ((double)(g > 0) - (g < 0));
                x_candidate = x + change;
                x_candidate = std::max(std::min(x_candidate, t_max), t_min);
                fx_candidate = getSDFAtTimeStamp<useScale>(pos_eva, x_candidate);
                if ((fx_candidate - fx) < (0.5 * (x_candidate - x) * g)) 
                {
                    x = x_candidate;
                    fx = fx_candidate;
                    break;
                }
                tau = 0.5 * tau;
                if (div == 9)
                {
                    stop = true; 
                }
            }
        }

        if (in_existing_interval == -1) 
        {
            intervals.push_back(xmin);
            intervals.push_back(xmax);
            values.push_back(fx);
            minima.push_back(x);
        }
        else
        {

            intervals[2 * in_existing_interval] = std::min(intervals[2 * in_existing_interval], xmin);
            intervals[2 * in_existing_interval + 1] = std::max(intervals[2 * in_existing_interval + 1], xmax);
        }
    }
   

    std::function<double(const Eigen::RowVector3d &, double &, std::vector<std::vector<double>> &,
                         std::vector<std::vector<double>> &, std::vector<std::vector<double>> &)>
        scalarFunc = [&](const Eigen::RowVector3d &P, double &time_seed, 
                         std::vector<std::vector<double>> &intervals, std::vector<std::vector<double>> &values,
                         std::vector<std::vector<double>> &minima) -> double 
    {
        Eigen::RowVector3d running_closest_point = current_robot_shape->V.row(0); 
        double running_sign = 1.0;
        // Run gradient descent
        double distance, seed;
        if (intervals.size() == 0)
        {
            std::vector<double> temp_interval;
            temp_interval.resize(0);
            intervals.push_back(temp_interval);
            values.push_back(temp_interval);
            minima.push_back(temp_interval);
        }
        // momentum=0
        gradient_descent(0, std::max(time_seed - 0.1, t_min), std::min(time_seed + 0.1, t_max), time_seed, distance, seed, P.transpose(), intervals[0], values[0], minima[0]); 
        time_seed = seed;                                                                                                                                                     
        return distance;
    };

};
#endif
