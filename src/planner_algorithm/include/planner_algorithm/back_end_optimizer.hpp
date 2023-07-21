#pragma once

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <map_manager/PCSmap_manager.h>
#include <swept_volume/sw_manager.hpp>
#include <std_msgs/Float32MultiArray.h>
#include <utils/minco.hpp>
#include <utils/config.hpp>
#include <utils/flatness.hpp>
#include <utils/lbfgs.hpp>
#include <utils/lmbm.h>
#include <utils/Visualization.hpp>
#include <utils/debug_publisher.hpp>
#include <iostream>
#include <chrono>
#include <omp.h>
#include <fstream>
#include <atomic>
#define TRAJ_ORDER 5
#define GNOW ros::Time::now()
#define DEBUGOUT(x) std::cout << x << std::endl
using namespace std;
#define USE_INT_VER 1
#define USE_DES_VER 2
#define SEQUENTIAL 1 
#define PARALLEL 2   
#define USE_GRID_MAP 1
#define USE_FEATURE_MAP 2
#define DEBUG_PARALLEL 1
class TrajOptimizer
{
public:
    int optimizer_type;
    int execution_type;
    double integration_sequential_ms{0.0}; 
    double integration_parallel_ms{0.0};   
    double discrete_sequential_ms{0.0};    
    double discrete_parallel_ms{0.0};      

    int cost_iter{0};
    int map_type;

    Eigen::VectorXd opt_x;
    double total_opt_time = 0.0;
    double total_sdf_time = 0.0;
    double total_AABB_time = 0.0;
    // ros
    ros::NodeHandle nh;
    ros::Publisher debug_pub;
    ros::Publisher debug_wp_pub;
    ros::Publisher debug_vec_pub;
    ros::Publisher debug_wplists_pub;
    ros::Publisher test_map_pub;
    ros::Publisher debugesdfpub;
    minco::MINCO_S3NU minco;
    flatness::FlatnessMap flatmap;
    SweptVolumeManager::Ptr sv_manager; 
    std::vector<double> lastTstar;      
    PCSmapManager::Ptr pcsmap_manager;
    std::vector<Eigen::Vector3d> parallel_points; 
    int parallel_points_num;                      

    bool outputdebug{false};     
    bool enable_prefetch{false}; 
    Trajectory<TRAJ_ORDER> step_traj;
    vis::Visualization::Ptr vis;

    double rho;
    Eigen::Matrix3d initState;
    Eigen::Matrix3d finalState;
    double *x_variable; 

    int pieceN;
    int spatialDim;
    int temporalDim;

    int integralRes;
    Eigen::VectorXd physicalPm;

    Eigen::Matrix3Xd points;
    Eigen::VectorXd times;
    Eigen::Matrix3Xd gradByPoints;
    Eigen::VectorXd gradByTimes;
    Eigen::MatrixX3d partialGradByCoeffs;
    Eigen::VectorXd partialGradByTimes;


    Config conf;
    Eigen::Matrix3Xd test_obs;
    int threads_num{30};
    double vmax;
    double omgmax;
    double thetamax;
    double weight_v;
    double weight_a;
    double weight_p;
    double weight_omg;
    double weight_theta;
    double smooth_fac;
    double safety_hor;

    double bdx; //=config.kernel_size*config.occupancy_resolution;//自身bounding box的长宽高
    double bdy; //=config.kernel_size*config.occupancy_resolution;//自身bounding box的长宽高
    double bdz; //=config.kernel_size*config.occupancy_resolution;//自身bounding box的长宽高

    vector<Eigen::Matrix<double, 6, 3>> last_gdc;
    vector<double> last_gdt;
    vector<double> last_ts;


    double cost_pos;
    double cost_other;
    double cost_total;

    bool exit = false;
    bool pause = false;
    bool next_step = false;

    int iter = 0;

    void clearDebugVector();
    void drawDebug();

    void renderAABBpoints();

    inline Eigen::Matrix3d getQuatTransDW(const Eigen::Vector4d &quat)
    { 
        Eigen::Matrix3d ret;
        double x = quat(1);
        double y = quat(2);
        double z = quat(3);
        ret << 0, 2 * z, -2 * y,
            -2 * z, 0, 2 * x,
            2 * y, -2 * x, 0;
        return ret;
    }
    inline Eigen::Matrix3d getQuatTransDX(const Eigen::Vector4d &quat)
    { 
        Eigen::Matrix3d ret;
        double w = quat(0);
        double x = quat(1);
        double y = quat(2);
        double z = quat(3);
        ret << 0, 2 * y, 2 * z,
            2 * y, -4 * x, 2 * w,
            2 * z, -2 * w, -4 * x;
        return ret;
    }
    inline Eigen::Matrix3d getQuatTransDY(const Eigen::Vector4d &quat)
    { 
        Eigen::Matrix3d ret;
        double w = quat(0);
        double x = quat(1);
        double y = quat(2);
        double z = quat(3);
        ret << -4 * y, 2 * x, -2 * w,
            2 * x, 0, 2 * z,
            2 * w, 2 * z, -4 * y;
        return ret;
    }
    inline Eigen::Matrix3d getQuatTransDZ(const Eigen::Vector4d &quat)
    { 
        Eigen::Matrix3d ret;
        double w = quat(0);
        double x = quat(1);
        double y = quat(2);
        double z = quat(3);
        ret << -4 * z, 2 * w, 2 * x,
            -2 * w, -4 * z, 2 * y,
            2 * x, 2 * y, 0;
        return ret;
    }

    static inline void forwardP(const Eigen::VectorXd &xi,
                                Eigen::Matrix3Xd &P)
    {
        const int sizeP = xi.size() / 3;
        P.resize(3, sizeP);
        for (int i = 0; i < sizeP; i++)
        {
            P.col(i) = xi.segment(3 * i, 3);
        }
        return;
    }

    static inline void forwardP(const double *xi,
                                Eigen::Matrix3Xd &P, int Space_dim)
    {

        P.resize(3, Space_dim);
        for (int i = 0; i < Space_dim; i++)
        {
            Eigen::Map<const Eigen::VectorXd> vec(xi + 3 * i, 3);
            P.col(i) = vec;
            // P.col(i) = xi.segment(3 * i, 3);
        }
        return;
    }

    template <typename EIGENVEC>
    static inline void backwardP(const Eigen::Matrix3Xd &P,
                                 EIGENVEC &xi)
    {
        const int sizeP = P.cols();
        for (int i = 0; i < sizeP; ++i)
        {
            xi.segment(3 * i, 3) = P.col(i);
        }
        return;
    }

    // tao--->T
    static inline void forwardT(const Eigen::VectorXd &tau,
                                Eigen::VectorXd &T)
    {
        const int sizeTau = tau.size();
        T.resize(sizeTau);
        for (int i = 0; i < sizeTau; i++)
        {
            T(i) = tau(i) > 0.0
                       ? ((0.5 * tau(i) + 1.0) * tau(i) + 1.0)
                       : 1.0 / ((0.5 * tau(i) - 1.0) * tau(i) + 1.0);
        }
        return;
    }
    // tao--->T
    static inline void forwardT(const double *tau,
                                Eigen::VectorXd &T, const int Time_dim)
    {
        T.resize(Time_dim);
        double temp;
        for (int i = 0; i < Time_dim; i++)
        {
            temp = *(tau + i);
            T(i) = temp > 0.0
                       ? ((0.5 * temp + 1.0) * temp + 1.0)
                       : 1.0 / ((0.5 * temp - 1.0) * (temp) + 1.0);
        }
        return;
    }
    // T--->tao
    template <typename EIGENVEC>
    static inline void backwardT(const Eigen::VectorXd &T,
                                 EIGENVEC &tau)
    {
        const int sizeT = T.size();
        tau.resize(sizeT);
        for (int i = 0; i < sizeT; i++)
        {
            tau(i) = T(i) > 1.0
                         ? (sqrt(2.0 * T(i) - 1.0) - 1.0)
                         : (1.0 - sqrt(2.0 / T(i) - 1.0));
        }
        return;
    }

    template <typename EIGENVEC>
    static inline void backwardGradT(const Eigen::VectorXd &tau,
                                     const Eigen::VectorXd &gradT,
                                     EIGENVEC &gradTau)
    {
        const int sizeTau = tau.size();
        gradTau.resize(sizeTau);
        double denSqrt;

        for (int i = 0; i < sizeTau; i++)
        {
            if (tau(i) > 0)
            {
                gradTau(i) = gradT(i) * (tau(i) + 1.0);
            }
            else
            {
                denSqrt = (0.5 * tau(i) - 1.0) * tau(i) + 1.0;
                gradTau(i) = gradT(i) * (1.0 - tau(i)) / (denSqrt * denSqrt);
            }
        }

        return;
    }

    static inline void backwardGradT(const double *tau,
                                     const Eigen::VectorXd &gradT,
                                     double *gradTau,
                                     const int sizeTau)
    {

        double denSqrt;

        for (int i = 0; i < sizeTau; i++)
        {
            if (tau[i] > 0)
            {
                gradTau[i] = gradT[i] * (tau[i] + 1.0);
            }
            else
            {
                denSqrt = (0.5 * tau[i] - 1.0) * tau[i] + 1.0;
                gradTau[i] = gradT[i] * (1.0 - tau[i]) / (denSqrt * denSqrt);
            }
        }

        return;
    }

    template <typename EIGENVEC>
    static inline void backwardGradP(
        const Eigen::Matrix3Xd &gradP,
        EIGENVEC &gradXi)
    {
        const int sizeP = gradP.cols();
        for (int i = 0; i < sizeP; ++i)
        {
            gradXi.segment(3 * i, 3) = gradP.col(i);
        }
        return;
    }

    static inline void backwardGradP(
        const Eigen::Matrix3Xd &gradP,
        double *gradXi,
        const int Space_dim)
    {
        for (int i = 0; i < Space_dim; ++i)
        {
            Eigen::Map<Eigen::VectorXd>(gradXi + 3 * i, 3) = gradP.col(i); 
        }
        return;
    }

    static inline bool smoothedL1(const double &x,
                                  const double &mu,
                                  double &f,
                                  double &df)
    {
        if (x < 0.0)
        {
            return false;
        }
        else if (x > mu)
        {
            f = x - 0.5 * mu;
            df = 1.0;
            return true;
        }
        else
        {
            const double xdmu = x / mu;
            const double sqrxdmu = xdmu * xdmu;
            const double mumxd2 = mu - 0.5 * x;
            f = mumxd2 * sqrxdmu * xdmu;
            df = sqrxdmu * ((-0.5) * xdmu + 3.0 * mumxd2 / mu);
            return true;
        }
    }

    static inline double costFunctionLmbm(void *ptr,
                                          const double *x_variable,
                                          double *g,
                                          const int n)
    {
        TrajOptimizer &obj = *(TrajOptimizer *)ptr;
        obj.cost_iter++;
        const int dimTau = obj.temporalDim;
        const int dimXi = obj.spatialDim;
        const double weightT = obj.rho;
        forwardT(x_variable, obj.times, dimTau);
        forwardP((x_variable + dimTau), obj.points, dimXi / 3);

        double energy_cost = 0.0;
        double pos_cost = 0.0;
        double dyn_cost = 0.0;
        double time_cost = 0.0;

        double cost;
        obj.minco.setParameters(obj.points, obj.times);
        obj.minco.getEnergy(cost);
        obj.minco.getEnergyPartialGradByCoeffs(obj.partialGradByCoeffs); // ∂E/∂c
        obj.minco.getEnergyPartialGradByTimes(obj.partialGradByTimes);   // ∂E/∂T
        obj.minco.getTrajectory(obj.step_traj);
        obj.sv_manager->updateTraj(obj.step_traj);
        energy_cost = cost;

        auto start1 = chrono::high_resolution_clock::now();
        addSaftyPenaOnSweptVolumeParallel(ptr,
                                          obj.times,
                                          obj.minco.getCoeffs(),
                                          cost,
                                          obj.partialGradByTimes,
                                          obj.partialGradByCoeffs);
        auto end1 = chrono::high_resolution_clock::now();
        auto duration = chrono::duration_cast<chrono::nanoseconds>(end1 - start1);
        pos_cost = cost - energy_cost;

        double pos_cost_tmp;
        auto start2 = chrono::high_resolution_clock::now();

        addTimeIntPenaltyParallel(ptr,
                                  obj.times,
                                  obj.minco.getCoeffs(),
                                  cost,
                                  pos_cost_tmp,
                                  obj.partialGradByTimes,
                                  obj.partialGradByCoeffs);

        auto end2 = chrono::high_resolution_clock::now();
        auto duration2 = chrono::duration_cast<chrono::nanoseconds>(end2 - start2);

        obj.integration_parallel_ms += duration2.count() / 1000000.0;
        obj.discrete_parallel_ms += duration.count() / 1000000.0;

        pos_cost += pos_cost_tmp;
        dyn_cost = cost - pos_cost - energy_cost;

        obj.minco.propogateGrad(obj.partialGradByCoeffs, obj.partialGradByTimes, // ∂Cost/∂c,T -> ∂Cost/∂q,T
                                obj.gradByPoints, obj.gradByTimes);
        cost += weightT * obj.times.sum();
        time_cost = cost - pos_cost - energy_cost - dyn_cost;

        obj.gradByTimes.array() += weightT;
        obj.cost_pos = pos_cost;
        obj.cost_other = cost - pos_cost;
        obj.cost_total = cost;

        backwardGradT(x_variable, obj.gradByTimes, g, dimTau);
        backwardGradP(obj.gradByPoints, g + dimTau, dimXi / 3);

        return cost;
    }

    static inline void addTimeIntPenaltyParallel(void *ptr,
                                                 const Eigen::VectorXd &T,
                                                 const Eigen::MatrixX3d &coeffs,
                                                 double &cost,
                                                 double &pos_cost,
                                                 Eigen::VectorXd &gradT,
                                                 Eigen::MatrixX3d &gradC)
    {
        TrajOptimizer &obj = *(TrajOptimizer *)ptr;
        const double velSqrMax = obj.vmax * obj.vmax;     
        const double omgSqrMax = obj.omgmax * obj.omgmax; 
        const double thetaMax = obj.thetamax;             
        const double weightVel = obj.weight_v;
        const double weightPos = obj.weight_p;
        const double weightOmg = obj.weight_omg;
        const double weightTheta = obj.weight_theta;
        const int pieceNum = T.size();
        const int integralResolution = obj.integralRes;
        const double smoothFactor = obj.smooth_fac;
        const double integralFrac = 1.0 / integralResolution;
        pos_cost = 0;
#pragma omp parallel for num_threads(obj.threads_num) schedule(dynamic) 
        for (int count = 0; count < pieceNum * (integralResolution + 1); count++)
        {
            int j = count % (integralResolution + 1);
            int i = count / (integralResolution + 1);

            double step = T(i) * integralFrac;
            const Eigen::Matrix<double, 6, 3> &c = coeffs.block<6, 3>(i * 6, 0);

            Eigen::Vector4d quat;
            Eigen::Vector3d omg;
            Eigen::Vector4d gradQuat, violaQuatPenaGrad;
            Eigen::Vector3d gradVel, gradAcc, gradPos, gradOmg, violaPosPenaGrad;
            Eigen::Vector3d gradVelTotal, gradAccTotal, gradPosTotal, gradJerTotal;
            Eigen::Vector3d outerNormal;

            double violaVelPenaD, violaOmgPenaD, violaThetaPenaD;
            double violaVelPena, violaOmgPena, violaPosPena, violaThetaPena;
            double totalGradPsi, totalGradPsiD, thr, gradThr;
            double s1 = j * step;
            double s2 = s1 * s1;
            double s3 = s2 * s1;
            double s4 = s2 * s2;
            double s5 = s4 * s1;

            Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3, beta4;
            beta0(0) = 1.0, beta0(1) = s1, beta0(2) = s2, beta0(3) = s3, beta0(4) = s4, beta0(5) = s5;
            beta1(0) = 0.0, beta1(1) = 1.0, beta1(2) = 2.0 * s1, beta1(3) = 3.0 * s2, beta1(4) = 4.0 * s3, beta1(5) = 5.0 * s4;
            beta2(0) = 0.0, beta2(1) = 0.0, beta2(2) = 2.0, beta2(3) = 6.0 * s1, beta2(4) = 12.0 * s2, beta2(5) = 20.0 * s3;
            beta3(0) = 0.0, beta3(1) = 0.0, beta3(2) = 0.0, beta3(3) = 6.0, beta3(4) = 24.0 * s1, beta3(5) = 60.0 * s2;
            beta4(0) = 0.0, beta4(1) = 0.0, beta4(2) = 0.0, beta4(3) = 0.0, beta4(4) = 24.0, beta4(5) = 120.0 * s1;
            Eigen::Vector3d pos = c.transpose() * beta0;
            Eigen::Vector3d vel = c.transpose() * beta1;
            Eigen::Vector3d acc = c.transpose() * beta2;
            Eigen::Vector3d jer = c.transpose() * beta3;
            Eigen::Vector3d sna = c.transpose() * beta4;


            obj.flatmap.optimizated_forward(vel, acc, jer, quat, omg);
            Eigen::Matrix3d rotate = Eigen::Quaterniond(quat[0], quat[1], quat[2], quat[3]).toRotationMatrix(); 
            Eigen::Matrix3d St = obj.sv_manager->getScale(s1);                                                 
            double pena = 0.0;
            gradVel.setZero();
            gradAcc.setZero();
            gradPos.setZero();
            gradOmg.setZero();
            gradQuat.setZero();
            gradPosTotal.setZero();
            gradVelTotal.setZero();
            gradAccTotal.setZero();
            gradJerTotal.setZero();

            double cos_theta = 1.0 - 2.0 * (quat(1) * quat(1) + quat(2) * quat(2));
            double violaVel = vel.squaredNorm() - velSqrMax;
            double violaOmg = omg.squaredNorm() - omgSqrMax;
            double violaTheta = acos(cos_theta) - thetaMax;
            double node = (j == 0 || j == integralResolution) ? 0.5 : 1.0;
            double alpha = j * integralFrac;



            if (smoothedL1(violaVel, smoothFactor, violaVelPena, violaVelPenaD))
            {
                gradVel += weightVel * violaVelPenaD * 2.0 * vel; 
                pena += weightVel * violaVelPena;
            }

  
            if (smoothedL1(violaOmg, smoothFactor, violaOmgPena, violaOmgPenaD))
            {
                gradOmg += weightOmg * violaOmgPenaD * 2.0 * omg;
                pena += weightOmg * violaOmgPena;
            }

        
            if (smoothedL1(violaTheta, smoothFactor, violaThetaPena, violaThetaPenaD))
            {
                gradQuat += weightTheta * violaThetaPenaD /
                            sqrt(1.0 - cos_theta * cos_theta) * 4.0 *
                            Eigen::Vector4d(0.0, quat(1), quat(2), 0.0);
                pena += weightTheta * violaThetaPena;
            }
           
            obj.flatmap.backwardthreadsafe(vel, acc, jer, gradPos, gradVel, gradQuat, gradOmg, gradPosTotal, gradVelTotal, gradAccTotal, gradJerTotal);
#pragma omp critical 
            {
                gradC.block<6, 3>(i * 6, 0) += (beta0 * gradPosTotal.transpose() +
                                                beta1 * gradVelTotal.transpose() +
                                                beta2 * gradAccTotal.transpose() +
                                                beta3 * gradJerTotal.transpose()) *
                                               node * step;
                gradT(i) += (gradPosTotal.dot(vel) +
                             gradVelTotal.dot(acc) +
                             gradAccTotal.dot(jer) +
                             gradJerTotal.dot(sna)) *
                                alpha * node * step +
                            node * integralFrac * pena;
                cost += node * step * pena;
            }
        }
        return;
    }


    static inline void addSaftyPenaOnSweptVolumeParallel(void *ptr,
                                                         const Eigen::VectorXd &T,
                                                         const Eigen::MatrixX3d &coeffs,
                                                         double &cost,
                                                         Eigen::VectorXd &gradT,
                                                         Eigen::MatrixX3d &gradC /**,bool output=false*/)
    {

        TrajOptimizer &obj = *(TrajOptimizer *)ptr;
        const double weightPos = obj.weight_p;
        const double neighbor_dis = 2 * obj.conf.occupancy_resolution;
        const int pieceNum = T.size();

#pragma omp parallel for num_threads(obj.threads_num) schedule(dynamic)
        for (int pointindex = 0; pointindex < obj.parallel_points_num; ++pointindex)
        {

            Eigen::Vector3d gradp_rel;
            Eigen::Vector3d pos_eva = obj.parallel_points[pointindex];                                                         
            double sdf_value = obj.sv_manager->getSDFofSweptVolume<true>(pos_eva, obj.lastTstar[pointindex], gradp_rel, false); 
          
            double time_seed_f = obj.lastTstar[pointindex]; 
            double time_local = time_seed_f;
            int i = obj.step_traj.locatePieceIdx(time_local);

            const Eigen::Matrix<double, 6, 3> &c = coeffs.block<6, 3>(i * 6, 0); 
            double s1 = time_local;
            double s2 = s1 * s1;
            double s3 = s2 * s1;
            double s4 = s2 * s2;
            double s5 = s4 * s1;
            Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3, beta4;
            beta0(0) = 1.0, beta0(1) = s1, beta0(2) = s2, beta0(3) = s3, beta0(4) = s4, beta0(5) = s5;
            beta1(0) = 0.0, beta1(1) = 1.0, beta1(2) = 2.0 * s1, beta1(3) = 3.0 * s2, beta1(4) = 4.0 * s3, beta1(5) = 5.0 * s4;
            beta2(0) = 0.0, beta2(1) = 0.0, beta2(2) = 2.0, beta2(3) = 6.0 * s1, beta2(4) = 12.0 * s2, beta2(5) = 20.0 * s3;
            beta3(0) = 0.0, beta3(1) = 0.0, beta3(2) = 0.0, beta3(3) = 6.0, beta3(4) = 24.0 * s1, beta3(5) = 60.0 * s2;
            beta4(0) = 0.0, beta4(1) = 0.0, beta4(2) = 0.0, beta4(3) = 0.0, beta4(4) = 24.0, beta4(5) = 120.0 * s1;

            Eigen::Vector3d pos = c.transpose() * beta0;
            Eigen::Vector3d vel = c.transpose() * beta1;
            Eigen::Vector3d acc = c.transpose() * beta2;
            Eigen::Vector3d jer = c.transpose() * beta3;
            Eigen::Vector3d sna = c.transpose() * beta4;
            Eigen::Vector3d gradp_eva, gradVel, gradPos, gradOmg, violaPosPenaGrad, gradPosTotal, gradVelTotal, gradAccTotal, gradJerTotal;
            Eigen::Vector4d gradQuat, quat, violaQuatPenaGrad;
            gradp_eva.setZero();
            gradVel.setZero();
            gradPos.setZero();
            gradOmg.setZero();
            gradQuat.setZero();
            gradPosTotal.setZero();
            gradVelTotal.setZero();
            gradAccTotal.setZero();
            gradJerTotal.setZero();
            violaPosPenaGrad.setZero();

            obj.flatmap.optimizated_forward(vel, acc, jer, quat);
            Eigen::Matrix3d rotate = Eigen::Quaterniond(quat[0], quat[1], quat[2], quat[3]).toRotationMatrix();
            Eigen::Matrix3d St = obj.sv_manager->getScale(time_seed_f);
            double pena = 0.0;
            double violaPosPena = 0.0;

            if (obj.grad_cost_p_sw(pos_eva, pos, rotate, quat,
                                   St, sdf_value, gradp_eva, gradp_rel,
                                   violaPosPenaGrad, violaQuatPenaGrad, violaPosPena))
            {
                gradPos += weightPos * violaPosPenaGrad;
                gradQuat += weightPos * violaQuatPenaGrad;
                pena += weightPos * violaPosPena;
            }
            obj.flatmap.backwardthreadsafe(vel, acc, jer, gradPos, gradVel, gradQuat, gradOmg, gradPosTotal, gradVelTotal, gradAccTotal, gradJerTotal);
            Eigen::Matrix<double, 6, 3> gdC = (beta0 * gradPosTotal.transpose() +
                                               beta1 * gradVelTotal.transpose() +
                                               beta2 * gradAccTotal.transpose() +
                                               beta3 * gradJerTotal.transpose());

            double gdT = (-gradPosTotal.dot(vel) +
                          -gradVelTotal.dot(acc) +
                          -gradAccTotal.dot(jer) +
                          -gradJerTotal.dot(sna));

#pragma omp critical
            {
                cost += pena;
                gradC.block<6, 3>(i * 6, 0) += gdC;
                for (int j = 0; j < i; j++)
                {
                    gradT(j) += gdT;
                }
            }
        }
        return;
    }

public:
    TrajOptimizer()
    {

        x_variable = NULL;
    }
    ~TrajOptimizer()
    {

        if (x_variable != NULL)
        {
            free(x_variable);
            x_variable = NULL;
        }
    }

    void setParam(ros::NodeHandle &nh, Config &config)
    {
        this->conf = config;
        this->nh = nh;

        vis.reset(new vis::Visualization(nh));

        optimizer_type = USE_DES_VER;
        map_type = USE_GRID_MAP;

        execution_type = PARALLEL;
        vmax = config.vmax;
        omgmax = config.omgmax;
        thetamax = config.thetamax;
        threads_num = config.threads_num;
        rho = config.rho;
        weight_a = config.weight_a;
        weight_p = config.weight_p;
        weight_v = config.weight_v;
        weight_omg = config.weight_omg;
        weight_theta = config.weight_theta;

        smooth_fac = config.smoothingEps;
        integralRes = config.integralIntervs;
        safety_hor = config.safety_hor;
        bdx = bdy = bdz = config.kernel_size * config.occupancy_resolution;


        ROS_WARN_STREAM("==[back_end]==");
        ROS_WARN_STREAM("==vmax==" << vmax);
        ROS_WARN_STREAM("==omgmax==" << omgmax);
        ROS_WARN_STREAM("==thetamax==" << thetamax);
        ROS_WARN_STREAM("==rho==" << rho);
        ROS_WARN_STREAM("==weight_v==" << weight_v);
        ROS_WARN_STREAM("==weight_a==" << weight_a);
        ROS_WARN_STREAM("==weight_p==" << weight_p);
        ROS_WARN_STREAM("==smooth_fac==" << smooth_fac);
        ROS_WARN_STREAM("==safety_hor==" << safety_hor);
        ROS_WARN_STREAM("==init time allocate perpiece for minco==" << config.inittime); 
        ROS_WARN_STREAM("bounding box bdx" << bdx);
        ROS_WARN_STREAM("bounding box bdy" << bdy);
        ROS_WARN_STREAM("bounding box bdz" << bdz);

        test_obs = Eigen::Map<Eigen::Matrix3Xd>(config.test_obs.data(), 3, config.test_obs.size() / 3);

        // flatmap;
        flatmap.reset(config.vehicleMass, config.gravAcc, config.horizDrag,
                      config.vertDrag, config.parasDrag, config.speedEps);

        debug_pub = nh.advertise<visualization_msgs::Marker>("/traj_opt/debug_path", 10);
        debugesdfpub = nh.advertise<std_msgs::Float32MultiArray>("/traj_opt/esdf", 10);
        debug_wplists_pub = nh.advertise<visualization_msgs::MarkerArray>("/traj_opt/debug_wplists", 10); 
        debug_wp_pub = nh.advertise<visualization_msgs::Marker>("/traj_opt/debug_wp", 10);
        debug_vec_pub = nh.advertise<visualization_msgs::Marker>("/traj_opt/debug_vec", 10);
        test_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/traj_opt/debug_test_map", 10);
    }

    void setEnvironment(SweptVolumeManager::Ptr sv) { sv_manager = sv; }
    void setGridMap(PCSmapManager::Ptr psm) { pcsmap_manager = psm; }

    void clearvisAABBpoints();


    bool optimize_traj(const Eigen::Matrix3d &initS,
                       const Eigen::Matrix3d &finalS,
                       const Eigen::VectorXd &opt_x,
                       const int N,
                       Trajectory<TRAJ_ORDER> &traj);

    int optimize_traj_lmbm(const Eigen::Matrix3d &initS,
                           const Eigen::Matrix3d &finalS,
                           const std::vector<Eigen::Vector3d> &Q,
                           const Eigen::VectorXd &T,
                           const int N,
                           Trajectory<TRAJ_ORDER> &traj);

    int optimize_traj_lmbm(const Eigen::Matrix3d &initS,
                           const Eigen::Matrix3d &finalS,
                           const Eigen::VectorXd &opt_x,
                           const int N,
                           Trajectory<TRAJ_ORDER> &traj);

    /**
     * @return when x > 0: x^3 ，else: 0
     */
    inline void cubic(double x, double &cost_x, double &grad_x)
    {
        if (x < 0)
        {
            cost_x = 0;
            grad_x = 0;
            return;
        }
        cost_x = pow(x, 3);
        grad_x = 3 * x * x;
        return;
    }


    bool inline grad_cost_p(const Eigen::Vector3d &pos,
                            const Eigen::Matrix3d &R,
                            const Eigen::Matrix3d &St,
                            const Eigen::Vector4d &quat,
                            Eigen::Vector3d &gradp,
                            Eigen::Vector4d &grad_quat,
                            double &costp)
    {
        costp = 0.0;
        gradp.setZero();
        grad_quat.setZero();

        double sdf_value = 0.0;
        double sdf_cost = 0.0;
        double grad_out = 0.0;
        Eigen::Vector3d sdf_grad{Eigen::Vector3d::Zero()};
        Eigen::Vector3d gradp_rel{Eigen::Vector3d::Zero()};
        Eigen::Vector3d p_minus_x;
        Eigen::Vector3d pos_eva;
        Eigen::Vector4d step_gradq;

        double turcation = safety_hor; 
        std::vector<Eigen::Vector3d> ob_pts;
        ob_pts.clear();

        pcsmap_manager->getPointsInAABB(pos, bdx / 2, bdy / 2, bdz / 2, ob_pts);
      
        Eigen::Vector3d p_rel;
        for (int i = 0; i < ob_pts.size(); i++) 
        {
            pos_eva = ob_pts[i];
            p_minus_x = pos_eva - pos;
            step_gradq.setZero();

            p_rel = R.transpose() * St.inverse() * p_minus_x;                                    
            if (abs(p_rel.x()) > bdx / 2 || abs(p_rel.y()) > bdy / 2 || abs(p_rel.z()) > bdz / 2) 
            {
                continue; 
            }
         
            sdf_value = sv_manager->getSDFWithGradWhenRobotAtState(pos_eva, gradp_rel, pos, R, St);
          
            sdf_cost = 0;
          
            smoothedL1(turcation - sdf_value, smooth_fac, sdf_cost, grad_out); 
            sdf_grad = -grad_out * (-(St.inverse()).transpose() * R * gradp_rel);
            if (sdf_cost > 0)
            {
                costp += sdf_cost;
                gradp += sdf_grad; 

                step_gradq(0) = gradp_rel.transpose() * getQuatTransDW(quat) * p_minus_x;
                step_gradq(1) = gradp_rel.transpose() * getQuatTransDX(quat) * p_minus_x;
                step_gradq(2) = gradp_rel.transpose() * getQuatTransDY(quat) * p_minus_x;
                step_gradq(3) = gradp_rel.transpose() * getQuatTransDZ(quat) * p_minus_x;
                grad_quat += -grad_out * step_gradq;
            }
        }
        return (costp > 0);
    }


    bool inline grad_cost_p_sw(const Eigen::Vector3d &pos_eva,
                               const Eigen::Vector3d &pos_obj, 
                               const Eigen::Matrix3d &rotate,
                               const Eigen::Vector4d &quat,
                               const Eigen::Matrix3d &St,
                               const double sdf_value,
                               const Eigen::Vector3d &gradp_eva,
                               Eigen::Vector3d &gradp_rel,
                               Eigen::Vector3d &gradp,
                               Eigen::Vector4d &grad_quat,
                               double &costp)
    {
        // std::cout<<"success"<<std::endl;
        costp = 0;
        gradp.setZero();
        grad_quat.setZero();

        double sdf_cost = -1.0;
        Eigen::Vector3d sdf_grad;
        Eigen::Vector3d p_minus_x;
        Eigen::Vector4d step_gradq;

        step_gradq.setZero();
        double sdf_out_grad = 0.0;

        smoothedL1(safety_hor - sdf_value, 0.01, sdf_cost, sdf_out_grad);
        sdf_grad = -sdf_out_grad * (-(St.inverse()).transpose() * rotate * gradp_rel);
        if (sdf_cost > 0)
        {

            costp += sdf_cost;
            gradp += sdf_grad; 
            p_minus_x = pos_eva - pos_obj;
            step_gradq(0) = gradp_rel.transpose() * getQuatTransDW(quat) * p_minus_x;
            step_gradq(1) = gradp_rel.transpose() * getQuatTransDX(quat) * p_minus_x;
            step_gradq(2) = gradp_rel.transpose() * getQuatTransDY(quat) * p_minus_x;
            step_gradq(3) = gradp_rel.transpose() * getQuatTransDZ(quat) * p_minus_x;
            grad_quat += -sdf_out_grad * step_gradq;
        }
        return (costp > DBL_EPSILON);
    }

    static void AsyncSleepMS(const int ms)
    {
        auto start = std::chrono::steady_clock::now();
        auto duration = std::chrono::milliseconds(ms);
        bool done = false;
        while (!done)
        {
            auto elapsed = std::chrono::steady_clock::now() - start;
            if (elapsed > duration)
            {
                done = true;
            }
            else
            {
                ros::spinOnce();
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
    }

    static inline int earlyexitLmbm(void *instance,
                                    const double *x,
                                    const int k)
    {
        std::cout << "\033[32lmbm miter:\033[0m" << k << std::endl;
        TrajOptimizer &obj = *(TrajOptimizer *)instance;
        obj.iter++;
        if (obj.conf.enableearlyExit)
        {
            obj.drawDebug();
            vector<double> costs;
            costs.push_back(obj.cost_pos);
            costs.push_back(obj.cost_other);
            costs.push_back(obj.cost_other);

            debug_publisher::DBSendLogCost(costs);
            // NOTE pauseea
            AsyncSleepMS(obj.conf.debugpause);
            std::cout << "[total cost] = " << obj.cost_total << " , [other cost] = " << obj.cost_other << std::endl;

            while (obj.pause)
            {
                if (obj.next_step)
                {
                    obj.next_step = false;
                    break;
                }
                AsyncSleepMS(obj.conf.debugpause);
            }
        }

        if (obj.exit)
        {
            obj.exit = false;
            return 1;
        }

        return obj.iter > 5 * 1e4;
        // return 0;
    }

    static inline int earlyExit(void *instance,
                                const Eigen::VectorXd &x,
                                const Eigen::VectorXd &g,
                                const double fx,
                                const double step,
                                const int k,
                                const int ls)
    {
        TrajOptimizer &obj = *(TrajOptimizer *)instance;
        obj.iter++;
        std::cout << "lbfgs iter = " << obj.iter << " step = " << step << std::endl;
        if (obj.conf.enableearlyExit)
        {
            obj.drawDebug();
            vector<double> costs;
            costs.push_back(obj.cost_pos);
            costs.push_back(obj.cost_other);
            costs.push_back(obj.cost_other);

            debug_publisher::DBSendLogCost(costs);
            AsyncSleepMS(obj.conf.debugpause);
            std::cout << "[total cost] = " << obj.cost_total << " , [other cost] = " << obj.cost_other << std::endl;

            while (obj.pause)
            {
                if (obj.next_step)
                {
                    obj.next_step = false;
                    break;
                }
                AsyncSleepMS(obj.conf.debugpause);
            }
        }

        if (obj.exit)
        {
            obj.exit = false;
            return 1;
        }

        // return obj.iter > 5 * 1e4;
        return obj.iter > 1e4;
    }

   


public:
    typedef shared_ptr<TrajOptimizer> Ptr;
};
