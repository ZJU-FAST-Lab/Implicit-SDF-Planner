#pragma once

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <map_manager/PCSmap_manager.h>
#include <swept_volume/sw_manager.hpp>

#include <utils/minco.hpp>
#include <utils/config.hpp>
#include <utils/flatness.hpp>
#include <utils/lbfgs.hpp>
#include <utils/debug_publisher.hpp>

#define TRAJ_ORDER 5

#define GNOW ros::Time::now()
#define DEBUGOUT(x) std::cout << x << std::endl

class OriTraj
{
public:
    // ros
    ros::NodeHandle nh;
    ros::Publisher debug_pub;

    minco::MINCO_S3NU minco;
    flatness::FlatnessMap flatmap;
    Trajectory<TRAJ_ORDER> step_traj;

    double rho;
    double vmax;
    double omgmax;
    double weight_v;
    double weight_omg;

    int integralRes;
    Eigen::Matrix3d initState;
    Eigen::Matrix3d finalState;

    Eigen::Matrix3Xd accelerations; 
    Eigen::Matrix3Xd ref_points;    
    std::vector<Eigen::Matrix3d> att_constraints;

    Eigen::Matrix3Xd points; 
    Eigen::VectorXd times;
    Eigen::Matrix3Xd gradByPoints;
    Eigen::VectorXd gradByTimes;
    Eigen::MatrixX3d partialGradByCoeffs;
    Eigen::VectorXd partialGradByTimes;

    int pieceN;
    int spatialDim;
    int temporalDim;

    double smooth_fac;

    Config conf;
    double weightPR;
    double weightAR;

    int iter = 0;
    void drawDebugTraj();

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

    template <typename EIGENVEC>
    static inline void backwardGradP(const Eigen::VectorXd &xi,
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


    bool inline grad_cost_dir(const Eigen::Vector3d &pos,
                              Eigen::Vector3d &gradp,
                              double &cost_p,
                              const int indexi) // j=0-obj.integralRes
    {

        cost_p = 0.0;
        gradp.setZero();
        Eigen::Vector3d ref_pos = ref_points.col(indexi);
        Eigen::Vector3d pos_diff = pos - ref_pos;
        cost_p = pow(pos_diff.norm(), 3);
        gradp = 3 * pow(pos_diff.norm(), 2) * pos_diff.normalized();
        double cost = 0.0;
        cost += cost_p;
        return (cost > 0);
    }

    static inline void addPosePenalty(void *ptr,
                                      const Eigen::VectorXd &T,
                                      const Eigen::MatrixX3d &coeffs,
                                      double &cost,
                                      Eigen::VectorXd &gradT,
                                      Eigen::MatrixX3d &gradC)
    {
        OriTraj &obj = *(OriTraj *)ptr;
        ros::Time begin_stamp = ros::Time::now();
        Eigen::Vector3d pos, vel, acc, jer;
        Eigen::Vector3d grad_tmp_p;
        // double thr;
        Eigen::Vector4d quat;
        double cost_tmp, cost_tmp_p, cost_tmp_a;
        Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3;
        Eigen::Matrix<double, 6, 3> gradViolaPc, gradViolaVc, gradViolaAc;
        double s1, s2, s3, s4, s5;
        double gradViolaPt, gradViolaVt, gradViolaAt;

        int innerLoop;

        int constraints_count = obj.pieceN - 1;
        assert(constraints_count == obj.points.cols());
        int segment;
        const double alpha = 1.0 / obj.integralRes;
        ;
        // const double alpha = 0.0;
        for (int i = 0; i < constraints_count; ++i)
        {

            segment = i + 1;
            const auto &c = coeffs.block<6, 3>(segment * 6, 0);

            s1 = alpha * T(segment);
            s2 = s1 * s1;
            s3 = s2 * s1;
            s4 = s2 * s2;
            s5 = s4 * s1;
            beta0 << 1.0, s1, s2, s3, s4, s5;
            beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
            beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
            beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;

            pos = c.transpose() * beta0;
            vel = c.transpose() * beta1;
            acc = c.transpose() * beta2;
            jer = c.transpose() * beta3;

            if (obj.grad_cost_dir(pos, grad_tmp_p, cost_tmp_p, i))
            {

                gradViolaPc = beta0 * grad_tmp_p.transpose();
                gradViolaPt = alpha * grad_tmp_p.dot(vel);

                gradC.block<6, 3>(segment * 6, 0) += obj.weightPR * gradViolaPc;
                gradT(segment) += obj.weightPR * (cost_tmp_p * gradViolaPt);
                cost += obj.weightPR * cost_tmp_p;
            }
        }
    }

    static inline double costFunction(void *ptr,
                                      const Eigen::VectorXd &x,
                                      Eigen::VectorXd &g,
                                      double &p_cost)
    {
        OriTraj &obj = *(OriTraj *)ptr;
        const int dimTau = obj.temporalDim;
        const int dimXi = obj.spatialDim;
        const double weightT = obj.rho;
        Eigen::Map<const Eigen::VectorXd> tau(x.data(), dimTau);
        Eigen::Map<const Eigen::VectorXd> xi(x.data() + dimTau, dimXi);
        Eigen::Map<Eigen::VectorXd> gradTau(g.data(), dimTau);
        Eigen::Map<Eigen::VectorXd> gradXi(g.data() + dimTau, dimXi);
        forwardT(tau, obj.times); // tao--->T
        forwardP(xi, obj.points);

        double cost = 0;
        obj.minco.setParameters(obj.points, obj.times);

        obj.minco.getEnergy(cost);
        obj.minco.getEnergyPartialGradByCoeffs(obj.partialGradByCoeffs); // ∂E/∂c
        obj.minco.getEnergyPartialGradByTimes(obj.partialGradByTimes);   // ∂E/∂T

        obj.minco.getTrajectory(obj.step_traj);

        addPosePenalty(ptr,
                       obj.times,
                       obj.minco.getCoeffs(),
                       cost,
                       obj.partialGradByTimes,
                       obj.partialGradByCoeffs);


        obj.minco.propogateGrad(obj.partialGradByCoeffs, obj.partialGradByTimes, // ∂Cost/∂c,T -> ∂Cost/∂q,T
                                obj.gradByPoints, obj.gradByTimes);

        cost += weightT * obj.times.sum();

        obj.gradByTimes.array() += weightT;
        backwardGradT(tau, obj.gradByTimes, gradTau);
        backwardGradP(xi, obj.gradByPoints, gradXi);
        return cost;
    }

public:
    OriTraj() {}
    ~OriTraj() {}

    void setParam(ros::NodeHandle &nh, Config &config)
    {
        this->conf = config;
        this->nh = nh;

        weightPR = config.weight_pr;
        weightAR = config.weight_ar;
        rho = config.rho_mid_end;
        vmax = config.vmax;
        omgmax = config.omgmax;
        weight_v = config.weight_v;
        weight_omg = config.weight_omg;
        smooth_fac = config.smoothingEps;
        integralRes = config.integralIntervs;
        ROS_WARN_STREAM("==smooth_fac==" << smooth_fac);
        ROS_WARN_STREAM("==rho==" << rho);

        Eigen::VectorXd physicalParams(6);
        physicalParams(0) = config.vehicleMass;
        physicalParams(1) = config.gravAcc;
        physicalParams(2) = config.horizDrag;
        physicalParams(3) = config.vertDrag;
        physicalParams(4) = config.parasDrag;
        physicalParams(5) = config.speedEps;
        // flatmap;
        flatmap.reset(physicalParams(0), physicalParams(1), physicalParams(2),
                      physicalParams(3), physicalParams(4), physicalParams(5));

        debug_pub = nh.advertise<visualization_msgs::Marker>("/ori_traj/debug_traj", 10);
    }

    bool getOriTraj(const Eigen::Matrix3d initS,
                    const Eigen::Matrix3d finalS,
                    const std::vector<Eigen::Vector3d> &Q,
                    Eigen::VectorXd T,
                    std::vector<Eigen::Vector3d> acc_list,
                    std::vector<Eigen::Matrix3d> rot_list,
                    const int N,
                    Trajectory<TRAJ_ORDER> &traj,
                    Eigen::VectorXd &opt_x);

    static inline double costaltitude(const Eigen::Vector4d &wxyz_rotate, const Eigen::Matrix3d &rotate_ref)
    {
        const double w = wxyz_rotate(0);
        const double x = wxyz_rotate(1);
        const double y = wxyz_rotate(2);
        const double z = wxyz_rotate(3);
        const double a0 = rotate_ref(0, 0);
        const double a1 = rotate_ref(0, 1);
        const double a2 = rotate_ref(0, 2);
        const double b0 = rotate_ref(1, 0);
        const double b1 = rotate_ref(1, 1);
        const double b2 = rotate_ref(1, 2);
        const double c0 = rotate_ref(2, 0);
        const double c1 = rotate_ref(2, 1);
        const double c2 = rotate_ref(2, 2);
        double cost = 2 * a0 * (2 * y * y + 2 * z * z - 1) + 2 * b1 * (2 * x * x + 2 * z * z - 1) + 2 * c2 * (2 * x * x + 2 * y * y - 1) + 2 * a1 * (2 * w * z - 2 * x * y) - 2 * a2 * (2 * w * y + 2 * x * z) - 2 * b0 * (2 * w * z + 2 * x * y) + 2 * b2 * (2 * w * x - 2 * y * z) + 2 * c0 * (2 * w * y - 2 * x * z) - 2 * c1 * (2 * w * x + y * z) + 6;
        return cost;
    }
    static inline void gradaltitude(const Eigen::Vector4d &wxyz_rotate, const Eigen::Matrix3d &rotate_ref, double &dfdw, double &dfdx, double &dfdy, double &dfdz)
    {
        dfdw = 0;
        dfdx = 0;
        dfdy = 0;
        dfdz = 0;
        const double w = wxyz_rotate(0);
        const double x = wxyz_rotate(1);
        const double y = wxyz_rotate(2);
        const double z = wxyz_rotate(3);
        const double a0 = rotate_ref(0, 0);
        const double a1 = rotate_ref(0, 1);
        const double a2 = rotate_ref(0, 2);
        const double b0 = rotate_ref(1, 0);
        const double b1 = rotate_ref(1, 1);
        const double b2 = rotate_ref(1, 2);
        const double c0 = rotate_ref(2, 0);
        const double c1 = rotate_ref(2, 1);
        const double c2 = rotate_ref(2, 2);
        dfdw = 4 * (b2 * x - a2 * y + a1 * z - c1 * x - b0 * z + c0 * y);
        dfdx = 4 * (b2 * w - a1 * y + 2 * b1 * x - c1 * w - b0 * y - a2 * z + 2 * c2 * x - c0 * z);
        dfdy = 4 * (2 * a0 * y - a1 * x - a2 * w - b0 * x + c0 * w - b2 * z + 2 * c2 * y - c1 * z);
        dfdz = 4 * (a1 * w - b0 * w - a2 * x + 2 * a0 * z - c0 * x - b2 * y + 2 * b1 * z - c1 * y);
    }

    static inline double WC2(const double x, double &dx)
    {
        if (x < -1)
        {
            dx = 0;
            return 0;
        }
        else if (x < -0.5)
        {
            dx = 4 * (x + 1);
            return 2 * (x + 1) * (x + 1);
        }
        else if (x < 0.5)
        {
            dx = -4 * x;
            return 1 - 2 * x * x;
        }
        else if (x < 1)
        {
            dx = 4 * (x - 1);
            return 2 * (x - 1) * (x - 1);
        }
        else
        {
            dx = 0;
            return 0;
        }
    }


    static inline void addTimeIntPenalty(void *ptr,
                                         const Eigen::VectorXd &T,
                                         const Eigen::MatrixX3d &coeffs,
                                         double &cost,
                                         Eigen::VectorXd &gradT,
                                         Eigen::MatrixX3d &gradC)
    {
        OriTraj &obj = *(OriTraj *)ptr;
        const double velSqrMax = obj.vmax * obj.vmax;     // Vmax^2
        const double omgSqrMax = obj.omgmax * obj.omgmax; // Wmax^2

        const double weightVel = obj.weight_v;
        const double weightOmg = obj.weight_omg;

        double totalGradPsi, totalGradPsiD;
        double thr;
        double cost_altitude;
        double gradThr;
        Eigen::Vector3d pos, vel, acc, jer, sna;
        Eigen::Vector3d totalGradVel, totalGradAcc;
        Eigen::Vector4d quat;
        Eigen::Vector3d omg;
        Eigen::Vector4d gradQuat, violaQuatPenaGrad;
        Eigen::Vector3d gradVel, gradAcc, gradPos, gradOmg, violaPosPenaGrad;
        Eigen::Vector3d gradVelTotal, gradAccTotal, gradPosTotal, gradJerTotal;
        Eigen::Matrix3d rotate, St;

        // double gradPsiT, gradDPsiT;

        double step, alpha;
        double s1, s2, s3, s4, s5;
        Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3, beta4;
        Eigen::Vector3d outerNormal;
        double violaVel, violaAcc, violaOmg, violaTheta, violaThrust;
        double violaVelPenaD, violaOmgPenaD, violaThetaPenaD;
        double violaVelPena, violaOmgPena, violaPosPena, violaThetaPena;
        double node, pena;
        double last_ttime = 0.0;

        const int pieceNum = T.size();
        const int integralResolution = obj.integralRes;
        const double smoothFactor = obj.smooth_fac;
        const double integralFrac = 1.0 / integralResolution;
        Eigen::Vector3d grad_tmp_p, grad_tmp_a;
        double cost_tmp, cost_tmp_p, cost_tmp_a;
        double dfdw, dfdx, dfdy, dfdz; 
        double midT;                   
        double norm_T;                 
        double kRT;                    
        double dkRT;
        double total_att_cost = 0.0;

        for (int i = 0; i < pieceNum; i++)
        {
            const Eigen::Matrix<double, 6, 3> &c = coeffs.block<6, 3>(i * 6, 0);
            step = T(i) * integralFrac;
            midT = T(i) * 0.5;

            Eigen::Matrix3d rotate_ref_L = Eigen::Matrix3d::Identity();
            Eigen::Matrix3d rotate_ref_R = Eigen::Matrix3d::Identity();
            Eigen::Matrix3d rotate_ref = Eigen::Matrix3d::Identity();
            // I a a a I
            if (i > 0)
            {
                rotate_ref_L = obj.att_constraints[i - 1];
            }
            if (i < pieceNum - 1)
            {
                rotate_ref_R = obj.att_constraints[i];
            }

            for (int j = 0; j <= integralResolution; j++)
            {
                s1 = j * step;
                s2 = s1 * s1;
                s3 = s2 * s1;
                s4 = s2 * s2;
                s5 = s4 * s1;
                beta0(0) = 1.0, beta0(1) = s1, beta0(2) = s2, beta0(3) = s3, beta0(4) = s4, beta0(5) = s5;
                beta1(0) = 0.0, beta1(1) = 1.0, beta1(2) = 2.0 * s1, beta1(3) = 3.0 * s2, beta1(4) = 4.0 * s3, beta1(5) = 5.0 * s4;
                beta2(0) = 0.0, beta2(1) = 0.0, beta2(2) = 2.0, beta2(3) = 6.0 * s1, beta2(4) = 12.0 * s2, beta2(5) = 20.0 * s3;
                beta3(0) = 0.0, beta3(1) = 0.0, beta3(2) = 0.0, beta3(3) = 6.0, beta3(4) = 24.0 * s1, beta3(5) = 60.0 * s2;
                beta4(0) = 0.0, beta4(1) = 0.0, beta4(2) = 0.0, beta4(3) = 0.0, beta4(4) = 24.0, beta4(5) = 120.0 * s1;
                pos = c.transpose() * beta0;
                vel = c.transpose() * beta1;
                acc = c.transpose() * beta2;
                jer = c.transpose() * beta3;
                sna = c.transpose() * beta4;

                obj.flatmap.forward(vel, acc, jer, 0.0, 0.0, thr, quat, omg);
                rotate = Eigen::Quaterniond(quat[0], quat[1], quat[2], quat[3]).toRotationMatrix();

                if (s1 <= midT)
                {
                    rotate_ref = rotate_ref_L;
                    norm_T = s1 / midT;      
                    kRT = WC2(norm_T, dkRT); 
                }

                else
                {
                    rotate_ref = rotate_ref_R;
                    norm_T = (s1 - midT) / midT - 1.0; 
                    kRT = WC2(norm_T, dkRT);
                }

                pena = 0.0;
                gradVel.setZero();
                gradAcc.setZero();
                gradPos.setZero();
                gradOmg.setZero();
                gradQuat.setZero();
                gradPosTotal.setZero();
                gradVelTotal.setZero();
                gradAccTotal.setZero();
                gradJerTotal.setZero();

                cost_altitude = costaltitude(quat, rotate_ref);
                gradaltitude(quat, rotate_ref, dfdw, dfdx, dfdy, dfdz);

             
                violaVel = vel.squaredNorm() - velSqrMax;
                violaOmg = omg.squaredNorm() - omgSqrMax;

                
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

                if (smoothedL1(cost_altitude, smoothFactor, violaThetaPena, violaThetaPenaD))
                {
                    gradQuat += kRT * obj.weightAR * violaThetaPenaD * Eigen::Vector4d(dfdw, dfdx, dfdy, dfdz);
                    pena += kRT * obj.weightAR * violaThetaPena;
                    total_att_cost += pena * node * step;
                }

                obj.flatmap.backwardthreadsafe(vel, acc, jer, gradPos, gradVel, gradQuat, gradOmg, gradPosTotal, gradVelTotal, gradAccTotal, gradJerTotal);

                node = (j == 0 || j == integralResolution) ? 0.5 : 1.0;
                alpha = j * integralFrac;

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
        // std::cout<<"total_att_cost: "<<total_att_cost<<std::endl;
        return;
    }

 
    static inline void smoothMinabs(const double &x,
                                    const double &c,
                                    const double &mu,
                                    double &f,
                                    double &df)
    {
        double f1 = 0, f2 = 0, f3 = 0, f4 = 0;
        double df1 = 0, df2 = 0, df3 = 0, df4 = 0;

        smoothedL1(x, mu, f1, df1);
        smoothedL1(x - c, mu, f2, df2);
        smoothedL1(-x, mu, f3, df3);
        smoothedL1(-x - c, mu, f4, df4);

        f = f1 - f2 + f3 - f4;
        df = df1 - df2 - df3 + df4;
    }

    static inline int earlyExit(void *instance,
                                const Eigen::VectorXd &x,
                                const Eigen::VectorXd &g,
                                const double fx,
                                const double step,
                                const int k,
                                const int ls)
    {
        OriTraj &obj = *(OriTraj *)instance;
        obj.iter++;
        std::cout << "[mid] iter = " << obj.iter << std::endl;
        if (obj.conf.enableearlyExit)
        {
            obj.drawDebugTraj(); 
       
            std::this_thread::sleep_for(std::chrono::milliseconds(obj.conf.debugpause));
        }
        return k > 8e3;
    }

public:
    typedef shared_ptr<OriTraj> Ptr;
};
