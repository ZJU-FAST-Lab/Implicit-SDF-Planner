#ifndef SE3_STATE_HPP
#define SE3_STATE_HPP

#include <Eigen/Eigen>

class SE3State
{
public:
    Eigen::Vector3d position;
    Eigen::Matrix3d rot;
    Eigen::Vector4d quat;
    SE3State(){ position.setZero(); quat.setZero(); quat(0) = 1;}
    inline Eigen::Matrix3d getRotMatrix()const{ 
        if( rot != Eigen::Matrix3d::Zero() ){ return rot;}
        else {
            return Eigen::Quaterniond(quat[0], quat[1], quat[2], quat[3]).toRotationMatrix(); 
        }
    }
};
#endif