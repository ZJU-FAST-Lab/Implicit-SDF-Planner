#include <ros/ros.h>
#include <ros/package.h>
#include <utils/Shape.hpp>
#include <utils/config.hpp>
#include <utils/Visualization.hpp>


#include <igl/read_triangle_mesh.h> 
#include <igl/readOBJ.h>
#include <igl/writeOBJ.h>
#include <igl/signed_distance.h>
#include <igl/sparse_voxel_grid.h>
#include <igl/marching_cubes.h>
#include <igl/fast_winding_number.h>
#include <igl/winding_number.h>
#include <igl/writeDMAT.h>
#include <igl/random_points_on_mesh.h>
#include <igl/AABB.h>
#include <igl/doublearea.h>
#include <igl/per_face_normals.h>
#include <igl/readDMAT.h>

#define PI 3.14159265358979323846
namespace shape
{
    using namespace vis;
    Generalshape::Generalshape(const Config &config,bool analytic_) : BasicShape(config),analytic(analytic_)
    {
        if (config.use_objfile_as_body)
        {
            ROS_WARN_STREAM("use obj file as body");
            std::vector<double> para;
            para = config.poly_params;
            assert(para.size() == 6 && " set the para size==6");
            string input = ros::package::getPath(string("plan_manager")) + "/" + config.inputdata;
            igl::read_triangle_mesh(input, V, F); 
            ROS_WARN_STREAM("use input obj file as Polyhedron");
            trans=Eigen::Vector3d(para[0], para[1], para[2]);
            roll = AngleAxisd(para[3] * PI / 180.0, Vector3d::UnitX());
            pitch = AngleAxisd(para[4] * PI / 180.0, Vector3d::UnitY());
            yaw = AngleAxisd(para[5] * PI / 180.0, Vector3d::UnitZ());
            euler = Eigen::Vector3d(para[5], para[4], para[3]);
            Rotate = yaw.toRotationMatrix() * pitch.toRotationMatrix() * roll.toRotationMatrix();
            Eigen::Matrix4d Trans{Eigen::Matrix4d::Identity()};
            Trans.block(0, 0, 3, 3) = Rotate;
            Eigen::Matrix4d Transorig{Eigen::Matrix4d::Identity()};

            Trans.block(0, 3, 3, 1) = trans;
            const Eigen::Matrix4d TT = (Trans * Transorig.inverse()).cast<double>().transpose();
            V = (V.rowwise().homogeneous() * TT).rowwise().hnormalized();

            vertices = V.transpose();
            mesh.conservativeResize(3, 3 * F.rows());
            int faces = F.rows();
            for (int i = 0; i < faces; i++)
            {
                mesh.col(3 * i) = V.row(F.row(i)[0]);
                mesh.col(3 * i + 1) = V.row(F.row(i)[1]);
                mesh.col(3 * i + 2) = V.row(F.row(i)[2]);
            }
            ids = faces;
            hPoly.resize(ids, 4);
            hPoly_var.resize(ids, 4);
            Eigen::Vector3d normal, point, edge0, edge1;
            for (int i = 0; i < ids; i++)
            {
                point = mesh.col(3 * i + 1);
                edge0 = point - mesh.col(3 * i);
                edge1 = mesh.col(3 * i + 2) - point;
                normal = edge0.cross(edge1).normalized();
                hPoly(i, 0) = normal(0);
                hPoly(i, 1) = normal(1);
                hPoly(i, 2) = normal(2);
                hPoly(i, 3) = -normal.dot(point);
    
            }
            
        }
        else //============================use yaml file read vertices ===========================
        {
            assert(false && "目前暂时不支持,需要实现F,以便fastwinding number计算SDF"); 
            //TODO
        }

        tree.init(V, F);
        igl::fast_winding_number(V, F, 2, fwn_bvh);

        double bdx=config.kernel_size*config.occupancy_resolution;
        double bdy=config.kernel_size*config.occupancy_resolution;
        double bdz=config.kernel_size*config.occupancy_resolution;
       
        if(!analytic_)
        {
        std::cout<<"use Generalshape,calculate with libigl and store self sdf map"<<std::endl;
        initShape<true,true>(0.8*bdx,0.8* bdy,0.8* bdz);
        }
        else
        {
        std::cout<<"use some analytic shape,calculate analytically"<<std::endl;
        initShape<true,false>(0.8*bdx, 0.8*bdy,0.8* bdz);
        }
        init = true;
    }

    inline double Generalshape::getonlySDF(const Eigen::RowVector3d &pos_rel)
    {
        int i;
        Eigen::RowVector3d c;
        Eigen::VectorXd w{Eigen::VectorXd::Zero(1)};
        igl::fast_winding_number(fwn_bvh, 2.0, pos_rel, w);
        double s = 1. - 2. * w(0);
        return s * sqrt(tree.squared_distance(V, F, pos_rel, i, c));
    }

    inline double Generalshape::getonlySDF(const Eigen::RowVector3d &pos, const Eigen::Matrix3d &R_obj)
    {
        int i;
        Eigen::RowVector3d c;
        Eigen::VectorXd w{Eigen::VectorXd::Zero(1)};
        Eigen::RowVector3d pos_rel = pos * R_obj;
        igl::fast_winding_number(fwn_bvh, 2.0, pos_rel, w);
        double s = 1. - 2. * w(0);
        return s * sqrt(tree.squared_distance(V, F, pos_rel, i, c));
    }
    inline Eigen::Vector3d Generalshape::getonlyGrad1(const Eigen::RowVector3d &pos_rel)
    {

        int i;
        Eigen::RowVector3d c;
        Eigen::VectorXd w{Eigen::VectorXd::Zero(1)};
        igl::fast_winding_number(fwn_bvh, 2.0, pos_rel, w);
        double s = 1. - 2. * w(0);
        tree.squared_distance(V, F, pos_rel, i, c); 
        Eigen::Vector3d grad_rel = -(c - pos_rel).transpose();
        grad_rel *= s; 
        grad_rel.normalize();
        return grad_rel;
    }
    inline double Generalshape::getSDFwithGrad1(const Eigen::RowVector3d &pos_rel, Eigen::Vector3d &grad)
    {
        int i;
        Eigen::RowVector3d c;
        Eigen::VectorXd w{Eigen::VectorXd::Zero(1)};
        igl::fast_winding_number(fwn_bvh, 2.0, pos_rel, w);
        double s = 1. - 2. * w(0);
        double distance = tree.squared_distance(V, F, pos_rel, i, c); 
        grad = -(c - pos_rel).transpose();
        grad *= s; 
        grad.normalize();
        return s * sqrt(distance);
    }
    
}