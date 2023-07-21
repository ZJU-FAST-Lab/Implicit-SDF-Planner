#ifndef SW_CALCULATE_HPP
#define SW_CALCULATE_HPP
#define IGL_STATIC_LIBRARY 1 
#include <unordered_map>
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/Eigen>
#include <igl/readOBJ.h>
#include <igl/writeOBJ.h>
#include <igl/signed_distance.h>
#include <igl/sparse_voxel_grid.h>
#include <igl/marching_cubes.h>
#include <igl/fast_winding_number.h>
#include <igl/winding_number.h>
#include <igl/writeDMAT.h>
#include <igl/read_triangle_mesh.h>
#include <igl/random_points_on_mesh.h>
#include <igl/AABB.h>
#include <igl/doublearea.h>
#include <igl/per_face_normals.h>
#include <igl/readDMAT.h>
#include <igl/decimate.h> 

class sw_calculate
{
private:
    Eigen::MatrixXd U; 
    Eigen::MatrixXi G; 
    double t_min{0.0};
    double t_max{1.0};
    int grad_descent_queries{0};
    double eps{0.02};                            
  
    std::vector<Eigen::RowVector3d> CV_vector;                  
    std::vector<Eigen::Matrix<int, 1, 8>> CI_vector;            
    std::vector<std::vector<std::vector<double>>> CV_intervals; 
    std::vector<std::vector<std::vector<double>>> CV_values;
    std::vector<std::vector<std::vector<double>>> CV_minima;
    std::vector<double> CS_vector; 
    std::vector<std::vector<double>> argmins;
    std::vector<Eigen::RowVector3i> queue; 
    std::vector<double> time_queue;        
    std::vector<int> correspondence_queue; 
    Eigen::VectorXd CV_argmins_vector;     
    Eigen::MatrixXi CI;
    Eigen::MatrixXd CV;
    Eigen::VectorXd CS;
    double momentum{0};            // With Nesterov Momentum Acceleration
    std::vector<double> intervals; 
    std::vector<double> values;
    std::vector<double> minima;
    inline static auto sgn = [](double val) -> double
    {
        return (double)((double(0) < val) - (val < double(0)));
    };
    const Eigen::RowVector3i bx{1, 0, 0}, by{0, 1, 0}, bz{0, 0, -1};
    const std::array<Eigen::RowVector3i, 30> neighbors = {
        bx, -bx, by, -by, bz, -bz,
        by - bz, -by + bz,
        bx + by, -bx - by,
        by + bz, -by - bz,
        by - bx, -by + bx,
        bx - bz, -bx + bz,
        bx + bz, -bx - bz,
        -bx + by + bz, bx - by - bz,
        bx + by + bz, -bx - by - bz,
        bx + by - bz, -bx - by + bz,
        -bx + by - bz, bx - by + bz,
        bx - bx, bx - bx,
        bx - bx, bx - bx};
    static constexpr std::array<uint8_t, 30> zv {
        (1 << 0) | (1 << 1) | (1 << 4) | (1 << 5),
        (1 << 2) | (1 << 3) | (1 << 6) | (1 << 7),
        (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3),
        (1 << 4) | (1 << 5) | (1 << 6) | (1 << 7),
        (1 << 0) | (1 << 3) | (1 << 4) | (1 << 7),
        (1 << 1) | (1 << 2) | (1 << 5) | (1 << 6),
        (1 << 1) | (1 << 2),
        (1 << 4) | (1 << 7),
        (1 << 0) | (1 << 1),
        (1 << 6) | (1 << 7),
        (1 << 0) | (1 << 3),
        (1 << 5) | (1 << 6),
        (1 << 2) | (1 << 3),
        (1 << 4) | (1 << 5),
        (1 << 1) | (1 << 5),
        (1 << 3) | (1 << 7),
        (1 << 0) | (1 << 4),
        (1 << 2) | (1 << 6),
        (1 << 3),
        (1 << 5),
        (1 << 0),
        (1 << 6),
        (1 << 1),
        (1 << 7),
        (1 << 2),
        (1 << 4),
        (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3),
        (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3),
        (1 << 4) | (1 << 5) | (1 << 6) | (1 << 7),
        (1 << 4) | (1 << 5) | (1 << 6) | (1 << 7),
    };
    std::array<double, 8> cubeScalars;
    static constexpr std::array<std::array<int, 4>, 30> zvv{{{{0, 1, 4, 5}}, {{3, 2, 7, 6}}, {{0, 1, 2, 3}}, {{4, 5, 6, 7}}, {{0, 3, 4, 7}}, {{1, 2, 5, 6}}, {{-1, -1, 1, 2}}, {{-1, -1, 4, 7}}, {{-1, -1, 0, 1}}, {{-1, -1, 7, 6}}, {{-1, -1, 0, 3}}, {{-1, -1, 5, 6}}, {{-1, -1, 2, 3}}, {{-1, -1, 5, 4}}, {{-1, -1, 1, 5}}, {{-1, -1, 3, 7}}, {{-1, -1, 0, 4}}, {{-1, -1, 2, 6}}, {{-1, -1, -1, 3}}, {{-1, -1, -1, 5}}, {{-1, -1, -1, 0}}, {{-1, -1, -1, 6}}, {{-1, -1, -1, 1}}, {{-1, -1, -1, 7}}, {{-1, -1, -1, 2}}, {{-1, -1, -1, 4}}, {{0, 1, 2, 3}}, {{0, 1, 2, 3}}, {{4, 5, 6, 7}}, {{4, 5, 6, 7}}}};

public:
    typedef std::shared_ptr<sw_calculate> Ptr;
    inline void getmesh(Eigen::MatrixXd &U_, Eigen::MatrixXi &G_)
    {
        CV.conservativeResize(CV_vector.size(), 3);
        CS.conservativeResize(CS_vector.size(), 1);
        CI.conservativeResize(CI_vector.size(), 8);
        CV_argmins_vector.conservativeResize(CV_vector.size(), 1);
        for (int i = 0; i < CV_vector.size(); i++)
        {
            CV.row(i) = CV_vector[i];
        }
        for (int i = 0; i < CS_vector.size(); i++)
        {
            CS(i) = CS_vector[i];
        }
        for (int i = 0; i < CI_vector.size(); i++)
        {
            CI.row(i) = CI_vector[i];
        }
        igl::marching_cubes(CS, CV, CI, 0.0, U, G);
        U_ = U;
        G_ = G;
    }
    inline void sweptvolumeclear()
    {
        CV_vector.clear();
        CI_vector.clear();
        CV_intervals.clear();
        CV_values.clear();
        CV_minima.clear();
        CS_vector.clear();
        argmins.clear();
        queue.clear();
        time_queue.clear();
        correspondence_queue.clear();
    }
    inline void updatebounds(double tmin, double tmax)
    {
        t_min = tmin;
        t_max = tmax;
    }
    sw_calculate(/* args */){};
    ~sw_calculate(){};
    void calculation(const Eigen::RowVector3d &p0, const std::vector<Eigen::RowVector3i> &init_voxels, // init种子点对应小体素正方体的索引Index，相对于p0
                            const std::vector<double> &t0, const std::function<double(const Eigen::RowVector3d &, double &, std::vector<std::vector<double>> &, std::vector<std::vector<double>> &, std::vector<std::vector<double>> &)> scalarFunc, const double eps,
                            const int expected_number_of_cubes);
};


#endif