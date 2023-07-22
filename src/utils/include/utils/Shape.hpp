#ifndef SHAPE_HPP
#define SHAPE_HPP
#define IGL_STATIC_LIBRARY 1
#include <iostream>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <vector>
#include <utils/config.hpp>
#include <utils/Visualization.hpp>

#include <unordered_set>
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
using namespace vis;
using namespace std;
#define DEFINE_USEFUL_FUNCTION()                                                            \
    inline Eigen::Vector3d getonlyGrad1(const Eigen::RowVector3d &pos_rel)                  \
    {                                                                                       \
        double dx = 0.000005;                                                               \
        Eigen::RowVector3d temp = pos_rel;                                                  \
        temp(0) -= dx;                                                                      \
        double sdfold = getonlySDF(temp);                                                   \
        temp(0) += 2 * dx;                                                                  \
        double gradx = getonlySDF(temp) - sdfold;                                           \
                                                                                            \
        temp = pos_rel;                                                                     \
                                                                                            \
        temp(1) -= dx;                                                                      \
        sdfold = getonlySDF(temp);                                                          \
        temp(1) += 2 * dx;                                                                  \
        double grady = getonlySDF(temp) - sdfold;                                           \
        temp = pos_rel;                                                                     \
                                                                                            \
        temp(2) -= dx;                                                                      \
        sdfold = getonlySDF(temp);                                                          \
        temp(2) += 2 * dx;                                                                  \
        double gradz = getonlySDF(temp) - sdfold;                                           \
                                                                                            \
        Eigen::Vector3d grad = Eigen::Vector3d(gradx, grady, gradz) / (2 * dx);             \
        return grad.normalized();                                                           \
    }                                                                                       \
    inline Eigen::Vector3d helperfunc(const Eigen::RowVector3d &pos_rel, double &sdf)       \
    {                                                                                       \
        double dx = 0.000005;                                                               \
        Eigen::RowVector3d temp = pos_rel;                                                  \
        sdf = getonlySDF(pos_rel);                                                          \
        temp(0) -= dx;                                                                      \
        double sdfold = getonlySDF(temp);                                                   \
        temp(0) += 2 * dx;                                                                  \
        double gradx = getonlySDF(temp) - sdfold;                                           \
                                                                                            \
        temp = pos_rel;                                                                     \
                                                                                            \
        temp(1) -= dx;                                                                      \
        sdfold = getonlySDF(temp);                                                          \
        temp(1) += 2 * dx;                                                                  \
        double grady = getonlySDF(temp) - sdfold;                                           \
        temp = pos_rel;                                                                     \
                                                                                            \
        temp(2) -= dx;                                                                      \
        sdfold = getonlySDF(temp);                                                          \
        temp(2) += 2 * dx;                                                                  \
        double gradz = getonlySDF(temp) - sdfold;                                           \
        Eigen::Vector3d grad = Eigen::Vector3d(gradx, grady, gradz) / (2 * dx);             \
        return grad.normalized();                                                           \
    }                                                                                       \
    inline double getSDFwithGrad1(const Eigen::RowVector3d &pos_rel, Eigen::Vector3d &grad) \
    {                                                                                       \
        double sdf;                                                                         \
        grad = helperfunc(pos_rel, sdf);                                                    \
        return sdf;                                                                         \
    }

namespace shape
{
    const uint8_t or_mask[8] = {0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01};
    typedef std::function<double(const Eigen::Vector3d &)> SDFFunction;
    class BasicShape
    {

    private:
        Config config;

        /**
         * 单个碰撞检查的kernel
         */
        typedef class shapeKernel
        {
        public:
            bool *map;
            double rotx;
            double roty;
            int totalsize;
            int X_size;
            int Y_size;
            int Z_size;
            shapeKernel()
            {
                map = nullptr;
            }
            shapeKernel(const shapeKernel &other)
            {
                map = new bool[other.totalsize];
                memcpy(map, other.map, other.totalsize * sizeof(bool));

                rotx = other.rotx;
                roty = other.roty;
                totalsize = other.totalsize;
                X_size = other.X_size;
                Y_size = other.Y_size;
                Z_size = other.Z_size;
            }
            ~shapeKernel()
            {
                delete[] map;
            }
            /**
             * 初始化kernel地图碰撞检查用
             * @param xsize_ x index 长
             * @param ysize_ y index 长
             * @param zsize_ z index 长
             */
            inline void init(int xsize_, int ysize_, int zsize_)
            {

                X_size = xsize_;
                Y_size = ysize_;
                Z_size = zsize_;
                totalsize = X_size * Y_size * Z_size;
                map = new bool[totalsize]();
            }
            /**
             * 地址转换，转化为一维
             * @param idx x index
             * @param idy y index
             * @param idz z index
             */
            inline int toAddr(const int idx, const int idy, const int idz) const
            {
                return idx * Y_size * Z_size + idy * Z_size + idz;
            }

            /**
             * 查询kernel中具体位置是否被占用
             * @param idx x index
             * @param idy y index
             * @param idz z index
             * @return true or false
             */
            inline bool getOccupied(const int idx, const int idy, const int idz) const
            {
                return map[toAddr(idx, idy, idz)];
            }
        } Shapekernel;

        typedef class byteShapeKernel
        {
        public:
            uint8_t *map;
            double rotx;
            double roty;
            int totalsize;
            int X_size;
            int Y_size;
            int Z_size;
            byteShapeKernel()
            {
                map = nullptr;
            }
            byteShapeKernel(const byteShapeKernel &other)
            {
                map = new uint8_t[other.totalsize];
                memcpy(map, other.map, other.totalsize * sizeof(uint8_t));

                rotx = other.rotx;
                roty = other.roty;
                totalsize = other.totalsize;
                X_size = other.X_size;
                Y_size = other.Y_size;
                Z_size = other.Z_size;
            }
            ~byteShapeKernel()
            {
                delete[] map;
            }

            /**
             * 查询kernel中具体位置是否被占用
             * @param idx x index
             * @param idy y index
             * @param idz z index
             * @return true or false
             */
            inline bool getOccupied(const int idx, const int idy, const int idz) const
            {
                int bytes_of_last_dim = (Z_size + 7) / 8;
                int byteIdx = (idx * Y_size * bytes_of_last_dim + idy * bytes_of_last_dim) + (idz) / 8;
                int offset = idz % 8;

                if ((map[byteIdx] & or_mask[offset]))
                {
                    return true;
                }
                else
                {
                    return false;
                }
                return false;
            }

            /**
             * 使用原始kernel初始化byte_kernel地图碰撞检查用
             * @param kernel_size kernel边长
             * @param ori_kernel 原始kernel
             */
            inline void generateByteKernel(int kernel_size, const Shapekernel &ori_kernel)
            {
                int kernel_size_yz = kernel_size * kernel_size;
                int bytes_len_of_last_dim = (kernel_size + 7) / 8;
                int bytes_len = kernel_size * kernel_size * bytes_len_of_last_dim;
                X_size = Y_size = Z_size = kernel_size;
                map = new uint8_t[bytes_len];
                for (int x = 0; x < kernel_size; x++)
                { 
                    for (int y = 0; y < kernel_size; y++)
                    { 
                        for (int z = 0; z < kernel_size; z++)
                        { 
                            int byte_idx = (x * kernel_size * bytes_len_of_last_dim + y * bytes_len_of_last_dim) + (z + 0) / 8;
                            int byte_offset = z % 8;
                            if (ori_kernel.getOccupied(x, y, z) == true)
                            {
                                map[byte_idx] |= or_mask[byte_offset];
                            }
                        }
                    }
                }
            }

        } ByteShapeKernel;

        // ================for collision kernels==================
        int xkernel_size{-1};
        int ykernel_size{-1};
        int kernelsize{-1};    //必须是奇数

        double kernelresu{-1}; //恒定为map的reso
        double kernel_max_roll;
        double kernel_max_pitch;
        double kernel_ang_res;

        typedef struct numSDFGridCell
        {
            Eigen::Vector3d gradient{Eigen::Vector3d::Zero()};
            double distance{0.0};
            numSDFGridCell(){}
            numSDFGridCell(const Eigen::Vector3d&grad,double dis):gradient(grad),distance(dis){}
        } NumSDFGridCell;

        // for numeric gradients
        NumSDFGridCell *num_sdf_map;
        int num_sdf_map_X_size{-1};
        int num_sdf_map_Y_size;
        int num_sdf_map_Z_size;
        double num_sdf_map_res;
        double num_sdf_map_xlength;
        double num_sdf_map_ylength;
        double num_sdf_map_zlength;
        double x_min;
        double y_min;
        double z_min;
        bool initsdfdone{false};
        bool initselfkerneldone{false};

    public:
        typedef std::shared_ptr<BasicShape> Ptr;
        BasicShape() = delete;
        Shapekernel *shape_kernels{nullptr};
        ByteShapeKernel *byte_shape_kernels{nullptr};
        BasicShape(const Config &conf) : config(conf)
        {
            xkernel_size = floor(2 * conf.kernel_max_roll / conf.kernel_ang_res) + 1;
            ykernel_size = floor(2 * conf.kernel_max_pitch / conf.kernel_ang_res) + 1;

            kernelsize = config.kernel_size;

            kernelresu = config.occupancy_resolution;
            kernel_max_roll = config.kernel_max_roll;
            kernel_max_pitch = config.kernel_max_pitch;
            kernel_ang_res = config.kernel_ang_res;
            num_sdf_map_res = 0.1;
            num_sdf_map_xlength = num_sdf_map_ylength = num_sdf_map_zlength = 1.0;
            num_sdf_map_X_size = num_sdf_map_Y_size = num_sdf_map_Z_size = -1;
        }

        ~BasicShape()
        {
            delete[] num_sdf_map;
            delete[] shape_kernels;
            delete[] byte_shape_kernels;
        }

        /**
         * 返回自身SDF的bound
         * @attention 优化前检查自身SDF地图是否超出边界
         */
        vector<double> getbound()
        {
            vector<double> bounds;
            bounds.clear();
            if (!initsdfdone) // 没有初始化，analytic map
            {
                bounds.emplace_back(-2e1);
                bounds.emplace_back(-2e1);
                bounds.emplace_back(-2e1);
                bounds.emplace_back(2e1);
                bounds.emplace_back(2e1);
                bounds.emplace_back(2e1);
                return bounds;
            }
            else
            {
                double xmax = x_min + (num_sdf_map_X_size - 1) * num_sdf_map_res;
                double ymax = y_min + (num_sdf_map_Y_size - 1) * num_sdf_map_res;
                double zmax = z_min + (num_sdf_map_Z_size - 1) * num_sdf_map_res;
                bounds.emplace_back(x_min);
                bounds.emplace_back(y_min);
                bounds.emplace_back(z_min);
                bounds.emplace_back(xmax);
                bounds.emplace_back(ymax);
                bounds.emplace_back(zmax);
            }
            return bounds;
        }

        /**
         * 在原点处初始化自身SDF map，保证一定要是在原点
         * @param ndx x boundingbox 长
         * @param ndy y boundingbox 长
         * @param ndz z boundingbox 长
         * @param nres 分辨率
         * @param enablekernel 模板参数是否初始化自身kernelmap
         * @param enableselfmap 模板参数是否初始化自身SDF地图
         */
        template <bool enablekernel = false, bool enableselfmap = true>
        void initShape(const double ndx, const double ndy, const double ndz, const double nres = 0.1)
        {
            std::cout << "==============enter initShape==============" << std::endl;
            std::cout << "ndx: " << ndx << "ndy: " << ndy << "ndz: " << ndz << "nres: " << nres << std::endl;
            num_sdf_map_res = nres;
            num_sdf_map_xlength = ndx;
            num_sdf_map_ylength = ndy;
            num_sdf_map_zlength = ndz;
            num_sdf_map_X_size = ceil(ndx / nres);
            num_sdf_map_Y_size = ceil(ndy / nres);
            num_sdf_map_Z_size = ceil(ndz / nres);
            x_min = -ndx / 2;
            y_min = -ndy / 2;
            z_min = -ndz / 2;
            if (enableselfmap)
            {
                std::cout << "==============calculate sdf map==============" << std::endl;
                num_sdf_map = new NumSDFGridCell[num_sdf_map_X_size * num_sdf_map_Y_size * num_sdf_map_Z_size];
                std::cout << "num_sdf_map_X_size: " << num_sdf_map_X_size << "num_sdf_map_Y_size: "
                          << num_sdf_map_Y_size << "num_sdf_map_Z_size: " << num_sdf_map_Z_size << std::endl;
                double dis;
                Eigen::Vector3d grad;
                int index = 0;
                int total = (num_sdf_map_X_size) * (num_sdf_map_Y_size) * (num_sdf_map_Z_size);
                double per;
                std::cout << "==============before loop==============" << std::endl;
                for (int i = 0; i < num_sdf_map_X_size; ++i)
                {
                    for (int j = 0; j < num_sdf_map_Y_size; ++j)
                    {
                        for (int k = 0; k < num_sdf_map_Z_size; ++k)
                        {
                            Eigen::Vector3d p_rel(x_min + i * nres, y_min + j * nres, z_min + k * nres);
                            dis = getSDFwithGrad1(p_rel, grad);
                            num_sdf_map[toAddr(i, j, k)] = NumSDFGridCell{grad, dis};
                            index++;
                        }
                    }
                    per = 100.0 * index / total;
                    std::cout << "percentage:" << per << std::endl;
                }
                initsdfdone = true;
            }

            if (enablekernel)
            {
                std::cout << "==============calculate self kernels==============" << std::endl;
                int zero_pose_i = (xkernel_size - 1) / 2;
                int zero_pose_j = (ykernel_size - 1) / 2;

                int size_side = 0.5 * (kernelsize - 1);
                double rotx_, roty_, x, y, z, sdf;
                Eigen::Matrix3d rot_mat;
                Eigen::RowVector3d pos_ori;
                shape_kernels = new Shapekernel[xkernel_size * ykernel_size];
                byte_shape_kernels = new ByteShapeKernel[xkernel_size * ykernel_size];
                int i, j;
                i = 0;
                j = 0;
                double safemargin = max(config.front_end_safeh, config.occupancy_resolution / 2);
                for (double roll = -kernel_max_roll; roll <= kernel_max_roll; roll += kernel_ang_res, i++)
                {

                    j = 0;
                    for (double pitch = -kernel_max_pitch; pitch <= kernel_max_pitch; pitch += kernel_ang_res, j++)
                    {
                       
                        shape_kernels[i * ykernel_size + j].init(kernelsize, kernelsize, kernelsize);
                        rotx_ = roll * M_PI / 180.0;
                        roty_ = pitch * M_PI / 180.0;
                        rot_mat = Eigen::AngleAxisd(rotx_, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(roty_, Eigen::Vector3d::UnitY());
                        shape_kernels[i * ykernel_size + j].rotx = rotx_;
                        shape_kernels[i * ykernel_size + j].roty = roty_;
                        byte_shape_kernels[i * ykernel_size + j].rotx = rotx_;
                        byte_shape_kernels[i * ykernel_size + j].roty = roty_;

                        for (int a = 0; a < kernelsize; a++)
                        {
                            for (int b = 0; b < kernelsize; b++)
                            {
                                for (int c = 0; c < kernelsize; c++)
                                {
                                    x = kernelresu * a - size_side * kernelresu;
                                    y = kernelresu * b - size_side * kernelresu;
                                    z = kernelresu * c - size_side * kernelresu;
                                    pos_ori(0) = x;
                                    pos_ori(1) = y;
                                    pos_ori(2) = z;
                                    sdf = getonlySDF(pos_ori, rot_mat);
                                    if (sdf <= safemargin)
                                    {
                                        shape_kernels[i * ykernel_size + j].map[a * kernelsize * kernelsize + b * kernelsize + c] = true;
                                    }
                                }
                            }
                        }
                        byte_shape_kernels[i * ykernel_size + j].generateByteKernel(kernelsize, shape_kernels[i * ykernel_size + j]); // fix bug
                    }
                }
                initselfkerneldone = true;
            }
        }


        inline int toAddr(int idx, int idy, int idz) { return idx * num_sdf_map_Y_size * num_sdf_map_Z_size + idy * num_sdf_map_Z_size + idz; }

    public:
        virtual inline double getonlySDF(const Eigen::RowVector3d &pos_rel) = 0;
        virtual inline double getonlySDF(const Eigen::RowVector3d &pos_rel, const Eigen::Matrix3d &R_obj) = 0;
        virtual inline Eigen::Vector3d getonlyGrad1(const Eigen::RowVector3d &pos_rel) = 0;
        virtual inline double getSDFwithGrad1(const Eigen::RowVector3d &pos_rel, Eigen::Vector3d &grad) = 0;
      
        // inline void prefetch()
        // {
        //     for (int i = 0; i < num_sdf_map_X_size * num_sdf_map_Y_size * num_sdf_map_Z_size; ++i)
        //     {
        //         _mm_prefetch(reinterpret_cast<const char *>(&num_sdf_map[i]), _MM_HINT_T0);
        //     }
        // }
        inline double getonlySDFNum(const Eigen::Vector3d &pos_rel)
        {
            double x = pos_rel(0);
            double y = pos_rel(1);
            double z = pos_rel(2);
            const int i = floor((x - x_min) / num_sdf_map_res);
            const int j = floor((y - y_min) / num_sdf_map_res);
            const int k = floor((z - z_min) / num_sdf_map_res);

            if (i >= 0 && i < num_sdf_map_X_size - 1 &&
                j >= 0 && j < num_sdf_map_Y_size - 1 &&
                k >= 0 && k < num_sdf_map_Z_size - 1)
            {
                int ijk = i * num_sdf_map_Y_size * num_sdf_map_Z_size + j * num_sdf_map_Z_size + k;
                int i1_jk = (i + 1) * num_sdf_map_Y_size * num_sdf_map_Z_size + j * num_sdf_map_Z_size + k;
                int ij1_k = i * num_sdf_map_Y_size * num_sdf_map_Z_size + (j + 1) * num_sdf_map_Z_size + k;
                int i1_j1_k = (i + 1) * num_sdf_map_Y_size * num_sdf_map_Z_size + (j + 1) * num_sdf_map_Z_size + k;
                int ij_k1 = i * num_sdf_map_Y_size * num_sdf_map_Z_size + j * num_sdf_map_Z_size + k + 1;
                int i1_j_k1 = (i + 1) * num_sdf_map_Y_size * num_sdf_map_Z_size + j * num_sdf_map_Z_size + k + 1;
                int ij1_k1 = i * num_sdf_map_Y_size * num_sdf_map_Z_size + (j + 1) * num_sdf_map_Z_size + k + 1;
                int i1_j1_k1 = (i + 1) * num_sdf_map_Y_size * num_sdf_map_Z_size + (j + 1) * num_sdf_map_Z_size + k + 1;
                double xd = (x - x_min) / num_sdf_map_res - i;
                double yd = (y - y_min) / num_sdf_map_res - j;
                double zd = (z - z_min) / num_sdf_map_res - k;
                double c00 = num_sdf_map[ijk].distance * (1 - xd) + num_sdf_map[i1_jk].distance * xd;
                double c10 = num_sdf_map[ij1_k].distance * (1 - xd) + num_sdf_map[i1_j1_k].distance * xd;
                double c01 = num_sdf_map[ij_k1].distance * (1 - xd) + num_sdf_map[i1_j_k1].distance * xd;
                double c11 = num_sdf_map[ij1_k1].distance * (1 - xd) + num_sdf_map[i1_j1_k1].distance * xd;
                double c0 = c00 * (1 - yd) + c10 * yd;
                double c1 = c01 * (1 - yd) + c11 * yd;
                return (c0 * (1 - zd) + c1 * zd);
            }
            return 1e20;
        };

        inline Eigen::Vector3d getonlyGrad1Num(const Eigen::Vector3d &pos_rel)
        {
            double x = pos_rel(0);
            double y = pos_rel(1);
            double z = pos_rel(2);
            const int i = floor((x - x_min) / num_sdf_map_res);
            const int j = floor((y - y_min) / num_sdf_map_res);
            const int k = floor((z - z_min) / num_sdf_map_res);
            Eigen::Vector3d Grad;
            if (i >= 0 && i < num_sdf_map_X_size - 1 &&
                j >= 0 && j < num_sdf_map_Y_size - 1 &&
                k >= 0 && k < num_sdf_map_Z_size - 1)
            {
                int ijk = i * num_sdf_map_Y_size * num_sdf_map_Z_size + j * num_sdf_map_Z_size + k;
                int i1_jk = (i + 1) * num_sdf_map_Y_size * num_sdf_map_Z_size + j * num_sdf_map_Z_size + k;
                int ij1_k = i * num_sdf_map_Y_size * num_sdf_map_Z_size + (j + 1) * num_sdf_map_Z_size + k;
                int i1_j1_k = (i + 1) * num_sdf_map_Y_size * num_sdf_map_Z_size + (j + 1) * num_sdf_map_Z_size + k;
                int ij_k1 = i * num_sdf_map_Y_size * num_sdf_map_Z_size + j * num_sdf_map_Z_size + k + 1;
                int i1_j_k1 = (i + 1) * num_sdf_map_Y_size * num_sdf_map_Z_size + j * num_sdf_map_Z_size + k + 1;
                int ij1_k1 = i * num_sdf_map_Y_size * num_sdf_map_Z_size + (j + 1) * num_sdf_map_Z_size + k + 1;
                int i1_j1_k1 = (i + 1) * num_sdf_map_Y_size * num_sdf_map_Z_size + (j + 1) * num_sdf_map_Z_size + k + 1;
                double xd = (x - x_min) / num_sdf_map_res - i;
                double yd = (y - y_min) / num_sdf_map_res - j;
                double zd = (z - z_min) / num_sdf_map_res - k;
                Eigen::Vector3d g000 = num_sdf_map[ijk].gradient;
                Eigen::Vector3d g100 = num_sdf_map[i1_jk].gradient;
                Eigen::Vector3d g010 = num_sdf_map[ij1_k].gradient;
                Eigen::Vector3d g110 = num_sdf_map[i1_j1_k].gradient;
                Eigen::Vector3d g001 = num_sdf_map[ij_k1].gradient;
                Eigen::Vector3d g101 = num_sdf_map[i1_j_k1].gradient;
                Eigen::Vector3d g011 = num_sdf_map[ij1_k1].gradient;
                Eigen::Vector3d g111 = num_sdf_map[i1_j1_k1].gradient;
                Grad = g000 * (1 - xd) * (1 - yd) * (1 - zd) + g100 * xd * (1 - yd) * (1 - zd) + g010 * (1 - xd) * yd * (1 - zd) + g110 * xd * yd * (1 - zd) + g001 * (1 - xd) * (1 - yd) * zd + g101 * xd * (1 - yd) * zd + g011 * (1 - xd) * yd * zd + g111 * xd * yd * zd;
                Grad.normalize();
                return Grad;
            }
            return Eigen::Vector3d{0, 0, 0}; // 不迭代了
        };

        inline double getSDFwithGrad1Num(const Eigen::Vector3d &pos_rel, Eigen::Vector3d &grad)
        {
            double x = pos_rel(0);
            double y = pos_rel(1);
            double z = pos_rel(2);
            const int i = floor((x - x_min) / num_sdf_map_res);
            const int j = floor((y - y_min) / num_sdf_map_res);
            const int k = floor((z - z_min) / num_sdf_map_res);

            if (i >= 0 && i < num_sdf_map_X_size - 1 &&
                j >= 0 && j < num_sdf_map_Y_size - 1 &&
                k >= 0 && k < num_sdf_map_Z_size - 1)
            {
                int ijk = i * num_sdf_map_Y_size * num_sdf_map_Z_size + j * num_sdf_map_Z_size + k;
                int i1_jk = (i + 1) * num_sdf_map_Y_size * num_sdf_map_Z_size + j * num_sdf_map_Z_size + k;
                int ij1_k = i * num_sdf_map_Y_size * num_sdf_map_Z_size + (j + 1) * num_sdf_map_Z_size + k;
                int i1_j1_k = (i + 1) * num_sdf_map_Y_size * num_sdf_map_Z_size + (j + 1) * num_sdf_map_Z_size + k;
                int ij_k1 = i * num_sdf_map_Y_size * num_sdf_map_Z_size + j * num_sdf_map_Z_size + k + 1;
                int i1_j_k1 = (i + 1) * num_sdf_map_Y_size * num_sdf_map_Z_size + j * num_sdf_map_Z_size + k + 1;
                int ij1_k1 = i * num_sdf_map_Y_size * num_sdf_map_Z_size + (j + 1) * num_sdf_map_Z_size + k + 1;
                int i1_j1_k1 = (i + 1) * num_sdf_map_Y_size * num_sdf_map_Z_size + (j + 1) * num_sdf_map_Z_size + k + 1;
                double xd = (x - x_min) / num_sdf_map_res - i;
                double yd = (y - y_min) / num_sdf_map_res - j;
                double zd = (z - z_min) / num_sdf_map_res - k;
                double c00 = num_sdf_map[ijk].distance * (1 - xd) + num_sdf_map[i1_jk].distance * xd;
                double c10 = num_sdf_map[ij1_k].distance * (1 - xd) + num_sdf_map[i1_j1_k].distance * xd;
                double c01 = num_sdf_map[ij_k1].distance * (1 - xd) + num_sdf_map[i1_j_k1].distance * xd;
                double c11 = num_sdf_map[ij1_k1].distance * (1 - xd) + num_sdf_map[i1_j1_k1].distance * xd;
                double c0 = c00 * (1 - yd) + c10 * yd;
                double c1 = c01 * (1 - yd) + c11 * yd;
                Eigen::Vector3d g000 = num_sdf_map[ijk].gradient;
                Eigen::Vector3d g100 = num_sdf_map[i1_jk].gradient;
                Eigen::Vector3d g010 = num_sdf_map[ij1_k].gradient;
                Eigen::Vector3d g110 = num_sdf_map[i1_j1_k].gradient;
                Eigen::Vector3d g001 = num_sdf_map[ij_k1].gradient;
                Eigen::Vector3d g101 = num_sdf_map[i1_j_k1].gradient;
                Eigen::Vector3d g011 = num_sdf_map[ij1_k1].gradient;
                Eigen::Vector3d g111 = num_sdf_map[i1_j1_k1].gradient;
                grad = g000 * (1 - xd) * (1 - yd) * (1 - zd) + g100 * xd * (1 - yd) * (1 - zd) + g010 * (1 - xd) * yd * (1 - zd) + g110 * xd * yd * (1 - zd) + g001 * (1 - xd) * (1 - yd) * zd + g101 * xd * (1 - yd) * zd + g011 * (1 - xd) * yd * zd + g111 * xd * yd * zd;
                grad.normalize();
                return (c0 * (1 - zd) + c1 * zd);
            }
            // assert(false && "尝试使用数值方法getSDFwithGrad1Num,查寻点超出存储的SDF地图范围");
            // std::cout<<"========================getSDFwithGrad1Num shitshitshit====================="<<std::endl;
            return 1e20;
        }
    };

    class Ball : public BasicShape
    {
    private:
        double radius;

    public:
        Ball(const Config &conf) : BasicShape(conf) { radius = 1.0; }
        Ball(double r, double num_res, Config &conf) : BasicShape(conf)
        {
            radius = r;
            initShape<true>(2 * r, 2 * r, 2 * r, num_res);
        }

    public:
        inline double getonlySDF(const Eigen::RowVector3d &pos_rel)
        {
            return pos_rel.norm() - radius;
        }
        inline double getonlySDF(const Eigen::RowVector3d &pos_rel, const Eigen::Matrix3d &R_obj)
        {
            return pos_rel.norm() - radius;
        }
        inline Eigen::Vector3d getonlyGrad1(const Eigen::RowVector3d &pos_rel)
        {
            return pos_rel.normalized().transpose();
        }
        double getSDFwithGrad1(const Eigen::RowVector3d &pos_rel, Eigen::Vector3d &grad)
        {
            grad = pos_rel.normalized();
            return pos_rel.norm() - radius;
        }
    };

    class Point : public BasicShape
    {
    public:
        Point(const Config &conf) : BasicShape(conf) {}
        Point(double r, double num_res, Config &conf) : BasicShape(conf)
        {
            initShape<true>(2 * r, 2 * r, 2 * r, num_res);
        }

    public:
        inline double getonlySDF(const Eigen::RowVector3d &pos_rel)
        {
            return pos_rel.norm();
        }
        inline double getonlySDF(const Eigen::RowVector3d &pos_rel, const Eigen::Matrix3d &R_obj)
        {
            return pos_rel.norm();
        }

        inline Eigen::Vector3d getonlyGrad1(const Eigen::RowVector3d &pos_rel)
        {
            return pos_rel.normalized().transpose();
        }
        double getSDFwithGrad1(const Eigen::RowVector3d &pos_rel, Eigen::Vector3d &grad)
        {
            grad = pos_rel.normalized();
            return pos_rel.norm();
        }
      
    };

    class Generalshape : public BasicShape
    {
    public:
        bool analytic{false};
        Eigen::MatrixX4d hPoly;
        Eigen::Matrix3Xd vertices;
        Eigen::Matrix3Xd mesh;
        Eigen::MatrixXi F;
        Eigen::MatrixXd V;
        Eigen::Vector3d interior{Eigen::Vector3d::Zero()};

        Eigen::MatrixX4d hPoly_var;
        Eigen::Matrix3Xd vertices_var;
        Eigen::Matrix3Xd mesh_var;
        Eigen::Vector3d interior_var;
        Eigen::Vector3d trans{Eigen::Vector3d::Zero()};
        Eigen::Matrix3d Rotate{Eigen::Matrix3d::Identity()};
        Eigen::AngleAxisd roll;
        Eigen::AngleAxisd pitch;
        Eigen::AngleAxisd yaw;
        Eigen::Vector3d euler{Eigen::Vector3d::Zero()}; //{yaw,pitch,roll}弧度制
        bool init{false};
        int ids;

        igl::AABB<Eigen::MatrixXd, 3> tree;
        igl::FastWindingNumberBVH fwn_bvh;

    public:
        double getradius() const
        {

            int size = vertices.cols();
            Eigen::Vector3d dis;
            double radius = -1;
            for (int i = 0; i < size; i++)
            {
                dis = vertices.col(i) - Eigen::Vector3d::Zero();
                if (dis.norm() > radius)
                {
                    radius = dis.norm();
                }
            }
            return radius;
        }

        //
        void getattitude(Eigen::Vector3d &trans_, double &yaw)
        {
            trans_ = trans;
            yaw = euler(0);
        }
        struct hashFunction
        {
            std::size_t operator()(const pair<Eigen::Vector3d, Eigen::Vector3d> &x) const
            {
                size_t seed = 0;
                for (size_t i = 0; i < 3; i++)
                {
                    seed *= x.first[i] + 0x9e3779b9 + (seed << 6) + (seed >> 2);
                    seed *= x.second[i] + 0x9e3779b9 + (seed << 6) + (seed >> 2);
                }

                return seed;
            }
        };
        typedef unordered_set<std::pair<Eigen::Vector3d, Eigen::Vector3d>, hashFunction> edgepariset;
        typedef std::shared_ptr<Generalshape> Ptr;

        /**
         * 是否在凸多面体内
         *
         * @param pt 查询点
         */
        bool IsPointInside(const Vector3d &pt)
        {
            Eigen::Vector4d P;
            P << pt, 1;
            Eigen::VectorXd results = hPoly * P;
            return results.maxCoeff() <= 0;
        }

        /**
         * 变换
         * @param quat 旋转变换
         * @param trans 平移变换
         */
        void inline Transform(const Eigen::Vector4d &quat, const Vector3d &trans)
        {
            Eigen::Quaterniond q(quat[0], quat[1], quat[2], quat[3]);
            Transform(q.toRotationMatrix(), trans);
        }

        /**
         * 可视化重置
         */
        void inline reset()
        {
            Rotate.setIdentity();
            trans.setZero();
            euler.setZero();
            roll = AngleAxisd(0, Vector3d::UnitX());
            pitch = AngleAxisd(0, Vector3d::UnitY());
            yaw = AngleAxisd(0, Vector3d::UnitZ());
        }
        /**
         * 得到yaml中的平移与旋转变换,方便子类继承使用
         * @param R 旋转变换
         * @param trans_ 平移变换
         */
        void getTransform(Matrix3d &R, RowVector3d &trans_)
        {
            R = Rotate;
            trans_ = trans.transpose();
        }
        /**
         * 变换
         * @param R 旋转变换
         * @param trans_ 平移变换
         */
        void inline Transform(const Matrix3d &R, const Vector3d &trans_)
        {

            trans = trans_;
            Rotate = R;
            vertices_var = ( R * vertices).colwise() + trans;
            mesh_var = (R * mesh).colwise() + trans;
            interior_var = (R * interior).colwise() + trans;
            euler = Rotate.eulerAngles(2, 1, 0);
            roll = AngleAxisd(euler(2), Vector3d::UnitX());
            pitch = AngleAxisd(euler(1), Vector3d::UnitY());
            yaw = AngleAxisd(euler(0), Vector3d::UnitZ());
            Eigen::Vector3d normal, point, edge0, edge1;

            for (int i = 0; i < ids / 3; i++)
            {
                point = mesh_var.col(3 * i + 1);
                edge0 = point - mesh_var.col(3 * i);
                edge1 = mesh_var.col(3 * i + 2) - point;
                normal = edge0.cross(edge1).normalized();

                hPoly_var(i, 0) = normal(0);
                hPoly_var(i, 1) = normal(1);
                hPoly_var(i, 2) = normal(2);
                hPoly_var(i, 3) = -normal.dot(point);
            }
        }
        Generalshape(/* args */) = delete;
        Generalshape(const Config &conf, bool analytic = false);                                 
        virtual inline double getonlySDF(const Eigen::RowVector3d &pos_rel);                               
        virtual inline double getonlySDF(const Eigen::RowVector3d &pos_rel, const Eigen::Matrix3d &R_obj); 
        virtual inline Eigen::Vector3d getonlyGrad1(const Eigen::RowVector3d &pos_rel);                   
        virtual inline double getSDFwithGrad1(const Eigen::RowVector3d &pos_rel, Eigen::Vector3d &grad);   
      

            ~Generalshape(){};
        };

        class Torus : public Generalshape
        {
        private:
            const double tubeRadius{2.5}; 
            const double torusRadius{0.3}; 
            Eigen::RowVector3d trans{Eigen::RowVector3d::Zero()};
            Eigen::Matrix3d Rotate{Eigen::Matrix3d::Identity()};

        public:
            typedef std::shared_ptr<Torus> Ptr;
            Torus() = delete;
            Torus(const Config &conf) : Generalshape(conf, true) 
            {
                getTransform(Rotate, trans);
                // std::cout<<"Rotate:"<<Rotate<<std::endl;
            }

      
            inline double getonlySDF(const Eigen::RowVector3d &pos_rel)
            {
       
                Eigen::RowVector3d Pos_rel = (pos_rel - trans) * Rotate; 
                Eigen::Vector2d q(std::sqrt(Pos_rel.x() * Pos_rel.x() + Pos_rel.z() * Pos_rel.z()) - tubeRadius, Pos_rel.y());
                return q.norm() - torusRadius;
            }
   
            inline double getonlySDF(const Eigen::RowVector3d &pos, const Eigen::Matrix3d &R_obj)
            {
                Eigen::RowVector3d pos_rel = (pos - trans) * Rotate * R_obj;
                Eigen::Vector2d q(std::sqrt(pos_rel.x() * pos_rel.x() + pos_rel.z() * pos_rel.z()) - tubeRadius, pos_rel.y());
                return q.norm() - torusRadius;
            }
            DEFINE_USEFUL_FUNCTION()

            ~Torus(){};
        };
        class Torus_big : public Generalshape
        {
        private:
            const double tubeRadius{3.5}; 
            const double torusRadius{0.3}; 
            Eigen::RowVector3d trans{Eigen::RowVector3d::Zero()};
            Eigen::Matrix3d Rotate{Eigen::Matrix3d::Identity()};

        public:
            typedef std::shared_ptr<Torus_big> Ptr;
            Torus_big() = delete;
            Torus_big(const Config &conf) : Generalshape(conf, true) 
            {
                getTransform(Rotate, trans);
                // std::cout<<"Rotate:"<<Rotate<<std::endl;
            }

            inline double getonlySDF(const Eigen::RowVector3d &pos_rel)
            {
         
                Eigen::RowVector3d Pos_rel = (pos_rel - trans) * Rotate; 
                Eigen::Vector2d q(std::sqrt(Pos_rel.x() * Pos_rel.x() + Pos_rel.z() * Pos_rel.z()) - tubeRadius, Pos_rel.y());
                return q.norm() - torusRadius;
            }
   
            inline double getonlySDF(const Eigen::RowVector3d &pos, const Eigen::Matrix3d &R_obj)
            {
                Eigen::RowVector3d pos_rel = (pos - trans) * Rotate * R_obj;
                Eigen::Vector2d q(std::sqrt(pos_rel.x() * pos_rel.x() + pos_rel.z() * pos_rel.z()) - tubeRadius, pos_rel.y());
                return q.norm() - torusRadius;
            }
            DEFINE_USEFUL_FUNCTION()

            ~Torus_big(){};
        };
        class Cappedtorus : public Generalshape
        {
        private:
            const Eigen::Vector2d sc{Eigen::Vector2d(std::sin(40), std::cos(40))};
            const double ra{3.5};
            const double rb{0.3};
            Eigen::RowVector3d trans{Eigen::RowVector3d::Zero()};
            Eigen::Matrix3d Rotate{Eigen::Matrix3d::Identity()};

        public:
            typedef std::shared_ptr<Cappedtorus> Ptr;
            Cappedtorus() = delete;
            Cappedtorus(const Config &conf) : Generalshape(conf, true) 
            {
                getTransform(Rotate, trans);
            }

 
            inline double getonlySDF(const Eigen::RowVector3d &pos_rel)
            {
                Eigen::RowVector3d Pos_rel = (pos_rel - trans) * Rotate; 
                Pos_rel.x() = std::abs(Pos_rel.x());
                double k = (sc.y() * Pos_rel.x() > sc.x() * Pos_rel.y()) ? (Pos_rel.x() * sc.x() + Pos_rel.y() * sc.y()) : Pos_rel.head<2>().norm();
                return std::sqrt(Pos_rel.dot(Pos_rel) + ra * ra - 2.0f * ra * k) - rb;
            }
       
            inline double getonlySDF(const Eigen::RowVector3d &pos, const Eigen::Matrix3d &R_obj)
            {
                Eigen::RowVector3d Pos_rel = (pos - trans) * Rotate * R_obj;
                Pos_rel.x() = std::abs(Pos_rel.x());
                double k = (sc.y() * Pos_rel.x() > sc.x() * Pos_rel.y()) ? (Pos_rel.x() * sc.x() + Pos_rel.y() * sc.y()) : Pos_rel.head<2>().norm();
                return std::sqrt(Pos_rel.dot(Pos_rel) + ra * ra - 2.0f * ra * k) - rb;
            }
            DEFINE_USEFUL_FUNCTION()

            ~Cappedtorus(){};
        };

        class CappedCone : public Generalshape
        {
        private:
            const double ra{2};
            const double rb{0.8};
            const Eigen::Vector3d a{0, 0, -1};
            const Eigen::Vector3d b{0, 0, 1};
            

            Eigen::RowVector3d trans{Eigen::RowVector3d::Zero()};
            Eigen::Matrix3d Rotate{Eigen::Matrix3d::Identity()};

        public:
            typedef std::shared_ptr<CappedCone> Ptr;
            CappedCone() = delete;
            CappedCone(const Config &conf) : Generalshape(conf, true) 
            {
                getTransform(Rotate, trans);
            }
            inline double clip(double value, double min_val, double max_val)
            {
                return std::max(std::min(value, max_val), min_val);
            }
    
            inline double getonlySDF(const Eigen::RowVector3d &Pos_rel)
            {
                Eigen::RowVector3d pos_rel = (Pos_rel - trans) * Rotate; 
                double rba = rb - ra;
                double baba = (b - a).dot(b - a);
                double papa = (pos_rel.transpose() - a).dot(pos_rel.transpose() - a);
                double paba = (pos_rel.transpose() - a).dot(b - a) / baba;
                double x = std::sqrt(papa - paba * paba * baba);
                double cax = std::max(0.0, x - (paba < 0.5 ? ra : rb));
                double cay = std::abs(paba - 0.5) - 0.5;
                double k = rba * rba + baba;
                double f = clip((rba * (x - ra) + paba * baba) / k, 0.0, 1.0);
                double cbx = x - ra - f * rba;
                double cby = paba - f;
                double s = (cbx < 0 && cay < 0) ? -1 : 1;
                double d = std::sqrt(std::min(cax * cax + cay * cay * baba, cbx * cbx + cby * cby * baba));
                return s * std::sqrt(std::abs(d)) / std::abs(baba);
            }
            // 子类实现
            inline double getonlySDF(const Eigen::RowVector3d &pos, const Eigen::Matrix3d &R_obj)
            {
       
                Eigen::RowVector3d pos_rel = (pos - trans) * Rotate * R_obj;
                double rba = rb - ra;
                double baba = (b - a).dot(b - a);
                double papa = (pos_rel.transpose() - a).dot(pos_rel.transpose() - a);
                double paba = (pos_rel.transpose() - a).dot(b - a) / baba;
                double x = std::sqrt(papa - paba * paba * baba);
                double cax = std::max(0.0, x - (paba < 0.5 ? ra : rb));
                double cay = std::abs(paba - 0.5) - 0.5;
                double k = rba * rba + baba;
                double f = clip((rba * (x - ra) + paba * baba) / k, 0.0, 1.0);
                double cbx = x - ra - f * rba;
                double cby = paba - f;
                double s = (cbx < 0 && cay < 0) ? -1 : 1;
                double d = std::sqrt(std::min(cax * cax + cay * cay * baba, cbx * cbx + cby * cby * baba));
                return s * std::sqrt(std::abs(d)) / std::abs(baba);
            }
            DEFINE_USEFUL_FUNCTION()

            ~CappedCone(){};
        };

        class RoundedCone : public Generalshape
        {
        private:
            const double r1{1.5};
            const double r2{0.6};
            const double h{4.5};
          
            Eigen::RowVector3d trans{Eigen::RowVector3d::Zero()};
            Eigen::Matrix3d Rotate{Eigen::Matrix3d::Identity()};

        public:
            typedef std::shared_ptr<RoundedCone> Ptr;
            RoundedCone() = delete;
            RoundedCone(const Config &conf) : Generalshape(conf, true) 
            {
                getTransform(Rotate, trans);
            }

            inline double getonlySDF(const Eigen::RowVector3d &pos_rel)
            {
                Eigen::RowVector3d p = (pos_rel - trans) * Rotate; 
                Eigen::Vector2d q(std::sqrt(p.x() * p.x() + p.y() * p.y()), p.z());
                double b = (r1 - r2) / h;
                double a = std::sqrt(1.0 - b * b);
                double k = -b * q.x() + a * q.y();
                double c1 = q.norm() - r1;
                double c2 = std::sqrt(q.x() * q.x() + (q.y() - h) * (q.y() - h)) - r2;
                double c3 = (a * q.x() + b * q.y()) - r1;
                return (k < 0) ? c1 : ((k > a * h) ? c2 : c3);
            }
  
            inline double getonlySDF(const Eigen::RowVector3d &pos, const Eigen::Matrix3d &R_obj)
            {
                // Eigen::RowVector3d p;
                Eigen::RowVector3d p = (pos - trans) * Rotate * R_obj;
                Eigen::Vector2d q(std::sqrt(p.x() * p.x() + p.y() * p.y()), p.z());
                double b = (r1 - r2) / h;
                double a = std::sqrt(1.0 - b * b);
                double k = -b * q.x() + a * q.y();
                double c1 = q.norm() - r1;
                double c2 = std::sqrt(q.x() * q.x() + (q.y() - h) * (q.y() - h)) - r2;
                double c3 = (a * q.x() + b * q.y()) - r1;
                return (k < 0) ? c1 : ((k > a * h) ? c2 : c3);
            }
            DEFINE_USEFUL_FUNCTION()

            ~RoundedCone(){};
        };

        class WireframeBox : public Generalshape
        {
        private:
            const Eigen::Vector3d size{Eigen::Vector3d(1.8, 2.5, 3.5)};
            const double thickness{0.1};

            Eigen::RowVector3d trans{Eigen::RowVector3d::Zero()};
            Eigen::Matrix3d Rotate{Eigen::Matrix3d::Identity()};

        public:
            typedef std::shared_ptr<WireframeBox> Ptr;
            WireframeBox() = delete;
            WireframeBox(const Config &conf) : Generalshape(conf, true)
            {
                getTransform(Rotate, trans);
            }
            inline double g(const double a, const double b, const double c)
            {
                Eigen::Vector3d v(a, b, c);
                Eigen::Vector3d vMax = v.cwiseMax(Eigen::Vector3d::Zero());
                return vMax.norm() + std::min(std::max(a, std::max(b, c)), 0.0);
            }

            inline double getonlySDF(const Eigen::RowVector3d &Pos_rel)
            {
                Eigen::RowVector3d p = (Pos_rel - trans) * Rotate; 
                Eigen::Vector3d pShifted = p.cwiseAbs().transpose().array() - size.array() / 2 - thickness / 2;
                Eigen::Vector3d q = (pShifted.transpose().array() + thickness / 2).cwiseAbs() - thickness / 2;
                double px = pShifted.x();
                double py = pShifted.y();
                double pz = pShifted.z();
                double qx = q.x();
                double qy = q.y();
                double qz = q.z();
                return std::min({std::min(g(px, qy, qz), g(qx, py, qz)), g(qx, qy, pz)});
            }

            inline double getonlySDF(const Eigen::RowVector3d &pos, const Eigen::Matrix3d &R_obj)
            {
                // Eigen::RowVector3d p;
                Eigen::RowVector3d p = (pos - trans) * Rotate * R_obj;
                Eigen::Vector3d pShifted = p.cwiseAbs().transpose().array() - size.array() / 2 - thickness / 2;
                Eigen::Vector3d q = (pShifted.transpose().array() + thickness / 2).cwiseAbs() - thickness / 2;
                double px = pShifted.x();
                double py = pShifted.y();
                double pz = pShifted.z();
                double qx = q.x();
                double qy = q.y();
                double qz = q.z();
                return std::min({std::min(g(px, qy, qz), g(qx, py, qz)), g(qx, qy, pz)});
            }
            DEFINE_USEFUL_FUNCTION()

            ~WireframeBox(){};
        };

        class BendLinear : public Generalshape
        {
        private:
            Eigen::RowVector3d trans{Eigen::RowVector3d::Zero()};
            Eigen::Matrix3d Rotate{Eigen::Matrix3d::Identity()};

        public:
            typedef std::shared_ptr<BendLinear> Ptr;
            inline double in_out_quad(double t)
            {
                double u = 2 * t - 1;
                double a = 2 * t * t;
                double b = -0.5 * (u * (u - 2) - 1);
                return (t < 0.5) ? a : b;
            }
            BendLinear() = delete;
            BendLinear(const Config &conf) : Generalshape(conf, true) 
            {
                getTransform(Rotate, trans);
            }
            inline double clip(double value, double min_val, double max_val)
            {
                return std::max(std::min(value, max_val), min_val);
            }
            inline SDFFunction capsule(const Eigen::Vector3d &a, const Eigen::Vector3d &b, double radius)
            {
                auto f = [a, b, radius, this](const Eigen::Vector3d &p) -> double
                {
                    Eigen::Vector3d pa = p - a;
                    Eigen::Vector3d ba = b - a;
                    double h = std::max(std::min(pa.dot(ba) / ba.dot(ba), 1.0), 0.0); // clip
                    Eigen::Vector3d pointOnLine = a + h * ba;
                    return (pa - h * ba).norm() - radius;
                };
                return f;
            }
   
            inline double getonlySDF(const Eigen::RowVector3d &pos_rel)
            {
                Eigen::RowVector3d p = (pos_rel - trans) * Rotate; 
                const Vector3d p0{Eigen::Vector3d(0, 0, -1)};      
                const Vector3d p1{Eigen::Vector3d(0, 0, 1)};
                const Vector3d v{Eigen::Vector3d(-1, 0, 0)};
                const Vector3d ab = p1 - p0;
                double t = clip((p.transpose() - p0).dot(ab) / (ab.dot(ab)), 0, 1);
                auto f = capsule(Eigen::Vector3d(0, 0, -2), Eigen::Vector3d(0, 0, 2), 0.25);
                return f(in_out_quad(t) * v + p.transpose());
            }
         
            inline double getonlySDF(const Eigen::RowVector3d &pos, const Eigen::Matrix3d &R_obj)
            {
                Eigen::RowVector3d p = (pos - trans) * Rotate * R_obj;
                const Vector3d p0{Eigen::Vector3d(0, 0, -1)};
                const Vector3d p1{Eigen::Vector3d(0, 0, 1)};
                const Vector3d v{Eigen::Vector3d(-1, 0, 0)};
                const Vector3d ab = p1 - p0;
           
                double t = clip((p.transpose() - p0).dot(ab) / (ab.dot(ab)), 0, 1);
                auto f = capsule(Eigen::Vector3d(0, 0, -2), Eigen::Vector3d(0, 0, 2), 0.25);
                return f(in_out_quad(t) * v + p.transpose());
            }
            DEFINE_USEFUL_FUNCTION()

            ~BendLinear(){};
        };
        class BendLinear_big : public Generalshape
        {
        private:
            Eigen::RowVector3d trans{Eigen::RowVector3d::Zero()};
            Eigen::Matrix3d Rotate{Eigen::Matrix3d::Identity()};

        public:
            typedef std::shared_ptr<BendLinear_big> Ptr;
            inline double in_out_quad(double t)
            {
                double u = 2 * t - 1;
                double a = 2 * t * t;
                double b = -0.5 * (u * (u - 2) - 1);
                return (t < 0.5) ? a : b;
            }
            BendLinear_big() = delete;
            BendLinear_big(const Config &conf) : Generalshape(conf, true) 
            {
                getTransform(Rotate, trans);
            }
            inline double clip(double value, double min_val, double max_val)
            {
                return std::max(std::min(value, max_val), min_val);
            }
            inline SDFFunction capsule(const Eigen::Vector3d &a, const Eigen::Vector3d &b, double radius)
            {
                auto f = [a, b, radius, this](const Eigen::Vector3d &p) -> double
                {
                    Eigen::Vector3d pa = p - a;
                    Eigen::Vector3d ba = b - a;
                    double h = std::max(std::min(pa.dot(ba) / ba.dot(ba), 1.0), 0.0); // clip
                    Eigen::Vector3d pointOnLine = a + h * ba;
                    return (pa - h * ba).norm() - radius;
                };
                return f;
            }
    
            inline double getonlySDF(const Eigen::RowVector3d &pos_rel)
            {
                Eigen::RowVector3d p = (pos_rel - trans) * Rotate; 
                const Vector3d p0{Eigen::Vector3d(0, 0, -1)};      
                const Vector3d p1{Eigen::Vector3d(0, 0, 1)};
                const Vector3d v{Eigen::Vector3d(-1, 0, 0)};
                const Vector3d ab = p1 - p0;
                double t = clip((p.transpose() - p0).dot(ab) / (ab.dot(ab)), 0, 1);
                auto f = capsule(Eigen::Vector3d(0, 0, -3.2), Eigen::Vector3d(0, 0, 3.2), 0.45);
                return f(in_out_quad(t) * v + p.transpose());
            }
   
            inline double getonlySDF(const Eigen::RowVector3d &pos, const Eigen::Matrix3d &R_obj)
            {
                Eigen::RowVector3d p = (pos - trans) * Rotate * R_obj;
                const Vector3d p0{Eigen::Vector3d(0, 0, -1)}; 
                const Vector3d p1{Eigen::Vector3d(0, 0, 1)};
                const Vector3d v{Eigen::Vector3d(-1, 0, 0)};
                const Vector3d ab = p1 - p0;
                // Eigen::RowVector3d p;
                double t = clip((p.transpose() - p0).dot(ab) / (ab.dot(ab)), 0, 1);
                auto f = capsule(Eigen::Vector3d(0, 0, -3.2), Eigen::Vector3d(0, 0, 3.2), 0.45);
                return f(in_out_quad(t) * v + p.transpose());
            }
            DEFINE_USEFUL_FUNCTION()

            ~BendLinear_big(){};
        };

        class TwistBox : public Generalshape
        {
        private:
            const Eigen::Vector3d size{Eigen::Vector3d(2.0, 2.0, 2.0)}; // Box size
            const double k{PI / 6};                                     // twist angle

            Eigen::RowVector3d trans{Eigen::RowVector3d::Zero()};
            Eigen::Matrix3d Rotate{Eigen::Matrix3d::Identity()};

        public:
            typedef std::shared_ptr<TwistBox> Ptr;
            TwistBox() = delete;
            TwistBox(const Config &conf) : Generalshape(conf, true) 
            {
                getTransform(Rotate, trans);
            }

    
            inline double getonlySDF(const Eigen::RowVector3d &P_rel)
            {
                Eigen::RowVector3d p_rel = (P_rel - trans) * Rotate;
                double x = p_rel(0); 
                double y = p_rel(1);
                double z = p_rel(2);
                double c = cos(k * z);
                double s = sin(k * z);
                Eigen::Vector3d p(c * x - s * y, s * x + c * y, z); 
                Eigen::Vector3d q = p.cwiseAbs() - size / 2.0;
                double qx = std::max(q(0), 0.0);
                double qy = std::max(q(1), 0.0);
                double qz = std::max(q(2), 0.0);
                return std::sqrt(qx * qx + qy * qy + qz * qz) + std::min(std::max(q(0), std::max(q(1), q(2))), 0.0);
            }
       
            inline double getonlySDF(const Eigen::RowVector3d &pos, const Eigen::Matrix3d &R_obj)
            {
 
                Eigen::RowVector3d p_rel = (pos - trans) * Rotate * R_obj;
                double x = p_rel(0); 
                double y = p_rel(1);
                double z = p_rel(2);
                double c = cos(k * z);
                double s = sin(k * z);
                Eigen::Vector3d p(c * x - s * y, s * x + c * y, z); 
                Eigen::Vector3d q = p.cwiseAbs() - size / 2.0;
                double qx = std::max(q(0), 0.0);
                double qy = std::max(q(1), 0.0);
                double qz = std::max(q(2), 0.0);
                return std::sqrt(qx * qx + qy * qy + qz * qz) + std::min(std::max(q(0), std::max(q(1), q(2))), 0.0);
            }
            DEFINE_USEFUL_FUNCTION()
            ~TwistBox(){};
        };

        class BendBox : public Generalshape
        {
        private:
            const Eigen::Vector3d size{Eigen::Vector3d(2.0, 2.0, 2.0)}; // Box size
            const double k{0.5};                                        // twist angle

            Eigen::RowVector3d trans{Eigen::RowVector3d::Zero()};
            Eigen::Matrix3d Rotate{Eigen::Matrix3d::Identity()};

        public:
            typedef std::shared_ptr<BendBox> Ptr;
            BendBox() = delete;
            BendBox(const Config &conf) : Generalshape(conf, true) 
            {
                getTransform(Rotate, trans);
            }

   
            inline double getonlySDF(const Eigen::RowVector3d &P_rel)
            {
                Eigen::RowVector3d p_rel = (P_rel - trans) * Rotate;
                double x = p_rel(0); 
                double y = p_rel(1);
                double z = p_rel(2);
                double c = std::cos(k * x);
                double s = std::sin(k * x);
                Eigen::Vector3d p(c * x - s * y, s * x + c * y, z); 
                Eigen::Vector3d q = p.cwiseAbs() - size / 2.0;
                double qx = std::max(q(0), 0.0);
                double qy = std::max(q(1), 0.0);
                double qz = std::max(q(2), 0.0);
                return std::sqrt(qx * qx + qy * qy + qz * qz) + std::min(std::max(q(0), std::max(q(1), q(2))), 0.0);
            }
      
            inline double getonlySDF(const Eigen::RowVector3d &pos, const Eigen::Matrix3d &R_obj)
            {
  
                Eigen::RowVector3d p_rel = (pos - trans) * Rotate * R_obj;
                double x = p_rel(0); 
                double y = p_rel(1);
                double z = p_rel(2);
                double c = std::cos(k * x);
                double s = std::sin(k * x);
                Eigen::Vector3d p(c * x - s * y, s * x + c * y, z); 
                Eigen::Vector3d q = p.cwiseAbs() - size / 2.0;
                double qx = std::max(q(0), 0.0);
                double qy = std::max(q(1), 0.0);
                double qz = std::max(q(2), 0.0);
                return std::sqrt(qx * qx + qy * qy + qz * qz) + std::min(std::max(q(0), std::max(q(1), q(2))), 0.0);
            }
            DEFINE_USEFUL_FUNCTION()
            ~BendBox(){};
        };
        class Table : public Generalshape
        {
        private:
            const Eigen::Vector3d a1{Eigen::Vector3d(0.0, 0.0, 0.0)};  
            const Eigen::Vector3d b1{Eigen::Vector3d(3.5, 1.75, 0.7)}; 
            const Eigen::Vector3d a2{Eigen::Vector3d(2.8, 1.05, 0.0)}; 
            const Eigen::Vector3d b2{Eigen::Vector3d(3.5, 1.75, 2.8)}; 

            Eigen::RowVector3d trans{Eigen::RowVector3d::Zero()};
            Eigen::Matrix3d Rotate{Eigen::Matrix3d::Identity()};

        public:
            typedef std::shared_ptr<Table> Ptr;
            Table() = delete;
            Table(const Config &conf) : Generalshape(conf, true) 
            {
                getTransform(Rotate, trans);
            }

     
            inline double getonlySDF(const Eigen::RowVector3d &P_rel)
            {
                Eigen::RowVector3d p_rel = (P_rel - trans) * Rotate;
                double x = std::abs(p_rel(0));
                double y = std::abs(p_rel(1));
                double z = p_rel(2);

                Eigen::Vector3d q = (Eigen::Vector3d(x, y, z) - (a1 + b1) / 2).cwiseAbs() - (b1 - a1) / 2.0;
                double qx = std::max(q(0), 0.0);
                double qy = std::max(q(1), 0.0);
                double qz = std::max(q(2), 0.0);
                double f1 = std::sqrt(qx * qx + qy * qy + qz * qz) + std::min(std::max(q(0), std::max(q(1), q(2))), 0.0);
                q = (Eigen::Vector3d(x, y, z) - (a2 + b2) / 2).cwiseAbs() - (b2 - a2) / 2.0;
                qx = std::max(q(0), 0.0);
                qy = std::max(q(1), 0.0);
                qz = std::max(q(2), 0.0);
                double f2 = std::sqrt(qx * qx + qy * qy + qz * qz) + std::min(std::max(q(0), std::max(q(1), q(2))), 0.0);
                return std::min(f1, f2);
            }
 
            inline double getonlySDF(const Eigen::RowVector3d &pos, const Eigen::Matrix3d &R_obj)
            {
  
                Eigen::RowVector3d p_rel = (pos - trans) * Rotate * R_obj;
                double x = std::abs(p_rel(0));
                double y = std::abs(p_rel(1));
                double z = p_rel(2);

                Eigen::Vector3d q = (Eigen::Vector3d(x, y, z) - (a1 + b1) / 2).cwiseAbs() - (b1 - a1) / 2.0;
                double qx = std::max(q(0), 0.0);
                double qy = std::max(q(1), 0.0);
                double qz = std::max(q(2), 0.0);
                double f1 = std::sqrt(qx * qx + qy * qy + qz * qz) + std::min(std::max(q(0), std::max(q(1), q(2))), 0.0);
                q = (Eigen::Vector3d(x, y, z) - (a2 + b2) / 2).cwiseAbs() - (b2 - a2) / 2.0;
                qx = std::max(q(0), 0.0);
                qy = std::max(q(1), 0.0);
                qz = std::max(q(2), 0.0);
                double f2 = std::sqrt(qx * qx + qy * qy + qz * qz) + std::min(std::max(q(0), std::max(q(1), q(2))), 0.0);
                return std::min(f1, f2);
            }
            DEFINE_USEFUL_FUNCTION()
            ~Table(){};
        };

        class Blobby : public Generalshape
        {
        private:
            Eigen::RowVector3d trans{Eigen::RowVector3d::Zero()};
            Eigen::Matrix3d Rotate{Eigen::Matrix3d::Identity()};

        public:
            typedef std::shared_ptr<Blobby> Ptr;
            Blobby() = delete;
            Blobby(const Config &conf) : Generalshape(conf, true) 
            {
                getTransform(Rotate, trans);
            }

      
            inline double getonlySDF(const Eigen::RowVector3d &P_rel)
            {
                Eigen::RowVector3d p_rel = (P_rel - trans) * Rotate;
                double x = p_rel(0); 
                double y = p_rel(1);
                double z = p_rel(2);
            }
  
            inline double getonlySDF(const Eigen::RowVector3d &pos, const Eigen::Matrix3d &R_obj)
            {
   
                Eigen::RowVector3d p_rel = (pos - trans) * Rotate * R_obj;
                double x = p_rel(0); 
                double y = p_rel(1);
                double z = p_rel(2);
            }
            DEFINE_USEFUL_FUNCTION()
            ~Blobby(){};
        };

        class Trefoil : public Generalshape
        {
        private:
            Eigen::RowVector3d trans{Eigen::RowVector3d::Zero()};
            Eigen::Matrix3d Rotate{Eigen::Matrix3d::Identity()};
#define DMINQ(id) \
    if (d < dMin) \
    {             \
        dMin = d; \
    }

        public:
            typedef std::shared_ptr<Trefoil> Ptr;
            Trefoil() = delete;
            Trefoil(const Config &conf) : Generalshape(conf, true) 
            {
                getTransform(Rotate, trans);
            }
            typedef Eigen::Vector2d vec2;
            typedef Eigen::Vector3d vec3;
            typedef Eigen::Matrix3d mat3;

            double dstFar = 100;

            inline double trefoil_PrBox2Df(const vec2 &p, const vec2 &b)
            {
                vec2 d = p.cwiseAbs() - b;
                return std::min(std::max(d.x(), d.y()), 0.0) + (d.cwiseMax(0.0)).norm();
            }

            inline vec2 trefoil_Rot2D(const vec2 &q, double a)
            {
                double cos_a = std::cos(a);
                double sin_a = std::sin(a);
                return q * cos_a + vec2(q.y(), -q.x()) * sin_a; //
            }

            inline double ObjDf(const vec2 &p, double r, double py)
            {
                vec2 q = p;
                double dMin = dstFar;
                double a = std::atan2(q.y(), q.x());
                // q.xz() = vec2(q.xz().norm() - r, q.y());
                q.x() = q.norm() - r;
                q.y() = py;
                q = trefoil_Rot2D(q, 1.5 * a);
                q = trefoil_Rot2D(q, -M_PI * (std::floor(std::atan2(q.y(), q.x()) / M_PI + 0.5)));
                q.x() -= 1.0;
                double d = (trefoil_PrBox2Df(q, vec2(0.2, 0.2))) - 0.05;
                DMINQ(1);
                return 0.4 * dMin;
            }

            inline double getonlySDF(const Eigen::RowVector3d &P_rel)
            {
                Eigen::RowVector3d p_rel = (P_rel - trans) * Rotate;
                double x = p_rel(0); 
                double y = p_rel(1);
                double z = p_rel(2);
                return ObjDf(vec2(x, y), 3.5, -z);
            }

            inline double getonlySDF(const Eigen::RowVector3d &pos, const Eigen::Matrix3d &R_obj)
            {

                Eigen::RowVector3d p_rel = (pos - trans) * Rotate * R_obj;
                double x = p_rel(0); 
                double y = p_rel(1);
                double z = p_rel(2);
                return ObjDf(vec2(x, y), 3.5, -z);
            }
            DEFINE_USEFUL_FUNCTION()
            ~Trefoil(){};
        };

        class SmoothDifference : public Generalshape
        {
        private:
            const Eigen::Vector3d size{Eigen::Vector3d(3.0, 3.0, 0.5)}; // Box size
            const double radius{1.0};                                   // twist angle

            Eigen::RowVector3d trans{Eigen::RowVector3d::Zero()};
            Eigen::Matrix3d Rotate{Eigen::Matrix3d::Identity()};

        public:
            typedef std::shared_ptr<SmoothDifference> Ptr;
            SmoothDifference() = delete;
            inline double clip(double value, double min_val, double max_val)
            {
                return std::max(std::min(value, max_val), min_val);
            }
            SmoothDifference(const Config &conf) : Generalshape(conf, true) 
            {
                getTransform(Rotate, trans);
            }

    
            inline double getonlySDF(const Eigen::RowVector3d &P_rel)
            {
                Eigen::RowVector3d p_rel = (P_rel - trans) * Rotate;

                Eigen::Vector3d q = p_rel.cwiseAbs().transpose() - size / 2.0;
                double qx = std::max(q(0), 0.0);
                double qy = std::max(q(1), 0.0);
                double qz = std::max(q(2), 0.0);
                double box_sdf = std::sqrt(qx * qx + qy * qy + qz * qz) + std::min(std::max(q(0), std::max(q(1), q(2))), 0.0);
                double sphere_sdf = p_rel.norm() - radius;
                double h = clip(0.5 - 0.5 * (box_sdf + sphere_sdf) / 0.25, 0.0, 1.0);
                double m = box_sdf - (box_sdf + sphere_sdf) * h;
                return m + 0.25 * h * (1.0 - h);
            }
            // 子类实现
            inline double getonlySDF(const Eigen::RowVector3d &pos, const Eigen::Matrix3d &R_obj)
            {
                // Eigen::RowVector3d p_rel;
                Eigen::RowVector3d p_rel = (pos - trans) * Rotate * R_obj;
                Eigen::Vector3d q = p_rel.cwiseAbs().transpose() - size / 2.0;
                double qx = std::max(q(0), 0.0);
                double qy = std::max(q(1), 0.0);
                double qz = std::max(q(2), 0.0);
                double box_sdf = std::sqrt(qx * qx + qy * qy + qz * qz) + std::min(std::max(q(0), std::max(q(1), q(2))), 0.0);
                double sphere_sdf = p_rel.norm() - radius;
                double h = clip(0.5 - 0.5 * (box_sdf + sphere_sdf) / 0.25, 0.0, 1.0);
                double m = box_sdf - (box_sdf + sphere_sdf) * h;
                return m + 0.25 * h * (1.0 - h);
            }
            DEFINE_USEFUL_FUNCTION()
            ~SmoothDifference(){};
        };

        class SmoothIntersection : public Generalshape
        {
        private:
            const Eigen::Vector3d size{Eigen::Vector3d(3.0, 3.0, 0.5)}; // Box size
            const double radius{1.0};                                   // twist angle

            Eigen::RowVector3d trans{Eigen::RowVector3d::Zero()};
            Eigen::Matrix3d Rotate{Eigen::Matrix3d::Identity()};

        public:
            typedef std::shared_ptr<SmoothIntersection> Ptr;
            SmoothIntersection() = delete;
            inline double clip(double value, double min_val, double max_val)
            {
                return std::max(std::min(value, max_val), min_val);
            }
            SmoothIntersection(const Config &conf) : Generalshape(conf, true)
            {
                getTransform(Rotate, trans);
            }


            inline double getonlySDF(const Eigen::RowVector3d &P_rel)
            {
                Eigen::RowVector3d p_rel = (P_rel - trans) * Rotate;

                Eigen::Vector3d q = p_rel.cwiseAbs().transpose() - size / 2.0;
                double qx = std::max(q(0), 0.0);
                double qy = std::max(q(1), 0.0);
                double qz = std::max(q(2), 0.0);
                double box_sdf = std::sqrt(qx * qx + qy * qy + qz * qz) + std::min(std::max(q(0), std::max(q(1), q(2))), 0.0);
                double sphere_sdf = p_rel.norm() - radius;
                double h = clip(0.5 - 0.5 * (sphere_sdf - box_sdf) / 0.25, 0.0, 1.0);
                double m = sphere_sdf + (box_sdf - sphere_sdf) * h;
                return m + 0.25 * h * (1.0 - h);
            }
      
            inline double getonlySDF(const Eigen::RowVector3d &pos, const Eigen::Matrix3d &R_obj)
            {

                Eigen::RowVector3d p_rel = (pos - trans) * Rotate * R_obj;
                Eigen::Vector3d q = p_rel.cwiseAbs().transpose() - size / 2.0;
                double qx = std::max(q(0), 0.0);
                double qy = std::max(q(1), 0.0);
                double qz = std::max(q(2), 0.0);
                double box_sdf = std::sqrt(qx * qx + qy * qy + qz * qz) + std::min(std::max(q(0), std::max(q(1), q(2))), 0.0);
                double sphere_sdf = p_rel.norm() - radius;
                double h = clip(0.5 - 0.5 * (sphere_sdf - box_sdf) / 0.25, 0.0, 1.0);
                double m = sphere_sdf + (box_sdf - sphere_sdf) * h;
                return m + 0.25 * h * (1.0 - h);
            }
            DEFINE_USEFUL_FUNCTION()

            ~SmoothIntersection(){};
        };
        class SmoothIntersection_big : public Generalshape
        {
        private:
            const Eigen::Vector3d size{Eigen::Vector3d(9.0, 9.0, 1.5)}; // Box size
            const double radius{3.0};                                   // twist angle

            Eigen::RowVector3d trans{Eigen::RowVector3d::Zero()};
            Eigen::Matrix3d Rotate{Eigen::Matrix3d::Identity()};

        public:
            typedef std::shared_ptr<SmoothIntersection_big> Ptr;
            SmoothIntersection_big() = delete;
            inline double clip(double value, double min_val, double max_val)
            {
                return std::max(std::min(value, max_val), min_val);
            }
            SmoothIntersection_big(const Config &conf) : Generalshape(conf, true) 
            {
                getTransform(Rotate, trans);
            }


            inline double getonlySDF(const Eigen::RowVector3d &P_rel)
            {
                Eigen::RowVector3d p_rel = (P_rel - trans) * Rotate;
 
                Eigen::Vector3d q = p_rel.cwiseAbs().transpose() - size / 2.0;
                double qx = std::max(q(0), 0.0);
                double qy = std::max(q(1), 0.0);
                double qz = std::max(q(2), 0.0);
                double box_sdf = std::sqrt(qx * qx + qy * qy + qz * qz) + std::min(std::max(q(0), std::max(q(1), q(2))), 0.0);
                double sphere_sdf = p_rel.norm() - radius;
                double h = clip(0.5 - 0.5 * (sphere_sdf - box_sdf) / 0.25, 0.0, 1.0);
                double m = sphere_sdf + (box_sdf - sphere_sdf) * h;
                return m + 0.25 * h * (1.0 - h);
            }

            inline double getonlySDF(const Eigen::RowVector3d &pos, const Eigen::Matrix3d &R_obj)
            {
      
                Eigen::RowVector3d p_rel = (pos - trans) * Rotate * R_obj;
                Eigen::Vector3d q = p_rel.cwiseAbs().transpose() - size / 2.0;
                double qx = std::max(q(0), 0.0);
                double qy = std::max(q(1), 0.0);
                double qz = std::max(q(2), 0.0);
                double box_sdf = std::sqrt(qx * qx + qy * qy + qz * qz) + std::min(std::max(q(0), std::max(q(1), q(2))), 0.0);
                double sphere_sdf = p_rel.norm() - radius;
                double h = clip(0.5 - 0.5 * (sphere_sdf - box_sdf) / 0.25, 0.0, 1.0);
                double m = sphere_sdf + (box_sdf - sphere_sdf) * h;
                return m + 0.25 * h * (1.0 - h);
            }
            DEFINE_USEFUL_FUNCTION()

            ~SmoothIntersection_big(){};
        };

   
        class CSG : public Generalshape
        {
        private:
            Eigen::Vector3d X{Eigen::Vector3d(1, 0, 0)};
            Eigen::Vector3d Y{Eigen::Vector3d(0, 1, 0)};
            Eigen::Vector3d Z{Eigen::Vector3d(0, 0, 1)};
            double tubeRadius{1.5};  
            double torusRadius{3.0}; 
            Eigen::RowVector3d trans{Eigen::RowVector3d::Zero()};
            Eigen::Matrix3d Rotate{Eigen::Matrix3d::Identity()};

        public:
            typedef std::shared_ptr<CSG> Ptr;
            CSG() = delete;
            CSG(const Config &conf) : Generalshape(conf, true) 
            {
                getTransform(Rotate, trans);
            }

            inline double clip(double value, double min_val, double max_val)
            {
                return std::max(std::min(value, max_val), min_val);
            }
            inline Vector3d _perpendicular(const Vector3d &v)
            {
                if (v[1] == 0 && v[2] == 0)
                {
                    if (v[0] == 0)
                    {
                        throw std::invalid_argument("zero vector");
                    }
                    else
                    {
                        return v.cross(Vector3d(0, 1, 0));
                    }
                }
                return v.cross(Vector3d(1, 0, 0));
            }
            // basic shapes

            inline SDFFunction sphere(double radius, const Vector3d &center = Eigen::Vector3d::Zero())
            {
                auto f = [center, radius](const Vector3d &p) -> double
                {
                    return (p - center).norm() - radius;
                };

                return f;
            }

            inline SDFFunction capsule(const Eigen::Vector3d &a, const Eigen::Vector3d &b, double radius)
            {
                auto f = [a, b, radius, this](const Eigen::Vector3d &p) -> double
                {
                    Eigen::Vector3d pa = p - a;
                    Eigen::Vector3d ba = b - a;
                    double h = this->clip(pa.dot(ba) / ba.dot(ba), 0.0, 1.0);
                    Eigen::Vector3d pointOnLine = a + h * ba;
                    double distanceToLine = (pa - h * ba).norm();
                    return distanceToLine - radius;
                };
                return f;
            }

            inline SDFFunction box(const Eigen::Vector3d &size, const Eigen::Vector3d &center = Eigen::Vector3d::Zero())
            {
                auto f = [size, center](const Eigen::Vector3d &p) -> double
                {
                    Eigen::Vector3d q = (p - center).cwiseAbs() - size / 2.0;
                    double qx = std::max(q(0), 0.0);
                    double qy = std::max(q(1), 0.0);
                    double qz = std::max(q(2), 0.0);
                    return std::sqrt(qx * qx + qy * qy + qz * qz) + std::min(std::max(q(0), std::max(q(1), q(2))), 0.0);
                };
                return f;
            }

            inline SDFFunction rounded_box(const Eigen::Vector3d &size, double radius)
            {
                auto f = [size, radius](const Eigen::Vector3d &p) -> double
                {
                    Eigen::Vector3d q = p.cwiseAbs().array() - size.array() / 2 + radius;
                    Eigen::Vector3d maxQ = q.cwiseMax(Eigen::Vector3d::Zero());
                    double lengthQ = maxQ.norm();
                    double minQ = std::min({q.x(), q.y(), q.z(), 0.0});
                    return lengthQ + minQ - radius;
                };
                return f;
            }

            inline SDFFunction wireframe_box(const Eigen::Vector3d &size, double thickness)
            {

                auto g = [](double a, double b, double c) -> double
                {
                    Eigen::Vector3d v(a, b, c);
                    Eigen::Vector3d vMax = v.cwiseMax(Eigen::Vector3d::Zero());
                    return vMax.norm() + std::min(std::max(a, std::max(b, c)), 0.0);
                };

                auto f = [size, thickness, g](const Eigen::Vector3d &p) -> double
                {
                    Eigen::Vector3d pShifted = p.cwiseAbs().array() - size.array() / 2 - thickness / 2;
                    Eigen::Vector3d q = (p.array() + thickness / 2).cwiseAbs() - thickness / 2;
                    double px = pShifted.x();
                    double py = pShifted.y();
                    double pz = pShifted.z();
                    double qx = q.x();
                    double qy = q.y();
                    double qz = q.z();
                    return std::min({std::min(g(px, qy, qz), g(qx, py, qz)), g(qx, qy, pz)});
                };
                return f;
            }

            inline SDFFunction torus(const double r1, const double r2)
            {
                auto f = [r1, r2](const Eigen::Vector3d &p) -> double
                {
                    Eigen::Vector2d xy = p.head<2>();
                    double z = p.z();
                    double a = xy.norm() - r1;
                    double b = std::sqrt(a * a + z * z) - r2;
                    return b;
                };
                return f;
            }

            inline SDFFunction cylinder(const double radius)
            {
                auto f = [radius](const Eigen::Vector3d &p) -> double
                {
                    Eigen::Vector2d xy = p.head<2>();
                    double distance = (xy).norm() - radius;
                    return distance;
                };
                return f;
            }

            inline SDFFunction capped_cylinder(const Eigen::Vector3d &a, const Eigen::Vector3d &b, const double radius)
            {
                auto f = [a, b, radius](const Eigen::Vector3d &p) -> double
                {
                    Eigen::Vector3d ba = b - a;
                    Eigen::Vector3d pa = p - a;
                    double baba = ba.dot(ba);
                    double paba = pa.dot(ba);
                    double x = (pa * baba - ba * paba).norm() - radius * baba;
                    double y = std::abs(paba - (baba * 0.5)) - (baba * 0.5);
                    double x2 = x * x;
                    double y2 = y * y * baba;
                    double d = 0.0;
                    if (std::max(x, y) < 0)
                    {
                        d = -std::min(x2, y2);
                    }
                    else
                    {
                        d = (x > 0 ? x2 : 0) + (y > 0 ? y2 : 0);
                    }
                    d = std::copysign(std::sqrt(std::abs(d)) / baba, d);

                    return d;
                };
                return f;
            }

            inline SDFFunction rounded_cylinder(double ra, double rb, double h)
            {
                auto f = [ra, rb, h](const Eigen::Vector3d &p) -> double
                {
                    Eigen::Vector2d d(
                        (p.head<2>()).norm() - ra + rb,
                        std::abs(p.z()) - (h / 2) + rb);
                    double distance = std::min(std::max(d.x(), d.y()), 0.0) + (d.cwiseMax(Eigen::Vector2d::Zero())).norm() - rb;
                    return distance;
                };
                return f;
            }

            inline SDFFunction capped_cone(const Eigen::Vector3d &a, const Eigen::Vector3d &b, const double ra, const double rb)
            {
                auto f = [a, b, ra, rb, this](const Eigen::Vector3d &p) -> double
                {
                    double rba = rb - ra;
                    double baba = (b - a).dot(b - a);
                    double papa = (p - a).dot(p - a);
                    double paba = (p - a).dot(b - a) / baba;
                    double x = std::sqrt(papa - paba * paba * baba);
                    double cax = std::max(0.0, x - (paba < 0.5 ? ra : rb));
                    double cay = std::abs(paba - 0.5) - 0.5;
                    double k = rba * rba + baba;
                    double f = this->clip((rba * (x - ra) + paba * baba) / k, 0.0, 1.0);
                    double cbx = x - ra - f * rba;
                    double cby = paba - f;
                    double s = (cbx < 0 && cay < 0) ? -1 : 1;
                    double d = std::sqrt(std::min(cax * cax + cay * cay * baba, cbx * cbx + cby * cby * baba));
                    return s * std::sqrt(std::abs(d)) / std::abs(baba);
                };
                return f;
            }

            inline SDFFunction rounded_cone(const double r1, const double r2, const double h)
            {
                auto f = [r1, r2, h](const Eigen::Vector3d &p) -> double
                {
                    Eigen::Vector2d q = Eigen::Vector2d(std::sqrt(p.x() * p.x() + p.y() * p.y()), p.z());
                    double b = (r1 - r2) / h;
                    double a = std::sqrt(1.0 - b * b);
                    double k = -b * q.x() + a * q.y();
                    double c1 = q.norm() - r1;
                    double c2 = std::sqrt((q.x() - 0) * (q.x() - 0) + (q.y() - h) * (q.y() - h)) - r2;
                    double c3 = (a * q.x() + b * q.y()) - r1;
                    return (k < 0) ? c1 : ((k > a * h) ? c2 : c3);
                };
                return f;
            }

            inline SDFFunction ellipsoid(const Eigen::Vector3d &size)
            {
                auto f = [size](const Eigen::Vector3d &p) -> double
                {
                    double k0 = p.norm() / size.norm();
                    double k1 = p.norm() / (size.norm() * size.norm());
                    return k0 * (k0 - 1.0) / k1;
                };
                return f;
            }

            inline SDFFunction pyramid(double h)
            {
                return [h, this](const Vector3d &p) -> double
                {

                    Vector3d a;
                    a << std::abs(p.x()) - 0.5, std::abs(p.y()) - 0.5, 0.0;
                    bool w = a.y() > a.x();
                    if (w)
                    {
                        std::swap(a.x(), a.y());
                    }
                    double px = a.x();
                    double py = p.z();
                    double pz = a.y();
                    double m2 = h * h + 0.25;
                    double qx = pz;
                    double qy = h * py - 0.5 * px;
                    double qz = h * px + 0.5 * py;
                    double s = std::max(-qx, 0.0);
                    double t = this->clip((qy - 0.5 * pz) / (m2 + 0.25), 0.0, 1.0);
                    double aTerm = m2 * std::pow(qx + s, 2) + std::pow(qy, 2);
                    double bTerm = m2 * std::pow(qx + 0.5 * t, 2) + std::pow(qy - m2 * t, 2);
                    double d2 = (std::min(qy, -qx * m2 - qy * 0.5) > 0) ? 0 : std::min(aTerm, bTerm);
                    return std::sqrt((d2 + std::pow(qz, 2)) / m2) * std::copysign(1, std::max(qz, -py));
                };
            }

            inline SDFFunction tetrahedron(double r)
            {
                auto f = [r](const Eigen::Vector3d &p) -> double
                {
                    double x = p[0];
                    double y = p[1];
                    double z = p[2];
                    return (max(abs(x + y) - z, abs(x - y) + z) - r) / sqrt(3);
                };
                return f;
            }

            inline SDFFunction octahedron(double r)
            {
                auto f = [r](const Eigen::Vector3d &p) -> double
                {
                    return (p.cwiseAbs().sum() - r) * tan(M_PI / 6.0);
                };
                return f;
            }

            inline SDFFunction dodecahedron(double r)
            {
                Eigen::Vector3d x(1 + std::sqrt(5) / 2.0, 1, 0);
                x.normalize();
                auto f = [r, x](const Eigen::Vector3d &p) -> double
                {
                    Eigen::Vector3d p_norm = p.cwiseAbs() / r;
                    double a = p_norm.dot(x);
                    double b = p_norm.dot(Eigen::Vector3d(x.z(), x.x(), x.y()));
                    double c = p_norm.dot(Eigen::Vector3d(x.y(), x.z(), x.x()));
                    double q = (std::max(std::max(a, b), c) - x.x()) * r;
                    return q;
                };
                return f;
            }

            inline SDFFunction icosahedron(double r)
            {
                r *= 0.8506507174597755;
                Eigen::Vector3d x((3 + std::sqrt(5)) / 2, 1, 0);
                x.normalize();
                double w = sqrt(3.0) / 3.0;
                auto f = [r, x, w](const Eigen::Vector3d &p) -> double
                {
                    Eigen::Vector3d p_norm = p.cwiseAbs() / r;
                    double a = p_norm.dot(x);
                    double b = p_norm.dot(Eigen::Vector3d(x.z(), x.x(), x.y()));
                    double c = p_norm.dot(Eigen::Vector3d(x.y(), x.z(), x.x()));
                    double d = p_norm.dot(Eigen::Vector3d(w, w, w)) - x.x();
                    return std::max(std::max(std::max(a, b), c) - x.x(), d) * r;
                };
                return f;
            }

            inline SDFFunction translate(const SDFFunction &other, const Eigen::Vector3d &offset)
            {
                auto f = [other, offset](const Eigen::Vector3d &p) -> double
                {
                    return other(p - offset);
                };

                return f;
            }

            inline SDFFunction scale(const SDFFunction &other, const Eigen::Vector3d &factor)
            {
                double x = factor.x();
                double y = factor.y();
                double z = factor.z();
                double m = std::min(x, std::min(y, z));

                auto f = [other, factor, m](const Eigen::Vector3d &p) -> double
                {
                    return other(p.array() / factor.array()) * m;
                };

                return f;
            }

            inline SDFFunction rotate(const SDFFunction &other, double angle, const Eigen::Vector3d &vector = Eigen::Vector3d(0, 0, 1))
            {
                Eigen::Vector3d normalizedVector = vector.normalized();
                double x = normalizedVector.x();
                double y = normalizedVector.y();
                double z = normalizedVector.z();
                double s = std::sin(angle);
                double c = std::cos(angle);
                double m = 1 - c;

                Eigen::Matrix3d rotationMatrix;
                rotationMatrix << m * x * x + c, m * x * y + z * s, m * z * x - y * s,
                    m * x * y - z * s, m * y * y + c, m * y * z + x * s,
                    m * z * x + y * s, m * y * z - x * s, m * z * z + c;
                auto f = [other, rotationMatrix](const Eigen::Vector3d &p) -> double
                {
                    return other(rotationMatrix * p);
                };

                return f;
            }

            inline SDFFunction rotate_to(const SDFFunction &other, Eigen::Vector3d &a, Eigen::Vector3d &b)
            {
                a.normalize();
                b.normalize();
                double dot = b.dot(a);
                if (std::abs(dot - 1) < FLT_EPSILON)
                {
                    return other;
                }
                else if (std::abs(dot + 1) < FLT_EPSILON)
                {
                    return rotate(other, PI, _perpendicular(a));
                }
                double angle = std::acos(dot);
                Eigen::Vector3d v = b.cross(a);
                return rotate(other, angle, v);
            }

            inline SDFFunction unionOp(const SDFFunction &a, const std::vector<SDFFunction> &bs, double k = 0.0)
            {
                auto f = [a, bs, k, this](const Eigen::Vector3d &p) -> double
                {
                    double d1 = a(p);
                    for (const auto &b : bs)
                    {
                        double d2 = b(p);
                        if (k == 0.0)
                        {
                            d1 = std::min(d1, d2);
                        }
                        else
                        {
                            double h = this->clip(0.5 + 0.5 * (d2 - d1) / k, 0.0, 1.0);
                            double m = d2 + (d1 - d2) * h;
                            d1 = m - k * h * (1.0 - h);
                        }
                    }

                    return d1;
                };

                return f;
            }

            inline SDFFunction unionOp(const SDFFunction &a, const SDFFunction &b, double k = 0.0)
            {
                auto f = [a, b, k, this](const Eigen::Vector3d &p) -> double
                {
                    double d1 = a(p);
                    double d2 = b(p);
                    if (k == 0.0)
                    {
                        d1 = std::min(d1, d2);
                    }
                    else
                    {
                        double h = this->clip(0.5 + 0.5 * (d2 - d1) / k, 0.0, 1.0);
                        double m = d2 + (d1 - d2) * h;
                        d1 = m - k * h * (1.0 - h);
                    }

                    return d1;
                };

                return f;
            }

            inline SDFFunction differenceOp(const SDFFunction &a, const std::vector<SDFFunction> &bs, double k = 0.0)
            {
                auto f = [a, bs, k, this](const Eigen::Vector3d &p) -> double
                {
                    double d1 = a(p);
                    for (const auto &b : bs)
                    {

                        double d2 = b(p);
                        if (k == 0.0)
                        {
                            return std::max(d1, -d2);
                        }
                        else
                        {
                            double h = this->clip(0.5 - 0.5 * (d2 + d1) / k, 0.0, 1.0);
                            double m = d1 + (-d2 - d1) * h;
                            return m + k * h * (1.0 - h);
                        }
                    }
                };
                return f;
            }

            inline SDFFunction differenceOp(const SDFFunction &a, const SDFFunction &b, double k = 0.0)
            {
                auto f = [a, b, k, this](const Eigen::Vector3d &p) -> double
                {
                    double d1 = a(p);

                    double d2 = b(p);
                    if (k == 0.0)
                    {
                        return std::max(d1, -d2);
                    }
                    else
                    {
                        double h = this->clip(0.5 - 0.5 * (d2 + d1) / k, 0.0, 1.0);
                        double m = d1 + (-d2 - d1) * h;
                        return m + k * h * (1.0 - h);
                    }
                };
                return f;
            }

            inline SDFFunction intersectionOp(const SDFFunction &a, const std::vector<SDFFunction> &bs, double k = 0.0)
            {
                auto f = [a, bs, k, this](const Eigen::Vector3d &p) -> double
                {
                    double d1 = a(p);
                    for (const auto &b : bs)
                    {
                        double d2 = b(p);
                        if (k == 0.0)
                        {
                            return std::max(d1, d2);
                        }
                        else
                        {
                            double h = this->clip(0.5 - 0.5 * (d2 - d1) / k, 0.0, 1.0);
                            double m = d2 + (d1 - d2) * h;
                            return m + k * h * (1.0 - h);
                        }
                    }
                };
                return f;
            }

            inline SDFFunction intersectionOp(const SDFFunction &a, const SDFFunction &b, double k = 0.0)
            {
                auto f = [a, b, k, this](const Eigen::Vector3d &p) -> double
                {
                    double d1 = a(p);
                    double d2 = b(p);
                    if (k == 0.0)
                    {
                        return std::max(d1, d2);
                    }
                    else
                    {
                        double h = this->clip(0.5 - 0.5 * (d2 - d1) / k, 0.0, 1.0);
                        double m = d2 + (d1 - d2) * h;
                        return m + k * h * (1.0 - h);
                    }
                };
                return f;
            }

            inline SDFFunction twistOp(const SDFFunction &other, double k)
            {
                auto f = [other, k](const Eigen::Vector3d &p) -> double
                {
                    double x = p(0);
                    double y = p(1);
                    double z = p(2);
                    double c = cos(k * z);
                    double s = sin(k * z);
                    double x2 = c * x - s * y;
                    double y2 = s * x + c * y;
                    double z2 = z;
                    return other(Eigen::Vector3d(x2, y2, z2));
                };
                return f;
            }
            // bendOp
            inline SDFFunction bendOp(const SDFFunction &other, double k)
            {
                auto f = [other, k](const Eigen::Vector3d &p) -> double
                {
                    double x = p.x();
                    double y = p.y();
                    double z = p.z();
                    double c = std::cos(k * x);
                    double s = std::sin(k * x);
                    double x2 = c * x - s * y;
                    double y2 = s * x + c * y;
                    double z2 = z;
                    return other(Eigen::Vector3d(x2, y2, z2));
                };
                return f;
            }
            // blend操作
            inline SDFFunction blendOp(const SDFFunction &a, const std::vector<SDFFunction> &bs, double k = 0.5)
            {
                auto f = [a, bs, k](const Eigen::Vector3d &p) -> double
                {
                    double d1 = a(p);

                    for (const auto &b : bs)
                    {
                        double d2 = b(p);
                        d1 = k * d2 + (1.0 - k) * d1;
                    }

                    return d1;
                };

                return f;
            }
            // blend操作
            inline SDFFunction negateOp(const SDFFunction &other)
            {
                auto f = [other](const Eigen::Vector3d &p) -> double
                {
                    return -other(p);
                };
                return f;
            }
            // dilate操作
            inline SDFFunction dilateOp(const SDFFunction &other, double r)
            {
                auto f = [other, r](const Eigen::Vector3d &p) -> double
                {
                    return other(p) - r;
                };
                return f;
            }

            inline SDFFunction erodeOp(const SDFFunction &other, double r)
            {
                auto f = [other, r](const Eigen::Vector3d &p) -> double
                {
                    return other(p) + r;
                };
                return f;
            }

            inline SDFFunction shellOp(const SDFFunction &other, double thickness)
            {
                auto f = [other, thickness](const Eigen::Vector3d &p) -> double
                {
                    return std::abs(other(p)) - thickness / 2;
                };
                return f;
            }

            inline double getonlySDF(const Eigen::RowVector3d &pos_rel)
            {
                // CSG1=======================================================================
                auto f = intersectionOp(sphere(3), box(Eigen::Vector3d(4.5, 4.5, 4.5)));
                auto c = cylinder(1.5);
                auto c1 = rotate_to(c, X, X);
                auto c2 = rotate_to(c, X, Y);
                auto c3 = rotate_to(c, X, Z);
                auto c4 = unionOp(unionOp(c1, c2), c3);
                auto finial = differenceOp(f, c4);
                Eigen::RowVector3d P_rel = (pos_rel - trans) * Rotate;
                return finial(P_rel.transpose());
            }

            inline double getonlySDF(const Eigen::RowVector3d &pos, const Eigen::Matrix3d &R_obj)
            {

                auto f = intersectionOp(sphere(3), box(Eigen::Vector3d(4.5, 4.5, 4.5)));
                auto c = cylinder(1.5);
                auto c1 = rotate_to(c, X, X);
                auto c2 = rotate_to(c, X, Y);
                auto c3 = rotate_to(c, X, Z);
                auto c4 = unionOp(unionOp(c1, c2), c3);
                auto finial = differenceOp(f, c4);

                Eigen::RowVector3d pos_rel = (pos - trans) * Rotate * R_obj;
                return finial(pos_rel.transpose());
            }
            DEFINE_USEFUL_FUNCTION()

            ~CSG(){};
        };


        class Box : public Generalshape
        {
        private:
            double box_x, box_y, box_z; 
            Eigen::Vector3d selfbox;
            Eigen::RowVector3d trans{Eigen::RowVector3d::Zero()};
            Eigen::Matrix3d Rotate{Eigen::Matrix3d::Identity()};
            Eigen::AngleAxisd roll;
            Eigen::AngleAxisd pitch;
            Eigen::AngleAxisd yaw;

        public:
            typedef std::shared_ptr<Box> Ptr;
            Box() = delete;
            Box(const Config &conf) : Generalshape(conf, true)
            {
                getTransform(Rotate, trans);
                box_x = conf.box_x;
                box_y = conf.box_y;
                box_z = conf.box_z;
                std::cout << "Box bounds:(" << box_x << "," << box_y << "," << box_z << ")" << std::endl;
                selfbox = Eigen::Vector3d(box_x, box_y, box_z);
            }
            inline double getonlySDF(const Eigen::RowVector3d &pos_rel)
            {
                Eigen::RowVector3d P_rel = (pos_rel - trans) * Rotate; 
                Eigen::Vector3d q = P_rel.cwiseAbs().transpose() - selfbox;
                double qx = std::max(q(0), 0.0);
                double qy = std::max(q(1), 0.0);
                double qz = std::max(q(2), 0.0);
                return std::sqrt(qx * qx + qy * qy + qz * qz) + std::min(std::max(q(0), std::max(q(1), q(2))), 0.0);
            }

            inline double getonlySDF(const Eigen::RowVector3d &pos, const Eigen::Matrix3d &R_obj)
            {
                Eigen::RowVector3d pos_rel;
                pos_rel = (pos - trans) * Rotate * R_obj;
                Eigen::Vector3d q = pos_rel.cwiseAbs().transpose() - selfbox;
                double qx = std::max(q(0), 0.0);
                double qy = std::max(q(1), 0.0);
                double qz = std::max(q(2), 0.0);
                return std::sqrt(qx * qx + qy * qy + qz * qz) + std::min(std::max(q(0), std::max(q(1), q(2))), 0.0);
            }
            inline Eigen::Vector3d getonlyGrad1(const Eigen::RowVector3d &pos_rel)
            {
                double dx = 0.01;
                Eigen::RowVector3d temp = pos_rel;
                double sdfold = getonlySDF(pos_rel);
                temp(0) += dx;
                double gradx = getonlySDF(temp) - sdfold;
                temp = pos_rel;
                temp(1) += dx;
                double grady = getonlySDF(temp) - sdfold;
                temp = pos_rel;
                temp(2) += dx;
                double gradz = getonlySDF(temp) - sdfold;
                return Eigen::Vector3d(gradx, grady, gradz) / dx;
            } 
            inline double getSDFwithGrad1(const Eigen::RowVector3d &pos_rel, Eigen::Vector3d &grad)
            {
                grad = getonlyGrad1(pos_rel);
                Eigen::RowVector3d P_rel = (pos_rel - trans) * Rotate; 
                Eigen::Vector3d q = P_rel.cwiseAbs().transpose() - selfbox;
                double qx = std::max(q(0), 0.0);
                double qy = std::max(q(1), 0.0);
                double qz = std::max(q(2), 0.0);
                return std::sqrt(qx * qx + qy * qy + qz * qz) + std::min(std::max(q(0), std::max(q(1), q(2))), 0.0);

            } 

            ~Box(){};
        };
    }
#endif