#ifndef __GRID_MAP_
#define __GRID_MAP_

#include <ros/ros.h>
#include <ros/package.h>
#include <Eigen/Eigen>
#include <string>
using namespace std;
using namespace Eigen;
class GridMap3D
{
    public:
        /** 
         * 清空珊格地图
        */
        void clearGridMap();

        /** 
         * 释放珊格地图申请的内存
        */
        void releaseMemory();

        /** 
         * 申请地图内存
         * 
         * @param boundary_xyzmin 点云中x.y.z坐标值最小的点的坐标
         * @param boundary_xyzmax 点云中x.y.z坐标值最大的点的坐标
         * @return void
        */
        void createGridMap(const Vector3d& boundary_xyzmin, const Vector3d& boundary_xyzmax);

        /** 
         * 判断点是否在地图内部
         * 
         * @param pos_w 查询点的x.y.z坐标
         * @return 如果查询点在地图内部，返回True
        */
        bool isInMap(const Vector3d& pos_w)const;

        /** 
         * 判断珊格的索引是否越界
         * 
         * @param index 三维珊格索引
         * @return 如果查询点在地图内部，返回True
        */
        bool isIndexValid(const Vector3i& index)const;
        bool isIndexValid(const int ix,const int iy,const int iz)const;

        /** 
         * 判断某一个位置是否被占据
         * 
         * @param pos_w 三维位置点
         * @return 如果查询点被占据，返回True
        */
        bool isCoordOccupied(const Vector3d& pos_w);
        bool isIndexOccupied(int ix, int iy, int iz);
        bool isIndexOccupied(const Vector3i& index);

        /** 
         * 判断某一个位置在地图膨胀后是否被占据
         * 
         * @param index 三维珊格索引
         * @param flate_pix 膨胀的格子数
         * @return 如果查询点被占据，返回True
        */
        bool isIndexOccupiedFlate(const Vector3i& index, const int flate_pix);

        /** 
         * 获得某个位置所在的珊格索引
         * 
         * @param pos_w 三维位置点
         * @return 返回pos_w所在的三维珊格索引
        */
        Vector3i getGridIndex(const Vector3d& pos_w);

        /** 
         * 获得某个珊格立方体中心坐标
         * 
         * @param index 三维珊格索引
         * @return 返回index所在的三维珊格立方体中心坐标
        */
        Vector3d getGridCubeCenter(const Vector3i& index);
        Vector3d getGridCubeCenter(int ix, int iy, int iz);

        int getVoxelNum(int dim);
        bool isGridBoundOfObs(const Vector3i& index); //return true while the grid is on the boundary of a obstacle.

        /** 
         * 利用占据珊格地图生成ESDF地图
        */
        void generateESDF3d();

        //to generate ESDF map from occupancy map
        void clearGridESDF();
        template <typename F_get_val, typename F_set_val>
        void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim);


        /** 
         * 获得某个珊格的SDF值
         * 
         * @param index 三维珊格索引
         * @return SDF的值
        */
        double getGridSDFValue(const Vector3i& index);
        double getGridSDFValue(int ix, int iy, int iz);

        /** 
         * 利用三线性插值，获得某个位置的SDF值
         * 
         * @param pos_w 三维位置点
         * @return SDF的值
        */
        inline double getSDFValue(const Vector3d& pos_w)
        {
            /* use trilinear interpolation */
            Vector3d pos_m = pos_w - 0.5 * grid_resolution * Vector3d::Ones();
            Vector3i idx   = getGridIndex(pos_m);
            Vector3d idx_pos, diff;
            idx_pos        = getGridCubeCenter(idx);
            diff           = (pos_w - idx_pos) * (1.0/grid_resolution);

            double values[2][2][2];
            Vector3i current_idx;
            for (int x = 0; x < 2; x++) {
                for (int y = 0; y < 2; y++) {
                    for (int z = 0; z < 2; z++) {
                        current_idx = idx + Vector3i(x, y, z);
                        if (!isInMap( getGridCubeCenter(current_idx) )) {
                            values[x][y][z] = 0;
                        }
                        values[x][y][z] = getGridSDFValue(current_idx);
                    }
                }
            }

            double v00 = (1 - diff[0]) * values[0][0][0] + diff[0] * values[1][0][0];
            double v01 = (1 - diff[0]) * values[0][0][1] + diff[0] * values[1][0][1];
            double v10 = (1 - diff[0]) * values[0][1][0] + diff[0] * values[1][1][0];
            double v11 = (1 - diff[0]) * values[0][1][1] + diff[0] * values[1][1][1];
            double v0 = (1 - diff[1]) * v00 + diff[1] * v10;
            double v1 = (1 - diff[1]) * v01 + diff[1] * v11;
            double dist = (1 - diff[2]) * v0 + diff[2] * v1;

            return dist;
        }

        /** 
         * 利用三线性插值，获得某个位置的SDF值，以及对应的SDF梯度
         * 
         * @param pos_w 三维位置点
         * @param grad 引用格式，接收SDF的梯度
         * @return SDF的值
        */
        inline double getSDFValueWithGrad(const Vector3d& pos_w, Vector3d& grad)
        {
            /* use trilinear interpolation */
            Vector3d pos_m = pos_w - 0.5 * grid_resolution * Vector3d::Ones();
            Vector3i idx   = getGridIndex(pos_m);
            Vector3d idx_pos, diff;
            idx_pos        = getGridCubeCenter(idx);
            diff           = (pos_w - idx_pos) * (1.0/grid_resolution);

            double values[2][2][2];
            Vector3i current_idx;
            for (int x = 0; x < 2; x++) {
                for (int y = 0; y < 2; y++) {
                    for (int z = 0; z < 2; z++) {
                        current_idx = idx + Vector3i(x, y, z);
                        if (!isInMap( getGridCubeCenter(current_idx) )) {
                            values[x][y][z] = 0;
                        }
                        values[x][y][z] = getGridSDFValue(current_idx);
                    }
                }
            }
            double v00 = (1 - diff[0]) * values[0][0][0] + diff[0] * values[1][0][0];
            double v01 = (1 - diff[0]) * values[0][0][1] + diff[0] * values[1][0][1];
            double v10 = (1 - diff[0]) * values[0][1][0] + diff[0] * values[1][1][0];
            double v11 = (1 - diff[0]) * values[0][1][1] + diff[0] * values[1][1][1];
            double v0 = (1 - diff[1]) * v00 + diff[1] * v10;
            double v1 = (1 - diff[1]) * v01 + diff[1] * v11;
            double dist = (1 - diff[2]) * v0 + diff[2] * v1;

            grad[2] = (v1 - v0) * (1.0/grid_resolution);
            grad[1] = ((1 - diff[2]) * (v10 - v00) + diff[2] * (v11 - v01)) * (1.0/grid_resolution);
            grad[0] = (1 - diff[2]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]);
            grad[0] += (1 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]);
            grad[0] += diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]);
            grad[0] += diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]);
            grad[0] *= (1.0/grid_resolution);
            return dist;
        }
    inline int toAddr(const int idx,const int idy,const int idz) 
    { return idx * Y_size * Z_size + idy * Z_size + idz; }

    public:
        // max index size
        int X_size;
        int Y_size;
        int Z_size;
        int totalsize;//X_size*Y_size*Z_size
        // map size box
        Vector3d boundary_xyzmin;
        Vector3d boundary_xyzmax;

        // map resolution
        double grid_resolution;

        // debug switch
        bool debug_output;
    
        // occupancy map and its ESDF map

        double* grid_map;
        double* grid_esdf;

        // for esdf map generation, temp buffer
        double* grid_map_buffer_neg; 
        double* grid_map_buffer_all; 
        double* grid_esdf_buffer1; 
        double* grid_esdf_buffer2; 

        
        // a flag for custom usage.
        bool* grid_map_flags;

    
    public:
        typedef shared_ptr<GridMap3D> Ptr;

};

#endif