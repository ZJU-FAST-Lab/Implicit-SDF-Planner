#ifndef PCSMAP_MANAGER_H
#define PCSMAP_MANAGER_H

#include <string.h>
#include <ros/ros.h>
#include <unordered_map>
#include <ros/package.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <utils/config.hpp>
#include <utils/debug_publisher.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include "map_manager/GridMap3D.h"



class PCSmapManager
{
    public:
        PCSmapManager(const Config&conf);
        ~PCSmapManager();
        static constexpr uint8_t or_mask[8]={0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01};
        // param load
        void init(ros::NodeHandle& nh);

        // callback functions
        void rcvGlobalMapHandler(const sensor_msgs::PointCloud2& globalmap);
        void rcvOdomHandler(const nav_msgs::Odometry odom);
        void rcvRenderGrad(const std_msgs::Int16 msg);

        /**
         * 生成map_kernel
         * @pre 必须已经初始化完成地图
         * @param kernel_size kernel的大小
        */
       inline uint8_t* generateMapKernel(const int& kernel_size)
       {
            
            int side_size = (kernel_size - 1)/2;
            int x_size    = occupancy_map -> X_size + 2 * side_size;
            int y_size    = occupancy_map -> Y_size + 2 * side_size;
            int z_size    = occupancy_map -> Z_size + 2 * side_size;
            int bytes_len_of_last_dim = (z_size + 7) / 8;
            int size_yz   = y_size * bytes_len_of_last_dim;
            int bytes_len = x_size * y_size * bytes_len_of_last_dim;
            uint8_t *map_kernel = new uint8_t[bytes_len]();
            for (int x = 0; x < occupancy_map -> X_size; x++)
            { 
                for (int y = 0; y < occupancy_map -> Y_size; y++)
                { 
                    for (int z = 0; z < occupancy_map -> Z_size; z++)
                    { 
                        int flate_x  = x + side_size;
                        int flate_y  = y + side_size;
                        int flate_z  = z + side_size;

                        int byte_idx = flate_x * size_yz + flate_y * bytes_len_of_last_dim + (flate_z + 0) / 8;
                        int byte_offset = flate_z % 8;
                        if ( occupancy_map -> isIndexOccupied(x,y,z) == true)
                        {
                            map_kernel[byte_idx] |= or_mask[byte_offset];
                        }
                    }
                }
            }
            mk = map_kernel;
            return map_kernel;
       }

        /**
         * 清空地图
         * 1.释放珊格地图内存
         * 2.重新接收地图
         */
        inline void clearMap() {
            if( recieved_globalmap == true ){
                occupancy_map -> releaseMemory();
                recieved_globalmap = false;
            }
        }


        /**
         * 获得索引的唯一ID
         * 实际上是获得三维索引的一维地址
         * @param i 三维索引第一位
         * @param j 三维索引第二位
         * @param k 三维索引第三位
         * @return 唯一ID
         */
        inline int unifiedID(const int& i, const int& j, const int& k) const
        {
            int unified_id = 0;
            unified_id += k * (occupancy_map->X_size) * (occupancy_map->Y_size);
            unified_id += j * (occupancy_map->X_size) ;
            unified_id += i;
            return unified_id;
        }

        /**
         * 获得某个位置所在珊格的唯一ID
         * 实际上是获得三维索引的一维地址
         * @param p 位置点
         * @return 唯一ID
         */
        inline int unifiedID(const Eigen::Vector3d& p) const
        {
            Eigen::Vector3i index = occupancy_map -> getGridIndex(p);
            int unified_id = 0;
            unified_id += index(2) * (occupancy_map->X_size) * (occupancy_map->Y_size);
            unified_id += index(1) * (occupancy_map->X_size) ;
            unified_id += index(0);
            return unified_id;
        }

        /**
         * 将位置限制在地图内部
         * @param pt 位置点,引用格式
         */
        inline void projInMap(Eigen::Vector3d& pt) const{
            if( pt(0) < boundary_xyzmin(0) ){ pt(0) = boundary_xyzmin(0);}
            if( pt(1) < boundary_xyzmin(1) ){ pt(1) = boundary_xyzmin(1);}
            if( pt(2) < boundary_xyzmin(2) ){ pt(2) = boundary_xyzmin(2);}
            if( pt(0) > boundary_xyzmax(0) ){ pt(0) = boundary_xyzmax(0);}
            if( pt(1) > boundary_xyzmax(1) ){ pt(1) = boundary_xyzmax(1);}
            if( pt(2) > boundary_xyzmax(2) ){ pt(2) = boundary_xyzmax(2);}
        }
        
        /**
         * 获得某个点附近的被占据栅格
         * 具体地，会将被占据栅格立方体中心的坐标放入points内
         * @param center AABB框的中心位置(不是索引，是位置)
         * @param dx AABB框x方向边长的一半
         * @param dy AABB框y方向边长的一半
         * @param dz AABB框z方向边长的一半
         * @param points 引用格式，用于接收结果
         */
        inline void getPointsInAABB(const Eigen::Vector3d &center, double halfbdx,double halfbdy,double halfbdz, std::vector<Eigen::Vector3d>& ob_pts) const
        {
            Eigen::Vector3d corner1 = center - Eigen::Vector3d(halfbdx, halfbdy, halfbdz);
            Eigen::Vector3d corner2 = center + Eigen::Vector3d(halfbdx, halfbdy, halfbdz);
            projInMap(corner1);
            projInMap(corner2);
            const Eigen::Vector3i idcorner1 = occupancy_map -> getGridIndex( corner1 );
            const Eigen::Vector3i idcorner2 = occupancy_map -> getGridIndex( corner2 );

            for (int i = idcorner1(0); i <= idcorner2(0); i++)
            {
                for (int j = idcorner1(1); j <= idcorner2(1); j++)
                {
                    for (int k = idcorner1(2); k <= idcorner2(2); k++)
                    {
                        if( occupancy_map -> isIndexOccupied(i,j,k) )
                        {
                            ob_pts.emplace_back( occupancy_map->getGridCubeCenter(i,j,k) ); 
                        }
                    }
                }
            }
        }

        /**
         * 获得某个点附近的被占据栅格,但是它又不在上一个AABB内
         * 具体地，会将被占据栅格立方体中心的坐标放入points内
         * @param center AABB框的中心位置(不是索引，是位置)
         * @param center_last 上一个AABB框的中心位置(不是索引，是位置)
         * @param halfbdx AABB框x方向边长的一半
         * @param halfbdy AABB框y方向边长的一半
         * @param halfbdz AABB框z方向边长的一半
         * @param points 引用格式，用于接收结果
         */
        inline void getPointsInAABBOutOfLastOne(const Eigen::Vector3d &center, const Eigen::Vector3d &center_last, double halfbdx,double halfbdy,double halfbdz,const Eigen::Vector3d offset=Eigen::Vector3d::Zero())
        {
            Eigen::Vector3d corner1 = center - Eigen::Vector3d(halfbdx, halfbdy, halfbdz)+offset;
            Eigen::Vector3d corner2 = center + Eigen::Vector3d(halfbdx, halfbdy, halfbdz)+offset;
            projInMap(corner1);
            projInMap(corner2);
            const Eigen::Vector3i idcorner1 = occupancy_map -> getGridIndex( corner1 );
            const Eigen::Vector3i idcorner2 = occupancy_map -> getGridIndex( corner2 );

            Eigen::Vector3d corner1_l = center_last - Eigen::Vector3d(halfbdx, halfbdy, halfbdz);
            Eigen::Vector3d corner2_l = center_last + Eigen::Vector3d(halfbdx, halfbdy, halfbdz);
            projInMap(corner1_l);
            projInMap(corner2_l);
            const Eigen::Vector3i idcorner1_l = occupancy_map -> getGridIndex( corner1_l );
            const Eigen::Vector3i idcorner2_l = occupancy_map -> getGridIndex( corner2_l );

            for (int i = idcorner1(0); i <= idcorner2(0); i++)
            {
                for (int j = idcorner1(1); j <= idcorner2(1); j++)
                {
                    for (int k = idcorner1(2); k <= idcorner2(2); k++)
                    {   
                        if( i > idcorner2_l(0) || i < idcorner1_l(0) ||
                            j > idcorner2_l(1) || j < idcorner1_l(1) ||
                            k > idcorner2_l(2) || k < idcorner1_l(2) )
                        {
                            if( occupancy_map -> isIndexOccupied(i,j,k) )
                            {
                                aabb_points.emplace( unifiedID(i,j,k), occupancy_map -> getGridCubeCenter(i,j,k) );
                            }
                        }
                    }
                }
            }
        }

        // map size box
        Vector3d boundary_xyzmin;
        Vector3d boundary_xyzmax;

        // map resolutions
        double occupancy_resolution;

        // some params
        // point count threshold while generating occupancy grid map using point cloud
        int sta_threshold;
        // map received flag
        bool recieved_globalmap;
        // debug output switch
        bool debug_output;
        
        // global 3D occupancy grid map
        GridMap3D::Ptr occupancy_map;

        uint8_t *mk;

        std::unordered_map<int, Eigen::Vector3d> aabb_points;


    private:

        pcl::PointCloud<pcl::PointXYZ>::Ptr global_pointcloud;
        pcl::KdTreeFLANN<pcl::PointXYZ> global_kdtree;

        // ros
        ros::Subscriber globalmap_sub;
        ros::Subscriber odometry_sub;
        ros::Subscriber debug_grad_sub;

        ros::Publisher  globalmap_vis_pub;
        ros::Publisher  gridmap_vis_pub;
        ros::Publisher  rcvmap_signal_pub;
        ros::Publisher  debug_grad_pub;
        
     public:
         typedef shared_ptr<PCSmapManager> Ptr;
};

#endif