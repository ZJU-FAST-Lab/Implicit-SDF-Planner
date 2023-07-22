#include "map_manager/PCSmap_manager.h"
#include <pcl/io/pcd_io.h>
#define RENDER_OCCUPANCY

#define INF 0x7fffffff
#define HUGE_NUMBER 999999999
constexpr uint8_t PCSmapManager::or_mask[8];
PCSmapManager::PCSmapManager(const Config &conf)
{
    recieved_globalmap = false;
    boundary_xyzmin = Vector3d(HUGE_NUMBER, HUGE_NUMBER, HUGE_NUMBER);
    boundary_xyzmax = Vector3d(-HUGE_NUMBER, -HUGE_NUMBER, -HUGE_NUMBER);
    occupancy_resolution = conf.occupancy_resolution;
    sta_threshold = conf.sta_threshold;
    debug_output = conf.debug_output;
}

PCSmapManager::~PCSmapManager()
{
    // release memory
    occupancy_map->releaseMemory();
}

void PCSmapManager::init(ros::NodeHandle &nh)
{


    occupancy_map.reset(new GridMap3D);
    occupancy_map->grid_resolution = occupancy_resolution;
    occupancy_map->debug_output    = debug_output;

    globalmap_sub = nh.subscribe("globalmap", 1, &PCSmapManager::rcvGlobalMapHandler, this);
    odometry_sub = nh.subscribe("odom", 1, &PCSmapManager::rcvOdomHandler, this);
    debug_grad_sub = nh.subscribe("/renderGrad", 1, &PCSmapManager::rcvRenderGrad, this);

    globalmap_vis_pub = nh.advertise<sensor_msgs::PointCloud2>("globalmap_vis", 10);
    gridmap_vis_pub = nh.advertise<sensor_msgs::PointCloud2>("gridmap_vis", 10);
    rcvmap_signal_pub = nh.advertise<std_msgs::Empty>("rcvmap_signal", 10);

    debug_grad_pub = nh.advertise<sensor_msgs::PointCloud2>("grad_vis", 10);
}

void PCSmapManager::rcvOdomHandler(const nav_msgs::Odometry odom)
{
}

void PCSmapManager::rcvRenderGrad(const std_msgs::Int16 msg)
{

    int layer_z = msg.data;
    if (layer_z < 0)
    {
        layer_z = 0;
    }
    if (layer_z >= occupancy_map->Z_size)
    {
        layer_z = occupancy_map->Z_size - 1;
    }

    sensor_msgs::PointCloud2 grad_vis;
    pcl::PointCloud<pcl::PointXYZI> grad_vis_pcl;
    pcl::PointXYZI pt;
    Vector3d pos;
    double z = layer_z * occupancy_resolution;
    double i;

    double resolution = 0.04;
    for (double x = boundary_xyzmin(0) + 0.5; x < boundary_xyzmax(0) - 0.5; x += resolution)
    {
        for (double y = boundary_xyzmin(1) + 0.5; y < boundary_xyzmax(1) - 0.5; y += resolution)
        {
            pos = Vector3d(x, y, z);
            pt.x = x;
            pt.y = y;
            pt.z = z; // + 0.01 * travelcost_map -> getSDFValue(pos);
            i = occupancy_map->getSDFValue(pos);
            pt.intensity = i;
            grad_vis_pcl.points.push_back(pt);
        }
    }

    pcl::toROSMsg(grad_vis_pcl, grad_vis);
    grad_vis.header.frame_id = "map";
    debug_grad_pub.publish(grad_vis);
}

void PCSmapManager::rcvGlobalMapHandler(const sensor_msgs::PointCloud2 &globalmap)
{
    if (recieved_globalmap == true)
    {
        return;
    }

    debug_publisher::DBSendNew("map_manager", "received global map");
    sensor_msgs::PointCloud2 globalmap_vis;
    pcl::PointCloud<pcl::PointXYZ> gridmap_vis;

    pcl::PointCloud<pcl::PointXYZ> global_cloud;
    pcl::fromROSMsg(globalmap, global_cloud);

    // EX STEP visualize global point cloud
    globalmap_vis = globalmap;
    globalmap_vis.header.frame_id = "map";
    globalmap_vis_pub.publish(globalmap_vis);

    // STEP1 generate occupancy map using point cloud
    global_pointcloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(global_cloud, *global_pointcloud); // 复制
    global_kdtree.setInputCloud(global_pointcloud);

    //measure boundary
    for(size_t i = 0 ; i < global_cloud.points.size(); i++)
    {
        if (global_cloud.points[i].x > boundary_xyzmax(0))
        {
            boundary_xyzmax(0) = global_cloud.points[i].x;
        }

        if (global_cloud.points[i].x < boundary_xyzmin(0))
        {
            boundary_xyzmin(0) = global_cloud.points[i].x;
        }

        if (global_cloud.points[i].y > boundary_xyzmax(1))
        {
            boundary_xyzmax(1) = global_cloud.points[i].y;
        }

        if (global_cloud.points[i].y < boundary_xyzmin(1))
        {
            boundary_xyzmin(1) = global_cloud.points[i].y;
        }

        if (global_cloud.points[i].z > boundary_xyzmax(2))
        {
            boundary_xyzmax(2) = global_cloud.points[i].z;
        }

        if (global_cloud.points[i].z < boundary_xyzmin(2))
        {
            boundary_xyzmin(2) = global_cloud.points[i].z;
        }
    }
    // malloc 3D grid map space
    occupancy_map->createGridMap(boundary_xyzmin, boundary_xyzmax);

    // generate occupancy grid map
    Vector3i index;
    Vector3d center_coord;
    pcl::PointXYZ  s_point;
    for(size_t i = 0 ; i < global_cloud.points.size(); i++)
    {
        index = occupancy_map->getGridIndex(Vector3d(global_cloud.points[i].x, global_cloud.points[i].y, global_cloud.points[i].z));
        occupancy_map->grid_map[occupancy_map->toAddr(index(0), index(1), index(2))]++;
    }

    for (int i = 0; i < occupancy_map->X_size; i++)
    {
        for (int j = 0; j < occupancy_map->Y_size; j++)
        {
            for (int k = 0; k < occupancy_map->Z_size; k++)
            {
                if (occupancy_map->grid_map[occupancy_map->toAddr(i, j, k)] >= sta_threshold)
                {

                    center_coord = occupancy_map->getGridCubeCenter(i, j, k);
#ifdef RENDER_OCCUPANCY
                    s_point.x = center_coord(0);
                    s_point.y = center_coord(1);
                    s_point.z = center_coord(2);
                    gridmap_vis.push_back(s_point);
#endif
                    occupancy_map->grid_map[occupancy_map->toAddr(i, j, k)] = 1;
                }
                else
                {
                    occupancy_map->grid_map[occupancy_map->toAddr(i, j, k)] = 0;
                }
            }
        }
    }

    ROS_INFO("[map_manager]generate occupancy map done!");

// EX STEP visualize occupancy map
#ifdef RENDER_OCCUPANCY
    sensor_msgs::PointCloud2 occupancy_map_vis;
    pcl::toROSMsg(gridmap_vis, occupancy_map_vis);
    occupancy_map_vis.header.frame_id = "map";
    gridmap_vis_pub.publish(occupancy_map_vis);
#endif


    // flag trig
    recieved_globalmap = true;
    std_msgs::Empty s;
    rcvmap_signal_pub.publish(s);

    debug_publisher::DBSendNew("map_manager", "global map init done");
}
