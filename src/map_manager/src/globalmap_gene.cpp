#include <ros/ros.h>
#include <Eigen/Eigen> 
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>
#include <math.h>
#include <ros/package.h>
#include <stdlib.h>
#include <time.h>

using namespace std;
int map_id;
ros::Publisher global_map_pub;
double cloud_resolution;
double init_x, init_y, init_z;

string pcd_file_name;
bool read_from_pcd; 
pcl::PointCloud<pcl::PointXYZ> global_map_pcl_cloud;

using namespace Eigen;


void geneWall(double ori_x , double ori_y , double length, double width,double height)
{
    pcl::PointXYZ  s_point;

    for( double t_z = 0.0; t_z < height ; t_z += cloud_resolution )
    {
        for( double t_y = ori_y; t_y < ori_y + width ; t_y += cloud_resolution )
        {
            for( double t_x = ori_x; t_x < ori_x + length; t_x += cloud_resolution)
            {
                s_point.x = t_x + (rand() % 10) / 250.0 ;
                s_point.y = t_y + (rand() % 10) / 250.0 ;
                s_point.z = t_z + (rand() % 10) / 800.0 ;

                global_map_pcl_cloud.push_back(s_point);
            }
        }
    }
}

void geneWall(double ori_x , double ori_y ,double ori_z, double length, double width,double height)
{
    pcl::PointXYZ  s_point;

    for( double t_z = ori_z  ; t_z < height +ori_z  ; t_z += cloud_resolution )
    {
        for( double t_y = ori_y; t_y < ori_y + width ; t_y += cloud_resolution )
        {
            for( double t_x = ori_x; t_x < ori_x + length; t_x += cloud_resolution)
            {
                s_point.x = t_x + (rand() % 10) / 250.0 ;
                s_point.y = t_y + (rand() % 10) / 250.0 ;
                s_point.z = t_z + (rand() % 10) / 800.0 ;

                global_map_pcl_cloud.push_back(s_point);
            }
        }
    }
}

void geneTrangle(double ori_x , double ori_y, double height, double depth, double length)
{
    pcl::PointXYZ s_point;
    for(double t_x = ori_x; t_x < ori_x + depth; t_x += cloud_resolution)
    {   
        for(double t_y = ori_y ;  t_y < ori_y + length; t_y += cloud_resolution)
        {
            for(double t_z = 0.0 ;  t_z < (length - t_y + ori_y)*height/length ; t_z += cloud_resolution / 3.0)
            {
                if( abs(t_x - ori_x) >= 0.2 && abs(t_x - ori_x - depth) >= 0.2  &&
                    abs(t_y - ori_y) >= 0.2 && abs(t_y - ori_y - length) >= 0.2 &&
                    abs(t_z - 0.0) >= 0.2 && abs(t_z - (length - t_y + ori_y)*height/length) >= 0.2){continue;}
                s_point.x = t_x + (rand() % 10) / 250.0 ;
                s_point.y = t_y + (rand() % 10) / 250.0 ;
                s_point.z = t_z + (rand() % 10) / 800.0 ;
                global_map_pcl_cloud.push_back(s_point);
            }
        }
    }
}

void geneTrangle(double ori_x , double ori_y, double ori_z, double height, double depth, double length)
{
    pcl::PointXYZ s_point;
    for(double t_x = ori_x; t_x < ori_x + depth; t_x += cloud_resolution)
    {   
        for(double t_y = ori_y ;  t_y < ori_y + length; t_y += cloud_resolution)
        {
            for(double t_z = ori_z ;  t_z < ori_z + (length - t_y + ori_y)*height/length ; t_z += cloud_resolution / 3.0)
            {
                if( abs(t_x - ori_x) >= 0.2 && abs(t_x - ori_x - depth) >= 0.2  &&
                    abs(t_y - ori_y) >= 0.2 && abs(t_y - ori_y - length) >= 0.2 &&
                    abs(t_z - 0.0) >= 0.2 && abs(t_z - ori_z - (length - t_y + ori_y)*height/length) >= 0.2){continue;}
                s_point.x = t_x + (rand() % 10) / 250.0 ;
                s_point.y = t_y + (rand() % 10) / 250.0 ;
                s_point.z = t_z + (rand() % 10) / 800.0 ;
                global_map_pcl_cloud.push_back(s_point);
            }
        }
    }
}

void geneSinPlane(double ori_x, double ori_y, double c_z , double end_x, double end_y, double t, double h)
{
    pcl::PointXYZ s_point;
    double z,dz;
    for(double t_x = ori_x; t_x < end_x; t_x += cloud_resolution)
    {   
        for(double t_y = ori_y ;  t_y < end_y; t_y += cloud_resolution)
        {   
            dz =  h * sin(t*t_x ) + h* sin(t*t_y);
            z  =  c_z + dz;
            if(z < c_z) { z = c_z;}

            s_point.x = t_x + (rand() % 10) / 250.0 ;
            s_point.y = t_y + (rand() % 10) / 250.0 ;
            s_point.z = z + (rand() % 10) / 800.0 ;
            global_map_pcl_cloud.push_back(s_point);
        }
    }
}

void geneRoad(Vector3d start_pt, Vector3d end_pt, double width){

    pcl::PointXYZ s_point;
    Vector3d dir = end_pt - start_pt;
    double length = dir.norm();
    double t_step = cloud_resolution/length;
    double k_step = cloud_resolution/width;
    Vector3d expand(-dir(1), dir(0), 0);
    expand.normalize();
    expand *= width;

    Vector3d paver_pos, paver_hand_pos;    
    for(double t = 0; t <= 1 ; t += t_step)
    {
        paver_pos = start_pt + t * dir;
        for(double k = -0.5; k <= 0.5 ; k += k_step)
        {
            paver_hand_pos = paver_pos + k*expand;
            s_point.x = paver_hand_pos(0) + (rand() % 10) / 250.0 ;
            s_point.y = paver_hand_pos(1) + (rand() % 10) / 250.0 ;
            s_point.z = paver_hand_pos(2) + (rand() % 10) / 800.0 ;
            global_map_pcl_cloud.push_back(s_point);
        }
    }



}

void geneSpiral3D(double center_x, double center_y, double ori_z, double end_z, double radius, double width, double t)
{
    pcl::PointXYZ s_point;
    double phi = 0;
    for(double t_z = ori_z; t_z < end_z; t_z += cloud_resolution / (6*t))
    {   
        phi = t * (t_z - ori_z);
        for(double w = radius ;  w < radius + width; w += cloud_resolution)
        {    
            s_point.x = center_x + w*sin(phi) + (rand() % 10) / 250.0 ;
            s_point.y = center_y + w*cos(phi) + (rand() % 10) / 250.0 ;
            s_point.z = t_z + (rand() % 10) / 800.0 ;
            global_map_pcl_cloud.push_back(s_point);
        }
    }
}

void map1Gene()
{
    geneWall(0,0, 0.2, 0.2, 3.0);
    geneWall(50,20, 15.0 , 0.2, 0.2, 3.0);

    geneWall(25.0, 0.0, 2.0, 10.0, 5.0);
    geneWall(25.0, 0.0, 7.0, 2.0, 10.0, 5.0);
    
}

void map2Gene()
{
    int tree_number = 20;
    double x,y;
    geneWall(0,0, 0.2, 0.2, 3.0);
    geneWall(100,100, 0.2, 0.2, 3.0);
    while(tree_number--)
    {
        x = (rand() % 3000) / 50.0 ;
        y = (rand() % 3000) / 50.0 ;
        if( sqrt((x - init_x)*(x - init_x) + (y - init_y)*(y - init_y)) < 0.09){
            tree_number++;
            continue;
        } 
        else{
            geneWall(x,y, 5, 5, 20);
        }
    }
}

void map3Gene()
{
    geneWall(0,0, 0.2, 0.2, 3.0);
    geneWall(50,50, 15.0 , 0.2, 0.2, 3.0);

    geneWall(10.0, 0.0, 2.0, 2.0, 14.0);
    geneWall(10.0, 10.0, 2.0, 2.0, 14.0);
    geneWall(10.0, 2.0, 2.0, 8.0, 3.0);
    geneWall(10.0, 2.0, 12.0, 2.0, 8.0, 2.0);
    geneWall(10.0, 5.0, 3.0, 2.0, 5.0, 5.5);
    geneWall(10.0, 10.0,  2.0, 40.0, 15.0);


    geneWall(20.0, 0.0, 2.0, 2.0, 14.0);
    geneWall(20.0, 10.0, 2.0, 2.0, 14.0);
    geneWall(20.0, 2.0, 2.0, 8.0, 5.0);
    geneWall(20.0, 2.0, 14.0, 2.0, 8.0, 0.0);
    geneWall(20.0, 5.0, 5.0, 2.0, 5.0, 5.5);
    geneWall(20.0, 10.0,  2.0, 40.0, 15.0);


    geneWall(10.0, 0.0, 13.0, 2.0, 50.0, 5.0);
    geneWall(20.0, 0.0, 13.0, 2.0, 50.0, 5.0);
}

void map10Gene()
{
    geneWall(0,0, 0.2, 0.2, 3.0);
    geneWall(50,50, 15.0 , 0.2, 0.2, 3.0);




    geneWall(10.0, 0.0, 2.0, 50.0, 35.0);
    geneWall(25.0, 0.0, 2.0, 50.0, 35.0);
    geneWall(40.0, 0.0, 2.0, 50.0, 35.0);

    geneWall(10.0, 49.0, 30.0, 1.0, 35.0);
                                         
    geneWall(0.0, 0.0, -1.0,50.0, 50.0, 1.0); 

}





void map4Gene()
{
    double x,y,z;
    double len, wid, hig;
    geneWall(0,0, 0.2, 0.2, 3.0);
    geneWall(60,60, 35.0 , 0.2, 0.2, 3.0);

    double num = 250;
    srand(time(NULL));
    pcl::PointXYZ s_point;
    for(int i = 0 ; i < num; i++)
    {
        x = (rand()%450 + 50)/10;
        y = (rand()%450 + 50)/10;
        z = (rand()%250 + 50)/10;
        // len = (rand()%20 + 3)/10;
        // wid = (rand()%20 + 3)/10;
        // hig = (rand()%20 + 3)/10;
        len = 1.5 *cloud_resolution;
        wid = 1.5 *cloud_resolution;
        hig = 1.5 *cloud_resolution;
        // s_point.x = x ;
        // s_point.y = y ;
        // s_point.z = z ;
        // global_map_pcl_cloud.push_back(s_point);

        geneWall(x,y,z,len,wid,hig);
    }
        
}

void map11Gene()
{
    double x,y,z;
    double len, wid, hig;
    geneWall(0,0, 0.2, 0.2, 3.0);
    geneWall(60,60, 35.0 , 0.2, 0.2, 3.0);

    double num = 300;
    srand(time(NULL));
    pcl::PointXYZ s_point;
    for(int i = 0 ; i < num; i++)
    {
        x = (rand()%450 + 15)/10;
        y = (rand()%450 + 15)/10;
        z = (rand()%250 + 50)/10;
        // len = (rand()%20 + 3)/10;
        // wid = (rand()%20 + 3)/10;
        // hig = (rand()%20 + 3)/10;
        len = 1.5 *cloud_resolution;
        wid = 1.5 *cloud_resolution;
        hig = 1.5 *cloud_resolution;
        // s_point.x = x ;
        // s_point.y = y ;
        // s_point.z = z ;
        // global_map_pcl_cloud.push_back(s_point);

        geneWall(x,y,z,len,wid,hig);
    }
        
}

void map5Gene()
{
    geneWall(0,0, 0.2, 0.2, 3.0);
    geneWall(60,60, 35.0 , 0.2, 0.2, 3.0);

    // geneWall(31,0,3.0, 10, 35.0);
    // geneWall(30,12,3.0, 10, 35.0);

    geneWall(30, 0,  2.0, 50, 15.0);
    geneWall(30, 0,  18.0, 2.0, 50, 16.0);
}

void map6Gene()
{

    geneWall(0,0, 0.2, 0.2, 3.0);
    geneWall(60,60, 35.0 , 0.2, 0.2, 3.0);
}

void map7Gene()
{

    geneWall(0,0, 0.2, 0.2, 3.0);
    geneWall(60,60, 35.0 , 0.2, 0.2, 3.0);

    geneWall(30,0 , 1.1, 60.0, 8.0);
    geneWall(30,0 , 5.0, 1.1, 25.0, 7.0);
    geneWall(30,35.0 , 5.0, 1.1, 25.0, 7.0);


    geneWall(30,0.0 , 12.0, 1.1, 21.0, 9.0);
    geneWall(30,39.0 , 12.0, 1.1, 21.0, 9.0);

    geneWall(30,0.0 , 21.0, 1.1, 60.0, 4.0);
    geneWall(30,28.0 , 14.0, 1.1, 4.0, 2.0);
    // geneWall(30,28.0 , 17.0, 1.1, 4.0, 1.0);

}

void map8Gene()
{

    geneWall(0,0, 0.2, 0.2, 3.0);
    geneWall(60,60, 35.0 , 0.2, 0.2, 3.0);

    pcl::PointXYZ  s_point;

    s_point.x = 30 ;
    s_point.y = 30 ;
    s_point.z = 2.0 ;
    global_map_pcl_cloud.push_back(s_point);
}

void map9Gene()
{

    geneWall(0,0, 0.2, 0.2, 3.0);
    geneWall(60,60, 35.0 , 0.2, 0.2, 3.0);

    Eigen::Vector3d spt(0,20,3);
    Eigen::Vector3d ept(50,20,30);
    // geneRoad(spt, ept, 1.0);

    // spt(2) += 4.0;
    // ept(2) += 4.0;
    // geneRoad(spt, ept, 1.0);

    for(double h = -60; h < 30; h += 0.5)
    {
        if(h > 0.0 && h < 5.5){ continue;}
        spt(0) = 0;
        spt(1) = 20;
        spt(2) = h;

        ept    = spt;
        ept(2) += 45;
        ept(0) = 50;

        if(h < 0)
        {
            spt(0) -= h;
            spt(2)  = 0;
            ept    = spt;
            ept(2) += 45;
            ept(0) = 50;
        }
        geneRoad(spt, ept, 0.5);
    }


}

void mapGene(int id)
{
    if(id == 1){map1Gene();}
    if(id == 2){map2Gene();}
    if(id == 3){map3Gene();}
    if(id == 4){map4Gene();}
    if(id == 5){map5Gene();}
    if(id == 6){map6Gene();}
    if(id == 7){map7Gene();}
    if(id == 8){map8Gene();}
    if(id == 9){map9Gene();}
    if(id == 10){map10Gene();}
    if(id == 11){map11Gene();}
}

void pubGlobalMap(int map_id)
{
    sensor_msgs::PointCloud2          global_map_cloud;

    mapGene(map_id);    

    pcl::toROSMsg(global_map_pcl_cloud, global_map_cloud);
    global_map_cloud.header.frame_id = "map";
    global_map_pub.publish(global_map_cloud);
    
    ROS_INFO("global map published! ");
}

void pubPCDMap()
{
    std::cout << "[GlobalMap gene]Read map from pcd file "<<std::endl;
    pcl::PCDReader reader;
    string file_name = ros::package::getPath(string("plan_manager"))+"/map_pcds/" + pcd_file_name;
    reader.read<pcl::PointXYZ>(file_name, global_map_pcl_cloud);

    sensor_msgs::PointCloud2          global_map_cloud;
    pcl::toROSMsg(global_map_pcl_cloud, global_map_cloud);
    global_map_cloud.header.frame_id = "map";
    global_map_pub.publish(global_map_cloud);

    ROS_INFO("global map published! ");
}

int main (int argc, char** argv) 
{ 
    ros::init(argc, argv, "globalmap_generator"); 
    ros::NodeHandle nh("~"); 

    nh.param("cloud_resolution", cloud_resolution, 0.8);
    nh.param("map_id", map_id, 2);
    nh.param("init_x", init_x, 1.0);
    nh.param("init_y", init_y, 1.0);
    nh.param("init_z", init_z, 1.0);

    nh.param("read_from_pcd", read_from_pcd, false);
    nh.param("pcd_file_name", pcd_file_name, std::string("scene1.pcd"));

    global_map_pub      = nh.advertise<sensor_msgs::PointCloud2>("globalmap", 5); 

    ros::Duration(2).sleep();

    int t = 3;

    while(t--)
    {   
        if(read_from_pcd){
            pubPCDMap();
        }
        else{
            pubGlobalMap(map_id);
        }
        ros::Duration(1).sleep();
    }
    ros::spin();
    return 0;
}