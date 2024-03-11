#ifndef VIS_HPP
#define VIS_HPP

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <iostream>
#include <unordered_map>
#include <pcl_conversions/pcl_conversions.h>
// for visualization geometry


#include <utils/trajectory.hpp>
#include <string>
#include <utils/se3_state.hpp>


#define TRAJ_ORDER 5
#define Delayforvis(pub,topic) \
    do { \
        double wait_time = 0.0; \
        while (pub.getNumSubscribers() < 1) { \
            ros::Duration(0.1).sleep(); \
            wait_time += 0.1; \
            if (wait_time > 0.5) { \
                std::cout << "Looks like Topic " << topic << " is not subscribed by any subscriber. Check rviz config."<<std::endl; \
                break; \
            } \
        } \
    } while(0);

// for visualization geometry
using namespace Eigen;
using namespace std;
namespace vis
{
    // 可视化球类
    struct BALL
    {
        Eigen::Vector3d center;
        double radius;
        BALL(const Eigen::Vector3d &c, double r) : center(c), radius(r){};
        BALL(){};
    };
    // 可视化椭球类
    struct ELLIPSOID
    {
        Eigen::Vector3d c;
        double rx, ry, rz;
        Eigen::Matrix3d R;
        ELLIPSOID(const Eigen::Vector3d &center, const Eigen::Vector3d &r, const Eigen::Matrix3d &rot)
            : c(center), rx(r.x()), ry(r.y()), rz(r.z()), R(rot){};
        ELLIPSOID(){};
    };

    using PublisherMap = std::unordered_map<std::string, ros::Publisher>; // 存储发布的话题与ros发布者的键值对
                                                                          // 可视化颜色分类
    enum Color
    {
        white,
        red,
        green,
        blue,
        light_blue,
        yellow,
        chartreuse,
        black,
        gray,
        orange,
        purple,
        pink,
        steelblue
    };


    class Visualization
    {
    private:
        ros::NodeHandle nh_;
        PublisherMap publisher_map_; // 存储发布的话题与ros发布者的键值对
        // 根据颜色设定可视化，透明度，存入marker
        inline void setMarkerColor(visualization_msgs::Marker &marker,
                                   Color color = blue,
                                   double a = 1)
        {
            marker.color.a = a;
            switch (color)
            {
            case white:
                marker.color.r = 1;
                marker.color.g = 1;
                marker.color.b = 1;
                break;
            case red:
                marker.color.r = 1;
                marker.color.g = 0;
                marker.color.b = 0;
                break;
            case green:
                marker.color.r = 0;
                marker.color.g = 1;
                marker.color.b = 0;
                break;
            case blue:
                marker.color.r = 0;
                marker.color.g = 0;
                marker.color.b = 1;
                break;
            case light_blue:
                marker.color.r = 0.4;
                marker.color.g = 0.6;
                marker.color.b = 1;
                break;
            case yellow:
                marker.color.r = 1;
                marker.color.g = 1;
                marker.color.b = 0;
                break;
            case chartreuse:
                marker.color.r = 0.5;
                marker.color.g = 1;
                marker.color.b = 0;
                break;
            case black:
                marker.color.r = 0;
                marker.color.g = 0;
                marker.color.b = 0;
                break;
            case gray:
                marker.color.r = 0.5;
                marker.color.g = 0.5;
                marker.color.b = 0.5;
                break;
            case orange:
                marker.color.r = 1;
                marker.color.g = 0.5;
                marker.color.b = 0;
                break;
            case purple:
                marker.color.r = 0.5;
                marker.color.g = 0;
                marker.color.b = 1;
                break;
            case pink:
                marker.color.r = 1;
                marker.color.g = 0;
                marker.color.b = 0.6;
                break;
            case steelblue:
                marker.color.r = 0.4;
                marker.color.g = 0.7;
                marker.color.b = 1;
                break;
            }
        }

        // 根据rgb，透明度，存入marker
        inline void setMarkerColor(visualization_msgs::Marker &marker,
                                   double a,
                                   double r,
                                   double g,
                                   double b)
        {
            marker.color.a = a;
            marker.color.r = r;
            marker.color.g = g;
            marker.color.b = b;
        }

        inline void setMarkerScale(visualization_msgs::Marker &marker,
                                   const double &x,
                                   const double &y,
                                   const double &z)
        {
            marker.scale.x = x;
            marker.scale.y = y;
            marker.scale.z = z;
        }
        // 导入xyz位置，姿态默认水平
        inline void setMarkerPose(visualization_msgs::Marker &marker,
                                  const double &x,
                                  const double &y,
                                  const double &z)
        {
            marker.pose.position.x = x;
            marker.pose.position.y = y;
            marker.pose.position.z = z;
            marker.pose.orientation.w = 1;
            marker.pose.orientation.x = 0;
            marker.pose.orientation.y = 0;
            marker.pose.orientation.z = 0;
        }
        // 根据旋转信息，xyz坐标设定位置与姿态
        template <class ROTATION>
        inline void setMarkerPose(visualization_msgs::Marker &marker,
                                  const double &x,
                                  const double &y,
                                  const double &z,
                                  const ROTATION &R)
        {
            marker.pose.position.x = x;
            marker.pose.position.y = y;
            marker.pose.position.z = z;
            Eigen::Quaterniond r(R);
            marker.pose.orientation.w = r.w();
            marker.pose.orientation.x = r.x();
            marker.pose.orientation.y = r.y();
            marker.pose.orientation.z = r.z();
        }

    public:
        Visualization() = delete;
        Visualization(ros::NodeHandle &nh) : nh_(nh) {}
        typedef std::shared_ptr<Visualization> Ptr;

        /**
       

        /**
         * 发布Float64消息
         * 
         * @param topic 话题名
         * @param msg   要发布的内容
         */
        template <class TOPIC>
        inline void pubFloat64(const TOPIC &topic, const double msg)
        {
            auto got = publisher_map_.find(topic); // 如果unordered_map中没有存储这个键值对，即插入这个对
            if (got == publisher_map_.end())
            {
                ros::Publisher pub = nh_.advertise<std_msgs::Float64>(topic, 10000);
                publisher_map_[topic] = pub;
            }
            std_msgs::Float64 datamsg;
            datamsg.data = msg;
            publisher_map_[topic].publish(datamsg);
        }


        /**
         * 可视化一个球
         * 
         * @param c     球心坐标
         * @param r     半径
         * @param topic 话题名
         * @param color 颜色
         * @param a     不透明度(0~1)
         * @param keep  保留之前的绘制内容
         */
        template <class CENTER, class TOPIC>
        inline void visABall(const CENTER &c,
                             const double &r,
                             const TOPIC &topic,
                             const Color color = blue,
                             const double a = 1,
                             const bool keep = false)
        {
            auto got = publisher_map_.find(topic); // 如果unordered_map中没有存储这个键值对，即插入这个对
            if (got == publisher_map_.end())
            {
                ros::Publisher pub = nh_.advertise<visualization_msgs::Marker>(topic, 10000);
                publisher_map_[topic] = pub;
                Delayforvis(pub,topic)
            }
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            setMarkerColor(marker, color, a);
            setMarkerScale(marker, 2 * r, 2 * r, 2 * r);
            setMarkerPose(marker, c[0], c[1], c[2]);
            marker.header.stamp = ros::Time::now();
            if (keep)
            {
                static int i = 0;
                marker.id = i;
                i++;
            }
            publisher_map_[topic].publish(marker);
        }

        /**
         * 可视化一个cube,可以给定Marker的id
         * 
         * @param c     球心坐标
         * @param r     半径
         * @param topic 话题名
         * @param color 颜色
         * @param a     不透明度(0~1)
         * @param id    指定id
         */
        template <class CENTER, class TOPIC>
        inline void visABoxWithId(const CENTER &center,
                                     const CENTER &size,
                                     const TOPIC &topic,
                                     const Color color = Color::yellow,
                                     const double a = 0.4,
                                     const int id = 1)
        {
            auto got = publisher_map_.find(topic); // 如果unordered_map中没有存储这个键值对，即插入这个对
            if (got == publisher_map_.end())
            {
                ros::Publisher pub = nh_.advertise<visualization_msgs::Marker>(topic, 10000);
                publisher_map_[topic] = pub;
                 Delayforvis(pub,topic)
            }
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            setMarkerColor(marker, color, a);
            setMarkerScale(marker, size[0], size[1], size[2]);
            setMarkerPose(marker, center[0], center[1], center[2]);
            marker.header.stamp = ros::Time::now();
            marker.id = id;
            publisher_map_[topic].publish(marker);
        }


        /**
         * 可视化点云,但是传入的是vector<Vector3d>
         * 
         * @param pc    要可视化的点云
         * @param topic 话题名
         */
        template <class PC, class TOPIC>
        inline void visPointcloudByVector(const PC &pc, const TOPIC &topic)
        {
            auto got = publisher_map_.find(topic);
            if (got == publisher_map_.end())
            {
                ros::Publisher pub = nh_.advertise<sensor_msgs::PointCloud2>(topic, 10000);
                publisher_map_[topic] = pub;
                 Delayforvis(pub,topic)
            }
            pcl::PointCloud<pcl::PointXYZ> point_cloud;
            sensor_msgs::PointCloud2 point_cloud_msg;
            point_cloud.reserve(pc.size());
            for (const auto &pt : pc)
            {
                point_cloud.points.emplace_back(pt[0], pt[1], pt[2]);
            }
            pcl::toROSMsg(point_cloud, point_cloud_msg);
            point_cloud_msg.header.frame_id = "map";
            point_cloud_msg.header.stamp = ros::Time::now();
            publisher_map_[topic].publish(point_cloud_msg);
        }

        /**
         * 可视化点云,包括强度
         * 
         * @param pc    要可视化的点云
         * @param topic 话题名
         */
        template <class PC, class TOPIC>
        inline void visPointcloudXYZI(const PC &pc, const TOPIC &topic)
        {
            auto got = publisher_map_.find(topic);
            if (got == publisher_map_.end())
            {
                ros::Publisher pub = nh_.advertise<sensor_msgs::PointCloud2>(topic, 10000);
                publisher_map_[topic] = pub;
                 Delayforvis(pub,topic)
            }
            sensor_msgs::PointCloud2 point_cloud_msg;
            pcl::toROSMsg(pc, point_cloud_msg);
            point_cloud_msg.header.frame_id = "map";
            point_cloud_msg.header.stamp = ros::Time::now();
            publisher_map_[topic].publish(point_cloud_msg);
        }

        /**
         * 可视化路径
         * 
         * @param path  要可视化的路径
         * @param topic 话题名
         */
        template <class PATH, class TOPIC>
        inline void visPath(const PATH &path, const TOPIC &topic)
        {
            auto got = publisher_map_.find(topic);
            if (got == publisher_map_.end())
            {
                ros::Publisher pub = nh_.advertise<nav_msgs::Path>(topic, 10000);
                publisher_map_[topic] = pub;
                 Delayforvis(pub,topic)
            }
            nav_msgs::Path path_msg;
            geometry_msgs::PoseStamped tmpPose;
            tmpPose.header.frame_id = "map";
            for (const auto &pt : path)
            {
                tmpPose.pose.position.x = pt[0];
                tmpPose.pose.position.y = pt[1];
                tmpPose.pose.position.z = pt[2];
                path_msg.poses.push_back(tmpPose);
            }
            path_msg.header.frame_id = "map";
            path_msg.header.stamp = ros::Time::now();
            publisher_map_[topic].publish(path_msg);
        }

        /**
         * 可视化很多球体
         * 
         * @param balls 要可视化的球体
         * @param topic 话题名
         * @param keep  保留之前绘制的内容
         * @param color 颜色
         * @param a     不透明度(0~1)
         */
        template <class BALLS, class TOPIC>
        inline void visBalls(const BALLS &balls,
                                    const TOPIC &topic,
                                    bool keep = false,
                                    const Color color = blue,
                                    const double a = 0.2)
        {
            auto got = publisher_map_.find(topic);
            if (got == publisher_map_.end())
            {
                ros::Publisher pub =
                    nh_.advertise<visualization_msgs::MarkerArray>(topic, 10000);
                publisher_map_[topic] = pub;
                 Delayforvis(pub,topic)
            }
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            if (!keep)
            {
                marker.id = 0;
            }
            else
            {
                static int id = 0;
                marker.id = id;
                id++;
            }
            setMarkerColor(marker, color, a);
            visualization_msgs::MarkerArray marker_array;
            marker_array.markers.reserve(balls.size() + 1);
            marker.action = visualization_msgs::Marker::DELETEALL; // 清空信息
            marker_array.markers.push_back(marker);
            marker.action = visualization_msgs::Marker::ADD;
            for (const auto &ball : balls)
            {
                setMarkerPose(marker, ball.center[0], ball.center[1], ball.center[2]);
                auto d = 2 * ball.radius;
                setMarkerScale(marker, d, d, d);
                marker_array.markers.push_back(marker);
                marker.id++;
            }
            publisher_map_[topic].publish(marker_array);
        }

        template <class TOPIC>
        inline void visBalls(const Eigen::Matrix3Xd &balls,
                                    const TOPIC &topic,
                                    bool keep = false,
                                    const Color color = blue,
                                    const double a = 0.2,
                                    const double scale = 1)
        {
            auto got = publisher_map_.find(topic);
            if (got == publisher_map_.end())
            {
                ros::Publisher pub =
                    nh_.advertise<visualization_msgs::MarkerArray>(topic, 10000);
                publisher_map_[topic] = pub;
                 Delayforvis(pub,topic)
            }
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            if (!keep)
            {
                marker.id = 0;
            }
            else
            {
                static int id = 0;
                marker.id = id;
                id++;
            }
            setMarkerColor(marker, color, a);
            visualization_msgs::MarkerArray marker_array;
            int size = balls.cols();
            marker_array.markers.reserve(size + 1);
            marker.action = visualization_msgs::Marker::DELETEALL; // 清空信息
            marker_array.markers.push_back(marker);
            marker.action = visualization_msgs::Marker::ADD;
            for (int i = 0; i < size; i++)
            {
                setMarkerPose(marker, balls.col(i)[0], balls.col(i)[1], balls.col(i)[2]);
                setMarkerScale(marker, scale, scale, scale);
                marker_array.markers.push_back(marker);
                marker.id++;
            }

            publisher_map_[topic].publish(marker_array);
        }

        /**
         * 可视化很多椭球
         * 
         * @param ellipsoids 要可视化的椭球
         * @param topic 话题名
         * @param color 颜色
         * @param a     不透明度(0~1)
         */
        template <class ELLIPSOIDS, class TOPIC>
        inline void visEllipsoids(const ELLIPSOIDS &ellipsoids,
                                         const TOPIC &topic,
                                         const Color color = blue,
                                         const double a = 0.8)
        {
            auto got = publisher_map_.find(topic);
            if (got == publisher_map_.end())
            {
                ros::Publisher pub =
                    nh_.advertise<visualization_msgs::MarkerArray>(topic, 10000);
                publisher_map_[topic] = pub;
                 Delayforvis(pub,topic)
            }
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.id = 0;
            setMarkerColor(marker, color, a);
            visualization_msgs::MarkerArray marker_array;
            marker_array.markers.reserve(ellipsoids.size() + 1);
            marker.action = visualization_msgs::Marker::DELETEALL;
            marker_array.markers.push_back(marker);
            marker.action = visualization_msgs::Marker::ADD;
            for (const auto &e : ellipsoids)
            {
                setMarkerPose(marker, e.c[0], e.c[1], e.c[2], e.R); // 导入xyz位置，姿态默认水平
                setMarkerScale(marker, 2 * e.rx, 2 * e.ry, 2 * e.rz);
                marker_array.markers.push_back(marker);
                marker.id++;
            }
            publisher_map_[topic].publish(marker_array);
        }
        
        
        /**
         * 可视化连线对
         * 
         * @param pairline 要可视化的连线对
         * @param topic 话题名
         * @param color 颜色
         * @param scale 尺寸因子
         */
        template <class PAIRLINE, class TOPIC>
        inline void visPairline(const PAIRLINE &pairline, const TOPIC &topic, const Color &color = green, double scale = 0.1)
        {
            auto got = publisher_map_.find(topic);
            if (got == publisher_map_.end())
            {
                ros::Publisher pub = nh_.advertise<visualization_msgs::Marker>(topic, 10000);
                publisher_map_[topic] = pub;
                 Delayforvis(pub,topic)
            }
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.type = visualization_msgs::Marker::LINE_LIST;
            marker.action = visualization_msgs::Marker::ADD;
            setMarkerPose(marker, 0, 0, 0);
            setMarkerColor(marker, color, 1);
            setMarkerScale(marker, scale, scale, scale);
            marker.points.resize(2 * pairline.size());
            for (size_t i = 0; i < pairline.size(); ++i)
            {
                marker.points[2 * i + 0].x = pairline[i].first[0];
                marker.points[2 * i + 0].y = pairline[i].first[1];
                marker.points[2 * i + 0].z = pairline[i].first[2];
                marker.points[2 * i + 1].x = pairline[i].second[0];
                marker.points[2 * i + 1].y = pairline[i].second[1];
                marker.points[2 * i + 1].z = pairline[i].second[2];
            }
            publisher_map_[topic].publish(marker);
        }

        /**
         * 可视化一堆箭头
         * 
         * @param arrows 要可视化的箭头
         * @param topic 话题名
         * @param color 颜色
         */
        template <class ARROWS, class TOPIC>
        inline void visArrows(const ARROWS &arrows, const TOPIC &topic, const Color &color)
        {
            auto got = publisher_map_.find(topic);
            if (got == publisher_map_.end())
            {
                ros::Publisher pub =
                    nh_.advertise<visualization_msgs::MarkerArray>(topic, 10000);
                publisher_map_[topic] = pub;
                 Delayforvis(pub,topic)
            }
            visualization_msgs::Marker clear_previous_msg;
            clear_previous_msg.action = visualization_msgs::Marker::DELETEALL;
            visualization_msgs::Marker arrow_msg;
            arrow_msg.type = visualization_msgs::Marker::ARROW;
            arrow_msg.action = visualization_msgs::Marker::ADD;
            arrow_msg.header.frame_id = "map";
            arrow_msg.id = 0;
            arrow_msg.points.resize(2);
            setMarkerPose(arrow_msg, 0, 0, 0);
            setMarkerScale(arrow_msg, 0.4, 0.7, 0);
            setMarkerColor(arrow_msg, color, 0.7);
            visualization_msgs::MarkerArray arrow_list_msg;
            arrow_list_msg.markers.reserve(1 + arrows.size());
            arrow_list_msg.markers.push_back(clear_previous_msg);
            for (const auto &arrow : arrows)
            {
                arrow_msg.points[0].x = arrow.first[0];
                arrow_msg.points[0].y = arrow.first[1];
                arrow_msg.points[0].z = arrow.first[2];
                arrow_msg.points[1].x = arrow.second[0];
                arrow_msg.points[1].y = arrow.second[1];
                arrow_msg.points[1].z = arrow.second[2];
                arrow_list_msg.markers.push_back(arrow_msg);
                arrow_msg.id += 1;
            }
            publisher_map_[topic].publish(arrow_list_msg);
        }

        /**
         * 可视化三维矢量
         * 
         * @param vec_pos 矢量位置
         * @param vec_dir 矢量方向
         * @param topic 话题名
         * @param color 颜色
         * @param id    指定id
         * @param clear_old 保留之前绘制的内容
         */
        template <class TOPIC>
        inline void visVector(const Eigen::Vector3d &vec_pos, Eigen::Vector3d vec_dir,  const TOPIC &topic, const Color &color, const int& id, bool clear_old = false)
        {
            auto got = publisher_map_.find(topic);
            if (got == publisher_map_.end())
            {
                ros::Publisher pub =
                    nh_.advertise<visualization_msgs::Marker>(topic, 10000);
                publisher_map_[topic] = pub;
                 Delayforvis(pub,topic)
            }

            if(clear_old){
                visualization_msgs::Marker clc;
                clc.header.frame_id = "map";
                clc.type = visualization_msgs::Marker::DELETEALL;
                publisher_map_[topic].publish(clc);
            }

            visualization_msgs::Marker vec;
            vec.type = visualization_msgs::Marker::ARROW;
            vec.action = visualization_msgs::Marker::ADD;
            vec.header.frame_id = "map";
            vec.id = id;

            vec.scale.x = 1.0;
            vec.scale.y = 0.2;
            vec.scale.z = 0.2;
            vec.color.a = 1;
            vec.color.r = 1.0;
            vec.color.g = 1.0;
            vec.color.b = 0.0;
            vec.pose.position.x = vec_pos(0);
            vec.pose.position.y = vec_pos(1);
            vec.pose.position.z = vec_pos(2);
            vec_dir.normalize();
            Eigen::Vector3d final_pose = 0.5 * (Eigen::Vector3d(1, 0, 0) + vec_dir);
            // final_pose.normalize();
            Eigen::AngleAxisd t_V(M_PI, final_pose);
            Eigen::Quaterniond q(t_V);
            q.normalize();
            vec.pose.orientation.w = q.w();
            vec.pose.orientation.x = q.x();
            vec.pose.orientation.y = q.y();
            vec.pose.orientation.z = q.z();
            publisher_map_[topic].publish(vec);
        }

        /**
         * 可视化两点之间连线
         * 
         * @param edge_topic edge话题名
         * @param start      起点
         * @param end        终点
         */
        template <class TOPIC>
        inline void visEdge(const TOPIC &edge_topic, const Eigen::MatrixX3d &start, const Eigen::MatrixX3d &end)
        {
            assert(start.rows() == end.rows() && "start point size != end point size");
            auto got = publisher_map_.find(edge_topic);
            if (got == publisher_map_.end())
            {
                ros::Publisher pub =
                    nh_.advertise<visualization_msgs::Marker>(edge_topic, 10000);
                publisher_map_[edge_topic] = pub;
                 Delayforvis(pub,edge_topic)
            }
            visualization_msgs::Marker edgeMarker;
            edgeMarker.header.stamp = ros::Time::now();
            edgeMarker.header.frame_id = "map";
            edgeMarker.pose.orientation.w = 1.00;
            edgeMarker.action = visualization_msgs::Marker::ADD;
            edgeMarker.type = visualization_msgs::Marker::LINE_LIST;
            edgeMarker.ns = "edge";
            edgeMarker.color.r = 0.00;
            edgeMarker.color.g = 1.00;
            edgeMarker.color.b = 0.00;
            edgeMarker.color.a = 1.00;
            edgeMarker.scale.x = 0.5;
            edgeMarker.scale.y = 0.5;
            edgeMarker.scale.z = 0.5;
            static int id = 0;
            id++;
            edgeMarker.id = id;
            geometry_msgs::Point point;
            int size = start.rows();
            for (int i = 0; i < start.rows(); i++)
            {
                point.x = start(i, 0);
                point.y = start(i, 1);
                point.z = start(i, 2);
                edgeMarker.points.push_back(point);
                point.x = end(i, 0);
                point.y = end(i, 1);
                point.z = end(i, 2);
                edgeMarker.points.push_back(point);
            }
            publisher_map_[edge_topic].publish(edgeMarker);
        }
        // 可视化两点之间连线，碰撞检测debug
        template <class TOPIC>
        inline void visEdge(const TOPIC &edge_topic, const Eigen::Vector3d &start, const Eigen::Vector3d &end)
        {

            auto got = publisher_map_.find(edge_topic);
            if (got == publisher_map_.end())
            {
                ros::Publisher pub =
                    nh_.advertise<visualization_msgs::Marker>(edge_topic, 10000);
                publisher_map_[edge_topic] = pub;
                Delayforvis(pub,edge_topic)
            }
            visualization_msgs::Marker edgeMarker;
            edgeMarker.header.stamp = ros::Time::now();
            edgeMarker.header.frame_id = "map";
            edgeMarker.pose.orientation.w = 1.00;
            edgeMarker.action = visualization_msgs::Marker::ADD;
            edgeMarker.type = visualization_msgs::Marker::LINE_LIST;
            edgeMarker.ns = "edge";
            edgeMarker.color.r = 1.00;
            edgeMarker.color.g = 0.00;
            edgeMarker.color.b = 0.00;
            edgeMarker.color.a = 1.00;
            edgeMarker.scale.x = 0.00;
            edgeMarker.scale.y = 0.09;
            edgeMarker.scale.z = 0.09;
            static int id = 0;
            id++;
            edgeMarker.id = id;
            geometry_msgs::Point point;

            point.x = start[0];
            point.y = start[1];
            point.z = start[2];
            edgeMarker.points.push_back(point);
            point.x = end[0];
            point.y = end[1];
            point.z = end[2];
            edgeMarker.points.push_back(point);

            publisher_map_[edge_topic].publish(edgeMarker);
        }

     
        /**
         * 可视化Mesh
         * 
         * @param meth_topic mesh话题名
         * @param edge_topic edge话题名
         * @param U          顶点集
         * @param G          边集
         * @param s          尺寸因子
         * @param a          不透明度(0~1)
         * @param color      颜色
         */
        template <class TOPIC1,class TOPIC2>//实际上顶点U是X*3维度，G是Y*3维度
        inline void visMesh(const TOPIC1 &meth_topic, const TOPIC2 &edge_topic, const Eigen::MatrixXd &U, const Eigen::MatrixXi &G, double s = 0.1, const Color color = steelblue, const double a = 0.02)
        {
            auto got = publisher_map_.find(meth_topic);
            if (got == publisher_map_.end())
            {
                ros::Publisher pub1 =
                    nh_.advertise<visualization_msgs::Marker>(meth_topic, 10000); // 有可能是队列长度不够导致可视化swept volume有问题
                publisher_map_[meth_topic] = pub1;
                Delayforvis(pub1,meth_topic)
            }
            auto got2 = publisher_map_.find(edge_topic);
            if (got2 == publisher_map_.end())
            {
                ros::Publisher pub2 =
                    nh_.advertise<visualization_msgs::Marker>(edge_topic, 10000);
                publisher_map_[edge_topic] = pub2;
                Delayforvis(pub2,edge_topic)
            }

            visualization_msgs::Marker meshMarker, edgeMarker;

            meshMarker.header.stamp = ros::Time::now();
            meshMarker.header.frame_id = "map";
            meshMarker.pose.orientation.w = 1.00;
            meshMarker.action = visualization_msgs::Marker::ADD;
            meshMarker.type = visualization_msgs::Marker::TRIANGLE_LIST;
            meshMarker.ns = "mesh";
            meshMarker.color.r = 0.00;
            meshMarker.color.g = 0.00;
            meshMarker.color.b = 1.00;
            meshMarker.color.a = a;

            setMarkerColor(meshMarker, color,a);

            meshMarker.scale.x = 1.00;
            meshMarker.scale.y = 1.00;
            meshMarker.scale.z = 1.00;

            edgeMarker = meshMarker;
            edgeMarker.type = visualization_msgs::Marker::LINE_LIST;
            edgeMarker.ns = "edge";
            edgeMarker.color.r = 0.00;
            edgeMarker.color.g = 1.00;
            edgeMarker.color.b = 0.00;
            edgeMarker.color.a = 1.00;
            edgeMarker.scale.x = 0.005 * s;
            edgeMarker.scale.y = 0.005 * s;
            edgeMarker.scale.z = 0.005 * s;

            geometry_msgs::Point point;

            int faces = G.rows();
            for (int i = 0; i < faces; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    point.x = U.row(G.row(i)[j])[0];
                    point.y = U.row(G.row(i)[j])[1];
                    point.z = U.row(G.row(i)[j])[2];
                    meshMarker.points.push_back(point);

                    edgeMarker.points.push_back(point);
                    point.x = U.row(G.row(i)[(j + 1) % 3])[0];
                    point.y = U.row(G.row(i)[(j + 1) % 3])[1];
                    point.z = U.row(G.row(i)[(j + 1) % 3])[2];
                    edgeMarker.points.push_back(point);
                }
            }
            publisher_map_[edge_topic].publish(edgeMarker);
            publisher_map_[meth_topic].publish(meshMarker);

        }


        /**
         * 可视化多面体
         * 
         * @param mesh       顶点集？
         * @param meth_topic mesh话题名
         * @param edge_topic edge话题名
         * @param keep       保留之前绘制的结果
         * @param s          尺寸因子
         * @param a          不透明度(0~1)
         * @param color      颜色
         */
        template <class TOPIC1,class TOPIC2>
        inline void visPolytope(const Eigen::Matrix3Xd &mesh, const TOPIC1 &meth_topic, const TOPIC2 &edge_topic, bool keep = false, double s = 0.1, const Color color = steelblue,const double a=1.0, bool del = false)
        {

            auto got = publisher_map_.find(meth_topic);
            if (got == publisher_map_.end())
            {
                ros::Publisher pub1 =
                    nh_.advertise<visualization_msgs::Marker>(meth_topic, 10000); // 有可能是队列长度不够导致可视化swept volume有问题
                publisher_map_[meth_topic] = pub1;
                Delayforvis(pub1,meth_topic)
            }
            auto got2 = publisher_map_.find(edge_topic);
            if (got2 == publisher_map_.end())
            {
                ros::Publisher pub2 =
                    nh_.advertise<visualization_msgs::Marker>(edge_topic, 10000);
                publisher_map_[edge_topic] = pub2;
                Delayforvis(pub2,edge_topic)
            }

            visualization_msgs::Marker meshMarker, edgeMarker;
            if (!keep)
            {
                meshMarker.id = 0;
            }
            else
            {
                static int id = 700;
                meshMarker.id = id;
                id++;
            }

            if(del == true){
                meshMarker.action = visualization_msgs::Marker::DELETEALL;
                edgeMarker=meshMarker;
                publisher_map_[meth_topic].publish(meshMarker);
                publisher_map_[edge_topic].publish(edgeMarker);
                return ;
            }
            else{
                meshMarker.action = visualization_msgs::Marker::ADD;
            }
            meshMarker.header.stamp = ros::Time::now();
            meshMarker.header.frame_id = "map";
            meshMarker.pose.orientation.w = 1.00;
            meshMarker.type = visualization_msgs::Marker::TRIANGLE_LIST;
            meshMarker.ns = "mesh";


            setMarkerColor(meshMarker, color,a);//透明度无

            meshMarker.scale.x = 1.00;
            meshMarker.scale.y = 1.00;
            meshMarker.scale.z = 1.00;

            edgeMarker = meshMarker;
            edgeMarker.type = visualization_msgs::Marker::LINE_LIST;
            edgeMarker.ns = "edge";
            edgeMarker.color.r = 0.00;
            edgeMarker.color.g = 1.00;
            edgeMarker.color.b = 0.00;
            edgeMarker.color.a = a;
            edgeMarker.scale.x = 0.005 * s;
            edgeMarker.scale.y = 0.005 * s;
            edgeMarker.scale.z = 0.005 * s;

            geometry_msgs::Point point;

            int ptnum = mesh.cols(); // ptnum/3

            for (int i = 0; i < ptnum; i++)
            {
                point.x = mesh(0, i);
                point.y = mesh(1, i);
                point.z = mesh(2, i);
                meshMarker.points.push_back(point);
            }

            for (int i = 0; i < ptnum / 3; i++) // mesh.cols() 12
            {
                for (int j = 0; j < 3; j++)
                {
                    point.x = mesh(0, 3 * i + j);
                    point.y = mesh(1, 3 * i + j);
                    point.z = mesh(2, 3 * i + j);
                    edgeMarker.points.push_back(point);
                    point.x = mesh(0, 3 * i + (j + 1) % 3);
                    point.y = mesh(1, 3 * i + (j + 1) % 3);
                    point.z = mesh(2, 3 * i + (j + 1) % 3);
                    edgeMarker.points.push_back(point);
                }
            }
            publisher_map_[edge_topic].publish(edgeMarker);
            publisher_map_[meth_topic].publish(meshMarker);
        }

        ////////////////////////////////////////////////////////////////
        ///////////////////////  more useful ///////////////////////////
        ////////////////////////////////////////////////////////////////
        /**
         * 可视化R3路径
         * 
         * @param topic       话题名
         * @param path        需要渲染的路径
         */
        template <class TOPIC>
        inline void visR3Path( const TOPIC &topic, const vector<Vector3d>& path, int path_id = 114514)
        {
            auto got = publisher_map_.find(topic);
            if (got == publisher_map_.end())
            {
                ros::Publisher pub1 =
                    nh_.advertise<visualization_msgs::Marker>(topic, 10000); // 有可能是队列长度不够导致可视化swept volume有问题
                publisher_map_[topic] = pub1;
                Delayforvis(pub1,topic)
            }

            visualization_msgs::Marker sphere, line_strip;
            sphere.header.frame_id = line_strip.header.frame_id = "map";
            sphere.header.stamp    = line_strip.header.stamp = ros::Time::now();

            sphere.type     = visualization_msgs::Marker::SPHERE_LIST;
            line_strip.type = visualization_msgs::Marker::LINE_STRIP;

            sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
            sphere.id     = path_id;
            line_strip.id = path_id + 1000;

            sphere.pose.orientation.w   = line_strip.pose.orientation.w = 1.0;
            sphere.color.r              = line_strip.color.r            = 0.4;
            sphere.color.g              = line_strip.color.g            = 1.0;
            sphere.color.b              = line_strip.color.b            = 0.4;
            sphere.color.a              = line_strip.color.a            = 0.8;
            sphere.scale.x              = line_strip.scale.x            = 0.1;
            sphere.scale.y              = line_strip.scale.y            = 0.1;
            sphere.scale.z              = line_strip.scale.z            = 0.1;

            geometry_msgs::Point pt;
            Eigen::Vector3d ptv;
            for (size_t i = 0; i < path.size(); i++)
            {
                ptv = path[i];
                pt.x = ptv(0);
                pt.y = ptv(1);
                pt.z = ptv(2);
                sphere.points.push_back(pt);
                line_strip.points.push_back(pt);
            }
            publisher_map_[topic].publish(sphere);
            publisher_map_[topic].publish(line_strip);
        }

        /**
         * 可视化SE3轨迹
         * 
         * @param topic       话题名
         * @param mesh_var    需要渲染的模型
         * @param se3_path    需要渲染的SE3路径
         * @param se3_path_id    可指定ID
         */
        template <class TOPIC>
        inline void visSE3Path( const TOPIC &topic, const Eigen::Matrix3Xd& mesh, const vector<SE3State>& se3_path,const double a=0.1, int se3_path_id = 1918)
        {
            if (se3_path.size() == 0)
            {
                return;
            }
            auto got = publisher_map_.find(topic);
            if (got == publisher_map_.end())
            {
                ros::Publisher pub1 =
                    nh_.advertise<visualization_msgs::Marker>(topic, 10000); // 有可能是队列长度不够导致可视化swept volume有问题
                publisher_map_[topic] = pub1;
                Delayforvis(pub1,topic)
            }
            
            Eigen::Vector3d pos;
            Eigen::Matrix3d rotate;
            Eigen::Matrix3Xd debugmesh_var;
            bool keep = true;
            visPolytope(debugmesh_var, topic, "SE3edge", keep, 0.1,  orange, a, true);//先清除轨迹
            for (int i = 0; i < se3_path.size(); i++)    
            {
                pos    = se3_path[i].position;
                rotate = se3_path[i].getRotMatrix();
                debugmesh_var = (rotate * mesh).colwise() + pos;
                if (i == se3_path.size() - 1){ keep = false;}
                visPolytope(debugmesh_var, topic, "SE3edge", keep, 10,  orange, a, false);


            }
        }

         /**
         * 可视化SE3轨迹,但是用向量表示
         * 
         * @param topic       话题名
         * @param mesh_var    需要渲染的模型
         * @param se3_path    需要渲染的SE3路径
         * @param se3_path_id    可指定ID
         */
        template <class TOPIC>
        inline void visSE3Vec( const TOPIC &topic, const vector<Eigen::Vector3d>& points, const vector<Eigen::Vector3d>& acc_list, int se3_path_id = 191018)
        {
            if (acc_list.size() == 0)
            {
                return;
            }
            auto got = publisher_map_.find(topic);
            if (got == publisher_map_.end())
            {
                ros::Publisher pub1 =
                    nh_.advertise<visualization_msgs::Marker>(topic, 10000); // 有可能是队列长度不够导致可视化swept volume有问题
                publisher_map_[topic] = pub1;
                Delayforvis(pub1,topic)
            }
            
            visualization_msgs::Marker line_strip;
            line_strip.header.frame_id = "map";
            line_strip.header.stamp = ros::Time::now();
            line_strip.type   = visualization_msgs::Marker::LINE_STRIP;
            line_strip.action = visualization_msgs::Marker::ADD;
            line_strip.id = se3_path_id;

            line_strip.color.r = 1;
            line_strip.color.g = 0;
            line_strip.color.b = 0;
            line_strip.color.a = 1;
            line_strip.scale.x = 0.05 / 2;

            geometry_msgs::Point pt;
            for (double i = 0; i < points.size(); i++)
            {
                Eigen::Vector3d dur_p = points[i];
                pt.x = dur_p(0);
                pt.y = dur_p(1);
                pt.z = dur_p(2);
                line_strip.points.push_back(pt);
            }
            publisher_map_[topic].publish(line_strip);

            const double alpha = 0.1;
            for (double i = 0; i < points.size(); i++)
            {
                Eigen::Vector3d arr_p = points[i];
                Eigen::Vector3d arr_a = acc_list[i];

                visualization_msgs::Marker acc_dir;
                acc_dir.header.frame_id = "map";
                acc_dir.id   = se3_path_id + 1 + i;
                acc_dir.type = visualization_msgs::Marker::ARROW;
                //   acc_dir.scale.x = arr_p.norm();
                acc_dir.scale.x = 1.0;
                acc_dir.scale.y = 0.2;
                acc_dir.scale.z = 0.2;
                acc_dir.color.a = 1;
                acc_dir.color.r = 0.2;
                acc_dir.color.g = 0.2;
                acc_dir.color.b = 1.0;
                acc_dir.pose.position.x = arr_p(0);
                acc_dir.pose.position.y = arr_p(1);
                acc_dir.pose.position.z = arr_p(2);

                arr_a.normalize();
                Eigen::Vector3d final_pose = 0.5 * (Eigen::Vector3d(1, 0, 0) + arr_a);
                // final_pose.normalize();
                Eigen::AngleAxisd t_V(M_PI, final_pose);
                Eigen::Quaterniond q(t_V);
                q.normalize();
                acc_dir.pose.orientation.w = q.w();
                acc_dir.pose.orientation.x = q.x();
                acc_dir.pose.orientation.y = q.y();
                acc_dir.pose.orientation.z = q.z();
                publisher_map_[topic].publish(acc_dir);
            }
        }

        /**
         * 可视化轨迹
         * 
         * @param topic       话题名
         * @param traj        需要渲染的轨迹
         * @param traj_id     可指定id
         */
        template <class TOPIC>
        inline void visTraj(const TOPIC &topic, Trajectory<TRAJ_ORDER> traj, int traj_id = 15526)
        {
            auto got = publisher_map_.find(topic);
            if (got == publisher_map_.end())
            {
                ros::Publisher pub1 =
                    nh_.advertise<visualization_msgs::Marker>(topic, 10000); // 有可能是队列长度不够导致可视化swept volume有问题
                publisher_map_[topic] = pub1;
                Delayforvis(pub1,topic)
            }
            visualization_msgs::Marker traj_vis;
            traj_vis.header.stamp       = ros::Time::now();
            traj_vis.header.frame_id    = "map";
            traj_vis.id                 = traj_id;
            traj_vis.type               = visualization_msgs::Marker::LINE_STRIP;
            traj_vis.scale.x            = 0.1;
            traj_vis.scale.y            = 0.1;
            traj_vis.scale.z            = 0.1;
            traj_vis.pose.orientation.x = 0.0;
            traj_vis.pose.orientation.y = 0.0;
            traj_vis.pose.orientation.z = 0.0;
            traj_vis.pose.orientation.w = 1.0;

            traj_vis.color.a            = 1.0;
            traj_vis.color.r            = 1.0;
            traj_vis.color.g            = 1.0;
            traj_vis.color.b            = 0.5;

            geometry_msgs::Point pt;
            Vector3d pos;
            double t_duration = traj.getTotalDuration();
            for (double t = 0; t < t_duration; t += 0.05)
            {
                pos = traj.getPos(t);
                pt.x = pos(0);
                pt.y = pos(1);
                pt.z = pos(2);
                traj_vis.points.push_back(pt);
            }
            publisher_map_[topic].publish(traj_vis);
            
        }

}; // namespace vis
}
#endif