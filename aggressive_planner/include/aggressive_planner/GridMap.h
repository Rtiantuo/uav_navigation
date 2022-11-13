//
// Created by wangzichen on 2022/10/26.
//

#ifndef SRC_GRIDMAP_H
#define SRC_GRIDMAP_H
#include <iostream>
#include <ros/ros.h>
#include <string>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_sequencer.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>


#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/StdVector>

#include <ctime>

#include <cv_bridge/cv_bridge.h>

#include <tf/tf.h>

enum {REAL_FLIGHT_VIO_T265 = 1, SIM_FLIGHT = 0};

struct GridIndex
{
    int x;
    int y;
    int z;

    void SetIndex(int x_,int y_,int z_)
    {
        x=x_;
        y=y_;
        z=z_;
    }
};

struct MappingParameters{
    int real_or_sim;//1 is real, -1 is sim

    double cx,cy;
    double fx,fy;
    int  sizex,sizey,sizez;
    double resolution;
    double origin_x,origin_y,origin_z;
    int inflate_factor_x,inflate_factor_y,inflate_factor_z;
    int offset_x,offset_y;
    double depth_filter_min,depth_filter_max;
    double output_low_bound,output_up_bound;

    std::string frame_id;
};

struct MappingData{
    Eigen::Vector3d Pose;
    Eigen::Matrix3d Rotation;

    cv::Mat depth_image_;
    bool has_image;
    bool has_map;

    int *pMap;
    int *pMap_inflated;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inflated;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdTree;
};

class GridMap{
public:
    //建图时用到变量
    MappingParameters mp;
    MappingData md;
    ros::NodeHandle nh;

    //建图用到的publisher以及timer等
    ros::Publisher map_un_inflated_pub;
    ros::Publisher map_pub;
    ros::Timer update_timer;
    ros::Timer publisher_timer;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry>
            SyncPolicyImageOdom;
    typedef std::shared_ptr<message_filters::Synchronizer<SyncPolicyImageOdom>> SynchronizerImageOdom;
    SynchronizerImageOdom sync_image_odom;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub;
    std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub;

    //建图时用到的主要函数
    GridMap() {};
    ~GridMap() {};

    void initMap(ros::NodeHandle& node);
    void depthOdomCallback(const sensor_msgs::ImageConstPtr&img,
                           const nav_msgs::OdometryConstPtr&odom);
    void updateMap(const ros::TimerEvent&event);
    void publishMap(const ros::TimerEvent&event);

    //坐标转换等小函数
    inline Eigen::Vector3d cal_point(int u,int v,double depth);//相机转世界
    inline GridIndex ConvertWorld2GridIndex(double x,double y,double z);//世界转栅格
    inline bool isValidGridIndex(GridIndex index);//判断是否超出范围，超出范围则不处理
    inline int GridIndex2LinearIndex(GridIndex index);//栅格地图转线性，对应mMap中的指向某个栅格的指针
    inline Eigen::Vector3d IndexToPos(int index);//线性转世界
    inline int PosToIndex(Eigen::Vector3d pos);//世界转线性

    inline std::vector<GridIndex> raycast(int x1,int y1,int z1,int x2, int y2, int z2);//计算经过的网格
};


//相机转世界
inline Eigen::Vector3d GridMap::cal_point(int u,int v,double depth)
{
    Eigen::Vector3d proj_pt;
    proj_pt(1)=-1*(u-mp.cx)*depth/mp.fx;
    proj_pt(2)=-1*(v-mp.cy)*depth/mp.fy;
    proj_pt(0)=depth;

    proj_pt=md.Rotation*proj_pt+md.Pose;

    return proj_pt;
}

//世界转栅格
inline GridIndex GridMap::ConvertWorld2GridIndex(double x,double y,double z)
{
    GridIndex index;
    index.x=std::ceil((x-mp.origin_x)/mp.resolution)+mp.offset_x;
    index.y=std::ceil((y-mp.origin_y)/mp.resolution)+mp.offset_y;
    index.z=std::ceil((z-mp.origin_z)/mp.resolution);

    return index;
}

//判断是否超出范围，超出范围则不处理
inline bool GridMap::isValidGridIndex(GridIndex index)
{
    if(index.x>=0&&index.y>=0&&index.z>=0&&index.x<mp.sizex&&index.y<mp.sizey&&index.z<mp.sizez)
    {
        return true;
    }
    return false;
}

//栅格地图转线性，对应mMap中的指向某个栅格的指针
inline int GridMap::GridIndex2LinearIndex(GridIndex index)
{
    int linear_index;
    linear_index=index.z*mp.sizex*mp.sizey+index.y*mp.sizex+index.x;
    return linear_index;
}

//线性转世界
inline Eigen::Vector3d GridMap::IndexToPos(int index)
{
    Eigen::Vector3d pos;
    int  idz=std::ceil(index/(mp.sizex*mp.sizey));
    int idxy=index-idz*mp.sizex*mp.sizey;
    int idy=std::ceil(idxy/mp.sizex);
    int idx=idxy-idy*mp.sizex;

    double px=(idx-mp.offset_x)*mp.resolution+mp.origin_x;
    double py=(idy-mp.offset_y)*mp.resolution+mp.origin_y;
    double pz=idz*mp.resolution+mp.origin_z;

    return Eigen::Vector3d(px,py,pz);
}

//世界转线性
inline int GridMap::PosToIndex(Eigen::Vector3d pos)
{
    int index;
    double px = pos(0);
    double py = pos(1);
    double pz = pos(2);

    int idx = std::round((px - mp.origin_x)/mp.resolution) + mp.offset_x;
    int idy = std::round((py - mp.origin_y)/mp.resolution) + mp.offset_y;
    int idz = std::round((pz - mp.origin_z)/mp.resolution);

    if(idx>=mp.sizex||idx<0||idy>=mp.sizey||idy<0||idz>=mp.sizez||idz<0)
    {
        index=-1;
        return index;
    }

    index = idz*mp.sizex*mp.sizey + idy*mp.sizex + idx;
    return index;
}

//计算经过的网格
inline std::vector<GridIndex> GridMap::raycast(int x1,int y1,int z1,int x2, int y2, int z2)
{
    GridIndex tmpIndex;
    std::vector<GridIndex> gridIndexVector;

    int dx, dy, dz;
    int sx, sy, sz;

    dx  = abs(x2 - x1);
    dy  = abs(y2 - y1);
    dz  = abs(z2 - z1);

    if (x1 > x2)
    {
        sx = -1;
    }
    else
    {
        sx = +1;
    }

    if (y1 > y2)
    {
        sy = -1;
    }
    else
    {
        sy = +1;
    }

    if (z1 > z2)
    {
        sz = -1;
    }
    else
    {
        sz = +1;
    }

    int PX;
    int PY;
    int PZ;
    if (dx>=dy && dx >= dz)
    {
        int deltaX = abs(x2 - x1);
        int deltaY = abs(y2 - y1);
        int deltaZ = abs(z2 - z1);
        int errorY = 0;
        int errorZ = 0;
        int y = y1;
        int z = z1;
        for(int x = x1; x!=x2;x=x+sx)
        {
            PX = x;
            PY = y;
            PZ = z;

            errorY += deltaY;
            errorZ += deltaZ;

            if (2*errorY > deltaX)
            {
                y += sy;
                errorY -= deltaX;
            }

            if (2*errorZ > deltaX)
            {
                z += sz;
                errorZ -= deltaX;
            }

            if (PX == x2 && PY == y2 && PZ == z2) continue;

            tmpIndex.SetIndex(PX,PY,PZ);
            gridIndexVector.push_back(tmpIndex);
        }
    }

    else if (dy>=dx && dy >= dz)
    {
        int deltaX = abs(x2 - x1);
        int deltaY = abs(y2 - y1);
        int deltaZ = abs(z2 - z1);
        int errorX = 0;
        int errorZ = 0;
        int x = x1;
        int z = z1;
        for(int y = y1; y!=y2;y=y+sy)
        {
            PX = x;
            PY = y;
            PZ = z;

            errorX += deltaX;
            errorZ += deltaZ;

            if (2*errorX > deltaY)
            {
                x += sx;
                errorX -= deltaY;
            }

            //if (2*errorZ > deltaX)
            if (2*errorZ > deltaY)
            {
                z += sz;
                errorZ -= deltaY;
            }

            if (PX == x2 && PY == y2 && PZ == z2) continue;

            tmpIndex.SetIndex(PX,PY,PZ);
            gridIndexVector.push_back(tmpIndex);
        }
    }
    else
    {
        int deltaX = abs(x2 - x1);
        int deltaY = abs(y2 - y1);
        int deltaZ = abs(z2 - z1);
        int errorX = 0;
        int errorY = 0;
        int x = x1;
        int y = y1;
        for(int z = z1; z!=z2;z=z+sz)
        {
            PX = x;
            PY = y;
            PZ = z;

            errorX += deltaX;
            errorY += deltaY;

            if (2*errorX > deltaZ)
            {
                x += sx;
                errorX -= deltaZ;
            }

            if (2*errorY > deltaZ)
            {
                y += sy;
                errorY -= deltaZ;
            }

            if (PX == x2 && PY == y2 && PZ == z2) continue;

            tmpIndex.SetIndex(PX,PY,PZ);
            gridIndexVector.push_back(tmpIndex);
        }
    }

    return gridIndexVector;
}



#endif //SRC_GRIDMAP_H
