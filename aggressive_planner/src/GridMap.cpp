//
// Created by wangzichen on 2022/10/26.
//

//todo 写成按区域进行更新的方式
//todo 膨胀的方法改成在pmap中进行

//todo 这个代码会在建图范围增大以后变慢

#include <aggressive_planner/GridMap.h>

void GridMap::initMap(ros::NodeHandle &node)
{
    nh = node;
    //初始化map parameters
    nh.param("/real_or_sim",mp.real_or_sim,1);

    nh.param("/grid_map/cx",mp.cx,315.917);
    nh.param("/grid_map/cy",mp.cy,243.442);
    nh.param("/grid_map/fx",mp.fx,387.355);
    nh.param("/grid_map/fy",mp.fy,387.355);
    nh.param("/grid_map/map_sizex",mp.sizex,400);
    nh.param("/grid_map/map_sizey",mp.sizey,400);
    nh.param("/grid_map/map_sizez",mp.sizez,100);
    nh.param("/grid_map/map_resolution",mp.resolution,0.1);
    nh.param("/grid_map/map_origin_x",mp.origin_x,0.0);
    nh.param("/grid_map/map_origin_y",mp.origin_y,0.0);
    nh.param("/grid_map/map_origin_z",mp.origin_z,0.0);
    nh.param("/grid_map/map_inflate_factor_x",mp.inflate_factor_x,2);
    nh.param("/grid_map/map_inflate_factor_y",mp.inflate_factor_y,2);
    nh.param("/grid_map/map_inflate_factor_z",mp.inflate_factor_z,1);
    mp.offset_x=mp.sizex/2;
    mp.offset_y=mp.sizey/2;
    nh.param("/grid_map/depth_filter_min",mp.depth_filter_min,0.3);
    nh.param("/grid_map/depth_filter_max",mp.depth_filter_max,5.0);
    nh.param("/grid_map/output_low_bound",mp.output_low_bound,0.0);
    nh.param("/grid_map/output_up_bound",mp.output_up_bound,2.5);

    std::cout<<"[MAP]The resolution of the grid map is : "<<mp.resolution<<"m."<<std::endl;

    //初始化map data
    md.has_image = false;
    md.has_map = false;

    md.pMap=new int[mp.sizex*mp.sizey*mp.sizez];
    //初始化空间中所有点的信息均为50
    for(int i=0;i<mp.sizex*mp.sizey*mp.sizez;i++)
    {
        md.pMap[i]=50;
    }

    md.pMap_inflated=new int[mp.sizex*mp.sizey*mp.sizez];
    for(int i=0;i<mp.sizex*mp.sizey*mp.sizez;i++)
    {
        md.pMap_inflated[i]=0;
    }

    map_un_inflated_pub = nh.advertise<sensor_msgs::PointCloud2>("/grid_map/occupancy_un_inflated",10);
    map_pub= nh.advertise<sensor_msgs::PointCloud2>("/grid_map/occupancy",10);


    if(mp.real_or_sim == REAL_FLIGHT_VIO_T265)
    {
        std::cout<<"\033[7m"<<"\033[33m[MAP]On the real flight (using t265) ! Watch out!"<<"\033[0m"<< std::endl;//yellow
        depth_sub.reset(new message_filters::Subscriber<sensor_msgs::Image>(nh,"/d400/depth/image_rect_raw",1000));
        odom_sub.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh,"/t265/odom/sample",1000));

        sync_image_odom.reset(new message_filters::Synchronizer<SyncPolicyImageOdom>(
                SyncPolicyImageOdom(10), *depth_sub, *odom_sub));
        mp.frame_id = "t265_odom_frame";
    }

    else if (mp.real_or_sim == SIM_FLIGHT)
    {
        std::cout<<"[MAP]In the sim, have fun."<<std::endl;
        depth_sub.reset(new message_filters::Subscriber<sensor_msgs::Image>(nh,"/camera/depth/image_raw",1000));
        odom_sub.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh,"/mavros/local_position/odom",1000));
        sync_image_odom.reset(new message_filters::Synchronizer<SyncPolicyImageOdom>(
                SyncPolicyImageOdom(10), *depth_sub, *odom_sub));
        mp.frame_id = "map";
    }

    sync_image_odom->registerCallback(boost::bind(&GridMap::depthOdomCallback, this,_1,_2));

    update_timer = nh.createTimer(ros::Duration(0.05),&GridMap::updateMap, this);
    publisher_timer = nh.createTimer(ros::Duration(0.05),&GridMap::publishMap, this);
}

//depth和odom的同步回调函数
void GridMap::depthOdomCallback(const sensor_msgs::ImageConstPtr&img,
                                const nav_msgs::OdometryConstPtr&odom)
{
    //step1:获得位置和姿态，pose是位置,Rotation从四元数转化过来
    md.Pose(0)=odom->pose.pose.position.x;
    md.Pose(1)=odom->pose.pose.position.y;
    md.Pose(2)=odom->pose.pose.position.z;
    md.Rotation=Eigen::Quaterniond(odom->pose.pose.orientation.w,odom->pose.pose.orientation.x,
                                   odom->pose.pose.orientation.y,odom->pose.pose.orientation.z).toRotationMatrix();

    //step2：解码相机信息
    cv_bridge::CvImagePtr  cv_prt;
    cv_prt=cv_bridge::toCvCopy(img,img->encoding);
    md.depth_image_=cv_prt->image;

    if(!md.has_image)md.has_image=true;
}

void GridMap::updateMap(const ros::TimerEvent&event)
{
    if(!md.has_image) return;

    for (int u=0;u<640;u=u+3)
    {
        for(int v=0;v<480;v=v+3)
        {

            double depth= (double)(md.depth_image_.at<uint16_t>(v,u))/1000.0;
            if(depth < mp.depth_filter_min) continue;
            if(depth > mp.depth_filter_max) depth = mp.depth_filter_max;
            Eigen::Vector3d pos=cal_point(u,v,depth);

            //起始xyz
            double sx=md.Pose(0),sy=md.Pose(1),sz=md.Pose(2);
            GridIndex start_index=ConvertWorld2GridIndex(sx,sy,sz);

            //终点xyz
            double gx=pos(0),gy=pos(1),gz=pos(2);
            GridIndex goal_index=ConvertWorld2GridIndex(gx,gy,gz);

            GridIndex occ_index;
            occ_index.SetIndex(goal_index.x,goal_index.y,goal_index.z);

            //超出范围不进行处理
            if(!isValidGridIndex(occ_index))
            {
                continue;
            }

            int occ_idx=GridIndex2LinearIndex(occ_index);

            //如果在depth_filter_max范围内,则对终点进行增加,表示被占据的概率增加
            // 如果在depth_filter_max之外则视为无效
            if(depth < mp.depth_filter_max)
            {
                md.pMap[occ_idx]=md.pMap[occ_idx]+2;
            }
            if(md.pMap[occ_idx]>=100)
            {
                md.pMap[occ_idx]=100;
            }

            auto points=raycast(start_index.x,start_index.y,start_index.z,goal_index.x,goal_index.y,goal_index.z);
            for(int i=0;i<(int)points.size();i++)
            {
                GridIndex free_index;
                free_index.SetIndex(points[i].x,points[i].y,points[i].z);
                int idx=GridIndex2LinearIndex(free_index);
                md.pMap[idx]=md.pMap[idx]-1;
                if(md.pMap[idx]<=10)
                {
                    md.pMap[idx]=10;
                }
            }
        }
    }
    if(!md.has_map)md.has_map = true;
}







void GridMap::publishMap(const ros::TimerEvent&event)
{
    if (!md.has_map)return;

    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ>cloud;

    for(int i=0;i<mp.sizex*mp.sizey*mp.sizez;i++)
    {
        if(md.pMap[i]>90)
        {
            Eigen::Vector3d pos;
            pos=IndexToPos(i);

            //高度大于某个值也不发布
            if(pos(2) > mp.output_up_bound) continue;
            if(pos(2) < mp.output_low_bound) continue;
            //针对地面进行了优化排除，低于0.4m的不发布

            pt.x=pos(0);
            pt.y=pos(1);
            pt.z=pos(2);

            cloud.push_back(pt);
        }
    }

    //进行统计滤波
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_before(new pcl::PointCloud<pcl::PointXYZ>);
    *cloud_before = cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_StatisticalRemoval(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> Statistical;
    Statistical.setInputCloud(cloud_before);
    Statistical.setMeanK(50);
    Statistical.setStddevMulThresh(0.5);
    Statistical.filter(*cloud_after_StatisticalRemoval);

    md.cloud_raw = cloud_after_StatisticalRemoval;
    pcl::PointCloud<pcl::PointXYZ> cloud_un_inflated;

    md.cloud_raw->height = 1;
    md.cloud_raw->width = md.cloud_raw->points.size();
    md.cloud_raw->is_dense = true;
    md.cloud_raw->header.frame_id = mp.frame_id;

    sensor_msgs::PointCloud2 cloud_un_inflated_ros;
    pcl::toROSMsg(*md.cloud_raw,cloud_un_inflated_ros);
    map_un_inflated_pub.publish(cloud_un_inflated_ros);

    //进行膨胀
    pcl::PointXYZ pt_out;
    pcl::PointCloud<pcl::PointXYZ> cloud_out;

    //进行障碍物的膨胀
    for(auto & point : md.cloud_raw->points)
    {
        for(int x=-mp.inflate_factor_x;x<=mp.inflate_factor_x;x++)
        {
            for(int y=-mp.inflate_factor_y;y<=mp.inflate_factor_y;y++)
            {
                for(int z=-mp.inflate_factor_z;z<=mp.inflate_factor_z;z++)
                {
                    pt_out.x = point.x + mp.resolution*x;
                    pt_out.y = point.y + mp.resolution*y;
                    pt_out.z = point.z + mp.resolution*z;
                    cloud_out.push_back(pt_out);
                }
            }
        }
    }
    cloud_out.width = cloud_out.points.size();
    cloud_out.height = 1;
    cloud_out.is_dense = true;
    cloud_out.header.frame_id = mp.frame_id;

    md.cloud_inflated = cloud_out.makeShared();

    //发布点云
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*md.cloud_inflated, cloud_msg);
    map_pub.publish(cloud_msg);

    //更新膨胀过的地图，膨胀过的地图用来导航可以
    for(int i=0;i<mp.sizex*mp.sizey*mp.sizez;i++)
    {
        md.pMap_inflated[i]=0;
    }

    for(int i=0;i<cloud_out.size();i++)
    {
        Eigen::Vector3d pos_inflated;
        pos_inflated(0)=cloud_out.points[i].x;
        pos_inflated(1)=cloud_out.points[i].y;
        pos_inflated(2)=cloud_out.points[i].z;

        int index_inflated = PosToIndex(pos_inflated);
        if (index_inflated==-1)continue;

        md.pMap_inflated[index_inflated] = 1;
    }
}