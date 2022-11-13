//
// Created by wangzichen on 2022/10/26.
//

#ifndef SRC_TRAJOPT_H
#define SRC_TRAJOPT_H
#include <nav_msgs/Path.h>
#include <aggressive_planner/GridMap.h>
#include <aggressive_planner/Bspline.h>

struct MaxAndId{
    double max_value_;
    int id_;
};



class TrajOpt{
public:
    TrajOpt(){};
    ~TrajOpt(){};

    void initTrajOpt(ros::NodeHandle &nh,MappingParameters input_mp_);
    Bspline OptimizeTraj(std::vector<Eigen::Vector3d>path,GridMap gridMap,
                         Eigen::Vector3d startV,Eigen::Vector3d goalV,
                         Eigen::Vector3d startA,Eigen::Vector3d goalA);

private:
    ros::NodeHandle node_;
    double v_max_,a_max_;
    int k;
    int cpts_adjust_epoch,time_adjust_epoch;
    double weight_obs_,weight_smooth_,weight_length_;
    double thrObs;
    double init_gap;
    MappingParameters mp_;

    nav_msgs::Path traj_;
    ros::Publisher traj_pub_;
    ros::Timer traj_timer_;

    std::vector<Eigen::Vector3d> initCpts(std::vector<Eigen::Vector3d> path);
    std::vector<double> initTime(std::vector<Eigen::Vector3d> cpts);
    std::vector<Eigen::Vector3d> adjustCpts(std::vector<Eigen::Vector3d> cpts,std::vector<double>nodeVector,GridMap gridMap,
                                            Eigen::Vector3d startV,Eigen::Vector3d goalV,
                                            Eigen::Vector3d startA,Eigen::Vector3d goalA);
    std::vector<double> adjustTime(std::vector<double> nodeVector,std::vector<Eigen::Vector3d> cpts);
    std::vector<double> adjustNodeVector(std::vector<double> nodeVector,std::vector<Eigen::Vector3d> cpts);
    std::vector<Eigen::Vector3d> calDiffPts(std::vector<Eigen::Vector3d> cpts,int k_,std::vector<double>nodeVector);

    inline Eigen::Vector3d calGrad(Eigen::Vector3d pt,double thr,pcl::KdTreeFLANN<pcl::PointXYZ> kdTree,
                                   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    inline Eigen::Vector3d calSmooth(int i,std::vector<Eigen::Vector3d>cpts);
    inline MaxAndId getMaxValueAndId(std::vector<Eigen::Vector3d> pts);

    void TrajPubTimerCallback(const ros::TimerEvent&);

};

inline Eigen::Vector3d TrajOpt::calGrad(Eigen::Vector3d pt, double thr, pcl::KdTreeFLANN<pcl::PointXYZ> kdtree,
                                        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    if (cloud->points.empty())
    {
        Eigen::Vector3d grad(0,0,0);
        return grad;
    }
    // 最近障碍物查询
    std::vector<int> pointIdxKNNSearch(1);
    std::vector<float>pointNKNSquaredDistance(1);
    pcl::PointXYZ searchPoint;
    searchPoint.x = pt(0);
    searchPoint.y = pt(1);
    searchPoint.z = pt(2);
    kdtree.nearestKSearch(searchPoint,1,pointIdxKNNSearch,pointNKNSquaredDistance);
    double dist = std::sqrt(pointNKNSquaredDistance[0]);
    Eigen::Vector3d nearob(cloud->points[pointIdxKNNSearch[0]].x, cloud->points[pointIdxKNNSearch[0]].y, cloud->points[pointIdxKNNSearch[0]].z);

    // 计算梯度
    if (dist > thr)
    {
        Eigen::Vector3d grad(0,0,0);
        return grad;
    }
    double distx = pt(0) - nearob(0);
    double disty = pt(1) - nearob(1);
    double distz = pt(2) - nearob(2);

    double grad_x = -(dist-thr)/dist*2*distx;
    double grad_y = -(dist-thr)/dist*2*disty;
    double grad_z = -(dist-thr)/dist*2*distz;

    Eigen::Vector3d grad(grad_x,grad_y,grad_z);
    return grad;

}

inline Eigen::Vector3d TrajOpt::calSmooth(int i, std::vector<Eigen::Vector3d> cpts)
{
    double smoothx = 0,smoothy = 0,smoothz = 0;
    //第一个点和最后的点不优化
    if (i == 1 || i == (int)cpts.size()-2)
    {
        smoothx = 2*(cpts[i-1](0)+cpts[i+1](0)-2*cpts[i](0));
        smoothy = 2*(cpts[i-1](1)+cpts[i+1](1)-2*cpts[i](1));
        smoothz = 2*(cpts[i-1](2)+cpts[i+1](2)-2*cpts[i](2));
    }
    else
    {
        smoothx = 2*(cpts[i-1](0)+cpts[i+1](0)-2*cpts[i](0)) - (cpts[i-2](0)+cpts[i](0)-2*cpts[i-1](0)) - (cpts[i](0)+cpts[i+2](0)-2*cpts[i+1](0));
        smoothy = 2*(cpts[i-1](1)+cpts[i+1](1)-2*cpts[i](1)) - (cpts[i-2](1)+cpts[i](1)-2*cpts[i-1](1)) - (cpts[i](1)+cpts[i+2](1)-2*cpts[i+1](1));
        smoothz = 2*(cpts[i-1](2)+cpts[i+1](2)-2*cpts[i](2)) - (cpts[i-2](2)+cpts[i](2)-2*cpts[i-1](2)) - (cpts[i](2)+cpts[i+2](2)-2*cpts[i+1](2));
    }
    Eigen::Vector3d smooth(smoothx,smoothy,smoothz);
    return smooth;
}


inline MaxAndId TrajOpt::getMaxValueAndId(std::vector<Eigen::Vector3d> pts)
{
    MaxAndId result{};
    result.id_ = -1;
    result.max_value_ = -1.0;

    double tmp_value_;

    for(int i=0;i<=pts.size()-1;i++){
        tmp_value_ = std::max(std::max(abs(pts[i](0)),abs(pts[i](1))),abs(pts[i](2)));
        if(tmp_value_ > result.max_value_){
            result.max_value_ = tmp_value_;
            result.id_ = i;
        }
    }
    return result;
}


#endif //SRC_TRAJOPT_H
