//
// Created by wangzichen on 2022/10/26.
//

#include <aggressive_planner/TrajOpt.h>

void TrajOpt::initTrajOpt(ros::NodeHandle &nh,MappingParameters input_mp_)
{
    mp_ = input_mp_;

    node_ = nh;
    node_.param("/traj_opt/v_max",v_max_,1.5);
    node_.param("/traj_opt/a_max",a_max_,1.0);
    node_.param("/traj_opt/k",k,4);
    node_.param("/traj_opt/init_gap",init_gap,0.2);
    node_.param("/traj_opt/weight_obs",weight_obs_,0.05);
    node_.param("/traj_opt/weight_smooth",weight_smooth_,0.2);
    node_.param("/traj_opt/weight_length",weight_length_,0.025);
    node_.param("/traj_opt/thr_obs",thrObs,0.6);
    node_.param("/traj_opt/cpts_adjust_epoch",cpts_adjust_epoch,200);
    node_.param("/traj_opt/time_adjust_epoch",time_adjust_epoch,300);

    traj_.header.frame_id = input_mp_.frame_id;
    traj_pub_ = node_.advertise<nav_msgs::Path>("/traj/control_pts",10);
    traj_timer_ = node_.createTimer(ros::Duration(1.0),&TrajOpt::TrajPubTimerCallback, this);
}

Bspline TrajOpt::OptimizeTraj(std::vector<Eigen::Vector3d> path, GridMap gridMap,
                              Eigen::Vector3d startV,Eigen::Vector3d goalV,
                              Eigen::Vector3d startA, Eigen::Vector3d goalA)
{
    //step1 生成control pts,node vector
    std::vector<Eigen::Vector3d> control_pts = initCpts(path);
    std::vector<double> nodeVector = initTime(control_pts);

    //step2 调整control pts
    control_pts = adjustCpts(control_pts,nodeVector,gridMap,
                             startV,goalV,startA,goalA);

    //step3 调整node vector
    nodeVector = adjustTime(nodeVector,control_pts);
    //nodeVector = adjustNodeVector(nodeVector,control_pts);


    //step4 生成B样条
    Bspline traj;
    traj.getParams(k,control_pts,nodeVector);

    return traj;
}

std::vector<Eigen::Vector3d> TrajOpt::initCpts(std::vector<Eigen::Vector3d> path)
{
    std::vector<Eigen::Vector3d> cpts;
    Eigen::Vector3d cpt= path[0];
    cpts.push_back(cpt);
    for (int i=1; i< (int)path.size()-1;i++)
    {
        cpt = path[i];
        Eigen::Vector3d lastpt = cpts[cpts.size()-1];
        double dist = sqrt(pow(cpt(0)-lastpt(0),2)+pow(cpt(1)-lastpt(1),2)+pow(cpt(2)-lastpt(2),2));
        if (dist >= init_gap)
        {
            cpts.push_back(cpt);
        }
    }
    cpt = path[path.size()-1];
    cpts.push_back(cpt);

    std::cout<<"======================"<<std::endl;
    std::cout<<"the init control pts size is :"<<cpts.size()<<std::endl;
    for(int i=0;i<cpts.size();i++){
        std::cout<<cpts[i](0)<<","<<cpts[i](1)<<","<<cpts[i](2)<<std::endl;
    }
    std::cout<<"the above is the init control pts"<<std::endl;
    std::cout<<"======================"<<std::endl;


    return cpts;
}

//这里和原版的不太一样
std::vector<double> TrajOpt::initTime(std::vector<Eigen::Vector3d> cpts)
{
    double path_length = 0.0;
    for(int i=0;i<=(int)cpts.size()-2;i++){
        double len = sqrt(pow(cpts[i](0)-cpts[i+1](0),2) +
                          pow(cpts[i](1)-cpts[i+1](1),2) +
                          pow(cpts[i](2)-cpts[i+1](2),2) );
        path_length = path_length + len;
    }

    double tmp_t = path_length/v_max_;
    int num = cpts.size() + k + 1;

    std::vector<double> nodeVector;

    std::cout<<"k is "<<k<<std::endl;

    for(int i=1;i<=k;i++){
        nodeVector.push_back(0.0);
    }

    for(int i=0;i<=num-2*k-1;i++){
        double value = 0+i*tmp_t/(num-2*k-1);
        nodeVector.push_back(value);
    }

    for(int i=1;i<=k;i++){
        nodeVector.push_back(tmp_t);
    }

    std::cout<<"========================"<<std::endl;
    std::cout<<"the node vector size is : "<<nodeVector.size()<<std::endl;
    for(int i=0;i<=(int)nodeVector.size()-1;i++){
        std::cout<<nodeVector[i]<<std::endl;
    }
    std::cout<<"the above is the init nodevector"<<std::endl;
    std::cout<<"========================"<<std::endl;

    return nodeVector;
}

std::vector<Eigen::Vector3d> TrajOpt::adjustCpts(std::vector<Eigen::Vector3d> cpts, std::vector<double> nodeVector,
                                                 GridMap gridMap,
                                                 Eigen::Vector3d startV, Eigen::Vector3d goalV,
                                                 Eigen::Vector3d startA, Eigen::Vector3d goalA)
{
    //todo 膨胀后的点云太大了,会严重影响查询速度,考虑在数组里进行膨胀

    //step1 建立kdtree,用于进行最近点查询
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(gridMap.md.cloud_raw);

    //step2 固定的初末状态
    //step2.1 已知初始速度和初始位置,计算第二个点的坐标
    //pi+1 = pi + vi*(ui+k+1 - ui+1)/k
    cpts[1](0) = cpts[0](0) + startV(0)*(nodeVector[0+k+1]-nodeVector[0+1])/k;
    cpts[1](1) = cpts[0](1) + startV(1)*(nodeVector[0+k+1]-nodeVector[0+1])/k;
    cpts[1](2) = cpts[0](2) + startV(2)*(nodeVector[0+k+1]-nodeVector[0+1])/k;

    //step2.2 已知初始速度和初始加速度,计算第二个速度点
    //vi+1 = vi + ai*(ui+k+1 - ui+2)/(k-1)
    Eigen::Vector3d secondV(0.0,0.0,0.0);
    secondV(0) = startV(0) + startA(0)*(nodeVector[0+k+1] - nodeVector[0+2])/(k-1);
    secondV(1) = startV(1) + startA(1)*(nodeVector[0+k+1] - nodeVector[0+2])/(k-1);
    secondV(2) = startV(2) + startA(2)*(nodeVector[0+k+1] - nodeVector[0+2])/(k-1);
    //step2.3 已知第二个速度点,第二个位置点,计算第三个位置控制点
    //pi+1 = pi + vi*(ui+k+1 - ui+1)/k
    cpts[2](0) = cpts[1](0) + secondV(0)*(nodeVector[1+k+1] - nodeVector[1+1])/k;
    cpts[2](1) = cpts[1](1) + secondV(1)*(nodeVector[1+k+1] - nodeVector[1+1])/k;
    cpts[2](2) = cpts[1](2) + secondV(2)*(nodeVector[1+k+1] - nodeVector[1+1])/k;

    int num = cpts.size();
    //step2.4 已知末速度,末位置,计算倒数第二个点的坐标
    //pi = pi+1 - vi*(ui+k+1 - ui+1)/k
    cpts[num-2](0) = cpts[num-1](0) - goalV(0)*(nodeVector[num-2+k+1] - nodeVector[num-2+1])/k;
    cpts[num-2](1) = cpts[num-1](1) - goalV(1)*(nodeVector[num-2+k+1] - nodeVector[num-2+1])/k;
    cpts[num-2](2) = cpts[num-1](2) - goalV(2)*(nodeVector[num-2+k+1] - nodeVector[num-2+1])/k;

    //step2.5 由末端加速度,末端速度计算倒数第二速度位置
    //vi = vi+1 - ai*(ui+k+1 - ui+2)/(k-1)
    Eigen::Vector3d secondLastV(0.0,0.0,0.0);
    secondLastV(0) = goalV(0) - goalA(0)*(nodeVector[num-3+k+1] - nodeVector[num-3+2])/(k-1);
    secondLastV(1) = goalV(1) - goalA(1)*(nodeVector[num-3+k+1] - nodeVector[num-3+2])/(k-1);
    secondLastV(2) = goalV(2) - goalA(2)*(nodeVector[num-3+k+1] - nodeVector[num-3+2])/(k-1);

    //step2.6 由倒数第二点位置,倒数第二点速度,计算倒数第三点位置
    //pi = pi+1 - vi*(ui+k+1 - ui+1)/k
    cpts[num-3](0) = cpts[num-2](0) - secondLastV(0)*(nodeVector[num-3+k+1] - nodeVector[num-3+1])/k;
    cpts[num-3](1) = cpts[num-2](1) - secondLastV(1)*(nodeVector[num-3+k+1] - nodeVector[num-3+1])/k;
    cpts[num-3](2) = cpts[num-2](2) - secondLastV(2)*(nodeVector[num-3+k+1] - nodeVector[num-3+1])/k;

    //step3 安全性和光滑性调节
    for(int epoch = 0;epoch<=cpts_adjust_epoch;epoch++){
        for(int i=3;i<=(int)cpts.size()-4;i++){
            Eigen::Vector3d grad = calGrad(cpts[i],thrObs,kdtree,gridMap.md.cloud_raw);
            Eigen::Vector3d smooth = calSmooth(i,cpts);
            Eigen::Vector3d pt = cpts[i];

            pt(0) = pt(0) + weight_obs_*grad(0) + weight_smooth_*smooth(0);
            pt(1) = pt(1) + weight_obs_*grad(1) + weight_smooth_*smooth(1);
            pt(2) = pt(2) + weight_obs_*grad(2) + weight_smooth_*smooth(2);

            // todo 判断不会与障碍物碰撞，才进行更新
            if (true){
                cpts[i] = pt;
            }
        }
    }

    std::cout<<"======================"<<std::endl;
    std::cout<<"the control pts size is :"<<cpts.size()<<std::endl;
    for(int i=0;i<cpts.size();i++){
        std::cout<<cpts[i](0)<<","<<cpts[i](1)<<","<<cpts[i](2)<<std::endl;
    }
    std::cout<<"the above is the control pts"<<std::endl;
    std::cout<<"======================"<<std::endl;


    traj_.poses.clear();
    for(auto & i : cpts)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.pose.position.x = i(0);
        pose_stamped.pose.position.y = i(1);
        pose_stamped.pose.position.z = i(2);

        pose_stamped.pose.orientation.x = 0;
        pose_stamped.pose.orientation.y = 0;
        pose_stamped.pose.orientation.z = 0;
        pose_stamped.pose.orientation.w = 1;

        traj_.poses.push_back(pose_stamped);
    }
    traj_pub_.publish(traj_);
    return cpts;
}

std::vector<double> TrajOpt::adjustTime(std::vector<double> nodeVector, std::vector<Eigen::Vector3d> cpts)
{
    int num_cpts = cpts.size();
    int num_vpts = num_cpts - 1;
    int num_ats = num_vpts - 1;

    for(int epoch = 0;epoch<time_adjust_epoch;epoch++){
        //step1 计算速度控制点
        for(int i=2;i<=num_vpts-3;i++){
            //计算第i个速度控制点
            Eigen::Vector3d V_i(0.0,0.0,0.0);
            //V_i = k*(Q_i+1 - Q_i)/(u_i+p+1 + u_i+1)
            //这里取i-1是因为vector的索引是从0开始的
            V_i(0) = k*(cpts[i+1](0) - cpts[i](0))/(nodeVector[i+k+1] + nodeVector[i+1]);
            V_i(1) = k*(cpts[i+1](1) - cpts[i](1))/(nodeVector[i+k+1] + nodeVector[i+1]);
            V_i(2) = k*(cpts[i+1](2) - cpts[i](2))/(nodeVector[i+k+1] + nodeVector[i+1]);
            double maxv = std::max(std::max(abs(V_i(0)),abs(V_i(1))),abs(V_i(2)));
            if(maxv <= v_max_){
                continue;
            }
            double dt = (nodeVector[i+k+1] - nodeVector[i+1])*(maxv/v_max_ - 1.0);
            std::cout<<"adjust v dt is :"<<dt<<std::endl;
            //todo 这里的时间是不是不对是不是要均匀的增大
            for(int j = i+k+1;j<(int)nodeVector.size();j++){
                nodeVector[j] = nodeVector[j] + dt;
            }
        }

        //计算新的速度控制点
        std::vector<Eigen::Vector3d> cvts;
        for(int i = 1;i <= num_vpts;i++){
            Eigen::Vector3d V_i(0,0,0);
            V_i(0) = k*(cpts[i-1+1](0) - cpts[i-1](0))/(nodeVector[i-1+k+1] + nodeVector[i-1+1]);
            V_i(1) = k*(cpts[i-1+1](1) - cpts[i-1](1))/(nodeVector[i-1+k+1] + nodeVector[i-1+1]);
            V_i(2) = k*(cpts[i-1+1](2) - cpts[i-1](2))/(nodeVector[i-1+k+1] + nodeVector[i-1+1]);
            cvts.push_back(V_i);
        }

        //计算加速度控制点并进行调整
//        for(int i = 3;i <= num_ats - 2;i++){
//            Eigen::Vector3d A_i(0.0,0.0,0.0);
//            //todo 直接写成这样可以吗
//            //A_i = (k-1)*(cvts[i-1+1] - cvts[i-1])/(nodeVector[i-1+k+1] - nodeVector[i-1+2]);
//            A_i(0) = (k-1)*(cvts[i-1+1](0) - cvts[i-1](0))/(nodeVector[i-1+k+1] - nodeVector[i-1+2]);
//            A_i(1) = (k-1)*(cvts[i-1+1](1) - cvts[i-1](1))/(nodeVector[i-1+k+1] - nodeVector[i-1+2]);
//            A_i(2) = (k-1)*(cvts[i-1+1](2) - cvts[i-1](2))/(nodeVector[i-1+k+1] - nodeVector[i-1+2]);
//            double maxa = std::max(std::max(abs(A_i(0)),abs(A_i(1))),abs(A_i(2)));
//            if(maxa <= a_max_){
//                continue;
//            }
//            double dt = (nodeVector[i-1+k+1] - nodeVector[i-1+2])*(maxa/a_max_ - 1.0);
//            std::cout<<"adjust a dt is :"<<dt<<std::endl;
//            for(int j = i-1+k+1;j < (int)nodeVector.size();j++){
//                nodeVector[j] = nodeVector[j] + dt;
//            }
//        }
        //todo a2 和倒数第二个a需要单独调整
    }

    std::cout<<"============"<<std::endl;

    for(int i=0;i<=(int)nodeVector.size()-1;i++){
        std::cout<<nodeVector[i]<<std::endl;
    }
    std::cout<<"the above is the adjust done nodevector"<<std::endl;

    return nodeVector;
}


std::vector<double> TrajOpt::adjustNodeVector(std::vector<double> nodeVector, std::vector<Eigen::Vector3d> cpts)
{
    std::cout<<"here in the new adjust time!"<<std::endl;
    double max_v_ = v_max_*2.0;
    double max_a_ = a_max_*2.0;
    MaxAndId tmp_v_{};
    MaxAndId tmp_a_{};
    std::vector<Eigen::Vector3d> vts;
    std::vector<Eigen::Vector3d> ats;
    std::vector<double> velNodeVector;

    while (max_v_ > v_max_*1.01 || max_a_ > a_max_*1.01){
        while (max_v_ > v_max_ * 1.01){
            vts.clear();
            vts = calDiffPts(cpts,k,nodeVector);
            tmp_v_ = getMaxValueAndId(vts);
            max_v_ = tmp_v_.max_value_;
            if(max_v_ > v_max_){
                double addt = (max_v_/v_max_ - 1.0)*(nodeVector[tmp_v_.id_+k+1] - nodeVector[tmp_v_.id_+1]);
                std::cout<<"the v addt is : "<<addt<<std::endl;
                for(int i=tmp_v_.id_+k+1;i<=nodeVector.size()-1;i++){
                    nodeVector[i] = nodeVector[i]+addt;
                }
            }
        }

        vts.clear();
        vts = calDiffPts(cpts,k,nodeVector);
        velNodeVector.clear();
        for(int i=1;i<=nodeVector.size()-2;i++){
            velNodeVector.push_back(nodeVector[i]);
        }

        std::cout<<"======================"<<std::endl;
        std::cout<<"the control vts size is :"<<vts.size()<<std::endl;
        for(int i=0;i<vts.size();i++){
            std::cout<<vts[i](0)<<","<<vts[i](1)<<","<<vts[i](2)<<std::endl;
        }
        std::cout<<"the above is the control vts"<<std::endl;
        std::cout<<"======================"<<std::endl;



        while (max_a_ > a_max_ * 1.01){
            ats = calDiffPts(vts,k-1,velNodeVector);
            tmp_a_ = getMaxValueAndId(ats);
            max_a_ = tmp_a_.max_value_;
            if(max_a_ > a_max_){
                double addt = (max_a_/a_max_-1.0)*(velNodeVector[tmp_a_.id_+k+1] - velNodeVector[tmp_a_.id_+1]);
                std::cout<<"the a addt is : "<<addt<<std::endl;
                for(int i=tmp_a_.id_+k;i<=velNodeVector.size()-1;i++){
                    velNodeVector[i] = velNodeVector[i] + addt;
                }
            }
        }

        nodeVector.clear();
        for(int i=0;i<=velNodeVector.size()-1;i++){
            nodeVector.push_back(velNodeVector[i]);
        }
        nodeVector.push_back(velNodeVector[velNodeVector.size()-1]);
        vts.clear();
        vts = calDiffPts(cpts,k,nodeVector);
        tmp_v_ = getMaxValueAndId(vts);
        max_v_ = tmp_v_.max_value_;

        ats = calDiffPts(vts,k-1,velNodeVector);
        tmp_a_ = getMaxValueAndId(ats);
        max_a_ = tmp_a_.max_value_;
    }
    std::cout<<"============"<<std::endl;

    for(int i=0;i<=(int)nodeVector.size()-1;i++){
        std::cout<<nodeVector[i]<<std::endl;
    }
    std::cout<<"the above is the adjust done nodevector"<<std::endl;
    std::cout<<"the adjust done nodevector size is :"<<nodeVector.size()<<std::endl;
    return nodeVector;
}




std::vector<Eigen::Vector3d> TrajOpt::calDiffPts(std::vector<Eigen::Vector3d> cpts,int k_, std::vector<double> nodeVector)
{
    std::vector<Eigen::Vector3d> diffPts;
    Eigen::Vector3d diff_pt_;
    for(int i=0;i<=cpts.size()-2;i++){
        diff_pt_[0] = k_ * (cpts[i+1](0) - cpts[i](0))/(nodeVector[i+k_+1] - nodeVector[i+1]);
        diff_pt_[1] = k_ * (cpts[i+1](1) - cpts[i](1))/(nodeVector[i+k_+1] - nodeVector[i+1]);
        diff_pt_[2] = k_ * (cpts[i+1](2) - cpts[i](2))/(nodeVector[i+k_+1] - nodeVector[i+1]);
        diffPts.push_back(diff_pt_);
    }
    return diffPts;
}









void TrajOpt::TrajPubTimerCallback(const ros::TimerEvent &)
{
    if (traj_.poses.empty()){
        return;
    }
    traj_pub_.publish(traj_);
}