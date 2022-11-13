//
// Created by wangzichen on 2022/10/26.
//
//std::cout<<"\033[7m"<<"\033[31m修改"<<"\033[0m"<< std::endl;//red 7m开启反显 0m关闭效果
//std::cout<<"\033[7m"<<"\033[32m修改"<<"\033[0m"<< std::endl;//green
//std::cout<<"\033[7m"<<"\033[33m修改"<<"\033[0m"<< std::endl;//yellow

#include <aggressive_planner/Fsm.h>

void Fsm::initFsm(ros::NodeHandle &nh)
{
    node_ = nh;

    node_.param("/fsm/default_goal_height",default_goal_height,1.2);
    node_.param("/real_or_sim",real_or_sim,1);
    node_.param("/max_yaw_rate",max_yaw_rate_,1.0);

    gridMap_.initMap(node_);
    astar_.initAstar3d(node_,gridMap_.mp);
    trajOpt_.initTrajOpt(node_,gridMap_.mp);

    goal_sub_ = node_.subscribe<geometry_msgs::PoseStamped>
            ("/move_base_simple/goal",10,&Fsm::GoalCallback, this);
    if(real_or_sim == REAL_FLIGHT_VIO_T265){
        odom_sub_ = node_.subscribe<nav_msgs::Odometry>
                ("/t265/odom/sample",10,&Fsm::OdomCallback, this);
    } else if (real_or_sim == SIM_FLIGHT){
        odom_sub_ = node_.subscribe<nav_msgs::Odometry>
                ("/mavros/local_position/odom",10,&Fsm::OdomCallback, this);
    } else{
        std::cout<<"\033[7m"<<"\033[31m[FSM]State error!"<<"\033[0m"<< std::endl;//red
    }

    mavros_state_sub_ = node_.subscribe<mavros_msgs::State>
            ("/mavros/state",10,&Fsm::MavrosStateCallback, this);

    cmd_pub_ = node_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local",10);
    cmd_timer_ = node_.createTimer(ros::Duration(0.02),&Fsm::CmdTimerCb, this);

    display_timer_ = node_.createTimer(ros::Duration(1.0),&Fsm::DisplayStateTimerCb, this);


    bird_state_.have_goal = false;
    bird_state_.have_traj = false;
    bird_state_.plan_state_ = DISARMED;
}

void Fsm::GoalCallback(const geometry_msgs::PoseStampedConstPtr &goal)
{
    bird_state_.state_goal_.position_<<goal->pose.position.x,goal->pose.position.y,default_goal_height;
    bird_state_.state_goal_.vel_<<0.0,0.0,0.0;
    bird_state_.state_goal_.acc_<<0.0,0.0,0.0;

    bird_state_.have_goal = true;
    std::cout<<"[FSM]The goal now is : ["<<bird_state_.state_goal_.position_(0)<<","<<
                                           bird_state_.state_goal_.position_(1)<<","<<
                                           bird_state_.state_goal_.position_(2)<<"]."<<std::endl;


    //如果当前状态是未解锁,那么只进行路径规划的display,不会更新bird state中的内容
    if(bird_state_.plan_state_ == DISARMED){
        std::cout<<"[FSM]DISARMED,just show the traj."<<std::endl;
        // step1 find path
        ros::Time t1 = ros::Time::now();
        Eigen::Vector3d startP = bird_state_.state_now_.position_;
        Eigen::Vector3d goalP = bird_state_.state_goal_.position_;
        astar_.getPath(gridMap_.md,startP,goalP);
        if(!astar_.have_path)return;

        // step2 opt traj
        ros::Time t2 = ros::Time::now();

        Eigen::Vector3d startV(0.0,0.0,0.0);
        Eigen::Vector3d goalV(0.0,0.0,0.0);
        Eigen::Vector3d startA(0.0,0.0,0.0);
        Eigen::Vector3d goalA(0.0,0.0,0.0);

        Bspline traj = trajOpt_.OptimizeTraj(astar_.path_for_trajOpt_,gridMap_,
                                             startV,goalV,startA,goalA);

        traj.displayBspline();

        ros::Time t_end = ros::Time::now();

        std::cout<<"\033[7m"<<"\033[32m";
        std::cout<<"Path search "<<(t2-t1).toSec()<<" || ";
        std::cout<<"Time consumption total "<<(t_end-t1).toSec();
        std::cout<<"\033[0m"<<std::endl;
        return;
    }

    //如果当前状态不是DISAMRED,则正常进行路径搜索
    Eigen::Vector3d startP = bird_state_.state_now_.position_;
    Eigen::Vector3d goalP = bird_state_.state_goal_.position_;
    Eigen::Vector3d startV(0.0,0.0,0.0);
    Eigen::Vector3d goalV(0.0,0.0,0.0);
    Eigen::Vector3d startA(0.0,0.0,0.0);
    Eigen::Vector3d goalA(0.0,0.0,0.0);
    if(bird_state_.plan_state_ == EXE_TRAJ){
        double time = std::min((ros::Time::now() - bird_state_.start_time_).toSec()+0.1, bird_state_.exe_traj.total_time_-0.001);
        startP = bird_state_.exe_traj.getPos(time);
        startV = bird_state_.exe_traj.getVel(time);
        startA = bird_state_.exe_traj.getAcc(time);
    }

    // step1 find path
    ros::Time t1 = ros::Time::now();
    astar_.getPath(gridMap_.md,bird_state_.state_now_.position_,bird_state_.state_goal_.position_);
    if(!astar_.have_path)return;

    // step2 opt traj
    ros::Time t2 = ros::Time::now();
    Bspline traj = trajOpt_.OptimizeTraj(astar_.path_for_trajOpt_,gridMap_,
                                         startV,goalV,startA,goalA);

    ros::Time t_end = ros::Time::now();

    std::cout<<"\033[7m"<<"\033[32m";
    std::cout<<"Path search "<<(t2-t1).toSec()<<" || ";
    std::cout<<"Traj "<<(t_end-t2).toSec()<<" || ";
    std::cout<<"Time consumption total "<<(t_end-t1).toSec();
    std::cout<<"\033[0m"<<std::endl;

    // step change state,start exe
    // 如果上一状态是等待目标,或者正在执行初始转向,就更新traj,然后继续将状态设置为wait exe
    if(bird_state_.plan_state_ == WAIT_TARGET || bird_state_.plan_state_ == WAIT_EXE){
        bird_state_.plan_state_ = WAIT_EXE;
        bird_state_.start_time_ = ros::Time::now();
        bird_state_.have_traj = true;
        bird_state_.exe_traj = traj;
        Eigen::Vector3d yaw_vel_ = bird_state_.exe_traj.getVel(bird_state_.exe_traj.total_time_-0.1);
        bird_state_.state_goal_.yaw_ = atan2(yaw_vel_(1),yaw_vel_(0));
        return;
    }

    // 如果上一状态是执行中,难么该状态继续设置为执行中,然后更行traj
    if(bird_state_.plan_state_ == EXE_TRAJ){
        bird_state_.plan_state_ = EXE_TRAJ;
        bird_state_.start_time_ = ros::Time::now();
        bird_state_.have_traj = true;
        bird_state_.exe_traj = traj;
        Eigen::Vector3d yaw_vel_ = bird_state_.exe_traj.getVel(bird_state_.exe_traj.total_time_-0.1);
        bird_state_.state_goal_.yaw_ = atan2(yaw_vel_(1),yaw_vel_(0));
    }
}

void Fsm::OdomCallback(const nav_msgs::OdometryConstPtr &odom)
{
    bird_state_.state_now_.position_(0) = odom->pose.pose.position.x;
    bird_state_.state_now_.position_(1) = odom->pose.pose.position.y;
    bird_state_.state_now_.position_(2) = odom->pose.pose.position.z;

    //这个是机体系的,不对的是
//    bird_state_.state_now_.vel_(0) = odom->twist.twist.linear.x;
//    bird_state_.state_now_.vel_(1) = odom->twist.twist.linear.y;
//    bird_state_.state_now_.vel_(2) = odom->twist.twist.linear.z;

    double roll, pitch, yaw;
    tf::Quaternion q;
    tf::quaternionMsgToTF(odom->pose.pose.orientation, q);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    bird_state_.state_now_.yaw_ = yaw;

    if(bird_state_.plan_state_ == EXE_TRAJ){
        Eigen::Vector3d err_ = bird_state_.state_now_.position_ - bird_state_.state_goal_.position_;
        double dist = abs(err_(0)) + abs(err_(1)) + abs(err_(2));
        double time = (ros::Time::now() - bird_state_.start_time_).toSec();
        if(dist<0.2 || time >= bird_state_.exe_traj.total_time_ -0.2){
            bird_state_.state_hold_.position_ = bird_state_.state_now_.position_;
            bird_state_.state_hold_.yaw_ = bird_state_.state_now_.yaw_;
            bird_state_.plan_state_ = WAIT_TARGET;
        }
    }
}

void Fsm::MavrosStateCallback(const mavros_msgs::StateConstPtr &state)
{
    //没有解锁和没有进offboard两个但凡有一个没有满足,就进入这个if中,更新state hold,保持DISARMD状态
    if(!(state->armed && state->mode == "OFFBOARD")){
        bird_state_.plan_state_ = DISARMED;
        bird_state_.state_hold_ = bird_state_.state_now_;
    }
    //如果满足了进入armed和offboard两个状态,可以让状态转换为WAIT TARGET
    else{
        if(bird_state_.plan_state_ == DISARMED)bird_state_.plan_state_ = WAIT_TARGET;
    }
}


//todo here
mavros_msgs::PositionTarget Fsm::get_cmd()
{
    //没有设置超出时间,因为临近末尾点的时候会直接设置为终点
    mavros_msgs::PositionTarget cmd;
    cmd.coordinate_frame = 1;

    //如果是还没解锁,也没进入offboard,那么就是DISARMED,就将当前位置赋值给goal
    //如果进入了offboard,那么现在就是在wait target,此时不再更新state hold,只进行发布
    if(bird_state_.plan_state_ == DISARMED||bird_state_.plan_state_ == WAIT_TARGET){
        cmd.position.x = bird_state_.state_hold_.position_(0);
        cmd.position.y = bird_state_.state_hold_.position_(1);
        cmd.position.z = bird_state_.state_hold_.position_(2);

        cmd.velocity.x = 0.0;
        cmd.velocity.y = 0.0;
        cmd.velocity.z = 0.0;

        cmd.acceleration_or_force.x = 0.0;
        cmd.acceleration_or_force.y = 0.0;
        cmd.acceleration_or_force.z = 0.0;

        cmd.yaw = bird_state_.state_hold_.yaw_;
        cmd.yaw_rate = 0.0;

        cmd.type_mask = 0;
    }

    //静止情况下,已经有了目标点,先完成转向
    if(bird_state_.plan_state_ == WAIT_EXE){
        double time = 0.0;
        Eigen::Vector3d uav_pos_ = bird_state_.exe_traj.getPos(time);

        cmd.position.x = uav_pos_(0);
        cmd.position.y = uav_pos_(1);
        cmd.position.z = uav_pos_(2);

        cmd.velocity.x = 0.0;
        cmd.velocity.y = 0.0;
        cmd.velocity.z = 0.0;

        cmd.acceleration_or_force.x = 0.0;
        cmd.acceleration_or_force.y = 0.0;
        cmd.acceleration_or_force.z = 0.0;

        Eigen::Vector3d uav_vel_ = bird_state_.exe_traj.getVel(0.1);

        double yaw_target_ = atan2(uav_vel_(1),uav_vel_(0));
        double yaw_now_ = bird_state_.state_now_.yaw_;
        double yaw_err_ = yaw_target_ - yaw_now_;

        if(yaw_err_ > 3.1415926){
            yaw_err_ = yaw_err_ - 3.1415926*2;
        }
        if(yaw_err_ < -3.1415926){
            yaw_err_ = yaw_err_ + 3.1415926*2;
        }
        cmd.yaw_rate = std::min(std::max(yaw_err_,-max_yaw_rate_),+max_yaw_rate_);
        cmd.type_mask = 1024;

        if(abs(yaw_err_) < 3.1415926/12){
            bird_state_.plan_state_ = EXE_TRAJ;
            bird_state_.start_time_ = ros::Time::now();
        }
    }

    //执行轨迹中
    if(bird_state_.plan_state_ == EXE_TRAJ){
        double time = (ros::Time::now() - bird_state_.start_time_).toSec();
        Eigen::Vector3d uav_pos_ = bird_state_.exe_traj.getPos(time);
        cmd.position.x = uav_pos_(0);
        cmd.position.y = uav_pos_(1);
        cmd.position.z = uav_pos_(2);

        Eigen::Vector3d uav_vel_ = bird_state_.exe_traj.getVel(time);
        cmd.velocity.x = uav_vel_(0);
        cmd.velocity.y = uav_vel_(1);
        cmd.velocity.z = uav_vel_(2);

        Eigen::Vector3d uav_acc_ = bird_state_.exe_traj.getAcc(time);
        cmd.acceleration_or_force.x = uav_acc_(0);
        cmd.acceleration_or_force.y = uav_acc_(1);
        cmd.acceleration_or_force.z = uav_acc_(2);

        cmd.yaw = atan2(uav_vel_(1),uav_vel_(0));
        // 防止初速度为0，导致角度计算失败
        if(time < 0.1)
        {
            Eigen::Vector3d uavvel = bird_state_.exe_traj.getVel(0.1);
            cmd.yaw = atan2(uavvel(1),uavvel(0));
        }
        cmd.type_mask = 2048;
    }
    return cmd;
}


void Fsm::CmdTimerCb(const ros::TimerEvent &)
{
    mavros_msgs::PositionTarget cmd = get_cmd();
//    std::cout<<"going to :"<<cmd.position.x<<","<<cmd.position.y<<","<<cmd.position.z<<std::endl;
//    std::cout<<"vel is :"<<cmd.velocity.x<<","<<cmd.velocity.y<<","<<cmd.velocity.z<<std::endl;
//    std::cout<<"acc is :"<<cmd.acceleration_or_force.x<<","<<cmd.acceleration_or_force.y<<","<<cmd.acceleration_or_force.z<<std::endl;
    cmd_pub_.publish(cmd);
}





void Fsm::DisplayStateTimerCb(const ros::TimerEvent &)
{
    displayState();
}

void Fsm::displayState()
{
//    std::cout<<"====="<<std::endl;
//    std::cout<<"\033[7m"<<"\033[31m修改"<<"\033[0m"<< std::endl;//red 7m开启反显 0m关闭效果
//    std::cout<<"\033[7m"<<"\033[32m修改"<<"\033[0m"<< std::endl;//green
//    std::cout<<"\033[7m"<<"\033[33m修改"<<"\033[0m"<< std::endl;//yellow
}
