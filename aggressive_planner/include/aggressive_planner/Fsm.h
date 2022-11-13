//
// Created by wangzichen on 2022/10/24.
//

#ifndef SRC_FSM_H
#define SRC_FSM_H

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <visualization_msgs/Marker.h>//TODO 用这个进行可视化,参考fastplanner

#include <aggressive_planner/GridMap.h>
#include <aggressive_planner/Astar.h>
#include <aggressive_planner/TrajOpt.h>

#define DISARMED 'a'
#define WAIT_TARGET 'b'
#define WAIT_EXE 'c'
#define EXE_TRAJ 'd'

struct State{
    Eigen::Vector3d position_;
    Eigen::Vector3d vel_;
    Eigen::Vector3d acc_;
    double yaw_;
};


struct BirdState{
    char plan_state_;
    ros::Time start_time_;

    State state_now_;//无人机当前状态
    State state_goal_;//无人机的目标状态
    State state_hold_; // 无人机悬停位置

    bool have_goal;
    bool have_traj;

    Bspline exe_traj;
};

class Fsm{
public:
    ros::NodeHandle node_;
    BirdState bird_state_;
    double default_goal_height;
    double max_yaw_rate_;
    int real_or_sim; // 1 real  0 sim

    ros::Subscriber goal_sub_,odom_sub_,mavros_state_sub_;
    ros::Publisher cmd_pub_;
    ros::Timer display_timer_,cmd_timer_,replan_timer_;

    GridMap gridMap_;
    Astar3d astar_;
    TrajOpt trajOpt_;

    Fsm(){};
    ~Fsm(){};

    void initFsm(ros::NodeHandle &node);

    void GoalCallback(const geometry_msgs::PoseStampedConstPtr &goal);
    void OdomCallback(const nav_msgs::OdometryConstPtr &odom);
    void MavrosStateCallback(const mavros_msgs::StateConstPtr &state);

    void DisplayStateTimerCb(const ros::TimerEvent&);
    void displayState();
    mavros_msgs::PositionTarget get_cmd();
    void CmdTimerCb(const ros::TimerEvent&);


};
#endif //SRC_FSM_H
