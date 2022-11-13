//
// Created by wangzichen on 2022/10/26.
//

#ifndef SRC_ASTAR_H
#define SRC_ASTAR_H
#include <iostream>
#include <queue>
#include <vector>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <unordered_map>
#include <nav_msgs/Path.h>
#include <utility>

#include <aggressive_planner/GridMap.h>

#define IN_CLOSE_SET 'a'
#define IN_OPEN_SET 'b'
#define NOT_EXPAND 'c'
#define inf 1>>30

enum PLAN_STATE{
    WAIT_FOR_TARGET,
    TARGET_UNUSEFUL,
    SEARCH_FAILED,
    NO_NEED_REPLAN,
    NEED_REPLAN,
    REPLAN_SUCCESSFULLY,
    REPLAN_FAILED
};


struct Node
{
    int index;
    Eigen::Vector3d position;
    double g_score,f_score;
    Node* parent;
    char node_state;

    Node()
    {
        parent = NULL;
        node_state=NOT_EXPAND;
    }

    ~Node()
    {}
};


struct cmp
{
    bool operator()(const Node *a,const Node *b)
    {
        return a->f_score>b->f_score;    //小顶堆
    }
};

struct OptParams
{
    int opt_epoch;
    double w_smooth;
    double w_obs;
    double w_len;
    double thr_obs;
    double thr_len;
};


class Astar3d{
public:
    ros::NodeHandle nh;
    MappingParameters mp;
    MappingData md;
    int up_flying_bound,down_flying_bound;//规划路径的时的空域,down bound加上地图发布的最低点,就是飞行最低高度.上限同理
    double interpolation_dist;
    int astar_map_sizex,astar_map_sizey,astar_map_sizez;

    OptParams opt_params;
    PLAN_STATE planState;

    int*** pMap;//用来做规划的二维数组

    std::vector<Node*>path_nodes;//原始路径,栅格坐标
    std::vector<Node*>path_cut;//剪枝后的路径,栅格坐标
    std::vector<Node*>path_interpolation;//插值后的路径,世界坐标
    std::vector<Node*>path_final;//推离障碍物后的路径,世界坐标

    std::priority_queue<Node*,std::vector<Node*>,cmp>open_set;     //将f按升序排列
    std::unordered_map<int,Node*>expand_nodes;

    bool have_path;
    bool start_available;
    bool goal_available;

    nav_msgs::Path AstarPath_raw;
    nav_msgs::Path AstarPath_interpolated;
    nav_msgs::Path AstarPath_final;
    ros::Publisher path_raw_publisher;
    ros::Publisher path_interpolated_publisher;
    ros::Publisher path_final_publisher;
    ros::Timer path_raw_timer;
    ros::Timer path_interpolated_timer;
    ros::Timer path_final_timer;

    std::vector<Eigen::Vector3d> path_for_trajOpt_;


    Astar3d(){};
    ~Astar3d(){};

    void initAstar3d(ros::NodeHandle &node,MappingParameters mp);

    void getPath(MappingData MapData,Eigen::Vector3d start_world,Eigen::Vector3d goal_world);

    void collisionCheck(MappingData MapData);
    void rePlan(MappingData MapData,Eigen::Vector3d start_world,Eigen::Vector3d goal_world);

    void getMap(MappingData MapData);
    void search(Eigen::Vector3d start_world,Eigen::Vector3d goal_world);
    void cutPath();
    void interpolatePath();
    void adjust_pts();

    int posToIndex(Eigen::Vector3d pt);
    int getDiagHeu(Eigen::Vector3d x1,Eigen::Vector3d x2);
    bool isObs(Eigen::Vector3d pos);

    bool isCollision(Eigen::Vector3d start_pt,Eigen::Vector3d goal_pt);
    bool noTo(Eigen::Vector3d pt);
    Eigen::Vector3d cal_grad(const Node* point);
    Eigen::Vector3d cal_smooth(int index);
    Eigen::Vector3d cal_length(int index,std::vector<Node*> &path);

    void retrievePath(Node* end_node);
    void reset();

    void publish_path_raw(const ros::TimerEvent&event);
    void publish_path_interpolated(const ros::TimerEvent&event);
    void publish_path_final(const ros::TimerEvent&event);

    //坐标转换等函数
    inline Eigen::Vector3d ConvertWorld2AstarMap(double world_x,double world_y,double world_z);


};
#endif //SRC_ASTAR_H
