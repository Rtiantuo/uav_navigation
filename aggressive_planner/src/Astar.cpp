//
// Created by wangzichen on 2022/10/26.
//

//TODO 启发函数有错的

//TODO 设置是否搜索成功的flag

#include <aggressive_planner/Astar.h>

void Astar3d::initAstar3d(ros::NodeHandle &node, MappingParameters MP)
{
    std::cout<<"[ASTAR]Using the Astar3D."<<std::endl;

    nh.param("/Astar3d/up_flying_bound", this->up_flying_bound,60);//第20层,代表的高度就是大概map low bound 加上 20 * resolution
    nh.param("/Astar3d/down_flying_bound", this->down_flying_bound,0);//当前只能是0,代表的是地图的第一层,即map low bound
    nh.param("/Astar3d/interpolation_dist", this->interpolation_dist,0.2);
    nh.param("/Astar3d/opt_params/adjust_epoch", this->opt_params.opt_epoch,100);
    nh.param("/Astar3d/opt_params/w_smooth", this->opt_params.w_smooth,0.1);
    nh.param("/Astar3d/opt_params/w_obs", this->opt_params.w_obs,0.02);
    nh.param("/Astar3d/opt_params/w_len", this->opt_params.w_len,0.05);
    nh.param("/Astar3d/opt_params/thr_obs", this->opt_params.thr_obs,0.6);
    nh.param("/Astar3d/opt_params/thr_len", this->opt_params.thr_len,0.1);

    this->nh = node;
    this->mp = std::move(MP);

    this->have_path = false;
    this->start_available = true;
    this->goal_available = true;
    this->planState = NO_NEED_REPLAN;
    this->astar_map_sizex = mp.sizex;
    this->astar_map_sizey = mp.sizey;
    this->astar_map_sizez = up_flying_bound - down_flying_bound + 1;

    path_raw_publisher = nh.advertise<nav_msgs::Path>("/Astar3d/path_raw",10);
    path_interpolated_publisher = nh.advertise<nav_msgs::Path>("/Astar3d/path_interpolated",10);
    path_final_publisher = nh.advertise<nav_msgs::Path>("/Astar3d/path_final",10);

    path_raw_timer = nh.createTimer(ros::Duration(0.1),&Astar3d::publish_path_raw, this);
    path_interpolated_timer = nh.createTimer(ros::Duration(0.1),&Astar3d::publish_path_interpolated, this);
    path_final_timer = nh.createTimer(ros::Duration(0.1),&Astar3d::publish_path_final, this);

    AstarPath_raw.header.frame_id = mp.frame_id;
    AstarPath_interpolated.header.frame_id = mp.frame_id;
    AstarPath_final.header.frame_id = mp.frame_id;

    //三维数组,规划用的地图
    pMap=(int***)malloc(astar_map_sizex * sizeof(int*));
    for(int i=0;i<astar_map_sizex;++i)
    {
        pMap[i]=(int**)malloc(astar_map_sizey * sizeof(int*));
    }
    for(int i=0;i<astar_map_sizex;++i)
    {
        for(int j=0;j<astar_map_sizey;++j)
        {
            pMap[i][j]=(int*)malloc(astar_map_sizez*sizeof(int));
        }
    }
}

void Astar3d::getPath(MappingData MapData, Eigen::Vector3d start_world, Eigen::Vector3d goal_world)
{
    reset();
    ros::Time start_search = ros::Time::now();
    getMap(std::move(MapData));
    Eigen::Vector3d start = ConvertWorld2AstarMap(start_world(0),start_world(1),start_world(2));
    Eigen::Vector3d goal = ConvertWorld2AstarMap(goal_world(0),goal_world(1),goal_world(2));
//    if (isObs(start)){
//        std::cout<<"[PATH]The start is obs.Please move the bird."<<std::endl;
//        have_path = false;
//        return;
//    }
    if (isObs(goal)){
        std::cout<<"\033[7m"<<"\033[31m[PATH]The goal is obs.Please give another goal point."<<"\033[0m"<< std::endl;
        have_path = false;
        return;
    }

    search(std::move(start_world),std::move(goal_world));
    cutPath();
    interpolatePath();
    //adjust_pts();
    if(AstarPath_raw.poses.empty())
    {
        std::cout<<"\033[7m"<<"\033[31m[PATH]Search failed."<<"\033[0m"<< std::endl;
        have_path = false;
        return;
    }
    ros::Time end_search = ros::Time::now();

//    std::cout<<"\033[7m"<<"\033[32m";
//    std::cout<<"[ASTAR]Path size "<<path_for_trajOpt_.size()<<" || ";
//    std::cout<<"Time consumption "<<(end_search-start_search).toSec();
//    std::cout<<"\033[0m"<<std::endl;

    planState = NO_NEED_REPLAN;
}

void Astar3d::collisionCheck(MappingData MapData)
{
    if (!have_path)return;
    getMap(std::move(MapData));
    for(auto & pose : AstarPath_final.poses)
    {
        Eigen::Vector3d tmp_pt;
        tmp_pt = ConvertWorld2AstarMap(pose.pose.position.x,
                                       pose.pose.position.y,
                                       pose.pose.position.z);
        if(isObs(tmp_pt))
        {
            planState = NEED_REPLAN;
            ROS_WARN_STREAM("[COLLISION]Collision will happens, need replan.");
            return;
        }
    }
    planState = NO_NEED_REPLAN;
}

void Astar3d::rePlan(MappingData MapData,Eigen::Vector3d start_world,Eigen::Vector3d goal_world)
{
    if(planState == NEED_REPLAN)
    {
        getPath(std::move(MapData),std::move(start_world),std::move(goal_world));
        if(AstarPath_final.poses.empty())
        {
            ROS_WARN_STREAM("[REPLAN]Replan failed.");
            planState = REPLAN_FAILED;
        } else{
            ROS_INFO_STREAM("[REPLAN]Replan done");
            planState = REPLAN_SUCCESSFULLY;
        }
        return;
    } else{
        return;
    }
}

void Astar3d::getMap(MappingData MapData)
{
    this->md = std::move(MapData);
    this->md.kdTree.setInputCloud(this->md.cloud_raw);

    //初始化三维地图
    for(int x=0;x<astar_map_sizex;x++)
    {
        for(int y=0;y<astar_map_sizey;y++)
        {
            for(int z= down_flying_bound;z<=up_flying_bound;z++)
            {
                int index = z * mp.sizex * mp.sizey + y * mp.sizex + x;
                if(md.pMap_inflated[index] > 0 ){
                    pMap[x][y][z] = 1;
                } else{
                    pMap[x][y][z] = 0;
                }
            }
        }
    }
}

void Astar3d::search(Eigen::Vector3d start_world,Eigen::Vector3d goal_world)
{
    Eigen::Vector3d start = ConvertWorld2AstarMap(start_world(0),start_world(1),start_world(2));
    Eigen::Vector3d goal = ConvertWorld2AstarMap(goal_world(0),goal_world(1),goal_world(2));
    Node* cur_node = new Node();
    cur_node->parent = nullptr;
    cur_node->position = start;
    cur_node->index = posToIndex(start);
    cur_node->g_score = 0.0;
    cur_node->f_score = getDiagHeu(start,goal);
    cur_node->node_state = IN_OPEN_SET;

    open_set.push(cur_node);
    expand_nodes[cur_node->index] = cur_node;

    while(!open_set.empty())
    {
        cur_node = open_set.top();
        cur_node->node_state = IN_CLOSE_SET;
        open_set.pop();

        // 判断是否到达
        bool reach_end = abs(cur_node->position(0)-goal(0))<0.1&&abs(cur_node->position(1)-goal(1))<0.1;
        if(reach_end)
        {
            retrievePath(cur_node);
            break;
        }

        Eigen::Vector3d cur_pos = cur_node->position;
        Eigen::Vector3d pro_pos;
        Eigen::Vector3d d_pos;
        //对当前节点进行扩展
        for(int dx = -1;dx<=1;dx = dx+1)
        {
            for(int dy = -1;dy<=1;dy = dy+1)
            {
                for(int dz = -1;dz<=1; dz = dz+1)
                {
                    d_pos<<dx,dy,dz;
                    //去除当前节点，因为当前节点已经扩展过了
                    if(dx==0&&dy==0)continue;
                    pro_pos = cur_pos+d_pos;
                    //排除障碍物的点
                    if(isObs(pro_pos))continue;
                    //计算当前扩展点的cost
                    double tmp_g_score,tmp_f_score;
                    tmp_g_score = d_pos.squaredNorm()+cur_node->g_score;
                    //cout<<"  g_score: "<<tmp_g_score<<endl;
                    tmp_f_score = tmp_g_score+getDiagHeu(pro_pos,goal);
                    //cout<<" h_score:"<<getDiagHeu(pro_pos,goal)<<endl;
                    //判断当前扩展节点的状态
                    int idx = posToIndex(pro_pos);
                    Node* pro_node = nullptr;

                    //如果在close list中找到，即在expanded中有，即为close，continue
                    if(expand_nodes.find(idx)!=expand_nodes.end())
                    {
                        pro_node = expand_nodes[idx];
                    }

                    //如果没找到，即代表该节点没有被扩展  追加到openLIST
                    if (pro_node == nullptr)
                    {
                        pro_node = new Node();
                        pro_node->parent = cur_node;
                        pro_node->position = pro_pos;
                        pro_node->index = posToIndex(pro_pos);
                        pro_node->g_score = tmp_g_score;
                        pro_node->f_score = tmp_f_score;
                        pro_node->node_state = IN_OPEN_SET;
                        open_set.push(pro_node);
                        expand_nodes[pro_node->index] = pro_node;
                        continue;
                    }

                    //如果找到且为close，则该节点不需要操作
                    if(pro_node->node_state == IN_CLOSE_SET){
                        continue;
                    }

                    //如果找到为open，则考虑是否更新g
                    if (pro_node->node_state == IN_OPEN_SET)
                    {
                        if(tmp_g_score<pro_node->g_score)
                        {
                            pro_node->g_score = tmp_g_score;
                            pro_node->f_score = tmp_f_score;
                            pro_node->parent = cur_node;
                        }
                        continue;
                    }
                }
            }
        }
    }

    AstarPath_raw.poses.clear();
    for(auto & path_node : path_nodes)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.pose.position.x = (path_node->position(0)-mp.offset_x)*mp.resolution + mp.origin_x;
        pose_stamped.pose.position.y = (path_node->position(1)-mp.offset_y)*mp.resolution + mp.origin_y;
        pose_stamped.pose.position.z = path_node->position(2)*mp.resolution + mp.origin_z + mp.output_low_bound;

        pose_stamped.pose.orientation.x = 0;
        pose_stamped.pose.orientation.y = 0;
        pose_stamped.pose.orientation.z = 0;
        pose_stamped.pose.orientation.w = 1;

        AstarPath_raw.poses.push_back(pose_stamped);
    }
}

void Astar3d::cutPath()
{
    Node* pt = new Node();
    pt->position = path_nodes[0]->position;
    path_cut.push_back(pt);

    Eigen::Vector3d start_pt = path_nodes[0]->position;
    for(int i = 1;i<(int)path_nodes.size();i++)
    {
        Eigen::Vector3d goal_pt = path_nodes[i]->position;
        if(isCollision(start_pt,goal_pt))
        {
            Node* pt = new Node();
            pt->position = path_nodes[i-1]->position;
            path_cut.push_back(pt);
            start_pt = path_nodes[i-1]->position;
        }
    }
    int n = path_nodes.size() - 1;
    Node* pt_final = new Node();
    pt_final->position = path_nodes[n]->position;
    path_cut.push_back(pt_final);
}

void Astar3d::interpolatePath()
{
    std::vector<Node*>path_tmp;
    for(auto & i : path_cut)
    {
        Node* pt_tmp = new Node();
        pt_tmp->position(0) = (i->position(0)-mp.offset_x)*mp.resolution + mp.origin_x;
        pt_tmp->position(1) = (i->position(1)-mp.offset_y)*mp.resolution + mp.origin_y;
        pt_tmp->position(2) = i->position(2)*mp.resolution + mp.origin_z + mp.output_low_bound;
        path_tmp.push_back(pt_tmp);
    }

    for(int i=0;i<(int)path_tmp.size()-1;i++)
    {
        Node* pt = new Node();
        pt->position = path_tmp[i]->position;
        path_interpolation.push_back(pt);

        double dist = std::sqrt((path_tmp[i+1]->position(0)-path_tmp[i]->position(0))*(path_tmp[i+1]->position(0)-path_tmp[i]->position(0)) +
                                (path_tmp[i+1]->position(1)-path_tmp[i]->position(1))*(path_tmp[i+1]->position(1)-path_tmp[i]->position(1)) +
                                (path_tmp[i+1]->position(2)-path_tmp[i]->position(2))*(path_tmp[i+1]->position(2)-path_tmp[i]->position(2)));
        if (interpolation_dist < dist <= 2*interpolation_dist)
        {
            Node* pt_mid = new Node();
            pt_mid->position = (path_tmp[i]->position + path_tmp[i+1]->position)/2;
            path_interpolation.push_back(pt_mid);
            continue;
        }
        if (dist > 2*interpolation_dist)
        {
            double n = dist/interpolation_dist;
            for(int j = 1;j < n-1 ;j++)
            {
                Node* pt_interpolation = new Node();
                pt_interpolation ->position(0) = path_tmp[i]->position(0) + j/n * (path_tmp[i+1]->position(0)-path_tmp[i]->position(0));
                pt_interpolation ->position(1) = path_tmp[i]->position(1) + j/n * (path_tmp[i+1]->position(1)-path_tmp[i]->position(1));
                pt_interpolation ->position(2) = path_tmp[i]->position(2) + j/n * (path_tmp[i+1]->position(2)-path_tmp[i]->position(2));
                path_interpolation.push_back(pt_interpolation);
            }
        } else{
            continue;
        }
    }
    int path_cut_size = path_tmp.size();
    Node* pt = new Node();
    pt->position = path_tmp[path_cut_size-1]->position;
    path_interpolation.push_back(pt);

    AstarPath_interpolated.poses.clear();
    path_for_trajOpt_.clear();
    for(auto & i : path_interpolation)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.pose.position.x = i->position(0);
        pose_stamped.pose.position.y = i->position(1);
        pose_stamped.pose.position.z = i->position(2);

        pose_stamped.pose.orientation.x = 0;
        pose_stamped.pose.orientation.y = 0;
        pose_stamped.pose.orientation.z = 0;
        pose_stamped.pose.orientation.w = 1;

        AstarPath_interpolated.poses.push_back(pose_stamped);

        Eigen::Vector3d tmp_pt;
        tmp_pt[0] = i->position(0);
        tmp_pt[1] = i->position(1);
        tmp_pt[2] = i->position(2);

        path_for_trajOpt_.push_back(tmp_pt);
    }
    have_path = true;
}

void Astar3d::adjust_pts()
{
    path_final = path_interpolation;
    for(int epoch = 0; epoch<opt_params.opt_epoch;epoch++)
    {
        for (int i=1;i<(int)path_final.size()-2;i++)
        {
            //计算梯度
            Eigen::Vector3d grad = cal_grad(path_final[i]);
            //计算光滑系数
            Eigen::Vector3d smooth = cal_smooth(i);
            //计算长度损失
            Eigen::Vector3d length = cal_length(i,path_final);

            path_final[i]->position(0) += opt_params.w_smooth * smooth(0) + opt_params.w_obs * grad(0) + opt_params.w_len * length(0);
            path_final[i]->position(1) += opt_params.w_smooth * smooth(1) + opt_params.w_obs * grad(1) + opt_params.w_len * length(1);
            path_final[i]->position(2) += opt_params.w_smooth * smooth(2) + opt_params.w_obs * grad(2) + opt_params.w_len * length(2);
        }
    }
    AstarPath_final.poses.clear();
    for(auto & i : path_final)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.pose.position.x = i->position(0);
        pose_stamped.pose.position.y = i->position(1);
        pose_stamped.pose.position.z = i->position(2);

        pose_stamped.pose.orientation.x = 0;
        pose_stamped.pose.orientation.y = 0;
        pose_stamped.pose.orientation.z = 0;
        pose_stamped.pose.orientation.w = 1;

        AstarPath_final.poses.push_back(pose_stamped);
    }
    have_path = true;
//    ROS_INFO_STREAM("[PATH]Adjust done.");
}

int Astar3d::posToIndex(Eigen::Vector3d pt)
{
    int value1 = pt(0);  //x
    int value2 = pt(1);  //y
    int value3 = pt(2);  //z
    return value3*astar_map_sizex*astar_map_sizey + value2*astar_map_sizex + value1;
}

int Astar3d::getDiagHeu(Eigen::Vector3d x1,Eigen::Vector3d x2)
{
    //Eigen::Vector2d dis;
    int dx = fabs(x1(0)-x2(0));
    int dy = fabs(x1(1)-x2(1));
    int dz = fabs(x1(2)-x2(2));
    //dis<<dx,dy;
    int heu = dx + dy + dz;
    return heu;
}

bool Astar3d::isObs(Eigen::Vector3d pos)
{
    int idx = pos(0);
    int idy = pos(1);
    int idz = pos(2);

    if(idx<0||idy<0||idz<0||idx>=astar_map_sizex||idy>=astar_map_sizey||idz>=astar_map_sizez){
        //std::cout<<"beyond"<<std::endl;
        return true;
    }
    if(this->pMap[idx][idy][idz] == 1){
        //std::cout<<"is obs"<<std::endl;
        return true;
    }


    return false;
}

bool Astar3d::isCollision(Eigen::Vector3d start_pt,Eigen::Vector3d goal_pt)
{
    Eigen::Vector3d dirVec = goal_pt - start_pt;
    double dis = dirVec.norm();
    dirVec.normalize();
    Eigen::Vector3d checkPt;
    for(int i =0;i<dis;++i)
    {
        checkPt = start_pt + i*dirVec;
        if(noTo(checkPt)) return true;
    }
    return false;
}

bool Astar3d::noTo(Eigen::Vector3d pt)
{
    int x1 = floor(pt(0));
    int x2 = ceil(pt(0));
    int y1 = floor(pt(1));
    int y2 = ceil(pt(1));
    int z1 = floor(pt(2));
    int z2 = ceil(pt(2));
    if(pMap[x1][y1][z1]==1 || pMap[x1][y1][z2]==1|| pMap[x1][y2][z1]==1 || pMap[x1][y2][z2]==1 ||
       pMap[x2][y1][z1]==1 || pMap[x2][y1][z2]==1 || pMap[x2][y2][z1]==1 || pMap[x2][y2][z2]==1)  return true;
    return false;
}

Eigen::Vector3d Astar3d::cal_grad(const Node *point)
{
    //查找最近点
    Eigen::Vector3d grad;
    std::vector<int> pointIdxKNNSearch(1);
    std::vector<float>pointNKNSquaredDistance(1);
    pcl::PointXYZ searchPoint;
    searchPoint.x = point->position(0);
    searchPoint.y = point->position(1);
    searchPoint.z = point->position(2);
    md.kdTree.nearestKSearch(searchPoint,1,pointIdxKNNSearch,pointNKNSquaredDistance);
    double dist = std::sqrt(pointNKNSquaredDistance[0]);
    if (dist>opt_params.thr_obs)
    {
        grad<<0,0,0;
        return grad;
    }
    double dist_x = point->position[0] - md.cloud_raw->points[pointIdxKNNSearch[0]].x;
    double dist_y = point->position[1] - md.cloud_raw->points[pointIdxKNNSearch[0]].y;
    double dist_z = point->position[2] - md.cloud_raw->points[pointIdxKNNSearch[0]].z;
    double grad_x = -(dist - opt_params.thr_obs)/dist*2*dist_x;
    double grad_y = -(dist - opt_params.thr_obs)/dist*2*dist_y;
    double grad_z = -(dist - opt_params.thr_obs)/dist*2*dist_z;
    grad<<grad_x,grad_y,grad_z;
    return grad;
}

Eigen::Vector3d Astar3d::cal_smooth(int index)
{
    if (index>1 && index<path_final.size()-2)
    {
        Eigen::Vector3d smooth = 2*(path_final[index-1]->position+path_final[index+1]->position-2*path_final[index]->position)
                                 -(path_final[index-2]->position+path_final[index]->position-2*path_final[index-1]->position)
                                 -(path_final[index+2]->position+path_final[index]->position-2*path_final[index+1]->position);
        return smooth;
    } else{
        Eigen::Vector3d smooth = 2*(path_final[index-1]->position+path_final[index+1]->position-2*path_final[index]->position);
        return smooth;
    }
}

Eigen::Vector3d Astar3d::cal_length(int index, std::vector<Node*> &path)
{
    //当前点
    Eigen::Vector3d pt_now(path_interpolation[index]->position(0),path_interpolation[index]->position(1),path_interpolation[index]->position(2));
    //当前点的前一个点
    Eigen::Vector3d pt_before(path_interpolation[index-1]->position(0),path_interpolation[index-1]->position(1),path_interpolation[index-1]->position(2));
    //后一个点
    Eigen::Vector3d pt_after(path_interpolation[index+1]->position(0),path_interpolation[index+1]->position(1),path_interpolation[index+1]->position(2));

    double len1 = (pt_now-pt_before).norm();
    double len2 = (pt_after-pt_now).norm();
    Eigen::Vector3d length;
    if (len1 >= opt_params.thr_len){
        length = 2*(len1 - opt_params.thr_len)/len1*2*(pt_before-pt_now);
    }
    if (len2 >= opt_params.thr_len){
        length += 2*(len2 - opt_params.thr_len)/len2*2*(pt_after-pt_now);
    }
    return length;
}



void Astar3d::retrievePath(Node* end_node)
{
    Node* cur_node = end_node;
    path_nodes.push_back(cur_node);

    while (cur_node->parent!=nullptr)
    {
        cur_node = cur_node->parent;
        path_nodes.push_back(cur_node);
    }

    reverse(path_nodes.begin(),path_nodes.end());
}


void Astar3d::reset()
{
    nh.param("/Astar3d/opt_params/thr_obs", this->opt_params.thr_obs,0.5);

    this->have_path = false;
    this->start_available = true;
    this->goal_available = true;

    for(auto & expand_node : expand_nodes)
    {
        delete expand_node.second;
        expand_node.second=nullptr;
    }
    expand_nodes.clear();

    path_nodes.clear();
    path_cut.clear();
    path_final.clear();
    path_interpolation.clear();

    AstarPath_raw.poses.clear();
    AstarPath_interpolated.poses.clear();
    AstarPath_final.poses.clear();

    path_for_trajOpt_.clear();

    open_set.empty();

    std::priority_queue<Node*,std::vector<Node*>,cmp>empty_queue;
    open_set.swap(empty_queue);
}


void Astar3d::publish_path_raw(const ros::TimerEvent&event)
{
    if(!have_path)return;
    AstarPath_raw.header.stamp = ros::Time::now();
    path_raw_publisher.publish(AstarPath_raw);
}

void Astar3d::publish_path_interpolated(const ros::TimerEvent &event)
{
    if(!have_path)return;
    AstarPath_interpolated.header.stamp = ros::Time::now();
    path_interpolated_publisher.publish(AstarPath_interpolated);
}

void Astar3d::publish_path_final(const ros::TimerEvent &event)
{
    if(!have_path)return;
    AstarPath_final.header.stamp = ros::Time::now();
    path_final_publisher.publish(AstarPath_final);
}

//世界转栅格
inline Eigen::Vector3d Astar3d::ConvertWorld2AstarMap(double world_x,double world_y,double world_z)
{
    int sx = (world_x - mp.origin_x)/mp.resolution + mp.offset_x;
    int sy = (world_y - mp.origin_y)/mp.resolution + mp.offset_y;
    int sz = (world_z - mp.output_low_bound - mp.origin_z)/mp.resolution;
    Eigen::Vector3d AstarPt(sx,sy,sz);
    return AstarPt;
}


