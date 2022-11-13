//
// Created by wangzichen on 2022/10/26.
//
#include <aggressive_planner/Bspline.h>

void Bspline::getParams(int input_k, std::vector<Eigen::Vector3d> input_pts, std::vector<double> input_nodes)
{
    k = input_k;
    control_pts = std::move(input_pts);
    nodeVector = std::move(input_nodes);
    total_time_ = nodeVector[nodeVector.size()-1];
}

Eigen::Vector3d Bspline::getPos(double t)
{
    double px=0.0, py=0.0, pz=0.0;
    for(int i=0;i<control_pts.size();i++){
        px = px + control_pts[i](0) * b_spline_basic(i,k,t,nodeVector);
        py = py + control_pts[i](1) * b_spline_basic(i,k,t,nodeVector);
        pz = pz + control_pts[i](2) * b_spline_basic(i,k,t,nodeVector);
    }
    Eigen::Vector3d pos(px,py,pz);
    return pos;
}

Eigen::Vector3d Bspline::getVel(double t)
{
    double dt = 0.001;
    Eigen::Vector3d Vel = (getPos(t+dt) - getPos(t))/dt;
    return Vel;
}

Eigen::Vector3d Bspline::getAcc(double t)
{
    double dt = 0.001;
    Eigen::Vector3d Acc = (getVel(t+dt) - getVel(t))/dt;
    return Acc;
}

void Bspline::displayBspline()
{
    std::cout<<"========"<<std::endl;
    for(int i=0;i<=99;i++){
        double t = i/100.0*total_time_;
        Eigen::Vector3d pos = getPos(t);
        Eigen::Vector3d vel = getVel(t);
        Eigen::Vector3d acc = getAcc(t);
        std::cout<<"Time is :"<<t<<std::endl;
        std::cout<<"p:["<<pos(0)<<","<<pos(1)<<","<<pos(2)<<"] ";
        std::cout<<"v:["<<vel(0)<<","<<vel(1)<<","<<vel(2)<<"] ";
        std::cout<<"a:["<<acc(0)<<","<<acc(1)<<","<<acc(2)<<"]"<<std::endl;
    }
}








