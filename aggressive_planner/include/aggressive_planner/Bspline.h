//
// Created by wangzichen on 2022/10/26.
//

#ifndef SRC_BSPLINE_H
#define SRC_BSPLINE_H
#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <unordered_map>
#include <queue>
#include <vector>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <utility>

//TODO 完全可以按照fast的代码,不需要每次计算基函数,直接用一个矩阵表示即可

class Bspline{
public:
    Bspline(){};
    ~Bspline(){};

    void getParams(int input_k,std::vector<Eigen::Vector3d> pts,std::vector<double> nodes);

    Eigen::Vector3d getPos(double t);
    Eigen::Vector3d getVel(double t);
    Eigen::Vector3d getAcc(double t);

    void displayBspline();

    double total_time_;

private:
    std::vector<Eigen::Vector3d> control_pts;
    std::vector<double> nodeVector;
    int k;
    inline double b_spline_basic(int i,int k_,double u,std::vector<double>nodeVector);
};

inline double Bspline::b_spline_basic(int i, int k_, double u, std::vector<double> nodeVector)
{
    double result;
    if(k_==0){
        if (nodeVector[i] <= u && u <= nodeVector[i+1]){
            result = 1;
        } else{
            result = 0;
        }
    } else{
        double length1 = nodeVector[i+k_] - nodeVector[i];
        double length2 = nodeVector[i+k_+1] - nodeVector[i+1];
        double alpha;
        double beta;

        if(length1 == 0){
            alpha = 0;
        } else{
            alpha = (u-nodeVector[i])/length1;
        }

        if(length2 == 0){
            beta = 0;
        } else{
            beta = (nodeVector[i+k_+1]-u)/length2;
        }

        result = alpha * b_spline_basic(i,k_-1,u,nodeVector) +
                 beta * b_spline_basic(i+1,k_-1,u,nodeVector);
    }
    return result;
}





#endif //SRC_BSPLINE_H
