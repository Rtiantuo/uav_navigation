#include <aggressive_planner/Fsm.h>

int main(int argc, char** argv)
{
    ros::init(argc,argv,"AggressivePlanner");
    ros::NodeHandle nh("~");

    std::cout<<"[CONTROLLER]Init."<<std::endl;

    Fsm fsm;
    fsm.initFsm(nh);

    ros::spin();
}