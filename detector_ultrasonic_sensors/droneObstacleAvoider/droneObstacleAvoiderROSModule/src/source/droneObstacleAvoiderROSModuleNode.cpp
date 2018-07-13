#include <iostream>

//ROS
#include "ros/ros.h"
#include "communication_definition.h"
#include "droneObstacleAvoiderROSModule.h"

int main(int argc,char **argv)
{
    // Ros Init
    ros::init(argc, argv, "droneObstacleAvoider");
    ros::NodeHandle n;


    DroneObstacleAvoiderROSModule MyRosObstacleAvoider;
    MyRosObstacleAvoider.open(n,MODULE_NAME_DRONE_OBSTACLE_AVOIDER);

    try
    {
        while(ros::ok())
        {
            //Read messages
            ros::spinOnce();

            MyRosObstacleAvoider.run();

            MyRosObstacleAvoider.sleep();

        }
        return 1;
    }
    catch (std::exception &ex)
    {
        std::cout<<"[ROSNODE] Exception :"<<ex.what()<<std::endl;
    }
}
