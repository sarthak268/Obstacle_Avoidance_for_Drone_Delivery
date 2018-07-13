#ifndef DRONE_OBSTACLE_AVOIDER_H_
#define DRONE_OBSTACLE_AVOIDER_H_

#include "ros/ros.h"
#include "droneModuleROS.h"
#include "droneObstacleAvoider.h"
#include "droneMsgsROS/dronePitchRollCmd.h"
#include "communication_definition.h"
#include "droneMsgsROS/vector2Stamped.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"


const float OBSTACLE_AVOIDER_RATE = 10.0;

class DroneObstacleAvoiderROSModule : public DroneModule
{

protected:
    DroneObstacleAvoider MyObstacleAvoider;

public: // Constructors and destructors
    DroneObstacleAvoiderROSModule();
    ~DroneObstacleAvoiderROSModule();
public: // Init and close, related to Constructors and destructors
    void init();
    void close();

protected: // DroneModule
    bool resetValues(); //Reset
    bool startVal(); //Start
    bool stopVal(); //Stop
public:
    bool run();     //Run
public: // Open, initialize subscribers and publishers
    void open(ros::NodeHandle & nIn, std::string moduleName);

    // Subscribers
private:
    ros::Subscriber drone_ultrasonic_subscriber;
    void droneUltrasonicCallback(const droneMsgsROS::UltrasonicRangeArduino& msg);
    ros::Subscriber drone_ground_optical_flow_subscriber;
    void droneGroundOpticalFlowCallback(const droneMsgsROS::vector2Stamped& msg);
    ros::Subscriber drone_stop_ultrasonic_subscriber;
    void droneStopUltrasonicControlCallback(const std_msgs::Bool &msg);


    ros::Publisher ultrasonic_danger_publisher;
    void publishUltrasonicDanger();

    ros::Publisher ultrasonic_command_pitch_roll_publisher;
    void publishUltrasonicCommand();

    ros::Publisher ultrasonic_filtered_publisher;
    void publishUltrasonicFiltered();

    double pitch_command;
    double roll_command;
};



#endif
