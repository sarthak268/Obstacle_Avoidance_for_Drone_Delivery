#include "droneObstacleAvoiderROSModule.h"

DroneObstacleAvoiderROSModule::DroneObstacleAvoiderROSModule():
    DroneModule(droneModule::active,OBSTACLE_AVOIDER_RATE),
    MyObstacleAvoider(getId(), stackPath)
{
    std::cout << "DroneObstacleAvoiderROSModule(...), enter and exit" << std::endl;
    init();
    return;
}

DroneObstacleAvoiderROSModule::~DroneObstacleAvoiderROSModule()
{
    close();
    return;
}

void DroneObstacleAvoiderROSModule::init()
{
    return;
}


void DroneObstacleAvoiderROSModule::close()
{
    if(!MyObstacleAvoider.close())
        return;
    return;
}

bool DroneObstacleAvoiderROSModule::resetValues()
{
    if(!DroneModule::resetValues())
        return false;

    if(!MyObstacleAvoider.reset())
        return false;

    return true;
}

//Start
bool DroneObstacleAvoiderROSModule::startVal()
{
    if(!DroneModule::startVal())
        return false;

    if(!MyObstacleAvoider.start())
        return false;

    return true;
}

//Stop
bool DroneObstacleAvoiderROSModule::stopVal()
{
    if(!DroneModule::stopVal())
        return false;

    if(!MyObstacleAvoider.stop())
        return false;

    return true;
}

void DroneObstacleAvoiderROSModule::open(ros::NodeHandle & nIn, std::string moduleName)
{
    DroneModule::open(nIn,moduleName);

    /*
    // Sensor Measurement
    */
    ultrasonic_command_pitch_roll_publisher = n.advertise<droneMsgsROS::vector2Stamped> (DRONE_ULTRASONIC_PITCH_ROLL_COMMAND,1,true);
    ultrasonic_danger_publisher             = n.advertise<std_msgs::Bool>               (DRONE_ULTRASONIC_DANGER_DETECTED,   1,true);
    ultrasonic_filtered_publisher           = n.advertise<std_msgs::Float32>            (DRONE_ULTRASONIC_FILTERED,          1,true);

    drone_ultrasonic_subscriber             = n.subscribe(      DRONE_DRIVER_SENSOR_ULTRASONIC, 4, &DroneObstacleAvoiderROSModule::droneUltrasonicCallback       , this);
    drone_stop_ultrasonic_subscriber        = n.subscribe(      DRONE_DRIVER_STOP_ULTRASONIC, 1, &DroneObstacleAvoiderROSModule::droneStopUltrasonicControlCallback, this);
    drone_ground_optical_flow_subscriber    = n.subscribe(      DRONE_DRIVER_SENSOR_GROUND_SPEED,1, &DroneObstacleAvoiderROSModule::droneGroundOpticalFlowCallback, this);

    //Flag of module opened
    droneModuleOpened=true;

    //Autostart the module
    moduleStarted=true;
    MyObstacleAvoider.start();



    //End
    return;
}

void DroneObstacleAvoiderROSModule::droneStopUltrasonicControlCallback(const std_msgs::Bool &msg)
{
    MyObstacleAvoider.stopUltrasonicControl(msg.data);
}


void DroneObstacleAvoiderROSModule::droneGroundOpticalFlowCallback(const droneMsgsROS::vector2Stamped &msg)

{
    MyObstacleAvoider.setDroneMeasurementGroundOpticalFlow(msg.vector.x, msg.vector.y);

}

void DroneObstacleAvoiderROSModule::droneUltrasonicCallback(const droneMsgsROS::UltrasonicRangeArduino &msg)  ///Acordarse de meter este mensaje bien
{
    MyObstacleAvoider.filterUltrasonic(msg);
}


bool DroneObstacleAvoiderROSModule::run(){

    MyObstacleAvoider.checkDanger();
    MyObstacleAvoider.run(&roll_command, &pitch_command);

    publishUltrasonicDanger();
    publishUltrasonicCommand();
    publishUltrasonicFiltered();
    return true;
}

void DroneObstacleAvoiderROSModule::publishUltrasonicCommand()
{
    droneMsgsROS::vector2Stamped current_msg;
    current_msg.vector.x = pitch_command;
    current_msg.vector.y  = roll_command;
    current_msg.header.stamp = ros::Time::now();
    ultrasonic_command_pitch_roll_publisher.publish(current_msg);
}

void DroneObstacleAvoiderROSModule::publishUltrasonicDanger()
{
    std_msgs::Bool current_msg;
    current_msg.data = MyObstacleAvoider.dangerDetected();
    ultrasonic_danger_publisher.publish(current_msg);
}

void DroneObstacleAvoiderROSModule::publishUltrasonicFiltered()
{
    std_msgs::Float32 current_msg;
    current_msg.data = MyObstacleAvoider.getRange(0)*100.0;
    ultrasonic_filtered_publisher.publish(current_msg);
}
