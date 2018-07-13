#include "droneObstacleAvoider.h"

DroneObstacleAvoider::DroneObstacleAvoider(int idDrone, const std::string &stackPath_in) :
    ultrasonicEstimator(idDrone, stackPath_in)
{

    try {
            XMLFileReader my_xml_reader(stackPath_in+"configs/drone"+cvg_int_to_string(idDrone)+"/midlevel_autopilot.xml");

            GAIN_DIST =       my_xml_reader.readDoubleValue(     "midlevel_autopilot_config:obstacle_controller:GAIN_DIST" );
            GAIN_SPEED =      my_xml_reader.readDoubleValue(     "midlevel_autopilot_config:obstacle_controller:GAIN_SPEED" );
            MAX_OUTPUT =      my_xml_reader.readDoubleValue(     "midlevel_autopilot_config:obstacle_controller:MAX_OUTPUT" );
            int sys_mounted = my_xml_reader.readIntValue(        "midlevel_autopilot_config:obstacle_controller:SYSTEM_MOUNTED" );
            if(sys_mounted==1) SYSTEM_MOUNTED = true;
            else SYSTEM_MOUNTED = false;

        } catch ( cvg_XMLFileReader_exception &e) {
            throw cvg_XMLFileReader_exception(std::string("[cvg_XMLFileReader_exception! caller_function: ") + BOOST_CURRENT_FUNCTION + e.what() + "]\n");
        }

    enabled = false;
    for(int i=0;i<number_of_sensors;i++)danger[i]=false;
    safety = false;
    pitch_cmd = 0;

    roll_cmd = 0;
    time_gap = 0.1;
}

void DroneObstacleAvoider::enable(bool e)
{
    if (e && SYSTEM_MOUNTED && !safety) enabled = e;
    else (enabled = false);
}

bool DroneObstacleAvoider::run(double* roll_, double* pitch_)
{
    roll_cmd=0;
    pitch_cmd=0;
    double action_command;

    for(int i=0; i<number_of_sensors; i++)
    {
        if(danger[i]){
            float theta = ultrasonicEstimator.getSensorAngle(i);
            action_command = 3 * GAIN_DIST * (0.55 - ultrasonicEstimator.getRange(i)) + GAIN_SPEED / time_gap * (ultrasonicEstimator.getPreviousRange(i) - ultrasonicEstimator.getRange(i));

            pitch_cmd += action_command*cos(theta);
            roll_cmd  -= action_command*sin(theta);
        }
    }

    if (pitch_cmd > MAX_OUTPUT) pitch_cmd = MAX_OUTPUT;
    if (pitch_cmd < -MAX_OUTPUT) pitch_cmd = -MAX_OUTPUT;

    if (roll_cmd > MAX_OUTPUT) roll_cmd = MAX_OUTPUT;
    if (roll_cmd < -MAX_OUTPUT) roll_cmd = -MAX_OUTPUT;

    ROS_INFO("Pitch command = %lf!", pitch_cmd);
    ROS_INFO("Roll command  = %lf!",  roll_cmd);

    *roll_  = roll_cmd;
    *pitch_ = pitch_cmd;
}

void DroneObstacleAvoider::checkDanger()
{
    for(int i=0; i<number_of_sensors; i++)
    {
        float vel_sensor = (ultrasonicEstimator.getPreviousRange(i) - ultrasonicEstimator.getRange(i)) / time_gap;
        if (vel_sensor > 1) vel_sensor = 1;
        if (vel_sensor < -1) vel_sensor =-1;
        float speed_safety = 0;
        if(vel_sensor > 0.25) speed_safety = 0.15;
        if( i==0) detection_distance = 0.4 + speed_safety +vel_sensor*vel_sensor*2;
        if(!danger[i] && ultrasonicEstimator.getRange(i) < 0.4 + speed_safety +fmax(0,vel_sensor)*vel_sensor*2) {
            token[i]++;
            if(token[i]>=2){
                danger[i] = true;
                dist_to_safe[i]= ultrasonicEstimator.getRange(i);
                if(dist_to_safe[i] > 1) dist_to_safe[i] = 1;
                if(dist_to_safe[i] < 0.4) dist_to_safe[i] = 0.4;
            }
        }
        else token[i]=0;
        if(danger[i]  && ultrasonicEstimator.getRange(i) > dist_to_safe[i]) { danger[i] = false; ROS_INFO("DANGER %d",i);}
    }
}

float DroneObstacleAvoider::getDetectionDistance(){
    return detection_distance;
}

bool DroneObstacleAvoider::dangerDetected(){
    for(int i=0; i<number_of_sensors; i++) if(danger[i]) return true;
    return false;
}

void DroneObstacleAvoider::filterUltrasonic(droneMsgsROS::UltrasonicRangeArduino msg)
{
    ultrasonicEstimator.filter(last_ground_speed_X_measurement,last_ground_speed_Y_measurement,msg);
}

void DroneObstacleAvoider::stopUltrasonicControl(bool stop)
{
    safety = stop;
    if (stop==true) enabled = false;
}

void DroneObstacleAvoider::setDroneMeasurementGroundOpticalFlow(float vx_mps, float vy_mps)
{
    last_ground_speed_X_measurement = vx_mps;
    last_ground_speed_Y_measurement = vy_mps;
}

float DroneObstacleAvoider::getRange(int i)
{
    return ultrasonicEstimator.getRange(i);
}