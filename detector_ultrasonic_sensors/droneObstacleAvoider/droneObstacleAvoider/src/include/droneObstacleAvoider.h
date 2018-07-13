#ifndef DRONEOBSTACLEAVOIDER_H_
#define DRONEOBSTACLEAVOIDER_H_

#include "droneUltrasonicEstimator.h"
#include "xmlfilereader.h"
#include <ros/console.h>
#include <string.h>

class DroneObstacleAvoider {
public:
    DroneObstacleAvoider(int idDrone, const std::string &stackPath_in);

    static const int number_of_sensors=4;

    void enable(bool e);
    bool dangerDetected();
    inline bool isEnabled() { return enabled; }
    inline double getPitch() { return pitch_cmd; }
    inline double getRoll() { return roll_cmd; }
    bool run(double* roll_, double* pitch_);
    void checkDanger();
    inline void setSafety(bool stop) { safety = stop; }
    float getDetectionDistance();
    float getRange(int i);
    void filterUltrasonic(droneMsgsROS::UltrasonicRangeArduino msg);
    void stopUltrasonicControl (bool stop);
    void setDroneMeasurementGroundOpticalFlow(float vx_mps, float vy_mps);

private:
    bool enabled;
    bool danger[number_of_sensors];
    bool safety;
    int token[number_of_sensors];
    float detection_distance;
    float dist_to_safe[number_of_sensors];
    double pitch_cmd;
    double roll_cmd;
    double time_gap;
    UltrasonicEstimator ultrasonicEstimator;

    // Configuration parameters
    bool   SYSTEM_MOUNTED;
    double GAIN_DIST, GAIN_SPEED;
    double MAX_OUTPUT;
    float last_ground_speed_X_measurement, last_ground_speed_Y_measurement;

public:
    bool close();
    bool reset();
    bool start();
    bool stop();
    bool run();
};


#endif
