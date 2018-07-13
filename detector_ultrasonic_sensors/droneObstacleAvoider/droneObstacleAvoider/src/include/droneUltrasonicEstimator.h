#ifndef ULTRASONICESTIMATOR_H_
#define ULTRASONICESTIMATOR_H_

#include "UltrasonicRangeArduino.h"
#include "xmlfilereader.h"
#include <ros/console.h>

#include <string.h>

class UltrasonicEstimator {

private:
    static const int number_of_sensors=4;  //CONFIG

    double estimated_range[number_of_sensors];
    double unfiltered_range[number_of_sensors];
    double previous_estimated_range[number_of_sensors];
    float P[number_of_sensors];
    float Q;//????     Mayor R/Q  -> Mayor importancia a la velocidad que al sensor (el ruido del sensor desaparece pero se vuelve mas lento)
    float R;//??
    float sensorAngle[number_of_sensors];  //CONFIG


public:
    UltrasonicEstimator(int idDrone, const std::string &stackPath_in);

    void filter(float velocity_x, float velocity_y, droneMsgsROS::UltrasonicRangeArduino measure);
    inline float getSensorAngle(int i) { return sensorAngle[i]; }
    inline double getRange(int i) { return estimated_range[i]; }
    inline double getPreviousRange(int i) { return previous_estimated_range[i]; }
    inline double getUnfilteredRange(int i) { return unfiltered_range[i]; }
    inline void setRQratio (float ratio) { R=ratio; }   //Greater ratio -> greater weight for velocities rather than new measures when updating distances
};

#endif
