#include "droneUltrasonicEstimator.h"

#define FEEDFORWARD 0

UltrasonicEstimator::UltrasonicEstimator(int idDrone, const std::string &stackPath_in){

    try {
            XMLFileReader my_xml_reader(stackPath_in+"configs/drone"+cvg_int_to_string(idDrone)+"/midlevel_autopilot.xml");

            Q =                      my_xml_reader.readDoubleValue(     "midlevel_autopilot_config:ultrasound_estimator:Q" );
            R =                      my_xml_reader.readDoubleValue(     "midlevel_autopilot_config:ultrasound_estimator:R" );
            for (int i=0; i< number_of_sensors; i++){
                sensorAngle[i] =     my_xml_reader.readDoubleValue(     "midlevel_autopilot_config:ultrasound_estimator:SENSOR_ANGLE_" + cvg_int_to_string(i) );
                P[i] =               my_xml_reader.readDoubleValue(     "midlevel_autopilot_config:ultrasound_estimator:INITIAL_UNCERTAINTY");
                estimated_range[i] = my_xml_reader.readDoubleValue(     "midlevel_autopilot_config:ultrasound_estimator:INITIAL_RANGE");

            }

        } catch ( cvg_XMLFileReader_exception &e) {
            throw cvg_XMLFileReader_exception(std::string("[cvg_XMLFileReader_exception! caller_function: ") + BOOST_CURRENT_FUNCTION + e.what() + "]\n");
        }
}



void UltrasonicEstimator::filter(float velocity_x, float velocity_y, droneMsgsROS::UltrasonicRangeArduino measure){

    unfiltered_range[measure.id] = measure.range.data/100.0;

    float theta = sensorAngle[measure.id];
    float v = velocity_x*cos(theta)+velocity_y*sin(theta);
    float t = 1/measure.freq.data + FEEDFORWARD;

    //Filtro de Kalman sobre rango
    double predicted_range = estimated_range[measure.id] - v*t;
    float predicted_P = P[measure.id]+Q;
    float Kg = predicted_P/(predicted_P+R);
    previous_estimated_range[measure.id] = estimated_range[measure.id];
    estimated_range[measure.id] = predicted_range + Kg * (measure.range.data/100.0 - predicted_range);
    P[measure.id] = (1 - Kg)*predicted_P;
    ROS_INFO("Received measure from sensor %d: %lf",measure.id, estimated_range[measure.id]);

}
