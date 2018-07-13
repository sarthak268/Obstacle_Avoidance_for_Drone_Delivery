/*    run with rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0
*
*     Arduino Board Ultrasonic Sensor - ROS interface, designed for Arduino Nano.
*     Author: Rafael Gomez-Jordana.
*
----------------------------------*/

/*--------------------------------
* Includes
---------------------------------*/

#include <ros.h>
#include <ros/time.h>
#include <ultrasound/RequestUpdate.h>


#include <droneMsgsROS/ultrasonicArduinoService.h>
#include <droneMsgsROS/UltrasonicRangeArduino.h>

ros::NodeHandle nh;
using droneMsgsROS::ultrasonicArduinoService;

droneMsgsROS::UltrasonicRangeArduino range_msg;

ros::Publisher range_publisher( "/drone2/ultrasonic", &range_msg);

void ultrasonicArduinoServiceCallback(const ultrasonicArduinoService::Request & req, ultrasonicArduinoService::Response & res);
ros::ServiceServer<ultrasonicArduinoService::Request, ultrasonicArduinoService::Response> ultrasonicArduinoService_service("ultrasonicArduinoService_srv",&ultrasonicArduinoServiceCallback);
unsigned int minimum_close_range = 100;
const int number_of_sensors = 1;
const int max_range = 200;
unsigned long range_time;



class uSensor{

  private:

    char id;
    unsigned int pin;
    unsigned int range;
    unsigned int frequency=10;
    unsigned int state=2;  //0 Off, 1 Close,  2 Normal

    long last_publish;
    ros::Time measurement_time;

  public:
              
    uSensor(int myID, int myPin){
      id=myID;
      pin=myPin;
      last_publish=millis()+500; // 500ms Antes de realizar la primera medidicón para que se estabilice.
    }
    

    void updateRange(){      
      /**
      Filtra la lectura del puerto analógico para reducir posibles ruidos y convierte el valor a centimetros.
      */

      range = 0;
      int range_measurement[4], measurement_difference[4];
      int max_difference=0, max_difference_index=0;

      // Lee 4 veces el pin analogico correspondiente
      for(int i=0;i<4;i++) range_measurement[i]= analogRead(pin);
      // Se elimina la medida más alejada de las demás
      for(int i=0;i<4;i++) measurement_difference[i] =
      abs(range_measurement[i]-range_measurement[0]) + 
      abs(range_measurement[i]-range_measurement[1]) + 
      abs(range_measurement[i]-range_measurement[2]) + 
      abs(range_measurement[i]-range_measurement[3]);

      for(int i=0;i<4;i++){
        if(measurement_difference[i]>max_difference){
          max_difference=measurement_difference[i];
          max_difference_index=i;
        }
      }
      range_measurement[max_difference_index]=0;

      // Se promedian las tres restantes (la eliminada valdrá cero en la suma)
      for(int i=0;i<4;i++) range += range_measurement[i];

      range = floor(range / 2.4188); // (0.0124023437 /3) -> conversion a centimetros
      if ( range > max_range ) range = max_range;
      measurement_time=nh.now();

    }


    void publish(){
      /**
      Vuelca los datos del sensor en el mensaje y lo envía por Serial al ordenador de a bordo.
      */

      range_msg.id=id;
      range_msg.header.stamp=measurement_time;
      range_msg.range.data=range;
      range_msg.freq.data=frequency;
      range_publisher.publish(&range_msg);
      last_publish = millis() - 2; 

    }
    

    int checkPublish(){

                                       
      if( millis() > last_publish + 1000/frequency ) {

        if      ( state == 2 ) { publish(); return 1;}
        else if ( state == 1 && range < minimum_close_range ) { publish(); return 1;}

      }

    return 0;
    
    }


    /**
    Getters y setters
    */

    unsigned int getRange(){

      return range;	

    }

    char getId(){

      return id;	

    }

    unsigned int getFrequency(){

      return frequency;	

    }

    unsigned int getState(){

      return state;	

    }

    void setRange( unsigned int range_){

	range=range_;

    }

    void setFrequency( unsigned int freq_){

	frequency=freq_;

    }

    void setState( unsigned int state_){

	state=state_;

    }
      
};

uSensor sensor[4]={uSensor(0,A0),uSensor(1,A1),uSensor(2,A2), uSensor(3,A3)};
void setup(){
  
  nh.initNode();
  nh.advertise(range_publisher);
  nh.advertiseService(ultrasonicArduinoService_service);
  range_time=millis()+500;
  range_msg.ok=true;
  
}



void loop(){
  
  if ( millis() >= range_time ){  
  
    for(int i=0;i<number_of_sensors;i++){
      if(sensor[i].getState() != 0) sensor[i].updateRange();
    }
      
    range_time=millis()+50; // Cada 50ms se miden los sensores
  }
  
  for(int i=0;i<number_of_sensors;i++){
    sensor[i].checkPublish();
  }
  
  nh.spinOnce();
}


void ultrasonicArduinoServiceCallback(const ultrasonicArduinoService::Request & req, ultrasonicArduinoService::Response & res){                           

  
  //Set
  if( req.req_id == -1 ){
    for(int i=0;i<number_of_sensors;i++){
      if( req.req_mode.data >=0 && req.req_mode.data <=2 ) sensor[i].setState(req.req_mode.data);
      if( req.req_frequency.data >= 1 && req.req_frequency.data <=20 ) sensor[i].setFrequency(req.req_frequency.data);
    }
  }
  
  else if (req.req_id < number_of_sensors ){
    if( req.req_mode.data >=0 && req.req_mode.data <=2 ) sensor[req.req_id].setState(req.req_mode.data);
    if( req.req_frequency.data >= 1 && req.req_frequency.data <=20 ) sensor[req.req_id].setFrequency(req.req_frequency.data);
  }
  
  //Get
  if ( req.req_id >=0 && req.req_id <= number_of_sensors ){
    if(sensor[req.req_id].getState() != 0) sensor[req.req_id].updateRange();
    
  }  
}
