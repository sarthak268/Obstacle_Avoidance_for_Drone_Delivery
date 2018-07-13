Drone my_drone;
Wall[] wall;
int wallcount=0;
int max_distance=200;
float[] ranges;

void setup(){
  
  size(900,900);
  background(255);
  my_drone = new Drone();
  wall = new Wall[50];
  ranges = new float[4];
}


void draw(){
  background(255);
  my_drone.update();
  my_drone.draw();
  for (int i=0; i<wallcount; i++) wall[i].draw();
  controlAction();
}


void mousePressed(){
   
  if(mouseButton==LEFT && mouseButton!=RIGHT){
    wall[wallcount] = new Wall(mouseX,mouseY); 
    wallcount++;
  }
  
}

void mouseReleased(){
  
  if(mouseButton==LEFT && mouseButton!=RIGHT) if(wallcount!=0)wall[wallcount-1].complete();
  
}



boolean linesIntersect(PVector p1, PVector q1, PVector p2, PVector q2)
{
    // Find the four orientations needed for general and
    // special cases
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    if (o1 != o2 && o3 != o4)
        return true;
        
    return false; // Doesn't fall in any of the above cases
}

int orientation(PVector p, PVector q, PVector r)
{
    float val = (q.y - p.y) * (r.x - q.x) -
              (q.x - p.x) * (r.y - q.y);
 
    if (val == 0) return 0;  // colinear
 
    return (val > 0)? 1: 2; // clock or counterclock wise
}

float linesDistance(PVector p1, PVector q1, PVector p2, PVector q2)
{
    
  float A1 = q1.y - p1.y;
  float B1 = p1.x - q1.x;
  float C1 = A1*p1.x + B1*p1.y;
  
  float A2 = q2.y - p2.y;
  float B2 = p2.x - q2.x;
  float C2 = A2*p2.x + B2*p2.y;
  
  float det = A1*B2 - A2*B1;
  
  if(det==0) return 0;
  else{
    PVector intersection = new PVector((B2*C1-B1*C2)/det, (A1*C2 - A2*C1)/det);
    if ( angleBetweenLines ( p1, q1, p2, q2 ) > PI/3){
      strokeWeight(5);
      stroke(0,200,0);
      point(intersection.x,intersection.y);
      if (p1.dist(intersection) < 25) return 25;
      else return p1.dist(intersection);
    }
    else{
      strokeWeight(5);
      stroke(200,0,0);
      point(intersection.x,intersection.y);  
      return 200;
    }
  }
  
}
  
float angleBetweenLines ( PVector p1, PVector q1, PVector p2, PVector q2 ){
    float angle1 = atan ( - ( q1.y - p1.y) / (q1.x - p1.x)); 
    float angle2 = atan ( - ( q2.y - p2.y) / (q2.x - p2.x)); 
  
    float angle = abs( angle2 - angle1 );
    if (angle > PI/2 ) angle = PI - angle;
    
    return angle;
  
  
}
 
 
float[] integral = {0,0,0,0};
float[] last_proportional = {0,0,0,0};
boolean[] control_on= {false,false,false,false};
boolean[] safe = {true,true,true,true};
float[] last_derivative = {0,0,0,0};

void controlAction(){
    
    float control_pitch=0;
    float control_roll=0;
    float derivative;
    float proportional;
    
    for (int i=0; i<4; i++){
    ranges[i] = ranges[i]+my_drone.armlength*cos(my_drone.pitch)*abs(cos(my_drone.sensor[i].angle - my_drone.angle))+my_drone.armlength*cos(my_drone.roll)*abs(sin(my_drone.sensor[i].angle - my_drone.angle))-my_drone.armlength;
    if(ranges[i] < 25) ranges[i]=25;
    if (ranges[i] < 75 + 20*cos(my_drone.sensor[i].angle-my_drone.angle)*my_drone.speedx - 20*sin(my_drone.sensor[i].angle-my_drone.angle)*my_drone.speedy && control_on[i] ==false) { control_on[i] = true; integral[i] = 0; last_proportional[i] = 100 - ranges[i]; safe[i]=false;}
    if (ranges[i] > 75 && safe[i]) control_on[i] = false;
   
    if( control_on[i]) {
      proportional = max(0, 100 - ranges[i]); 
      if(ranges[i]!=25) {
        derivative = proportional - last_proportional[i];
      }
      else derivative = proportional - last_proportional[i];
      integral[i] += proportional;
  
      // Remember the last position.
      float debug = last_proportional[i];
      last_proportional[i] = proportional;
      last_derivative[i] = derivative;

      if(derivative < 0) { /*derivative = 0;*/ safe[i] = true;}
      if(derivative > 10) derivative = 10;
      float action = - (proportional + integral[i]/3000 + derivative*8) / 150; // PARAMETROS A AJUSTAR para la dinámica de nuestro robot 
      //println(pitch + "  -  prop:: " + proportional + ";  - int:: " + integral[i]/3000 + ";  - der:: " + derivative*6);
      println(i + "P: " + proportional + " I: " + integral[i]/3000 + " D: " + derivative*8 + " LP: " + debug);
      control_pitch += cos(my_drone.sensor[i].angle-my_drone.angle)*action;
      control_roll += -sin(my_drone.sensor[i].angle-my_drone.angle)*action;
    }
    }
    //if (control_pitch != 0) println("control pitch:  " + control_pitch + ";  |||  control roll:  " + control_roll +";");
    control2drone(control_pitch,control_roll);
}

void control2drone(float pitch, float roll)
{
  
    if (abs(pitch) >= 0.01){
      if(pitch < my_drone.pitch) my_drone.pitch += max( -0.1, (pitch - my_drone.pitch) );
      if(pitch > my_drone.pitch) my_drone.pitch += min(  0.1, (pitch - my_drone.pitch) );
    }
    if (abs(roll ) >= 0.01) {
      if(roll < my_drone.roll) my_drone.roll += max( -0.1, (roll - my_drone.roll) );
      if(roll > my_drone.roll) my_drone.roll += min(  0.1, (roll - my_drone.roll) );
    }
}

void keyPressed(){
 switch(key){
    case 'w':
      my_drone.pitch+=0.03;
      break; 
    
    case 's':
      my_drone.pitch-=0.03;
      break;
    
    case 'q':
      my_drone.roll-=0.03;
      break;
      
    case 'e':
      my_drone.roll+=0.03;
      break;  
      
    case 'a':
      my_drone.angle+=0.1;
      break; 
      
    case 'd':
      my_drone.angle-=0.1;
      break;
      
      
    case 'r':
      wallcount=0;
      break;
    
    case ' ':
      my_drone.roll=0;
      my_drone.pitch=0;
      my_drone.speedx=0;
      my_drone.speedy=0;
      break;
 }
  
}


