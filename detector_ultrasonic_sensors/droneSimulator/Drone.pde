class Drone{
  PVector position;
  float R=0.01;
  
  float angle=PI/2;
  float speedx=0;
  float speedy=0;
  float roll=0;
  float pitch=0;
  float dyaw=0;
  
  float armlength=50;
  float xlength;
  float ylength;
  
  Sensor[] sensor;
  
  Drone(){
    sensor = new Sensor[4];
    position = new PVector(500,200);
  }  
  
  void update(){
    position.x = position.x + speedx*cos(angle) + speedy*sin(angle);  
    position.y = position.y + speedy*cos(angle) - speedx*sin(angle);  
    
    speedy = speedy + 0.1*sin(roll);
    speedx = speedx + 0.1*sin(pitch);
    
    angle +=dyaw;
    
    if (speedx > 3) speedx = 3;
    if (speedx < -3) speedx = -3;
    if (speedy > 3) speedy = 3;
    if (speedy < -3) speedy = -3;
    
    speedx=speedx*(1-R);
    speedy=speedy*(1-R);
    
    roll = roll*(1-R);
    pitch = pitch*(1-R);
    
    xlength = armlength*cos(pitch);
    ylength = armlength*cos(roll);
    
    sensor[0] = new Sensor (position.x+xlength*cos(angle),position.y-xlength*sin(angle), angle, 5-3*sin(pitch));
    sensor[1] = new Sensor (position.x-ylength*sin(angle),position.y-ylength*cos(angle), angle+PI/2, 5+3*sin(roll));
    sensor[2] = new Sensor (position.x-xlength*cos(angle),position.y+xlength*sin(angle), angle+PI, 5+3*sin(pitch));
    sensor[3] = new Sensor (position.x+ylength*sin(angle),position.y+ylength*cos(angle), angle-PI/2, 5-3*sin(roll));
   
    for(int i=0;i<4;i++){
      textSize(10);
      fill(200,0,0);
      ranges[i] = sensor[i].getMeasure();
      text( ranges[i] ,sensor[i].position.x ,sensor[i].position.y );
    }
  
  }
  
  void draw(){
    //FRENT
    strokeWeight(4-3*sin(pitch));
    stroke(200,0,0);
    line(position.x+xlength*cos(angle),position.y-xlength*sin(angle),position.x,position.y);

    strokeWeight(4+3*sin(pitch));
    stroke(0);
    line(position.x-xlength*cos(angle),position.y+xlength*sin(angle),position.x,position.y);
    
    //IZQ
    strokeWeight(4+3*sin(roll));
    line(position.x-ylength*sin(angle),position.y-ylength*cos(angle),position.x,position.y);

    strokeWeight(4-3*sin(roll));
    line(position.x,position.y,position.x+ylength*sin(angle),position.y+ylength*cos(angle));

    fill(#8E8C8C);
    strokeWeight(1);
    
    translate(position.x,position.y);
    rotate(-angle);
    ellipse(0,0,30*cos(pitch),30*cos(roll));
    rotate(angle);
    translate(-position.x,-position.y);
    
    for(int i=0;i<4;i++)sensor[i].draw();
    
    }
  
};
