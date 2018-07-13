class Wall{
  
  PVector start, end;
  boolean booting=true;
  boolean detected=false;
  float angle;
  
  Wall (int startX, int startY){
    start = new PVector(startX, startY);
    end = new PVector(startX, startY);
     
  }
  
  
  void draw(){
    strokeWeight(2);
    if(detected) stroke(255,0,0);
    else stroke(0);
    if(booting) line(start.x,start.y, mouseX, mouseY);
    else line(start.x,start.y,end.x,end.y);
    stroke(0);
  }
  
  void complete(){
    end.x=mouseX;
    end.y=mouseY;
    booting=false;
    angle = atan ( - ( end.y - start.y ) / (end.x - start.x));
    if ( end.x < start.x) angle = angle + PI;
    println(angle);
  }
  
};
