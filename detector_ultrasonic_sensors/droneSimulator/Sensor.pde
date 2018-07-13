

class Sensor{
 PVector position;
 float angle,apperture=PI/4, size;
 int number_of_beams=15;
 

 Sensor(float x_, float y_, float angle_, float size_){
   position = new PVector(x_, y_);
   angle=angle_;
   size=size_;
 }
  
 void draw(){
   ellipse(position.x,position.y,size,size);
 }
 
 float getMeasure(){
   PVector beam[];
   beam = new PVector[number_of_beams];
   float range;
   float min_range=200;
   for (int i=0; i< number_of_beams; i++){
      beam[i] = new PVector(position.x + 200*cos(angle - apperture/2 + i*apperture/(number_of_beams-1)), position.y - 200 * sin(angle - apperture/2 + i*apperture/(number_of_beams-1)));      
      if ( i==0 || i == number_of_beams-1) {
        
        strokeWeight(1);
        stroke(0,200,0,100);
        fill(0,200,0,10);
        arc(position.x,position.y, 400, 400, 2*PI - (angle + apperture/2), 2*PI - (angle - apperture/2), PIE);
        line(position.x,position.y, beam[i].x, beam[i].y);
      }
   }
   for(int i=0; i<wallcount; i++){     
      for (int j=0; j<number_of_beams; j++){
        if(linesIntersect( position, beam[j], wall[i].start, wall[i].end)) {
          range = linesDistance ( position, beam[j], wall[i].start, wall[i].end );
          if (range < min_range) min_range = range;
        }
        
      }
   }
   
   fill(200,0,0);
   return min_range;
 }
 
};

