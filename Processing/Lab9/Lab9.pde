// Citations for where I got the socket/ server idea between Processing and Python
// Some code inspired by https://stackoverflow.com/questions/24564587/communicate-between-a-processing-sketch-and-a-python-program
// Also inspired by https://discourse.processing.org/t/sending-data-from-python-to-processing/25771
// https://stackoverflow.com/questions/72909272/processing-loading-obj-file

import processing.net.*; 
Server s;
PShape checker;
PImage img;
boolean isMeshLoaded;
PImage txtr;
float yaw = 0;
float pitch = 0;
float roll = 0;
float distance = -1;

int mode = 1;

float text_z = 120;

     
void setup() { 
    size(900, 900, P3D); 
    /* Connect to the local machine at port 50007
     *  (or whichever port you choose to run the
     *  server on).
     * This example will not run if you haven't
     *  previously started a server on this port.
     */
    s = new Server(this, 12000); 
    checker = loadShape("Checkerboard.obj");
    txtr = loadImage("texture-01.jpg");
    checker.setTexture(txtr);
    
} 
     
void draw() { 
  background(123, 173, 201);
  if(!isMeshLoaded){
    checker = loadShape("Checkerboard.obj");
    checker.setTexture(txtr);
    isMeshLoaded = true;
  }

  Client c = s.available();
  if (c != null) {
    String input = c.readStringUntil(byte('\n'));
    String[] split_text = splitTokens(input, "\t");
    yaw = float(split_text[0]);
    pitch = float(split_text[1]);
    roll = float(split_text[2]);
    distance = float(split_text[3]);
    
    println("Yaw " + yaw);
    println("Pitch " + pitch);
    println("Roll  " + roll);
  }
  textSize(30);
  fill(36, 54, 107);
  text("Camera View", width/5, 100, text_z);
  text("Distance", width/5, height-200, text_z);
  if (distance == -1){
  fill(255, 0, 0);
  text(str(distance)+" cm", width/5, height-150, text_z);
  fill(36, 54, 107);
  }
  else{
  text(str(distance)+" cm", width/5, height-150, text_z);
  img = loadImage("Dots.png");
  
  // Wait until image is no long null
  while (img == null){
    img = loadImage("Dots.png");
    delay(20);
  }
  
  //while (img.width == 0){
  //}
  image(img, 2*width/3, 2*height/3, width/3, height/3);
  }
  // https://processing.org/examples/loaddisplayobj.html
  //background(123, 173, 201);
  lights();
  // https://www.youtube.com/watch?v=61NvrPsrcVc
  // Sometimes the image is flipped due to multiple solutions, 
  // so implemented an easy way to switch between modes
  if(mode == 0){ //
    translate(width/4, 5*height/8);
    pushMatrix();
    //float theta_x = PI/8;
    rotateZ(yaw);
    rotateY(pitch);
    rotateX(roll+PI/2);
  }
  else{
    translate(3*width/4, height/4);
    pushMatrix();
    //float theta_x = PI/8;
    rotateZ(PI - yaw);
    rotateY(pitch);
    rotateX(roll+PI/2);
  }
  
  scale(2000);
  shape(checker);
  popMatrix();
  
  delay(200);
  
}
