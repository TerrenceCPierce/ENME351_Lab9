// Citations for where I got the socket/ server idea between Processing and Python
// Some code inspired by https://stackoverflow.com/questions/24564587/communicate-between-a-processing-sketch-and-a-python-program
// Also inspired by https://discourse.processing.org/t/sending-data-from-python-to-processing/25771
// https://stackoverflow.com/questions/72909272/processing-loading-obj-file

// Import statements
import processing.net.*; 

// Global Variables
Server s;

// Initialize checker shape for board
PShape checker;
PImage img;

// Initialize boolean to help with making sure the load went through
boolean isMeshLoaded;
PImage txtr;

// Initialize angle and distance variables
float yaw = 0;
float pitch = 0;
float roll = 0;
float distance = -1;

// NOTE: IMPORTANT, sometimes the matrix undergoes an underexpected transformation
// in which switching mode can be helpful to get the output pose to align with
// expectations
// This will be worked on and hopefully resolved in the future to apply preprocessing
// of the input data to maintain a consistent coordinate frmae
int mode = 1;

// Set the z value of the text in the image since I'm using a 3D render
float text_z = 120;

     
void setup() { 
    // Render a 900 by 900 and 3D window
    size(900, 900, P3D); 
    
    // Create a server on port 12000 (same as Python code)
    s = new Server(this, 12000); 
    // Load Checkerboard object
    checker = loadShape("Checkerboard.obj");
    
    // load texture for testing purposes, no longer needed
    txtr = loadImage("texture-01.jpg");
    checker.setTexture(txtr);
    
} 
     
void draw() {
  // set background as a nice blueish color
  background(123, 173, 201);
  
  // ensure the shape loads correctly
  if(!isMeshLoaded){
    checker = loadShape("Checkerboard.obj");
    checker.setTexture(txtr);
    isMeshLoaded = true;
  }
  
  // Create client object
  Client c = s.available();
  if (c != null) {
    // Get data from the python code
    String input = c.readStringUntil(byte('\n'));
    String[] split_text = splitTokens(input, "\t");
    yaw = float(split_text[0]);
    pitch = float(split_text[1]);
    roll = float(split_text[2]);
    distance = float(split_text[3]);
    
    // Print out data to make sure it's correct
    println("Yaw " + yaw);
    println("Pitch " + pitch);
    println("Roll  " + roll);
  }
  
  // Display text for the window
  textSize(30);
  fill(36, 54, 107);
  text("Camera View", width/5, 100, text_z);
  text("Distance", width/5, height-200, text_z);
  
  // Check if a invalid image was obtained
  if (distance == -1){
  // Make red since invalid
  fill(255, 0, 0);
  text(str(distance)+" cm", width/5, height-150, text_z);
  fill(36, 54, 107);
  }
  else{
    // distance normal distance in same color as text
  text(str(distance)+" cm", width/5, height-150, text_z);
  img = loadImage("Dots.png");
  
  // Wait until image is no long null
  while (img == null){
    img = loadImage("Dots.png"); //loads what the camera sees the corners as
    delay(20);
  }
  
  // opens in the bottom right
  image(img, 2*width/3, 2*height/3, width/3, height/3);
  }
  // https://processing.org/examples/loaddisplayobj.html
  //background(123, 173, 201);
  lights();
  // https://www.youtube.com/watch?v=61NvrPsrcVc
  // Sometimes the image is flipped due to multiple solutions, 
  // so implemented an easy way to switch between modes
  if(mode == 0){ //
    // Put roughly in the middle for this coordinate frame
    translate(width/4, 5*height/8);
    pushMatrix();
    
    // Apply rotations (in order)
    rotateZ(yaw);
    rotateY(pitch);
    rotateX(roll+PI/2);
  }
  else{
    // Put roughly in the middle for this coordinate frame
    translate(3*width/4, height/4);
    pushMatrix();
    // Apply rotations (in order)
    rotateZ(PI - yaw);
    rotateY(pitch);
    rotateX(roll+PI/2);
  }
  
  // Make object bigger so easier to see
  scale(2000);
  shape(checker);
  popMatrix(); // pop the rotations
  
  delay(200); // small delay to keep up somewhat with Python
  
}
