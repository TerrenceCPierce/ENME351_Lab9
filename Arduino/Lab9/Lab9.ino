// Terrence Pierce, ENME351, November 2024

// Initializing Global Variables (starting with variable values)
float dist_threshold = 5;
float dist_num_readings = 4;

// set important global variables
float yaw = 0;
float pitch = 0;
float roll = 0;
float distance = -1;

// set Pins
int pot_Pin = A0;
int Servo_Pin = 15; // This is A1 (digital address)
int ENA1 = 2; //Left SIde
int ENA2 = 3;
int ENA3 = 4; // Right Side
int ENA4 = 5; 
int frontLeft_Echo = 6;
int frontLeft_Trig = 7;
int frontRight_Echo = 8;
int frontRight_Trig = 9;
int backLeft_Echo = 10;
int backLeft_Trig = 11;
int backRight_Echo = 12;
int backRight_Trig = 13;

unsigned long oldTime = 0;

int tPeriod = 20000;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  

  // Initialize pins for motor, servo, and ultrasonics
  pinMode(frontLeft_Echo, INPUT);
  pinMode(frontLeft_Trig, OUTPUT);
  pinMode(frontRight_Echo, INPUT);
  pinMode(frontRight_Trig, OUTPUT);
  pinMode(backLeft_Echo, INPUT);
  pinMode(backLeft_Trig, OUTPUT);
  pinMode(backRight_Echo, INPUT);
  pinMode(backRight_Trig, OUTPUT);

  pinMode(ENA1, OUTPUT);
  pinMode(ENA2, OUTPUT);
  pinMode(ENA3, OUTPUT);
  pinMode(ENA4, OUTPUT);

  pinMode(Servo_Pin, OUTPUT);

  //get Initial Time
  oldTime = micros();
}

void loop() {
  // put your main code here, to run repeatedly:
  // Debug code
  // debug_nav_code();
  debug_serial_code();
  //setServo(0);

  // get data from the serial monitor informing direction to move servo and wheels
  // if data is valid

  // get mode from potentiometer

  // if mode 1, alignment:
  // save current distance and approximate x, y in case this is the closest location to alignment

  // calculate intersection with camera plane roughly

  // if z above threshold, then move to location first and then align servo

  // check if distance is greater than the threshold before moving
  // if new position is closer to alignment than previous, then save position

  // else, try moving around the object

  // if new position is closer to alignment than previous, then save position

  // else go back to previous closest position but move the opposite way around the object

  // if new position is closer to alignment than previous, then save position

  // else go back to previous closest position but move the opposite way around the object

  // align servo with projected angle

  // if mode 2, tracking

  // else, data is not valid, so rotate bot and adjust servo
}

// modified from https://www.geeksforgeeks.org/distance-measurement-using-ultrasonic-sensor-and-arduino/
float readDist(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);  // wait for 2 ms to avoid
                         // collision in serial monitor

  digitalWrite(
    trigPin,
    HIGH);  // turn on the Trigger to generate pulse
  delayMicroseconds(
    10);  // keep the trigger "ON" for 10 ms to generate
          // pulse for 10 ms.

  digitalWrite(trigPin,
               LOW);  // Turn off the pulse trigger to stop
                      // pulse generation

  long duration = pulseIn(echoPin, HIGH);
  float distance = duration * 0.0344 / 2;  // Expression to calculate distance using time
  return distance;
}

float medianDist(int trigPin, int echoPin, int numReadings){
  float* distArr = new float[numReadings];
  for(int i = 0; i < numReadings; i++){
    distArr[i]= readDist(trigPin, echoPin);
  }
  float median = findMedian(distArr, numReadings);
  delete distArr; // save memory, prevent leaks
  return median; 
}

float findMedian(float arr[], int size){
    //bubble sort from https://www.geeksforgeeks.org/bubble-sort/
    int i, j;
    for (i = 0; i < size - 1; i++){
        // Last i elements are already
        // in place
        for (j = 0; j < size - i - 1; j++){
            if (arr[j] > arr[j + 1]){
                float temp = arr[j];
                arr[j] = arr[j+1];
                arr[j+1] = temp;
            }
        }
    }
    
    if(size % (2)){
      return (arr[(int)(size/2)] + arr[(int)(size/2+1)])/2;
    }
    else{
      return arr[(int)(size/2)];
    }
}

float getMinDist(){
  float distArr[4] = {medianDist(frontLeft_Trig, frontLeft_Echo, dist_num_readings),medianDist(frontRight_Trig, frontRight_Echo, dist_num_readings),medianDist(backLeft_Trig, backLeft_Echo, dist_num_readings),medianDist(backRight_Trig, backRight_Echo, dist_num_readings)};

  float min_dist = 99999.0;
  for(int i = 0; i < 4; i++){
    if (distArr[i] < min_dist){
      min_dist = distArr[i];
    }
  }
  return min_dist;

}

int meanAnalog_mode(int PIN, int numReadings, int numModes) {
  float sum = 0;
  for (int i = 0; i < numReadings; i++) {
    sum += float(analogRead(PIN)) / 1023 * numModes;
  }
  return round(sum / numReadings);
}

void debug_nav_code(){
/*
  Serial.println("Distances");
  Serial.println(medianDist(frontLeft_Trig, frontLeft_Echo, dist_num_readings));
  Serial.println(medianDist(frontRight_Trig, frontRight_Echo, dist_num_readings));
  Serial.println(medianDist(backLeft_Trig, backLeft_Echo, dist_num_readings));
  Serial.println(medianDist(backRight_Trig, backRight_Echo, dist_num_readings));
  delay(500);
  */

  Serial.println(getMinDist());
}

void debug_serial_code(){
  // https://docs.arduino.cc/language-reference/en/functions/communication/serial/read/
  if (Serial.available() > 0) {
        // read the incoming byte:
        String incomingString = Serial.readString();        
        // say what you got:
        Serial.print("I received: ");
        Serial.println(incomingString);
        
        // wasn't sure if when there's a transmit error if empty or space with \n, so to be safe:
        
        if(incomingString.length() > 5){
          // inspired by https://forum.arduino.cc/t/split-string-by-delimiters/373124/6
          int i1 = incomingString.indexOf('\t');  //finds location of first \t
          yaw = incomingString.substring(0, i1).toFloat();   //captures first data 
          int i2 = incomingString.indexOf('\t', i1+1 );   
          pitch = incomingString.substring(i1+1, i2+1).toFloat();  
          int i3 = incomingString.indexOf('\t', i2+1 );
          roll = incomingString.substring(i2+1, i3+1).toFloat();
          int i4 = incomingString.indexOf('\t', i3+1 );
          distance = incomingString.substring(i3+1).toFloat();

          Serial.println(yaw);

          
        }
        // Strings can cause memory leaks on Arduino so I would like to delete the variable manually
        // delete incomingString; 
        // but Arduino does not support this
      }
      /*
  else{
    Serial.println("Unavailable");
  }
  */
}

void forward(int time){
  digitalWrite(ENA1, HIGH);
  digitalWrite(ENA2, LOW);
  digitalWrite(ENA3, HIGH);
  digitalWrite(ENA4, LOW);
  delay(time);
}

void stop(int time){
  digitalWrite(ENA1, LOW);
  digitalWrite(ENA2, LOW);
  digitalWrite(ENA3, LOW);
  digitalWrite(ENA4, LOW);
  delay(time);
}

void setServo(int angle){
  angle = -angle;
  // Datasheet:
  // https://lonelybinary.com/en-us/products/ds3225mg?srsltid=AfmBOooq_KMl9johkt2W__o180iy3EyvWUtdmS_Bg-ScADnqo5929rpG
  int t_control = map(angle, 0, 90, 1500, 2250);
  while((micros() - oldTime+t_control) < tPeriod){
  }
  digitalWrite(Servo_Pin, HIGH);
  //Serial.println("High");
  while((micros()-oldTime) < tPeriod){
  }
  oldTime = micros();
  digitalWrite(Servo_Pin, LOW);
  //Serial.println("Low");
}
