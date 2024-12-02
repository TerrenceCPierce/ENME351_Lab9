// Terrence Pierce, ENME351, November 2024

// Initializing Global Variables (starting with variable values)
float dist_threshold = 5;
float dist_num_readings = 4;
int search_angle_step = 5;  // degrees
int optim_angle_step = 2;   // degrees
float roll_threshold = .1;  // radians
int forward_time_thres = 300; // ms
int backward_time_thres = 500; // ms

// set important global variables
float yaw = 0;
float pitch = 0;
float roll = 0;
float distance = -1;
int servo_angle = 0;  // degrees

// set Pins
int pot_Pin = A0;
int Servo_Pin = 15;  // This is A1 (digital address)
int ENA1 = 2;        //Left SIde
int ENA2 = 3;
int ENA3 = 4;  // Right Side
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
unsigned int oldTime_move = 0;

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
  setServo(0);
}

void loop() {
  // put your main code here, to run repeatedly:
  // Debug code
  // debug_nav_code();
  //debug_serial_code();
  //setServo(0);

  // Implement basic version of optimization/gradient decent in 2D space
  // where the 2D space is that of a linear degree of freedom (robot moves forward or backward)
  // and a rotational dof (servo angle)
  // Together, this should be able to determine the rough location for the minimal off-normal error
  // Meaning the target has a surface normal vector, and this should roughly be able to line up the servo with it
  // The linear dof is of course limited by the environment, so ultrasonics are used to tell when
  // the robot is too close to an obstacle and therefore should not try to further optimize in that direction

  // Get and save values of angle and distance from Python Script
  get_Vals_serial();
  // First, check if marker is in sight
  if (distance == -1) {  // if not in sight, look for it
    findChessboard();
  }

  // Now that the chessboard is detected, apply optimization algorithm

  // First Option:
  // There's 3 possibilities for each dof
  // Stationary - angle up, angle steady, angle down
  // Forward - angle up, angle steady, angle down
  // Backward - angle up, angle steady, angle down
  // Can save the distance and roll for each combination (out of 9)
  // and select the best combination to move on to the next round until
  // the threshold is reached

  // Second Option:
  // A less naive approach would be to look at the roll angle and
  // estimate how much to move forward or back and move the servo by
  // For instance if the angle indicates the target points down
  // move servo angle up
  // However, this is less robust and relies on imposing constraint
  // to avoid inverting the roll angle when the python code outputs it
  // This method becomes stronger when first optimizing linearly in the axis projection
  // of the camera in the target frame, but since I don't output those parameters in python
  // (makes this challenge a bit trivial) that doesn't matter for this application

  // Thus, to further showcase the Processing digital twin (non-normal possibilities) and robustness, I will go for the first method

  while ((distance == -1) || (roll > roll_threshold)) {
    // Indexed based on order given above, Stationary angle up: 1, Stationary angle steady: 2, etc.
    float rollArr[9];
    float minRoll = 999;
    int minIdx = -1;
    int angle_down = servo_angle - optim_angle_step;
    int angle_up = servo_angle + optim_angle_step;

    for (int i = 0; i < 9; i+= 3) {
      // NOTE: moving on a time basis is not an accurate way of moving the same amount each time due to a variety of factors
      // including battery charge and environmental conditions, but this works for what it needs to do
      if(i == 3){ //move Forward
        forward(forward_time_thres);
      }
      else if(i == 6){ // move backward
        backward(backward_time_thres);
      }
      
      // do angle up
      if (angle_up > 90) {
        rollArr[i] = 999;
      }  // do nothing if this angle is greater than 90
      else {
        setServo(angle_up);
        get_Vals_serial();
        if (distance == -1) {  //if no longer able to see the target
          rollArr[i] = 999;
        } else {
          if (roll < minRoll) {
            minRoll = roll;
            minIdx = i;
            rollArr[i] = roll;
          }
        }
      }

      // Do original angle
      setServo(servo_angle);
      get_Vals_serial();
      if (distance == -1) {  //if no longer able to see the target
        rollArr[i + 1] = 999;
      } else {
        if (roll < minRoll) {
          minRoll = roll;
          minIdx = i + 1;
          rollArr[i + 1] = roll;
        }
      }

      if (angle_down < 0) {
        rollArr[i + 2] = 999;
      } // do nothing
      else{
        setServo(angle_down);
        get_Vals_serial();
        if (distance == -1) {  //if no longer able to see the target
          rollArr[i + 2] = 999;
        } else {
          if (roll < minRoll) {
            minRoll = roll;
            minIdx = i + 2;
            rollArr[i + 2] = roll;
          }
        }
      }
    }

    // Set the distance to the optimal location
    // Operates under constant time assumption aforementioned which in reality doesn't hold true, but is close enough
    if(minIdx == 0 || minIdx == 1 || minIdx == 2){ // Finished at back, needs to come to middle
      forward(backward_time_thres-forward_time_thres);
    }
    else if(minIdx == 3 || minIdx == 4 || minIdx == 5){ // Finished at back, needs to come to front
      forward(backward_time_thres);
    }

    // Set the angle to the optimal angle
    if(minIdx == 0 || minIdx == 3 || minIdx == 6){ // New angle is angle up
      servo_angle = angle_up;
    }
    else if(minIdx == 2 || minIdx == 5 || minIdx == 8){ // New angle is angle down
      servo_angle = angle_down;
    }

    if (minIdx == -1) {
      findChessboard();  // None of the combos found the chessboard
    }

    // After finding best angle, make sure servo_angle is within bounds
    if (servo_angle < 0) {
      servo_angle = 0;
    }
    if (servo_angle > 90) {
      servo_angle = 90;
    }

    // Repears until breaks out of while loop (close enough to normal to the chessboard)
  }
  // If optimized servo angle is below a threshold, move forward until distance threshold
  // This means there are infinite solutions so it makes sense to get as close as possible








  // PSEUDOCODE OF ORIGINAL PLAN: (similar to that of above, but in 3D space)
  // I'll likely implement some/all (cool) aspects of this in the code above eventually, but
  // hit more barriers with construction and

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

float medianDist(int trigPin, int echoPin, int numReadings) {
  float* distArr = new float[numReadings];
  for (int i = 0; i < numReadings; i++) {
    distArr[i] = readDist(trigPin, echoPin);
  }
  float median = findMedian(distArr, numReadings);
  delete distArr;  // save memory, prevent leaks
  return median;
}

float findMedian(float arr[], int size) {
  //bubble sort from https://www.geeksforgeeks.org/bubble-sort/
  int i, j;
  for (i = 0; i < size - 1; i++) {
    // Last i elements are already
    // in place
    for (j = 0; j < size - i - 1; j++) {
      if (arr[j] > arr[j + 1]) {
        float temp = arr[j];
        arr[j] = arr[j + 1];
        arr[j + 1] = temp;
      }
    }
  }

  if (size % (2)) {
    return (arr[(int)(size / 2)] + arr[(int)(size / 2 + 1)]) / 2;
  } else {
    return arr[(int)(size / 2)];
  }
}

float getMinDist() {
  float distArr[4] = { medianDist(frontLeft_Trig, frontLeft_Echo, dist_num_readings), medianDist(frontRight_Trig, frontRight_Echo, dist_num_readings), medianDist(backLeft_Trig, backLeft_Echo, dist_num_readings), medianDist(backRight_Trig, backRight_Echo, dist_num_readings) };

  float min_dist = 99999.0;
  for (int i = 0; i < 4; i++) {
    if (distArr[i] < min_dist) {
      min_dist = distArr[i];
    }
  }
  return min_dist;
}

float getMinDistFront() {
  float distArr[2] = { medianDist(frontLeft_Trig, frontLeft_Echo, dist_num_readings), medianDist(frontRight_Trig, frontRight_Echo, dist_num_readings) };

  float min_dist = 99999.0;
  for (int i = 0; i < 2; i++) {
    if (distArr[i] < min_dist) {
      min_dist = distArr[i];
    }
  }
  return min_dist;
}

float getMinDistBack() {
  float distArr[2] = { medianDist(frontLeft_Trig, frontLeft_Echo, dist_num_readings), medianDist(frontRight_Trig, frontRight_Echo, dist_num_readings) };

  float min_dist = 99999.0;
  for (int i = 0; i < 2; i++) {
    if (distArr[i] < min_dist) {
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

void debug_nav_code() {
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

void debug_serial_code() {
  // https://docs.arduino.cc/language-reference/en/functions/communication/serial/read/
  if (Serial.available() > 0) {
    // read the incoming byte:
    String incomingString = Serial.readString();
    // say what you got:
    Serial.print("I received: ");
    Serial.println(incomingString);

    // wasn't sure if when there's a transmit error if empty or space with \n, so to be safe:

    if (incomingString.length() > 5) {
      // inspired by https://forum.arduino.cc/t/split-string-by-delimiters/373124/6
      int i1 = incomingString.indexOf('\t');            //finds location of first \t
      yaw = incomingString.substring(0, i1).toFloat();  //captures first data
      int i2 = incomingString.indexOf('\t', i1 + 1);
      pitch = incomingString.substring(i1 + 1, i2 + 1).toFloat();
      int i3 = incomingString.indexOf('\t', i2 + 1);
      roll = incomingString.substring(i2 + 1, i3 + 1).toFloat();
      int i4 = incomingString.indexOf('\t', i3 + 1);
      distance = incomingString.substring(i3 + 1).toFloat();

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

void get_Vals_serial() {
  // https://docs.arduino.cc/language-reference/en/functions/communication/serial/read/
  while (!Serial.available()) {}
  if (Serial.available() > 0) {
    // read the incoming byte:
    String incomingString = Serial.readString();
    // say what you got:
    // Ensure transmission in python console
    Serial.print("I received: ");
    Serial.println(incomingString);

    // wasn't sure if when there's a transmit error if empty or space with \n, so to be safe:

    if (incomingString.length() > 5) {
      // inspired by https://forum.arduino.cc/t/split-string-by-delimiters/373124/6
      int i1 = incomingString.indexOf('\t');            //finds location of first \t
      yaw = incomingString.substring(0, i1).toFloat();  //captures first data
      int i2 = incomingString.indexOf('\t', i1 + 1);
      pitch = incomingString.substring(i1 + 1, i2 + 1).toFloat();
      int i3 = incomingString.indexOf('\t', i2 + 1);
      roll = incomingString.substring(i2 + 1, i3 + 1).toFloat();
      int i4 = incomingString.indexOf('\t', i3 + 1);
      distance = incomingString.substring(i3 + 1).toFloat();

      // Ensure transmission in python console
      //Serial.println(yaw);
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

void forward(int time) {
  oldTime_move = millis();
  digitalWrite(ENA1, HIGH);
  digitalWrite(ENA2, LOW);
  digitalWrite(ENA3, HIGH);
  digitalWrite(ENA4, LOW);
  while(((millis()-oldTime_move) < time) && (getMinDistFront() > dist_threshold)){}
  stop();
}

void backward(int time) {
  oldTime_move = millis();
  digitalWrite(ENA1, LOW);
  digitalWrite(ENA2, HIGH);
  digitalWrite(ENA3, LOW);
  digitalWrite(ENA4, HIGH);
  while(((millis()-oldTime_move) < time) && (getMinDistBack() > dist_threshold)){}
  stop();
}

void stop() {
  digitalWrite(ENA1, LOW);
  digitalWrite(ENA2, LOW);
  digitalWrite(ENA3, LOW);
  digitalWrite(ENA4, LOW);
}

void setServo(int angle) {
  angle = -angle;
  // Datasheet:
  // https://lonelybinary.com/en-us/products/ds3225mg?srsltid=AfmBOooq_KMl9johkt2W__o180iy3EyvWUtdmS_Bg-ScADnqo5929rpG
  int t_control = map(angle, 0, 90, 1500, 2250);
  while ((micros() - oldTime + t_control) < tPeriod) {
  }
  digitalWrite(Servo_Pin, HIGH);
  //Serial.println("High");
  while ((micros() - oldTime) < tPeriod) {
  }
  oldTime = micros();
  digitalWrite(Servo_Pin, LOW);
  //Serial.println("Low");
}

void findChessboard() {
  // start from an angle close to current angle, in case there was just a momentary drop of sight
  int original_angle = servo_angle;
  int first_angle = servo_angle + search_angle_step;
  int second_angle = servo_angle - search_angle_step;
  while (distance == -1) {
    // catch boundary conditions to avoid servo damage
    if (first_angle > 90) {
      if (original_angle < 90 - search_angle_step) {
        first_angle = original_angle + search_angle_step;
      } else {
        first_angle = 0;
      }
    }
    if (second_angle < 0) {
      if (original_angle > 0 + search_angle_step) {
        second_angle = original_angle - search_angle_step;
      } else {
        second_angle = 90;
      }
    }

    // set servos to angle and see if the checkerboard is detected
    setServo(first_angle);
    get_Vals_serial();
    if (distance == -1) {  //if not detected then iterate first_angle and move on
      first_angle = first_angle + search_angle_step;
      setServo(second_angle);
      get_Vals_serial();
      if (distance == -1) {
        second_angle = second_angle - search_angle_step;
      } else {  // if detected, then set servo angle variable equal to the second angle
        servo_angle = second_angle;
      }
    } else {  // if detected, then set servo angle variable equal to the first angle
      servo_angle = first_angle;
    }
  }
}
