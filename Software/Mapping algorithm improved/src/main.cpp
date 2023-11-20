#include <Wire.h>
#include "Micromouse.h"
#include <Maze.h>
#include <Node.h>
#include "dfr0548.h" //This is an ADA library Steven set up for us. Lars followed along and explained his understanding of it to the rest of the software people.

//Ultrasonic sensors triggers and echos:
int trigger_l = 12;
int trigger_f = 2;  
int trigger_r = 0;
int echo_r = 1;
int echo_f = 8;
int echo_l = 15;

//Sensor variables + additional required filter, PID variables:
double speed_of_sound = 0.0343;
bool is_wall = false;
double old_val_str = 0.5;
double new_val_str = 0.5;
double filt_val_f = 0;
double filt_val_r = 0;
double filt_val_l = 0;
double av_RW_dist = 0;
double setpoint = 0;
double process_variable = 0; //Current value of the process variable (difference between right and left sensor readings)
double pre_dist_r;
double pre_dist_f;
double pre_dist_l;
int millis_last_turn;
double adj_dist_f;
double adj_dist_l;
double adj_dist_r;
double duration;
double calc_pre_dist;
int wall_count = 0;
int last_millis = 0;
bool inititated = false;
Vector<int> visited_dir;
bool is_goal = false;

//PID Controller parameters:
double Kp = 1.0;   // Proportional gain
double Ki = 0.1;   // Integral gain

// Variables for PID calculation:
double error = 0;
double lastError = 0;
double integral = 0;

// Output variable:
double output = 0;

//global variables for turn decisions and navigation:
std::shared_ptr<Maze> maze;
std::shared_ptr<Micromouse> micromouse;
dfr0548 motor_driver = dfr0548();

Vector<Vector<std::shared_ptr<Node>>> grid;
//inc means increased by one, dec means decreased by one.
int act_x;
int act_x_inc;
int act_x_dec;
int act_y;
int act_y_inc;
int act_y_dec;


void forward();
void reverse();
void rightTurn();
void leftTurn();
void leftStrafe();
void rightStrafe();
void onSpotRot();
void stop();
void hardRight();
void hardLeft();
void applyMotorMovement(double cont_output, double front_val);
void sendPulse(int trigger);
double readsensor(int echo, int trigger);
bool checkSquare(double r_sens_val, double l_sens_val);
double adj_measure_to_RW_dist(double measurement);
double lowPass(double& filt_val, double nv_str, double oldv_str, double measurement);
void makeMovementDecision();
double calculatePID(double right, double left);
void read_adj_set_sensor_val();
void update_visited_dir();
void deadEndLeftRot();
void deadEndRightRot();


void setup(){
  pinMode(trigger_f, OUTPUT);
  pinMode(trigger_l, OUTPUT);
  pinMode(trigger_r, OUTPUT);
  pinMode(echo_f, INPUT);
  pinMode(echo_l, INPUT);
  pinMode(echo_r, INPUT);
  Wire.begin();
  Serial.begin(115200);
  motor_driver.Initialize();
  }

//Developers of the loop: All software people. Discussed and worked on it together. 
void loop(){ 
  if(inititated == false){ //setting required objects ONLY once.  
    maze = std::make_shared<Maze>(6,8);
    micromouse = std::make_shared<Micromouse>();
    micromouse->setMazeObj(maze);
    micromouse->setActiveCell(maze->getGrid()[0][0]);
  }
  inititated = true;

  if (is_goal == false){ //as long as is_goal is false for each loop iteration -> continue search
    grid = micromouse->getMazeObj()->getGrid(); //every grid and act_... variable has to be updated each loop iteration.
    act_x = micromouse->getActiveCell()->getXidx();
    act_x_inc = micromouse->getActiveCell()->getXidx() + 1;
    act_x_dec = micromouse->getActiveCell()->getXidx() - 1;
    act_y = micromouse->getActiveCell()->getYidx();
    act_y_inc = micromouse->getActiveCell()->getYidx() + 1;
    act_y_dec = micromouse->getActiveCell()->getYidx() - 1;

    read_adj_set_sensor_val(); //read, adjust and filter, and connect the sensor values to the micromouse object.
    applyMotorMovement(calculatePID(adj_dist_l, adj_dist_r), adj_dist_f); //PID controller with motor-movement.
    delay(50);
    is_wall = checkSquare(adj_dist_r, adj_dist_l); //Check to see wether the mouse has moved into a new cell or not. 

    if (is_wall){ //If it has moved into a new cell/square do the following:
      wall_count++;
      forward(); //hardcoded forward to get a more centered position in the new cell.
      delay(1000);
      stop(); //Give time for system processing:)
      delay(100);
      micromouse->changeActiveCell(); //update the active cell member variable of the micromouse object to the new cell it entered.
      read_adj_set_sensor_val(); //because sensor has changed after the hardcoded forward movement, we need new sensor values to run the functions below properly.
      micromouse->createAllNodes(); //update node network.
      update_visited_dir(); //check the open directions for neighbouring visisted/unvisited cells.
      makeMovementDecision(); //Move accordingly. 
      delay(100);
    }
  } else {
      onSpotRot(); //Celebration for finding goal:) 
      micromouse->getMazeObj()->printNetwork(); //print the maze network/graph.
  }
  is_goal = micromouse->isGoal(); //test to check if micromouse has entered goal.
  }

//Developer: Ask. //This functions takes a measurement and returns an approximate real-world distance based on that measurement. The logic behind this 
//can be read in the blog from week 12 and in earlier blogs aswell. 
double adj_measure_to_RW_dist(double measurement){
  if (measurement < 1.84){
    return 0;
  } //Standard cases:
  if (measurement >= 1.84 && measurement <= 8.28){
    return 1.33806 * measurement - 0.682216;
  }
  if (measurement >= 10.17 && measurement <= 16.94){
    return 1.16526 * measurement + 0.203794;
  }
  if (measurement >= 18.25 && measurement <= 24.95){
    return 1.25449 * measurement - 1.05304;
  }
  if (measurement >= 26.31 && measurement <= 31.25){
    return 1.26056 * measurement - 1.40949; 
  } 
  //Edge cases: Getting an average real world distance between y element in <10,12>, <20,22>, <30,32>
  if (measurement > 8.28 && measurement < 10.17){
    av_RW_dist = ((1.33806 * measurement - 0.682216)+(1.16526 * measurement + 0.203794))/2;
    return av_RW_dist;
  }
  if (measurement > 16.94 && measurement < 18.25){
    av_RW_dist = ((1.16526 * measurement + 0.203794)+(1.25449 * measurement - 1.05304))/2;
    return av_RW_dist;
  }
  if (measurement > 24.95 && measurement < 26.31){
    av_RW_dist = ((1.25449 * measurement - 1.05304)+(1.26056 * measurement - 1.40949))/2;
    return av_RW_dist;
  }
  if (measurement > 31.25){
    return measurement;
  }
}

//Developer: Ask, with help from Erik on the theory. 
double lowPass(double& filt_val, double nv_str, double oldv_str, double measurement){
  if (filt_val == 0){
    filt_val = measurement;
    return filt_val;

  } else {
    filt_val = filt_val * oldv_str + measurement * nv_str;
    return filt_val;
  }
}
//Developer: All software people.
void read_adj_set_sensor_val(){
  pre_dist_r = readsensor(echo_r, trigger_r);
  pre_dist_l = readsensor(echo_l, trigger_l);
  pre_dist_f = readsensor(echo_f, trigger_f);
  adj_dist_f = lowPass(filt_val_f, old_val_str, new_val_str, adj_measure_to_RW_dist(pre_dist_f));
  adj_dist_l = lowPass(filt_val_l, old_val_str, new_val_str, adj_measure_to_RW_dist(pre_dist_l));
  adj_dist_r = lowPass(filt_val_r, old_val_str, new_val_str, adj_measure_to_RW_dist(pre_dist_r));
  micromouse->setFrontSensorVal(adj_dist_f);
  micromouse->setLeftSensorVal(adj_dist_l);
  micromouse->setRightSensorVal(adj_dist_r);
}

//Developer: Erik. Calibration: Erik/Lars. Control output is the error calculation between left and right sensor. 
//Fitting movement is set depending on the error- and front value.
void applyMotorMovement(double control_output, double frontSensor){
  if (frontSensor > 4.5){
    if (control_output > -1 && control_output < 1){
    forward();
  }
    if (control_output <= -1.75){
    rightTurn();
  }
    if (control_output >= 1.75){
    leftTurn();
  }
    if (control_output > -1.75 && control_output < -1){
    rightStrafe();
  }
    if (control_output < 1.75 && control_output > 1){
    leftStrafe();
  }
  }
  else {
    reverse();
  }
}
//Developer: Ask. Basic function to send pulse to Ultrasonic sensors.
void sendPulse(int trigger_pin){
  digitalWrite(trigger_pin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigger_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger_pin, LOW);
}
//Developer: Ask. Calculates the time after a pulse has been sent and uses speed of sound variable to get the proper distance in cm.  
double readsensor(int echo, int trigger){
  sendPulse(trigger);
  duration = pulseIn(echo, HIGH);
  calc_pre_dist = duration * speed_of_sound/2;
  return calc_pre_dist;
}

//Developer: Erik. Column between cells check. Debounce-time of two seconds.
bool checkSquare(double right_sensor_val, double left_sensor_val){
  if ((right_sensor_val + left_sensor_val) >= 7 && (right_sensor_val + left_sensor_val) <= 12){
    if (millis() - last_millis >= 2000){
        last_millis = millis();
        return true;
    } else {
        return false;
    }
  }
  return false;
}

//Developer: Erik. Calculates the error between right and left. If a wall is not present on either sides, a default value of 3.5 will be set to right and left.
//The PID uses proportional and integrated calculations(not derivative); 
double calculatePID(double right, double left){
  if(right < limit && left < limit){
    if(((left + right) > 7.5) && ((left + right) < 10.5)){
      if (millis() - last_millis > 4000){
        stop();
        delay(1000);
        Serial.println("New square");
        wall_count++;
        last_millis = millis();
      }
    }
  }
  if (left > limit){
    left = 3.5;
  }
  if (right > limit){
    right = 3.5;
  }
  //Calculate error
    process_variable = right - left;
    error = setpoint + process_variable;

    // Calculate integral using trapezoidal rule
    integral += (error + lastError) / 2.0;

    // Calculate PID output
    output = Kp * error + Ki * integral;

    // Update last error for the next iteration
    lastError = error;
    return output;
}

//Developers for each motor function: All software people.
void forward(){
  motor_driver.SetPWMWheels(1,0,1,0,1,0,1,0); //forward right wheel + forward left wheel.
  Serial.println("Forward");
}
void stop(){
  motor_driver.SetPWMWheels(0,0,0,0,0,0,0,0); //stop.
  Serial.println("Stop");
}

void leftTurn(){
  motor_driver.SetPWMWheels(1,0,0,1,1,0,0,1); //forward right wheel + reverse left wheel.
  Serial.println("LeftTurn");
}

void rightTurn(){
  motor_driver.SetPWMWheels(0,1,1,0,0,1,1,0); //reverse right wheel + forward left wheel.
  Serial.println("RightTurn");
}
void reverse(){
  motor_driver.SetPWMWheels(0,1,0,1,0,1,0,1); //reverse right wheel + reverse left wheel.
  Serial.println("Reverse");
}
void onSpotRot(){
  motor_driver.SetPWMWheels(0,1,1,0,0,1,1,0); 
  Serial.println("Right rotation");
}
void rightStrafe(){
   motor_driver.SetPWMWheels(0,0,1,0,0,0,1,0);
   Serial.println("RightStrafe");
 }

 void leftStrafe(){
   motor_driver.SetPWMWheels(1,0,0,0,1,0,0,0);
   Serial.println("LeftStrafe");
 }

//Developers: Lars/Erik. Simple modifying: Ask. //Hard coded right and left turn for turning scenarios.
void hardRight(){

    forward();
    delay(2000);
    stop();
    delay(100);
    rightTurn();
    delay(1750);
    stop();
    delay(100);
    micromouse->setPosititon(90);
}

void hardLeft(){
    forward();
    delay(2000);
    stop();
    delay(100);
    leftTurn();
    delay(1750);
    stop();
    delay(100);
    micromouse->setPosititon(-90);
}

//Developers: Erik/Ask. Helped with logic and theory: Lars. "dir" means direction. 
void update_visited_dir(){
  if (micromouse->getPosition() == 0){
    if (adj_dist_f > limit && micromouse->getMazeObj()->getGrid()[act_x][act_y_inc]->getIsVisited() == false){
      visited_dir.push_back(0); //0 means open cell in the forward direction and is not visited. 
    } else if (adj_dist_f > limit && micromouse->getMazeObj()->getGrid()[act_x][act_y_inc]->getIsVisited() == true){
      visited_dir.push_back(1); //1 means open cell in the forward direction and is visited.
    } else if (adj_dist_f < limit){
      visited_dir.push_back(2); //2 means wall, so no cell in the forward direction. 
    }
   
    if (adj_dist_r > limit && micromouse->getMazeObj()->getGrid()[act_x_inc][act_y]->getIsVisited() == false){
      visited_dir.push_back(0); //same as above but for right sensor.
    } else if (adj_dist_r > limit && micromouse->getMazeObj()->getGrid()[act_x_inc][act_y]->getIsVisited() == true){
      visited_dir.push_back(1); 
    } else if (adj_dist_r < limit){
      visited_dir.push_back(2);  
    }
    if (adj_dist_l > limit && micromouse->getMazeObj()->getGrid()[act_x_dec][act_y]->getIsVisited() == false){
      visited_dir.push_back(0); //same as above but for left sensor.
    } else if (adj_dist_l > limit && micromouse->getMazeObj()->getGrid()[act_x_dec][act_y]->getIsVisited() == true){
      visited_dir.push_back(1); 
    } else if (adj_dist_l < limit){
      visited_dir.push_back(2);  
    }
  }
  //Same as above but for different positions.
   else if (micromouse->getPosition() == 90 || micromouse->getPosition() == -270){
    if (adj_dist_f > limit && micromouse->getMazeObj()->getGrid()[act_x_inc][act_y]->getIsVisited() == false){
      visited_dir.push_back(0); 
    } else if (adj_dist_f > limit && micromouse->getMazeObj()->getGrid()[act_x_inc][act_y]->getIsVisited() == true){
      visited_dir.push_back(1); 
    } else if (adj_dist_f < limit){
      visited_dir.push_back(2);  
    }
   
    if (adj_dist_r > limit && micromouse->getMazeObj()->getGrid()[act_x][act_y_dec]->getIsVisited() == false){
      visited_dir.push_back(0); 
    } else if (adj_dist_r > limit && micromouse->getMazeObj()->getGrid()[act_x][act_y_dec]->getIsVisited() == true){
      visited_dir.push_back(1); 
    } else if (adj_dist_r < limit){
      visited_dir.push_back(2);  
    }
    if (adj_dist_l > limit && micromouse->getMazeObj()->getGrid()[act_x][act_y_inc]->getIsVisited() == false){
      visited_dir.push_back(0); 
    } else if (adj_dist_l > limit && micromouse->getMazeObj()->getGrid()[act_x][act_y_inc]->getIsVisited() == true){
      visited_dir.push_back(1); 
    } else if (adj_dist_l < limit){
      visited_dir.push_back(2);  
    }
  }
  else if (micromouse->getPosition() == 180 || micromouse->getPosition() == -180){
    if (adj_dist_f > limit && micromouse->getMazeObj()->getGrid()[act_x][act_y_dec]->getIsVisited() == false){
      visited_dir.push_back(0); 
    } else if (adj_dist_f > limit && micromouse->getMazeObj()->getGrid()[act_x][act_y_dec]->getIsVisited() == true){
      visited_dir.push_back(1); 
    } else if (adj_dist_f < limit){
      visited_dir.push_back(2);  
    }
   
    if (adj_dist_r > limit && micromouse->getMazeObj()->getGrid()[act_x_dec][act_y]->getIsVisited() == false){
      visited_dir.push_back(0); 
    } else if (adj_dist_r > limit && micromouse->getMazeObj()->getGrid()[act_x_dec][act_y]->getIsVisited() == true){
      visited_dir.push_back(1); 
    } else if (adj_dist_r < limit){
      visited_dir.push_back(2);  
    }
    if (adj_dist_l > limit && micromouse->getMazeObj()->getGrid()[act_x_inc][act_y]->getIsVisited() == false){
      visited_dir.push_back(0); 
    } else if (adj_dist_l > limit && micromouse->getMazeObj()->getGrid()[act_x_inc][act_y]->getIsVisited() == true){
      visited_dir.push_back(1); 
    } else if (adj_dist_l < limit){
      visited_dir.push_back(2);  
    }
  }
  else if (micromouse->getPosition() == -90 || micromouse->getPosition() == 270){
    if (adj_dist_f > limit && micromouse->getMazeObj()->getGrid()[act_x_dec][act_y]->getIsVisited() == false){
      visited_dir.push_back(0); 
    } else if (adj_dist_f > limit && micromouse->getMazeObj()->getGrid()[act_x_dec][act_y]->getIsVisited() == true){
      visited_dir.push_back(1); 
    } else if (adj_dist_f < limit){
      visited_dir.push_back(2);  
    }
   
    if (adj_dist_r > limit && micromouse->getMazeObj()->getGrid()[act_x][act_y_inc]->getIsVisited() == false){
      visited_dir.push_back(0); 
    } else if (adj_dist_r > limit && micromouse->getMazeObj()->getGrid()[act_x][act_y_inc]->getIsVisited() == true){
      visited_dir.push_back(1); 
    } else if (adj_dist_r < limit){
      visited_dir.push_back(2);  
    }
    if (adj_dist_l > limit && micromouse->getMazeObj()->getGrid()[act_x][act_y_dec]->getIsVisited() == false){
      visited_dir.push_back(0); 
    } else if (adj_dist_l > limit && micromouse->getMazeObj()->getGrid()[act_x][act_y_dec]->getIsVisited() == true){
      visited_dir.push_back(1); 
    } else if (adj_dist_l < limit){
      visited_dir.push_back(2);  
    }
  }
}
   
//Developers: Erik/Ask.
void makeMovementDecision(){
   //A list of appropriate choices. 0 means open path and not visited. 1 means path but visited. 2 means wall.
   //We want the micromouse to make choices based on the visited_dir vector. First element has priority over second element etc. 
   //First element = forward action, second element = right turn, and third element = left turn.   
  if (visited_dir[0] == 0){ //node forward not visited.
    forward();
  } else if (visited_dir[1] == 0){ //node right not visited.
    hardRight();
  } else if (visited_dir[2] == 0){ //node left not visited.
    hardLeft();
  } else if (visited_dir[0] == 1){ //if all nodes visited and no wall forward.
    forward();
  } else if (visited_dir[1] == 1){ //if all nodes visited and wall in front.
    hardRight();
  } else if (visited_dir[2] == 1){ //if all nodes visited and wall in front and right.
    hardLeft();
  } else if (visited_dir[0] == 2 && visited_dir[1] == 2 && visited_dir[2] == 2){ //all nodes visited and dead-end.
    deadEndLeftRot(); 
    deadEndRightRot(); //in case the other rotation function doesnt trigger and vice versa.

  }
}

//For both functions below: Developers and calibration of hardcoded movement: Erik/Lars. Simple modification: Ask.
void deadEndRightRot(){
  if (adj_dist_r < 5 && adj_dist_l < 5 && adj_dist_f < 4 && (adj_dist_r > adj_dist_l)){
  rightTurn();
  delay(1500);
  reverse();
  delay(500);
  rightTurn();
  delay(1500);
  stop();
  millis_last_turn = millis();
  micromouse->setPosititon(180);
}
}

void deadEndLeftRot(){
  if (adj_dist_r < 5 && adj_dist_l < 5 && adj_dist_f < 4 && (adj_dist_r < adj_dist_l)){
  leftTurn();
  delay(1500);
  reverse();
  delay(500);
  leftTurn();
  delay(1500);
  stop();
  millis_last_turn = millis();
  micromouse->setPosititon(-180);
}
}
