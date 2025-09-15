#include <string.h>
#include <stdio.h>
#include <math.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/camera.h>
#include <webots/supervisor.h>
#define TIME_STEP 8

/*---------------------Định nghĩa---------------------*/

// Signal
#define NOP  -1
#define DEFAULT   0
#define BLANK_SIGNAL_FAILSAFE 1
#define WAIT_LEFT_INTERSECTION  2
#define ENTER_LEFT_INTERSECTION 3
#define WAIT_RIGHT_INTERSECTION 4
#define ENTER_RIGHT_INTERSECTION 5
#define FULL_SIGNAL 3
#define BLANK_SIGNAL 4
#define STOP_SIGNAL 5
#define MAX_SPEED 10

// Sensors
#define NB_GROUND_SENS 8

// LEDs
#define NB_LEDS 5

/*--------------Khởi tạo thông tin robot--------------*/

// KHÔNG CHỈNH SỬA TIME_STEP !!!
// Khai báo biến cho các sensors
unsigned short threshold[NB_GROUND_SENS] = { 300 , 300 , 300 , 300 , 300 , 300 , 300 , 300 };
unsigned int filted[8] = {0 , 0 , 0 , 0 , 0 , 0 , 0 , 0};
const float weights[8] = {-3, -2, -1.5, -1, 1, 1.5, 2, 3};
const float weight_multiplier = 1.0;
float inner_wheel_multiplier = 1.1;
int switch_state = 0;
float previous_weight = 0;
float current_weight = 0;


// Velocities
double left_ratio = 0.0;
double right_ratio = 0.0;
const double base = 4.0;

// Sensors
WbDeviceTag gs[NB_GROUND_SENS];

// LEDs 
WbDeviceTag led[NB_LEDS];

// Motors
WbDeviceTag left_motor, right_motor;

// Vận tốc MIN , MAX
void constrain(double *value, double min, double max) {
  if (*value > max) *value = max;
  if (*value < min) *value = min;
};

/*----------------Phần code code set up---------------*/

/* Hàm đọc giá trị sensors
   KHÔNG ĐƯỢC THIẾU!!!     */
void ReadSensors(){
  unsigned short gs_value[NB_GROUND_SENS] = {0, 0, 0, 0, 0, 0, 0, 0};
  for(int i=0; i<NB_GROUND_SENS; i++){
    gs_value[i] = wb_distance_sensor_get_value(gs[i]);
    // So sánh giá trị gs_value với threshold -> chuyển đổi sang nhị phân
    if (gs_value[i] < threshold[i])
      filted[i] = 1;
    else filted[i] = 0;
  };
};

// Trả về vi trí của xe

/*
float Position() {
  for (int i = 0; i < 4; i++){
    if ((filted[3] == 1) && (filted[4] == 1))
      return 0;
    else if (filted[i] == 1)
      return weights[i];
    else if (filted[7-i] == 1)
      return weights[7-i];
  };
  return current_weight;
};
*/


float Position() {
  if ((filted[3] == 1) && (filted[4] == 1))
      return 0;
  else {
    for (int i = 0; i < 4; i++){
      if (filted[i] == 1)
        return weights[i];
      else if (filted[7-i] == 1)
        return weights[7-i];
    };
  };
  return current_weight;
};


/*
float Position() {
  if ((filted[3] == 1) && (filted[4] == 1))
    return 0;
  else if (filted[0] == 1)
    return weights[0];
  else if (filted[7] == 1)
    return weights[7];
  else if (filted[1] == 1)
    return weights[1];
  else if (filted[6] == 1)
    return weights[6];
  else if (filted[2] == 1)
    return weights[2];
  else if (filted[5] == 1)
    return weights[5];
  else if (filted[3] == 1)
    return weights[3];
  else if (filted[4] == 1)
    return weights[4];
  else
    return current_weight;
};
*/

/*-----Challenge patterns recognition-----*/
unsigned char prevSensors = 0;
unsigned char currSensors = 0;

bool inPatterns(unsigned char value, const unsigned char *patterns, int size) {
    for (int i = 0; i < size; i++) {
        if (value == patterns[i]) return true;
    };
    return false;
};

// Blank signal failsafe
bool BlankSignal() {
  if (((prevSensors == 0b00010000) || (prevSensors == 0b00001000)) && (currSensors == 0b00000000)) {
    printf("\n\t\tBlank signal failsafe triggered!");
    return true;
  };
  return false;
};

// Left intersection
const unsigned char prevLeftInterPatterns[] = {
  0b11110000,
  0b11111000
};

const unsigned char currLeftInterPatterns[] = {
  0b10010000,
  0b11010000,
  0b10011000,
  0b11001000,
  0b11011000,
  0b11101000
};

bool LeftInter(){
  if (inPatterns(prevSensors, prevLeftInterPatterns, 2) && inPatterns(currSensors, currLeftInterPatterns, 6)) {
    printf("\n\t\tLeft intersection ahead!");
    return true;
  };
  return false;
};

const unsigned char enterLeftInterPatterns[] = {
  0b00111100,
  0b01111110,
  0b11111111
};

void EnterLeftInter() {
  if (inPatterns(currSensors, enterLeftInterPatterns, 3)){
    printf("\n\t\tEntered left intersection!");
    switch_state = ENTER_LEFT_INTERSECTION;
  };
};

// Right intersection
const unsigned char prevRightInterPatterns[] = {
  0b00001111,
  0b00011111
};

const unsigned char currRightInterPatterns[] = {
  0b00001001,
  0b00001011,
  0b00011001,
  0b00010011,
  0b00011011,
  0b00010111
};

bool RightInter(){
  if (inPatterns(prevSensors, prevRightInterPatterns, 2) && inPatterns(currSensors, currRightInterPatterns, 6)) {
    printf("\n\t\tRight intersection ahead!");
    return true;
  };
  return false;
};

const unsigned char enterRightInterPatterns[] = {
  0b00111100,
  0b01111110,
  0b11111111
};

void EnterRightInter() {
  if (inPatterns(currSensors, enterRightInterPatterns, 3)){
    printf("\n\t\tEntered right intersection!");
    switch_state = ENTER_RIGHT_INTERSECTION;
  };
};




int StateChecker(){
  if (BlankSignal() == true)
    return BLANK_SIGNAL_FAILSAFE;
  else if (LeftInter() == true)
    return WAIT_LEFT_INTERSECTION;
  else if (RightInter() == true)
    return WAIT_RIGHT_INTERSECTION;
  return DEFAULT;
};


void SwitchFunction() {
  switch (switch_state){
    case DEFAULT:
      
      previous_weight = current_weight;
      current_weight = Position();
    
    // Chỉnh tỉ lệ động cơ dựa trên IR
      // Rẽ phải
      if (current_weight > 0){
        left_ratio = 0.8*(base + current_weight*weight_multiplier);
        right_ratio = 0.7*(base - current_weight*weight_multiplier*inner_wheel_multiplier);
      }
      // Rẽ trái
      else if (current_weight < 0){
        left_ratio = 0.7*(base + current_weight*weight_multiplier*inner_wheel_multiplier);
        right_ratio = 0.8*(base - current_weight*weight_multiplier);
      }
      // Đi thẳng
      else {
        left_ratio = base;
        right_ratio = base;
      };
    
      switch_state = StateChecker();
      break;
      
    case BLANK_SIGNAL_FAILSAFE:
      current_weight = 0;
      left_ratio = base;
      right_ratio = base;
      switch_state = StateChecker();
      break;
    
    case WAIT_LEFT_INTERSECTION:
      left_ratio = base;
      right_ratio = base;
      EnterLeftInter();
      break;
    
    case ENTER_LEFT_INTERSECTION:
      left_ratio = 0.7*(base + weights[0]*weight_multiplier*inner_wheel_multiplier);
      right_ratio = 0.8*(base - weights[0]*weight_multiplier);
      if (currSensors == 0b10000000 || currSensors == 0b00010000 || currSensors == 0b00100000 || currSensors == 0b00110000) {
        printf("\n\t\t>>> Finished left turn, back to DEFAULT");
        switch_state = DEFAULT;
      };
      break;
    
    case WAIT_RIGHT_INTERSECTION:
      left_ratio = base;
      right_ratio = base;
      EnterRightInter();
      break;
    
    case ENTER_RIGHT_INTERSECTION:
      left_ratio = 0.8*(base + weights[7]*weight_multiplier*inner_wheel_multiplier);
      right_ratio = 0.7*(base - weights[7]*weight_multiplier);
      if (currSensors == 0b00000001 || currSensors == 0b00001000 || currSensors == 0b00000100 || currSensors == 0b00001100) {
        printf("\n\t\t>>> Finished right turn, back to DEFAULT");
        switch_state = DEFAULT;
      };
      break;
  };
};

/*---------------------Main loop---------------------*/

int main() {
/*------------------Khởi động robot------------------*/

/* Khởi động robot 
   KHÔNG ĐƯỢC BỎ!!! */
  wb_robot_init();    

// Khởi động camera 
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, 64);
  
// Khởi động sensors 
  char name[20];
  for (int i = 0; i < NB_GROUND_SENS; i++) {
    sprintf(name, "gs%d", i);
    gs[i] = wb_robot_get_device(name); /* ground sensors */
    wb_distance_sensor_enable(gs[i], TIME_STEP);
  };
  
// Khởi động LEDs 
  for (int i = 0; i < NB_LEDS; i++) {
    sprintf(name, "led%d", i);
    led[i] = wb_robot_get_device(name);
    wb_led_set(led[i], 1);
  }; 

// Khởi động Motors
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
    
// Chương trình sẽ được lặp lại vô tận trong hàm while
  while (wb_robot_step(TIME_STEP) != -1) {
    ReadSensors();
    
    
  // Convert filted[] into a single 8-bit number
    prevSensors = currSensors;  // shift last reading
    currSensors = 0;
    for (int i = 0; i < NB_GROUND_SENS; i++) {
      currSensors <<= 1;
      currSensors |= (filted[i] & 1);
    };

  // In giá trị của cảm biến ra màn hình
    printf ("\n\t\tPosition: 0b");
    for (int i = 0 ; i < 8 ; i ++)
    {
      printf ("%u" , filted[i] );
    };
    
  /*----------Điều khiển xe----------*/
    SwitchFunction();
  
  // Điều chỉnh tốc độ động cơ
    wb_motor_set_velocity(left_motor,left_ratio * MAX_SPEED);
    wb_motor_set_velocity(right_motor,right_ratio * MAX_SPEED);
  };
  wb_robot_cleanup();
  return 0;
};
    
