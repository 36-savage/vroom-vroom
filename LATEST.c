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
  #define BLANK_SIGNAL 1
  #define PREPARE_LEFT 2
  #define PREPARE_RIGHT 3
  #define TURN_LEFT 4
  #define TURN_RIGHT 5
  #define FULL_SIGNAL 6
  #define STOP_SIGNAL 7
  #define MAX_SPEED 10
  
  // Sensors
  #define NB_GROUND_SENS 8

  // LEDs
  #define NB_LEDS 5

  /*--------------Khởi tạo thông tin robot--------------*/

  // KHÔNG CHỈNH SỬA TIME_STEP !!!
  // Khai báo biến cho các sensors
  unsigned short threshold[NB_GROUND_SENS] = {300, 300, 300, 300, 300, 300, 300, 300};
  unsigned int filted[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  const float weights[8] = {-6, -4, -3, -1, 1, 3, 4, 6};
  const float weightMultiplier = 0.5;
  float prevWeight = 0;
  float currWeight = 0;
  int state = DEFAULT;
  unsigned char currSensors = 0;

  // Velocities
  double leftRatio = 0.0;
  double rightRatio = 0.0;
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
    for(int i = 0; i<NB_GROUND_SENS; i++){
      gs_value[i] = wb_distance_sensor_get_value(gs[i]);
      // So sánh giá trị gs_value với threshold -> chuyển đổi sang nhị phân
      if (gs_value[i] < threshold[i])
        filted[i] = 1;
      else filted[i] = 0;
    };
  };

  // Trả về vi trí của xe
void getSensors(){
  for (int i = 0; i < NB_GROUND_SENS; i++) {
    currSensors <<= 1;
    currSensors |= (filted[i] & 1);
  };
}
      


int getWeight(){
  int activeLeft = 0;
  int activeRight = 0;
  
  for (int i = 0; i < 5; i++){
    if (filted[i] == 1) activeLeft++;
  }
  printf("\tactiveLeft = %d", activeLeft);
  if (activeLeft >= 4) state = PREPARE_LEFT;
  
  for (int i = 7; i > 2; i--){
    if (filted[i] == 1) activeRight++;
  }
  printf("\tactiveRight = %d", activeRight);
  if (activeRight >= 4) state = PREPARE_RIGHT;
  
  if ((filted[3] == 1) && (filted[4] == 1)) return 0;
  for (int i = 0; i < 4; i++){
    if (filted[i] == 1) return weights[i];
    else if (filted[7-i] == 1) return weights[7-i];
  }
  return 0;
}

void Drive(){
  switch(state){
    case PREPARE_LEFT:
      leftRatio = base;
      rightRatio = base;
      if ((currSensors == 0) || (currSensors == 255)) state = TURN_LEFT; 
      break;
    case PREPARE_RIGHT:
      leftRatio = base;
      rightRatio = base;
      if ((currSensors == 0) || (currSensors == 255)) state = TURN_RIGHT;
      break;
    case TURN_LEFT:
      leftRatio = -1.0;
      rightRatio = 3.5;
      if ((currSensors == 128)) state = DEFAULT;
      break;
    case TURN_RIGHT:
      leftRatio = 3.5;
      rightRatio = -1.0;
      if ((currSensors >= 1) && (currSensors <= 3)) state = DEFAULT;
      break;
    case DEFAULT:
      currWeight = getWeight();
      if (currWeight > 0){
      // lower rightRatio
        leftRatio = base;
        rightRatio = base - currWeight*weightMultiplier;      
      }
      else if (currWeight < 0){
      // lower leftRatio
        leftRatio = base + currWeight*weightMultiplier;
        rightRatio = base;
      }
      else {
        leftRatio = base;
        rightRatio = base;
      }
      break;
  }
}

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

    // In giá trị của cảm biến ra màn hình
      getSensors();
      printf ("\n\t\tPosition: 0b");
      for (int i = 0 ; i < 8 ; i ++)
      {
        printf ("%u" , filted[i] );
      };
      printf("\tCurrent sensors: %d", currSensors);
      printf("\tCurrent state: %d", state);
      
    /*----------Điều khiển xe----------*/
      Drive();
      printf("\tCurrent weight: %f", currWeight);
      
    // Điều chỉnh tốc độ động cơ
      wb_motor_set_velocity(left_motor,leftRatio * MAX_SPEED);
      wb_motor_set_velocity(right_motor,rightRatio * MAX_SPEED);
    };
    wb_robot_cleanup();
    return 0;
  };
      
