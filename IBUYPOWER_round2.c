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
#define DEFAULT 0
#define PREPARE_LEFT 1
#define PREPARE_RIGHT 2
#define TURN_LEFT 3
#define TURN_RIGHT 4
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
const float weights[8] = {-3.5, -2.5, -1, -0.5, 0.5, 1, 2.5, 3.5};
float currWeight = 0;
int state = DEFAULT;
unsigned char sensors[6] = {0, 0, 0, 0, 0, 0};
bool stop = false;
double stopTime = -1.0;
bool noise = false;

// Velocities
float leftRatio = 0.0;
float rightRatio = 0.0;
const float base = 4.0;

// Sensors
WbDeviceTag gs[NB_GROUND_SENS];

// LEDs
WbDeviceTag led[NB_LEDS];

// Motors
WbDeviceTag left_motor, right_motor;

/*----------------Phần code code set up---------------*/

/* Hàm đọc giá trị sensors
  KHÔNG ĐƯỢC THIẾU!!!     */
void ReadSensors()
{
    unsigned short gs_value[NB_GROUND_SENS] = {0, 0, 0, 0, 0, 0, 0, 0};
    for (int i = 0; i < NB_GROUND_SENS; i++)
    {
        gs_value[i] = wb_distance_sensor_get_value(gs[i]);
        // So sánh giá trị gs_value với threshold -> chuyển đổi sang nhị phân
        if (gs_value[i] < threshold[i])
            filted[i] = 1;
        else
            filted[i] = 0;
    };
};

// Lưu trữ 6 bộ giá trị sensors gần nhất
void getSensors()
{
    for (int i = 0; i < 5; i++)
        sensors[i] = sensors[i + 1];
    for (int i = 0; i < NB_GROUND_SENS; i++)
    {
        sensors[5] <<= 1;
        sensors[5] |= (filted[i] & 1);
    };
}

// Kiểm tra điều kiện dừng
void stopHandler()
{
    stop = true;
    for (int i = 0; i < 6; i++)
    {
        if (sensors[i] != 255)
        {
            stop = false;
            return;
        }
    }
    if (stopTime == -1.0){
      stopTime = wb_robot_get_time();
      printf("\n\t\tStopping at %f seconds", stopTime);
    }
}

// Kiểm tra tín hiệu nhiễu
void noiseHandler()
{
    if ((sensors[4] != 24) && (sensors[4] > sensors[3]) && ((sensors[5] == 24) || (sensors[5] == 16) || (sensors[5] == 8)))
    {
        printf("\n\t\tNoise detected, ignoring signal!");
        noise = true;
    }
}

// Xử lý tín hiệu sensors, tìm trọng số
float getWeight()
{
    int activeLeft = 0;
    int activeRight = 0;

    for (int i = 0; i < 5; i++)
    {
        if (filted[i] == 1)
            activeLeft++;
    }
    // printf("\tactiveLeft = %d", activeLeft);
    if ((activeLeft >= 3) && (activeLeft <= 5) && (filted[0] == 1))
        state = PREPARE_LEFT;

    for (int i = 7; i > 2; i--)
    {
        if (filted[i] == 1)
            activeRight++;
    }
    // printf("\tactiveRight = %d", activeRight);
    if ((activeRight >= 3) && (activeRight <= 5) && (filted[7] == 1))
        state = PREPARE_RIGHT;

    if ((filted[3] == 1) && (filted[4] == 1))
        return 0;
    for (int i = 0; i < 4; i++)
    {
        if (filted[i] == 1)
            return weights[i];
        else if (filted[7 - i] == 1)
            return weights[7 - i];
    }
    return 0;
}

// Hàm điều khiển robot
void Drive()
{
    switch (state)
    {
    case PREPARE_LEFT:
        noiseHandler();
        leftRatio = base;
        rightRatio = base;
        if ((sensors[5] == 0) || (sensors[5] == 255))
            state = TURN_LEFT;
        break;
    case PREPARE_RIGHT:
        noiseHandler();
        leftRatio = base;
        rightRatio = base;
        if ((sensors[5] == 0) || (sensors[5] == 255))
            state = TURN_RIGHT;
        break;
    case TURN_LEFT:
        leftRatio = -1.0;
        rightRatio = base;
        if ((sensors[5] == 128))
            state = DEFAULT;
        break;
    case TURN_RIGHT:
        leftRatio = base;
        rightRatio = -1.0;
        if ((sensors[5] >= 1) && (sensors[5] <= 3))
            state = DEFAULT;
        break;
    case DEFAULT:
        noiseHandler();
        currWeight = getWeight();

        if (currWeight > 0)
        {
            leftRatio = base + 1.0;
            rightRatio = base - currWeight;
        }
        else if (currWeight < 0)
        {
            leftRatio = base + currWeight;
            rightRatio = base + 1.0;
        }
        else
        {
            leftRatio = base;
            rightRatio = base;
        }
        break;
    }
}

/*---------------------Main loop---------------------*/

int main()
{
    /*------------------Khởi động robot------------------*/

    /* Khởi động robot
      KHÔNG ĐƯỢC BỎ!!! */
    wb_robot_init();

    // Khởi động camera
    WbDeviceTag camera = wb_robot_get_device("camera");
    wb_camera_enable(camera, 64);

    // Khởi động sensors
    char name[20];
    for (int i = 0; i < NB_GROUND_SENS; i++)
    {
        sprintf(name, "gs%d", i);
        gs[i] = wb_robot_get_device(name); /* ground sensors */
        wb_distance_sensor_enable(gs[i], TIME_STEP);
    };

    // Khởi động LEDs
    for (int i = 0; i < NB_LEDS; i++)
    {
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
    while (wb_robot_step(TIME_STEP) != -1)
    {
        ReadSensors();

        getSensors();
        // In giá trị của cảm biến ra màn hình
        printf("\n\t\tPosition: 0b");
        for (int i = 0; i < 8; i++)
        {
            printf("%u", filted[i]);
        };
        printf("\tCurrent sensors: %d", sensors[5]);

        // In giá trị trạng thái của xe
        printf("\tCurrent state: %d", state);

        /*----------Điều khiển xe----------*/
        Drive();

        // Nếu có tín hiệu nhiễu, bỏ qua tín hiệu và trở về trạng thái mặc định
        if (noise)
        {
            leftRatio = base;
            rightRatio = base;
            state = DEFAULT;
            noise = false;
        }
        // Điều chỉnh tốc độ động cơ
        printf("\tLeft/Right Ratio: %f/%f", leftRatio, rightRatio);
        wb_motor_set_velocity(left_motor, leftRatio * MAX_SPEED);
        wb_motor_set_velocity(right_motor, rightRatio * MAX_SPEED);
        double testVar = wb_robot_get_time();
        printf("\n\t\tCurrent time: %f", testVar);

        // Kiểm tra điều kiện dừng và dừng robot nếu thoả mãn điều kiện
        stopHandler();
        if (stop && (stopTime != -1.0))
        {
            printf("\n\t\tSTOPPING!");
            wb_motor_set_velocity(left_motor, base);
            wb_motor_set_velocity(right_motor, base);
            if ((wb_robot_get_time() - stopTime) > 2.0){
              printf("\n\t\tStopped at %f seconds", wb_robot_get_time());
              wb_motor_set_velocity(left_motor, 0);
              wb_motor_set_velocity(right_motor, 0);
              break;
            }
        }
    };
    wb_robot_cleanup();
    return 0;
};
