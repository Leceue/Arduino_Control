#include <SimpleFOC.h>
#include <Servo.h>
#include "cmath"
#include "mpu6050_crl.h"
#include <math.h>
#include "FOC.h"
#include "LQRControl.h"

#define Pi 3.14159265358979323846
Servo servo1;

// 开启俯仰角检测
#define OUTPUT_READABLE_YAWPITCHROLL



//--------------------引脚定义--------------------
#define motor1_pwmA 2
#define motor1_pwmB 3
#define motor1_pwmC 4
#define motor1_enablepin 5
#define motor1_sensor_pin 6
// 记着测量下raw值
#define motor1_min_raw 0
#define motor1_max_raw 1023

#define motor2_pwmA 7
#define motor2_pwmB 8
#define motor2_pwmC 9
#define motor2_enablepin 10
#define motor2_sensor_pin 11
// 记着测量下raw值
#define motor2_min_raw 0
#define motor2_max_raw 1023

#define servo11_pin 12
#define servo12_pin 13
#define servo21_pin 14
#define servo22_pin 15

#define mpu6050_interrupt_pin 16

//model parameter
const float M = 0.5; // 底盘质量
const float m = 0.3; // 上体质量
const float J = 0.002; // 整体惯性矩
const float t = 0.01; // 采样时间
const float g = 9.8; // 重力加速度
const float R = 0.03; // 轮子半径
float h = 0.4; // 重心高度

//--------------------全局变量定义--------------------

// 电机控制
motorCtrl motor1, motor2;
// 舵机控制
Servo servo1[2], servo2[2];
//LQR控制
LQRControl lqr(100);

float ypr_data[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float gyro_data[3];          // [yawv, pitchv, rollv]         gyro container

struct Model{
    // 平衡控制姿态
    float v;
    float theta;
    float w;
    // 电机控制力矩
    float torque;
    // 侧向控制
    float h;
    float roll;
} model;

//--------------------函数定义--------------------

// 舵机角度与高度换算，height单位mm
int height2angle(int height){
    return round(180 * (Pi - atan(height / 18) - acos((height * height - 5976) / (180 * sqrt(324 + height * height)))));
}

void Servo_init(){
    servo1[0].attach(servo11_pin);
    servo1[1].attach(servo12_pin);
    servo2[0].attach(servo21_pin);
    servo2[1].attach(servo22_pin);
    Serial.println("Servo ready.");
}

void motor_init(){
    motor1 = motorCtrl(motor1_pwmA, motor1_pwmB, motor1_pwmC, motor1_enablepin);
    motor1.motorsensor(motor1_sensor_pin, motor1_min_raw, motor1_max_raw);
    motor1.init();
    Serial.println("Motor1 ready.");

    motor2 = motorCtrl(motor2_pwmA, motor2_pwmB, motor2_pwmC, motor2_enablepin);
    motor2.motorsensor(motor2_sensor_pin, motor2_min_raw, motor2_max_raw);
    motor2.init();
    Serial.println("Motor2 ready.");
}

MatrixXd espA(MatrixXd A, int i = 1){
    MatrixXd I = MatrixXd::Identity(A.rows(), A.cols());
    return A * t + I * i + 1 / 2 * A * A * t * t;
}

void model_Calc(){
    MatrixXd A(3,3);
    MatrixXd B(3,1);
    MatrixXd Q(3,3);
    MatrixXd R(1,1);
    A << 0, 1, 0,
        m * g * h / (J + M * m * h * h / (M + m)), 0, 0,
        -(m * m * h * h * g) / (M * m * h * h + (M + m) * J), 0, 0;
    B << 0, -1 / (J + M * m * h * h / (M + m)), -m * h / (M * m * h * h + (M + m) * J);
    Q << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;
    R << 1;
    A = espA(A);
    B = espA(A, 0)*MatrixXd::inverse(A)*B;
    lqr.set_model(A, B, Q, R);
}

// 状态初始化
void state_init(){
    State target_state;
    target_state.v = 0;
    target_state.theta = 0;
    target_state.w = 0;
    lqr.set_tager_state(target_state);
    // 模型计算初始化
    model_Calc();
}

// 状态更新
void state_update(){
    State now_state;
    // 倾斜角度更新
    mpu_get(ypr_data, gyro_data);
    // 当前姿态更新
    now_state.v = (motor1.motor.getVelocity() + motor2.motor.getVelocity()) * R / 2;
    now_state.theta = ypr_data[1];
    now_state.w = gyro_data[1];
    lqr.set_now_state(now_state);

    // 模型更新
    model.v = now_state.v;
    model.theta = now_state.theta;
    model.w = now_state.w;
    model.torque = (motor1.motor.getTorque() + motor2.motor.getTorque()) / 2;
    //记着再次计算
    model.h = h;
    model.roll = ypr_data[2];
}

void state_Control(State Target){
    lqr.set_target_state(Target);
    double torque = lqr.lqrControl();
    motor1.torqueCtrl(torque);
    motor2.torqueCtrl(torque);
}

// 主函数

void setup() {
    // Initialize serial communication at 9600 bits per second:
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
    Serial.println("Openned the Serial\n");
    mpu_init(mpu6050_interrupt_pin); //占用了2号引脚，初始化MPU6050
    Servo_init(); //初始化舵机
    motor_init(); //初始化电机
    // 初始化舵机
    // servo1.attach(2);
    // int init_angle = 90;
    // servo1.write(init_angle);

}

int count = 0;

void loop() {
    motor1.Ctrl_loop();
    motor2.Ctrl_loop();
    state_update();

    if(++count % 100 == 0){
        State Target;
        Target.v = 0;
        Target.theta = 0;
        Target.w = 0;
        state_Control(Target);
        count = 0;
    }


    // 舵机转动一定角度
    // if(Serial.available() > 0){
    //     int angle = Serial.parseInt();
    //     Serial.println(angle);
    //     servo1.write(angle);
    // }

    // mpu_get(ypr_data);
    // Serial.print("ypr\t");
    // Serial.print(ypr_data[0]*180/M_PI);
    // Serial.print("\t");
    // Serial.print(ypr_data[1]*180/M_PI);
    // Serial.print("\t");
    // Serial.println(ypr_data[2]*180/M_PI);
    
    // delay(100);
}
