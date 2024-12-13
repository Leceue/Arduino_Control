#include <SimpleFOC.h>
#include <Servo.h>
#include <BasicLinearAlgebra.h>
#include "mpu6050_crl.h"
#include <math.h>
#include "FOC.h"
#include "LQRCtrl.h"
using namespace BLA;

#define Pi 3.14159265358979323846

// 开启俯仰角检测
#define OUTPUT_READABLE_YAWPITCHROLL

//--------------------引脚定义--------------------
#define motor1_pwmA 2
#define motor1_pwmB 3
#define motor1_pwmC 4
#define motor1_enablepin 50
#define motor1_sensor_pin 5
// 记着测量下raw值
#define motor1_min_raw 0
#define motor1_max_raw 1023

#define motor2_pwmA 6
#define motor2_pwmB 7
#define motor2_pwmC 8
#define motor2_enablepin 52
#define motor2_sensor_pin 9
// 记着测量下raw值
#define motor2_min_raw 0
#define motor2_max_raw 1023

#define servo11_pin 10
#define servo12_pin 11
#define servo21_pin 12
#define servo22_pin 13
#define servo11_initangle 170
#define servo12_initangle 0
#define servo21_initangle 0
#define servo22_initangle 180

#define mpu6050_interrupt_pin 18

// model parameter
float h_init = (52.305 + 30) * 0.001; // 重心高度 m
float h = (52.305+30)*0.001;       // 重心高度 m
const float M = 0.4;   // 底盘质量 kg
const float m = 1;   // 上体质量 kg
const float J = m*h*h*1.5; // 整体惯性矩 kg*m^2
const float t = 0.01;  // 采样时间 s
const float g = 9.8;   // 重力加速度 m/s^2
const float R = 0.03;  // 轮子半径 m
float L_h = h;
float R_h = h;
// 0度时的高度 (52.305+30)*0.001m

//--------------------全局变量定义--------------------

// 电机控制
motorCtrl motor1(0, motor1_pwmA, motor1_pwmB, motor1_pwmC, motor1_enablepin);
motorCtrl motor2(1, motor2_pwmA, motor2_pwmB, motor2_pwmC, motor2_enablepin);
// 舵机控制
Servo servo1[2];
Servo servo2[2];
// LQR控制
LQRControl lqr(100);

float ypr_data[3];  // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float gyro_data[3]; // [yawv, pitchv, rollv]         gyro container

// 当前状态
int current_angle = 0;

struct Model
{
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
int height2angle(float height)
{
    height -= 30;
    return round(180 * (Pi - atan(height / 18) - acos((height * height - 5976) / (180 * sqrt(324 + height * height)))));
}

// 高度控制
void HeightCtrl(float up_height)
{
    int angle = height2angle(up_height + h*1000);
    int currentAngleL1 = servo1[0].read();
    int currentAngleL2 = servo1[1].read();
    int currentAngleR1 = servo2[0].read();
    int currentAngleR2 = servo2[1].read();

    int stepL1 = (servo11_initangle - angle - currentAngleL1) / 10;
    int stepL2 = (servo12_initangle + angle - currentAngleL2) / 10;
    int stepR1 = (servo21_initangle + angle - currentAngleR1) / 10;
    int stepR2 = (servo22_initangle - angle - currentAngleR2) / 10;

    for (int i = 0; i < 10; i++) {
        servo1[0].write(currentAngleL1 + stepL1 * i);
        servo1[1].write(currentAngleL2 + stepL2 * i);
        servo2[0].write(currentAngleR1 + stepR1 * i);
        servo2[1].write(currentAngleR2 + stepR2 * i);
        delay(20); // Adjust delay as needed for smoothness
    }

    servo1[0].write(servo11_initangle - angle);
    servo1[1].write(servo12_initangle + angle);
    servo2[0].write(servo21_initangle + angle);
    servo2[1].write(servo22_initangle - angle);
    h = up_height*0.001 + h;
    L_h = h;
    R_h = h;
    model.h = h;
    Serial.print(h);
    model_Calc();
}

void Servo_init()
{
    servo1[0].attach(servo11_pin);
    servo1[1].attach(servo12_pin);
    servo2[0].attach(servo21_pin);
    servo2[1].attach(servo22_pin);
    int steps = 10;
    int delayTime = 20; // Adjust delay as needed for smoothness

    int currentAngleL1 = servo1[0].read();
    int currentAngleL2 = servo1[1].read();
    int currentAngleR1 = servo2[0].read();
    int currentAngleR2 = servo2[1].read();

    int stepL1 = (servo11_initangle - currentAngleL1) / steps;
    int stepL2 = (servo12_initangle - currentAngleL2) / steps;
    int stepR1 = (servo21_initangle - currentAngleR1) / steps;
    int stepR2 = (servo22_initangle - currentAngleR2) / steps;

    for (int i = 0; i < steps; i++) {
        servo1[0].write(currentAngleL1 + stepL1 * i);
        servo1[1].write(currentAngleL2 + stepL2 * i);
        servo2[0].write(currentAngleR1 + stepR1 * i);
        servo2[1].write(currentAngleR2 + stepR2 * i);
        delay(delayTime);
    }

    servo1[0].write(servo11_initangle);
    servo1[1].write(servo12_initangle);
    servo2[0].write(servo21_initangle);
    servo2[1].write(servo22_initangle);
    HeightCtrl(10);
    Serial.println("Servo ready.");
}

void motor_init()
{
    // motor1.motorsensor(motor1_sensor_pin, motor1_min_raw, motor1_max_raw);
    motor1.init();
    Serial.println("Motor1 ready.");

    // motor2.motorsensor(motor2_sensor_pin, motor2_min_raw, motor2_max_raw);
    motor2.init();
    Serial.println("Motor2 ready.");
}

BLA::Matrix<3, 3> espA(const BLA::Matrix<3, 3> &A, float i = 1.0)
{
    BLA::Matrix<3, 3> I = {
        1, 0, 0,
        0, 1, 0,
        0, 0, 1};
    return A * t + I * i + (A * A) * (t * t / 2);
}

void model_Calc()
{
    // 系统矩阵定义
    BLA::Matrix<3, 3> A = {
        0, 1, 0,
        m * g * h / (J + M * m * h * h / (M + m)), 0, 0,
        -(m * m * h * h * g) / (M * m * h * h + (M + m) * J), 0, 0};

    BLA::Matrix<3, 1> B = {
        0,
        -1 / (J + M * m * h * h / (M + m)),
        -m * h / (M * m * h * h + (M + m) * J)};

    // 权重矩阵定义 分别对应v, theta, w
    BLA::Matrix<3, 3> Q = {
        0.8, 0, 0,
        0, 2, 0,
        0, 0, 0};

    BLA::Matrix<1, 1> R = {1};

    // 离散化处理
    A = espA(A);
    B = espA(A, 0) * Inverse(A) * B; // 注意这里使用BasicLinearAlgebra的Inverse

    // 设置LQR控制器参数
    lqr.set_model(A, B, Q, R);
}

// 状态初始化
void state_init()
{
    State target_state;
    target_state.v = 0;
    target_state.theta = 0;
    target_state.w = 0;
    lqr.set_target_state(target_state);
    // 模型计算初始化
    model_Calc();
}

// 左右高度控制
void LRHeighCtrl(float delta_h)
{
    delta_h *= 0.001;
    if(delta_h < 0){
        L_h = L_h + delta_h / 2;
        R_h = R_h - delta_h / 2;
    }else if(delta_h > 0){
        L_h = L_h - delta_h / 2;
        R_h = R_h + delta_h / 2;
    }
    int angleL = height2angle(L_h*1000);
    int angleR = height2angle(R_h*1000);
    int currentAngleL1 = servo1[0].read();
    int currentAngleL2 = servo1[1].read();
    int currentAngleR1 = servo2[0].read();
    int currentAngleR2 = servo2[1].read();

    int stepL1 = (servo11_initangle - angleL - currentAngleL1) / 10;
    int stepL2 = (servo12_initangle + angleL - currentAngleL2) / 10;
    int stepR1 = (servo21_initangle + angleR - currentAngleR1) / 10;
    int stepR2 = (servo22_initangle - angleR - currentAngleR2) / 10;

    for (int i = 0; i < 10; i++) {
        servo1[0].write(currentAngleL1 + stepL1 * i);
        servo1[1].write(currentAngleL2 + stepL2 * i);
        servo2[0].write(currentAngleR1 + stepR1 * i);
        servo2[1].write(currentAngleR2 + stepR2 * i);
        delay(20); // Adjust delay as needed for smoothness
    }

    servo1[0].write(servo11_initangle - angleL);
    servo1[1].write(servo12_initangle + angleL);
    servo2[0].write(servo21_initangle + angleR);
    servo2[1].write(servo22_initangle - angleR);
}

// 状态更新
void state_update()
{
    static unsigned long last_time = 0;
    if (last_time == 0)
    {
        last_time = millis();
        return;
    }
    unsigned long now_time = millis();
    unsigned long dt = now_time - last_time;
    last_time = now_time;
    State now_state;
    // 倾斜角度更新
    float last_ypr[3];
    last_ypr[0] = ypr_data[0];
    last_ypr[1] = ypr_data[1];
    last_ypr[2] = ypr_data[2];
    mpu_get(ypr_data, gyro_data);

    // float delta_h = 185*sin(ypr_data[2]);
    // LRHeighCtrl(delta_h);

    // 当前姿态更新
    now_state.v = (motor1.getVelocity() + motor2.getVelocity()) * R / 2;
    // now_state.v = 0;
    now_state.theta = ypr_data[1];
    // now_state.w = (ypr_data[1] - last_ypr[1]) / dt * 1000;
    now_state.w = 0;
    lqr.set_now_state(now_state);

    // 模型更新
    model.v = now_state.v;
    model.theta = now_state.theta;
    model.w = now_state.w;
    // model.torque = (motor1.getTorque() + motor2.getTorque()) / 2;
    // 记着再次计算
    model.h = h;
    model.roll = ypr_data[2];
}

void state_Control(State Target)
{
    lqr.set_target_state(Target);
    double torque = lqr.lqrControl();
    motor1.torqueCtrl(torque);
    motor2.torqueCtrl(-torque);
    motor1.Ctrl_loop();
    motor2.Ctrl_loop();
}

// LQR控制
void Controls(){
    state_update();
    State target_state;
    // target_state.v = 0;
    target_state.theta = 0;
    target_state.w = 0;
    if(Serial.available()){
        char command = Serial.read();
        switch (command)
        {
        case 'F':
            target_state.v = 5;
            break;
        
        case 'B':
            target_state.v = -5;
            break;

        case 'U':
            HeightCtrl(5);
            delay(10);
            break;
        case 'D':
            HeightCtrl(-5);
            delay(10);
            break;

        case 'S':
            target_state.v = 0;
            break;
        
        default:
            break;
        }
        while(Serial.available()){
            Serial.read();
        }
    }
    state_Control(target_state);
}


// 主函数

void setup()
{
    // Initialize serial communication at 9600 bits per second:
    Serial.begin(9600);
    Serial.println("Openned the Serial\n");
    Servo_init(); // 初始化舵机
    mpu_init(mpu6050_interrupt_pin); //占用了18号引脚，初始化MPU6050,并设置中断引脚
    motor_init(); // 初始化电机
    state_init(); // 初始化状态
}

int count = 0;

void servo_test();
void mpu_test();
void motor_test();
void LQR_test();

void heighttest();

void loop()
{
    // heighttest();
    Controls();
    // LQR_test();
    //motor_test();
    //mpu_test();
    delay(10);
}


void heighttest(){
    if(Serial.available()){
        int height = Serial.parseInt();
        HeightCtrl(height);
    }

}


void LQR_test(){
    state_update();
    State target_state;
    target_state.v = 0;
    target_state.theta = 0;
    target_state.w = 0;
    lqr.set_target_state(target_state);
    float torque = lqr.lqrControl();
    motor1.torqueCtrl(torque);
    motor2.torqueCtrl(-torque);
    Serial.print("Torque ");
    Serial.println(torque);
    motor1.Ctrl_loop();
    motor2.Ctrl_loop();
}

void motor_test()
{
    motor1.Ctrl_loop();
    motor2.Ctrl_loop();
    while (Serial.available())
    {
        float torque = Serial.parseFloat();
        motor1.torqueCtrl(torque);
        motor2.torqueCtrl(-torque);
        Serial.print("Torque ");
        Serial.println(torque);
        while (Serial.available())
            Serial.read();
    }
}

void mpu_test()
{
    static unsigned long last_time = 0;
    if (last_time == 0)
    {
        last_time = millis();
        return;
    }
    unsigned long now_time = millis();
    unsigned long dt = now_time - last_time;
    last_time = now_time;
    Serial.print(dt);
    State now_state;
    // 倾斜角度更新
    float last_ypr[3];
    last_ypr[0] = ypr_data[0];
    last_ypr[1] = ypr_data[1];
    last_ypr[2] = ypr_data[2];
    mpu_get(ypr_data, gyro_data);
    Serial.print("ypr\t");
    Serial.print(ypr_data[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr_data[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(ypr_data[2] * 180 / M_PI);

    // Serial.println("ypr_v\t");
    // Serial.print((ypr_data[0] - last_ypr[0]) / dt * 1000);
    // Serial.print("\t");
    // Serial.print((ypr_data[1] - last_ypr[1]) / dt * 1000);
    // Serial.print("\t");
    // Serial.println((ypr_data[2] - last_ypr[2]) / dt * 1000);
}

void servo_test()
{
    int angle = Serial.parseInt();
    int id = angle / 1000;
    angle = angle % 1000;
    if (id == 1)
    {
        servo1[0].write(angle);
        Serial.print("Servo11 ");
        Serial.println(angle);
    }
    else if (id == 2)
    {
        servo1[1].write(angle);
        Serial.print("Servo12 ");
        Serial.println(angle);
    }
    else if (id == 3)
    {
        servo2[0].write(angle);
        Serial.print("Servo21 ");
        Serial.println(angle);
    }
    else if (id == 4)
    {
        servo2[1].write(angle);
        Serial.print("Servo22 ");
        Serial.println(angle);
    }
    else if (id == 5)
    {
        if (angle > 90)
            angle = 90;
        servo1[0].write(servo11_initangle - angle);
        servo1[1].write(servo12_initangle + angle);
    }
    else if (id == 6)
    {
        if (angle > 90)
            angle = 90;
        servo2[0].write(servo21_initangle + angle);
        servo2[1].write(servo22_initangle - angle);
    }
}
