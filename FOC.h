#ifndef FOC_H
#define FOC_H

#include "SimpleFOC.h"

#define motor1_pwmA 6
#define motor1_pwmB 7
#define motor1_pwmC 8
#define motor1_enablepin 50
#define motor1_sensor_pin 2
// 记着测量下raw值
#define motor1_min_raw 68
#define motor1_max_raw 2092

#define motor2_pwmA 9
#define motor2_pwmB 10
#define motor2_pwmC 11
#define motor2_enablepin 52
#define motor2_sensor_pin 3
// 记着测量下raw值
#define motor2_min_raw 64
#define motor2_max_raw 2106

const float Kt = 0.27; // Nm/A 电机转矩常数

//BLDCMotor motor1 = BLDCMotor(14, 10.9, 33, 0.00474);
BLDCMotor motor1 = BLDCMotor(14, 10.9);
BLDCMotor motor2 = BLDCMotor(14, 10.9);

BLDCDriver3PWM driver1 = BLDCDriver3PWM(motor1_pwmA, motor1_pwmB, motor1_pwmC, motor1_enablepin);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(motor2_pwmA, motor2_pwmB, motor2_pwmC, motor2_enablepin);

MagneticSensorPWM sensor1 = MagneticSensorPWM(motor1_sensor_pin, motor1_min_raw, motor1_max_raw);
MagneticSensorPWM sensor2 = MagneticSensorPWM(motor2_sensor_pin, motor2_min_raw, motor2_max_raw);

void doPWM1()
{
    sensor1.handlePWM();
}

void doPWM2()
{
    sensor2.handlePWM();
}

void motor_init()
{
    sensor1.init();
    sensor2.init();
    // sensor1.enableInterrupt(doPWM1);
    // sensor2.enableInterrupt(doPWM2);

    driver1.voltage_power_supply = 16;
    driver1.init();

    driver2.voltage_power_supply = 16;
    driver2.init();

    motor1.linkSensor(&sensor1);
    motor2.linkSensor(&sensor2);
    motor1.linkDriver(&driver1);
    motor2.linkDriver(&driver2);

    // Set control mode based on closed-loop flag
    motor1.voltage_sensor_align = 5;
    
    // motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;
    // motor1.controller = MotionControlType::torque;
    motor1.voltage_limit = 10;
    motor1.LPF_velocity.Tf = 0.01;
    motor1.useMonitoring(Serial);
    //motor1.velocity_limit = 40;

    motor2.voltage_sensor_align = 5;
    motor2.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor2.controller = MotionControlType::torque;
    motor2.voltage_limit = 10;
    motor2.LPF_velocity.Tf = 0.01;
    motor2.useMonitoring(Serial);
    //motor2.velocity_limit = 40;

    motor1.init();
    motor2.init();

    motor1.initFOC();
    motor2.initFOC();

    // motor1.disable();
    // motor2.disable();
    

}



void Ctrl_loop()
{
    motor1.loopFOC();
    motor2.loopFOC();
}

void torqueCtrl(float torque)
{
    motor1.target = torque / Kt;
    motor2.target = -torque / Kt;
    
}

float getVelocity_motor1()  { return motor1.shaft_velocity;}
float getVelocity_motor2()  { return motor2.shaft_velocity;}
float getAngle_motor1()  { return motor1.shaft_angle; }
float getAngle_motor2()  { return motor2.shaft_angle; }

#endif