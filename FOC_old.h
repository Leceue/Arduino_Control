#ifndef FOC_H
#define FOC_H

#include "SimpleFOC.h"

class motorCtrl {
private:
    BLDCMotor motor = BLDCMotor(14,10.9,33,0.00474);
    BLDCDriver3PWM* driver;
    MagneticSensorPWM* sensor;
    
    // Control parameters
    float torque_constant;       // Nm/V
    float now_torque = 0;
    float Kt = 0.27; // Nm/A 电机转矩常数

    // 存储中断处理函数指针
    void (*pwmHandler)();
    
    // 存储电机ID，用于区分不同电机
    int motorId;
    
    // 存储实例指针
    static motorCtrl* instances[2];  // 支持两个电机实例
    
public:
    motorCtrl(int id, int pwmA, int pwmB, int pwmC, int enablepin) 
        : driver(nullptr)
        , sensor(nullptr)
        , torque_constant(0.27)
        , pwmHandler(nullptr)
        , motorId(id) 
    {
        if (id >= 0 && id < 2) {  // 确保ID在有效范围内
            instances[id] = this;
            driver = new BLDCDriver3PWM(pwmA, pwmB, pwmC, enablepin);
        }
    }
    
    ~motorCtrl() {
        if (driver) delete driver;
        if (sensor) delete sensor;
        if (motorId >= 0 && motorId < 2) {
            instances[motorId] = nullptr;
        }
    }
    
    // 静态方法获取实例
    static motorCtrl* getInstance(int id) {
        if (id >= 0 && id < 2) {
            return instances[id];
        }
        return nullptr;
    }
    
    // 非静态的PWM处理方法
    void handlePWM() {
        if (sensor) {
            sensor->handlePWM();
        }
    }
    
    // 静态中断处理函数
    static void pwmHandlerMotor0() {
        if (instances[0]) {
            instances[0]->handlePWM();
        }
    }
    
    static void pwmHandlerMotor1() {
        if (instances[1]) {
            instances[1]->handlePWM();
        }
    }
    
    void motorsensor(int pin, int min_raw, int max_raw) {
        if (sensor) {
            delete sensor;
        }
        sensor = new MagneticSensorPWM(pin, min_raw, max_raw);
        sensor->init();
        // 根据电机ID选择对应的中断处理函数
        if (motorId == 0) {
            pwmHandler = pwmHandlerMotor0;
        } else if (motorId == 1) {
            pwmHandler = pwmHandlerMotor1;
        }
        
        // 设置中断
        if (sensor && pwmHandler) {
            sensor->enableInterrupt(pwmHandler);
        }
    }
    
    void init() {
        if (!sensor || !driver) return;
        // Link sensor and driver

        motor.linkSensor(sensor);
        
        // Driver configuration
        driver->voltage_power_supply = 16;
        driver->init();
        motor.linkDriver(driver);
        
        // Set control mode based on closed-loop flag
        motor.voltage_sensor_align = 5;
        motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
        motor.controller = MotionControlType::torque;

        motor.voltage_limit = 16;

        //速度低通滤波时间常数
        motor.LPF_velocity.Tf = 0.01;

        motor.useMonitoring(Serial);

        //设置最大速度限制
        motor.velocity_limit = 40;

        // Initialize motor
        motor.init();
        
        // Initialize FOC if using closed-loop control
        motor.initFOC();
    }

    void loopFOC(){
        motor.loopFOC();
    }
    
    void Ctrl_loop() {
        motor.loopFOC();
        motor.target = now_torque/Kt;
        motor.move();
    }
    
    void torqueCtrl(float torque) {
        now_torque = torque;
        // motor.move(target_velocity);
    }
    
    // 获取电机状态的方法
    int getId() const { return motorId; }
    float getVelocity() const { return motor.shaft_velocity;}
    float getAngle() const { return motor.shaft_angle; }
    bool isEnabled() const { return motor.enabled; }
};

// 初始化静态成员变量
motorCtrl* motorCtrl::instances[2] = {nullptr, nullptr};

#endif