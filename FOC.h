#ifndef FOC_H
#define FOC_H

#include "SimpleFOC.h"

class motorCtrl {
private:
    BLDCMotor motor;
    BLDCDriver3PWM* driver;
    MagneticSensorPWM* sensor;
    bool closedloop_enabled;
    
    // Control parameters
    float voltage_limit;         // [V]
    float velocity_limit;        // [rad/s]
    float torque_constant;       // Nm/V
    float now_velocity;
    
    // 存储中断处理函数指针
    void (*pwmHandler)();
    
    // 存储电机ID，用于区分不同电机
    int motorId;
    
    // 存储实例指针
    static motorCtrl* instances[2];  // 支持两个电机实例
    
public:
    motorCtrl(int id, int pole_pairs, int pwmA, int pwmB, int pwmC, int enablepin) 
        : motor(pole_pairs)
        , driver(nullptr)
        , sensor(nullptr)
        , closedloop_enabled(false)
        , voltage_limit(16)
        , velocity_limit(30)
        , torque_constant(0.21)
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
        
        // Sensor initialization
        sensor->init();
        
        // Driver configuration
        driver->voltage_power_supply = 22.2;
        driver->init();
        
        // Motor configuration
        motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
        
        // Set control mode based on closed-loop flag
        if (closedloop_enabled) {
            motor.controller = MotionControlType::velocity;
            motor.torque_controller = TorqueControlType::voltage;
        } else {
            motor.controller = MotionControlType::velocity_openloop;
        }
        
        // Set limits
        motor.voltage_limit = voltage_limit;
        motor.velocity_limit = velocity_limit;
        
        // Link sensor and driver
        if (closedloop_enabled) {
            motor.linkSensor(sensor);
        }
        motor.linkDriver(driver);
        
        // Initialize motor
        motor.init();
        
        // Initialize FOC if using closed-loop control
        if (closedloop_enabled) {
            motor.initFOC();
        }
        
        // Enable motor
        motor.enable();
    }
    
    float torque2velocity(float torque) {
        // Limit torque to maximum value
        if (torque > torque_constant) torque = torque_constant;
        if (torque < -torque_constant) torque = -torque_constant;
        
        // Convert torque to velocity
        float k = velocity_limit / torque_constant;
        return torque * k;
    }
    
    void Ctrl_loop() {
        if (closedloop_enabled) {
            motor.loopFOC();
        }
    }
    
    void torqueCtrl(float torque) {
        float target_velocity = torque2velocity(torque);
        now_velocity = target_velocity;
        motor.move(target_velocity);
    }
    
    void setClosedLoop(bool enable) {
        closedloop_enabled = enable;
        init();
    }
    
    void setParameters(float v_limit, float vel_limit, float torque_const) {
        voltage_limit = v_limit;
        velocity_limit = vel_limit;
        torque_constant = torque_const;
    }
    
    // 获取电机状态的方法
    int getId() const { return motorId; }
    // float getVelocity() const { return motor.shaft_velocity; }
    float getVelocity() const { return now_velocity; }
    // float getAngle() const { return motor.shaft_angle; }
    // bool isEnabled() const { return motor.enabled; }
};

// 初始化静态成员变量
motorCtrl* motorCtrl::instances[2] = {nullptr, nullptr};

#endif