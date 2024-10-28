#ifndef FOC_H
#define FOC_H

#include "SimpleFOC.h"

class motorCtrl {
    BLDCMotor motor = BLDCMotor(14);  // 14 pole pairs
    BLDCDriver3PWM driver;
    MagneticSensorPWM sensor;
    bool closedloop_enabled = false;  // Flag for closed-loop control
    
    // Control parameters
    float voltage_limit = 16;         // [V]
    float velocity_limit = 30;        // [rad/s]
    float torque_constant = 0.21;     // Nm/V - for conversion between torque and velocity
    
public:
    motorCtrl(int pwmA, int pwmB, int pwmC, int enablepin) {
        driver = BLDCDriver3PWM(pwmA, pwmB, pwmC, enablepin);
    }
    
    void doPWM() {
        sensor.handlePWM();
    }
    
    void motorsensor(int pin, int min_raw, int max_raw) {
        sensor = MagneticSensorPWM(pin, min_raw, max_raw);
    }
    
    void init() {
        // Sensor initialization
        sensor.init();
        sensor.enableInterrupt(doPWM);
        
        // Driver configuration
        driver.voltage_power_supply = 22.2;
        driver.init();
        
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
            motor.linkSensor(&sensor);
        }
        motor.linkDriver(&driver);
        
        // Initialize motor
        motor.init();
        
        // Initialize FOC if using closed-loop control
        if (closedloop_enabled) {
            motor.initFOC(2.15, Direction::CW);
        }
        
        // Enable motor
        motor.enable();
    }
    
    // Convert desired torque to equivalent velocity
    float torque2velocity(float torque) {
        // Limit torque to maximum value
        if (torque > torque_constant) torque = torque_constant;
        if (torque < -torque_constant) torque = -torque_constant;
        
        // Convert torque to velocity
        // Using a simple linear relationship: velocity = k * torque
        // Where k is a scaling factor to convert within acceptable velocity range
        float k = velocity_limit / torque_constant;
        return torque * k;
    }
    
    void Ctrl_loop() {
        if (closedloop_enabled) {
            motor.loopFOC();
        }
    }
    
    // Velocity-based torque control
    void torqueCtrl(float torque) {
        float target_velocity = torque2velocity(torque);
        motor.move(target_velocity);
    }
    
    // Method to enable/disable closed-loop control
    void setClosedLoop(bool enable) {
        closedloop_enabled = enable;
        // Reinitialize with new control mode
        init();
    }
    
    // Method to set control parameters
    void setParameters(float v_limit, float vel_limit, float torque_const) {
        voltage_limit = v_limit;
        velocity_limit = vel_limit;
        torque_constant = torque_const;
    }
};

#endif