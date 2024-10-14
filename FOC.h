#include "SimpleFOC.h"

class motorCtrl
{
    motor = BLDCMotor(14);
    BLDCDriver3PWM driver;
    MagneticSensorPWM sensor;
    motorCtrl(int pwmA, int pwmB, int pwmC, int enablepin){
        driver = BLDCDriver3PWM(pwmA, pwmB, pwmC, enablepin);
    }

    void doPWM(){sensor.handlePWM();}

    void motorsensor(int pin, int min_raw, int max_raw){
        sensor = MagneticSensorPWM(pin, min_raw, max_raw);
    }

    void init(){
        sensor.init();
        sensor.enableInterrupt(doPWM);

        // driver config
        driver.voltage_power_supply = 22.2;
        driver.init();

        motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
        motor.torque_controller = TorqueControlType::voltage;
        motor.controller = ControlType::torque;

        motor.linkSensor(&sensor);
        motor.linkDriver(&driver);

        motor.init();

        // 校正传感器并启动FOC
        //motor.initFOC(zero_electric_offset, sensor_direction);
        motor.initFOC(2.15, Direction::CW);
        
        // enable motor
        motor.enable();
    }

    // 电压扭矩转换
    double torque2voltage(double torque){
        if( torque > 0.21) torque = 0.21;
        if( torque < -0.21) torque = -0.21;
        return torque / 0.21 * 16;
    }

    void Ctrl_loop(){
        motor.loopFOC();
    }

    // 电压扭矩控制
    void torqueCtrl(double torque){
        motor.move(torque2voltage(torque));
    }
    
}