#ifndef _4S4D_CONTROL_HPP
#define _4S4D_CONTROL_HPP
#include <iostream>
#include <memory>
#include <vector>
#include <cmath>
#include "kilin_description/Base_Strategy.hpp"

class _4S4D_ : public ControlStrategy{
public:
    _4S4D_(const sensor_msgs::msg::JointState & JS) {
        start_time = JS.header.stamp.sec + JS.header.stamp.nanosec * 1e-9;
        std::cout << start_time << std::endl;
    }
    
    Robot_CMD _4s4d_init_() { //reset all motors to target motor mode
        State_buffer = ControlStrategy::_init_();
        std::cout << "4s4dinit" << std::endl;
        
        // Initialize wheel
        State_buffer.wheel.FNC = {
            {"FL", FNC_Type::echo}, 
            {"FR", FNC_Type::echo}, 
            {"RL", FNC_Type::control}, 
            {"RR", FNC_Type::control},
            {"ALL", FNC_Type::echo}};
        State_buffer.wheel.mode = Command_Type::velocity;
        
        return State_buffer;
    }

    Robot_CMD compute(std::string & reset, const sensor_msgs::msg::JointState & JS, const std::map<std::string, int> joint_indices_)override{
        if (reset=="unlocked"){
            reset="locked";
            return this->_4s4d_init_();
        }
      
        Pitch = 0;
        // Pitch = (JS.position[joint_indices_.at("FL_hip")])/2+
        //         (JS.position[joint_indices_.at("FR_hip")])/2;
                // (JS.position[joint_indices_.at("RL_hip")])/2+
                // (JS.position[joint_indices_.at("RR_hip")])/2; //change to IMU data in future
        Wheel_base = 0.45;//m
        Length_base = 0.48*cos(Pitch);
        Wheel_base/=2;
        Length_base/=2;
        // wheel_info Wheel_INFO;
        this->v_COM = {1,0,0};
        this->r_FL={Length_base, Wheel_base};
        this->r_FR={Length_base, -Wheel_base};
        this->r_RL={-Length_base, Wheel_base};
        this->r_RR={-Length_base, -Wheel_base};
        this->v_FL=vxvy(r_FL);
        this->v_FR=vxvy(r_FR);
        this->v_RL=vxvy(r_RL);
        this->v_RR=vxvy(r_RR);
        //TODO 
        steering_FL = reverse_check(steer_angle(v_FL)) ? (steer_angle(v_FL)-M_PI* (steer_angle(v_FL)/abs(steer_angle(v_FL)))) : steer_angle(v_FL);
        wheel_speed_FL = reverse_check(steer_angle(v_FL)) ? -wheel_speed(v_FL) : wheel_speed(v_FL);
        steering_FR = reverse_check(steer_angle(v_FR)) ? (steer_angle(v_FR)-M_PI* (steer_angle(v_FR)/abs(steer_angle(v_FR)))) : steer_angle(v_FR);
        wheel_speed_FR = reverse_check(steer_angle(v_FR)) ? -wheel_speed(v_FR) : wheel_speed(v_FR);
        steering_RL = reverse_check(steer_angle(v_RL)) ? (steer_angle(v_RL)-M_PI* (steer_angle(v_RL)/abs(steer_angle(v_RL)))) : steer_angle(v_RL);
        wheel_speed_RL = reverse_check(steer_angle(v_RL)) ? -wheel_speed(v_RL) : wheel_speed(v_RL);
        steering_RR = reverse_check(steer_angle(v_RR)) ? (steer_angle(v_RR)-M_PI* (steer_angle(v_RR)/abs(steer_angle(v_RR)))) : steer_angle(v_RR);
        wheel_speed_RR = reverse_check(steer_angle(v_RR)) ? -wheel_speed(v_RR) : wheel_speed(v_RR);

        Control_buffer.hip.unit = Control_Unit::hip;
        Control_buffer.steering.unit = Control_Unit::steering;
        Control_buffer.wheel.unit = Control_Unit::wheel;

        Control_buffer.hip.FNC = {
            {"FL", FNC_Type::control}, 
            {"FR", FNC_Type::control}, 
            {"RL", FNC_Type::control},
            {"RR", FNC_Type::control}, 
            {"ALL", FNC_Type::control}};
        Control_buffer.hip.mode = Command_Type::effort;
        Control_buffer.steering.FNC = {
            {"FL", FNC_Type::control}, 
            {"FR", FNC_Type::control}, 
            {"RL", FNC_Type::control}, 
            {"RR", FNC_Type::control},
            {"ALL", FNC_Type::control}};
        Control_buffer.steering.mode = Command_Type::effort;
        Control_buffer.wheel.FNC = {
            {"FL", FNC_Type::control}, 
            {"FR", FNC_Type::control}, 
            {"RL", FNC_Type::control}, 
            {"RR", FNC_Type::control},
            {"ALL", FNC_Type::control}};
        Control_buffer.wheel.mode = Command_Type::velocity;
        
        Hip_pid = {5000, 0, 500};
        Steering_pid = {50, 0, 5};
        Control_buffer.hip.command = {
            {"FL", ControlStrategy::PID_controller(Pitch, JS.position[joint_indices_.at("FL_hip")], integral_errors_hip[0], previous_errors_hip[0], Hip_pid, 0.0, 0.01)}, 
            {"FR", ControlStrategy::PID_controller(Pitch, JS.position[joint_indices_.at("FR_hip")], integral_errors_hip[1], previous_errors_hip[1], Hip_pid, 0.0, 0.01)}, 
            {"RL", ControlStrategy::PID_controller(Pitch, JS.position[joint_indices_.at("RL_hip")], integral_errors_hip[2], previous_errors_hip[2], Hip_pid, 0.0, 0.01)}, 
            {"RR", ControlStrategy::PID_controller(Pitch, JS.position[joint_indices_.at("RR_hip")], integral_errors_hip[3], previous_errors_hip[3], Hip_pid, 0.0, 0.01)}};

        Control_buffer.steering.command = {
            {"FL", ControlStrategy::PID_controller(this->steering_FL, JS.position[joint_indices_.at("FL_steering")], integral_errors_steering[0], previous_errors_steering[0], Steering_pid, 0.0, 0.01)}, 
            {"FR", ControlStrategy::PID_controller(this->steering_FR, JS.position[joint_indices_.at("FR_steering")], integral_errors_steering[1], previous_errors_steering[1], Steering_pid, 0.0, 0.01)}, 
            {"RL", ControlStrategy::PID_controller(this->steering_RL, JS.position[joint_indices_.at("RL_steering")], integral_errors_steering[2], previous_errors_steering[2], Steering_pid, 0.0, 0.01)}, 
            {"RR", ControlStrategy::PID_controller(this->steering_RR, JS.position[joint_indices_.at("RR_steering")], integral_errors_steering[3], previous_errors_steering[3], Steering_pid, 0.0, 0.01)}};
        
        Control_buffer.wheel.command = {
            {"FL", ControlStrategy::PID_controller(this->wheel_speed_FL, JS.velocity[joint_indices_.at("FL_wheel")], integral_errors_wheel[0], previous_errors_wheel[0], Wheel_pid, 0.0, 0.01)}, 
            {"FR", ControlStrategy::PID_controller(this->wheel_speed_FR, JS.velocity[joint_indices_.at("FR_wheel")], integral_errors_wheel[1], previous_errors_wheel[1], Wheel_pid, 0.0, 0.01)}, 
            {"RL", ControlStrategy::PID_controller(this->wheel_speed_RL, JS.velocity[joint_indices_.at("RL_wheel")], integral_errors_wheel[2], previous_errors_wheel[2], Wheel_pid, 0.0, 0.01)}, 
            {"RR", ControlStrategy::PID_controller(this->wheel_speed_RR, JS.velocity[joint_indices_.at("RR_wheel")], integral_errors_wheel[3], previous_errors_wheel[3], Wheel_pid, 0.0, 0.01)}};
        
        return Control_buffer;
    }
private:

    double Pitch, Wheel_base, Length_base;
    
    //method
    //COM_velocity
    // Velocity_Input v_COM;
    //position x,y
    std::vector<double> r_FL = {0.0, 0.0};
    std::vector<double> r_FR = {0.0, 0.0};
    std::vector<double> r_RL = {0.0, 0.0};
    std::vector<double> r_RR = {0.0, 0.0}; 
    
    //linear velocity x,y
    std::vector<double> vxvy(std::vector<double> r){
        return {v_COM.Vx-v_COM.Wz* r[1], v_COM.Vy+v_COM.Wz* r[0]};
    };
    std::vector<double> v_FL;
    std::vector<double> v_FR;
    std::vector<double> v_RL;
    std::vector<double> v_RR;
    //steering angle & wheel speed
    double steer_angle(std::vector<double> v){
        return atan2(v[1], v[0]);
    };
    double wheel_speed(std::vector<double> v){
        return sqrt(v[0]*v[0]+v[1]*v[1])/0.0525;//W_w
    };

    bool reverse_check(double angle){
        if (abs(angle)>=M_PI/2){
            return true;
        }else{
            return false;
        }
    };
    
    double steering_FL, wheel_speed_FL, steering_FR, wheel_speed_FR, steering_RL, wheel_speed_RL, steering_RR, wheel_speed_RR;

    //TODO 
    // double steering_FL = reverse_check(steer_angle(v_FL)) ? (steer_angle(v_FL)-M_PI* (steer_angle(v_FL)/abs(steer_angle(v_FL)))) : steer_angle(v_FL);
    // double wheel_speed_FL = reverse_check(steer_angle(v_FL)) ? -wheel_speed(v_FL) : wheel_speed(v_FL);
    // double steering_FR = reverse_check(steer_angle(v_FR)) ? (steer_angle(v_FR)-M_PI* (steer_angle(v_FR)/abs(steer_angle(v_FR)))) : steer_angle(v_FR);
    // double wheel_speed_FR = reverse_check(steer_angle(v_FR)) ? -wheel_speed(v_FR) : wheel_speed(v_FR);
    // double steering_RL = reverse_check(steer_angle(v_RL)) ? (steer_angle(v_RL)-M_PI* (steer_angle(v_RL)/abs(steer_angle(v_RL)))) : steer_angle(v_RL);
    // double wheel_speed_RL = reverse_check(steer_angle(v_RL)) ? -wheel_speed(v_RL) : wheel_speed(v_RL);
    // double steering_RR = reverse_check(steer_angle(v_RR)) ? (steer_angle(v_RR)-M_PI* (steer_angle(v_RR)/abs(steer_angle(v_RR)))) : steer_angle(v_RR);
    // double wheel_speed_RR = reverse_check(steer_angle(v_RR)) ? -wheel_speed(v_RR) : wheel_speed(v_RR);

    
};

#endif // _4S4D_CONTROL_HPP