// BaseStrategy.hpp
#ifndef BASE_STRATEGY_HPP
#define BASE_STRATEGY_HPP

#include "std_msgs/msg/float64_multi_array.hpp" 
#include "sensor_msgs/msg/joint_state.hpp"
#include <vector>
#include <string>
#include <map>

enum class Control_Unit{
    hip,
    steering,
    wheel
};
enum class FNC_Type{
    echo,        //1
    control,     //2
    motor_mode   //3
};
enum class Command_Type{
    idle,        //1
    position,    //2
    velocity,    //3
    effort,      //4
};

struct ControlOutput{
    // FL,FR,RL,RR
    Control_Unit unit;
    std::map<std::string, FNC_Type> FNC = {
        {"FL", {}},
        {"FR", {}},
        {"RL", {}},
        {"RR", {}},
        {"ALL", {}}
    };
    // std::map<std::string, Command_Type> mode = {
    //     {"FL", Command_Type::idle},
    //     {"FR", Command_Type::idle},
    //     {"RL", Command_Type::idle},
    //     {"RR", Command_Type::idle},
    //     {"ALL", Command_Type::idle}
    // };
    Command_Type mode = Command_Type::idle;

    std::map<std::string, double> command = {
        {"FL", 0.0},
        {"FR", 0.0},
        {"RL", 0.0},
        {"RR", 0.0}
    };
};

struct Robot_CMD{
    ControlOutput hip;
    ControlOutput steering;
    ControlOutput wheel;
};

struct Velocity_Input{
    double Vx;
    double Vy;
    double Wz;
};

// // ----------------------------------------------------------------------
// // Control Strategy Interface
// // ----------------------------------------------------------------------
class ControlStrategy{
public:
    virtual ~ControlStrategy() = default;
    virtual Robot_CMD compute(std::string & reset, const sensor_msgs::msg::JointState & JS, const std::map<std::string, int> joint_indices_ ) = 0;
    // void set_velocity_command(double Vx, double Vy, double Wz){
    //     this->v_COM = {Vx,Vy,Wz};
    // }
    
    double PID_controller(double target, double current, double& integral, double& previous_error, const std::array<int, 3>& K, double forward_torque, double dt){
        double error = target - current;
        integral += error * dt;
        double derivative = (error - previous_error) / dt;
        previous_error = error;
        return K[0] * error + K[1] * integral + K[2] * derivative + forward_torque;
    }

    Velocity_Input v_COM;
    Robot_CMD Control_buffer, State_buffer;
    std::array<int, 3> Hip_pid={50, 0, 5};
    std::array<int, 3> Steering_pid={50, 0, 5};
    std::array<int, 3> Wheel_pid={50, 0, 5};
    double start_time;
    std::array<double, 4> integral_errors_hip, integral_errors_steering, integral_errors_wheel, previous_errors_hip, previous_errors_steering, previous_errors_wheel;
    bool stop_signal = false;
    bool get_stop_signal(){
        return stop_signal;
    }
    Robot_CMD _init_(){ //reset all motors to target motor mode
        State_buffer.hip.unit = Control_Unit::hip;
        State_buffer.steering.unit = Control_Unit::steering;
        State_buffer.wheel.unit = Control_Unit::wheel;

        State_buffer.hip.FNC = {
            {"FL", FNC_Type::control}, 
            {"FR", FNC_Type::control}, 
            {"RL", FNC_Type::control}, 
            {"RR", FNC_Type::control},
            {"ALL", FNC_Type::control}};
        State_buffer.hip.mode = Command_Type::effort;
        State_buffer.steering.FNC = {
            {"FL", FNC_Type::control}, 
            {"FR", FNC_Type::control}, 
            {"RL", FNC_Type::control}, 
            {"RR", FNC_Type::control},
            {"ALL", FNC_Type::control}};
        State_buffer.steering.mode = Command_Type::effort;
        State_buffer.wheel.FNC = {
            {"FL", FNC_Type::control}, 
            {"FR", FNC_Type::control}, 
            {"RL", FNC_Type::control}, 
            {"RR", FNC_Type::control},
            {"ALL", FNC_Type::control}};
        State_buffer.wheel.mode = Command_Type::velocity;
        
        State_buffer.hip.command = {
            {"FL", 0.0}, 
            {"FR", 0.0}, 
            {"RL", 0.0}, 
            {"RR", 0.0}};
        State_buffer.steering.command = {
            {"FL", 0.0}, 
            {"FR", 0.0}, 
            {"RL", 0.0}, 
            {"RR", 0.0}};
        State_buffer.wheel.command = {
            {"FL", 0.0}, 
            {"FR", 0.0}, 
            {"RL", 0.0}, 
            {"RR", 0.0}};
        
        return State_buffer;
    }
};


#endif