#include "rclcpp/rclcpp.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "std_msgs/msg/float64_multi_array.hpp" 
#include "sensor_msgs/msg/joint_state.hpp"
#include <string>
#include <vector>
#include <map>
#include <chrono>
#include <memory>
#include <mutex>
#include "kilin_description/Base_Strategy.hpp"
#include "kilin_description/4S4D_control.hpp"

using namespace std::chrono_literals;

const std::vector<std::string> JOINT_NAMES = {
       "FL_hip", "FR_hip", "RL_hip", "RR_hip",
         "FL_steering", "FR_steering", "RL_steering", "RR_steering",
            "FL_wheel", "FR_wheel", "RL_wheel", "RR_wheel"
};

// ----------------------------------------------------------------------
// Node
// ----------------------------------------------------------------------
class Kilin_Coordinator : public rclcpp::Node
{
public:
    Kilin_Coordinator() : Node("Kilin_Coordinator_Node")
    {
        // 1. Initialize Publishers and Subscribers
        init_CMD_pub_and_STATE_sub();
        
        // 2. Set up SwitchController service client
        switch_cli_ = this->create_client<controller_manager_msgs::srv::SwitchController>(
            "/controller_manager/switch_controller"
        );
        
        RCLCPP_INFO(this->get_logger(), "Kilin controller node launched. Checking topic setup...");
    }

private:

    // parameters
    std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr> CMD_PUBS;
    std::map<std::string, std::string> ACTIVE_controllers_;
    std::string current_controller_; 
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr STATE_SUB;
    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_cli_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;
    std::mutex data_mutex_;
    std::shared_ptr<ControlStrategy> strategy_, strategy_A;
    double control_hz_ = 100.0;
    
    enum MotorID {
            FL_hip = 0, FR_hip, RL_hip, RR_hip,
            FL_steer, FR_steer, RL_steer, RR_steer,
            FL_wheel, FR_wheel, RL_wheel, RR_wheel,
            TOTAL_MOTORS = 12
        };

    bool state_map_init_ = false;
    std::string strategy_lock = "unlocked";
    std::map<std::string, int> joint_indices_;
    
    // --- Initialization ---
    void init_CMD_pub_and_STATE_sub()
    {
        std::string pos_name;
        std::string vel_name;
        std::string eff_name;
        const std::string VEL_SUFFIX = "_velocity_controller";
        const std::string POS_SUFFIX = "_position_controller";
        const std::string EFF_SUFFIX = "_effort_controller";

        // hip (Pos, Vel, Eff)
        pos_name = "ALL_hip" + POS_SUFFIX;
        CMD_PUBS[pos_name] = create_publisher<std_msgs::msg::Float64MultiArray>("/" + pos_name + "/commands", 10);
        vel_name = "ALL_hip" + VEL_SUFFIX;
        CMD_PUBS[vel_name] = create_publisher<std_msgs::msg::Float64MultiArray>("/" + vel_name + "/commands", 10);
        eff_name = "ALL_hip" + EFF_SUFFIX;
        CMD_PUBS[eff_name] = create_publisher<std_msgs::msg::Float64MultiArray>("/" + eff_name + "/commands", 10);
        ACTIVE_controllers_["ALL_hip"] = eff_name; // default effort

        // steering (Pos, Eff)
        pos_name = "ALL_steering" + POS_SUFFIX;
        CMD_PUBS[pos_name] = create_publisher<std_msgs::msg::Float64MultiArray>("/" + pos_name + "/commands", 10);
        eff_name = "ALL_steering" + EFF_SUFFIX;
        CMD_PUBS[eff_name] = create_publisher<std_msgs::msg::Float64MultiArray>("/" + eff_name + "/commands", 10);
        ACTIVE_controllers_["ALL_steering"] = eff_name; // default effort

        // wheel (Pos, Vel)
        // FR_wheel
        pos_name = "FR_wheel" + POS_SUFFIX;
        CMD_PUBS[pos_name] = create_publisher<std_msgs::msg::Float64MultiArray>("/" + pos_name + "/commands", 10);
        vel_name = "FR_wheel" + VEL_SUFFIX;
        CMD_PUBS[vel_name] = create_publisher<std_msgs::msg::Float64MultiArray>("/" + vel_name + "/commands", 10);
        eff_name = "FR_wheel" + EFF_SUFFIX;
        CMD_PUBS[eff_name] = create_publisher<std_msgs::msg::Float64MultiArray>("/" + eff_name + "/commands", 10);
        ACTIVE_controllers_["FR_wheel"] = vel_name; // default velocity

        // FL_wheel
        pos_name = "FL_wheel" + POS_SUFFIX;
        CMD_PUBS[pos_name] = create_publisher<std_msgs::msg::Float64MultiArray>("/" + pos_name + "/commands", 10);
        vel_name = "FL_wheel" + VEL_SUFFIX;
        CMD_PUBS[vel_name] = create_publisher<std_msgs::msg::Float64MultiArray>("/" + vel_name + "/commands", 10);
        eff_name = "FL_wheel" + EFF_SUFFIX;
        CMD_PUBS[eff_name] = create_publisher<std_msgs::msg::Float64MultiArray>("/" + eff_name + "/commands", 10);
        ACTIVE_controllers_["FL_wheel"] = vel_name; // default velocity

        // RR_wheel
        pos_name = "RR_wheel" + POS_SUFFIX;
        CMD_PUBS[pos_name] = create_publisher<std_msgs::msg::Float64MultiArray>("/" + pos_name + "/commands", 10);
        vel_name = "RR_wheel" + VEL_SUFFIX;
        CMD_PUBS[vel_name] = create_publisher<std_msgs::msg::Float64MultiArray>("/" + vel_name + "/commands", 10);
        eff_name = "RR_wheel" + EFF_SUFFIX;
        CMD_PUBS[eff_name] = create_publisher<std_msgs::msg::Float64MultiArray>("/" + eff_name + "/commands", 10);
        ACTIVE_controllers_["RR_wheel"] = vel_name; // default velocity

        // RL_wheel
        pos_name = "RL_wheel" + POS_SUFFIX;
        CMD_PUBS[pos_name] = create_publisher<std_msgs::msg::Float64MultiArray>("/" + pos_name + "/commands", 10);
        vel_name = "RL_wheel" + VEL_SUFFIX;
        CMD_PUBS[vel_name] = create_publisher<std_msgs::msg::Float64MultiArray>("/" + vel_name + "/commands", 10);
        eff_name = "RL_wheel" + EFF_SUFFIX;
        CMD_PUBS[eff_name] = create_publisher<std_msgs::msg::Float64MultiArray>("/" + eff_name + "/commands", 10);
        ACTIVE_controllers_["RL_wheel"] = vel_name; // default velocity

        
        STATE_SUB = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) { //use lambda instead of std::bind
                std::lock_guard<std::mutex> lock(this->data_mutex_);
                this->latest_joint_state_ = msg;
                //RCLCPP_INFO(this->get_logger(), "%f",this->latest_joint_state_->effort[2]);
                if (state_map_init_==false) {
                    RCLCPP_INFO(this->get_logger(), "Need to set the map of joint states");
                    this->map_indices_once(msg);
                }
            }
        );

        control_timer_ = this->create_wall_timer(
            1ms, // period
            std::bind(&Kilin_Coordinator::timer_control_loop, this)
        );

        RCLCPP_INFO(this->get_logger(), "Successfully Initializing ALL Publishers & Subscribers! Total Publishers: %zu", CMD_PUBS.size());
    }

    // --- State INFO Mapping ---
    void map_indices_once(const sensor_msgs::msg::JointState::SharedPtr msg) {
        int found_count = 0;
        for (size_t i = 0; i < msg->name.size(); ++i) {
            for (int j = 0; j < TOTAL_MOTORS; ++j) {
                if (msg->name[i].c_str() == JOINT_NAMES[j]) {
                    RCLCPP_INFO(this->get_logger(), "Joint %d name %s is at %d", found_count,msg->name[i].c_str(),j);
                    joint_indices_[JOINT_NAMES[j]] = i;
                    found_count++;
                }
            }
        }
        if (found_count == TOTAL_MOTORS) {
            state_map_init_ = true;
            RCLCPP_INFO(this->get_logger(), "Motor mapping complete！");
        }
    }

    void timer_control_loop() {

        std::lock_guard<std::mutex> lock(data_mutex_);
        
        if (!this->latest_joint_state_ && !state_map_init_) return;
        
        Robot_CMD results;
        // check if the strategy could be changed or not
        if(strategy_lock == "unlocked") {//
            RCLCPP_INFO(this->get_logger(), "strategy changing...");
            strategy_ = std::make_shared<_4S4D_>(*latest_joint_state_);
            //results = strategy_->_init_();
            RCLCPP_INFO(this->get_logger(), "strategy changed");
            //strategy_change = false;
        }
        results = strategy_->compute(strategy_lock, *latest_joint_state_,  joint_indices_);
        // --- Act ---
        this->process_unit_logic(results.hip);//"ALL_hip"
        this->process_unit_logic(results.steering);//"ALL_steering"
        this->process_unit_logic(results.wheel);//"ALL_wheel"

        return;
    }

    // --- Core function: switching joint cmd interface ---
    /* joint_name options: ALL_hip, ALL_steering, FL_wheel, FR_wheel, RL_wheel, RR_wheel */
    /* new_mode options: */
    /* ALL_hip: position, velocity, effort*/
    /* ALL_steering: position, velocity, effort*/
    /* FL_wheel: idle, position, velocity*/
    /* FR_wheel: idle, position, velocity*/
    /* RL_wheel: idle, position, velocity*/
    /* RR_wheel: idle, position, velocity*/
    void Switch_Joint_Mode(const std::string& joint_name, const std::string& new_mode)
    {
        switch_cli_->wait_for_service();
        RCLCPP_INFO(this->get_logger(), "Connected to SwitchController service.");

        std::string old_controller = this->ACTIVE_controllers_[joint_name]; 
        std::string new_controller;
        if (new_mode == "idle") {
            new_controller = "idle"; 
        }else{
            new_controller = joint_name + "_" + new_mode + "_controller"; 
        }
        
        if (old_controller == new_controller) {
            RCLCPP_INFO(this->get_logger(), "%s already in %s mode.", joint_name.c_str(), new_mode.c_str());
            return;
        }

        RCLCPP_WARN(this->get_logger(), "Preparing switching controller of %s: %s -> %s", 
                    joint_name.c_str(), old_controller.c_str(), new_controller.c_str());
        
        auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
        
        if (old_controller == "idle" || old_controller.empty()) {
            request->deactivate_controllers = {}; 
        }else{
            request->deactivate_controllers = {old_controller};
        }

        if (new_controller == "idle") {
            request->activate_controllers = {}; 
        }else{
            request->activate_controllers = {new_controller};
        }

        request->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
        request->timeout = rclcpp::Duration::from_seconds(1.0);
        
        if (!switch_cli_->wait_for_service(std::chrono::milliseconds(10))) {
            RCLCPP_WARN(this->get_logger(), "SwitchController service is available.");
            return;
        }

        switch_cli_->async_send_request(request, 
            [this, joint_name, old_controller, new_controller](rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future) {
                
                if (future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
                    RCLCPP_ERROR(this->get_logger(), "Switch service call failed (Not ready).");
                    return;
                }

                auto response = future.get();
                
                if (response && response->ok) {
                    this->ACTIVE_controllers_[joint_name] = new_controller;
                    RCLCPP_INFO(this->get_logger(), 
                                "✅ Successfully switched %s: %s -> %s.", 
                                joint_name.c_str(), 
                                old_controller.c_str(), 
                                new_controller.c_str());
                } else {
                    RCLCPP_ERROR(this->get_logger(), 
                                "❌ Failed to switch %s to %s. Keeping %s active.", 
                                joint_name.c_str(), new_controller.c_str(), old_controller.c_str());
                }
            }
        );
    } 
    
    std::map<Control_Unit, std::string> unit_ = {
        {Control_Unit::hip, "hip"},
        {Control_Unit::steering, "steering"},
        {Control_Unit::wheel, "wheel"}
    };
    std::map<FNC_Type, std::string> fnc_ = {
        {FNC_Type::echo,  "echo"},
        {FNC_Type::control, "control"},
        {FNC_Type::motor_mode, "motor_mode"}
    };
    std::map<Command_Type, std::string> mode_ = {
        {Command_Type::idle,     "idle"},
        {Command_Type::position, "position"},
        {Command_Type::effort,   "effort"},
        {Command_Type::velocity, "velocity"}
    };
    

    // Decode：
    void process_unit_logic(const ControlOutput& output) {
        std::string unit_name;
        std::string function_type;
        std::string target_mode; //idle, position, velocity, effort, 
        
        if(output.unit == Control_Unit::wheel){
            
            for(int i=0;i<4;i++){
                //RCLCPP_INFO(this->get_logger(), "wheel.");

                switch(i){
                    case 3:
                        unit_name = "FL";
                        break;
                    case 2:
                        unit_name = "FR";
                        break;
                    case 1:
                        unit_name = "RL";
                        break;
                    case 0:
                        unit_name = "RR";
                        break;
                }
                if (output.FNC.at(unit_name) == FNC_Type::echo) {
                    target_mode = "effort";//############### can't set uncontrolled as reality
                    unit_name += "_wheel";
                    RCLCPP_WARN(this->get_logger(), "echo motor %s, mode %s", unit_name.c_str(), target_mode.c_str());
                    this->Switch_Joint_Mode(unit_name, target_mode);
                }else if (output.FNC.at(unit_name) == FNC_Type::motor_mode){
                    // target_mode = mode_[output.mode.at(unit_name)];
                    target_mode = mode_[output.mode];
                    unit_name += "_wheel";
                    RCLCPP_WARN(this->get_logger(), "motor mode motor %s, mode %s", unit_name.c_str(), target_mode.c_str());

                    this->Switch_Joint_Mode(unit_name, target_mode);
                }else if (output.FNC.at(unit_name) == FNC_Type::control) {
                    // target_mode = mode_[output.mode.at(unit_name)];
                    // target_mode = mode_[output.mode];
                    auto sentdata = std_msgs::msg::Float64MultiArray();    
                    // Iteration: (FL, FR, RL, RR)
                    sentdata.data = {output.command.at(unit_name)};
                    //RCLCPP_WARN(this->get_logger(), "motor %f", output.command.at(unit_name));
                    unit_name += "_wheel";  
                    current_controller_ = ACTIVE_controllers_[unit_name];
                   //RCLCPP_WARN(this->get_logger(), "current_controller_: %s, %f", current_controller_.c_str(), sentdata.data[0]);
                    if (CMD_PUBS.count(current_controller_)) {
                        if(current_controller_.find("effort") != std::string::npos){
                            //RCLCPP_WARN(this->get_logger(), "idle");
                            sentdata.data = {0.0};
                            //RCLCPP_WARN(this->get_logger(), "current_controller_: %s, %f", current_controller_.c_str(), sentdata.data[0]);
                        }else{
                            //entdata.data = {300.0};
                            //RCLCPP_WARN(this->get_logger(), "control");
                            //RCLCPP_WARN(this->get_logger(), "current_controller_: %s, %f", current_controller_.c_str(), sentdata.data[0]);
                        }
                        CMD_PUBS[current_controller_]->publish(sentdata);
                    }else {
                        RCLCPP_WARN(this->get_logger(), "No publisher for %s", current_controller_.c_str());
                        return;
                    }
                }
            }
            
        }else{
            unit_name = "ALL"; //ALL_hip, ALL_steering
            if (output.FNC.at(unit_name) == FNC_Type::echo) {
                target_mode = "effort";//############### cant set uncontrolled as reality
                unit_name = unit_name+"_"+unit_[output.unit];
                this->Switch_Joint_Mode(unit_name, target_mode);
            }else if (output.FNC.at(unit_name) == FNC_Type::motor_mode){
                // target_mode = mode_[output.mode.at(unit_name)];
                target_mode = mode_[output.mode];
                unit_name = unit_name+"_"+unit_[output.unit];
                this->Switch_Joint_Mode(unit_name, target_mode);
            }else if (output.FNC.at(unit_name) == FNC_Type::control) {
                // target_mode = mode_[output.mode.at(unit_name)];
                target_mode = mode_[output.mode];
                auto sentdata = std_msgs::msg::Float64MultiArray();    
                // Iteration (FL, FR, RL, RR)
                sentdata.data = {output.command.at("FL"),
                            output.command.at("FR"),
                            output.command.at("RL"),
                            output.command.at("RR")};

                unit_name = unit_name+"_"+unit_[output.unit];
                current_controller_ = ACTIVE_controllers_[unit_name];
                if (CMD_PUBS.count(current_controller_)) CMD_PUBS[current_controller_]->publish(sentdata);
                else {
                    RCLCPP_WARN(this->get_logger(), "No publisher for %s", current_controller_.c_str());
                }
            }
        }
        return;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Kilin_Coordinator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
