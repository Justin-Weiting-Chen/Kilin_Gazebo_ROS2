#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp> // 建議改用 array 一次發送 12 顆，或維持個別發送
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <vector>
#include <map>
#include <string>



using namespace std::chrono_literals;

// 機器人的所有數據匯總
struct RobotContext {
    // Inputs (Sensors)
    std::vector<double> current_joint_state;
    // std::vector<double> imu_data;
    
    // Outputs (Commands)
    std::vector<double> target_joint_commands; // 12 joints

    // Parameters
    double step_height;
    double cycle_time;

    RobotContext() : current_joint_state(12, 0.0), target_joint_commands(12, 0.0) {}
};

class RobotCoordinator : public rclcpp::Node {
private:

    std::map<std::string, double> pos_, vel_, eff_;
    std::shared_ptr<LocomotionStrategy> current_strategy_;
    std::vector<std::string> joint_names_;
    std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> pubs_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    

    void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        for (size_t i = 0; i < msg->name.size(); ++i) {
            pos_[msg->name[i]] = msg->position[i];
            eff_[msg->name[i]] = msg->effort[i];
        }
    }

    void control_loop() {
        if (pos_.empty()) return;

        // --- 策略切換邏輯 ---
        // 假設你未來有一個感測器 Topic 告訴你 "is_stair_nearby"
        bool stair_detected = detect_stair_logic(); 

        if (stair_detected && current_strategy_->get_name() == "WHEELED") {
            RCLCPP_INFO(this->get_logger(), "Stair detected! Switching to FSM Strategy...");
            
            // 第一步：切換 ROS 2 Controller (例如從 Velocity 切換到 Position)
            switch_hardware_controller({"climbing_pos_controller"}, {"wheeled_vel_controller"});
            
            // 第二步：更換策略算法
            current_strategy_ = std::make_shared<StairClimberFSM>();
        }

        // --- 執行策略計算 ---
        std::vector<double> cmds = current_strategy_->compute_commands(pos_, vel_, eff_);

        // --- 發布指令給 12 顆馬達 ---
        if (cmds.size() == 12) {
            for (size_t i = 0; i < 12; ++i) {
                auto msg = std_msgs::msg::Float64();
                msg.data = cmds[i];
                pubs_[joint_names_[i]]->publish(msg);
            }
        }
    }

    // 調用 ROS 2 Control 服務
    void switch_hardware_controller(std::vector<std::string> start, std::vector<std::string> stop) {
        auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
        request->start_controllers = start;
        request->stop_controllers = stop;
        request->strictness = request->STRICT;
        switch_client_->async_send_request(request);
    }

    bool detect_stair_logic() {
        // 這裡未來可以放感測器判斷，目前可手動觸發測試
        return false; 
    }

 


};