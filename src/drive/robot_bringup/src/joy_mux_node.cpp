#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <string>
#include <algorithm>
#include <cmath>

class JoyMuxNode : public rclcpp::Node
{
public:
    JoyMuxNode() : Node("joy_mux_node"),
      is_autonomous_(false),
      is_emergency_stop_(false),
      last_share_pressed_(false), last_ps_pressed_(false),
      last_l1_pressed_(false), last_r1_pressed_(false),
      speed_level_(0)
    {
        // 구독자
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&JoyMuxNode::joyCallback, this, std::placeholders::_1));
        auto_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_auto", 10, std::bind(&JoyMuxNode::autoCmdCallback, this, std::placeholders::_1));

        // 퍼블리셔
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // rmd_x8_driver_node의 max_wheel_speed_dps 파라미터를 원격으로 설정하기 위한 클라이언트
        speed_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "rmd_x8_driver");

        // rmd_x8_driver_node가 뜨는 시점이 이 노드보다 늦을 수 있어, 파라미터 서비스가
        // 준비된 걸 확인한 후 딱 한 번 기본 프리셋(40%)으로 강제 동기화한다
        // (launch 파일이 max_wheel_speed_dps를 안 넘기면 노드 자체 기본값인
        // 3000dps로 뜨므로, 시작 시점 상태를 항상 확정적으로 맞추기 위함)
        speed_init_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            [this]() {
                if (speed_param_client_->service_is_ready()) {
                    setSpeedLevel(0);
                    speed_init_timer_->cancel();
                }
            });

        RCLCPP_INFO(this->get_logger(), "Joy MUX Node Initialized.");
    }

private:
    void autoCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (is_autonomous_ && !is_emergency_stop_) {
            cmd_pub_->publish(*msg);
        }
    }

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        bool share_pressed = msg->buttons[8];
        bool ps_pressed = msg->buttons[10];
        bool l1_pressed = msg->buttons[4];
        bool r1_pressed = msg->buttons[5];

        // SHARE (8): 수동/자율 전환
        if (share_pressed && !last_share_pressed_) {
            is_autonomous_ = !is_autonomous_;
            RCLCPP_INFO(this->get_logger(), "Mode Switched: %s", is_autonomous_ ? "AUTONOMOUS" : "MANUAL");
        }
        // PS (10): 비상 정지
        if (ps_pressed && !last_ps_pressed_) {
            is_emergency_stop_ = !is_emergency_stop_;
            if (is_emergency_stop_) {
                RCLCPP_ERROR(this->get_logger(), "!!! EMERGENCY STOP ACTIVE !!!");
                executeEmergencyStop();
            } else {
                RCLCPP_INFO(this->get_logger(), "Emergency Stop Released.");
            }
        }
        // L1 (4): 구동 속도 프리셋 한 단계 감속 (100% -> 70% -> 40%)
        if (l1_pressed && !last_l1_pressed_) {
            setSpeedLevel(speed_level_ - 1);
        }
        // R1 (5): 구동 속도 프리셋 한 단계 가속 (40% -> 70% -> 100%)
        if (r1_pressed && !last_r1_pressed_) {
            setSpeedLevel(speed_level_ + 1);
        }

        last_share_pressed_ = share_pressed;
        last_ps_pressed_ = ps_pressed;
        last_l1_pressed_ = l1_pressed;
        last_r1_pressed_ = r1_pressed;

        if (is_emergency_stop_) {
            executeEmergencyStop();
            return;
        }

        // --- 수동 제어 모드 ---
        if (!is_autonomous_) {
            geometry_msgs::msg::Twist manual_twist;
            manual_twist.linear.x = msg->axes[1] * 0.6;
            manual_twist.angular.z = msg->axes[0] * 0.8;
            cmd_pub_->publish(manual_twist);
        }
    }

    void executeEmergencyStop() {
        geometry_msgs::msg::Twist stop_twist;
        cmd_pub_->publish(stop_twist);
    }

    // RMD-X8-120 정격 속도 700dps 기준 40/70/100% 프리셋.
    // 인덱스: 0=40%(280dps, 기본), 1=70%(490dps), 2=100%(700dps, 정격 최대)
    double speedDpsForLevel(int level) const {
        switch (level) {
            case 0: return 280.0;
            case 1: return 490.0;
            default: return 700.0;
        }
    }

    int speedPercentForLevel(int level) const {
        switch (level) {
            case 0: return 40;
            case 1: return 70;
            default: return 100;
        }
    }

    void setSpeedLevel(int new_level) {
        new_level = std::clamp(new_level, 0, 2);
        speed_level_ = new_level;
        double dps = speedDpsForLevel(speed_level_);
        RCLCPP_INFO(this->get_logger(), "Drive speed preset: %d%% (%.1f dps)",
            speedPercentForLevel(speed_level_), dps);
        speed_param_client_->set_parameters({rclcpp::Parameter("max_wheel_speed_dps", dps)});
    }

    bool is_autonomous_;
    bool is_emergency_stop_;
    bool last_share_pressed_;
    bool last_ps_pressed_;
    bool last_l1_pressed_;
    bool last_r1_pressed_;
    int speed_level_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr auto_cmd_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::AsyncParametersClient::SharedPtr speed_param_client_;
    rclcpp::TimerBase::SharedPtr speed_init_timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyMuxNode>());
    rclcpp::shutdown();
    return 0;
}
