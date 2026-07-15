#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_msgs/msg/speed_limit.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <algorithm>
#include <cmath>

class StabilityMonitorNode : public rclcpp::Node
{
public:
    StabilityMonitorNode() : Node("stability_monitor_node"),
      current_pitch_(0.0), current_roll_(0.0),
      left_motor_current_(0.0), right_motor_current_(0.0),
      drive_motor_fault_(false),
      over_current_timer_(0.0)
    {
        // 1. 파라미터 선언 (가이드라인 및 안전마진 반영)
        this->declare_parameter<double>("critical_pitch_deg", 25.0); // 전복 임계각의 60~70% 수준
        this->declare_parameter<double>("critical_roll_deg", 20.0);

        critical_pitch_ = this->get_parameter("critical_pitch_deg").as_double();
        critical_roll_ = this->get_parameter("critical_roll_deg").as_double();

        // 2. 구독자 등록 (IMU 및 구동 모터 피드백)
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&StabilityMonitorNode::imuCallback, this, std::placeholders::_1));

        // RMD-X8 구동 모터 전류 피드백 구독: rmd_x8_driver_node가 실제로 발행하는
        // /wheel/joint_states(effort 필드에 토크전류 A) / /wheel/motor_status를 직접 구독
        // rmd_x8_driver_node가 best_effort QoS로 발행하므로 기본(RELIABLE) 구독과
        // 호환이 안 됨 -> 반드시 BEST_EFFORT로 맞춰야 실제로 메시지가 도착함
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/wheel/joint_states", rclcpp::QoS(10).best_effort(),
            std::bind(&StabilityMonitorNode::jointStateCallback, this, std::placeholders::_1));
        motor_status_sub_ = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
            "/wheel/motor_status", 10,
            std::bind(&StabilityMonitorNode::motorStatusCallback, this, std::placeholders::_1));

        // 3. 퍼블리셔 등록 (Nav2 속도 제한, 안전 비상 제어)
        speed_limit_pub = this->create_publisher<nav2_msgs::msg::SpeedLimit>("/speed_limit", 10);
        safety_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_safety", 10);

        // 100Hz 고속 제어 루프 스레드 기동 (지연 시간 최소화)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&StabilityMonitorNode::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "Advanced Stability & Compliance Control Node Initialized.");
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        tf2::Matrix3x3 m(q);
        double r, p, y;
        m.getRPY(r, p, y);

        current_roll_ = r * 180.0 / M_PI;
        current_pitch_ = p * 180.0 / M_PI;
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        for (size_t i = 0; i < msg->name.size() && i < msg->effort.size(); ++i) {
            if (msg->name[i] == "left_wheel_joint") {
                left_motor_current_ = msg->effort[i];
            } else if (msg->name[i] == "right_wheel_joint") {
                right_motor_current_ = msg->effort[i];
            }
        }
    }

    void motorStatusCallback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
    {
        // 구동 모터 하드웨어 에러/피드백 두절(STALE) 시 즉시 위험 상태로 취급
        drive_motor_fault_ = false;
        for (const auto & status : msg->status) {
            if (status.level == diagnostic_msgs::msg::DiagnosticStatus::ERROR ||
                status.level == diagnostic_msgs::msg::DiagnosticStatus::STALE) {
                drive_motor_fault_ = true;
                RCLCPP_ERROR(this->get_logger(), "[%s] fault: %s", status.name.c_str(), status.message.c_str());
            }
        }
    }

    void controlLoop()
    {
        int safety_level = 0; // 0: NORMAL, 1: CAUTION, 2: CRITICAL
        double abs_p = std::abs(current_pitch_);
        double abs_r = std::abs(current_roll_);

        // 구동 모터 중 더 큰 피크 전류 추출
        double max_drive_current = std::max(std::abs(left_motor_current_), std::abs(right_motor_current_));

        // [안전 로직 1] 자세 임계각 기반 상태 트리거 (2.3절 가이드)
        if (abs_p >= critical_pitch_ || abs_r >= critical_roll_) {
            safety_level = 2;
        } else if (abs_p >= critical_pitch_ * 0.6 || abs_r >= critical_roll_ * 0.6) {
            safety_level = 1;
        }

        // [안전 로직 2] 구동계 2중화 전류 보호: 피크 전류 45A 3초 컷오프 (3.3절 가이드)
        if (max_drive_current >= 45.0) {
            over_current_timer_ += 0.01;
            if (over_current_timer_ >= 3.0) {
                safety_level = 2; // 지속 과전류 발생 시 강제로 최고 위험 레벨 확정
                RCLCPP_ERROR(this->get_logger(), "Drive Motor Overcurrent (>=45A) sustained for 3s! Emergency Active.");
            }
        } else {
            over_current_timer_ = 0.0;
        }

        // [안전 로직 2-1] 구동 모터 하드웨어 에러 또는 피드백 두절(STALE) -> 즉시 최고 위험도
        if (drive_motor_fault_) {
            safety_level = 2;
        }

        // [안전 로직 3] 복합 경사 기반 속도 제한 및 비상 개입 명령 생성
        auto limit_msg = nav2_msgs::msg::SpeedLimit();

        if (safety_level == 0) {
            // Level 0: NORMAL - 제한 없음
            limit_msg.speed_limit = 0.0;
            limit_msg.percentage = false;
        }
        else if (safety_level == 1) {
            // Level 1: CAUTION - 속도 50% 제한
            limit_msg.speed_limit = 50.0;
            limit_msg.percentage = true;
        }
        else if (safety_level == 2) {
            // Level 2: CRITICAL - 속도 15% 서서히 감속, 비상 개입 명령 발행
            limit_msg.speed_limit = 15.0;
            limit_msg.percentage = true;

            // 비상 탈출 및 홀딩 제어 명령 생성
            geometry_msgs::msg::Twist safety_twist;
            if (max_drive_current >= 45.0 && over_current_timer_ >= 3.0) {
                // 구동계 전류 문제인 경우: 모터 홀딩 상태 정지를 위해 속도 0 주입
                safety_twist.linear.x = 0.0;
            } else {
                // 복합 경사 자세 문제인 경우: 전복 방지를 위해 회전은 잠그고 초저속 탈출
                safety_twist.linear.x = 0.05;
            }
            safety_twist.angular.z = 0.0; // 회전 명령 강제 차단
            safety_cmd_pub_->publish(safety_twist);
        }

        // 최종 안전 명령 발행
        speed_limit_pub->publish(limit_msg);
    }

    // 내부 변수
    double critical_pitch_;
    double critical_roll_;

    double current_pitch_;
    double current_roll_;
    double left_motor_current_;
    double right_motor_current_;
    bool drive_motor_fault_;

    double over_current_timer_;

    // ROS 2 인터페이스
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr motor_status_sub_;

    rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_limit_pub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr safety_cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StabilityMonitorNode>());
    rclcpp::shutdown();
    return 0;
}
