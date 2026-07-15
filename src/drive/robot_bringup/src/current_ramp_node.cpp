#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <algorithm>
#include <cmath>

// Manual-drive-only soft current limiter.
//
// This is a SEPARATE layer from stability_monitor_node's 45A/3s hard
// cutoff (see control guide 3.3). That cutoff is the last line of
// defense against sustained overcurrent. This node instead reacts to
// brief current spikes -- expected during pivot turns on rubber tracks,
// where ground friction can spike torque current well before any
// sustained-overload condition -- by continuously and proportionally
// scaling down the commanded /cmd_vel, angular.z (rotation) first, so a
// spike self-corrects without ever commanding an abrupt stop. If this
// works as intended, the 45A/3s cutoff should rarely if ever trigger
// during manual driving.
class CurrentRampNode : public rclcpp::Node
{
public:
    CurrentRampNode() : Node("current_ramp_node"),
        left_current_(0.0), right_current_(0.0),
        vx_scale_(1.0), wz_scale_(1.0)
    {
        this->declare_parameter<double>("soft_current_limit_a", 22.0);
        this->declare_parameter<double>("ramp_down_gain", 0.05);   // scale/s per Amp over limit
        this->declare_parameter<double>("ramp_up_rate", 0.2);      // scale/s recovery
        this->declare_parameter<double>("control_rate_hz", 50.0);

        soft_current_limit_ = this->get_parameter("soft_current_limit_a").as_double();
        ramp_down_gain_ = this->get_parameter("ramp_down_gain").as_double();
        ramp_up_rate_ = this->get_parameter("ramp_up_rate").as_double();
        double rate_hz = this->get_parameter("control_rate_hz").as_double();
        period_s_ = 1.0 / rate_hz;

        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_manual_raw", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) { last_cmd_ = *msg; });

        // rmd_x8_driver_node publishes best-effort; must match to receive it.
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/wheel/joint_states", rclcpp::QoS(10).best_effort(),
            std::bind(&CurrentRampNode::jointStateCallback, this, std::placeholders::_1));

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(period_s_),
            std::bind(&CurrentRampNode::controlLoop, this));

        RCLCPP_INFO(this->get_logger(),
            "current_ramp_node started: soft_limit=%.1fA, ramp_down_gain=%.3f, ramp_up_rate=%.3f",
            soft_current_limit_, ramp_down_gain_, ramp_up_rate_);
    }

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        for (size_t i = 0; i < msg->name.size() && i < msg->effort.size(); ++i) {
            if (msg->name[i] == "left_wheel_joint") {
                left_current_ = msg->effort[i];
            } else if (msg->name[i] == "right_wheel_joint") {
                right_current_ = msg->effort[i];
            }
        }
    }

    void controlLoop()
    {
        double max_current = std::max(std::abs(left_current_), std::abs(right_current_));
        double excess = std::max(0.0, max_current - soft_current_limit_);

        if (excess > 0.0) {
            // Rotation is the dominant contributor to track-friction current
            // spikes, so drain it first; only touch forward/back speed if
            // draining rotation alone isn't enough.
            if (wz_scale_ > 0.0) {
                wz_scale_ -= period_s_ * ramp_down_gain_ * excess;
                wz_scale_ = std::max(0.0, wz_scale_);
            } else {
                vx_scale_ -= period_s_ * ramp_down_gain_ * excess;
                vx_scale_ = std::max(0.0, vx_scale_);
            }
        } else {
            wz_scale_ = std::min(1.0, wz_scale_ + period_s_ * ramp_up_rate_);
            vx_scale_ = std::min(1.0, vx_scale_ + period_s_ * ramp_up_rate_);
        }

        geometry_msgs::msg::Twist out;
        out.linear.x = last_cmd_.linear.x * vx_scale_;
        out.angular.z = last_cmd_.angular.z * wz_scale_;
        cmd_pub_->publish(out);

        if (excess > 0.0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "current_ramp: max_current=%.1fA (limit %.1fA) -> vx_scale=%.2f wz_scale=%.2f",
                max_current, soft_current_limit_, vx_scale_, wz_scale_);
        }
    }

    double soft_current_limit_;
    double ramp_down_gain_;
    double ramp_up_rate_;
    double period_s_;

    double left_current_;
    double right_current_;
    double vx_scale_;
    double wz_scale_;
    geometry_msgs::msg::Twist last_cmd_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CurrentRampNode>());
    rclcpp::shutdown();
    return 0;
}
