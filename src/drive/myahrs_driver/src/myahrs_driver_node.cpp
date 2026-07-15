#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <string>
#include <sstream>
#include <vector>
#include <cstdint>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cmath>

class MyAhrsDriverNode : public rclcpp::Node
{
public:
    MyAhrsDriverNode() : Node("myahrs_driver_node"), serial_fd_(-1)
    {
        // 파라미터 선언 (기본값 설정)
        this->declare_parameter<std::string>("port", "/dev/ttyACM1");
        this->declare_parameter<int>("baudrate", 460800); // 가이드 기준 고속 보드레이트
        this->declare_parameter<std::string>("frame_id", "imu_link");

        // orientation covariance 대각 성분 (정지 상태 실측 기반, 2000 샘플, imu_covariance_calibrator.py 사용)
        this->declare_parameter<double>("orientation_covariance_roll", 0.00000594);
        this->declare_parameter<double>("orientation_covariance_pitch", 0.00003487);
        this->declare_parameter<double>("orientation_covariance_yaw", 0.00051956);

        std::string port = this->get_parameter("port").as_string();
        int baudrate = this->get_parameter("baudrate").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();

        orientation_covariance_roll_ = this->get_parameter("orientation_covariance_roll").as_double();
        orientation_covariance_pitch_ = this->get_parameter("orientation_covariance_pitch").as_double();
        orientation_covariance_yaw_ = this->get_parameter("orientation_covariance_yaw").as_double();

        // IMU 퍼블리셔 등록
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);

        // 시리얼 포트 오픈
        if (!initSerial(port, baudrate)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", port.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Success to open myAHRS+ on %s (%d bps)", port.c_str(), baudrate);

        // 데이터 수신을 위한 타이머 스레드 (100Hz 루프)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&MyAhrsDriverNode::readSerialData, this));
    }

    ~MyAhrsDriverNode()
    {
        if (serial_fd_ >= 0) {
            close(serial_fd_);
        }
    }

private:
    bool initSerial(const std::string& port, int baudrate)
    {
        serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd_ < 0) return false;

        struct termios toptions;
        tcgetattr(serial_fd_, &toptions);

        speed_t brate = B460800;
        if (baudrate == 115200) brate = B115200;

        cfsetispeed(&toptions, brate);
        cfsetospeed(&toptions, brate);

        toptions.c_cflag &= ~PARENB; // No parity
        toptions.c_cflag &= ~CSTOPB; // 1 stop bit
        toptions.c_cflag &= ~CSIZE;
        toptions.c_cflag |= CS8;     // 8 bits
        toptions.c_cflag |= CREAD | CLOCAL;

        toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        toptions.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INPCK | ISTRIP);
        toptions.c_oflag &= ~OPOST;

        tcsetattr(serial_fd_, TCSANOW, &toptions);
        return true;
    }

    void readSerialData()
    {
        char buf[256];
        int n = read(serial_fd_, buf, sizeof(buf) - 1);
        if (n <= 0) return;

        buf[n] = '\0';
        std::string data(buf);
        
        // 간단한 ASCII 데이터 파싱 예시 (myAHRS+ 표준 데이터 문자열 파싱)
        // 실제 데이터 사양: @IIMU,ax,ay,az,gx,gy,gz,mx,my,mz,roll,pitch,yaw,quaternion_x,y,z,w 등
        if (data.find("@IIMU") != std::string::npos || data.find("$") != std::string::npos) {
            parseAndPublish(data);
        }
    }

    void parseAndPublish(const std::string& raw_str)
    {
        // 1. 프로토콜 접두사 확인: "$RPY,<seq>,<roll_deg>,<pitch_deg>,<yaw_deg>*<checksum_hex>"
        if (raw_str.rfind("$RPY,", 0) != 0) {
            return;
        }

        // 2. '*' 기준으로 데이터부/체크섬부 분리 후 체크섬 검증
        size_t star_pos = raw_str.find('*');
        if (star_pos == std::string::npos || star_pos < 1 || star_pos + 2 >= raw_str.size()) {
            RCLCPP_WARN(this->get_logger(), "Malformed RPY frame (no checksum): %s", raw_str.c_str());
            return;
        }

        // 체크섬은 '$' 문자를 포함해 '*' 직전까지의 바이트를 XOR한 값
        // (실측 샘플 검증 결과, '$' 제외 시 체크섬이 일치하지 않음)
        std::string payload = raw_str.substr(1, star_pos - 1);
        uint8_t computed_checksum = 0;
        for (unsigned char c : raw_str.substr(0, star_pos)) {
            computed_checksum ^= c;
        }

        uint8_t received_checksum = 0;
        try {
            received_checksum = static_cast<uint8_t>(std::stoul(raw_str.substr(star_pos + 1, 2), nullptr, 16));
        }
        catch (...) {
            RCLCPP_WARN(this->get_logger(), "Invalid checksum field: %s", raw_str.c_str());
            return;
        }

        if (computed_checksum != received_checksum) {
            RCLCPP_WARN(this->get_logger(), "Checksum mismatch on IMU data: %s", raw_str.c_str());
            return;
        }

        // 3. ',' 기준 필드 파싱: [0]=RPY, [1]=seq, [2]=roll, [3]=pitch, [4]=yaw
        try {
            std::vector<std::string> fields;
            std::stringstream ss(payload);
            std::string field;
            while (std::getline(ss, field, ',')) {
                fields.push_back(field);
            }

            if (fields.size() < 5) {
                RCLCPP_WARN(this->get_logger(), "Incomplete RPY fields: %s", raw_str.c_str());
                return;
            }

            double roll_deg = std::stod(fields[2]);
            double pitch_deg = std::stod(fields[3]);
            double yaw_deg = std::stod(fields[4]);

            double roll_rad = roll_deg * M_PI / 180.0;
            double pitch_rad = pitch_deg * M_PI / 180.0;
            double yaw_rad = yaw_deg * M_PI / 180.0;

            auto imu_msg = sensor_msgs::msg::Imu();
            imu_msg.header.stamp = this->now();
            imu_msg.header.frame_id = frame_id_;

            // 4. roll/pitch/yaw(라디안) -> quaternion 변환
            tf2::Quaternion q;
            q.setRPY(roll_rad, pitch_rad, yaw_rad);
            imu_msg.orientation.x = q.x();
            imu_msg.orientation.y = q.y();
            imu_msg.orientation.z = q.z();
            imu_msg.orientation.w = q.w();

            // orientation covariance 대각 성분 (정지 상태 실측 기반, 2000 샘플, imu_covariance_calibrator.py 사용)
            imu_msg.orientation_covariance[0] = orientation_covariance_roll_;
            imu_msg.orientation_covariance[4] = orientation_covariance_pitch_;
            imu_msg.orientation_covariance[8] = orientation_covariance_yaw_;

            // 5. $RPY 모드는 각속도/가속도 raw 값을 제공하지 않으므로 REP-145 관례에 따라 "데이터 없음" 명시
            imu_msg.angular_velocity_covariance[0] = -1.0;
            imu_msg.linear_acceleration_covariance[0] = -1.0;

            // 7. 정상 파싱 성공 시에만 publish
            imu_pub_->publish(imu_msg);
        }
        catch (...) {
            // 6. 숫자 변환 실패(std::stod 예외 등) 시 원본 로그 남기고 publish 생략
            RCLCPP_WARN(this->get_logger(), "Parsing error on IMU data stream: %s", raw_str.c_str());
        }
    }

    int serial_fd_;
    std::string frame_id_;
    double orientation_covariance_roll_;
    double orientation_covariance_pitch_;
    double orientation_covariance_yaw_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyAhrsDriverNode>());
    rclcpp::shutdown();
    return 0;
}