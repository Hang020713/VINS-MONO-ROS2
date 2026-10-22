#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <memory>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace
{
constexpr int64_t kNanosecondsPerSecond = 1000000000LL;

int64_t toNanoseconds(const timespec &value)
{
    return static_cast<int64_t>(value.tv_sec) * kNanosecondsPerSecond + value.tv_nsec;
}

int64_t toNanoseconds(const builtin_interfaces::msg::Time &value)
{
    return static_cast<int64_t>(value.sec) * kNanosecondsPerSecond + value.nanosec;
}
}  // namespace

class ImuClockBridge final : public rclcpp::Node
{
public:
    ImuClockBridge()
    : Node("imu_clock_bridge")
    {
        const auto input_topic = declare_parameter<std::string>("input_topic", "/imu/data");
        const auto output_topic = declare_parameter<std::string>("output_topic", "/imu/data_vio");
        input_clock_ = declare_parameter<std::string>("input_clock", "system");
        max_input_time_error_sec_ = declare_parameter<double>("max_input_time_error_sec", 1.0);

        if (input_clock_ != "system" && input_clock_ != "boottime")
        {
            RCLCPP_FATAL(get_logger(), "input_clock must be 'system' or 'boottime'.");
            rclcpp::shutdown();
            return;
        }
        if (max_input_time_error_sec_ <= 0.0)
        {
            RCLCPP_FATAL(get_logger(), "max_input_time_error_sec must be positive.");
            rclcpp::shutdown();
            return;
        }

        const auto qos = rclcpp::QoS(rclcpp::KeepLast(50)).best_effort();
        publisher_ = create_publisher<sensor_msgs::msg::Imu>(output_topic, qos);
        subscription_ = create_subscription<sensor_msgs::msg::Imu>(
            input_topic, qos,
            [this](sensor_msgs::msg::Imu::ConstSharedPtr message) { onImu(message); });

        RCLCPP_INFO(
            get_logger(), "Bridging IMU stamps from %s clock on %s to CLOCK_BOOTTIME on %s",
            input_clock_.c_str(), input_topic.c_str(), output_topic.c_str());
    }

private:
    void onImu(const sensor_msgs::msg::Imu::ConstSharedPtr &message)
    {
        int64_t output_stamp_ns = toNanoseconds(message->header.stamp);

        if (input_clock_ == "system")
        {
            timespec realtime{};
            timespec boottime{};
            if (clock_gettime(CLOCK_REALTIME, &realtime) != 0 ||
                clock_gettime(CLOCK_BOOTTIME, &boottime) != 0)
            {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 5000,
                    "Could not read CLOCK_REALTIME/CLOCK_BOOTTIME: %s", std::strerror(errno));
                return;
            }

            const int64_t realtime_ns = toNanoseconds(realtime);
            const int64_t system_to_boottime_ns = realtime_ns - toNanoseconds(boottime);
            const int64_t max_error_ns = static_cast<int64_t>(max_input_time_error_sec_ * kNanosecondsPerSecond);
            if (std::llabs(output_stamp_ns - realtime_ns) > max_error_ns)
            {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 5000,
                    "Dropping IMU sample: its stamp is not close to the Pi system clock. Check PX4 uXRCE timestamp synchronization and the converter output.");
                return;
            }

            output_stamp_ns -= system_to_boottime_ns;
        }

        if (output_stamp_ns < 0)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "Dropping IMU sample because clock conversion produced a negative timestamp.");
            return;
        }

        auto output = std::make_shared<sensor_msgs::msg::Imu>(*message);
        output->header.stamp.sec = static_cast<int32_t>(output_stamp_ns / kNanosecondsPerSecond);
        output->header.stamp.nanosec = static_cast<uint32_t>(output_stamp_ns % kNanosecondsPerSecond);
        publisher_->publish(*output);
    }

    std::string input_clock_;
    double max_input_time_error_sec_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuClockBridge>());
    rclcpp::shutdown();
    return 0;
}
