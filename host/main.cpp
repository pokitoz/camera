#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

class VideoRecorder : public rclcpp::Node
{
public:
    VideoRecorder()
    : Node("video_recorder"),
      frame_rate_(10.0),
      out_initialized_(false),
      last_time_(this->now())
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "img", 10, std::bind(&VideoRecorder::listener_callback, this, _1));

        output_file_ = "output_video.mp4";
        codec_ = cv::VideoWriter::fourcc('m', 'p', '4', 'v'); // mp4v or XVID
    }

    ~VideoRecorder()
    {
        if (out_.isOpened()) {
            out_.release();
            RCLCPP_INFO(this->get_logger(), "Video saved and writer released.");
        }
    }

private:
    void listener_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat frame;

        try {
            // Try to decode compressed image
            std::vector<uint8_t> data(msg->data.begin(), msg->data.end());
            frame = cv::imdecode(data, cv::IMREAD_COLOR);
        } catch (const std::exception &e) {
            RCLCPP_WARN(this->get_logger(), "Failed to decode frame: %s", e.what());
            return;
        }

        if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty frame.");
            return;
        }

        // Initialize writer if needed
        if (!out_initialized_) {
            frame_size_ = cv::Size(frame.cols, frame.rows);
            out_.open(output_file_, codec_, frame_rate_, frame_size_);

            if (!out_.isOpened()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open video writer.");
                return;
            }

            out_initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "VideoWriter started: %s", output_file_.c_str());
        }

        out_.write(frame);

        auto now = this->now();
        double interval = (now - last_time_).seconds();
        RCLCPP_INFO(this->get_logger(), "Wrote frame at %.2fs interval", interval);
        last_time_ = now;
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    cv::VideoWriter out_;
    double frame_rate_;
    int codec_;
    std::string output_file_;
    cv::Size frame_size_;
    bool out_initialized_;
    rclcpp::Time last_time_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VideoRecorder>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
