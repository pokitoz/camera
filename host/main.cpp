/**
 * @file video_recorder_node.cpp
 * @brief A ROS 2 node that subscribes to an image topic and records the frames into an MP4 video file.
 */

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

/**
 * @class VideoRecorder
 * @brief A ROS 2 Node that subscribes to compressed image messages and records them into a video file using OpenCV.
 */
class VideoRecorder : public rclcpp::Node
{
public:
    /**
     * @brief Constructor: Initializes the node, subscription, and video parameters.
     */
    VideoRecorder()
        : Node("video_recorder"),
          frame_rate_(10.0),
          out_initialized_(false),
          last_time_(this->now())
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "img", 10, std::bind(&VideoRecorder::listener_callback, this, _1));

        output_file_ = "output_video.mp4";
        codec_ = cv::VideoWriter::fourcc('m', 'p', '4', 'v'); // Use 'mp4v' for .mp4 files
    }

    /**
     * @brief Destructor: Releases the video writer if it is open.
     */
    ~VideoRecorder()
    {
        if (out_.isOpened())
        {
            out_.release();
            RCLCPP_INFO(this->get_logger(), "Video saved and writer released.");
        }
    }

private:
    /**
     * @brief Callback function that receives image messages and writes them to the video file.
     *
     * @param msg The incoming sensor_msgs::msg::Image message.
     */
    void listener_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat frame;

        try
        {
            // Decode compressed image
            std::vector<uint8_t> data(msg->data.begin(), msg->data.end());
            frame = cv::imdecode(data, cv::IMREAD_COLOR);
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(this->get_logger(), "Failed to decode frame: %s", e.what());
            return;
        }

        if (frame.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Received empty frame.");
            return;
        }

        cv::Mat denoised;
        cv::fastNlMeansDenoisingColored(frame, denoised, 10, 10, 7, 21);

        cv::Mat sharpened;
        cv::Mat kernel = (cv::Mat_<float>(3, 3) << 0, -1, 0,
                          -1, 5, -1,
                          0, -1, 0);
        cv::filter2D(denoised, sharpened, -1, kernel);

        cv::Mat ycrcb;
        cv::cvtColor(sharpened, ycrcb, cv::COLOR_BGR2YCrCb);
        std::vector<cv::Mat> channels;
        cv::split(ycrcb, channels);
        cv::equalizeHist(channels[0], channels[0]); // Equalize the Y channel
        cv::merge(channels, ycrcb);
        cv::cvtColor(ycrcb, frame, cv::COLOR_YCrCb2BGR);

        // -----------------------------------------------

        // Initialize writer if needed
        if (!out_initialized_)
        {
            frame_size_ = cv::Size(frame.cols, frame.rows);
            out_.open(output_file_, codec_, frame_rate_, frame_size_);

            if (!out_.isOpened())
            {
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

    /** @brief Subscription to the image topic */
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

    /** @brief OpenCV video writer */
    cv::VideoWriter out_;

    /** @brief Desired frame rate for the video output */
    double frame_rate_;

    /** @brief Codec used for encoding (e.g., mp4v, XVID) */
    int codec_;

    /** @brief Output video file name */
    std::string output_file_;

    /** @brief Size of the video frames */
    cv::Size frame_size_;

    /** @brief Whether the video writer has been initialized */
    bool out_initialized_;

    /** @brief Timestamp of the last written frame */
    rclcpp::Time last_time_;
};

/**
 * @brief Main function: initializes the ROS 2 system and runs the VideoRecorder node.
 */
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VideoRecorder>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
