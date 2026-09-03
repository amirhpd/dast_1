// Standalone driver for the Kinect v1 (Xbox 360, model 1414).
//
// Deliberately independent of the rest of DAST-1: it depends only on rclcpp,
// sensor_msgs and libfreenect, and is not launched by sim_robot/run_robot. Its
// job is to prove the sensor reaches ROS 2, nothing more.
//
// To run:
// ros2 launch kinect kinect.launch.py
// ros2 topic hz /kinect/rgb/image_raw

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>

extern "C"
{
#include <libfreenect.h>
#include <libfreenect_sync.h>
}

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace
{
// FREENECT_RESOLUTION_MEDIUM, the only resolution the sync API exposes by default.
constexpr uint32_t kWidth = 640;
constexpr uint32_t kHeight = 480;

// Nominal VGA intrinsics for a Kinect v1. Good enough to look at; this is NOT a
// calibration of this particular unit. Replace with a real camera_info file
// (camera_calibration) before anything depends on metric accuracy.
constexpr double kFx = 525.0;
constexpr double kFy = 525.0;
constexpr double kCx = 319.5;
constexpr double kCy = 239.5;
}  // namespace

namespace kinect
{

class KinectNode : public rclcpp::Node
{
public:
  KinectNode()
  : Node("kinect")
  {
    frame_id_ = declare_parameter("frame_id", "kinect_rgb_optical_frame");
    device_index_ = declare_parameter("device_index", 0);
    publish_pointcloud_ = declare_parameter("publish_pointcloud", true);
    const double rate = declare_parameter("rate", 30.0);
    open_attempts_ = static_cast<size_t>(declare_parameter("open_attempts", 30));

    // Default (reliable) QoS on purpose: a reliable publisher satisfies both
    // reliable and best-effort subscribers, so RViz and rqt_image_view both see
    // these topics without anyone touching QoS settings.
    rgb_pub_ = create_publisher<sensor_msgs::msg::Image>("~/rgb/image_raw", 10);
    rgb_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("~/rgb/camera_info", 10);
    depth_pub_ = create_publisher<sensor_msgs::msg::Image>("~/depth/image_raw", 10);
    depth_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("~/depth/camera_info", 10);
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/points", 10);

    camera_info_ = makeCameraInfo();

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate),
      std::bind(&KinectNode::tick, this));

    RCLCPP_INFO(get_logger(), "Kinect node started on device %d.", device_index_);
  }

  ~KinectNode() override
  {
    // Stops the streams and closes the device; without it the Kinect is left
    // running and the next start fails to open it.
    freenect_sync_stop();
  }

private:
  sensor_msgs::msg::CameraInfo makeCameraInfo() const
  {
    sensor_msgs::msg::CameraInfo info;
    info.width = kWidth;
    info.height = kHeight;
    info.distortion_model = "plumb_bob";
    info.d.assign(5, 0.0);
    info.k = {kFx, 0.0, kCx, 0.0, kFy, kCy, 0.0, 0.0, 1.0};
    info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    info.p = {kFx, 0.0, kCx, 0.0, 0.0, kFy, kCy, 0.0, 0.0, 0.0, 1.0, 0.0};
    return info;
  }

  void tick()
  {
    void * rgb_buf = nullptr;
    void * depth_buf = nullptr;
    uint32_t rgb_stamp = 0;
    uint32_t depth_stamp = 0;

    // Both calls block until the next frame arrives, so the timer rate is an
    // upper bound -- the sensor's own 30 Hz is what actually paces this.
    if (freenect_sync_get_video(&rgb_buf, &rgb_stamp, device_index_, FREENECT_VIDEO_RGB) != 0) {
      handleFailure("no RGB frame");
      return;
    }
    // FREENECT_DEPTH_REGISTERED is depth in millimetres, already aligned to the
    // RGB image, so depth pixel (u,v) and colour pixel (u,v) are the same point
    // and the cloud below can be coloured without any extra registration step.
    if (freenect_sync_get_depth(
        &depth_buf, &depth_stamp, device_index_, FREENECT_DEPTH_REGISTERED) != 0)
    {
      handleFailure("no depth frame");
      return;
    }

    streaming_ = true;
    consecutive_failures_ = 0;

    const auto * rgb = static_cast<const uint8_t *>(rgb_buf);
    const auto * depth = static_cast<const uint16_t *>(depth_buf);
    const rclcpp::Time stamp = now();

    publishImages(rgb, depth, stamp);
    publishCameraInfo(stamp);

    if (publish_pointcloud_ && cloud_pub_->get_subscription_count() > 0) {
      publishCloud(rgb, depth, stamp);
    }
  }

  // Once frames are flowing a dropout is worth a warning and nothing more. But
  // if the device never opened at all, retrying forever just buries the real
  // cause under libfreenect's own stderr output, so give up and say why.
  void handleFailure(const char * what)
  {
    ++consecutive_failures_;

    if (streaming_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Kinect %d: %s.", device_index_, what);
      return;
    }

    if (consecutive_failures_ >= open_attempts_) {
      RCLCPP_FATAL(
        get_logger(),
        "Could not open Kinect %d after %zu attempts (%s). Usual causes: "
        "another process holds the device (close freenect-glview, check "
        "`pgrep -a freenect`), or the 12V supply is missing "
        "(`lsusb | grep 045e` must list 02b0, 02ad and 02ae).",
        device_index_, consecutive_failures_, what);
      rclcpp::shutdown();
    }
  }

  void publishImages(const uint8_t * rgb, const uint16_t * depth, const rclcpp::Time & stamp)
  {
    if (rgb_pub_->get_subscription_count() > 0) {
      sensor_msgs::msg::Image msg;
      msg.header.stamp = stamp;
      msg.header.frame_id = frame_id_;
      msg.height = kHeight;
      msg.width = kWidth;
      msg.encoding = "rgb8";
      msg.is_bigendian = 0;
      msg.step = kWidth * 3;
      msg.data.resize(static_cast<size_t>(msg.step) * kHeight);
      std::memcpy(msg.data.data(), rgb, msg.data.size());
      rgb_pub_->publish(std::move(msg));
    }

    if (depth_pub_->get_subscription_count() > 0) {
      sensor_msgs::msg::Image msg;
      msg.header.stamp = stamp;
      msg.header.frame_id = frame_id_;
      msg.height = kHeight;
      msg.width = kWidth;
      msg.encoding = "16UC1";  // millimetres, 0 where the sensor saw nothing
      msg.is_bigendian = 0;
      msg.step = kWidth * sizeof(uint16_t);
      msg.data.resize(static_cast<size_t>(msg.step) * kHeight);
      std::memcpy(msg.data.data(), depth, msg.data.size());
      depth_pub_->publish(std::move(msg));
    }
  }

  void publishCameraInfo(const rclcpp::Time & stamp)
  {
    camera_info_.header.stamp = stamp;
    camera_info_.header.frame_id = frame_id_;
    if (rgb_info_pub_->get_subscription_count() > 0) {
      rgb_info_pub_->publish(camera_info_);
    }
    // Registered depth shares the RGB camera's frame and intrinsics.
    if (depth_info_pub_->get_subscription_count() > 0) {
      depth_info_pub_->publish(camera_info_);
    }
  }

  void publishCloud(const uint8_t * rgb, const uint16_t * depth, const rclcpp::Time & stamp)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = frame_id_;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    modifier.resize(static_cast<size_t>(kWidth) * kHeight);

    // Organised cloud: one point per pixel, invalid ones left as NaN.
    cloud.height = kHeight;
    cloud.width = kWidth;
    cloud.row_step = cloud.point_step * kWidth;
    cloud.is_dense = false;

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud, "b");

    const float nan = std::numeric_limits<float>::quiet_NaN();

    for (uint32_t v = 0; v < kHeight; ++v) {
      for (uint32_t u = 0; u < kWidth; ++u,
        ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b)
      {
        const size_t i = static_cast<size_t>(v) * kWidth + u;
        const uint16_t mm = depth[i];

        if (mm == 0) {
          *iter_x = nan;
          *iter_y = nan;
          *iter_z = nan;
          *iter_r = 0;
          *iter_g = 0;
          *iter_b = 0;
          continue;
        }

        // Optical frame convention: z forward, x right, y down.
        const float z = static_cast<float>(mm) * 0.001f;
        *iter_z = z;
        *iter_x = static_cast<float>((u - kCx) * z / kFx);
        *iter_y = static_cast<float>((v - kCy) * z / kFy);
        *iter_r = rgb[i * 3 + 0];
        *iter_g = rgb[i * 3 + 1];
        *iter_b = rgb[i * 3 + 2];
      }
    }

    cloud_pub_->publish(std::move(cloud));
  }

  std::string frame_id_;
  int device_index_ {0};
  bool publish_pointcloud_ {true};
  size_t open_attempts_ {30};
  size_t consecutive_failures_ {0};
  bool streaming_ {false};

  sensor_msgs::msg::CameraInfo camera_info_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_, depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rgb_info_pub_, depth_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace kinect

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<kinect::KinectNode>());
  rclcpp::shutdown();
  return 0;
}
