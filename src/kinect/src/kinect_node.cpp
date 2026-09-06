// Standalone driver for the Kinect v1 (Xbox 360, model 1414).
//
// Deliberately independent of the rest of DAST-1: it depends only on rclcpp,
// sensor_msgs and libfreenect, and is not launched by sim_robot/run_robot. Its
// job is to prove the sensor reaches ROS 2, nothing more.
//
// To run:
// ros2 launch kinect kinect.launch.py
// ros2 topic hz /kinect/rgb/image_raw

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <thread>
#include <vector>

extern "C"
{
#include <libfreenect.h>
#include <libfreenect_sync.h>
}

#include "ament_index_cpp/get_package_share_path.hpp"
#include "camera_calibration_parsers/parse.hpp"
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

// Nominal VGA intrinsics for a Kinect v1, used only as a fallback when
// camera_info_url names no readable calibration. The measured intrinsics for
// this unit live in config/kinect_rgb.yaml.
constexpr double kFx = 525.0;
constexpr double kFy = 525.0;
constexpr double kCx = 319.5;
constexpr double kCy = 239.5;

// The tilt motor's usable travel; libfreenect refuses anything beyond this.
constexpr int kTiltMinDegrees = -30;
constexpr int kTiltMaxDegrees = 30;

// How long to wait for the motor to stop before reading back where it landed.
constexpr auto kTiltSettleTimeout = std::chrono::seconds(3);

// Allowed gap between the commanded angle and what the accelerometer reads.
// Wider than the motor's ~1 degree repeatability, narrow enough to catch a stall.
constexpr double kTiltToleranceDegrees = 3.0;

// Fixed-point iterations used to invert the plumb_bob distortion model. It
// converges in a handful, and this runs once per pixel at startup rather than
// once per point per frame.
constexpr int kUndistortIterations = 20;
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
    camera_info_url_ = declare_parameter(
      "camera_info_url", "package://kinect/config/kinect_rgb.yaml");
    // The point cloud is metric, but the DAST-1 model is not: its meshes are
    // millimetres scaled by 0.01, so one unit in that URDF is a decimetre. Set
    // this to 10.0 when publishing into that world; leave it at 1.0 for true
    // metres. Only the cloud is scaled -- the depth image stays SI millimetres.
    point_scale_ = declare_parameter("point_scale", 1.0);
    set_tilt_ = declare_parameter("set_tilt", true);
    tilt_degrees_ = static_cast<int>(declare_parameter("tilt_degrees", -16));

    // Default (reliable) QoS on purpose: a reliable publisher satisfies both
    // reliable and best-effort subscribers, so RViz and rqt_image_view both see
    // these topics without anyone touching QoS settings.
    rgb_pub_ = create_publisher<sensor_msgs::msg::Image>("~/rgb/image_raw", 10);
    rgb_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("~/rgb/camera_info", 10);
    depth_pub_ = create_publisher<sensor_msgs::msg::Image>("~/depth/image_raw", 10);
    depth_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("~/depth/camera_info", 10);
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/points", 10);

    loadCameraInfo();
    buildUnprojectionTable();
    applyTilt();

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
  sensor_msgs::msg::CameraInfo nominalCameraInfo() const
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

  // Resolves the package:// and file:// forms camera_info_url can take, so a
  // recalibration is a file edit and a restart, never a rebuild.
  std::string resolveUrl(const std::string & url) const
  {
    static constexpr char kPackagePrefix[] = "package://";
    static constexpr char kFilePrefix[] = "file://";

    if (url.rfind(kFilePrefix, 0) == 0) {
      return url.substr(std::strlen(kFilePrefix));
    }
    if (url.rfind(kPackagePrefix, 0) != 0) {
      return url;  // already a plain path
    }

    const std::string rest = url.substr(std::strlen(kPackagePrefix));
    const size_t slash = rest.find('/');
    if (slash == std::string::npos) {
      return {};
    }

    try {
      return (ament_index_cpp::get_package_share_path(rest.substr(0, slash)) /
             rest.substr(slash + 1)).string();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Cannot resolve '%s': %s", url.c_str(), e.what());
      return {};
    }
  }

  void loadCameraInfo()
  {
    camera_info_ = nominalCameraInfo();

    const std::string path = resolveUrl(camera_info_url_);
    std::string camera_name;
    sensor_msgs::msg::CameraInfo loaded;

    if (path.empty() ||
      !camera_calibration_parsers::readCalibration(path, camera_name, loaded))
    {
      RCLCPP_WARN(
        get_logger(),
        "No calibration read from '%s'; falling back to nominal Kinect v1 "
        "intrinsics. Cloud geometry will be approximate.",
        camera_info_url_.c_str());
      return;
    }

    // A calibration taken at another resolution would misplace every point
    // silently, and this driver only ever streams VGA.
    if (loaded.width != kWidth || loaded.height != kHeight) {
      RCLCPP_ERROR(
        get_logger(),
        "Calibration '%s' is %ux%u but the Kinect streams %ux%u; ignoring it.",
        path.c_str(), loaded.width, loaded.height, kWidth, kHeight);
      return;
    }

    camera_info_ = loaded;
    RCLCPP_INFO(
      get_logger(),
      "Loaded calibration '%s' from %s (fx=%.2f fy=%.2f cx=%.2f cy=%.2f).",
      camera_name.c_str(), path.c_str(),
      camera_info_.k[0], camera_info_.k[4], camera_info_.k[2], camera_info_.k[5]);
  }

  // Precomputes, per pixel, the undistorted ray direction at unit depth.
  // plumb_bob has no closed-form inverse, so this iterates -- but 640x480 times
  // at startup, not 307200 times per frame. libfreenect registers depth into the
  // RGB image using the device's own factory model, so the RGB intrinsics are
  // the right ones to unproject it with.
  void buildUnprojectionTable()
  {
    const double fx = camera_info_.k[0];
    const double fy = camera_info_.k[4];
    const double cx = camera_info_.k[2];
    const double cy = camera_info_.k[5];

    const auto coeff = [this](size_t i) {
        return i < camera_info_.d.size() ? camera_info_.d[i] : 0.0;
      };
    const double k1 = coeff(0);
    const double k2 = coeff(1);
    const double p1 = coeff(2);
    const double p2 = coeff(3);
    const double k3 = coeff(4);

    unproject_x_.resize(static_cast<size_t>(kWidth) * kHeight);
    unproject_y_.resize(unproject_x_.size());

    for (uint32_t v = 0; v < kHeight; ++v) {
      for (uint32_t u = 0; u < kWidth; ++u) {
        const double xd = (u - cx) / fx;
        const double yd = (v - cy) / fy;

        double x = xd;
        double y = yd;
        for (int n = 0; n < kUndistortIterations; ++n) {
          const double r2 = x * x + y * y;
          const double radial = 1.0 / (1.0 + ((k3 * r2 + k2) * r2 + k1) * r2);
          const double dx = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
          const double dy = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
          x = (xd - dx) * radial;
          y = (yd - dy) * radial;
        }

        const size_t i = static_cast<size_t>(v) * kWidth + u;
        unproject_x_[i] = static_cast<float>(x);
        unproject_y_[i] = static_cast<float>(y);
      }
    }
  }

  // The motorised base holds whatever angle it was last given and forgets it
  // when the 12V drops, so command it on every start. Left to itself the tilt
  // silently stops matching the extrinsic calibration, with nothing to show for
  // it but a point cloud that no longer lines up with the robot.
  void applyTilt()
  {
    if (!set_tilt_) {
      RCLCPP_INFO(get_logger(), "set_tilt is false; leaving the tilt motor where it is.");
      return;
    }

    const int angle = std::clamp(tilt_degrees_, kTiltMinDegrees, kTiltMaxDegrees);
    if (angle != tilt_degrees_) {
      RCLCPP_WARN(
        get_logger(), "tilt_degrees %d is outside [%d, %d]; using %d instead.",
        tilt_degrees_, kTiltMinDegrees, kTiltMaxDegrees, angle);
    }

    if (freenect_sync_set_tilt_degs(angle, device_index_) != 0) {
      RCLCPP_WARN(
        get_logger(),
        "Could not command the tilt motor on device %d; leaving it where it is.",
        device_index_);
      return;
    }

    // The motor takes a second or two. Poll until it reports it has stopped, so
    // the angle logged below is where the camera settled rather than somewhere
    // it was passing through.
    freenect_raw_tilt_state * state = nullptr;
    const auto deadline = std::chrono::steady_clock::now() + kTiltSettleTimeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (freenect_sync_get_tilt_state(&state, device_index_) != 0) {
        RCLCPP_WARN(
          get_logger(), "Tilt commanded to %d deg, but the tilt state is unreadable.", angle);
        return;
      }
      if (state->tilt_status != TILT_STATUS_MOVING) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (state == nullptr) {
      return;
    }

    // freenect_get_tilt_degs() derives the angle from the accelerometer, so it
    // reports where the camera physically is, not merely what the motor was told.
    const double measured = freenect_get_tilt_degs(state);

    if (state->tilt_status == TILT_STATUS_LIMIT) {
      RCLCPP_WARN(
        get_logger(),
        "Tilt hit its mechanical limit at %.1f deg while aiming for %d deg.", measured, angle);
    } else if (std::fabs(measured - angle) > kTiltToleranceDegrees) {
      RCLCPP_WARN(
        get_logger(),
        "Tilt commanded to %d deg but the accelerometer reads %.1f deg; the motor may have "
        "stalled or is still settling. Extrinsics assume the commanded angle.",
        angle, measured);
    } else {
      RCLCPP_INFO(
        get_logger(), "Tilt set to %d deg (accelerometer reads %.1f deg).", angle, measured);
    }
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

        // Optical frame convention: z forward, x right, y down. The table
        // already holds the undistorted ray for this pixel, so the per-point
        // cost here is one multiply.
        const float z = static_cast<float>(mm) * 0.001f * static_cast<float>(point_scale_);
        *iter_z = z;
        *iter_x = unproject_x_[i] * z;
        *iter_y = unproject_y_[i] * z;
        *iter_r = rgb[i * 3 + 0];
        *iter_g = rgb[i * 3 + 1];
        *iter_b = rgb[i * 3 + 2];
      }
    }

    cloud_pub_->publish(std::move(cloud));
  }

  std::string frame_id_;
  std::string camera_info_url_;
  int device_index_ {0};
  bool publish_pointcloud_ {true};
  double point_scale_ {1.0};
  bool set_tilt_ {true};
  int tilt_degrees_ {-16};
  size_t open_attempts_ {30};
  size_t consecutive_failures_ {0};
  bool streaming_ {false};

  sensor_msgs::msg::CameraInfo camera_info_;
  std::vector<float> unproject_x_;
  std::vector<float> unproject_y_;

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
