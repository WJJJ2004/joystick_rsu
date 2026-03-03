#include <chrono>
#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

static geometry_msgs::msg::Point toPoint(double x, double y, double z) {
  geometry_msgs::msg::Point p;
  p.x = x; p.y = y; p.z = z;
  return p;
}

static double norm3(double x, double y, double z) {
  return std::sqrt(x*x + y*y + z*z);
}

// Build quaternion that rotates +Z axis to direction vector (dx,dy,dz).
static geometry_msgs::msg::Quaternion quatFromZToDir(double dx, double dy, double dz) {
  geometry_msgs::msg::Quaternion q;
  const double len = norm3(dx, dy, dz);
  if (len < 1e-12) {
    q.w = 1.0; q.x = q.y = q.z = 0.0;
    return q;
  }
  dx /= len; dy /= len; dz /= len;

  // z_axis x dir = (0,0,1) x (dx,dy,dz) = (-dy, dx, 0)
  const double vx = -dy;
  const double vy =  dx;
  const double vz =  0.0;
  const double s  = std::sqrt((1.0 + dz) * 2.0); // dot(z,dir)=dz
  if (s < 1e-12) {
    // dir ~ -Z: 180deg about X axis
    q.w = 0.0; q.x = 1.0; q.y = 0.0; q.z = 0.0;
    return q;
  }
  q.w = s * 0.5;
  const double invs = 1.0 / s;
  q.x = vx * invs;
  q.y = vy * invs;
  q.z = vz * invs;
  return q;
}

struct RGBA {
  float r{0.f}, g{0.f}, b{0.f}, a{1.f};
};

class RSULinkPlotter : public rclcpp::Node {
public:
  RSULinkPlotter()
  : Node("rsu_link_plotter"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_) {

    world_frame_ = this->declare_parameter<std::string>("world_frame", "base_link");
    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 60.0);
    radius_ = this->declare_parameter<double>("radius", 0.004); // meters
    timeout_sec_ = this->declare_parameter<double>("tf_timeout_sec", 0.05);

    c1_frame_ = this->declare_parameter<std::string>("c1_frame", "point_c1_1");
    c2_frame_ = this->declare_parameter<std::string>("c2_frame", "point_c2_1");
    u1_frame_ = this->declare_parameter<std::string>("u1_frame", "point_u1_1");
    u2_frame_ = this->declare_parameter<std::string>("u2_frame", "point_u2_1");

    // Target lengths (meters)
    // 요청사항: 169.5mm, 81.0mm 고정
    target_len_1_m_ = this->declare_parameter<double>("target_len_1_m", 0.1695);
    target_len_2_m_ = this->declare_parameter<double>("target_len_2_m", 0.0810);

    // Colors (RGBA)
    // Link1 = red, Link2 = blue (요청사항: 서로 다른 색)
    link1_color_.r = static_cast<float>(this->declare_parameter<double>("link1_r", 1.0));
    link1_color_.g = static_cast<float>(this->declare_parameter<double>("link1_g", 0.1));
    link1_color_.b = static_cast<float>(this->declare_parameter<double>("link1_b", 0.1));
    link1_color_.a = static_cast<float>(this->declare_parameter<double>("link1_a", 0.9));

    link2_color_.r = static_cast<float>(this->declare_parameter<double>("link2_r", 0.1));
    link2_color_.g = static_cast<float>(this->declare_parameter<double>("link2_g", 0.4));
    link2_color_.b = static_cast<float>(this->declare_parameter<double>("link2_b", 1.0));
    link2_color_.a = static_cast<float>(this->declare_parameter<double>("link2_a", 0.9));

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("rsu_links", 10);

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&RSULinkPlotter::onTimer, this)
    );

    RCLCPP_INFO(this->get_logger(),
      "RSU Link Plotter started. world_frame=%s", world_frame_.c_str());
    RCLCPP_INFO(this->get_logger(),
      "Frames: C1=%s U1=%s | C2=%s U2=%s",
      c1_frame_.c_str(), u1_frame_.c_str(), c2_frame_.c_str(), u2_frame_.c_str());
    RCLCPP_INFO(this->get_logger(),
      "Targets: L1=%.1f mm, L2=%.1f mm",
      target_len_1_m_ * 1000.0, target_len_2_m_ * 1000.0);
  }

private:
  bool lookupPoint(const std::string& target_frame, geometry_msgs::msg::Point& out) {
    try {
      const auto tf = tf_buffer_.lookupTransform(
        world_frame_, target_frame,
        tf2::TimePointZero,
        tf2::durationFromSec(timeout_sec_)
      );
      out.x = tf.transform.translation.x;
      out.y = tf.transform.translation.y;
      out.z = tf.transform.translation.z;
      return true;
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "TF lookup failed: %s -> %s : %s", world_frame_.c_str(), target_frame.c_str(), ex.what());
      return false;
    }
  }

  visualization_msgs::msg::Marker makeCylinder(
      int id,
      const geometry_msgs::msg::Point& a,
      const geometry_msgs::msg::Point& b,
      double radius,
      const std::string& ns,
      const RGBA& color) {

    visualization_msgs::msg::Marker m;
    m.header.frame_id = world_frame_;
    m.header.stamp = this->now();
    m.ns = ns;
    m.id = id;
    m.type = visualization_msgs::msg::Marker::CYLINDER;
    m.action = visualization_msgs::msg::Marker::ADD;

    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double dz = b.z - a.z;
    const double L  = norm3(dx, dy, dz);

    m.pose.position = toPoint((a.x + b.x) * 0.5, (a.y + b.y) * 0.5, (a.z + b.z) * 0.5);
    m.pose.orientation = quatFromZToDir(dx, dy, dz);

    // Marker cylinder axis is Z
    m.scale.x = radius * 2.0;
    m.scale.y = radius * 2.0;
    m.scale.z = std::max(1e-6, L);

    m.color.r = color.r;
    m.color.g = color.g;
    m.color.b = color.b;
    m.color.a = color.a;

    m.lifetime = rclcpp::Duration(0, 0);
    return m;
  }

  // Make a delete marker for given id/ns so RViz removes previously published marker
  visualization_msgs::msg::Marker makeDeleteMarker(int id, const std::string& ns) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = world_frame_;
    m.header.stamp = this->now();
    m.ns = ns;
    m.id = id;
    m.action = visualization_msgs::msg::Marker::DELETE;
    // type/pose/scale/color not needed for DELETE, but fill header so RViz accepts it
    m.type = visualization_msgs::msg::Marker::CYLINDER;
    m.lifetime = rclcpp::Duration(0, 0);
    return m;
  }

  void onTimer() {
    geometry_msgs::msg::Point c1, c2, u1, u2;
    const bool ok_c1 = lookupPoint(c1_frame_, c1);
    const bool ok_c2 = lookupPoint(c2_frame_, c2);
    const bool ok_u1 = lookupPoint(u1_frame_, u1);
    const bool ok_u2 = lookupPoint(u2_frame_, u2);

    if (!(ok_c1 && ok_c2 && ok_u1 && ok_u2)) {
      return;
    }

    // Compute lengths
    const double L1 = norm3(c1.x - u1.x, c1.y - u1.y, c1.z - u1.z);
    const double L2 = norm3(c2.x - u2.x, c2.y - u2.y, c2.z - u2.z);

    // Throttled diagnostic (1Hz)
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "L1=%.1f mm (err %.1f), L2=%.1f mm (err %.1f)",
      L1 * 1000.0, (L1 - target_len_1_m_) * 1000.0,
      L2 * 1000.0, (L2 - target_len_2_m_) * 1000.0
    );

    visualization_msgs::msg::MarkerArray arr;
    int id = 0;

    // Threshold: 2.0 mm -> 0.002 m
    const double hide_threshold_m = 0.002;

    // Link 1: C1-U1 (red)
    const double err1 = std::fabs(L1 - target_len_1_m_);
    if (err1 < hide_threshold_m) {
      arr.markers.push_back(makeCylinder(id++, c1, u1, radius_, "rsu_link_1", link1_color_));
    } else {
      // publish delete so RViz doesn't keep stale marker
      arr.markers.push_back(makeDeleteMarker(id++, "rsu_link_1"));
    }

    // Link 2: C2-U2 (blue)
    const double err2 = std::fabs(L2 - target_len_2_m_);
    if (err2 < hide_threshold_m) {
      arr.markers.push_back(makeCylinder(id++, c2, u2, radius_, "rsu_link_2", link2_color_));
    } else {
      arr.markers.push_back(makeDeleteMarker(id++, "rsu_link_2"));
    }

    marker_pub_->publish(arr);
  }

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Params
  std::string world_frame_;
  double publish_rate_hz_{60.0};
  double radius_{0.004};
  double timeout_sec_{0.05};

  std::string c1_frame_, c2_frame_, u1_frame_, u2_frame_;

  double target_len_1_m_{0.1695};
  double target_len_2_m_{0.0810};

  RGBA link1_color_;
  RGBA link2_color_;

  // ROS
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RSULinkPlotter>());
  rclcpp::shutdown();
  return 0;
}
