#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "bumper_interfaces/msg/bumper.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses a fancy C++11 lambda
 * function to shorten the callback syntax, at the expense of making the
 * code somewhat more difficult to understand at first glance. */

class Bumper2Pc : public rclcpp::Node
{
public:
  Bumper2Pc()
  : Node("bumper2pc")
  {
    this->declare_parameter<double>("pointcloud_radius", 0.25);
    this->declare_parameter<double>("pointcloud_height", 0.04);
    this->declare_parameter<double>("side_point_angle", 0.34906585);
    this->declare_parameter<std::string>("base_link_frame", "base_link");

    // Prepare constant parts of the pointcloud message to be  published
    pointcloud_.width  = 3;
    pointcloud_.height = 1;
    pointcloud_.fields.resize(3);

    // Set x/y/z as the only fields
    pointcloud_.fields[0].name = "x";
    pointcloud_.fields[1].name = "y";
    pointcloud_.fields[2].name = "z";

    int offset = 0;
    // All offsets are *4, as all field data types are float32
    for (size_t d = 0; d < pointcloud_.fields.size(); ++d, offset += 4)
    {
      pointcloud_.fields[d].count    = 1;
      pointcloud_.fields[d].offset   = offset;
      pointcloud_.fields[d].datatype = sensor_msgs::msg::PointField::FLOAT32;
    }

    pointcloud_.point_step = offset;
    pointcloud_.row_step   = pointcloud_.point_step * pointcloud_.width;

    pointcloud_.data.resize(3 * pointcloud_.point_step);
    pointcloud_.is_bigendian = false;
    pointcloud_.is_dense     = true;

    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("bumper_point_cloud", 10);

    subscription_ = this->create_subscription<bumper_interfaces::msg::Bumper>(
      "bumper", 10, std::bind(&Bumper2Pc::bumper_topic_callback, this, _1));

//    timer_ = this->create_wall_timer(500ms, std::bind(&Bumper2Pc::respond, this));
  }

  void bumper_topic_callback(const bumper_interfaces::msg::Bumper::SharedPtr msg) 
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->left);

    float pc_radius_, pc_height, angle;
    this->get_parameter("pointcloud_radius", pc_radius_);
    this->get_parameter("pointcloud_height", pc_height);
    this->get_parameter("side_point_angle", angle);
    this->get_parameter("base_link_frame", base_link_frame_);

    pointcloud_.header.frame_id = base_link_frame_;
    // y: always 0 for central bumper
    memcpy(&pointcloud_.data[1 * pointcloud_.point_step + pointcloud_.fields[1].offset], &ZERO, sizeof(float));

    // z: constant elevation from base frame
    memcpy(&pointcloud_.data[0 * pointcloud_.point_step + pointcloud_.fields[2].offset], &pc_height, sizeof(float));
    memcpy(&pointcloud_.data[1 * pointcloud_.point_step + pointcloud_.fields[2].offset], &pc_height, sizeof(float));
    memcpy(&pointcloud_.data[2 * pointcloud_.point_step + pointcloud_.fields[2].offset], &pc_height, sizeof(float));


    /* ON EVENT START*/


    float p_side_x_ = + pc_radius_*sin(angle); // angle degrees from vertical
    float p_side_y_ = + pc_radius_*cos(angle); // angle degrees from vertical
    float n_side_y_ = - pc_radius_*cos(angle); // angle degrees from vertical

    if (msg->left) {
      memcpy(&pointcloud_.data[0 * pointcloud_.point_step + pointcloud_.fields[0].offset], &p_side_x_, sizeof(float));
      memcpy(&pointcloud_.data[0 * pointcloud_.point_step + pointcloud_.fields[1].offset], &p_side_y_, sizeof(float));
    } else {
      memcpy(&pointcloud_.data[0 * pointcloud_.point_step + pointcloud_.fields[0].offset], &P_INF_X, sizeof(float));
      memcpy(&pointcloud_.data[0 * pointcloud_.point_step + pointcloud_.fields[1].offset], &P_INF_Y, sizeof(float));
    }

    if (msg->center) {
      memcpy(&pointcloud_.data[1 * pointcloud_.point_step + pointcloud_.fields[0].offset], &pc_radius_, sizeof(float));
    } else {
      memcpy(&pointcloud_.data[1 * pointcloud_.point_step + pointcloud_.fields[0].offset], &P_INF_X, sizeof(float));
    }

    if (msg->right) {
      memcpy(&pointcloud_.data[2 * pointcloud_.point_step + pointcloud_.fields[0].offset], &p_side_x_, sizeof(float));
      memcpy(&pointcloud_.data[2 * pointcloud_.point_step + pointcloud_.fields[1].offset], &n_side_y_, sizeof(float));
    } else {
      memcpy(&pointcloud_.data[2 * pointcloud_.point_step + pointcloud_.fields[0].offset], &P_INF_X, sizeof(float));
      memcpy(&pointcloud_.data[2 * pointcloud_.point_step + pointcloud_.fields[1].offset], &N_INF_Y, sizeof(float));
    }

    pointcloud_.header.stamp = now();
    this->publisher_->publish(pointcloud_);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<bumper_interfaces::msg::Bumper>::SharedPtr subscription_;

  std::string base_link_frame_;

  sensor_msgs::msg::PointCloud2 pointcloud_;

  const float P_INF_X = +100*sin(0.34906585);  // somewhere out of reach from the robot (positive x)
  const float P_INF_Y = +100*cos(0.34906585);  // somewhere out of reach from the robot (positive y)
  const float N_INF_Y = -100*cos(0.34906585);  // somewhere out of reach from the robot (negative y)
  const float ZERO = 0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Bumper2Pc>());
  rclcpp::shutdown();
  return 0;
}