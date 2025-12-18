#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"

#include "pfoe_msg/msg/event.hpp"
#include "pfoe_msg/msg/pfoe_output.hpp"
#include "PFoE/Event.hpp"
#include "PFoE/Episodes.hpp"
#include "PFoE/ParticleFilter.hpp"

#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "rclcpp/serialization.hpp"

namespace pfoe
{

class ReplayNode : public rclcpp::Node
{
public:
  ReplayNode()
  : Node("replay_node"),
    pf_(1000, &ep_),
    replay_mode_(false),
    bag_read_(false)
  {
    // Parameters
    this->declare_parameter<int>("num_particles", 1000);
    this->declare_parameter<double>("loop_rate", 10.0);
    this->declare_parameter<std::string>("bag_file", "");

    num_particles_ = this->get_parameter("num_particles").as_int();
    loop_rate_ = this->get_parameter("loop_rate").as_double();

    // Publishers
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    pfoe_out_pub_ = this->create_publisher<
      pfoe_msg::msg::PfoeOutput>("pfoe_out", 10);

    // Subscribers
    feature_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "image_feature", 10,
      std::bind(&ReplayNode::featureCallback, this, std::placeholders::_1));

    replay_mode_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "replay_mode", 10,
      std::bind(&ReplayNode::replayModeCallback, this, std::placeholders::_1));

    // Timer for main loop
    auto timer_period = std::chrono::duration<double>(1.0 / loop_rate_);
    timer_ = this->create_wall_timer(
      timer_period,
      std::bind(&ReplayNode::timerCallback, this));

    // Initialize action
    current_action_.linear_x = 0.0;
    current_action_.angular_z = 0.0;

    RCLCPP_INFO(this->get_logger(), "Replay Node initialized");
    RCLCPP_INFO(this->get_logger(), "Number of particles: %d", num_particles_);
  }

private:
  void featureCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    current_observation_.setValues(msg->data);
  }

  void replayModeCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    bool new_mode = msg->data;

    if (new_mode && !replay_mode_) {
      // Start replay mode
      startReplay();
    } else if (!new_mode && replay_mode_) {
      // Stop replay mode
      stopReplay();
    }

    replay_mode_ = new_mode;
  }

  void startReplay()
  {
    std::string bag_file = this->get_parameter("bag_file").as_string();

    if (bag_file.empty()) {
      // Try to get from logger node parameter
      try {
        bag_file = this->get_parameter("current_bag_file").as_string();
      } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "No bag file specified");
        return;
      }
    }

    RCLCPP_INFO(this->get_logger(), "Reading episodes from: %s", bag_file.c_str());
    readEpisodes(bag_file);
    bag_read_ = true;

    // Initialize particle filter
    pf_.init();

    RCLCPP_INFO(this->get_logger(), "Replay mode started");
  }

  void stopReplay()
  {
    bag_read_ = false;
    RCLCPP_INFO(this->get_logger(), "Replay mode stopped");

    // Stop robot
    geometry_msgs::msg::Twist msg;
    msg.linear.x = 0.0;
    msg.angular.z = 0.0;
    cmd_vel_pub_->publish(msg);
  }

  void readEpisodes(const std::string & bag_path)
  {
    ep_.reset();

    // Setup rosbag2 reader
    rosbag2_cpp::Reader reader;
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_path;
    storage_options.storage_id = "sqlite3";

    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    reader.open(storage_options, converter_options);

    rclcpp::Serialization<pfoe_msg::msg::Event> serialization;

    // Get time window (discard first and last 5 seconds)
    auto metadata = reader.get_metadata();
    int64_t start_time = metadata.starting_time.time_since_epoch().count() + 5000000000LL;  // +5s
    int64_t end_time = start_time + metadata.duration.count() - 10000000000LL;  // -10s total

    int count = 0;
    while (reader.has_next()) {
      auto bag_message = reader.read_next();

      if (bag_message->topic_name != "/event") {
        continue;
      }

      int64_t timestamp = bag_message->time_stamp;

      // Skip messages outside time window
      if (timestamp < start_time) {
        continue;
      }
      if (timestamp > end_time) {
        break;
      }

      // Deserialize message
      rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
      pfoe_msg::msg::Event event_msg;

      try {
        serialization.deserialize_message(&serialized_msg, &event_msg);

        // Convert to internal Event structure
        Observation obs(event_msg.feature);
        Action act;
        act.linear_x = event_msg.linear_x;
        act.angular_z = event_msg.angular_z;

        Event e(obs, act, 0);
        e.time = rclcpp::Time(event_msg.timestamp);

        ep_.append(e);
        count++;
      } catch (const std::exception & e) {
        RCLCPP_WARN(this->get_logger(), "Failed to deserialize event: %s", e.what());
      }
    }

    RCLCPP_INFO(this->get_logger(), "Loaded %d events from bag file", count);
  }

  void timerCallback()
  {
    if (!replay_mode_) {
      return;
    }

    if (!bag_read_) {
      return;
    }

    if (ep_.data.empty()) {
      RCLCPP_WARN(this->get_logger(), "No episodes loaded");
      return;
    }

    if (current_observation_.feature_dim == 0) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "No observation available yet");
      return;
    }

    // Run particle filter
    pfoe_msg::msg::PfoeOutput out_msg;

    current_action_ = pf_.sensorUpdate(&current_observation_, &current_action_, &ep_, &out_msg);

    // Publish action
    geometry_msgs::msg::Twist cmd_vel_msg;
    cmd_vel_msg.linear.x = current_action_.linear_x;
    cmd_vel_msg.angular.z = current_action_.angular_z;
    cmd_vel_pub_->publish(cmd_vel_msg);

    // Publish PFoE output
    out_msg.linear_x = current_action_.linear_x;
    out_msg.angular_z = current_action_.angular_z;
    out_msg.feature = current_observation_.feature;
    out_msg.timestamp = this->now();
    pfoe_out_pub_->publish(out_msg);

    // Motion update
    pf_.motionUpdate(&ep_);
  }

  // Members
  Episodes ep_;
  ParticleFilter pf_;
  Observation current_observation_;
  Action current_action_;

  bool replay_mode_;
  bool bag_read_;

  int num_particles_;
  double loop_rate_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<pfoe_msg::msg::PfoeOutput>::SharedPtr pfoe_out_pub_;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr feature_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr replay_mode_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace pfoe

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pfoe::ReplayNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
