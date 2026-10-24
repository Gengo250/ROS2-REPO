// Read-only bounded edge-authority audit. Jazzy rclpy MessageInfo lacks publisher_gid.
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <thread>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("navigation_tf_authority_audit");
  std::map<std::string, std::set<std::string>> owners{{"map->odom", {}}, {"odom->base_link", {}}};
  auto subscription = node->create_subscription<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(100),
    [&](const tf2_msgs::msg::TFMessage::SharedPtr msg, const rclcpp::MessageInfo &info) {
      std::ostringstream gid;
      for (auto byte : info.get_rmw_message_info().publisher_gid.data)
        gid << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte);
      for (const auto &tf : msg->transforms) {
        auto key = tf.header.frame_id + "->" + tf.child_frame_id;
        if (owners.count(key)) owners[key].insert(gid.str());
      }
    });
  auto end = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < end) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  bool passed = true;
  std::cout << "{\"edge_publisher_gids\":{";
  bool first = true;
  for (const auto &[edge, gids] : owners) {
    if (!first) std::cout << ',';
    first = false;
    std::cout << '"' << edge << "\":[";
    bool first_gid = true;
    for (const auto &gid : gids) {
      if (!first_gid) std::cout << ',';
      first_gid = false;
      std::cout << '"' << gid << '"';
    }
    std::cout << ']';
    passed = passed && gids.size() == 1;
  }
  std::cout << "},\"passed\":" << (passed ? "true" : "false") << "}\n";
  rclcpp::shutdown();
  return passed ? 0 : 1;
}
