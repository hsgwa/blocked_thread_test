#include <chrono>
#include <iostream>
#include <vector>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>

using namespace std::chrono_literals;

class TimerNode : public rclcpp::Node
{
public:
  explicit TimerNode(std::string node_name)
  : rclcpp::Node(node_name)
  {
    this->declare_parameter<int>("thread_num", 0);

    this->get_parameter("thread_num", thread_num_);
    timer_ = this->create_wall_timer(
      1s, [&]() {
        RCLCPP_INFO(this->get_logger(), "hello, main thread");
      });
  }

  int get_thread_num() const
  {
    return thread_num_;
  }

private:
  int thread_num_;
  rclcpp::TimerBase::SharedPtr timer_;
};


class SubNode : public rclcpp::Node
{
public:
  explicit SubNode(std::string node_name)
  : rclcpp::Node(node_name)
  {
    sub_ = this->create_subscription<std_msgs::msg::Empty>(
      "run_thread", 10, [&](std_msgs::msg::Empty::UniquePtr msg) {
        (void) msg;
      }
    );
  }

private:
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TimerNode>("node");
  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  std::vector<std::thread> threads = {};

  exec->add_node(node);
  exec->spin_once();

  for (auto i = 0; i < node->get_thread_num(); i++) {
    threads.emplace_back(
      [i]() {
        auto node_name = std::string("node_") + std::to_string(i);
        auto node = std::make_shared<SubNode>(node_name);
        auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        exec->add_node(node);
        exec->spin();
      });
  }

  RCLCPP_INFO(node->get_logger(), "%d threads started", node->get_thread_num());

  threads.emplace_back(
    [&exec]() {
      exec->spin();
    });

  for (auto & thread : threads) {
    thread.join();
  }
}
