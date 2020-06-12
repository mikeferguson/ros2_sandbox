#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>

class CountSubscriberIssue : public rclcpp::Node
{
public:
  CountSubscriberIssue() :
    rclcpp::Node("count_subscriber_issue")
  {
    pub_ = create_publisher<geometry_msgs::msg::Twist>("test", 1);

    timer_ = create_wall_timer(std::chrono::milliseconds(1000),
               std::bind(&CountSubscriberIssue::callback, this));
  }

private:
  void callback()
  {
    auto count = this->count_subscribers("test");
    RCLCPP_INFO(this->get_logger(), "Num of subscribers: %d", count);

    geometry_msgs::msg::Twist twist;
    twist.linear.x = 42.0;
    pub_->publish(twist);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<CountSubscriberIssue> node(new CountSubscriberIssue());
  rclcpp::spin(node);
  return 0;
}
