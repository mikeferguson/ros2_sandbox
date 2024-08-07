#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

class SpinIssue : public rclcpp::Node
{
public:
  SpinIssue() :
    rclcpp::Node("spin_issue")
  {
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "head_camera/depth_registered/points",
      rclcpp::QoS(1).best_effort(),
      std::bind(&SpinIssue::callback, this, std::placeholders::_1));
  }

private:
  void callback(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
  {
    std::cout << "got cloud" << std::endl;
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<SpinIssue> node(new SpinIssue());
  rclcpp::spin(node);
  return 0;
}
