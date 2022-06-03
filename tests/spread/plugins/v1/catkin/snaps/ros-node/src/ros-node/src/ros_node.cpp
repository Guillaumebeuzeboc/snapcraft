#include <iostream>
#include <functional>

#include <ros/ros.h>

class Finally
{
public:
  Finally(std::function<void()> cb) : cb_(std::move(cb))
  {
  }
  ~Finally()
  {
    cb_();
  }

private:
  std::function<void()> cb_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "talker");

  ros::NodeHandle nodeHandle;

  ros::Rate loopRate(10);

  const auto good_bye = Finally([] { std::cout << "Good bye!" std::endl; });
  std::cout << "Hello" << std::endl;
  ros::spin();

  return 0;
}
