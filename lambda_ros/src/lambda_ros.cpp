#include <lambda_ros/lambda.h>
#include <memory>
#include <ros/ros.h>


class LambdaRos
{
public:
  LambdaRos()
  {
    // lambda_.reset(new lambda());
  }

private:
  ros::NodeHandle nh_;
  std::unique_ptr<lambda> lambda_;
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "lambda_ros");
  ros::spin();
}
