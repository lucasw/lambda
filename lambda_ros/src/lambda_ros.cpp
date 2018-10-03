#include <lambda_ros/lambda.h>
#include <memory>
#include <ros/ros.h>


class LambdaRos
{
public:
  LambdaRos(int argc, char* argv[])
  {
    lambda_.reset(new Lambda());
  }

private:
  ros::NodeHandle nh_;
  std::unique_ptr<Lambda> lambda_;
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "lambda_ros");
  LambdaRos lambda_ros(argc, argv);
  ros::spin();
}
