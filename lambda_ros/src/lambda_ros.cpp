#include <lambda_ros/lambda.h>
#include <ros/ros.h>


class LambdaRos
{
public:
  LambdaRos()
  {

  }

private:
  ros::NodeHandle nh_;
  lambda lambda_;
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "lambda_ros");
  ros::spin();
}
