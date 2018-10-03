#include <lambda_ros/lambda.h>
#include <memory>
#include <ros/ros.h>


class LambdaRos
{
public:
  LambdaRos(int argc, char* argv[])
  {
    lambda_.reset(new Lambda());

    lambda_->set("nX", 191);
    lambda_->set("nY", 191);

    lambda_->initSimulation();
    // setup the graphics
    lambda_->vis();

    while (ros::ok())
    {
      lambda_->processSim();
      lambda_->processVis();
      lambda_->draw();
      ros::Duration(0.1).sleep();
    }
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
