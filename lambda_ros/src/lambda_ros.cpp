#include <lambda_ros/lambda.h>
#include <memory>
#include <ros/ros.h>


class LambdaRos
{
public:
  LambdaRos(int argc, char* argv[])
  {
    lambda_.reset(new Lambda());

    lambda_->set("nX", 391);
    lambda_->set("nY", 391);

    lambda_->initSimulation();
    // setup the graphics
    lambda_->vis();

    float pressure = 1.0;
    ros::param::get("~pressure", pressure);
    lambda_->setPressure(120, 30, pressure);

    while (ros::ok())
    {
      lambda_->processSim();
      lambda_->processVis();
      lambda_->draw();
      ros::Duration(0.03).sleep();
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
