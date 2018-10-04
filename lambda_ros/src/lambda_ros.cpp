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

    lambda_->initSimulationPre();
    // env [-1.0 - 1.0] but excluding 0.0 is a wall
    // 1.0 - 1000.0 is something else- another kind of wall
    for (size_t i = 0; i < 90; ++i)
    {
      // positive reflection
      lambda_->setWall(20 + i, 30, 1.0);
      lambda_->setWall(20, 30 + i, 1.0);
      // this reflects a negative wave?
      lambda_->setWall(250, 100 + i, -1.0);
      lambda_->setWall(250 + i, 100, -1.0);
      // this seems to absorb?
      lambda_->setWall(50 + i / 2, 290 - i / 2, 5.0);
      lambda_->setWall(50 + i / 2, 290 + i / 2, 5.0);
    }
    lambda_->initEnvironmentSetup();

    lambda_->initSimulation();

    // setup the graphics
    lambda_->vis();

    float pressure = 1.0;
    ros::param::get("~pressure", pressure);
    for (size_t i = 0; i < 30; ++i)
      lambda_->setPressure(120, 80 + i, pressure);

    lambda_->processSim();
    // segfault if no processSim before processVis
    lambda_->processVis();
    lambda_->draw();

    ros::Duration(3.0).sleep();
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
