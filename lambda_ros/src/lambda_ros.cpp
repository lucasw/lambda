#include <lambda_ros/lambda.h>
#include <memory>
#include <ros/ros.h>


class LambdaRos
{
public:
  LambdaRos(int argc, char* argv[])
  {
    lambda_.reset(new Lambda());

    lambda_->set("nX", 1280);
    lambda_->set("nY", 720);

    lambda_->initSimulationPre();
    ROS_INFO_STREAM("setup walls");
    // env [-1.0 - 1.0] but excluding 0.0 is a wall
    // 1.0 - 1000.0 is something else- another kind of wall
    for (size_t i = 0; i < 90; ++i)
    {
      for (size_t j = 0; j < 4; ++j)
      {
        // positive reflection
        lambda_->setWall(20 + i + j, 30 + j, 1.0);
        lambda_->setWall(20 + j, 30 + i + j, 1.0);
        // this reflects a negative wave?
        lambda_->setWall(650 + j, 100 + i + j, -1.0);
        lambda_->setWall(650 + i + j, 100 + j, -1.0);
        // this seems to absorb?
        lambda_->setWall(50 + i / 2 + j, 590 - i / 2 + j, 5.0);
        lambda_->setWall(50 + i / 2 + j, 590 + i / 2 + j, 5.0);
      }
    }
    ROS_INFO_STREAM("init environment");
    lambda_->initEnvironmentSetup();
    ROS_INFO_STREAM("init sim");
    lambda_->initSimulation();

    ROS_INFO_STREAM("setup the graphics");
    lambda_->vis();

    ROS_INFO_STREAM("setup the initial pressure");
    float pressure = 1.0;
    ros::param::get("~pressure", pressure);
    for (size_t i = 0; i < 30; ++i) {
      for (size_t j = 0; j < 3; ++j) {
        lambda_->setPressure(120 + j, 80 + i, pressure);
      }
    }

    ROS_INFO_STREAM("first frame");
    lambda_->processSim();
    // segfault if no processSim before processVis
    lambda_->processVis();
    lambda_->draw();
    ROS_INFO_STREAM("wait a few seconds");

    ros::Duration(9.0).sleep();
    ROS_INFO_STREAM("start sim");
    while (ros::ok())
    {
      lambda_->processSim();
      lambda_->processVis();
      lambda_->draw();
      ros::Duration(0.01).sleep();
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
