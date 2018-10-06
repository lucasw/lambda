#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <lambda_ros/lambda.h>
#include <memory>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>


class LambdaRos
{
public:
  LambdaRos(int argc, char* argv[]) :
    spinner_(2)
  {
    pressure_pub_ = nh_.advertise<sensor_msgs::Image>("pressure_image", 1);
    point_sub_ = nh_.subscribe<geometry_msgs::Point>("pressure_image_mouse_left", 10,
        &LambdaRos::pointCallback, this);
    lambda_.reset(new Lambda());

    // assumine the speed of sound is 343 m/s, and that the sample rate is 44100 samples/s
    // the width of each cell is 343.0 / 44100.0 = 0.0078 meters, or 1/3"
    lambda_->set("nX", 500);
    lambda_->set("nY", 500);
    // rho doesn't change the sim at all, it is for the internal sources
    // float rho = 0.01;
    // ros::param::get("~rho", rho);
    // lambda_->set("rho", rho);

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
        lambda_->setWall(350 + j, 100 + i + j, -1.0);
        lambda_->setWall(350 + i + j, 100 + j, -1.0);
        // this seems to absorb?
        lambda_->setWall(50 + i / 2 + j, 390 - i / 2 + j, 5.0);
        lambda_->setWall(50 + i / 2 + j, 390 + i / 2 + j, 5.0);
      }
    }
    ROS_INFO_STREAM("init environment");
    lambda_->initEnvironmentSetup();
    ROS_INFO_STREAM("init sim");
    lambda_->initSimulation();

    ros::param::get("~point_pressure", point_pressure_);

    spinner_.start();

    ROS_INFO_STREAM("start sim");

    {
      ros::Time t0 = ros::Time::now();
      const float num = 400.0;
      for (size_t i = 0; i < num; ++i)
      {
        // 277 with process vis, 471 without
        lambda_->processSim();
        // lambda_->processVis();
      }
      ros::Time t1 = ros::Time::now();
      ROS_INFO_STREAM("speed = " << num / (t1 - t0).toSec());
    }

    while (ros::ok())
    {
      if (point_)
      {
        ROS_INFO_STREAM("using pressure point " << point_->x << " " << point_->y
            << " " << point_pressure_);
        for (int i = -1; i < 2; ++i)
          for (int j = -1; j < 2; ++j)
            lambda_->addPressure(point_->x + i, point_->y + j, point_pressure_);
        lambda_->addPressure(point_->x, point_->y, point_pressure_);
        point_.reset();
      }

      lambda_->processSim();
      // TODO(lucasw) if this is taking a long time can make a buffering
      // scheme to do it in a separate thread
      lambda_->processVis();
      publishImage();
      // Can get 203 fps and 100% one cpu core with no sleeping
      // 84 fps with this 5 ms sleep
      ros::Duration(0.005).sleep();
    }
  }

  void pointCallback(const geometry_msgs::PointConstPtr& msg)
  {
    ROS_INFO_STREAM("new pressure point " << msg->x << " " << msg->y);
    point_ = msg;
  }

  void publishImage()
  {
    cv_image_.header.stamp = ros::Time::now();
    cv_image_.image = lambda_->graphics_.frame_;
    cv_image_.encoding = "32FC1";
    pressure_pub_.publish(cv_image_.toImageMsg());
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher pressure_pub_;
  ros::Subscriber point_sub_;
  cv_bridge::CvImage cv_image_;
  std::unique_ptr<Lambda> lambda_;

  ros::AsyncSpinner spinner_;
  float point_pressure_ = 1.0;
  geometry_msgs::PointConstPtr point_;
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "lambda");
  LambdaRos lambda_ros(argc, argv);
}
