#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Point.h>
#include <lambda_ros/LambdaConfig.h>
#include <lambda_ros/lambda.h>
#include <memory>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>


class LambdaRos
{
public:
  LambdaRos() :
    spinner_(3)
  {
    pressure_pub_ = nh_.advertise<sensor_msgs::Image>("pressure_image", 1);
    point_sub_ = nh_.subscribe<geometry_msgs::Point>("pressure_image_mouse_left", 10,
        &LambdaRos::pointCallback, this);
    lambda_.reset(new Lambda());

    // assumine the speed of sound is 343 m/s, and that the sample rate is 44100 samples/s
    // the width of each cell is 343.0 / 44100.0 = 0.0078 meters, or 1/3"
    int wd = 5;
    ros::param::get("~width", wd);
    lambda_->set("nX", wd);
    int ht = 5;
    ros::param::get("~height", ht);
    lambda_->set("nY", ht);
    // rho doesn't change the sim at all, it is for the internal sources
    // float rho = 0.01;
    // ros::param::get("~rho", rho);
    // lambda_->set("rho", rho);

    lambda_->initSimulationPre();

    #if 0
    {
      // debug - look at initial pressures
      lambda_->processSim();
      std::cout << "no wall\n";
      lambda_->getPressure(10, 10);
      std::cout << "top wall\n";
      lambda_->getPressure(30, 0);
      lambda_->getPressure(30, 1);
      std::cout << "left wall\n";
      lambda_->getPressure(0, 30);
      lambda_->getPressure(1, 30);
      std::cout << "pos\n";
      lambda_->getPressure(20, 30);
      lambda_->getPressure(19, 30);
      lambda_->getPressure(20, 29);
      std::cout << "neg\n";
      lambda_->getPressure(350, 100);
      lambda_->getPressure(349, 100);
      std::cout << "absorb\n";
      lambda_->getPressure(50, 390);
      lambda_->getPressure(50, 389);
    }
    #endif
    ROS_INFO_STREAM("init environment");
    lambda_->initEnvironmentSetup();
    ROS_INFO_STREAM("init sim");
    lambda_->initSimulation();

    #if 1
    ROS_INFO_STREAM("setup walls");
    // env [-1.0 - 1.0] but excluding 0.0 is a wall
    // 1.0 - 1000.0 is something else- another kind of wall
    const float radius = wd * 0.4;
    for (size_t i = 0; i < 250; ++i)
    {
      const float angle = i * 0.02;
      const int x = wd * 0.5 + radius * cos(angle);
      const int y = ht * 0.5 + radius * sin(angle);
      // std::cout << x << " " << y << "\n";
      for (int ox = 0; ox < 3; ++ox)
        for (int oy = 0; oy < 3; ++oy)
          addWall(x + ox, y + oy, -1.0);
    }
    #endif
    #if 0
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
    #endif

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

    addPressure(wd / 2, ht / 2, 1.0);

    reconfigure_server_.reset(new ReconfigureServer(dr_mutex_, nh_));
    dynamic_reconfigure::Server<lambda_ros::LambdaConfig>::CallbackType cbt =
        boost::bind(&LambdaRos::reconfigureCallback, this, _1, _2);
    reconfigure_server_->setCallback(cbt);

    update_timer_ = nh_.createTimer(ros::Duration(0.02),
        &LambdaRos::update, this);

    spinner_.start();
  }

  void update(const ros::TimerEvent& e)
  {
    if (point_)
    {
      if (config_.click_mode == lambda_ros::Lambda_pressure_impulse)
        addPressure(point_->x, point_->y, config_.click_value);
      else if (config_.click_mode == lambda_ros::Lambda_wall)
      {
        for (int ox = -2; ox < 3; ++ox)
          for (int oy = -2; oy < 3; ++oy)
            addWall(point_->x + ox, point_->y + oy, config_.click_value);
      }
      point_.reset();
    }

    if (!config_.publish_rate == 0.0)
    {
      lambda_->processSim();
      publishImage();
      // Can get 203 fps and 100% one cpu core with no sleeping
      // 84 fps with this 5 ms sleep
      ros::Duration(0.005).sleep();
    }
    else
    {
      ros::Duration(0.03).sleep();
    }
  }

  void addWall(const float x, const float y, const float reflection)
  {
    lambda_->setWall(x, y, reflection);
  }

  void addPressure(const float x, const float y, const float pressure)
  {
    const float cur_pressure = lambda_->getPressure(x, y);
    ROS_INFO_STREAM("pressure point " << x << " " << y
        << " " << pressure << ", cur pressure " << cur_pressure);
    // TODO(lucasw) make a circle  with 1.0/distance from the center
    // as a modifier on point_pressure_
    for (int i = -1; i < 2; ++i)
      for (int j = -1; j < 2; ++j)
        lambda_->addPressure(x + i, y + j, pressure);
    lambda_->addPressure(x, y, pressure);

    // temp
    #if 0
    if (false)
    {
      // cv::Mat image;
      // lambda_->getPressure(image);
      for (size_t y = 0; y < ht; ++y)
      {
        for (size_t x = 0; x < ht; ++x)
        {
          const float pressure = lambda_->getPressure(x, y);
          std::cout << x << ", " << y << " : " << pressure << "\n";
        }
      }
    }
    #endif
  }

  void pointCallback(const geometry_msgs::PointConstPtr& msg)
  {
    ROS_INFO_STREAM("point " << msg->x << " " << msg->y);
    point_ = msg;
  }

  void publishImage()
  {
    cv_image_.header.stamp = ros::Time::now();
    lambda_->getPressure(cv_image_.image);
    cv_image_.encoding = "32FC1";
    pressure_pub_.publish(cv_image_.toImageMsg());
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher pressure_pub_;
  ros::Subscriber point_sub_;
  cv_bridge::CvImage cv_image_;
  std::unique_ptr<Lambda> lambda_;

  lambda_ros::LambdaConfig config_;
  boost::recursive_mutex dr_mutex_;
  typedef dynamic_reconfigure::Server<lambda_ros::LambdaConfig> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;
  void reconfigureCallback(
      lambda_ros::LambdaConfig& config,
      uint32_t level)
  {
    config_ = config;
  }

  ros::Timer update_timer_;
  ros::AsyncSpinner spinner_;
  geometry_msgs::PointConstPtr point_;
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "lambda");
  LambdaRos lambda_ros;
  ros::waitForShutdown();
}
