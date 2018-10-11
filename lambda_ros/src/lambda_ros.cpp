#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Point.h>
#include <lambda_ros/LambdaConfig.h>
#include <lambda_ros/lambda.h>
#include <memory>
#include <mutex>
#include <omp.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <spectrogram_paint_ros/Audio.h>


class LambdaRos
{
public:
  LambdaRos() :
    nh_private_("~"),
    spinner_(3)
  {
    pressure_pub_ = nh_.advertise<sensor_msgs::Image>("pressure_image", 1);
    environment_pub_ = nh_.advertise<sensor_msgs::Image>("environment_image", 1);
    vis_pub_ = nh_.advertise<sensor_msgs::Image>("vis_image", 1);
    audio_pub_ = nh_.advertise<spectrogram_paint_ros::Audio>("audio", 1);
    point_sub_ = nh_.subscribe<geometry_msgs::Point>("pressure_image_mouse_left", 10,
        &LambdaRos::pointCallback, this);
    wall_point_sub_ = nh_.subscribe<geometry_msgs::Point>("environment_image_mouse_left", 10,
        &LambdaRos::wallPointCallback, this);
    vis_point_sub_ = nh_.subscribe<geometry_msgs::Point>("vis_image_mouse_left", 10,
        &LambdaRos::visPointCallback, this);
    lambda_.reset(new Lambda());

    // assumine the speed of sound is 343 m/s, and that the sample rate is 44100 samples/s
    // the width of each cell is 343.0 / 44100.0 = 0.0078 meters, or 1/3"
    int wd = 5;
    ros::param::get("~width", wd);
    lambda_->setNX(wd);
    int ht = 5;
    ros::param::get("~height", ht);
    lambda_->setNY(ht);

    ros::param::get("~wrap", lambda_->wrap_);
    ROS_INFO_STREAM("using wrap " << lambda_->wrap_);
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
          addWall(x + ox, y + oy, -0.98);
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

    int test_cycles = 0;
    ros::param::get("~test_cycles", test_cycles);
    if (test_cycles > 0)
    {
      for (size_t j = 0; j < 4; ++j)
      {
      ROS_INFO_STREAM("start TEST " << test_cycles << " " << wd * ht);
      ros::Duration(1.0).sleep();
      ros::Time t0 = ros::Time::now();
      for (size_t i = 0; i < test_cycles; ++i)
      {
        lambda_->processSim();
      }
      ros::Time t1 = ros::Time::now();
      // this is currently around 1500
      ROS_INFO_STREAM("speed = " << test_cycles / (t1 - t0).toSec());
      ros::Duration(1.0).sleep();
      }
    }

    ROS_INFO_STREAM("start sim");
    addPressure(wd / 2, ht / 2, 0.5);
    // lambda_->processSim();
    // lambda_->processSim();
    // addPressure(wd / 2 + 6, ht / 2, 0.5);

    reconfigure_server_.reset(new ReconfigureServer(dr_mutex_, nh_private_));
    dynamic_reconfigure::Server<lambda_ros::LambdaConfig>::CallbackType cbt =
        boost::bind(&LambdaRos::reconfigureCallback, this, _1, _2);
    reconfigure_server_->setCallback(cbt);

    // TODO(lucasw) need to adjust the timer based on publish rate
    update_timer_ = nh_.createTimer(ros::Duration(0.04),
        &LambdaRos::update, this);
    audio_update_timer_ = nh_.createTimer(ros::Duration(1.0),
        &LambdaRos::audio_update, this);
    vis_update_timer_ = nh_.createTimer(ros::Duration(0.25),
        &LambdaRos::vis_update, this);

    spinner_.start();
  }

  float fr_accum_ = 0.0;
  float new_fr_accum_ = 0.0;

  void update(const ros::TimerEvent& e)
  {
    if (point_)
    {
      addPressure(point_->x, point_->y, config_.click_value);
      point_.reset();
    }

    if ((config_.publish_rate != 0.0) || (config_.step))
    {
      int num = 0;
      if (config_.publish_rate > 0.0)
      {
        const float fr = config_.update_rate / config_.publish_rate;
        new_fr_accum_ += fr;
        num = new_fr_accum_ - fr_accum_;
      }
      if (config_.step)
      {
        num = 1;
        config_.step = false;
      }
      // no gaurantee this amount can be processed in time
      for (int i = 0; i < num; ++i)
      {
        lambda_->processSim();
        {
          std::lock_guard<std::mutex> lock(audio_mutex_);
          new_samples_left_.push_back(lambda_->getPressure(210, 200));
          new_samples_right_.push_back(lambda_->getPressure(225, 200));
        }
        fr_accum_ += 1.0;
      }
      publishImage();
    }
  }

  void vis_update(const ros::TimerEvent& e)
  {
    if (wall_point_)
    {
      for (int ox = -2; ox < 3; ++ox)
        for (int oy = -2; oy < 3; ++oy)
          addWall(wall_point_->x + ox, wall_point_->y + oy, config_.click_value);
      wall_point_.reset();
    }
    publishVisImage();
  }

  void audio_update(const ros::TimerEvent& e)
  {
   // TODO(lucasw) config_.sample_rate
    // TODO(lucasw) make this a rolling buffer instead, every few thousand
    // after reaching some desired length
    // pop the front few thousand off
    audio_.sample_rate = config_.audio_rate;
    audio_.stereo = true;
    {
      std::lock_guard<std::mutex> lock(audio_mutex_);
      while (!new_samples_left_.empty())
      {
        audio_.data.push_back(new_samples_left_.front());
        new_samples_left_.pop_front();
      }
      while (!new_samples_right_.empty())
      {
        audio_.data_right.push_back(new_samples_right_.front());
        new_samples_right_.pop_front();
      }
    }

    if (audio_.data.size() > 2000)
      audio_pub_.publish(audio_);

    if (audio_.data.size() > audio_.sample_rate * 2)
    {
      const size_t diff = audio_.data.size() - audio_.sample_rate;
      std::vector<float>::const_iterator beg = audio_.data.begin() + diff;
      std::vector<float>::const_iterator end = audio_.data.end();
      audio_.data = std::vector<float>(beg, end);
    }
  }

  void addWall(const float x, const float y, const float reflection)
  {
    const bool rv = lambda_->setWall(x, y, reflection);
    if (!rv)
    {
      // ROS_ERROR_STREAM("bad wall " << x << " " << y << " " << reflection);
    }
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
    // ROS_INFO_STREAM("point " << msg->x << " " << msg->y);
    point_ = msg;
  }

  void wallPointCallback(const geometry_msgs::PointConstPtr& msg)
  {
    // ROS_INFO_STREAM("point " << msg->x << " " << msg->y);
    wall_point_ = msg;
  }

  void visPointCallback(const geometry_msgs::PointConstPtr& msg)
  {
    // ROS_INFO_STREAM("point " << msg->x << " " << msg->y);
    vis_point_ = msg;
  }


  void publishImage()
  {
    {
      cv_image_.header.stamp = ros::Time::now();
      lambda_->getPressure(cv_image_.image);
      cv_image_.encoding = "32FC1";
      pressure_pub_.publish(cv_image_.toImageMsg());
    }
  }

  void publishVisImage()
  {
    cv_bridge::CvImage cv_image;
    {
      cv_image.header.stamp = ros::Time::now();
      lambda_->getEnvironment(cv_image.image);
      cv_image.encoding = "32FC1";
      environment_pub_.publish(cv_image.toImageMsg());
    }

    if (config_.vis_mode != "")
    {
      cv_image.header.stamp = ros::Time::now();
      lambda_->getFilterImage(cv_image.image, config_.dir, config_.vis_mode, config_.ind);
      cv_image.encoding = "32FC1";
      vis_pub_.publish(cv_image.toImageMsg());
      if (vis_point_)
      {
        int y = vis_point_->y;
        if (y < 0)
          y = 0;
        if (y > cv_image.image.cols - 1)
          y = cv_image.image.cols - 1;
        int x = vis_point_->x;
        if (x < 0)
          x = 0;
        if (x > cv_image.image.rows - 1)
          x = cv_image.image.rows - 1;
        ROS_INFO_STREAM(config_.vis_mode << " " << y << " " << x << " : "
            << cv_image.image.at<float>(y, x));
        vis_point_.reset();
      }
    }
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Publisher pressure_pub_;
  ros::Publisher environment_pub_;
  ros::Publisher vis_pub_;
  spectrogram_paint_ros::Audio audio_;
  ros::Publisher audio_pub_;
  ros::Subscriber point_sub_;
  ros::Subscriber wall_point_sub_;
  ros::Subscriber vis_point_sub_;
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
    config.step = false;
  }

  std::mutex audio_mutex_;
  std::list<float> new_samples_left_;
  std::list<float> new_samples_right_;
  ros::Timer update_timer_;
  ros::Timer vis_update_timer_;
  ros::Timer audio_update_timer_;
  ros::AsyncSpinner spinner_;
  geometry_msgs::PointConstPtr point_;
  geometry_msgs::PointConstPtr wall_point_;
  geometry_msgs::PointConstPtr vis_point_;
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "lambda");
  LambdaRos lambda_ros;
  ros::waitForShutdown();
}
