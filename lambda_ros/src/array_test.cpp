#include <opencv2/core.hpp>
#include <ros/ros.h>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "array_test");
  ros::NodeHandle nh_;

  const size_t count = 500;
  const size_t wd = 800;
  const size_t ht = 800;
  {
    ros::Time t0 = ros::Time::now();
    float sum = 0.0;
    float* a = new float[wd*ht];
    for (size_t i = 0; i < count; ++i)
    {
      for (size_t y = 0; y < ht; ++y)
      {
        for (size_t x = 0; x < wd; ++x)
        {
          a[y * wd + x] += 0.1 * x + 0.01 * y;
          sum += a[y * wd + x];
        }
      }
    }
    delete a;
    ros::Time t1 = ros::Time::now();
    const float diff = (t1 - t0).toSec();

    ROS_INFO_STREAM(count / diff << ", " << sum);
  }

  {
    ros::Time t0 = ros::Time::now();

    float sum = 0.0;
    // this vector is a lot slower 200 vs. 277 above in normal mode,
    // release build all are equivalent
    std::vector<float> a(wd * ht);
    // std::array<float, wd * ht> a;
    for (size_t i = 0; i < count; ++i)
    {
      for (size_t y = 0; y < ht; ++y)
      {
        for (size_t x = 0; x < wd; ++x)
        {
          a[y * wd + x] += 0.1 * x + 0.01 * y;
          sum += a[y * wd + x];
        }
      }
    }
    ros::Time t1 = ros::Time::now();
    const float diff = (t1 - t0).toSec();

    ROS_INFO_STREAM(count / diff << ", " << sum);
  }

  {
    ros::Time t0 = ros::Time::now();
    float sum = 0.0;
    // just as fast as above
    cv::Mat a(cv::Size(wd, ht), CV_32FC1);
    for (size_t i = 0; i < count; ++i)
    {
      float* p = a.ptr<float>(0);
      for (size_t y = 0; y < ht; ++y)
      {
        // float* p = a.ptr<float>(y);
        for (size_t x = 0; x < wd; ++x)
        {
          p[y * wd + x] += 0.1 * x + 0.01 * y;
          sum += p[y * wd + x];
          // p[x] += 0.1 * x + 0.01 * y;
          // sum += p[x];
          // a.at<float>(y, x) += 0.1 * x + 0.01 * y;
          // sum += a.at<float>(y, x);
        }
      }
    }
    ros::Time t1 = ros::Time::now();
    const float diff = (t1 - t0).toSec();

    ROS_INFO_STREAM(count / diff << ", " << sum);
  }

}
