#include <array>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <lambda_ros/ocl.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// load the opencl program from the disk
// TBD optionally provide the old program, if it hasn't changed
// then don't rebuild it
bool loadProg(std::vector<cl::Device> &devices, cl::Context &context,
              cl::Program &program, std::string &old_text,
              const std::string cl_name = "buffer.cl");

// provide the location of the .cl file in the first argument
// then optionally provide the video number, 0 for /dev/video0 and so on,
class OclLambda {
public:
  // std::string cl_file = "buffer.cl";
  OclLambda(const std::string cl_file, const size_t width, const size_t height);

  bool update(const size_t num_kernel_loops,
      std::array<cv::Mat, 3>& pressure);
private:
  bool init(const std::string cl_file, const size_t width, const size_t height);

  cl::Event event_;
  static const int num_kernels_ = 3;
  std::array<cl::Kernel, num_kernels_> kernels_;
  std::array<cl::Buffer, num_kernels_> cl_image_;
  cl::CommandQueue queue_;
  cl::Program program_;
  cl::Context context_;
  cl::NDRange global_size_;
  const size_t origin_ = 0;
  const size_t region_;

  // local size doesn't mean much without having local memory
  // the null range causes automatic setting of local size,
  // and because the cpu has four cores it looks like it sets the
  // local size to be the global_size divided by 3 (leaving 1 core free for
  // other tasks) though other times it looks like sets the local size to the
  // full size
  cl::NDRange local_size_ = cl::NullRange;
  // cl::NDRange local_size(16, 16);
  // this is just a global offset, don't think global size has to shrink
  // by same amount
  // Null and 0,0 should be the same
  // cl::NDRange offset(0, 0);
  cl::NDRange offset_ = cl::NullRange;

  int outer_loops_ = 0;
  int inner_loops_ = 0;

  bool init_done_ = false;
};
