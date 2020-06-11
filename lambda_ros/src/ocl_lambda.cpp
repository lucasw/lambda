/**
 * Copyright 2018-2020 Lucas Walter
 */
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <lambda_ros/ocl_lambda.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <utility>
#include <vector>

// load the opencl program from the disk
// TBD optionally provide the old program, if it hasn't changed
// then don't rebuild it
bool loadProg(std::vector<cl::Device> &devices, cl::Context &context,
              cl::Program &program, std::string &old_text,
              const std::string cl_name) {
  // look at using inotfiy to see changes to file
  // http://stackoverflow.com/questions/4664975/monitoring-file-using-inotify
  // http://en.highscore.de/cpp/boost/asio.html
  // http://boost.2283326.n4.nabble.com/ASIO-file-monitoring-help-td4645105.html
  // http://stackoverflow.com/questions/12564039/alternatives-to-inotify-to-detect-when-a-new-file-is-created-under-a-folder/24970801#24970801
  try {
    std::ifstream cl_file;
    cl_file.open(cl_name.c_str());  //, std::ios::in);
    if (!cl_file.is_open()) {
      std::cerr << "Could not open " << cl_name.c_str() << "\n";
      return false;
    }
    std::string big_line;
    std::string line;
    while (std::getline(cl_file, line)) {
      big_line += line + "\n";
    }
    // std::cout << big_line << std::endl;
    if (old_text.compare(big_line) == 0) {
      // don't bother trying to load program if it hasn't changed
      return 1;
    }
    old_text = big_line;

    cl::Program::Sources source(
        1, std::make_pair(big_line.c_str(), strlen(big_line.c_str())));

    program = cl::Program(context, source);
    cl_int rv = program.build(devices);

    return rv;
  } catch (cl::Error err) {
    std::cerr << "ERROR: " << err.what() << "(" << err.err() << ")" << "\n";

    std::cout <<
        "build status: " << program.getBuildInfo<CL_PROGRAM_BUILD_STATUS>(
            devices[0]) << "\n";
    std::cout <<"build options:\t"
                    << program.getBuildInfo<CL_PROGRAM_BUILD_OPTIONS>(
                           devices[0]) << "\n";
    std::cout <<"build log:\t "
                    << program.getBuildInfo<CL_PROGRAM_BUILD_LOG>(devices[0]) << "\n";

    return err.err();
  }
}

// provide the location of the .cl file in the first argument
// then optionally provide the video number, 0 for /dev/video0 and so on,
  // std::string cl_file = "buffer.cl";
OclLambda::OclLambda(const std::string cl_file, const size_t width, const size_t height) :
  region_(width * height * sizeof(float)) {
  std::cout << "using " << cl_file << std::endl;
  init(cl_file, width, height);
}

bool OclLambda::init(const std::string cl_file, const size_t width, const size_t height) {
  cl_int err = CL_SUCCESS;
  try {
    std::vector<cl::Platform> platforms;
    cl::Platform::get(&platforms);
    if (platforms.size() == 0) {
      std::cout <<"Platform size 0" << "\n";
      return false;
    }

    for (cl::Platform &p : platforms) {
      std::cout <<"Platform name: "
                      << p.getInfo<CL_PLATFORM_NAME>() << "\n"
                      << "Vendor: " << p.getInfo<CL_PLATFORM_VENDOR>() << "\n"
                      << "Version: " << p.getInfo<CL_PLATFORM_VERSION>() << "\n"
                      << "Profile: " << p.getInfo<CL_PLATFORM_PROFILE>() << "\n"
                      << "Extenstions: "
                      << p.getInfo<CL_PLATFORM_EXTENSIONS>() << "\n";
    }

    cl_context_properties properties[] = {
        CL_CONTEXT_PLATFORM, (cl_context_properties)(platforms[0])(),
        0  // the callback
    };
    context_ = cl::Context(CL_DEVICE_TYPE_GPU, properties);
    std::vector<cl::Device> devices = context_.getInfo<CL_CONTEXT_DEVICES>();

    for (cl::Device &d : devices) {
      std::cout <<
          "Device name: "
          << d.getInfo<CL_DEVICE_NAME>() << ", "
          << "extensions: " << d.getInfo<CL_DEVICE_EXTENSIONS>() << ", "
          << "global mem size: "
          << d.getInfo<CL_DEVICE_GLOBAL_MEM_SIZE>() * 9.53674e-7 << "mb"
          << "\n"
          << "local mem size: "
          << d.getInfo<CL_DEVICE_LOCAL_MEM_SIZE>() * 0.000976562 << "kb"
          << "\n"
          << "availability: "
          << (d.getInfo<CL_DEVICE_AVAILABLE>() == true ? "true" : "false")
          << "\n"
          << "max work item dim: "
          << d.getInfo<CL_DEVICE_MAX_WORK_ITEM_DIMENSIONS>() << "\n"
          << "image processing support: "
          << (d.getInfo<CL_DEVICE_IMAGE_SUPPORT>() == CL_TRUE ? "true"
                                                              : "false") << "\n";

      auto work_sizes = d.getInfo<CL_DEVICE_MAX_WORK_ITEM_SIZES>();
      for (int i = 0; i < work_sizes.size(); ++i)
        std::cout << "dimension: " << i << " size: " << work_sizes.at(i) << "\n";
    }

    queue_ = cl::CommandQueue(context_, devices[0], 0, &err);

    const size_t buffer_size = width * height * sizeof(float);
    // past, present, future
    std::cout << width << " " << height << " " << buffer_size << "\n";
    try {
      for (size_t i = 0; i < cl_image_.size(); ++i) {
        cl_image_[i] = cl::Buffer(context_,
                              0,  // CL_MEM_READ_ONLY,  // | CL_MEM_COPY_HOST_PTR,
                              buffer_size,   // TODO(lucasw) * sizeof(float) when using float
                              (void *)NULL,  // (void*) &im,  // some random memory
                              &err);
      }
    } catch (cl::Error err) {
      std::cerr << "ERROR: " << err.what() << "(" << err.err() << ")" << "\n";
      return false;
    }

    std::string old_text = "";

    std::cout << cl_file << "\n";
    cl_int rv = loadProg(devices, context_, program_, old_text, cl_file);
    if (rv != CL_SUCCESS) {
      return false;
    }
    try {
      for (size_t i = 0; i < kernels_.size(); ++i) {
        kernels_[i] = cl::Kernel(program_, "pressure", &err);
        kernels_[i].setArg(0, cl_image_[i]);
        kernels_[i].setArg(1, cl_image_[(i + 1) % kernels_.size()]);
        kernels_[i].setArg(2, cl_image_[(i + 2) % kernels_.size()]);
        kernels_[i].setArg(3, static_cast<int>(width));
        kernels_[i].setArg(4, static_cast<int>(height));
      }
    } catch (cl::Error err) {
      std::cerr << "ERROR: " << err.what() << "(" << err.err()
                                 << ")" << "\n";
      return false;
    }
  } catch (cl::Error err) {
    std::cerr << "ERROR: " << err.what() << "(" << err.err() << ")" << "\n";
    return false;
  }

  global_size_ = cl::NDRange(width * height);
  std::cout << "global size " << global_size_ << " " << local_size_ << "\n";
  init_done_ = true;
  return true;
}

bool OclLambda::update(const size_t num_kernel_loops,
    std::array<cv::Mat, 3>& pressure) {
  if (!init_done_) {
    return false;
  }
  // ROS_DEBUG_STREAM("step " << outer_loops_ << " " << inner_loops_);
  size_t cur_inner_loops = 0;

  try {
    // now write the circle to the 'present' buffer
    for (size_t i = 0; i < cl_image_.size(); ++i) {
      queue_.enqueueWriteBuffer(cl_image_[i],
                                CL_TRUE,  // blocking write
                                origin_, region_,
                                pressure[i].data);  // ,
                                // NULL,
                                // &event);
    }
    queue_.enqueueBarrierWithWaitList();

    // TODO(lucasw) make sure this works with any size num_kernel_loops
    // not divisible by 3, and get rid of this multiplier.
    const size_t runs = cl_image_.size() * num_kernel_loops;
    // const size_t runs = kernels_.size() * num_kernel_loops;
    for (size_t i = 0; i < runs; ++i) {
      queue_.enqueueNDRangeKernel(kernels_[inner_loops_ % kernels_.size()],
                                 offset_,
                                 global_size_,
                                 local_size_,
                                 NULL,
                                 &event_);
      queue_.enqueueBarrierWithWaitList();
      cur_inner_loops++;
      inner_loops_++;
    }
    for (size_t i = 0; i < cl_image_.size(); ++i) {
      queue_.enqueueReadBuffer(cl_image_[i],
                               CL_TRUE,  // blocking read
                               origin_, region_,
                               pressure[i].data);
                               // NULL,
                               // &event);
    }
    ++outer_loops_;
  } catch (cl::Error err) {
    std::cerr << "ERROR: " << err.what() << "(" << err.err() << ")" << "\n";
    return false;
  }
  return true;
}  // update
