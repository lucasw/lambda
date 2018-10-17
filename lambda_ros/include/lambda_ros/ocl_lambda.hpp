#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <lambda/ocl.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>

// load the opencl program from the disk
// TBD optionally provide the old program, if it hasn't changed
// then don't rebuild it
bool loadProg(std::vector<cl::Device> &devices, cl::Context &context,
              cl::Program &program, std::string &old_text,
              const std::string cl_name = "buffer.cl") {

  // look at using inotfiy to see changes to file
  // http://stackoverflow.com/questions/4664975/monitoring-file-using-inotify
  // http://en.highscore.de/cpp/boost/asio.html
  // http://boost.2283326.n4.nabble.com/ASIO-file-monitoring-help-td4645105.html
  // http://stackoverflow.com/questions/12564039/alternatives-to-inotify-to-detect-when-a-new-file-is-created-under-a-folder/24970801#24970801
  try {
    std::ifstream cl_file;
    cl_file.open(cl_name.c_str()); //, std::ios::in);
    if (!cl_file.is_open()) {
      ROS_ERROR_STREAM("Could not open " << cl_name.c_str());
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
    ROS_ERROR_STREAM("ERROR: " << err.what() << "(" << err.err() << ")");

    ROS_INFO_STREAM(
        "build status: " << program.getBuildInfo<CL_PROGRAM_BUILD_STATUS>(
            devices[0]));
    ROS_INFO_STREAM("build options:\t"
                    << program.getBuildInfo<CL_PROGRAM_BUILD_OPTIONS>(
                           devices[0]));
    ROS_INFO_STREAM("build log:\t "
                    << program.getBuildInfo<CL_PROGRAM_BUILD_LOG>(devices[0]));

    return err.err();
  }
}

// provide the location of the .cl file in the first argument
// then optionally provide the video number, 0 for /dev/video0 and so on,
class OclLambda {
  // std::string cl_file = "buffer.cl";
  OclLambda(const std::string cl_file, const size_t width, const size_t height) {
    std::cout << "using " << cl_file << std::endl;
    init(cl_file, width, height);
  }

  bool init(const std::string cl_file, const size_t width, const size_t height) {
    cl_int err = CL_SUCCESS;
    try {
      std::vector<cl::Platform> platforms;
      cl::Platform::get(&platforms);
      if (platforms.size() == 0) {
        ROS_INFO_STREAM("Platform size 0");
        return false;
      }

      for (cl::Platform &p : platforms) {
        ROS_INFO_STREAM("Platform name: "
                        << p.getInfo<CL_PLATFORM_NAME>() << "\n"
                        << "Vendor: " << p.getInfo<CL_PLATFORM_VENDOR>() << "\n"
                        << "Version: " << p.getInfo<CL_PLATFORM_VERSION>() << "\n"
                        << "Profile: " << p.getInfo<CL_PLATFORM_PROFILE>() << "\n"
                        << "Extenstions: "
                        << p.getInfo<CL_PLATFORM_EXTENSIONS>());
      }

      cl_context_properties properties[] = {
          CL_CONTEXT_PLATFORM, (cl_context_properties)(platforms[0])(),
          0 // the callback
      };
      // cl::Context context(CL_DEVICE_TYPE_CPU, properties);
      context_ = cl::Context(CL_DEVICE_TYPE_GPU, properties);

      std::vector<cl::Device> devices = context.getInfo<CL_CONTEXT_DEVICES>();

      for (cl::Device &d : devices) {
        // TODO(lucasw) INFO_STREAM didn't print out much of this, just the first line
        // ROS_INFO_STREAM(
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
          ROS_INFO_STREAM("dimension: " << i << " size: " << work_sizes.at(i));
      }

      queue_ = cl::CommandQueue(context, devices[0], 0, &err);

      size_t origin = 0;
      size_t region = width * height * sizeof(float);

      const size_t buffer_size = width * height * sizeof(float);
      // past, present, future
      ROS_INFO_STREAM(width << " " << height << " " << buffer_size);
      try {
        for (size_t i = 0; i < cl_image_.size(); ++i) {
          cl_image_[i] = cl::Buffer(context,
                                0,  // CL_MEM_READ_ONLY, // | CL_MEM_COPY_HOST_PTR,
                                buffer_size,  // TODO(lucasw) * sizeof(float) when using float
                                (void *)NULL, //(void*) &im, // some random memory
                                &err);
        }
      } catch (cl::Error err) {
        ROS_ERROR_STREAM("ERROR: " << err.what() << "(" << err.err() << ")");
        return EXIT_FAILURE;
      }

      std::string old_text = "";

      cl_int rv = loadProg(devices, context, program, old_text, cl_file);
      if (rv != CL_SUCCESS) {
        return EXIT_FAILURE;
      }
      try {
        for (size_t i = 0; i < kernels.size(); ++i) {
          kernels[i] = cl::Kernel(program, "pressure", &err);
          kernels[i].setArg(0, cl_image_[i]);
          kernels[i].setArg(1, cl_image_[(i + 1) % kernels.size()]);
          kernels[i].setArg(2, cl_image_[(i + 2) % kernels.size()]);
          kernels[i].setArg(3, width);
          kernels[i].setArg(4, height);
        }
      } catch (cl::Error err) {
        ROS_ERROR_STREAM("ERROR: " << err.what() << "(" << err.err()
                                   << ")");
        return EXIT_FAILURE;
      }

      cv::imshow("input image", pressure[1]);
      Mouse mouse;
      cv::setMouseCallback("input image", onMouse, &mouse);
      int ch = cv::waitKey(200);

      // clear all the images
      for (size_t i = 0; i < cl_image_.size(); ++i) {
      queue.enqueueWriteBuffer(cl_image_[i],
                               CL_TRUE, // blocking write
                               origin_, region_,
                               pressure[i].data //,
                                        // NULL,
                                        //&event
      );
      }
      queue.enqueueBarrierWithWaitList();
    }
    } catch (cl::Error err) {
      std::cerr << "ERROR: " << err.what() << "(" << err.err() << ")" << "\n";
      return false;
    }

    global_size_ = cl::NDRange(width * height);

    return true;
  }

  cl::Event event_;
  const int num_kernels_ = 3;
  std::array<cl::Kernel, num_kernels_> kernels_;
  std::array<cl::Buffer, num_kernels_> cl_image_;
  cl::CommandQueue queue_;
  cl::Program program_;
  cl::Context context_;
  cl::NDRange global_size_;

  // local size doesn't mean much without having local memory
  // the null range causes automatic setting of local size,
  // and because the cpu has four cores it looks like it sets the
  // local size to be the global_size divided by 3 (leaving 1 core free for
  // other tasks) though other times it looks like sets the local size to the
  // full size
  cl::NDRange local_size = cl::NullRange;
  // cl::NDRange local_size(16, 16);
  // this is just a global offset, don't think global size has to shrink
  // by same amount
  // Null and 0,0 should be the same
  // cl::NDRange offset(0, 0);
  cl::NDRange offset_ = cl::NullRange;

  int outer_loops_ = 0;
  int inner_loops_ = 0;

  void update(const size_t num_kernel_loops = 100,
      std::array<cv::Mat, 3>& pressure) {
    // ROS_DEBUG_STREAM("step " << outer_loops_ << " " << inner_loops_);
    int cur_inner_loop = 0;

    try {
      // now write the circle to the 'present' buffer
      for (size_t i = 0; i < cl_image_.size(); ++i) {
        queue.enqueueWriteBuffer(cl_image_[i],
                                 CL_TRUE, // blocking write
                                 origin, region,
                                 pressure[i].data //,
                                          // NULL,
                                          //&event
        );
      }
      queue.enqueueBarrierWithWaitList();

      // TODO(lucasw) make sure this works with any size num_kernel_loops
      // not divisible by 3, and get rid of this multiplier.
      const size_t runs = cl_image_.size() * num_kernel_loops;
      // const size_t runs = kernels_.size() * num_kernel_loops;
      for (size_t i = 0; i < runs; ++i) {
        queue.enqueueNDRangeKernel(kernels_[inner_loops_ % kernels_.size()],
                                   offset_,
                                   global_size_,
                                   local_size_,
                                   NULL,
                                   &event_);
        queue.enqueueBarrierWithWaitList();
        cur_inner_loops++;
        inner_loops++;
      }
      for (size_t i = 0; i < cl_image_.size(); ++i) {
        queue.enqueueReadBuffer(cl_image_[i],
                                CL_TRUE,  // blocking read
                                origin, region,
                                pressure[i].data
                                  // NULL,
                                  //&event
        );
      }
      ++outer_loops_;
    } catch (cl::Error err) {
      std::cerr << "ERROR: " << err.what() << "(" << err.err() << ")" << "\n";
      return false;
    }
    return true;
  }  // update
}
