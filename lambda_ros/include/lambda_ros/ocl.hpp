// Copyright 2018-2020 Lucas Walter

#define __CL_ENABLE_EXCEPTIONS
// These produce compile errors like
// /usr/include/CL/cl.hpp:482:1: error: expected unqualified-id before ‘{’ token
//  {
//
// #define __NO_STD_VECTOR
// #define __NO_STD_STRING

#if defined(__APPLE__) || defined(__MACOSX)
#include <OpenCL/cl.hpp>
#else
#include <CL/cl.hpp>
#endif

