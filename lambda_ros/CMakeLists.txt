cmake_minimum_required(VERSION 2.8.3)
project(lambda_ros)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  # nodelet
  geometry_msgs
  message_generation
  roscpp
  roslint
  sensor_msgs
  spectrogram_paint_ros
  std_msgs
)

set (CMAKE_CXX_STANDARD 11)

find_package(OpenMP)
if (OPENMP_FOUND)
  message("Using OpenMP")
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

roslint_cpp()

# find_package(lambda REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/Lambda.cfg
)

add_message_files(
  FILES
  AudioSource.msg
)

# add_service_files(
#   FILES
#   AddAssembly.srv
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
# )

catkin_package(
  # INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS nodelet roscpp sensor_msgs std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
#   ${lambda_INCLUDE_DIRES}
)

add_library(lambda_synth
  src/lambda.cpp
)

# TODO(lucasw) replace cimg with opencv and eliminate this X11
target_link_libraries(lambda_synth ${catkin_LIBRARIES} X11)
# add_dependencies(lambda_synth ${PROJECT_NAME}_gencpp)

add_executable(${PROJECT_NAME} src/lambda_ros.cpp)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} lambda_synth)

add_executable(array_test src/array_test.cpp)
target_link_libraries(array_test ${catkin_LIBRARIES} lambda_synth)

# TODO(lucasw) is this needed?
if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_LIBRARIES})
endif()

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install(FILES
#   nodelet_${PROJECT_NAME}.xml
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
