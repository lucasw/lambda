# Lambda

![](https://raw.githubusercontent.com/gesellkammer/lambda/master/pics/icon2/lambdaicon128.png)

# INSTALLATION

Install ROS and clone this project into your catkin workspace src dir.

# Building from source

## Linux

Required packages:

  ROS Kinetic or Melodic - desktop-full
  dynamic_reconfigure_tools
  spectrogram_paint_Ros

# Usage

## Run the simulation

  roslaunch lambda_ros lambda.launch

## Defining a simulation

A Simulation consists of:

* a 2D space with a specific air propagation speed `C` and density `rho`
* one or more sources (ideal sources, noise or sampled sources)
* 0 or more walls - the simulation edges act as walls of a sort.


Original library authors:

simon.ahrens@stud.fh-oldenburg.de
matthias.blau@fh-oldenburg.de
marco.ruhland@stud.fh-oldenburg.de
[www.hoertechnik-audiologie.de](www.hoertechnik-audiologie.de)



[osxbin]: https://github.com/gesellkammer/lambda/raw/master/dist
