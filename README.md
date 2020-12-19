# Awesom-O
[![Build Status](https://travis-ci.org/clueless-bachu/Awesom-O.svg?branch=master)](https://travis-ci.org/github/clueless-bachu/Awesom-O)
[![Coverage Status](https://coveralls.io/repos/github/clueless-bachu/Awesom-O/badge.svg?branch=master)](https://coveralls.io/github/clueless-bachu/Awesom-O?branch=master)

## MIT Licence 
[![Packagist](https://img.shields.io/packagist/l/doctrine/orm.svg)](LICENSE) 
---

## Project contibutors

1) [Sneha Nayak](https://github.com/snehanyk05): 2nd year Mengg Robotics @UMD. Interests: Machine Learning, Reinforcement Learning, Motion Planning.
2) [Vasista Ayyagari](https://github.com/clueless-bachu): 1st year Mengg Robotics @UMD. Interests: Computer Vision, Path Planning.
3) [Vishnuu A. Dhanabalan](https://github.com/vishnuu95): 2nd year Mengg Robotics @UMD. Interests: Computer Vision, Autonomy.

## Agile Iterative Process
[![Solo Iterative Process](https://img.shields.io/badge/AIP-ClickHere-brightgreen.svg?style=flat)](https://docs.google.com/spreadsheets/d/1xvJm1XwD0x-FWnv0wH0hJFpIkV3F_uvVeWk7_voXG9g/edit?ts=5fc43c5c#gid=0) 

## Agile Planning
https://docs.google.com/document/d/1TP-40cn_BOVU-LJUg8t6y-jqjPbL3MI7luDLiohg9YM/edit?ts=5fc688f2
## Link to Presentation Slides
https://docs.google.com/presentation/d/1zoR2qmA_PsgM87A83aFrFod_hp6GY0hheZv6_P0coQ0/edit?usp=sharing
## Link to Presentation Video
https://drive.google.com/drive/folders/1QhVnbJzgOnSTSSQTxaUrr-i40nOv9kYT?usp=sharing
## Link to Demo
https://www.youtube.com/watch?v=feeuLlQeejQ

## Overview

Awesom-O is an autonomous threat detection and response robot who’s main objective is to de-tect threats and accordingly eliminate them.  The robot scans the environment, detects potentialthreats which are marked with an AruCo marker.  The Awesom-O then proceeds to autonomouslynavigate towards the object of interest (or threat) while avoiding collision with other obstacles inthe  environment.  When  it  is  within  a  certain  radius  of  the  object  of  interest,  it  disarms  it  andmoves on to its next target, thus eliminating all threats in the environment. 

We managed to implement two methods in which the robot can accurately detect its threats. One is the vision based technique, where the robot can detect it's threat, which is in the form of an arTag with an associated ID. The robot detects all ar tags in its map/environment before it can say that it's "mission" was complete. In the vicinity based method, the robot detects threats when it approches close to the target, and diverges from it's predefined path to disarm the target threat.

We intend on creating a robust set of test cases with:

- cmake
- gtest
- rostest

#### Threat Detection Sample Image
![Fig1. Threat Detection](https://github.com/clueless-bachu/Awesom-O/blob/master/output/image.png)

## Dependencies

* Ubuntu 18.04 (Operating System)
* Modern C++ 11(Programming Language)
* CMake (Build System)
* OpenCV >= 4.4 
* ROS Melodic
* Gazebo
* ar_track_alvar ROS package

## License 

```
MIT License

Copyright (c) 2020 Vasista Ayyagari, Vishnuu A. Dhanabalan, Sneha Nayak

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

## Standard install via command-line Vision Based

```
cd <<Your_catkin_workspace>>/src
git clone --recursive https://github.com/snehanyk05/Awesom-O/
cd ../..
export TURTLEBOT3_MODEL=waffle
catkin_make && source devel/setup.bash
roslaunch awesomo test.launch useVision:=true
```

## Standard install via command-line Vicinity Based

```
cd <<Your_catkin_workspace>>/src
git clone --recursive https://github.com/snehanyk05/Awesom-O/
cd ../..
export TURTLEBOT3_MODEL=waffle
catkin_make && source devel/setup.bash
roslaunch awesomo test.launch useVision:=false
```
## Run Tests 
```
catkin_make run_tests_awesomo
```

## Building for code coverage
```
sudo apt-get install lcov
cmake -D COVERAGE=ON -D CMAKE_BUILD_TYPE=Debug ../
make
make code_coverage
```
This generates a index.html page in the build/coverage sub-directory that can be viewed locally in a web browser.


## Generate Doxygen Documentation

To install doxygen run the following command:
```
sudo apt-get install doxygen
```
Now from the cloned directory run:
```
doxygen doxygen
```

Generated doxygen files are in html format and you can find them in ./docs folder. With the following command
```
cd docs
cd html
google-chrome index.html
```

