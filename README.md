# Awesom-O
[![Build Status](https://travis-ci.org/clueless-bachu/Awesom-O.svg?branch=iter-dev-1)](https://travis-ci.org/github/clueless-bachu/Awesom-O)
[![Coverage Status](https://coveralls.io/repos/github/clueless-bachu/Awesom-O/badge.svg?branch=master)](https://coveralls.io/github/clueless-bachu/Awesom-O?branch=master)

## MIT Licence 
[![Packagist](https://img.shields.io/packagist/l/doctrine/orm.svg)](LICENSE) 
---

## Project contibutors

## Project contibutors

1) [Sneha Nayak](https://github.com/snehanyk05)
2) [Vasista Ayyagari](https://github.com/clueless-bachu)
3) [Vishnuu A. Dhanabalan](https://github.com/vishnuu95)

## Agile Iterative Process
[![Solo Iterative Process](https://img.shields.io/badge/AIP-ClickHere-brightgreen.svg?style=flat)](https://docs.google.com/spreadsheets/d/1xvJm1XwD0x-FWnv0wH0hJFpIkV3F_uvVeWk7_voXG9g/edit?ts=5fc43c5c#gid=0) 

## Agile Planning
https://docs.google.com/document/d/1TP-40cn_BOVU-LJUg8t6y-jqjPbL3MI7luDLiohg9YM/edit?ts=5fc688f2


## Overview

Awesom-O is an autonomous threat detection and response robot who’s main objective is to de-tect threats and accordingly eliminate them.  The robot scans the environment, detects potentialthreats which are marked with an AruCo marker.  The Awesom-O then proceeds to autonomouslynavigate towards the object of interest (or threat) while avoiding collision with other obstacles inthe  environment.   When  it  is  within  a  certain  radius  of  the  object  of  interest,  it  disarms  it  andmoves on to its next target, thus eliminating all threats in the environment

We intend on creating a robust set of test cases with:

- cmake
- gtest
- rostest

## Dependencies

* Ubuntu 18.04 (Operating System)
* Modern C++ 11(Programming Language)
* CMake (Build System)
* OpenCV >= 4.4 with dnn and tracking library

## License 

```
MIT License

Copyright (c) 2020 Sukoon Sarin, Sneha Nayak

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
## Instructions to install Opencv4.4

```
git clone https://github.com/opencv/opencv.git
cd opencv
git checkout 4.4.0
cd ..
git clone https://github.com/opencv/opencv_contrib.git
cd opencv_contrib
git checkout 4.4.0
cd ..
cd opencv
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE \
-D CMAKE_INSTALL_PREFIX=/usr/local \
-D INSTALL_C_EXAMPLES=OFF \
-D INSTALL_PYTHON_EXAMPLES=OFF \
-D WITH_TBB=ON \
-D WITH_V4L=ON \
-D WITH_QT=ON \
-D WITH_OPENGL=ON \
-D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
-D BUILD_EXAMPLES=OFF ..

make -j$(nproc)
sudo make install
sudo sh -c "echo '/usr/local/lib' >> /etc/ld.so.conf.d/opencv.conf"
sudo ldconfig 
```
## Standard install via command-line

```
git clone --recursive https://github.com/snehanyk05/Awesom-O/
cd <path to repository>
mkdir build
cd build
cmake ..
make
Run tests: ./test/cpp-test
Run program: ./app/shell-app 
```

## Building for code coverage (for assignments beginning in Week 4)
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
