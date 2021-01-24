# Complementary_filter

## Description

A C++ implementation of a complementary filter to perform attitude estimation based on IMU measurements.

A ROS package is built around it to allow testing inside the ROS environment. But, the core part is not dependant on any ROS libraries. It is built using Eigen and a quaternion implementation. 

## Install

Install vcstools if you do not have it already:

    sudo apt-get install python-vcstool

Clone this repository and run vcstools:

    cd src/
    git clone git@github.com:fabien-colonnier/IMU_complementary_filter.git
    vcs-import < complementary_filter/dependencies_https.yaml


## Reference

[Mahony2008](https://ieeexplore.ieee.org/abstract/document/4608934):
R. Mahony, T. Hamel and J. Pflimlin, "Nonlinear Complementary Filters on the Special Orthogonal Group," in IEEE Transactions on Automatic Control, vol. 53, no. 5, pp. 1203-1218, June 2008, doi: 10.1109/TAC.2008.923738.
