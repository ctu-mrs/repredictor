# Re-predictor
The buffer re-prediction algorithm for fusion of variably delayed data, described in the conference publication
 * Václav Pritzl, Matouš Vrba, Claudio Tortorici, Reem Ashour, and Martin Saska, **Adaptive Estimation of UAV Altitude in Complex Indoor Environments using Degraded
and Time-Delayed Measurements with Time-Varying Uncertainties**, *submitted to Robotics and Autonomous Systems.*
**TODO:** update this when applicable 


## Prequisites
All prequisites can usually be installed in most Linux distributions through the package manager.
 * the [Eigen](https://eigen.tuxfamily.org/) linear algebra library
 * CMake version at least 3.1.2
 * a `gcc` or `clang` compiler, supporting at least C++17 (C++11 may work as well, not tested)

Instructions are for Linux-only, but the code is OS-agnostic.

## Building
After installing the prequisites, build simply with
```
mkdir -p build && cd build; cmake .. && make -j`nproc`; cd ..
```
This will build the example program, located in `src/example.cpp`.

## Starting the example
The example program can be started after building by running the command
```
build/repredictor_example
```
The program takes a single parameter, specifying number of state updates that should take place (approximately dictating the length of the simulation).
One state update is 100ms, so e.g. `build/repredictor_example 100` will run for approximately 10s.

## Other remarks
The code also contains an implementation of a Kalman filter and a "variable step-Kalman filter" (`varstepLKF`), which is used in the example.
The variable step KF differs from a normal KF implementation only in taking a `generateA(dt)` and `generateB(dt)` functions as parameters which should return the `A` and `B` matrices scaled according to the `dt` (time step), respectively.

## Original implementation
The original (ROS-dependent) implementation: https://github.com/ctu-mrs/mrs_lib

Documentation of the original implementation (the API is almost the same): https://ctu-mrs.github.io/mrs_lib/classmrs__lib_1_1Repredictor.html
