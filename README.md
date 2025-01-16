# OB_GINS_VLP

## Optimization-Based VLP/INS Integrated Navigation System

OB_GINS_VLP is derived from [OB_GINS](https://github.com/i2Nav-WHU/OB_GINS), commit df40ea5e3226c4cd8b74a9f1e55174660f7ad8de.

Loosely coupled and tightly coupled integration are both realized. We recommend you to use visual studio code on linux to run our program. We have provided the configuration files in `.vscode/`.

## 1 Prerequisites

### 1.1 System and compiler

We recommend you use Ubuntu 18.04 or Ubuntu 20.04 with the newest compiler (gcc>=8.0).

```shell
# Ubuntu 18.04 or 20.04

# gcc-8
sudo apt install gcc-8 g++-8
```

### 1.2 GTest (needed for new version of abseil-cpp)

```shell
sudo apt-get install libgtest-dev libgmock-dev
```

### 1.3 abseil-cpp

Follow [abseil-cpp installation instructions](https://abseil.io/docs/cpp/quickstart-cmake.html).

Don't forget to `sudo make install` after compiling.

### 1.4 Ceres

Follow [Ceres installation instructions](http://ceres-solver.org/installation.html). 

The version should be lower than 2.2.0. For example, [2.1.0](http://ceres-solver.org/ceres-solver-2.1.0.tar.gz).

### 1.5 yaml-cpp

```shell
sudo apt install libyaml-cpp-dev
```

## 2 Build OB_GINS_VLP and run demo

Once the prerequisites have been installed, you can clone this repository and build OB_GINS_VLP as follows:

```shell
# Clone the repository
git clone git@github.com:ShawnSun95/OB_GINS_VLP.git

# Build OB_GINS
cd OB_GINS_VLP
mkdir build && cd build

# gcc
cmake ../ -DCMAKE_BUILD_TYPE=Release

make -j4

# Run demo dataset
cd ../
./bin/ob_gins_vlp ./config/1203c0.yaml

# Wait until the program finish
```

## 3 Plot the results and evaluate the accuracy

We provide a program to plot the navigation results and evaluate the accuracy based on a ground truth positions. You could follow an example:

```shell
python3 ./plot_results.py --optimized_poses ./dataset/1203/OB_GINS_TXT.nav --ground_truth ./dataset/1203/ground_truth_2022123_190105.txt  --initial_poses ./dataset/1203/temp.nav
```
