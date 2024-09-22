# GeometricRobotics

This is a C++ implementation of robot kinematics and dynamics following the geometric robotics approach presented by: 

`A Mathematical Introduction to Robotic Manipulation`

We also combined pearls from the book: 

`Rigid Body Dynamics Algorithms` by Roy Featherstone

### Dependencies
##### Sample robots:  
Please install the following ROS packages:

- [Anymal](https://github.com/ANYbotics/anymal_b_simple_description)
- [JVRC](https://github.com/stephane-caron/jvrc_description)

Then define the variable `rosRobotPath` in the file [unittest/CMakeLists.txt].

##### Dependencies without unittest
The dependent libraries include:
- `CMake` According to the [gtest tutorial](https://google.github.io/googletest/quickstart-cmake.html), we use the `FetchContent` module of `CMake`  to configure `gtest` automatically. `FetchContent` requires `CMake 3.11` at the minimum. We can update `cmake` according to this [post](https://gite.lirmm.fr/yuquan/geometricrobotics/-/wikis/Update-cmake). 

- [RobotInterface](https://gite.lirmm.fr/yuquan/RobotInterface) An interface to modules of the [Dart library](https://dartsim.github.io/)
- [RoboticsUtils](https://gite.lirmm.fr/yuquan/roboticsutils) A collection of commonly-used functions.

- [FIDynamics](https://gite.lirmm.fr/yuquan/fidynamics) Please checkout the latest branch by:

```bash
    git checkout -b 3d-model remotes/origin/3d-model
```
##### Dependencies with unittest

- [mc_panda](https://github.com/jrl-umi3218/mc_panda) We need the `URDF` model of the Panda robot from this package.

### Installation

Please clone the package with: 
```sh
git clone --recursive git@gite.lirmm.fr:yuquan/geometricrobotics.git
```

Note that we have two submodules: 
  - [cmake] `https://github.com/jrl-umi3218/jrl-cmakemodules` 
  - [CMakeModules]  `git@github.com:wyqsnddd/CMakeModules.git`


Then install:

```sh
mkdir build && cd build 
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
sudo make install -j`expr $(nproc) / 2`
```

### Documentation

We summarize the APIs [here](https://yuquan.lirmm.net/geometricrobotics/). Or we can easily generate them locally by: 

```
doxygen Doxyfile && firefox docs/html/index.html &
```


### Known issues
1. URDF parsing: 

In the following example, the item `origin` contains multiple `spaces` between numeric values. Sometimes it eables `segmentation fault`.

``` xml
<joint name="RH_HFE" type="revolute">    
        <parent link="RH_HIP"/>    
        <child link="RH_THIGH"/>    
        <origin xyz="-0.0635          -0.041                    0.0"/>    
        <axis xyz="0 1 0"/>    
        <limit command_effort="40" current="10" effort="80" gear_velocity="10" lower="-9.42" upper="9.42" velocity="15.0"/>    
        <dynamics damping="0.0" friction="0.0"/>  
    </joint>
```

We need to manually remove the spaces and change it into 
```xml
<joint name="RH_HFE" type="revolute">    
        <parent link="RH_HIP"/>    
        <child link="RH_THIGH"/>    
        <origin xyz="-0.0635 -0.041 0.0"/>    
        <axis xyz="0 1 0"/>    
        <limit command_effort="40" current="10" effort="80" gear_velocity="10" lower="-9.42" upper="9.42" velocity="15.0"/>    
        <dynamics damping="0.0" friction="0.0"/>  
    </joint>
```

