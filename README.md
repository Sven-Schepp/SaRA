# SaRA: A Tool for Safe Human-Robot Coexistence and Collaboration through Reachability Analysis
[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
## Overview
SaRA implements our formally safe approach to reachability analysis in form of reachable sets [1,2] of humans. The human's reachable sets are given as a union of individual capsules (cylinder with a half sphere at both ends) for each body part. We can formally guarantee safety by executing a failsafe trajectory if an intersections between the human's and robot's reachable sets is detected [1,2]. We additionally provide classes for simplified cylinder based reachable sets as discussed in [3]. SaRA consist of a library (Reachlib) containing all functionalities required for reachability analysis as well as a [ROS](https://www.ros.org/) package (reachable_occupancy) which includes ReachLib and provides volume calculation and visualization in [RViz](http://wiki.ros.org/rviz). Both the C++ library as well as the ROS-package can be easily integrated, improved, and expanded upon.<br>
We utilize live captured Cartesian positions of human joints calculated from motion captured markers (shown in the figure below). Our efficient implementation is capable of calculating reachable sets and determines intersections between human and robotic reachable sets in only a few microseconds. The tool extends our previous work in [1,2] and is described in detail in our contribution to ICRA 2022. Minimal steps for an executable demo are given in section [Demo](#demo).
<br>A short video explanation and demonstration of reachable sets visualized in RViz based on motion capture data can be found [here](https://www.youtube.com/watch?v=QDYq_FQL1Ds).

<p align="center">
  <img align="center" src="/images/f1.PNG" alt="drawing" height="350" width="350"/><img align="center" src="/images/f1o.PNG" alt="drawing" height="350" width="350"/>
</p>

## Table of Contents

- [Features](#features)
- [How Does SaRA Work](#how-does-sara-work)
- [How to integrate ReachLib](#how-to-integrate-reachlib)
- [How to integrate reachable_occupancy](#how-to-integrate-reachable_occupancy)
- [Requirements](#requirements)
- [Installation](#installation)
- [Installing ReachLib](#installing-reachlib)
- [Installing reachable_occupancy](#installing-reachable_occupancy)
- [Repository Structure](#repository-structure)
- [Software and Hardware](#software-and-hardware)
- [Demo](#demo)
- [License](#license)


## Features
SaRA provides the following features through **ReachLib**:
- Compatibility with Linux and Windows based systems (makefiles are included and can be customized)
- Generation of full body articulated reachable sets previously defined by [Althoff et al.[1]](https://www.science.org/doi/10.1126/scirobotics.aaw1924) and [Pereira et al.[2]](https://ieeexplore.ieee.org/abstract/document/8206314?casa_token=3Lltavadmu0AAAAA:ZaQr5_E1PBVsZFOxVj4DbIRHQULRRCIS90PohfxvBvNjYjKSQ2l3PaFtRUkuuZDwKdaF-Zo)
- Generation of cylindrical reachable sets as defiend by [Liu et al.[3]](https://ieeexplore.ieee.org/abstract/document/8202313?casa_token=rCL9La4MzXYAAAAA:NjQLJit4p1xqzhjcvGqBX1NdzxSWeiqMObqT-NGLqNJKRG-e_kpa7MuQGZHsKl66qJBW-60)
- Measurement uncertainties and system delays are accounted for
- Intersection checks of all occupancy containers among themselves and with point clouds
- Fast C++ based implementation (three models in less than 20 micro seconds)
- No additional libraries required
- Extensible hierarchical class structure
- Built on a small codebase of less than 5000 LOC


SaRA provides the following ROS based functionality through **reachable_occupancy**:
- Visualization of reachable sets calculated within ReachLib
- Volume calculations for all models contained in ReachLib
- Reception and processing of live motion captured data over UDP and ROS topics
- Compatible with ros-bags
- Compatible with live and recorded motion capture data
- Compatibility with ROS melodic and kinetic
- Built on fast C++ based nodes
- Integrable in other ROS projects

## How does SaRA work?
SaRA is composed of a C++ based library (ReachLib) and a ROS-package (reachable_occupancy) both of which can be used independently. The former is portable and can be integrated with any project that supports C++ based libraries, while the latter contains ReachLib and provides further functionalities based on ROS as stated in section [Features](#features).<br>
SaRA provides two general classes of occupancy models: `Articulated` and `Pedestrian` which we generally refer to as a type of `Obstacle` that robots must circumnavigate. The `Articulated` models describe the human reachable sets as a union of the reachable sets of body parts or extremities. There exist three versions of `Articulated` models:<br>
- `ArticulatedAccel`: A set of `BodyPart` reachable sets; Uses live Cartesian **joint positions** and **velocities** with **estimated maximum acceleration** parameters per `BodyPart`
- `ArticulatedVel`: A set of `BodyPart` reachable sets; Uses live Cartesian **joint positions** and **estimated maximum velocity** parameters per `BodyPart`
- `ArticulatedPos`: A set of `Extremity` reachable sets; Uses live Cartesian **joint positions** as well as **estimated length** and **maximum velocity parameters** per `Extremity`
<br>The left image shows how the `BodyPart` based approaches define their reachable sets where each body part is enclosed by a `Capsule`, while `ArticulatedPos` consist of four extremities enclosed by balls (capsules with cylinder height 0).<br><br>

<p align="center">
  <img src="/images/human_accel_vel.PNG" alt="human_accel_vel" height="320" width="325"/><img src="/images/human_pos.PNG" alt="human_pos" height="320" width="350"/>
</p>

A `Capsule` is defined as a cylinder with a half-sphere cap at each end while a ball is given by a sphere and is a special type of capsule whose cylinder has height zero, as shown in section [Overview](#overview) and defined by [Pereira et al.[2]](https://ieeexplore.ieee.org/abstract/document/8206314?casa_token=3Lltavadmu0AAAAA:ZaQr5_E1PBVsZFOxVj4DbIRHQULRRCIS90PohfxvBvNjYjKSQ2l3PaFtRUkuuZDwKdaF-Zo). The red dots mark the joint positions that must be supplied to calculate a human's reachable sets. Additionally, a subset of body part reachable sets can be calculated if only a subset of poits is supplied.<br><br>
The `Pedestrian` models defined by [Liu et al.[3]](https://ieeexplore.ieee.org/abstract/document/8202313?casa_token=rCL9La4MzXYAAAAA:NjQLJit4p1xqzhjcvGqBX1NdzxSWeiqMObqT-NGLqNJKRG-e_kpa7MuQGZHsKl66qJBW-60) are two dimensional represented by circles around the pedestrians and mobile robots. We extend this approach to three dimensions through horizontal cylinders that enclose the reachable sets of a human. They are supplied with only one point (estimated center of the human body) and generate reachable sets based on two models:<br>


- `PedestrianAccel`: A `Cylinder` based aproach; Uses the live **Cartesian position** and **velocity** of a human with **estimated maximum acceleration** and **arm span**<br>
- `PedestrianVel`: A `Cylinder` based aproach; Uses the live** Cartesian position** of a human with **estimated maximum velocity** and **arm span**
While `Pedestrian` models produce obstacles with larger volumes, they also require much less information when comapared to `Articulated` models.
<img src="/images/Cyl.PNG" alt="pedestrian" width="300"/>

## How to integrate ReachLib?
SaRA is C++ based and thus all individual ReachLib headers can be included in any C++ compatible project within the dedicated include directory. To employ individual components of ReachLib within your project, the respective namespaces must be resolved first. An example would be the creation of an `ArticulatedAccel` instance as such:
```cpp
obstacles::articulated::accel::ArticulatedAccel human_a;
```
We provide `reach_lib.hpp` which includes all individual ReachLib definitions and shortcuts to types for ease of use:
```cpp
#include "reach_lib.hpp"
reach_lib::ArticulatedAccel human_a;
```
However, please make sure that the type-definitions and included files do not lead to resolution conflicts with those in your project!

The next step involves instatiation of the aforementioned models. The following example demonstrates how this can be done for the `ArticulatedAccel` model. `ArticulatedAccel` requires a map that links a **body part** with a unique **name** to **two joints** (only one joint per `Extremity` for `ArticulatedPos`), a parameter that estimates the **thickness** (radius of the body part|length of hands and feet), and **estimated maximum acceleration** (maximum velocity for `ArticulatedVel`) per body part. Finally a required `System` object containing measurement uncertainties for position and velocity of your system must be generated. The entire process is shown below:
```cpp
#include <string>
#include <vector>

#include "reach_lib.hpp"

// System parameters
reach_lib::System s = reach_lib::System();
// Joint body part map (name, joint_index1, joint_index2)
std::map<std::string, std::pair<int, int>> index;
// Thickness per body part (name, thickness)
std::map<std::string, double> thickness;
// Estimated maximum acceleration per joint (same order as joint vector during update)
std::vector<double> max_a;
// Instantiation of the model
reach_lib::ArticulatedAccel human_a = reach_lib::ArticulatedAccel(s, index, thickness, max_a);
```
The instatiated model must subsequently be updated at every timestep. `ArticulatedAccel` therefore must be supplied with a vector of `Point` for both joint position (p) and velocity (v), where the points of the body parts from above have the respective indices within p and v. Additionally a time interval of **[t_a, t_b]** must be defined over which the occupancies should be determined.
```cpp
while (simulation_on == true) {
  // Start of the interval 
  double t_a = 0.0;
  // End of the interval 
  double t_b = 0.2;
  // Vector of Cartesian joint positions
  std::vector<Point> p = {};
  // Vector of Cartesian joint velocities
  std::vector<Point> v = {};
  // Updating the human object
  human_a.update(t_a, t_b, p, v);
  // Get current occupancies
  std::vector<BodyPartAccel> occupancy = human_a.get_occupancy();
}
```
## How to integrate reachable_occupancy
The functions within reachable_occupancy can be used in your own ROS project after copying reachable_occupancy into your `catkin/src/` directory. We provide the ability to receive joint locations and velocities over UDP and translate them into ROS messages compatible with reachable_occupancy. <br>
To visualize received or calculated occupancies within RViz, a visualization node of type reachable_occupancy/reach_viz can be called while an instance of RViz is running. Designing a self defined visualization node using the `Visualizer` class within reachable_occupancy is also a viable option. This can be done as follows:
```cpp
#include "reach_lib.hpp"
#incldue "visualizer.hpp"

// Define ros_node, node handle (nh), rate ...
// Define marker publisher
ros::Publisher pub_vis = nh.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 1000);

// Instantiation of the model
reach_lib::ArticulatedAccel human_a = reach_lib::ArticulatedAccel(s, index, thickness, max_a);
// Generate a pointer of general type Articulated* that is passed to the Visualizer
reach_lib::Articulated* human_a_p = &human_a;
// Genearting a generic object of type Visualizer with default parameters
visualizer:.Visualizer v_a = Visualizer();
// Setting the mode of the visualizer
v_a.mode_ = "ARTICULATED-ACCEL";
// Generate an id-map for RViz markers from the human object
v_a.set_id_map(human_a_p, 1);
// Set the opacity of the RViz markers for this model
v_a.color_.a = 0.4;

while (ros::ok()) {
    // Get current psotions and velocities
    // Define t_a, t_b
    // ...
    // Update
    human_a.update(t_a, t_b, p, v);
    // Generate Markers
    v_a.vis_articulated(human_a_p);
    // Publish markers
    pub_vis.publish(v_a.markers_);
}
```



## Requirements
- ROS-melodic (runns identically on kinetic)<br>
- Ubuntu bionic (18.04) or newer
- RViz (comes preinstalled with ROS)
- Visual Studio (Windows only; if you require compilation)

## Installation
### Installing ReachLib
The ReachLib library can be copied into the `include` directory of your project and declared as an additional include directory to the compiler. The headers can then be included in your project. Should the library be linked (statically or dynamically) to your project three steps must be executed:

- Copy the library into your include directory or declare it as an additional include directory
- Set STATIC or SHARED (means dll) in CMakeLists.txt and call `cmake ..` then `make` in ReachLib's build folder (only if you have altered files within ReachLib)
- Build this library from within your own projects makefile and link it against your project

On Widowns this must be done using **Visual Studio** wherein ReachLib can be built and linked statically or dynamically!

This library can be integrated and built within ROS projects by copying ReachLib into the `include` directory of your package and adding the following lines to your packages CMakeLists.txt:
```cmake
## Specify additional locations of header files
## Your package locations should be listed before other locations
## Set location of ReachLib (REACH_RI_INCLUDE_DIRS)
set(SaRA_INCLUDE_DIRS "include/ReachLib/include")

include_directories(
  include
  include/reachable_occupancy
  ${catkin_INCLUDE_DIRS}
  ${SaRA_INCLUDE_DIRS}
)

# Set the path for ReachLib's source files
set(SaRA_SRC_PATH "include/ReachLib/src")

# Set all cpp files used to compile ReachLib
set(SaRA_SOURCES
  ${SaRA_SRC_PATH}/articulated.cpp
  ${SaRA_SRC_PATH}/articulated_accel.cpp
  ${SaRA_SRC_PATH}/articulated_vel.cpp
  ${SaRA_SRC_PATH}/articulated_pos.cpp
  ${SaRA_SRC_PATH}/body_part.cpp
  ${SaRA_SRC_PATH}/body_part_accel.cpp
  ${SaRA_SRC_PATH}/body_part_vel.cpp
  ${SaRA_SRC_PATH}/capsule.cpp
  ${SaRA_SRC_PATH}/cylinder.cpp
  ${SaRA_SRC_PATH}/cylinder_perimeter.cpp
  ${SaRA_SRC_PATH}/extremity.cpp
  ${SaRA_SRC_PATH}/occupancy.cpp
  ${SaRA_SRC_PATH}/pedestrian.cpp
  ${SaRA_SRC_PATH}/pedestrian_accel.cpp
  ${SaRA_SRC_PATH}/pedestrian_vel.cpp
  ${SaRA_SRC_PATH}/point.cpp
  ${SaRA_SRC_PATH}/system.cpp
  ${SaRA_SRC_PATH}/sphere.cpp
)

# Add (compile) the ReachLib library as (ReachLib)
add_library(ReachRI ${REACH_RI_SOURCES})

# Add your executable ROS node (demo_node)
add_executable(demo_node src/demo_node.cpp)

# Link the headers from your own package (required for any ROS project)
target_link_libraries(demo_node ${catkin_LIBRARIES})
# Link (ReachLib) to your node
target_link_libraries(demo_node ReachLib)
# Generate dependencies to your self defined message types if there are any
add_dependencies(demo_node demo_publisher_generate_messages_cpp)

```
### Installing reachable_occupancy
The ROS package reachable_occupancy must merely be copied into the `catkin_ws/src/` directory to be available for use. The entire workspace must then be recompiled by running `catkin_make` in your workspace after which `source devel/setup.bash` must be run. We suggest setting up tasks in [VSCode](https://code.visualstudio.com/) or any other editor that can automate these processes.


## Repository Structure:

- `experiments`: Contains the bag files and executable used for our experiments (can be used for demonstrations)
- `ReachLib`: The ReachLib library used to calculate occupancies
  - `reach_lib/include/`: The header files of ReachLib and reach_lib.hpp which provides type shortcuts and includes all ReachLib headers
  - `reach_lib/src/`: The sourcefiles implementing all classes and functions declared within the headers
- `reachable_occupancy`: A ROS package that include ReachLib and provides visualization of occupancies
  - `reachable_occupancy/include/`: Contains ReachLib and header files that declare functions for visualization, reception, and volume calculation
  - `reachable_occupancy/launch/`: The launchfiles used to run multiple nodes such as RViz in parallel to the visualization
  - `reachable_occupancy/msg/`: The definitions for all custom messages used in this ROS package (mainly time(t_a, t_b))
  - `reachable_occupancy/rviz/`: Contains the RViz configuration file that is loaded when RViz is called through our launchf files (feel free to update it to suit your preferences)
  - `reachable_occupancy/src/`: The source files that implement functions declared in the headers of this package as well as scripts that are called as nodes for visulaization, reception, and volume calculation
    - `reachable_occupancy/src/tests`: Test files for recorded motion capture data transformed into CSV files

## Software and Hardware
The system was primarily tested on native **Ubuntu 18.04(Bionic)** running **ROS Melodic 1.14.7**, **RViz 1.13.17** compiled against **OGRE 1.9.0**, and **Qt 5.9.5**.
All main nodes are written in C++ and adhere to the C++11 standard which is defined as the default C++ version in ROS.

The Simulation was carried out on an **ASUS Zephyrus GX502** containing an **i7-9750H(2.6GHz-4.5GHz)** and an **NVidia RTX 2070(8Gib VRAM)**.
This allows for smooth simulations at **30 frames per second** (Maximum supported by RViz).

## Demo:
A demo visualizing reachable sets of recorded motion captured data is availble after copying the reachable_occupancy package into your `catkin_ws/src` directory and running the following commands.<br>
Open a terminal in your `catkin_ws` directory. Default location: `cd ~/catkin_ws/`<br>
Compile your workspace: `catkin_make`<br>
In `catkin_ws`:<br>
- `source devel/setup.bash`.
- Copy the `SaRA_demo` executable into `catkin_ws/devel/lib/reachable_occupancy/`.
- Launch rviz with the provided config file: "roslaunch reachable_occupancy rviz_demo.launch".
- Run the executable in a separate terminal "rosrun reachable_occupancy SaRA_demo".
- Finally, play one of the chosen bag files provided in the experiments folder: `rosbag play punch.bag`<br><br>
Your visualization should open an RViz instance and display our test environment (two tables in a room)<br>
You should then see the robot (pink) and human reachable sets (blue).
Add these commands to tasks.json in [VSCode](https://code.visualstudio.com/) for convenience.

## Resources
[1] M. Althoff, A. Giusti, S. B. Liu, and A. Pereira, “Effortless creationof  safe  robots  from  modules  through  self-programming  and  self-verification,”Science Robotics, vol. 4, no. 31, Jun 2019<br><br>
[2] A. Pereira  and  M. Althoff,  “Calculating  human  reachable  occupancyfor guaranteed collision-free planning,” inProc. IEEE/RSJ Int. Conf.Intelligent Robots and Systems (IROS), 2017, pp. 4473–4480.<br><br>
[3] S.  B.  Liu,  H.  R ̈ohm,  C.  Heinzmann,  I.  L ̈utkebohle,  J.  Oehlerking,and  M.  Althoff,  “Provably  safe  motion  of  mobile  robots  in  humanenvironments,”  inProc.  IEEE/RSJ  Int.  Conf.  Intelligent  Robots  andSystems (IROS), 2017, pp. 1351–1357.

## License
SaRA is licensed under the terms of the GPL Open Source license and is freely available.<br>
All projects expanding upon or integrating Reach-RI are subject to the terms of the GPL Open Source license.
