# Occupancy-Grid-FastSLAM

1. [Introduction](#introduction)
2. [Implementation Details](#implementation-details)
   1. [Robot](#robot)
   2. [Rao-Blackwellized Particle Filter](#rao-blackwellized-particle-filter)
       1. [Particles](#particles)
       2. [Scan Matcher](#scan-matcher)
   3. [Sensor](#sensor)
   4. [Wheel Encoder](#wheel-encoder)
3. [How-To](#how-to)
4. [Limitations and Outlook](#limitations-and-outlook)

## Introduction

This repository presents a simulation environment for a differential drive robot along with an implementation of a FastSLAM algorithm for Occupancy Grid Maps based on Rao-Blackwellized Particle Filters. The FastSLAM algorithm is inspired by [Improved Techniques for Grid Mapping with Rao-Blackwellized Particle Filters](http://ais.informatik.uni-freiburg.de/publications/papers/grisetti07tro.pdf) and the implementation is largely based on the algorithms presented in [Probabilistic Robotics](http://www.probabilistic-robotics.org).
The simulator can be used in three different modes:

1. Localization 
2. Mapping
3. SLAM

In Localization mode, a ground truth map of the area is provided and the robot is tasked with localizing itself relative to this map. Mapping is performed given ground truth information about the pose of the robot and a map is constructed from the robot's sensor readings. In SLAM mode, no ground truth information is used at all and the robot has to solve the problem of simulatenously localizing itself while constructing a map of its surroundings.

The rest of this readme is structured as follows. In [Section 2](#implementation-details) we will present an overview of the project architecture and discuss all fundamental components of the implementation in detail. In [Section 3](#how-to) we will provide guidelines for using the simulator for SLAM as well as other robotics applications. [Section 4](#limitations-and-outlook) will be dedicated to analyzing limitations of the current version of the simulation environment and outline potential extensions of the implementation.

<p align="center">
<img src="Images/animation.gif">
</p>

## Implementation Details

The multi-purpose control framework presented in this repository consists of four main components. These components are intended to make the implementation as modular as possible, facilitating extensions to certain components without having to alter the overall structure of the framework. An illustration of the components and their interaction is displayed below.

<p align="center">
<img src="Images/MPC_Framework.png">
</p>

### Robot 

The Map class is a handler for the Occupancy Grid Map of the environment. The map is represented as a binary array classifying each cell as either free or occupied. Moreover, the Map class can act as a wrapper around a potential obstacle detection algorithm. By incorporating e.g. LiDAR measurements and updating the Occupancy Grid Map accordingly, new information about the drivable area can be passed to the Reference Path object.

### Rao Blackwellized Particle Filter

#### Particles

#### Scan Matcher

The Reference Path class is where most of the computations are performed. Ultimately, this is where all the available information is aggregated and processed before passing it to the Model Predictive Controller. In our simulation as well as the real-world test scenario, the entire reference path is known in advance. Consequently, the object contains a list of waypoints along the path at a specified resolution. Each waypoint contains information about its location and orientation within the world coordinate frame as well as the local curvature of the reference path. Furthermore, a speed profile can be computed that associates a reference velocity with each waypoint based on a maximum velocity of the car and the curvature of the path.
In order to be able to account for obstacles in the environment, each waypoint has an additional attribute which contains information about the width of the drivable area on both sides of the center-line. This information is computed dynamically from the information provided by the Map class. Consequently, the Reference Path object contains all necessary information to track the reference path while respecting constraints imposed by obstacles in the environment.

### Sensor

To model the dynamics of the car, we employ a simple Kinematic Bicycle Model. We transform the model into the spatial domain in order to be able to formulate the Path Tracking problem more intuitively by taking as state variables the deviation from the provided center-line in terms of location as well as orientation. Besides the more intuitivate formulation of the problem, the reformulation of the model allows for including time as a state variable and perform Time-Optimal Driving. 
The implementation of the Spatial Bicycle Model is based on [Stability Conditions for Linear Time-Varying Model Predictive Control in Autonomous
Driving](http://urn.kb.se/resolve?urn=urn:nbn:se:kth:diva-220576) by Lima, Mårtensson and Wahlberg as well as [Towards Time-Optimal Race Car Driving Using Nonlinear MPC in Real-Time](https://www.researchgate.net/profile/Robin_Verschueren/publication/269860931_Towards_Time-Optimal_Race_Car_Driving_Using_Nonlinear_MPC_in_Real-Time/links/56ab66e108aeadd1bdce436b/Towards-Time-Optimal-Race-Car-Driving-Using-Nonlinear-MPC-in-Real-Time.pdf?origin=publication_detail) by Vershuren, De Bruyne, Zanon and Frash. For more details, please consult the original publications.
In our implementation, we use the non-linear Spatial Bicycle Model for the simulation of the car. For the computation of the control signals, we resort to a Linear-Time-Variant formulation of the model based on the reference path.

### Wheel Encoder

The Model Predictive Controller is based on a Linear Time-Variant model of the car dynamics. The control signal is recomputed at a specified frequency. Based on the current position of the car, the Spatial Bicycle Model is linearized around the waypoints on the path. The horizon of the optimization problem corresponds to the number of predicted waypoint and can be transformed into a prediction distance using the uniform distance between the waypoints. 
The weight matrices of the cost function to be minimized allow for influencing the expected behavior of the car. Reference Path Tracking can be performed using large weights on the deviation from the center-line. Time-Optimal Driving, on the other hand, can be achieved by penalizing the time the car needs to arrive at the last waypoint of the horizon. This objective corresponds to maximizing the velocity of the car along the reference path, instructing the car to cut corners whenever possible. Finally, obstacle avoidance is performed by shifting the reference value away from the center-line towards the center of the detected drivable area. Thus, the car deviates from the original reference path in order to avoid obstacles with a maximum margin.

## How-To

The implementation is based on a small set of dependencies. For matrix calculations we make use of the Eigen libary. OpenCV is used for visualization. Below we provide a list of all required libraries.

```
opencv4
eigen3
```

### Run Simulation

To start the simulation, go to ```simulation.py```. This script provides a template for loading a map image, specifying a reference path, setting up the motion model and running the simulation. This script was used to generate the GIF displayed above. All simulation parameters can be changed in this script, allowing to expolore the capabilities of the algorithm in different scenarios. Furthermore, the environment can be modified by adding obstacles and boundaries to the map. The modular structure of the implementation allows for an intuitive setup of the simulation, centered around the two functions ```u = mpc.get_control()``` and ```car.send_control(u)```.

### Extend Simulator

In order to test the controller on a real car, we adapt certain components of the implementation to a ROS framework provided for the communication with the vehicle. Again, the modular structure facilitated a quick adaptation. For example, the pose attribute of the spatial bicycle model subscribes to the topic published to by the localization node. The map object is modified by an obstacle detection algorithm that subscribes to the LiDAR data collected by the car. Furthermore, the Spatial Bicycle Model is modified to include a low-level control interface that sends the computed control signals to the respective actuators. We chose not to include the code for the real-world test in this repository as most of the code is tailored towards the proprietary software of the RC car.

## Limitations and Outlook

The test scenarios, in simulation as well as real-world, provide valuable insights into limitations of the controller and illustrate how modifications to the algorithm might improvide performance. The greatest limitation is to be seen in the Kinematic Bicycle Model which doesn't account for a number of physical effects such as torque, traction and tire slip. Moreover, the car is assumed to be velocity-controlled, neglecting the dynamics of the low-level speed controller. We are looking to extend the implementation to include a dynamic bicycle model in the future. We expect a more advanced model to significantly increase the performance on Time-Optimal Driving. Using the current model, optimality corresponds to finding the shortest path. The algorithm is expected to display much more realistic behaviour upon the inclusion of additional physical effects.
A second limitation is related to the task of obstacle avoidance. The avoidance maneuver is entirely based on modifying the constraints of the optimization problem, limiting the allowed deviation from the center-line to the drivable area. This algorithm proves to work very well in simulation. However, uncertainties in the LiDAR measurements which lead to spurious elements in the dynamically updated map data, can lead to inconsitencies in the prediction. This is especially problematic in the case of obstacles that can be avoided on both sides. We are looking to introduce a more robust update of the driveable area constraints that ensures consistency across prediction horizons.
