# PolySimulator README

## Introduction
Welcome to the PolySimulator, a powerful platform for the empirical validation and exploration of UAV (Unmanned Aerial Vehicle) algorithms. This simulator is designed to provide a unique set of features that set it apart in the field of UAV research and development.

This class is enveloped by another class, called SimulatorManager which emphasize the security for the system by the encapsulation property allowing the user to enjoy the full functionalities of the simulator and in the same time, completely hide the instance from the user.

## Key Features

### 1. Seamless Integration of ORB-SLAM2
At the core of the PolySimulator lies the seamless integration of the cutting-edge ORB-SLAM2 system. This integration significantly enhances the precision of localization and mapping, offering an unparalleled level of accuracy. This high level of accuracy adds an authentic touch to simulated scenarios, making it an essential component for rigorous algorithmic evaluation.

### 2. Versatile Environmental Modeling
The PolySimulator takes environmental modeling to the next level by incorporating Polycam's 3D models. These models are highly adaptable and can be utilized for various environmental configurations, both indoor and outdoor. This flexibility ensures that the simulator can emulate a wide range of real-world scenarios.

### 3. Inherent Adaptability
One of the standout features of the PolySimulator is its inherent adaptability. Users have the ability to create custom SLAM (Simultaneous Localization and Mapping) maps within the simulated environment. This functionality provides a vast landscape for algorithm development and refinement. Leveraging SLAM maps allows users to iteratively enhance their algorithms, effectively reducing the need for resource-intensive real-world drone testing, particularly when dealing with complex algorithmic research.

### 4. Singleton
This class is designed by the singleton pattern which creates a safer environment for the user to use the simulator not allowing duplicated resources and parts of the system.
## Getting Started

### Installation
To use the PolySimulator, ensure that you have completed the installation process thoroughly, including configuring the required paths. Make sure to specify the location of Polycam's model files, including the `.obj` file, its corresponding `.mtl` file, and any texture files (`.png` or `.jpg`).
When creating the instance of the simulator by using the SimulatorManager, make sure that you choose if you want to load the pre-chosen model or just point a new path to it in the cmd.

### Feel free to add functionalities that will help improve the abilities of the simulator.

### Usage
1. Configure the simulator by pointing the configuration settings file to the location of Polycam's model files.
2. Create an instance of the SimulatorManager class.
3. Start exploring the wide range of options and tools available for developers.
4. Look for examples in runPolySimulator.cpp

## PolySimulator Class
The PolySimulator class offers a comprehensive set of functions and tools that developers can utilize. It includes a set of pre-integrated automated actions that align with the simulator's core functionalities.

In conclusion, the PolySimulator serves as a crucial bridge between theoretical algorithmic formulations and their practical implementation, making it an indispensable tool for UAV researchers and practitioners. Its feature-rich environment, seamless integration with ORB-SLAM2, and adaptability through SLAM maps make it a valuable asset in the field of UAV research and development.
