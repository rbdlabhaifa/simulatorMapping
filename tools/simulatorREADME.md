# Simulator Project

## Overview

Welcome to the Simulator Project, a versatile platform designed for simulating robotic navigation and mapping. This project leverages the power of ORBSLAM2 and 3D blender models to establish a robust testing bed for SLAM and navigation algorithms, eliminating the need for a physical robot.

By employing our simulator with a 3D blender model, you can virtually navigate a robot in diverse environments. The simulator utilizes Pangolin to capture the viewer image from the display window and then feed it into ORBSLAM2. This innovative methodology allows users to observe, scrutinize, and optimize the performance of their algorithms in real-time with expansive and adaptable datasets. Free models are available at [TurboSquid](https://www.turbosquid.com/Search/3D-Models/free/interior/blend#).

## Features

- Simulated robotic navigation within 3D environments.
- Real-time generation and navigation of ORBSLAM2 maps from the 3D model.
- Real-time visualization leveraging Pangolin.
- Test, validate, and enhance navigation and SLAM algorithms without requiring actual robotic hardware.

## Installation

To install the simulator project, execute the following commands in your terminal:

```
chmod +x install.sh
bash install.sh
```

## Usage

After a successful installation and build, observe the "runSimulator.cc" example located in our "exe" directory. You can run this from the "build" directory to get started.

Make sure that you update the config file to you environment, ***change user params method is needed**

## Contributing

We warmly welcome contributions and suggestions to enhance the Simulator Project. Please follow the standard 'fork -> feature branch -> pull request' workflow. Should you have any queries or suggestions, don't hesitate to contact us.

## Contact

For further assistance or inquiries, you can reach us at [tzuk9800@gmail.com](mailto:tzuk9800@gmail.com).

## License

This project is licensed under the GPL license.

---

**Note:** This project is actively under development, and more features are to be introduced in the future. Stay tuned for updates!
