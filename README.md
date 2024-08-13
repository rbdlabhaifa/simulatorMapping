# Simulator Mapping

## Requirements:

- Ubuntu 22.04
- At least 4 cores(If you're working from virtual machine please allocate at least 4 cores to the VM)
- Prefered: 6 cores

## Students Guide

For comprehensive guide about the simulator: 
Add Here

## I. Installation:

### 0. Install the simulator
- `./install.sh`*
- `cd Thirdparty/slam/Vocabulary`
- `tar -xf ORBvoc.txt.tar.gz`
- `cd -`

* There is a chance that you will encouter some make errors - they happen because you don't give enough resources to the Virtual Machine. Just give more resources(You can find how easily in the internet). When you run again don't run `./install.sh` instead `cd build` and then `make`.

### 1. Use existing data or build new simulator:
This part is only for users of existing simulator and not meant for users who want to create new simulator model.
If you want to use existing simulator(like the one of the TLV University Lab) download this zip file:
https://drive.google.com/file/d/1McpMjSu7-ziM0-tqMumX0LM7evFS9Irs/view?usp=sharing

After you downloaded the zip file go to `generalSettings.json` and change the paths described in part 1 as follows:
if the path is in the `simulatorMapping` directory - just change the path to your `simulatorMapping` path.
if the path isn't in the `simulatorMapping` directory - change it to the equivalent file in the all-data file you downloaded

### 2. Configure the parameters to your machine:
Open generalSettings.json and change:
- `slam_configuration/VocabularyPath` to the path of your vocabulary on the project
- `slam_configuration/DroneYamlPathSlam` to the path of your drone config file
- `offlineVideoTestPath` to the path of the video you want to run of offline orb slam(described later)
- `loadMapPath`to the path of the map you created with orb slam
- `simulatorOutputDir` to the directory you want the output maps will go to
- `simulator_configuration/modelPath` to the path of your `.obj` model


## II. Usage:
### 1. Run simulator:
- run `./exe/runSimulator` from the `build` folder in order to use the simulator within the model
pay attention to the prints:
you will need to press tab from the model viewer in order to start running the slam through the simulator with pre defined moves

### 2. Navigate to point:
- configure `target_point` in `generalSettings.json`
- run `./exe/navigate_to_point` from the `build` folder in order to use the simulator to navigate to the target point
### 3. Create orb slam map from webcam/drone:
- run `./exe/mapping` from the `build` folder

### 4. create Orb Slam map from video:
- run `./exe/offline_orb_slam` from the `build` folder

### 5. Add your room exit code:
optional if you want to find room exit and navigate to the destanation:
- open the roomExit file from src/RoomExit, include/RoomExit directories
- change the implementation to your implementation of the room exit
- You can check your code using runSimulator
