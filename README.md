# Simulator Mapping

## Requirements:

- Ubuntu 22.04
- At least 4 cores(If you're working from virtual machine please allocate at least 4 cores to the VM)
- Prefered: 6 cores

## Students Guide

For comprehensive guide about the simulator: 
Add Here

## I. Installation:

- `./install.sh`*

* There is a chance that you will encouter some make errors - they happen because you don't give enough resources to the Virtual Machine. Just give more resources(You can find how easily in the internet). When you run again don't run `./install.sh` instead `cd build` and then `make`.

## II. Usage:
### 0. Use existing data or build new simulator:
This part is only for users of existing simulator and not meant for users who want to create new simulator model.
If you want to use existing simulator(like the one of the TLV University Lab) download this zip file:
https://drive.google.com/file/d/1McpMjSu7-ziM0-tqMumX0LM7evFS9Irs/view?usp=sharing

After you downloaded the zip file go to `generalSettings.json` and change the paths described in part 1 as follows:
if the path is in the `simulatorMapping` directory - just change the path to your `simulatorMapping` path.
if the path isn't in the `simulatorMapping` directory - change it to the equivalent file in the all-data file you downloaded

### 1. Configure the parameters to your machine:
Open generalSettings.json and change:
- `VocabularyPath` to the path of your vocabulary on the project
- `DroneYamlPathSlam` to the path of your drone config file
- `offlineVideoTestPath` to the path of the video you want to run of offline orb slam(described later)
- `loadMapPath`to the path of the map you created with orb slam
- `simulatorOutputDir` to the directory you want the output maps will go to
- `mapInputDir` to the directory your map after offline orb slam in - for using in the simulator
- `modelPath` to the path of your `.obj` model

### 2. Create video for orb slam mapping:
This is optional and do it only if you have a physical drone next to you and tou want to create new simulator model
- run `./exe/mapping` from the `build` folder

### 3. create Orb Slam map from video:
this is optional and do it only if you want to create a new simulator model
in order to create orb-slam map for using in the simulator you need to:
- run `./exe/offline_orb_slam` from the `build` folder

### 4. Add your room exit code:
optional if you want to find room exit and navigate to the destanation:
- open the roomExit file from src/RoomExit, include/RoomExit directories
- change the implementation to your implementation of the room exit

### 5. Run runSimulator:
- run `./exe/runSimulator` from the `build` folder in order to use the simulator within the model
pay attention to the prints:
you will need to press tab from the model viewer in order to start running the slam through the simulator with pre defined moves
