# Simulator Mapping
## I. Installation:

- `git submodule update --init --recursive`
- `./install.sh`*

* There is a chance that you will encouter some make errors - they happen because you don't give enough resources to the Virtual Machine. Just give more resources(You can find how easily in the internet). When you run again don't run `./install.sh` but run `cd build` and then `make`.

## II. Usage:
### 0. Use existing data or build new simulator:
This part is only for users of existing simulator and not meant for users who want to create new simulator model.
If you want to use existing simulator(like the one of the TLV University Lab) download this zip file:
https://drive.google.com/file/d/1Raquzboq6UFSv8-rkuv3qI4IYJSXumCu/view?usp=sharing

After you downloaded the zip file go to `generalSettings.json` and change the paths described in part 1 as follows:
if the path is in the `simulatorMapping` directory - just change the path to your `simulatorMapping` path.
if the path isn't in the `simulatorMapping` directory - change it to the equivalent file in the all-data file you downloaded

### 1. Configure the parameters to your machine:
cd to `Vocabulary/` directory and run: `tar -xf ORBvoc.txt.tar.gz` 
Open generalSettings.json and change:
- `VocabularyPath` to the path of your project
- `DroneYamlPathSlam` to the path of your drone config file
- `offlineVideoTestPath` to the path of the video you want to run of offline orb slam(described later)
- `loadMapPath`to the path of the map you created with orb slam
- `simulatorOutputDir` to the directory you want the output maps will go to
- `mapInputDir` to the directory your map after offline orb slam in - for using in the simulator
- `modelPath` to the path of your `.obj` model
- `framesOutput` to the folder where `frames_lab_transformation_matrix.csv` is in for using the point cloud transformation we want

### 2. Create video for orb slam mapping:
This is optional and do it only if you have a physical drone next to you
- run `./exe/mapping` from the `build` folder

### 3. create Orb Slam map from video:
in order to create orb-slam map for using in the simulator you need to:
- run `./exe/offline_orb_slam` from the `build` folder

### 4. Run runSimulator:
- run `./exe/runSimulator` from the `build` folder

## Optional advanced stages:
### 5. localize on your own map:
for localize on the orb slam map from the known points on each frame(without model GUI)
- configure your parameters also on `demoSettings.json`
- run `./exe/demo` from the `build` folder

### 6. Build partial map:
- run `./exe/save_first_frame` from the `build` folder
- run `./exe/remove_map_points` from the `build` folder
- run `./exe/build_new_map_demo` from the `build` folder

