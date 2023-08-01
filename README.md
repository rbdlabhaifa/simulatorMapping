# Simulator Mapping
## I. Installation:

- `git submodule update --init --recursive`
- `./install.sh`

## II. Usage:
### 1. Configure the parameters to your machine:
cd to `Vocabulary/` directory and run: `tar -xf ORBVoc.txt.tar.gz` 
Open generalSettings.json and change:
- `VocabularyPath` to the path of you project
- `DroneYamlPathSlam` to the path of your drone config file
- `offlineVideoTestPath` to the path of the video you want to run of offline orb slam(described later)
- `loadMapPath`to the path of the map you created with orb slam
- `simulatorOutputDir` to the directory you want the output maps will go to
- `mapInputDir` to the directory your map after offline orb slam in - for using in the simulator
- `modelPath` to the path of your `.obj` model
- `framesOutput` to the folder where `frames_lab_transformation_matrix.csv` is in for using the point cloud transformation we want

### 2. Create video for orb slam mapping:
- run `./exe/mapping` from the `build` folder

### 3. create Orb Slam map from video:
in order to create orb-slam map for using in the simulator you need to:
- run `./exe/offline_orb_slam` from the `build` folder

### 4. Run orb slam on model:
- run `./exe/run_model_with_orb_slam` from the `build` folder

### 5. localize on your own map:
for localize on the orb slam map from the known points on each frame(without model GUI)
- configure your parameters also on `demoSettings.json`
- run `./exe/demo` from the `build` folder

