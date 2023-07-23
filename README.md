For running simulator:

Start with ./install.sh

After we installed the program we want to run it, this is the flow:

1. We want to run video with offline orb-slam for create for us simulator-compatible csv file.
For doing that we need:

I) cd Vocabulary; tar -xf ORBvoc.txt.tar.gz; cd ..

II) Change in generalSettings.json: *VocabularyPath* to ${simulatorMappingDir}/Vocabulary/ORBvoc.txt, *DroneYamlPathSlam* to ${simulatorMappingDir}/config/tello_9F5EC2_640.yaml, *offlineVideoTestPath* to where the video saved and *simulatorOutputDir* to where the directory of the result will be saved

Note: I added some videos to the shared simulator google drive directory

III) run build/offline_orb_slam

2. Now we want to adjust some things:

I) Change the result of the offline_orb_slam dir to the path and the name you want it to be

II) Change in generalSettings.json: *mapInputDir* to where the result of the offline saved.

III) Change *startingCameraPosX*, *startingCameraPosY*, *startingCameraPosZ*, *yawRad*, *pitchRad* and *rollRad* to where you want the first position of the simulator to be.

IV) Change *movingScale* to how much you want to move when press on moving button and *rotateScale* to how much you want to rotate in rotate buttons(in radians)

3. Now we want to run ./mapping to open orb-slam.

I) Run ./mapping

II) Click Open Simulator button when you want to and now the simulator opens in the first position you defined

III) Move with moving buttons and rotate with rotate buttons

salam
