* Clone the GitHub repo
```
git clone git@github.com:rbdlabhaifa/AutonomousDroneCPP.git
```
* Install Opencv with the following link: https://linuxize.com/post/how-to-install-opencv-on-ubuntu-20-04/

Make sure you do git checkout 3.4.16 in opencv and opencv_contrib.

you can check if the checkout worked with 

```
git describe --tags
```


Afterwards, you need to install the Eigen library. Go to 

Once you've done that, run
```
chmod +x installDep.sh
./installDep
```
 If there are problems with flip.cpp in ctello/examples, change add "include" to the ctello.h in the file.
 If there is an error with Eigen, make sure that all the Eigen addresses are crrect in CMakeLists.txt and if it still doesn't work, try:
 ```
 cd /usr/local/include
 sudo cp -r Eigen ..
 ```
 
 Change the appropiate paths in general_settings.json, make sure that the parameter loadMap is false.
 
