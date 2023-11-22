//
// Created by dean on 10/20/23.
//
#ifndef SIMULATOR_MANAGER
#define SIMULATOR_MANAGER

#include "polySimulator.h"
//#include "threadManager.h"
#include <iostream>
#include <unistd.h>
//#include "../navigation/ExitRoomFiles/exit_room_algo.cpp"


// This class provides high-level simulator controller in addition to another layer of security for the PolySimulator Class - [encapsulation]
// @TODO: import all the functionalities of polySimulator into the Manager.
class SimulatorManager{
public:
    std::thread simulatorThread;

    //this is a static member function allowing the creation of singleton class.
    static SimulatorManager& getInstance(const bool _loadCustomMap){
        static SimulatorManager sim(_loadCustomMap);
        return sim;
    }

    //this function makes a rotation of 360 deg and SLAMMING using the ORB_SLAM system and with the term that its initialized.
    //You can choose to split the 'angleToCover' parameter to 'angleToCover/angle' times.
    void rotateAndSlam(int angle, int angleToCover){
        actionLock.lock();

        int interval = 0;
        const int roofInterval = 8;
        bool topOrBottom = true;

        if (angle == 0){
            std::cout << "bad argument" << std::endl;
            return;
        }

        std::cout << "The spltting angle is: " << angle << " And the angleToCover is: " << angleToCover << std::endl;
        //simulatorPtr->setInPlaceFlag(true);

        if (!simulatorPtr->isTracking()) {
            simulatorPtr->setTrack();
        }
        if (simulatorPtr->isTracking()) {
            double last_i = 0;
            double partialMove = ((double) angleToCover / (double) angle);
            for (double i = 0, rotates = 0; i < std::ceil(angle) && systemActive; i += partialMove, rotates++) {
//                std::cout << i << std::endl;
                // if we did 2 rotations but the i variable didn't increment, then just finish loop.
                if (rotates > std::ceil(angle) * 2 && i == last_i) {
                    break;
                }
                std::string c;
                if (topOrBottom) {
                    c = "down 0.05";
                    interval++;
                } else {
                    c = "up 0.05";
                    interval++;
                }
                if (interval == roofInterval) {
                    interval = 0;
                    topOrBottom = !topOrBottom;
                }
                simulatorPtr->command(c);
                if (simulatorPtr->getSlamState() != 2) { //if SLAM's state is not 'OK' turn back.
                    c = "cw " + std::to_string(partialMove);
                    simulatorPtr->command(c);
                    i = i - partialMove;
                    continue;
                }
                c = "cw " + std::to_string(-partialMove);
                simulatorPtr->command(c);
                // sleep(1);

                if (simulatorPtr->slamFinishedLoopCloser()) {
                    break; //if we detected a loop, stop the loop and keep on.
                }
                last_i = i;
            }
        }
        if (simulatorPtr->slamFinishedLoopCloser()) {
            std::cout << "Finished Looping" << std::endl;
            simulatorPtr->setSlamIntoLocalizationMode();
            simulatorPtr->setTrack();
        }
        actionLock.unlock();
    }

    void rANDs(){
        simulatorPtr->setTrack();
        while(!simulatorPtr->slamFinishedLoopCloser()) {
            //wait for loop close
        }
        simulatorPtr->setTrack();
    }

    //If the SLAM system closed a loop, you can activate this function to calculate the exit point and render it on the simulator.
    void activateExitRoomAlgorithm(vector<vector<double>> mapPoints){
        std::cout << "Trying to find a room to exit throw" << std::endl;
        AlgoRunner algoClass;
        vector<vector<double>> pointsToRender;
        if (this->simulatorPtr->slamFinishedLoopCloser()){
            vector<double> drone_position_3d = this->simulatorPtr->getDroneLocation();

            vector<double> exit_p = algoClass.findExit(mapPoints, vector<double>(drone_position_3d[0], drone_position_3d[2]));

            if (exit_p.empty()){
                std::cout << "no points" << std::endl;
            } else {
                std::cout << "Number of exit points found: " << exit_p.size()/2 << std::endl;
                for(int i = 0 ; i < exit_p.size()/2 ; i = i + 2){
                    vector<double> temp;
                    temp.push_back(exit_p[i]);
                    temp.push_back(exit_p[i+1]);

                    pointsToRender.push_back(temp);
                }
            }
        }

        for(const auto& p: pointsToRender){
            std::cout << p[0] << " - " << p[1] << std::endl;
        }

        this->simulatorPtr->setPointsToRender(pointsToRender);
        this->simulatorPtr->setFoundExitFlag();
    }

    // this function allows the user control the drone by giving x,y,z coordinates and the drone will follow. ***not using path planning.
    void moveDroneToXYZ(const vector<double>& targetLocation){
        this->actionLock.lock();
        this->simulatorPtr->moveToXYZ(targetLocation);
        this->actionLock.unlock();
    }

    void alignDroneWithAxis(bool withAnimation, PolySimulator::axis ax){
        this->actionLock.lock();
        this->simulatorPtr->alignCameraWithFloor(withAnimation, ax);
        this->actionLock.unlock();
    }

    //This function returns the last frame caught by the drone view.
    cv::Mat captureLatestViewColored(){
        return (*simulatorPtr->getCurrentViewPointerColored()).clone();
    }

    cv::Mat captureLatestViewBW(){
        return (*simulatorPtr->getCurrentViewPointerBW()).clone();
    }

    const cv::Mat* captureLiveViewRef(){
        this->actionLock.lock();
        const cv::Mat* t = this->simulatorPtr->getCurrentViewPointerBW();
        this->actionLock.unlock();
        return t;
    }

    void moveAccordingToSavedPath(){
        vector<vector<double>> droneLocations = this->simulatorPtr->getDronePath();
        if (droneLocations.empty()){
            std::cout << "No path to follow" << std::endl;
        } else {
            for (const auto &p : droneLocations){
                this->simulatorPtr->moveToXYZ({p[0], p[1], p[2]});
            }
        }
    }

    void reflectDroneView(){
        static int i = 0;
        std::string topic = "Current Drone View ";
        std::string convertedI = std::to_string(i);

        std::string combinedString = topic + convertedI;

        if(!this->getSimulatorPointer()->getReflectMode()){
            std::cout << "should enable reflectViewBool for this option to work" << std::endl;
            return;
        }
        while(this->getSimulatorPointer()->getReflectMode()){
            cv::namedWindow(combinedString);
            cv::imshow(combinedString, getSimulatorPointer()->getLastDroneView());
            cv::waitKey(1);
        }
        i++;
        cv::destroyWindow(combinedString);
    }

    static void loadMenuOptions() {
        cout << "Choose an action from the menu: " << endl;
        cout << "1. Give drone coordinates " << endl;
        cout << "2. Place drone in user coordinates " << endl;
        cout << "3. Choose a coordinate for the drone to look at" << endl;
        cout << "4. Align drone by axis" << endl;
        cout << "5. Choose room to start with " << endl;
        cout << "6. Start SLAMMING" << endl;
        cout << "7. Switch to Localization mode " << endl;
        cout << "8. reflect drone's view" << endl;
        cout << "9. Apply rotateAndSlam" << endl;
        cout << "10. add drone locations visited" << endl;
        cout << "11. show drone locations visited" << endl;
        cout << "12. Set In Place Flag" << endl;
        cout << "13. show menu" << endl;
        cout << "14. Exit system" << endl;
    }

    void ManagerKeyListener(){
        int key;
        loadMenuOptions();
        while(listen){
            std::cout << std::endl << "Input 13 for the menu to show up." << std::endl;
            cin >> key;
            std::cout << std::endl << "user choice: " << key << std::endl;
            switch(key){
                case 1: {
                    this->simulatorPtr->printCurrentLocation();
                } break;

                case 2: {
                    double x, y, z;
                    cout << "Please enter 3D coordinate for the drone location: " << endl;
                    cin >> x >> y >> z;
//                    this->simulatorPtr->moveToXYZ({x,z,y}, 30000);
                    this->simulatorPtr->moveToXYZ({x, y, z});
                    this->simulatorPtr->curr_cam_pose = {x, z, y};

                } break;

                case 3: {
                    double x, y, z;
                    cout << "Please enter 3D coordinate for the drone to look at: " << endl;
                    cin >> x >> y >> z;
                    this->simulatorPtr->lookAtPoint({x, y, z});
                } break;

                case 4:{
                    int userInput;
                    cout << "Please enter the alignment way:\n0 - positive x\n1 - negative x\n2 - positive Z\n3 - negative z" << endl;
                    cin >> userInput;
                    switch(userInput){
                        case 0: simulatorPtr->chosenAxis = PolySimulator::posX; break;
                        case 1: simulatorPtr->chosenAxis = PolySimulator::negX; break;
                        case 2: simulatorPtr->chosenAxis = PolySimulator::posZ; break;
                        case 3: simulatorPtr->chosenAxis = PolySimulator::negZ; break;
                        default: simulatorPtr->chosenAxis = PolySimulator::posZ; break;
                    }
                    this->simulatorPtr->alignCameraWithFloor(true, simulatorPtr->chosenAxis);
                } break;

                case 5: {
                    int x;
                    cout << "Please choose a room to start mapping: \n" << "1. Lab \n" << "2. Meeting room \n" << "3. Working room \n" << endl;
                    cin >> x;
                    cout << "choosen x: " << x << endl;
                    switch(x){
                        case 1: {
                            this->simulatorPtr->moveToXYZ({4, -8, -0.17});
                            this->simulatorPtr->curr_cam_pose = {4, -8, -0.17};
                        } break;
                        case 2: {
                            this->simulatorPtr->moveToXYZ({4.58513, 3, 0});
                            this->simulatorPtr->curr_cam_pose = {4.58513, 3, 0};
                        } break;
                        case 3: {
                            this->simulatorPtr->moveToXYZ({-4, -9.30959, 0.174696});
                            this->simulatorPtr->curr_cam_pose = {-4, -9.30959, 0.174696};
                        } break;
                        default: { std::cout << "you entered a wrong value, please choose a number between 1-3" << std::endl; } break;
                    }
                } break;

                case 6: {
                    this->simulatorPtr->setTrack();
                } break;

                case 7: {
                    this->simulatorPtr->getSLAM()->ActivateLocalizationMode();
                } break;

                case 8: {
                    std::cout << "\n-For activating the option, you should click on the simulator window, then press 9 to activate the reflection enabler.\n-Do the same to stop this viewer.\n" << std::endl;
                    reflectDroneView();
                } break;

                case 9: {
                    int angleCoverage, splitAngle;
                    cout << "Please choose an angle to cover and after, choose the amount of splits of this angle\n\n";
                    cin >> angleCoverage >> splitAngle;

                    this->addThreadIntoSystemList(
                            std::thread([splitAngle, angleCoverage, this]() -> void  {
                                rotateAndSlam(splitAngle, angleCoverage);
                            })
                    );
                } break;

                case 10: {
                    this->simulatorPtr->shouldSaveDroneLocations();
                } break;

                case 11: {
                    this->simulatorPtr->showDroneLocations();
                } break;

                case 12: {
                    this->simulatorPtr->setInPlaceFlag(true);
                } break;

                case 13: {
                    loadMenuOptions();
                } break;

                case 14: {
                    this->simulatorPtr->setStopFlag(true);
                    this->listen = false;
                    this->systemActive = false;
                } break;

                default: {
                    cout << "you entered a wrong value!" << endl;
//                    listen = false;
                } break;
            }
            if (key == 404){
                listen = false;
            }
        }
    }

    void reflectViewBool(){
        this->shouldReflectView = !this->shouldReflectView;
        std::cout << "Reflection mode is now: " << this->shouldReflectView << std::endl;
    }

    //no discard allows the programmers and the compiler knows that the returned value is used somewhere in the program.
    [[nodiscard]] bool getReflectStatus() const{
        return this->shouldReflectView;
    }

    void addThreadIntoSystemList(std::thread _thread){
        this->threadSystemList.push_back(std::move(_thread));
    }

    // TODO: this function should be deleted after importing the necessary methods from polySimulator
    PolySimulator* getSimulatorPointer(){
        return this->simulatorPtr;
    }

    ~SimulatorManager(){
        this->systemActive = false;
        this->listenerThread.join();
        for(std::thread &listThread : this->threadSystemList){
            listThread.join();
        }
        delete this->simulatorPtr;
    }

    SimulatorManager(const SimulatorManager&) = delete;
    SimulatorManager& operator=(const SimulatorManager&) = delete;

private:
    PolySimulator *simulatorPtr;
    std::thread listenerThread;
    bool listen = true;
    bool shouldReflectView = false;
    bool systemActive = false;
    std::vector<std::thread> threadSystemList;
    std::mutex actionLock;

    explicit SimulatorManager(bool _loadCustomMap){
        this->simulatorPtr = PolySimulator::getInstance(_loadCustomMap);
        this->simulatorThread = this->simulatorPtr->run();
        this->listenerThread = std::thread(&SimulatorManager::ManagerKeyListener, this);
        this->systemActive = true;
    }
};


#endif