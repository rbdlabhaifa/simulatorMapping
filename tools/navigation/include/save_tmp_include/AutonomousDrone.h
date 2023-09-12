
#include "simulator.h"
#include "RoomExit.h"

class AutonomousDrone{
public:
    AutonomousDrone(Simulator *sim);
    void run();
private:
    Simulator *simulator;
};