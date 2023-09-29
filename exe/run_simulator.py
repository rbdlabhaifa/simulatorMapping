import sys
from os.path import dirname
sys.path.append(dirname("../build/"))
import simulator_python


def run_simulator():
    simulator = simulator_python.Simulator("/home/liam/dev/rbd/simulatorMapping/config/tello_9F5EC2_640.yaml", "/home/liam/Documents/aaa/untitled.obj", "floor", True, False, "/home/liam/Documents/slamMaps/example_mapping11/", False, "", 0.01, "/home/liam/dev/rbd/simulatorMapping/Vocabulary/ORBvoc.txt", 1)
    simulator.setTrack(True)
    simulator.simulatorRunThread()


if __name__ == "__main__":
    run_simulator()
