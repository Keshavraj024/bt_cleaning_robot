# Cleaning_Robot
# BehaviorTree_CPP Example

This repository contains an example C++ project that demonstrates how to use the BehaviorTree_CPP library to create and execute behavior trees for AI and robotic systems. The project simulates a cleaning robot that performs tasks to find dirt, move to the dirt, grasp it, move to the trash bin, and place the dirt in the bin. Each task is represented as a behavior tree node, and the overall behavior is coordinated using a combination of sequence and fallback nodes.

## Getting Started

To run this example, you'll need to clone the `behaviortree_cpp` package from GitHub. Follow the steps below:

1. Clone the `behaviortree_cpp` repository:
   ```
   git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git
   ```

2. Build and install the `behaviortree_cpp` package following the instructions provided in the repository's documentation.

3. After installing the `behaviortree_cpp` package, you can compile and run the example code provided in this repository. The main program executes the behavior tree to simulate the cleaning robot's actions.
    ```
    git clone https://github.com/Keshavraj024/bt_cleaning_robot.git
    cd bt_cleaning_robot
    mkdir build & cd build
    cmake ..
    make
    ```

Please refer to the code in this repository for an in-depth understanding of how the BehaviorTree_CPP library is utilized to create behavior trees for robotic systems.

Note: The provided instructions assume you have C++ development tools and the necessary dependencies installed on your system. For more information on the BehaviorTree_CPP library and its usage, refer to the official repository and documentation.