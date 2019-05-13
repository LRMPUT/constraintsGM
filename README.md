**Author/Maintenance**:
Dominik Belter

# This is the part of the Walkers library related to constraints modeling and evaluation using Gaussian Mixtures.

## Installer
    Installation instruction was tested on Ubuntu 18.04 operating system.

The clone_deps.sh script installs all required software from repositories, clones requires libraries and compiles them:

     $ mkdir ~/Sources
     $ cd ~/Sources
     $ git clone https://github.org/...
     $ mkdir ~/Libs
     $ cd ~/Sources/constraintsGM
     $ ./scritps/clone_deps.sh
     
## Running the application

Run ./demoVisualizer to visualize the robot:

     $ cd ~/Sources/constraintsGM/build/bin     
     $ ./demoVisualizer

Run ./robotModelGM to create GM model of the robot (kinematic margin and self-collisions):

     $ cd ~/Sources/constraintsGM/build/bin
     $ ./robotModelGM

## References
[1] Dominik Belter, Efficient Modeling and Evaluation of Constraints in Path Planning for Multi-legged Walking Robots, 2019
