#!/bin/bash

# 
printf "Installing ConstraintsGM dependencies\n"

read -p "The following libraries are required. The script will download them and compile: coldet, libccd, FCL, QGLViewer
The following system dependencies are required: freeglut3-dev qt4-dev-tools libqglviewer-dev-qt4 libqglviewer2-qt4 openjdk-8-jdk pkg-config build-essential git cmake libeigen3-dev libboost-dev libboost-thread-dev libeigen3-dev
Do you want to install them [y/n]? " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]
then
    sudo apt-get install freeglut3-dev qt4-dev-tools libqglviewer-dev-qt4 libqglviewer2-qt4 openjdk-8-jdk pkg-config build-essential git cmake libeigen3-dev libboost-dev libboost-thread-dev libeigen3-dev
fi

source_dir=$(pwd)

# install Coldet
cd $source_dir
pwd
cd ~/Sources/walkers/3rdParty/coldet
make all
sudo mkdir /usr/local/include/coldet
sudo cp *.h /usr/local/include/coldet
sudo cp *.a /usr/local/lib

#install libccd
cd ~/Libs
git clone https://github.com/danfis/libccd
cd libccd
mkdir build
cd build
cmake -G "Unix Makefiles" -DENABLE_DOUBLE_PRECISION=OFF ..
make
#sudo make install

# install FCL
cd ~/Libs
git clone https://github.com/flexible-collision-library/fcl
cd fcl
mkdir build
cd build
cmake -DCCD_INCLUDE_DIR="~/Libs/libccd/src" -DCCD_LIBRARY="~/Libs/libccd/build/src/libccd.so" ..
make
#sudo make install

#compile walkers
cd ~/Sources
mkdir build-constraintsGM-Desktop-Default
cd build-constraintsGM-Desktop-Default
cmake ../constraintsGM
make
cd ~/Sources/constraintsGM/build/bin

printf "Done!\n"
printf "Go to ~/Sources/constraintsGM/build/bin and run examples (./demoVisualizer)\n"


#hacks
# locate XnPlatform.h
# sudo kate /usr/include/ni/XnPlatform.h
# change ... to #elif (__x86_64__)
# locate XnOS.h
# sudo kate /usr/include/ni/XnOS.h
