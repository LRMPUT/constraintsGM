//CPP File
#include "Utilities/observer.h"
#include <algorithm>
#include <iostream>
using namespace std;
void Subject::attach(Observer *observer){
    list.push_back(observer);
}

void Subject::detach(Observer *observer){
    list.erase(std::remove(list.begin(), list.end(), observer), list.end());
}

void Subject::notify(const std::vector<walkers::Mat34>& envState){
    for(vector<Observer*>::const_iterator iter = list.begin(); iter != list.end(); ++iter) {
        if(*iter != 0) {
            (*iter)->update(envState);
        }
    }
}

void Subject::notify(std::vector<walkers::Mat34>& envState){
    for(vector<Observer*>::const_iterator iter = list.begin(); iter != list.end(); ++iter)
    {
        if(*iter != 0) {
            (*iter)->update(envState);
        }
    }
}

void Subject::notify(std::vector<walkers::JointInfo>& envState){
    for(vector<Observer*>::const_iterator iter = list.begin(); iter != list.end(); ++iter)
    {
        if(*iter != 0) {
            (*iter)->update(envState);
        }
    }
}

void Subject::notify(std::vector<walkers::Mat34>& envState, std::vector<walkers::JointInfo>& envState2){
    for(vector<Observer*>::const_iterator iter = list.begin(); iter != list.end(); ++iter)
    {
        if(*iter != 0) {
            (*iter)->update(envState);
            (*iter)->update(envState2);
        }
    }
}

void Subject::notify(std::vector<walkers::ObjectInfo>& envState, std::vector<walkers::JointInfo>& envState2){
    for(vector<Observer*>::const_iterator iter = list.begin(); iter != list.end(); ++iter)
    {
        if(*iter != 0) {
            (*iter)->update(envState, envState2);
        }
    }
}

void Subject::notifySimTime(int simTime){
    for(vector<Observer*>::const_iterator iter = list.begin(); iter != list.end(); ++iter) {
        if(*iter != 0) {
            (*iter)->updateSimTime(simTime);
        }
    }
}

void Subject::add3DSModel(Objects3DS *objects3DS){
    for(vector<Observer*>::const_iterator iter = list.begin(); iter != list.end(); ++iter){
        if(*iter != 0) {
            (*iter)->init3DS(objects3DS);
        }
    }

}

void Subject::setShadowsVisible(bool shadowVisible) {
    for(vector<Observer*>::const_iterator iter = list.begin(); iter != list.end(); ++iter){
        if(*iter != 0) {
            (*iter)->setShadowsVisible(shadowVisible);
        }
    }

}

void Subject::updateKinematicModel(const walkers::Mat34& pose, const std::vector<double>& configuration){
    for(vector<Observer*>::const_iterator iter = list.begin(); iter != list.end(); ++iter){
        if(*iter != 0) {
            (*iter)->updateKinematicModel(pose, configuration);
        }
    }
}
