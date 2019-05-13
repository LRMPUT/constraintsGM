#ifndef OBSERVER_H_
#define OBSERVER_H_

#include "Defs/simulator_defs.h"
#include "Utilities/objects3DS.h"
#include <vector>
#include <list>
#include <unordered_map>

class Observer {
public:

    virtual void update(const std::vector<walkers::Mat34>& envState) = 0;

    virtual void update(std::vector<walkers::Mat34>& envState) = 0;
    virtual void update(std::vector<walkers::JointInfo>& envState) = 0;
    virtual void update(std::vector<walkers::Mat34>& envState, std::vector<walkers::JointInfo>& envState2) = 0;
    virtual void update(std::vector<walkers::ObjectInfo>& envState, std::vector<walkers::JointInfo>& envState2) = 0;

    virtual void updateSimTime(int simTime) = 0;
    virtual void init3DS(Objects3DS* objects3DS) = 0;
    virtual void setShadowsVisible(bool shadowVisible) = 0;
    virtual void updateKinematicModel(const walkers::Mat34& pose, const std::vector<double>& configuration) = 0;
};

class Subject
{
    //Lets keep a track of all the shops we have observing
    std::vector<Observer*> list;

public:
    void attach(Observer *observer);
    void detach(Observer *observer);

    void notify(const std::vector<walkers::Mat34>& envState);

    void notify(std::vector<walkers::Mat34>& envState);
    void notify(std::vector<walkers::JointInfo>& envState);
    void notify(std::vector<walkers::Mat34>& envState, std::vector<walkers::JointInfo>& envState2);
    void notify(std::vector<walkers::ObjectInfo>& envState, std::vector<walkers::JointInfo>& envState2);

    void notifySimTime(int simTime);
    void add3DSModel(Objects3DS* objects3DS);
    void setShadowsVisible(bool shadowVisible);
    void updateKinematicModel(const walkers::Mat34& pose, const std::vector<double>& configuration);
};

#endif // OBSERVER_H_
