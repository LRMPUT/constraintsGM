/**
* @author Dominik Belter
* @author Maciej Okoniewski
* @author Emil Waledziak
* @author Jerzy Wiatrow
* @author Marcin Zgolinski
* 
*/

#ifndef _ROBOTMESSOR_H_INCLUDED
#define _ROBOTMESSOR_H_INCLUDED

#include "robot.h"
#include "Utilities/objects3DS.h"
#include <iostream>
#include "RobotModelMat/insectLeg.h"
#include "Optimizer/pso.h"
#include "Optimizer/cmaes.h"
#include "Defs/simulator_defs.h"
#include "CollisionDetection/CollisionDetectionColdet.h"
#ifdef BUILD_WITH_FCL
#include "CollisionDetection/CollisionDetectionFCL.h"
#endif

namespace walkers {
    std::unique_ptr<Robot> createRobotMessor(void);
    std::unique_ptr<Robot> createRobotMessor(std::string configFilename);
}

///Class RobotMessor includes all functions required for creating robot model

class RobotMessor: public walkers::Robot
{
public:

    /// Pointer
    typedef std::unique_ptr<walkers::Robot> Ptr;
    RobotMessor(void);
    RobotMessor(std::string configFilename);

    void load3Dobjects(Objects3DS& _objects3DS);
    /// get kinematic objects
    std::vector<simulator::RenderObject> getObjectsToRender(const std::vector<double>& configuration) const;

    /// check collision for leg
    bool checkCollision(size_t legNo, const std::vector<double>& conf);

    /// check collision for two legs
    bool checkCollision(size_t leg1No, const std::vector<double>& conf1, size_t leg2No, const std::vector<double>& conf2);

    /// check collision for the robot
    bool checkCollision(const std::vector<double>& conf, ConstraintType& constrViolated);

    /// check collision for two legs
    bool checkCollision(size_t leg1No, const std::vector<double>& conf1, size_t leg2No, const std::vector<double>& conf2, ConstraintType& constrViolated);

    ~RobotMessor(void);

private:

    walkers::Mat34 neutralMotion; /// neutral position of the robot

    ///Model 3D parameters
    std::string model3dName;
    bool model3dFlag;
    size_t model3dId;
    walkers::Vec3 model3dPosition;
    walkers::Vec3 model3dOrientation;
    std::vector<std::vector<size_t>> parts3Dids;
    ///collision detection
    std::string coldetConfig;
    std::string coldetType;
    std::unique_ptr<coldet::CollisionDetection> collisionChecker;
};

#endif


