/**
* @author Dominik Belter
*
*/

#ifndef _ROBOTANYMAL_H_INCLUDED
#define _ROBOTANYMAL_H_INCLUDED

#include "robot.h"
#include <iostream>
#include "RobotModelMat/insectLeg.h"
#include "CollisionDetection/CollisionDetectionColdet.h"
#ifdef BUILD_WITH_FCL
#include "CollisionDetection/CollisionDetectionFCL.h"
#endif
//#include "Regression/PolynomialFitting.h"
#include <string>

namespace walkers {
    std::unique_ptr<Robot> createRobotAnymal(void);
    std::unique_ptr<Robot> createRobotAnymal(std::string configFilename);
    std::unique_ptr<Robot> createRobotAnymal(std::string configFilename, const std::string& _name);
}

///Class RobotAnymal includes all functions required for creating robot model

class RobotAnymal: public walkers::Robot
{
public:

    class ConfigFootholdSelection{
        public:
        double footWidth;
        double footLength;

        double alphaRS;
        double alphaSlopeForward;
        double alphaSlopeLateral;
        double alphaPitchCurvature;
        double alphaRollCurvature;
        double meanSlopeForward;
        double meanSlopeLateral;
        double meanRollCurvature;
        double meanPitchCurvature;

        double maxSlopeRoll;
        double minSlopeRoll;
        double maxSlopePitch;
        double minSlopePitch;
        double maxCurvRoll;
        double minCurvRoll;
        double maxCurvPitch;
        double minCurvPitch;
        double maxJerkRoll;
        double minJerkRoll;
        double maxJerkPitch;
        double minJerkPitch;
    };

    /// Pointer
    typedef std::unique_ptr<walkers::Robot> Ptr;
    RobotAnymal(void);
    RobotAnymal(std::string configFilename);
    RobotAnymal(std::string configFilename, const std::string& _name);

    void load3Dobjects(Objects3DS& _objects3DS);

    /// get kinematic objects
    std::vector<simulator::RenderObject> getObjectsToRender(const std::vector<double>& configuration) const;

    /// check collision for leg
    bool checkCollision(size_t legNo, const std::vector<double>& conf);

    /// check collision for the robot
    bool checkCollision(const std::vector<double>& conf, ConstraintType& constrViolated);

    /// check collision for two legs
    bool checkCollision(size_t leg1No, const std::vector<double>& conf1, size_t leg2No, const std::vector<double>& conf2);

    /// check collision for two legs
    bool checkCollision(size_t leg1No, const std::vector<double>& conf1, size_t leg2No, const std::vector<double>& conf2, ConstraintType& constrViolated);

    /// check collision for two legs
    bool checkCollisionGM(size_t leg1No, const std::vector<double>& conf1, size_t leg2No, const std::vector<double>& conf2, ConstraintType& constrViolated);

    /// check collision for the robot
    bool checkCollisionGM(const std::vector<double>& conf, ConstraintType& constrViolated);

    /// check collision for the single leg and neighbouring legs only
    bool checkCollisionNeighGM(size_t legNo, const std::vector<double>& currConf, const std::vector<double>& robotConf, ConstraintType& constrViolated);

    ~RobotAnymal(void);

private:
    void initializeModel(std::string configFilename);

    walkers::Mat34 neutralMotion; /// neutral position of the robot

    void getTokenizedContent(std::string content, walkers::Vec3& collectedData);

    ///collision detection
    std::string coldetConfig;
    std::string coldetType;
    std::unique_ptr<coldet::CollisionDetection> collisionChecker;

    bool model3dFlag;
    std::string model3dName;
    size_t model3dId;
    walkers::Vec3 model3dPosition;
    walkers::Vec3 model3dOrientation;
    std::vector<std::vector<size_t>> parts3Dids;
    std::vector<std::vector<walkers::Vec3>> partScales;
};

#endif


