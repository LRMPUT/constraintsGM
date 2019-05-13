/** @file robot.h
 * Robot interface
 * Dominik Belter
 */

#include "Defs/defs.h"
#include "RobotModelMat/robot.h"
#include "3rdParty/tinyXML/tinyxml2.h"
#include "Optimizer/cmaes.h"
#include "Optimizer/pso.h"
#include <iostream>
#include <string>
#include <vector>
#include <stdexcept>
#include <functional>

using namespace walkers;

/// overloaded constructor
Robot::Robot(const std::string _name, Type _type) : name(_name), type(_type) {
}

///Compute configuration of the leg for the reference motion of the platform
/**
* @param motion - specified motion
* @return reference values for servomotors
*/
std::vector<double> Robot::computeLegConfiguration(int legNo, const walkers::Mat34& bodyMotion, const std::vector<double>& startConfiguration, bool& isMotionPossible) {
    walkers::Mat34 newLegMountPoint = bodyMotion * legMountPoints[legNo];
    walkers::Mat34 currentFootPosition;

    currentFootPosition = legMountPoints[legNo] * legModels[legNo]->forwardKinematic(startConfiguration, legModels[legNo]->getJointsNo()).matrix();

    walkers::Mat34 footInMount;
    footInMount.matrix() = newLegMountPoint.matrix().inverse() * currentFootPosition.matrix();

    std::vector<double> conf;
    conf = legModels[legNo]->inverseKinematic(footInMount, isMotionPossible, legModels[legNo]->getJointsNo());

    checkReferenceAngles(conf);
    return conf;
}

///Compute configuration of the leg for the pose defined in the platform coordinates
/**
* @param motion - specified pose
* @return reference values for servomotors
*/
std::vector<double> Robot::computeLegConfiguration(int legNo, const walkers::Mat34& pose, bool& isMotionPossible) {
    walkers::Mat34 footInMount;
    footInMount.matrix() = legMountPoints[legNo].matrix().inverse() * pose.matrix();
    std::vector<double> conf;
    conf = legModels[legNo]->inverseKinematic(footInMount, isMotionPossible, legModels[legNo]->getJointsNo());
    checkReferenceAngles(conf);
    return conf;
}

/// new method: computes forward kinematics for each leg and returns position of each link of the robot (body is [0,0,0]^T)
//virtual std::vector<walkers::Mat34> computeLinksPosition(const std::vector<double>& configuration) = 0;
/// new method: computes forward kinematics for each leg and returns position of each link of the robot (body is [0,0,0]^T)
std::vector<walkers::Mat34> Robot::computeLinksPosition(const std::vector<double>& configuration)
{

    std::vector<walkers::Mat34> linksPos;
    std::vector<double> conf;

    //-----------------------------------------
    size_t legNo=0;
    for (size_t h = 0; h<configuration.size(); h=h + legModels[legNo]->getJointsNo()) {
        for (int j = 0; j<legModels[legNo]->getJointsNo(); j++) {
            conf.push_back(configuration[j + h]);
        }
        if (h<(size_t)legsNo*legModels[legNo]->getJointsNo()/2)
        {
            for (int i = 0; i<legModels[legNo]->getJointsNo(); i++)
            {
                linksPos.push_back(legMountPoints[h / legModels[legNo]->getJointsNo()] * legModels[legNo]->forwardKinematic(conf, i,0));
            }
        }
        else {
             for (int i = 0; i<legModels[legNo]->getJointsNo(); i++){
                 linksPos.push_back(legMountPoints[h / legModels[legNo]->getJointsNo()] * legModels[legNo]->forwardKinematic(conf, i,1));
             }
        }
        if (h<(size_t)legsNo*legModels[legNo]->getJointsNo()/2){
            linksPos.push_back(legMountPoints[h /legModels[legNo]->getJointsNo()] *legModels[legNo]->forwardKinematic(conf, -1,0));
        }
        else
            linksPos.push_back(legMountPoints[h / legModels[legNo]->getJointsNo()] * legModels[legNo]->forwardKinematic(conf, -1,1));
        for (int k = 0; k<legModels[legNo]->getJointsNo(); k++) {
            conf.pop_back();
        }
        legNo++;
    }
    //-------------------------------------------

    return linksPos;
}

//virtual walkers::Mat34 legCPos(const std::vector<double>& configuration, int legNo) = 0;
walkers::Mat34 Robot::legCPos(const std::vector<double>& configuration, int legNo)
{
    walkers::Mat34 currentFootPosition;
    walkers::Mat34 startFootPosition;
    walkers::Mat34 returnFootPosition;
    walkers::Mat34 temp;

    int side = 0;//(legNo<3) ? 0 : 1;

    std::vector<double> currConf(configurationStart.begin()+legModels[legNo]->getJointsNo()*legNo,configurationStart.begin()+legModels[legNo]->getJointsNo()*legNo+legModels[legNo]->getJointsNo());
    startFootPosition.matrix() = legMountPoints[legNo] * legModels[legNo]->forwardKinematic(currConf, legModels[legNo]->getJointsNo(), side).matrix();
    currentFootPosition.matrix() = legModels[legNo]->forwardKinematic(configuration, legModels[legNo]->getJointsNo(), side).matrix();

    for(int i=0;i<4;i++){
        for(int j=0;j<3;j++) {
            currentFootPosition(i,j) = startFootPosition(i,j);
        }
    }

    temp.matrix() = startFootPosition.matrix() * currentFootPosition.matrix().inverse();

    returnFootPosition.matrix() = temp.matrix() * legMountPoints[legNo].matrix().inverse();

    return returnFootPosition;
}

/// returns foot pose in Leg coordinate system
walkers::Mat34 Robot::getLegMountPoint(size_t legNo){
    if (legNo>=(size_t)legsNo){
        throw std::runtime_error("getLegMountPoint: incorrect number of leg.\n");
    }
    return legMountPoints[legNo];
}

/// returns neutral position of the foot
//virtual walkers::Mat34 footPose(const walkers::Mat34& robotPose, int legNo) = 0;
/// returns neutral position of the foot
walkers::Mat34 Robot::footPose(const walkers::Mat34& robotPose, int legNo){
    std::vector<double> currConf(configurationStart.begin()+legModels[legNo]->getJointsNo()*legNo,configurationStart.begin()+legModels[legNo]->getJointsNo()*legNo+legModels[legNo]->getJointsNo());
    int side = 0;//(legNo<3) ? 0 : 1;
    return robotPose * legMountPoints[legNo] * legModels[legNo]->forwardKinematic(currConf, legModels[legNo]->getJointsNo(), side);
}

/// returns current position of the foot
//virtual walkers::Mat34 footPose(const walkers::Mat34& robotPose, const std::vector<double>& conf, int legNo) = 0;
walkers::Mat34 Robot::footPose(const walkers::Mat34& robotPose, const std::vector<double>& conf, int legNo){
    int side = 0;//(legNo<3) ? 0 : 1;
    return robotPose * legMountPoints[legNo] * legModels[legNo]->forwardKinematic(conf, 3, side);
}

/// compute area of the trianlge
double Robot::triangleArea2D(const Mat34& vertA, const Mat34& vertB, const Mat34& vertC) const{
    return fabs(0.5*(((vertA(0,3)-vertC(0,3))*(vertB(1,3)-vertA(1,3)))-((vertA(0,3)-vertB(0,3))*(vertC(1,3)-vertA(1,3)))));
}

/// check reference values
void Robot::checkReferenceAngles(std::vector<double>& refValues) const{
    for (auto & angle : refValues){
        if (angle>M_PI){
            angle-=2*M_PI;
        }
        else if (angle<-M_PI){
            angle+=2*M_PI;
        }
    }
}

/// compute fitness
double Robot::checkLegsMotion(const Mat34& bodyMotion, const walkers::RobotState& currentState, const walkers::RobotState& prevState, const std::vector<size_t>& movingLegs){
    double error(0);
    size_t jointsInLeg = prevState.jointsNo/prevState.legsNo;
    for (const auto& legNo : movingLegs){
        std::vector<double> prevConf = {prevState.currentValues[legNo*jointsInLeg], prevState.currentValues[legNo*jointsInLeg+1], prevState.currentValues[legNo*jointsInLeg+2]};
        bool isMotionPossible;
        std::vector<double> currentConf = computeLegConfiguration((int)legNo,bodyMotion,prevConf,isMotionPossible);
        if (!isMotionPossible)
            return std::numeric_limits<double>::max();
        for (size_t jointNo=0;jointNo<jointsInLeg;jointNo++){
            error+=pow(currentConf[jointNo] - currentState.currentValues[legNo*jointsInLeg+jointNo],2.0);
        }
    }
    return error;
}

/// compute inverse kinematic
std::vector<double> Robot::inverseKinematicGlobal(bool& isMotionPossible, const walkers::Mat34& robotPose, const std::vector<walkers::Mat34>& feetPoses) const{
    if (feetPoses.size()!=getLegsNo())
        throw std::runtime_error("inverseKinematicGlobal: Incorrect number of legs\n");
    std::vector<double> robotConf;
    for (size_t legNo=0;legNo<feetPoses.size();legNo++){
        std::vector<double> legConf = legModels[legNo]->inverseKinematic((robotPose*legMountPoints[legNo]).inverse()*feetPoses[legNo], isMotionPossible, -1);
        if (!isMotionPossible)
            return robotConf;
        robotConf.insert(robotConf.end(), legConf.begin(), legConf.end());
    }
    return robotConf;
}

/// compute kinematic margin for the leg (considers workspace and collisions)
double Robot::kinematicMargin(size_t legNo, const walkers::Mat34& footPose){
    bool isMotionPossible;
    std::vector<double> conf = inverseKinematicLeg(legNo,isMotionPossible,footPose);
    if (!isMotionPossible)
        return -1;
    else
        return kinematicMargin(legNo,conf);
}

/// compute distance to workspace for the leg (considers workspace and collisions)
double Robot::distance2workspace(size_t legNo, const walkers::Mat34& footPose){
    stats.dist2workspaceNo++;
    bool isMotionPossible;
    std::vector<double> conf = inverseKinematicLeg(legNo,isMotionPossible,footPose);
    walkers::Mat34 footPoseInit(footPose);
    if (!isMotionPossible) {
        double r=0;
        double increment(0.0025);
        footPoseInit.matrix().block<3,3>(0,0) = Mat33::Identity();
        walkers::Mat34 currFootPose(footPoseInit);
        bool isOK(true);
        double maxCheckDist = 0.6;
        while (isOK){
            r+=increment;
            if (r>maxCheckDist){
                return 1.0;
            }
//            std::vector<double> angle={-(3.0*M_PI)/4.0, -M_PI/2.0, - M_PI/4.0, 0, M_PI/4.0, M_PI/2.0, (3.0*M_PI)/4.0, M_PI};
            std::vector<double> angle={-(7.0*M_PI)/8.0, -(6*M_PI)/8.0, -(5*M_PI)/8.0, -(4*M_PI)/8.0, -(3*M_PI)/8.0, -(2*M_PI)/8.0, -(1*M_PI)/8.0, 0, (1*M_PI)/8.0, (2*M_PI)/8.0, (3*M_PI)/8.0, (4*M_PI)/8.0, (5*M_PI)/8.0, (6*M_PI)/8.0, (7*M_PI)/8.0, M_PI};
            for (const auto& roll : angle){
                for (const auto& pitch : angle){
                    for (const auto& yaw : angle){
                        Mat34 motion(Mat34::Identity());
                        motion(0,3) = r;
                        Vec3 rpy(roll, pitch, yaw);
                        Mat34 rot(walkers::Mat34::Identity());
                        rot.matrix().block<3,3>(0,0) = toRotationMat(rpy);
                        currFootPose = footPoseInit*(rot*motion);
                        if (isInsideWorkspaceLeg((int)legNo,currFootPose)){
                            return r-increment;
                        }
                    }
                }
            }
        }
        return r;
    }
    else
        return -1;
}

/// compute kinematic margin for the leg (considers workspace and collisions)
double Robot::kinematicMargin(size_t legNo, const std::vector<double>& conf){
    stats.kinemMargCheckNo++;
    if (conf.size()!=getLegJointsNo(legNo))
        throw std::runtime_error("Kinematic Margin leg: Incorrect number of ref angles\n");
    double r=0;
    double increment(0.0025);
    walkers::Mat34 footPose = forwardKinematic(legNo, conf);
    footPose.matrix().block<3,3>(0,0) = Mat33::Identity();
    walkers::Mat34 currFootPose(footPose);
    std::vector<double> currConf(conf);
    bool isOK(true);
    if (isInsideWorkspaceLeg((int)legNo,currFootPose)){
//        if(!checkCollisionGM(legNo,currConf)){
//        if(!checkCollision(legNo,currConf)){
            isOK = true;
//        }
//        else{
//            isOK = false;
//        }
    }
    else{
        isOK = false;
    }
    if (!isOK)
        return -1.0;
    while (isOK){
        r+=increment;
        std::vector<double> angle={-M_PI, -(3.0*M_PI)/4.0, -M_PI/2.0, - M_PI/4.0, 0, M_PI/4.0, M_PI/2.0, (3.0*M_PI)/4.0, M_PI};
        for (const auto& roll : angle){
            for (const auto& pitch : angle){
                for (const auto& yaw : angle){
                    Mat34 motion(Mat34::Identity());
                    motion(0,3) = r;
                    Vec3 rpy(roll, pitch, yaw);
                    Mat34 rot(walkers::Mat34::Identity());
                    rot.matrix().block<3,3>(0,0) = toRotationMat(rpy);
                    currFootPose = footPose*(rot*motion);
                    if (!isInsideWorkspaceLeg((int)legNo,currFootPose)){
                        return r-increment;
                    }
                    else{
                        bool isMotionPossible;
                        currConf = inverseKinematicLeg(legNo, isMotionPossible,currFootPose);
                        if (!isMotionPossible)
                            return r-increment;
//                        if (checkCollision(legNo,currConf)){// extremely slow (if you'd like to consired collisions when computing kienmatic margin uncomment this code)
//                        if (checkCollisionGM(legNo,currConf)){// extremely slow (if you'd like to consired collisions when computing kienmatic margin uncomment this code)
//                            return r-increment;
//                        }
                    }
                }
            }
        }
    }
    return r;
}

/// compute kinematic margin for the robot (considers workspace and collisions)
double Robot::kinematicMargin(const std::vector<double>& conf){
    if (conf.size()!=getLegJointsNo(0)*getLegsNo())
        throw std::runtime_error("Kinematic Margin robot: Incorrect number of ref angles\n");
    double min = std::numeric_limits<double>::max();
    size_t jointNums=0;
    for (size_t legNo=0; legNo<getLegsNo();legNo++){
        std::vector<double> currentConf(conf.begin()+jointNums, conf.begin()+jointNums+getLegJointsNo(legNo));
        double margin = kinematicMargin(legNo, currentConf);
        if (margin<min)
            min = margin;
        jointNums+=getLegJointsNo(legNo);
    }
    return min;
}

/// check collision for the robot
bool Robot::checkCollision(const std::vector<double>& conf){
    stats.collCheckFCLNo++;
    if (conf.size()!=getLegJointsNo(0)*getLegsNo())
        throw std::runtime_error("Kinematic Margin robot (GM): Incorrect number of ref angles\n");
    std::vector<bool> collisionTable;
    std::vector<simulator::RenderObject> kinemObjects = getObjectsToRender(conf);
    return collisionChecker->checkCollision(kinemObjects,collisionTable);
}

double Robot::checkLegsMotionOpt(const Eigen::VectorXd& x){
    Vec3 rpy(x(3),x(4),x(5));
    Mat34 bodyMotion;
    bodyMotion.matrix().block<3,3>(0,0) = toRotationMat(rpy);
    bodyMotion(0,3) = x(0); bodyMotion(1,3) = x(1); bodyMotion(2,3) = x(2);
    return checkLegsMotion(bodyMotion, currentStateOpt, prevStateOpt, movingLegsOpt);
}

/// is robot reference legs poses inside robot workspace (global coord)
bool Robot::isInsideWorkspaceGlobal(const walkers::Mat34& robotPose, const std::vector<walkers::Mat34>& feetPoses){
    int legNo = 0;
    for (const auto& foot : feetPoses){
        if (!isInsideWorkspaceGlobal(legNo, robotPose, foot))
            return false;
        legNo++;
    }
    return true;
}

/// compute forward kinematic for the robot
std::vector<Mat34> Robot::forwardKinematic(const std::vector<double>& conf) const{
    size_t jointsCnt=0;
    std::vector<Mat34> feetPos;
    for (size_t legNo=0;legNo<(size_t)legsNo;legNo++){
        std::vector<double> confLegs = std::vector<double>(conf.begin()+jointsCnt,conf.begin()+jointsCnt+legModels[legNo]->getJointsNo());
        feetPos.push_back(legMountPoints[legNo]*forwardKinematic(legNo, confLegs));
        jointsCnt+=legModels[legNo]->getJointsNo();
    }
    return feetPos;
}

/// clear stats
void Robot::clearStats(void){
    stats.reset();
}

/// get statistics
Robot::Statistics Robot::getStats(void){
    return stats;
}
