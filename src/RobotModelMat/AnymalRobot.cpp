#include "RobotModelMat/AnymalRobot.h"
#include <iostream>
#include "RobotModelMat/insectLeg.h"
#include "math.h"

using namespace walkers;

RobotAnymal::Ptr robotAnymal;

RobotAnymal::RobotAnymal(void) : Robot("Anymal", TYPE_ANYMAL){
}

RobotAnymal::RobotAnymal(std::string configFilename) : walkers::Robot("RobotAnymal", TYPE_ANYMAL) {
    initializeModel(configFilename);
}

RobotAnymal::RobotAnymal(std::string configFilename, const std::string& _name) : walkers::Robot(_name, TYPE_ANYMAL) {
    initializeModel(configFilename);
}

void RobotAnymal::initializeModel(std::string configFilename){
    setlocale(LC_NUMERIC,"C");
    tinyxml2::XMLDocument config;
    std::string filename = "../../resources/" + configFilename;
//        size_t found = filename.find_last_of("/\\");
//        std::string prefix = filename.substr(0,found+1);
    config.LoadFile(filename.c_str());
    if (config.ErrorID()){
        std::cout << "unable to load Anymal Robot config file: " << filename << "\n";
    }
    else {
        setlocale(LC_NUMERIC,"C");
        std::string legModelConfig;
        tinyxml2::XMLElement* robot = config.FirstChildElement("robot");

        legsNo = robot->IntAttribute("legs_no");
        legModels.resize(legsNo);
        feetNeutralPosition.resize(legsNo);
        int counter = 0;
        if (legsNo>0){
            for( const tinyxml2::XMLElement* leg = robot->FirstChildElement("leg"); leg; leg = leg->NextSiblingElement("leg")){
                std::string legXMLAttachmentParameters;
                legXMLAttachmentParameters = leg->FirstChildElement("attach")->Attribute("xyz");
                walkers::Vec3 position;
                getTokenizedContent(legXMLAttachmentParameters, position);
                walkers::Mat34 pose(walkers::Mat34::Identity());
                pose.matrix().block<3,1>(0,3) = position.vector();
                legXMLAttachmentParameters = leg->FirstChildElement("attach")->Attribute("rpy");
                walkers::Vec3 rpy;
                walkers::getTokenizedContent(legXMLAttachmentParameters, rpy);
                pose.matrix().block<3,3>(0,0) = walkers::toRotationMat(rpy);

//                std::cout << legMountPoints.size() << " \n" << pose.matrix() << "\n";
                legMountPoints.push_back(pose);
                std::string XMLinitConf = leg->FirstChildElement("attach")->Attribute("initConfiguration");

                legModelConfig = leg->Attribute("config");
                getTokenizedContent(XMLinitConf, position);
                for (int i=0;i<3;i++)
                    configurationStart.push_back(position.vector()(i));
                legModels[counter] = controller::createInsectLeg("../../resources/"+legModelConfig);
                feetNeutralPosition[counter] = legModels[counter]->forwardKinematic(std::vector<double>(configurationStart.begin()+counter*3,configurationStart.begin()+counter*3+3));
                counter++;
                if (counter == legsNo)
                    break;
            }
        }

        //mass

        tinyxml2::XMLElement* inertial = robot->FirstChildElement("robot_body")->FirstChildElement("inertial");
        massBody=inertial->FirstChildElement("mass")->DoubleAttribute("value");
        //visualizations
        tinyxml2::XMLElement* robotBodyVisual = robot->FirstChildElement("robot_body")->FirstChildElement("visualization");
        if(robotBodyVisual != NULL){
            std::vector<std::string> bodyXMLVisualizationParameters;
            std::vector<walkers::Vec3> bodyVisualization;

            model3dName = robotBodyVisual->FirstChildElement("model3ds")->Attribute("filename");
            bodyXMLVisualizationParameters.push_back(robotBodyVisual->FirstChildElement("model3ds")->Attribute("xyz"));
            bodyXMLVisualizationParameters.push_back(robotBodyVisual->FirstChildElement("model3ds")->Attribute("scale"));
            bodyXMLVisualizationParameters.push_back(robotBodyVisual->FirstChildElement("model3ds")->Attribute("rpy"));
            walkers::getTokenizedContent(bodyXMLVisualizationParameters, bodyVisualization);

            model3dPosition = walkers::Vec3(bodyVisualization.at(0).x(), bodyVisualization.at(0).z(), bodyVisualization.at(0).y());
            model3dScale = bodyVisualization.at(1);
            model3dOrientation = bodyVisualization.at(2);
        }

        // collision detection
        coldetConfig = robot->FirstChildElement( "CollisionDetection" )->FirstChildElement("config")->GetText();
        coldetType = robot->FirstChildElement( "CollisionDetection" )->FirstChildElement("type")->GetText();
        model3dFlag = true;
    }
    neutralMotion.setIdentity();
    neutralMotion(2, 3) = 0.12;

    configurationCurr=configurationStart;

    if (coldetType=="Coldet")
        collisionChecker =  coldet::createCollisionDetectionColdet(coldetConfig);
#ifdef BUILD_WITH_FCL
    else if (coldetType=="FCL")
        collisionChecker =  coldet::createCollisionDetectionFCL(coldetConfig);
#endif
    else
        collisionChecker =  coldet::createCollisionDetectionColdet(coldetConfig);

    size_t elementsNo = getLegsNo()*getLegJointsNo(0)+1;
    size_t modelsNo = getLegJointsNo(0)+1;
    std::vector<walkers::Vec3> scales;
    scales.push_back(getModelScale());
    for (size_t linkNo=0; linkNo<getLegLinksNo(0);linkNo++)
        scales.push_back(getLegModelScale(0, linkNo));
    Objects3DS objects3DS;
    load3Dobjects(objects3DS);
//        initKinematicModel(objects3DS);
    collisionChecker->initializeMeshModelWalker(objects3DS, modelsNo, elementsNo, scales);
}

RobotAnymal::~RobotAnymal(void) {

}

void RobotAnymal::getTokenizedContent(std::string content, Vec3& collectedData){
    std::stringstream str(content);
    std::string token;
    Vec3 parameters;
    for (int j = 0; j < 3; j++){
        str >> token;
        parameters.vector()(j) = std::stof(token);
    }
    collectedData = parameters;
}

void RobotAnymal::load3Dobjects(Objects3DS& _objects3DS){
    if(model3dFlag) {
        size_t object3dsId = _objects3DS.ObjLoad(model3dName);
        if(object3dsId)
            model3dId = object3dsId-1;
        else
            std::cout << "Could not load " << model3dName << "\n";
        parts3Dids.resize(legsNo);
        partScales.resize(legsNo);
        for(int i=0; i<legsNo; ++i) {
            std::vector<std::string> model3dNames;
            ((InsectLeg*)legModels[i].get())->get3DmodelNames(model3dNames);/// assume that all legs are the same
            parts3Dids[i].resize(model3dNames.size());
            std::vector<walkers::Vec3> model3dScales;
            partScales[i].resize(model3dNames.size());
            ((InsectLeg*)legModels[i].get())->get3DmodelScales(model3dScales);
            int linkNo=0;
            for(const auto& linkName : model3dNames){
                size_t obj3dsId = _objects3DS.ObjLoad(linkName);
                if(object3dsId){
                    parts3Dids[i][linkNo] = obj3dsId-1;
                    partScales[i] = model3dScales;
                }
                else
                    std::cout << "Could not load " << linkName << "\n";
                linkNo++;
            }
        }
    }
}

/// get kinematic objects
std::vector<simulator::RenderObject> RobotAnymal::getObjectsToRender(const std::vector<double>& configuration) const{
    std::vector<simulator::RenderObject> objects;
    size_t objNo=0;
    if(model3dFlag){
        simulator::RenderObject model;
        model.type = simulator::RenderObjectType::MODEL3D;
        Mat34 trans(Mat34::Identity());
        trans.matrix().block<3,3>(0,0) = walkers::toRotationMat(Vec3(0,0,-1.57));
        model.mat = trans;
        model.id = model3dId;
        model.scaleX = model3dScale.x();
        model.scaleY = model3dScale.y();
        model.scaleZ = model3dScale.z();
        model.mass = massBody;
        objects.push_back(model);
        objNo++;
    }
    std::vector<walkers::Mat34> model3dPoses(3,Mat34::Identity());
    walkers::Vec3 rpy0(0,0,0);
    model3dPoses[0].matrix().block<3,3>(0,0) = walkers::toRotationMat(rpy0);
    model3dPoses[0](0,3)=0.0;
    model3dPoses[0](2,3)=0.0;

    walkers::Vec3 rpy1(0,0,0);
    model3dPoses[1].matrix().block<3,3>(0,0) = walkers::toRotationMat(rpy1);
    model3dPoses[1](0,3)=0.0;
    model3dPoses[1](2,3)=0.0;

    walkers::Vec3 rpy2(0,0,0);
    model3dPoses[2].matrix().block<3,3>(0,0) = walkers::toRotationMat(rpy2);
    model3dPoses[2](0,3)=0.0;
    model3dPoses[2](2,3)=0.0;
//    ((InsectLeg*) legModel.get())->get3DmodelPoses(model3dPoses);

    std::vector<Mat34> transDH(3,Mat34::Identity());
    std::vector<Mat34> rotDH(3,Mat34::Identity());

    transDH[0](0,3)=0.0;

    if (name=="RobotAnymal"){//todo parameters in the xml file
        transDH[1](0,3)=0.0;
        transDH[1](1,3)=0.062;
        transDH[1](2,3)=0.0;

        transDH[2](0,3)=0.25;
        transDH[2](1,3)=0.14;
        transDH[2](2,3)=0.0;
    }
    else if (name=="RobotAnymal_C"){
        transDH[1](0,3)=0.0;
        transDH[1](1,3)=0.082;
        transDH[1](2,3)=0.0;

        transDH[2](0,3)=0.29;
        transDH[2](1,3)=0.12;
        transDH[2](2,3)=0.0;
    }

    for(int i=0; i<legsNo; ++i){
        std::vector<double> conf = {configuration[i*legModels[i]->getJointsNo()],configuration[i*legModels[i]->getJointsNo()+1],configuration[i*legModels[i]->getJointsNo()+2]};

        std::vector<std::string> model3dNames;
        ((InsectLeg*)legModels[i].get())->get3DmodelNames(model3dNames);
        if (i==0||i==2){
            transDH[0](1,3) = fabs(transDH[0](1,3));
            transDH[1](1,3) = fabs(transDH[1](1,3));
            transDH[2](1,3) = fabs(transDH[2](1,3));
        }
        else{
            transDH[0](1,3) = -fabs(transDH[0](1,3));
            transDH[1](1,3) = -fabs(transDH[1](1,3));
            transDH[2](1,3) = -fabs(transDH[2](1,3));
        }
        for(size_t j=0; j<3; ++j){
            simulator::RenderObject model;
            model.type = simulator::RenderObjectType::MODEL3D;
            Mat34 trans(Mat34::Identity());
            rotDH[0].matrix().block<3,3>(0,0) = walkers::toRotationMat(Vec3(0,0,conf[0]));
            rotDH[1].matrix().block<3,3>(0,0) = walkers::toRotationMat(Vec3(0,-conf[1],0));
            rotDH[2].matrix().block<3,3>(0,0) = walkers::toRotationMat(Vec3(0,-conf[2],0));
            for (size_t jointNo=0;jointNo<j+1;jointNo++){
                trans = trans*transDH[jointNo]*rotDH[jointNo];
            }
            model.mat = legMountPoints[i]*trans*model3dPoses[j];
            model.id = parts3Dids[i][j];
            model.scaleX = partScales[i][j].x();
            model.scaleY = partScales[i][j].y();
            model.scaleZ = partScales[i][j].z();
            model.mass = legModels[i]->getLinkMass(j);
            objects.push_back(model);
            objNo++;
        }
//        if(legs[i]->config.foot.isFoot&&legs[i]->config.foot.model3Dflag){//render foot
//            RenderObject model;
//            model.type = RenderObjectType::MODEL3D;
//            PxTransform trans = legs[i]->foot->getGlobalPose();
//            trans.q = trans.q * orientationToQuat(legs[i]->config.foot.model3dOrientation);
//            trans.p = trans.transform(legs[i]->config.foot.model3dPosition);
//            PxMat44 shapePose(trans);
//            model.mat = PxMat44ToMat34(shapePose);
//            model.id = legs[i]->config.foot.model3dId;
//            model.scaleX = legs[i]->config.foot.model3dScale.x;
//            model.scaleY = legs[i]->config.foot.model3dScale.y;
//            model.scaleZ = legs[i]->config.foot.model3dScale.z;
//            objects.push_back(model);
//        }
    }
    return objects;
}

/// check collision for leg
bool RobotAnymal::checkCollision(size_t legNo, const std::vector<double>& conf){
    if (conf.size()!=getLegJointsNo(legNo))
        throw std::runtime_error("Check collision: Incorrect number of ref angles\n");
    std::vector<double> configuration(getLegJointsNo(0)*getLegsNo(),0);
    configuration[legNo*3+0] = conf[0]; configuration[legNo*3+1] = conf[1]; configuration[legNo*3+2] = conf[2];

    if (legNo==0) {
        configuration[1*3+0] = 1.57;
        configuration[2*3+0] = -1.57;
        configuration[3*3+0] = 1.57;
    }
    if (legNo==1) {
        configuration[0*3+0] = -1.57;
        configuration[2*3+0] = -1.57;
        configuration[3*3+0] = 1.57;
    }
    if (legNo==2) {
        configuration[0*3+0] = -1.57;
        configuration[1*3+0] = 1.57;
        configuration[3*3+0] = 1.57;
    }
    if (legNo==3) {
        configuration[0*3+0] = -1.57;
        configuration[1*3+0] = 1.57;
        configuration[2*3+0] = -1.57;
    }
    std::vector<bool> collisionTable;
    std::vector<simulator::RenderObject> kinemObjects = getObjectsToRender(configuration);
    std::set<int> objects2check = {0, int(legNo*getLegJointsNo(0)+1), int(legNo*getLegJointsNo(0)+2), int(legNo*getLegJointsNo(0)+3)};
    return collisionChecker->checkCollision(kinemObjects, objects2check, collisionTable);
}

/// check collision for the robot
bool RobotAnymal::checkCollision(const std::vector<double>& conf, ConstraintType& constrViolated){
    std::vector<bool> collisionTable;
    std::vector<simulator::RenderObject> kinemObjects = getObjectsToRender(conf);
    if (collisionChecker->checkCollision(kinemObjects,collisionTable)){
        constrViolated = CONSTRAINT_SELF_COLLISIONS;
        return true;
    }
    else{
        constrViolated = CONSTRAINT_NO;
        return false;
    }
}

/// check collision for two legs
bool RobotAnymal::checkCollision(size_t leg1No, const std::vector<double>& conf1, size_t leg2No, const std::vector<double>& conf2, ConstraintType& constrViolated){
    if (conf1.size()!=getLegJointsNo(leg1No)||conf2.size()!=getLegJointsNo(leg2No))
        throw std::runtime_error("Check collision: (2 legs GM) Incorrect number of ref angles\n");
    if (legModels[leg1No]->isInsideWorkspace(conf1)&&legModels[leg2No]->isInsideWorkspace(conf2)){
        if ((leg1No==0&&leg2No==1)||(leg1No==1&&leg2No==2)||(leg1No==3&&leg2No==4)||(leg1No==4&&leg2No==5)){
            std::vector<bool> collisionTable;
            std::vector<double >configuration(legsNo*3,0);
            for (size_t jointNo=0; jointNo<conf1.size(); jointNo++){
                configuration[leg1No+3+0] = conf1[jointNo];
                configuration[leg2No+3+0] = conf2[jointNo];
            }
            std::vector<simulator::RenderObject> kinemObjects = getObjectsToRender(configuration);
            return collisionChecker->checkCollision(kinemObjects,collisionTable);
        }
        else
            throw std::runtime_error("No coll model\n");
    }
    else{
        constrViolated = CONSTRAINT_WORKSPACE;
        return true;
    }
    constrViolated = CONSTRAINT_NO;
    return false;
}

/// check collision for two legs
bool RobotAnymal::checkCollision(size_t leg1No, const std::vector<double>& conf1, size_t leg2No, const std::vector<double>& conf2){
    if (conf1.size()!=getLegJointsNo(leg1No)||conf2.size()!=getLegJointsNo(leg2No))
        throw std::runtime_error("Check collision (2 legs): Incorrect number of ref angles\n");
    std::vector<double> configuration(getLegJointsNo(0)*getLegsNo(),0);
    configuration[leg1No*3+0] = conf1[0]; configuration[leg1No*3+1] = conf1[1]; configuration[leg1No*3+2] = conf1[2];
    configuration[leg2No*3+0] = conf2[0]; configuration[leg2No*3+1] = conf2[1]; configuration[leg2No*3+2] = conf2[2];
    if (leg1No==0&&leg2No==1) {
    }
    else if (leg1No==2&&leg2No==3) {
    }
    else{
        throw std::runtime_error("this method is for neighbouring legs only\n");
    }
    std::vector<bool> collisionTable;
    std::vector<simulator::RenderObject> kinemObjects = getObjectsToRender(configuration);
    return collisionChecker->checkCollision(kinemObjects,collisionTable);
}

/// check collision for two legs
bool RobotAnymal::checkCollisionGM(size_t leg1No, const std::vector<double>& conf1, size_t leg2No, const std::vector<double>& conf2, ConstraintType& constrViolated){
    std::cout << constrViolated << leg1No << " " << leg2No << " " << conf1.size() << " " << conf2.size() << "\n";
    throw std::runtime_error("Collision detection for Anymal robot not implemented\n");
}

/// check collision for the robot
bool RobotAnymal::checkCollisionGM(const std::vector<double>& conf, ConstraintType& constrViolated){
    std::cout << constrViolated << conf.size() << "\n";
    throw std::runtime_error("Collision detection for Anymal robot not implemented\n");
}

/// check collision for the single leg and neighbouring legs only
bool RobotAnymal::checkCollisionNeighGM(size_t legNo, const std::vector<double>& currConf, const std::vector<double>& robotConf, ConstraintType& constrViolated){
    std::cout << constrViolated << legNo << " " << currConf.size() << " " << robotConf.size() << "\n";
    throw std::runtime_error("Collision detection for Anymal robot not implemented\n");
}

std::unique_ptr<walkers::Robot> walkers::createRobotAnymal(void) {
    return walkers::make_unique<RobotAnymal>();
}

std::unique_ptr<walkers::Robot> walkers::createRobotAnymal(std::string filename) {
    return walkers::make_unique<RobotAnymal>(filename);
}

std::unique_ptr<walkers::Robot> walkers::createRobotAnymal(std::string filename, const std::string& _name) {
    return walkers::make_unique<RobotAnymal>(filename, _name);
}
