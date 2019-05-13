/** @file insectLeg.cpp 
*
* @author Lukasz Mejlun
* @author Wojciech Nowakowski
*/

#include "RobotModelMat/insectLeg.h"
#include "RobotModelMat/kinematicLie.h"
#include <iostream>

using namespace controller;

/// A single instance of insect leg
InsectLeg::Ptr insectLeg;

InsectLeg::InsectLeg(void) : Leg("Insect Leg", TYPE_INSECT) 
{
	legKine = createKinematicLie();
}

InsectLeg::InsectLeg(std::string configFilename) : Leg("Insect Leg", TYPE_INSECT)
{
    setlocale(LC_NUMERIC,"C");
	tinyxml2::XMLDocument config;
    std::string filename = configFilename;
	config.LoadFile(filename.c_str());
	if (config.ErrorID())
	{
        std::cout << "unable to load insectLeg config file: "<< filename << "\n";
	}
    else {
        setlocale(LC_NUMERIC,"C");
        linksNo = config.FirstChildElement("leg")->IntAttribute("segment_no");
        jointsNo = config.FirstChildElement("leg")->IntAttribute("joint_no");
        footholdSearchRange = config.FirstChildElement("leg")->DoubleAttribute("footholdSearchRange");

        model3dName.resize(linksNo);
        model3dFlag.resize(linksNo);
        model3dId.resize(linksNo);
        model3dScale.resize(linksNo);
        model3dOrientation.resize(linksNo);
        model3dPosition.resize(linksNo);
        massLinks.resize(linksNo);

        /// Read segment data
        int counter = 0;
        tinyxml2::XMLElement* leg = config.FirstChildElement("leg");
        for( const tinyxml2::XMLElement* segment = leg->FirstChildElement("link"); segment; segment = segment->NextSiblingElement("link")) {
            lengths[counter] = segment->DoubleAttribute("length");

            const tinyxml2::XMLElement* inertial = segment->FirstChildElement("inertial");
            massLinks[counter]=inertial->FirstChildElement("mass")->DoubleAttribute("value");

            const tinyxml2::XMLElement* legXMLVisual = segment->FirstChildElement("visualization");

            if(legXMLVisual != NULL) {
                model3dFlag[counter] = true;
                model3dName[counter] = legXMLVisual->FirstChildElement("model3ds")->Attribute("filename");
                std::vector<walkers::Vec3> legVisualization;
                std::vector<std::string> legXMLVisualizationParamenters;
                legXMLVisualizationParamenters.push_back(legXMLVisual->FirstChildElement("model3ds")->Attribute("xyz"));
                legXMLVisualizationParamenters.push_back(legXMLVisual->FirstChildElement("model3ds")->Attribute("scale"));
                legXMLVisualizationParamenters.push_back(legXMLVisual->FirstChildElement("model3ds")->Attribute("rpy"));
                walkers::getTokenizedContent(legXMLVisualizationParamenters, legVisualization);
                model3dPosition[counter] = walkers::Vec3(legVisualization.at(0).x(), legVisualization.at(0).z(), legVisualization.at(0).y());
                model3dScale[counter] = legVisualization.at(1);
                model3dOrientation[counter] = legVisualization.at(2);
            }
            counter++;
        }

        ///Read joint data
        counter = 0;
        for( const tinyxml2::XMLElement* joint = leg->FirstChildElement("joint"); joint; joint = joint->NextSiblingElement("joint")) {
            jointLimits.resize(jointsNo);
            jointLimits[counter].first = std::stof(joint->FirstChildElement("limit")->Attribute("lower"));
            jointLimits[counter].second = std::stof(joint->FirstChildElement("limit")->Attribute("upper"));
            counter++;
        }

//        std::cout << "links no: " << linksNo << " joints no: " << jointsNo << "\n";
//		std::cout << "Lenght1: " << lengths[0] << std::endl;
//		std::cout << "Lenght2: " << lengths[1] << std::endl;
//        std::cout << "Lenght3: " << lengths[2] << std::endl;
	}
    legKine = createKinematicLie(configFilename);
}

InsectLeg::~InsectLeg(void) {
}

///get 3d model poses
void InsectLeg::get3DmodelPoses(std::vector<walkers::Mat34>& _model3dPoses){
    _model3dPoses.resize(jointsNo);
    for (size_t jointNo=0;jointNo<(size_t)jointsNo;jointNo++){
        _model3dPoses[jointNo].matrix().block<3,3>(0,0) = walkers::toRotationMat(model3dOrientation[jointNo]);
        _model3dPoses[jointNo].matrix().block<3,1>(0,3) = model3dPosition[jointNo].vector();
    }
}

/** Compute torque in each joint for given the force applied in the foot
* @param [in] force Indicator to the force vector which works in x,y,z axis
* @param [in] config vector of joints parameters of leg
* @return std::vector<double> load vector in individual nodes
*/
std::vector<double> InsectLeg::computLoad(walkers::Vec3& force, std::vector<double> config) {
    std::vector<double> result;
    walkers::Vec3 torque;
	torque.vector() = -computeJacobian_transposed(config) * force.vector();

	for (int i = 0; i < 3; i++)
	{
		result.push_back(torque.vector().data()[i]);
	}

	return result;
}

/** Compute torque in each joint for given the force applied in the foot
* @param [in] force Indicator to the force vector which works in x,y,z axis
* @param [in] config vector of joints parameters of leg
* @param [in] is_leg_left is the leg on the left side of robot
* @return std::vector<double> load vector in individual nodes
*/
std::vector<double> InsectLeg::computLoad(walkers::Vec3& force, std::vector<double> config, bool is_leg_left)
{
    std::vector<double> result;
    walkers::Vec3 torque;
	torque.vector() = -computeJacobian_transposed(config) * force.vector();

	for (int i = 0; i < 3; i++)
	{
		result.push_back(torque.vector().data()[i]);
	}
	if(is_leg_left)
	{
		result[0] = -result[0];
	}

	return result;
}

/** Compute forward kinematic, default (-1) -- the last joint
* @param [in] configuration configuration variables legs
* @param [in] linkNo the number of nodes kinematic
* @return Mat34 homogeneous matrix legs
*/
walkers::Mat34 InsectLeg::forwardKinematic(const std::vector<double>& configuration, int linkNo, bool is_leg_left){
	if(is_leg_left)	{
        walkers::Mat34 temp = legKine->forwardKinematic(configuration, linkNo);
		temp(0, 3) = -temp(0, 3);
		temp(1, 3) = -temp(1, 3);
		return temp;
	}
    else{
		return legKine->forwardKinematic(configuration, linkNo);
	}
}

/** Compute inverse kinematic, default (-1) -- the last joint
* @param [in] linkPose homogeneous matrix legs
* @param [in] linkNo the number of nodes kinematic
* @return std::vector<double> configuration variables legs
*/
std::vector<double> InsectLeg::inverseKinematic(const walkers::Mat34& linkPose, bool& motionPossible, int linkNo) {
    std::vector<double> legConf;
    legConf = legKine->inverseKinematic(linkPose, linkNo);
    motionPossible = isInsideWorkspace(legConf);
    return legConf;
}

/// is robot reference legs pose inside robot workspace
bool InsectLeg::isInsideWorkspace(const std::vector<double>& legConf) const{
    if (legConf.size()>(size_t)jointsNo)
        throw std::runtime_error("incorrect number of joint.\n");
    else {
        for (size_t jointNo=0;jointNo<legConf.size();jointNo++){
            if (legConf[jointNo]<jointLimits[jointNo].first||legConf[jointNo]>jointLimits[jointNo].second){
                return false;
            }
            if (std::isnan(legConf[jointNo])||std::isinf(legConf[jointNo])){
                return false;
            }
        }
    }
    return true;
}

/// is robot reference legs poses inside robot workspace
bool InsectLeg::isInsideWorkspace(const walkers::Mat34& linkPose){
    bool isMotionPossible;
    std::vector<double> refAngles = inverseKinematic(linkPose, isMotionPossible);
    if (!isMotionPossible){
        return false;
    }
    return isInsideWorkspace(refAngles);
}

/// Jacobian of Messor leg
walkers::Mat33 InsectLeg::computeJacobian_transposed(std::vector<double> config)
{
    walkers::Mat33 temp;

    temp(0, 0) = 0;
    temp(1, 0) = 0;
    temp(2, 0) = 0;
    temp(0, 1) = 0;
    temp(1, 1) = -lengths[2] * sin(config[1]) * sin(config[2]);
    temp(2, 1) = -lengths[2] * cos(config[1]) * sin(config[2]);
    temp(0, 2) = 0;
    temp(1, 2) = lengths[2] * cos(config[1]) * cos(config[2]) + lengths[1] * cos(config[1]);
    temp(2, 2) = -lengths[2] * sin(config[1]) * cos(config[2]) - lengths[1] * sin(config[1]);


	return temp;
}

/** Constructor without arguments of Leg object*
 * @return controller::Leg* indicator to the Leg object
 */
std::unique_ptr<controller::Leg> controller::createInsectLeg(void) {
    //insectLeg.reset(new InsectLeg());
    //return insectLeg.get();
    return walkers::make_unique<InsectLeg>();
}

/** Constructor of Leg object which argument is location of configuration file
 * @param [in] filename relative path to acces the file
 * @return controller::Leg* indicator to the Leg object
 */
std::unique_ptr<controller::Leg> controller::createInsectLeg(std::string filename) {
    //insectLeg.reset(new InsectLeg(filename));
    //return insectLeg.get();
    return walkers::make_unique<InsectLeg>(filename);
}
