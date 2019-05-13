//Norbert Werblinski
//Adam Czeszejkowski

#include "RobotModelMat/kinematicLie.h"
#include <iostream>


using namespace controller;

/// A single instance of BoardDynamixel
KinematicLie::Ptr kinematicLie;

KinematicLie::KinematicLie(void) : Kinematic("Kinematic Lie", TYPE_LIE) {
}

KinematicLie::KinematicLie(std::string configFilename) : Kinematic("Kienamtic Lie", TYPE_LIE){
    setlocale(LC_NUMERIC,"C");
	tinyxml2::XMLElement * pElement;
	tinyxml2::XMLElement * pListElement;
	tinyxml2::XMLDocument conf;
    std::string filename = configFilename;
	conf.LoadFile(filename.c_str());
	if (conf.FirstChildElement() == nullptr)
		std::cout << "unable to load Kinematic config file.\n";
	else {
            tinyxml2::XMLNode * pRoot = conf.FirstChildElement("leg")->FirstChildElement("parameters");
            double val;
            linksNo = conf.FirstChildElement("leg")->IntAttribute("segment_no");
            jointsNo = conf.FirstChildElement("leg")->IntAttribute("joint_no");
            for (size_t i = 0; i < linksNo; i++) {
                std::string tmp = "Joint" + std::to_string(i);
                pElement = pRoot->FirstChildElement(tmp.c_str());
				pListElement = pElement->FirstChildElement("value");
                std::vector<double> tmpV;
                while (pListElement != nullptr) {
                    pListElement->QueryDoubleText(&val);
					pListElement = pListElement->NextSiblingElement("value");
					tmpV.push_back(val);
				}
				ksi.push_back(tmpV);
			}
			pElement = pRoot->FirstChildElement("g0");
			pListElement = pElement->FirstChildElement("value");
			while (pListElement != nullptr)
			{
                pListElement->QueryDoubleText(&val);
				pListElement = pListElement->NextSiblingElement("value");
				g0.push_back(val);
			}
        }
}


KinematicLie::~KinematicLie(void) {
}

/*Compute forward kinematic, default (-1) -- the last joint
*
*
*/
walkers::Mat34 KinematicLie::forwardKinematic(const std::vector<double>& configuration, int linkNo){
    walkers::Mat34 fkmatrix;
	fkmatrix.setIdentity();
    if (linkNo == -1 || linkNo == (int)linksNo)
	{
        for (size_t i = 0; i < linksNo; i++)
		{
            fkmatrix *= createEMatrix(ksi[i], configuration[i]);
		}
        fkmatrix *= createGMatrix(g0);
	}
	else
	{
		for (int i = 0; i < linkNo; i++)
		{
			fkmatrix *= createEMatrix(ksi[i], configuration[i]);
		}
		fkmatrix *= createGMatrix(ksi[linkNo]);
    }
	return fkmatrix;
}

/*Compute inverse kinematic, default (-1) -- the last joint
*!!!!WORKS ONLY FOR Messor 2!!!
*
*/
std::vector<double> KinematicLie::inverseKinematic(const walkers::Mat34& linkPose, unsigned int linkNo){
    std::vector<double> configVector;
    std::vector<double> L; // lenghts of links
	L.push_back(fabs(ksi[1][2]));
	L.push_back(fabs(ksi[2][2])-fabs(ksi[1][2]));
    L.push_back(fabs(g0[0]) - fabs(ksi[2][2]));
    double x = linkPose(0, 3);
    double y = linkPose(1, 3);
    double z = linkPose(2, 3);
    configVector.push_back(atan2(y, x)+asin(-g0[1]/sqrt(pow(x,2.0)+pow(y,2.0))));//theta0
    double xp = sin(configVector[0])*y + cos(configVector[0])*x;
    //double yp = sin(configVector[0])*x + cos(configVector[0])*y;
	x = xp;
    //y = yp;
	switch (linkNo)
	{
	case 1:
		{
			break;
        }
	case 2:
        {
            configVector.push_back(atan2(z, x - L[0]));//theta1
			break;
		}
	default:
		{
            double l2 = (pow(x - L[0], 2) + pow(z, 2));
            double l = sqrt(l2);
            double B = atan2(z,x - L[0]);
            double g = (pow(L[1], 2) + l2 - pow(L[2], 2)) / (2 * L[1] * l);
            //if (g > 1) g = 1;
            //else if (g < -1) g = -1;
            double Y = acos(g);
			configVector.push_back(B + Y);//theta1
            double p = (l2 - pow(L[1], 2) - pow(L[2], 2)) / (2 * L[1] * L[2]);
            //if (p > 1) p = 1;
            //else if (p < -1) p = -1;
			configVector.push_back(-acos(p));//theta2
			break;
		}
    }
	return configVector;
}

/// Return set of link's poses
std::vector<walkers::Mat34> KinematicLie::getState(const std::vector<double>& configuration){
    std::vector<walkers::Mat34> linksPoses;
    for (size_t i = 0; i < linksNo; i++)
	{
        linksPoses.push_back(forwardKinematic(configuration, (int)i));
	}
	return linksPoses;
}

std::unique_ptr<Kinematic> controller::createKinematicLie(void) {
    //kinematicLie.reset(new KinematicLie());
    //return kinematicLie.get();
    return walkers::make_unique<KinematicLie>();
}

std::unique_ptr<Kinematic> controller::createKinematicLie(std::string filename) {
    //kinematicLie.reset(new KinematicLie(filename));
    //return kinematicLie.get();
    return walkers::make_unique<KinematicLie>(filename);
}
