#include "Defs/defs.h"
#include "../3rdParty/tinyXML/tinyxml2.h"
#include "Visualizer/Qvisualizer.h"
// robot model
#include "RobotModelMat/Messor2Robot.h"
#include "RobotModelMat/AnymalRobot.h"
// Utilities
#include "Utilities/recorder.h"
#include <GL/glut.h>
#include <qapplication.h>
#include <iostream>
#include <thread>

using namespace std;

int main(int argc, char** argv)
{
    try {
        setlocale(LC_NUMERIC,"C");
        tinyxml2::XMLDocument config;
        config.LoadFile("../../resources/configGlobal.xml");
        if (config.ErrorID()){
            std::cout << "unable to load config file.\n";
            return 0;
        }
        std::string robotConfig(config.FirstChildElement( "Robot" )->FirstChildElement("config")->GetText());
        std::string robotType(config.FirstChildElement( "Robot" )->FirstChildElement("type")->GetText());

        std::string visualizerConfig(config.FirstChildElement( "Visualizer" )->FirstChildElement("config")->GetText());
        std::string visualizerType(config.FirstChildElement( "Visualizer" )->FirstChildElement("type")->GetText());

        std::string coldetConfig(config.FirstChildElement( "CollisionDetection" )->FirstChildElement("config")->GetText());
        std::string coldetType(config.FirstChildElement( "CollisionDetection" )->FirstChildElement("type")->GetText());

        QApplication application(argc,argv);

        setlocale(LC_NUMERIC,"C");
        glutInit(&argc, argv);

        /// Robot model
        std::unique_ptr<walkers::Robot> robotMat;
        if (robotType=="MessorII"){
            robotMat = walkers::createRobotMessor(robotConfig);
        }
        else if (robotType=="Anymal"){
            robotMat = walkers::createRobotAnymal(robotConfig);
        }
        else
            robotMat = walkers::createRobotMessor(robotConfig);

        QGLVisualizer visu(visualizerConfig, robotConfig, robotType, coldetType, coldetConfig);
        visu.setWindowTitle("Simulator viewer");
        visu.show();

        ///kinematic model of the robot
        std::vector<double> robotConfHexa = {0.7854,0.41888,-114*M_PI/180, 0.0,0.41888,-1.9897, -0.7854,0.41888,-1.9897, 0.7854,0.41888,-1.9897, 0.0,0.41888,-1.9897, -0.7854,0.41888,-1.9897};
        std::vector<double> robotConfQuad = {-7.2*M_PI/180,24*M_PI/180,-74*M_PI/180, 7*M_PI/180,24*M_PI/180,-74*M_PI/180, 7*M_PI/180,24*M_PI/180,-74*M_PI/180, -7*M_PI/180,24*M_PI/180,-74*M_PI/180};
        std::vector<double> robotConf;
        if (robotType=="MessorII" || robotType=="PhantomX"){
            robotConf = robotConfHexa;
        }
        else if (robotType=="Anymal"){
            robotConf = robotConfQuad;
        }

        walkers::Mat34 robotPose(walkers::Mat34::Identity());
        robotPose(0,3)=0.0;         robotPose(1,3)=0.0;         robotPose(2,3)=-0.7;
        robotPose.matrix().block<3,3>(0,0) = walkers::toRotationMat(walkers::Vec3(0.0,0.0,0.0));
        visu.updateKinematicModel(robotPose, robotConf);

        std::cout << "Robot visualization\n";
        std::cout << "Use 'e', 'r', 't', 'd', 'f', 'g' keys to change reference values in joints.\n";
        std::cout << "Use 'c', 'v' to change the number of the leg.\n";
        // Run main loop.
        application.exec();
        std::cout << "Finished\n";
        return 1;
    }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }
    return 0;
}
