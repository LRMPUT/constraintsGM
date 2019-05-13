#include "Defs/defs.h"
#include "../3rdParty/tinyXML/tinyxml2.h"
#include "RobotModelMat/Messor2Robot.h"
#include "CollisionDetection/CollisionDetectionColdet.h"
#ifdef BUILD_WITH_FCL
#include "CollisionDetection/CollisionDetectionFCL.h"
#endif
#include "Regression/GaussianMixture.h"
#include <fstream>
#include <iostream>
#include <thread>

using namespace std;

// plot vertical cross section
void plotVerticalCrossSectionKM(size_t legNo, std::string filename, walkers::Robot* robotMat){
    std::ofstream mfile;
    mfile.open (filename);
    mfile << "close all\nclear all\nX = [";

    /// manualy set parameters
    double x = -0.05;
    double z=-0.3;
    double incX=0.0025;
    double incZ=0.0025;
    int ptsNo = 200;

    for (int i=0;i<ptsNo;i++){
        for (int j=0;j<ptsNo;j++){
            mfile << x+i*incX << ", ";
        }
        mfile << ";\n";
    }
    mfile << "];\n";
    mfile << "Z = [";
    for (int i=0;i<ptsNo;i++){
        for (int j=0;j<ptsNo;j++){
            mfile << z+j*incZ << ", ";
        }
        mfile << ";\n";
    }
    mfile << "];\n";
    mfile << "W = [";
    for (int i=0;i<ptsNo;i++){
        for (int j=0;j<ptsNo;j++){
            walkers::Mat34 linkPose(walkers::Mat34::Identity());
            linkPose(0,3)=x+i*incX;
            linkPose(2,3)=z+j*incZ;
            if (robotMat->isInsideWorkspaceLeg((int)legNo, linkPose)){
                bool isMotionPossible;
                std::vector<double> legConfTmp = robotMat->inverseKinematicLeg(legNo,isMotionPossible, linkPose);
                double margin = robotMat->kinematicMargin(legNo,legConfTmp);
                mfile << margin << ", ";
            }
            else
                mfile << -1 << ", ";
//                    std::cout << j << "//" << ptsNo <<"\n";
        }
        mfile << ";\n";
        if (i%10==0)
            std::cout << i << "/" << ptsNo <<"\n";
    }
    mfile << "];\n";
    mfile << "surf(X,Z,W);\nxlabel('x[m]');\nylabel('z[m]')";
    mfile.close();
}

// plot vertical cross section
void plotHorizontalCrossSectionKM(size_t legNo, std::string filename, walkers::Robot* robotMat){
    ///plot kinematic margin -- horizontal
    std::ofstream mfile;
    mfile.open (filename);
    mfile << "close all\nclear all\nX = [";

    /// manualy set parameters
    double x = -0.1;
    double y = -0.4;
    double incX=0.0025;
    double incY=0.0025;
    double ptsNo = 300;

    for (int i=0;i<ptsNo;i++){
        for (int j=0;j<ptsNo;j++){
            mfile << x+i*incX << ", ";
        }
        mfile << ";\n";
    }
    mfile << "];\n";
    mfile << "Y = [";
    for (int i=0;i<ptsNo;i++){
        for (int j=0;j<ptsNo;j++){
            mfile << y+j*incY << ", ";
        }
        mfile << ";\n";
    }
    mfile << "];\n";
    mfile << "W = [";
    for (int i=0;i<ptsNo;i++){
        for (int j=0;j<ptsNo;j++){
            walkers::Mat34 linkPose(walkers::Mat34::Identity());
            linkPose(0,3)=x+i*incX;
            linkPose(1,3)=y+j*incY;
            if (robotMat->isInsideWorkspaceLeg((int)legNo, linkPose)){
                bool isMotionPossible;
                std::vector<double> legConfTmp = robotMat->inverseKinematicLeg(legNo, isMotionPossible, linkPose);
                double margin = robotMat->kinematicMargin(legNo,legConfTmp);
                mfile << margin << ", ";
            }
            else
                mfile << -1 << ", ";
//                    std::cout << j << "//" << ptsNo <<"\n";
        }
        mfile << ";\n";
        if (i%10==0)
            std::cout << i << "/" << ptsNo <<"\n";
    }
    mfile << "];\n";
    mfile << "surf(X,Y,W);\nxlabel('x[m]');\nylabel('y[m]')";
    mfile.close();
}

///plot kinematic margin -- vertical (GM)
void plotKinematicMarginVertGM(size_t legNo, const std::string& filename, walkers::Robot* robotMat, regression::Regression* gm){
    std::ofstream mfile;
    mfile.open (filename);
    mfile << "close all\nclear all\nX = [";

    /// manualy set parameters
    double x = -0.05;
    double z=-0.3;
    double incX=0.0025;
    double incZ=0.0025;
    int ptsNo = 200;

    for (int i=0;i<ptsNo;i++){
        for (int j=0;j<ptsNo;j++){
            mfile << x+i*incX << ", ";
        }
        mfile << ";\n";
    }
    mfile << "];\n";
    mfile << "Z = [";
    for (int i=0;i<ptsNo;i++){
        for (int j=0;j<ptsNo;j++){
            mfile << z+j*incZ << ", ";
        }
        mfile << ";\n";
    }
    mfile << "];\n";
    mfile << "W = [";
    for (int i=0;i<ptsNo;i++){
        for (int j=0;j<ptsNo;j++){
            walkers::Mat34 linkPose(walkers::Mat34::Identity());
            linkPose(0,3)=x+i*incX;
            linkPose(2,3)=z+j*incZ;
            if (robotMat->isInsideWorkspaceLeg((int)legNo, linkPose)){
                bool isMotionPossible;
                std::vector<double> legConfig = robotMat->inverseKinematicLeg(legNo,isMotionPossible, linkPose);
                Eigen::MatrixXd input(1,3);
                input(0,0)=legConfig[0]; input(0,1)=legConfig[1]; input(0,2)=legConfig[2];
                double margin = gm->computeOutput(input,0);
                if (margin<0)
                    mfile << 0 << ", ";
                else
                    mfile << margin << ", ";
            }
            else
                mfile << 0 << ", ";
        }
        mfile << ";\n";
    }
    mfile << "];\n";
    mfile << "surf(X,Z,W);\nxlabel('x[m]');\nylabel('z[m]')";
    mfile.close();
}

///plot kinematic margin -- vertical (GM)
void plotKinematicMarginHorizGM(size_t legNo, const std::string& filename, walkers::Robot* robotMat, regression::Regression* gm){
    std::ofstream mfile;
    mfile.open (filename);
    mfile << "close all\nclear all\nX = [";

    /// manualy set parameters
    double x = -0.1;
    double y = -0.4;
    double incX=0.0025;
    double incY=0.0025;
    double ptsNo = 300;

    for (int i=0;i<ptsNo;i++){
        for (int j=0;j<ptsNo;j++){
            mfile << x+i*incX << ", ";
        }
        mfile << ";\n";
    }
    mfile << "];\n";
    mfile << "Y = [";
    for (int i=0;i<ptsNo;i++){
        for (int j=0;j<ptsNo;j++){
            mfile << y+j*incY << ", ";
        }
        mfile << ";\n";
    }
    mfile << "];\n";
    mfile << "W = [";
    for (int i=0;i<ptsNo;i++){
        for (int j=0;j<ptsNo;j++){
            walkers::Mat34 linkPose(walkers::Mat34::Identity());
            linkPose(0,3)=x+i*incX;
            linkPose(1,3)=y+j*incY;
            if (robotMat->isInsideWorkspaceLeg((int)legNo, linkPose)){
                bool isMotionPossible;
                std::vector<double> legConfig = robotMat->inverseKinematicLeg(legNo, isMotionPossible, linkPose);
                Eigen::MatrixXd input(1,3);
                input(0,0)=legConfig[0]; input(0,1)=legConfig[1]; input(0,2)=legConfig[2];
                double margin = gm->computeOutput(input,0);
                if (margin<0)
                    mfile << 0 << ", ";
                else
                    mfile << margin << ", ";
            }
            else
                mfile << 0.0 << ", ";
        }
        mfile << ";\n";
    }
    mfile << "];\n";
    mfile << "surf(X,Y,W);\nxlabel('x[m]');\nylabel('y[m]')";
    mfile.close();
}

void generateTrainingSamples(size_t legNo, size_t trainingSamples,  std::string filename, walkers::Robot* robotMat){
    std::ofstream trainFile;
    trainFile.open (filename);

    std::vector<std::pair<double,double>> limits = robotMat->getJointLimits(legNo);
    std::srand((unsigned int)time(NULL));
    auto seed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    std::mt19937 mt(seed);
    std::vector<std::unique_ptr<std::uniform_real_distribution<double>>> dists(limits.size());
    for (size_t jointNo=0;jointNo<limits.size();jointNo++){
        dists[jointNo].reset(new std::uniform_real_distribution<double>(limits[jointNo].first, limits[jointNo].second));
    }
    for (size_t sampleNo=0;sampleNo<trainingSamples;sampleNo++){
        std::vector<double> legConfTmp;
        for (size_t jointNo=0;jointNo<limits.size();jointNo++){
            legConfTmp.push_back((*dists[jointNo])(mt));
            trainFile << legConfTmp[jointNo] << ",";
        }
        double margin = robotMat->kinematicMargin(legNo,legConfTmp);
        if (margin>0)
            trainFile << "\n" << margin << "\n";
        else
            trainFile << "\n" << 0 << "\n";
        std::cout << "sample no " << sampleNo << "/ " << trainingSamples << "\n";
    }
    std::cout << "leg No " << legNo << "\n";
    trainFile.close();
}

/// generate training samples for collision model
void generateTrainingSamplesCollision(size_t legNo, size_t trainingSamples,  std::string filename, walkers::Robot* robotMat){
    std::ofstream trainFile;
    trainFile.open (filename);

    std::vector<std::pair<double,double>> limits = robotMat->getJointLimits(legNo);
    std::srand((unsigned int)time(NULL));
    auto seed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    std::mt19937 mt(seed);
    std::vector<std::unique_ptr<std::uniform_real_distribution<double>>> dists(limits.size());
    for (size_t jointNo=0;jointNo<limits.size();jointNo++){
        dists[jointNo].reset(new std::uniform_real_distribution<double>(limits[jointNo].first, limits[jointNo].second));
    }
    for (size_t sampleNo=0;sampleNo<trainingSamples;sampleNo++){
        bool isColl;
        std::vector<double> legConfTmp(limits.size(), 0.0);
        if (sampleNo%4==0) {//we need more samples with collisions
            bool success = false;
            while (!success){
                for (size_t jointNo=0;jointNo<limits.size();jointNo++){
                    legConfTmp[jointNo] = (*dists[jointNo])(mt);
                }
                isColl = robotMat->checkCollision(legNo, legConfTmp);
                if (isColl) success = true;
            }
        }
        else{
            for (size_t jointNo=0;jointNo<limits.size();jointNo++){
                legConfTmp[jointNo] = (*dists[jointNo])(mt);
            }
            isColl = robotMat->checkCollision(legNo, legConfTmp);
        }
        for (size_t jointNo=0;jointNo<limits.size();jointNo++){
            trainFile << legConfTmp[jointNo] << ",";
        }
        if (isColl)
            trainFile << "\n" << 1 << "\n";
        else
            trainFile << "\n" << 0 << "\n";
        if (sampleNo%1000==0)
            std::cout << sampleNo << "/" << trainingSamples << "\n";
    }
    trainFile.close();
}

int main(void) {
    try {
        setlocale(LC_NUMERIC,"C");
        tinyxml2::XMLDocument config;
        config.LoadFile("../../resources/configGlobal.xml");
        if (config.ErrorID())
            std::cout << "unable to load config file.\n";

        std::string robotConfig(config.FirstChildElement( "Robot" )->FirstChildElement("config")->GetText());
        std::string robotType(config.FirstChildElement( "Robot" )->FirstChildElement("type")->GetText());

        std::string coldetConfig(config.FirstChildElement( "CollisionDetection" )->FirstChildElement("config")->GetText());
        std::string coldetType(config.FirstChildElement( "CollisionDetection" )->FirstChildElement("type")->GetText());

        std::string regressionConfig(config.FirstChildElement( "Regression" )->FirstChildElement("config")->GetText());
        std::string regressionType(config.FirstChildElement( "Regression" )->FirstChildElement("type")->GetText());

        std::unique_ptr<walkers::Robot> robotMat;
        if (robotType=="MessorII")
            robotMat = walkers::createRobotMessor(robotConfig);
        else
            robotMat = walkers::createRobotMessor(robotConfig);

        std::unique_ptr<coldet::CollisionDetection> collisionChecker;
        if (coldetType=="Coldet")
            collisionChecker =  coldet::createCollisionDetectionColdet(coldetConfig);
#ifdef BUILD_WITH_FCL
        else if (coldetType=="FCL")
            collisionChecker =  coldet::createCollisionDetectionFCL(coldetConfig);
#endif
        else
            collisionChecker =  coldet::createCollisionDetectionColdet(coldetConfig);

        // initialize collision model
        size_t elementsNo = robotMat->getLegsNo()*robotMat->getLegJointsNo(0)+1;
        size_t modelsNo = robotMat->getLegJointsNo(0)+1;
        std::vector<walkers::Vec3> scales;
        scales.push_back(robotMat->getModelScale());
        for (size_t linkNo=0; linkNo<robotMat->getLegLinksNo(0);linkNo++)
            scales.push_back(robotMat->getLegModelScale(0, linkNo));
        Objects3DS objects3DS;
        robotMat->load3Dobjects(objects3DS);
        collisionChecker->initializeMeshModelWalker(objects3DS, modelsNo, elementsNo, scales);

        ///train and verify kinematic margin
        for (size_t legNo=0;legNo<robotMat->getLegsNo();legNo++){
            std::cout << "legNo " <<legNo << "\n";

            std::cout << "plot vertical cross-section over the workspace -- kinematic margin.\n";
            plotVerticalCrossSectionKM(legNo, "kinematicMarginVert"+std::to_string(legNo)+".m", robotMat.get());
            std::cout << "plot horizontal cross-section over the workspace -- kinematic margin.\n";
            plotHorizontalCrossSectionKM(legNo, "kinematicMarginHoriz"+std::to_string(legNo)+".m", robotMat.get());

            /// generate data for training the kinematic margin model (joint configurations)
            std::cout << "Generate data for training.\n";
            generateTrainingSamples(legNo, 10000, "kinemMarginTrain"+std::to_string(legNo)+".dat", robotMat.get());
            std::cout << "Generate data for testing.\n";
            /// and samples for testing
            generateTrainingSamples(legNo, 10000, "kinemMarginTest"+std::to_string(legNo)+".dat", robotMat.get());

            std::cout << "Train the GM model (kinematic margin).\n";
            std::shared_ptr<regression::Regression> gm;
            gm = regression::createGaussianApproximation(std::string("regressionKinemMargin.xml"));
            gm->initializeTraining("kinemMarginTrain"+std::to_string(legNo)+".dat",
                                   "kinemMarginTest"+std::to_string(legNo)+".dat",
                                   "kinemMarginTest"+std::to_string(legNo)+".dat");
            gm->train();
            gm->storeResult("leg"+std::to_string(legNo)+"KM.dat");
            gm->writeSummary("resultsTrainKM"+std::to_string(legNo)+".txt", "resultsTestKM"+std::to_string(legNo)+".txt");
            std::cout << "Leg " << legNo << " trained\n";

            gm->load("leg"+std::to_string(legNo)+"KM.dat");
            plotKinematicMarginVertGM(legNo, "marginVertGM"+std::to_string(legNo)+".m", robotMat.get(), gm.get());
            plotKinematicMarginHorizGM(legNo, "marginHorizGM"+std::to_string(legNo)+".m", robotMat.get(), gm.get());
        }

        /// generate data for training the self-collision model (joint configurations) and verify the model
        for (size_t legNo=0;legNo<6;legNo++){
            std::cout << "leg No " << legNo << "\n";
            /// generate data for training the collision model (joint configurations)
            std::cout << "Generate data for training collision model.\n";
            generateTrainingSamplesCollision(legNo, 20000,  "coldetTrain"+std::to_string(legNo)+".dat", robotMat.get());
            std::cout << "Generate data for testing collision model.\n";
            /// and samples for testing
            generateTrainingSamplesCollision(legNo, 20000,  "coldetTest"+std::to_string(legNo)+".dat", robotMat.get());

            std::shared_ptr<regression::Regression> gm;
            gm = regression::createGaussianApproximation(std::string("regressionColl.xml"));
            gm->initializeTraining("coldetTrain"+std::to_string(legNo)+".dat",
                                   "coldetTest"+std::to_string(legNo)+".dat",
                                   "coldetTest"+std::to_string(legNo)+".dat");
            gm->train();
            gm->storeResult("leg"+std::to_string(legNo)+"coldet.dat");
            gm->writeSummary("resultsTrainColdet"+std::to_string(legNo)+".txt", "resultsTestColdet"+std::to_string(legNo)+".txt");
            std::cout << "Leg " << legNo << " trained\n";

            gm->load("leg"+std::to_string(legNo)+"coldet.dat");
            // do something with the collision mode (the example output is in "resultsTrainColdetX.txt" and resultsTestColdetX.txt)
        }

        std::cout << "Finished!\n";
        std::cout << "Use Octave/Matlab to plot generated m-files\n";

        return 1;
    }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }
    return 0;
}
