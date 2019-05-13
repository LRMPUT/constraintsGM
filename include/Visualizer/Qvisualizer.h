/** @file QVisualizer.h
 *
 * implementation - QGLVisualizer
 *
 */

#ifndef QVISUALIZER_H_INCLUDED
#define QVISUALIZER_H_INCLUDED

#include "Defs/defs.h"
#include "Utilities/observer.h"
#include "Utilities/stopwatch.h"
#include "Utilities/recorder.h"
#include "../../3rdParty/tinyXML/tinyxml2.h"
#include "CollisionDetection/CollisionDetectionColdet.h"
#ifdef BUILD_WITH_FCL
#include "CollisionDetection/CollisionDetectionFCL.h"
#endif
#include "Utilities/objects3DS.h"
#include "RobotModelMat/Messor2Robot.h"
#include <QGLViewer/qglviewer.h>
#include <QKeyEvent>
#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>

/// Map implementation
class QGLVisualizer: public QGLViewer, public Observer{
public:
    /// Pointer
    typedef std::unique_ptr<QGLVisualizer> Ptr;

    class Config
    {

      public:
        Config(){
        }

        Config(std::string configFilename){
            load(configFilename);
        }

        void load(std::string configFilename);

        public:
            /// verbose
            bool verbose;
            /// Background color
            QColor backgroundColor;
            /// visualize collisions
            bool visualizeCollisions;
            /// visualize footholds
            bool visualizeFootholds;
            /// draw global axisxis
            bool drawAxis;
            /// draw normal vectors to the terrain surface
            bool drawNormals;
    };

    /// Construction
    QGLVisualizer(void);

    /// Construction
    QGLVisualizer(std::string configFilename);

    /// Construction
    QGLVisualizer(std::string configFilename, std::string& robotConfig, std::string& _robotType, std::string& _coldetType,
                  std::string& _coldetConfig);

    /// Destruction
    ~QGLVisualizer(void);

    /// Observer update
    void update(const std::vector<walkers::Mat34>& envState);
    void update(std::vector<walkers::Mat34>& envState);
    void update(std::vector<walkers::JointInfo>& envState);
    void update(std::vector<walkers::Mat34>& envState, std::vector<walkers::JointInfo>& envState2);
    void update(std::vector<walkers::ObjectInfo>& envState, std::vector<walkers::JointInfo>& envState2);

    void renderPhysxObject(simulator::RenderObject& object);
    void init3DS(Objects3DS* objects3DS);
    void initKinematicModel(const Objects3DS& _objects3DS);
    void setShadowsVisible(bool shadowVisible);
    void updateSimTime(int simTime);
    /// draw kinematic model
    void updateKinematicModel(const walkers::Mat34& pose, const std::vector<double>& configuration);

    void addMesh(const Objects3DS& mesh);

private:
    ///config
    Config config;
    /// dynamic model of the robot
    Objects3DS* object3ds;
    /// kinematic model of the robot
    Objects3DS object3dsKinem;
    /// flag to update kinematic objects
    bool init3DSModelKinem = false;
    /// update state of the kinematic model
    bool updateStateKinemModel = false;
    /// pose of the kinematic robot's model
    walkers::Mat34 kinemRobotPose;

    /// 3D meshes
    std::vector<Objects3DS> meshes3D;
    /// flag to update 3d mesh
    bool init3Dmesh = false;
    /// mesh Id
    std::vector<size_t> meshIds;
    /// robot and map pose (mesh)
    std::vector<walkers::Mat34> robotAndMapPose;

    ///objects 2 draw
    std::vector<walkers::Mat34> objects;
    std::vector<simulator::RenderObject> kinematicObj;
    std::vector<walkers::ObjectInfo> bodies;
    std::vector<walkers::JointInfo> joints;
    bool groundInit = false;
    bool normalsInit = false;

    ///mutex
    std::mutex mtxObjects;
    bool init3DSModel = false;
    /// draw objects
    void draw();

    /// draw objects
    void animate();

    ///render
    void renderScene();

    /// initialize visualizer
    void init();

    /// generate help string
    std::string help() const;

    bool shadowFlag = false;

    /// simulation time from start
    int simulationTime;// miliseconds

    /// simulation time mutex
    std::mutex mtxSimTime;

    /// simulation time from start
    Stopwatch<std::chrono::milliseconds> realTime;// miliseconds

    /// type of the robot
    std::string robotType;
    /// math model of the robot
    std::unique_ptr<walkers::Robot> robotMat;
    /// ref values for kinematic model
    std::vector<double> refAnglesKinem;
    /// selected leg
    size_t selectedLeg;

    std::list<std::pair<double,walkers::RobotState>>::iterator currentRobotState;
    std::list<std::pair<double,walkers::Mat34>>::iterator currentRobotPose;

    /// Robot collision checker
    std::unique_ptr<coldet::CollisionDetection> collisionChecker;

    /// capture the key
    void keyPressEvent(QKeyEvent* key);
    /// draw text status
    void drawTextStatus(void);
    /// update kinematic objects
    void updateKinematicObjects(const std::vector<simulator::RenderObject>& kinemObjects);
    /// visualize collisions
    void visualizeCollisions(void);

    double animateTime = 0;

    Objects3DS objects3DS;
};

#endif // QVISUALIZER_H_INCLUDED
