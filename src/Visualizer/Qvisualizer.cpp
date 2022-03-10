#include "Visualizer/Qvisualizer.h"
#include "RobotModelMat/AnymalRobot.h"
#include <memory>
#include <cmath>
#include <stdexcept>
#include <chrono>
#include <GL/glut.h>

using namespace simulator;

/// A single instance of Visualizer
QGLVisualizer::Ptr visualizer;

QGLVisualizer::QGLVisualizer(void) : selectedLeg(0){
    simulationTime = 0;
    updateStateKinemModel = false;
    config.visualizeCollisions = false;
}

/// Construction
QGLVisualizer::QGLVisualizer(std::string configFilename) : config(configFilename),
    selectedLeg(0){
    simulationTime = 0;
    updateStateKinemModel = false;
    config.visualizeCollisions = false;
}

/// Construction
QGLVisualizer::QGLVisualizer(std::string configFilename, std::string& robotConfig, std::string& _robotType, std::string& _coldetType, std::string& _coldetConfig) : config(configFilename),
    robotType(_robotType), selectedLeg(0){
    simulationTime = 0;
    updateStateKinemModel = false;
    config.visualizeCollisions = false;
    if (robotType=="MessorII"){
        robotMat = walkers::createRobotMessor(robotConfig);
    }
    else if (robotType=="Anymal"){
        robotMat = walkers::createRobotAnymal(robotConfig);
    }
    else if (robotType=="Anymal_C"){
        robotMat = walkers::createRobotAnymal(robotConfig, "RobotAnymal_C");
    }
    else
        robotMat = walkers::createRobotMessor(robotConfig);

    refAnglesKinem.resize(robotMat->getLegsNo()*robotMat->getLegJointsNo(0));

    if ((robotType=="MessorII")||(robotType=="PhantomX")){
        refAnglesKinem = std::vector<double> ({0.7854,0.41888,-114*M_PI/180, 0.0,0.41888,-1.9897, -0.7854,0.41888,-1.9897, 0.7854,0.41888,-1.9897, 0.0,0.41888,-1.9897, -0.7854,0.41888,-1.9897});
    }
    else if (robotType=="Anymal"||robotType=="Anymal_C") {
        refAnglesKinem = std::vector<double> ({7.0*M_PI/180,33*M_PI/180,-60*M_PI/180, -7*M_PI/180,33*M_PI/180,-60*M_PI/180, 7*M_PI/180,33*M_PI/180,-60*M_PI/180, -7*M_PI/180,33*M_PI/180,-60*M_PI/180});
    }

    robotMat->load3Dobjects(objects3DS);
    initKinematicModel(objects3DS);
    init3DS(&objects3DS);

    if (_coldetType=="Coldet")
        collisionChecker =  coldet::createCollisionDetectionColdet(_coldetConfig);
#ifdef BUILD_WITH_FCL
    else if (_coldetType=="FCL")
        collisionChecker =  coldet::createCollisionDetectionFCL(_coldetConfig);
#endif
    else
        collisionChecker =  coldet::createCollisionDetectionColdet(_coldetConfig);
    size_t elementsNo = robotMat->getLegsNo()*robotMat->getLegJointsNo(0)+1;
    size_t modelsNo = robotMat->getLegJointsNo(0)+1;
    std::vector<walkers::Vec3> scales;
    scales.push_back(robotMat->getModelScale());
    for (size_t linkNo=0; linkNo<robotMat->getLegLinksNo(0);linkNo++)
        scales.push_back(robotMat->getLegModelScale(0, linkNo));
    std::cout << "initialize mesh model\n";        collisionChecker->initializeMeshModelWalker(objects3DS, modelsNo, elementsNo, scales);
}

/// Destruction
QGLVisualizer::~QGLVisualizer(void) {
    //#ifdef DEBUG
    //std::cout << "QGLVisualizer destructor\n";
    //#endif
}


void QGLVisualizer::Config::load(std::string configFilename) {
    setlocale(LC_NUMERIC,"C");
    tinyxml2::XMLDocument config;
    std::string filename = "../../resources/" + configFilename;
    config.LoadFile(filename.c_str());
    if (config.ErrorID())
        std::cout << "unable to load Visualizer config file: " + filename << std::endl;

    tinyxml2::XMLElement * model = config.FirstChildElement( "VisualizerConfig" );
    model->FirstChildElement( "parameters" )->QueryBoolAttribute("verbose", &verbose);
    model->FirstChildElement( "parameters" )->QueryBoolAttribute("visualizeCollisions", &visualizeCollisions);
    model->FirstChildElement( "parameters" )->QueryBoolAttribute("visualizeFootholds", &visualizeFootholds);
    model->FirstChildElement( "parameters" )->QueryBoolAttribute("drawAxis", &drawAxis);
    model->FirstChildElement( "parameters" )->QueryBoolAttribute("drawNormals", &drawNormals);
    double rgba[4]={0,0,0,0};
    model->FirstChildElement( "background" )->QueryDoubleAttribute("red", &rgba[0]);
    model->FirstChildElement( "background" )->QueryDoubleAttribute("green", &rgba[1]);
    model->FirstChildElement( "background" )->QueryDoubleAttribute("blue", &rgba[2]);
    model->FirstChildElement( "background" )->QueryDoubleAttribute("alpha", &rgba[3]);

    backgroundColor.setRedF(rgba[0]); backgroundColor.setGreenF(rgba[1]);
    backgroundColor.setBlueF(rgba[2]); backgroundColor.setAlphaF(rgba[3]);
}

/// Observer update
void QGLVisualizer::update(std::vector<walkers::Mat34>& envState) {
    mtxObjects.lock();
    objects = envState;
    mtxObjects.unlock();
}
void QGLVisualizer::update(std::vector<walkers::JointInfo>& envState){
    mtxObjects.lock();
    joints = envState;
    mtxObjects.unlock();
}
void QGLVisualizer::update(std::vector<walkers::Mat34>& envState, std::vector<walkers::JointInfo>& envState2){
    mtxObjects.lock();
    objects = envState;
    joints = envState2;
    mtxObjects.unlock();
}
void QGLVisualizer::update(std::vector<walkers::ObjectInfo>& envState, std::vector<walkers::JointInfo>& envState2){
    mtxObjects.lock();
    std::vector<walkers::Mat34> objsTmp;
    for (const auto& obj : envState)
        objsTmp.push_back(obj.pos_mat);
    objects = objsTmp;
    //bodies = envState;
    joints = envState2;
    mtxObjects.unlock();
}

/// Observer update
void QGLVisualizer::update(const std::vector<walkers::Mat34>& envState) {
    mtxObjects.lock();
    objects = envState;
    mtxObjects.unlock();
}

/// draw kinematic model
void QGLVisualizer::updateKinematicModel(const walkers::Mat34& pose, const std::vector<double>& configuration){
    std::vector<simulator::RenderObject> kinemObjects2render = robotMat->getObjectsToRender(configuration);
    std::cout << "kinemObjects2render " << kinemObjects2render.size() << "\n";
    kinemRobotPose = pose;
    for (auto& obj : kinemObjects2render){
        obj.mat = kinemRobotPose*obj.mat;
    }
    updateKinematicObjects(kinemObjects2render);
}

void QGLVisualizer::updateKinematicObjects(const std::vector<simulator::RenderObject>& kinemObjects) {
    mtxObjects.lock();
    kinematicObj = kinemObjects;
    for (auto& ko : kinematicObj){
        ko.color[0]=0.7; ko.color[1]=0.7; ko.color[2]=0.5; ko.color[3]=0.8;
    }
    mtxObjects.unlock();
}

void QGLVisualizer::init3DS(Objects3DS* _objects3DS)
{
    object3ds = _objects3DS;
    init3DSModel = true;
}

void QGLVisualizer::initKinematicModel(const Objects3DS& objects3DSkinem) {
    object3dsKinem = objects3DSkinem;
    init3DSModelKinem = true;
}

void QGLVisualizer::addMesh(const Objects3DS& mesh) {
    meshes3D.push_back(mesh);
    init3Dmesh= true;
}

void QGLVisualizer::setShadowsVisible(bool shadowVisible) {
    shadowFlag = shadowVisible;
}

/// draw objects
void QGLVisualizer::draw(){
    if (config.drawAxis)
        drawAxis();
    if (updateStateKinemModel){
        std::vector<simulator::RenderObject> kinemObjects;
        kinemObjects = robotMat->getObjectsToRender(refAnglesKinem);
        for (auto& obj : kinemObjects)
            obj.mat = kinemRobotPose*obj.mat;
        updateKinematicObjects(kinemObjects);
        updateStateKinemModel = false;
    }
    glPushMatrix();
    renderScene();
    glPopMatrix();
    drawTextStatus();
    if (config.visualizeCollisions){
        visualizeCollisions();
    }

}

/// visualize collisions
void QGLVisualizer::visualizeCollisions(void){
    std::vector<bool> collisionTable;
    std::cout << "refAnglesKinem[0],refAnglesKinem[1],refAnglesKinem[2] " << refAnglesKinem[selectedLeg*3+0]*180/M_PI << ", " << refAnglesKinem[selectedLeg*3+1]*180/M_PI << ", " << refAnglesKinem[selectedLeg*3+2]*180/M_PI << "\n";
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    walkers::Mat34 robPoss(walkers::Mat34::Identity());
    robPoss(2,3)=0.6;
    std::vector<simulator::RenderObject> kinemObjects;
    kinemObjects = robotMat->getObjectsToRender(refAnglesKinem);
    for (auto& obj : kinemObjects)
        obj.mat = robPoss*obj.mat;
    collisionChecker->checkCollision(kinemObjects, collisionTable);
    std::chrono::steady_clock::time_point end= std::chrono::steady_clock::now();
    std::cout << "Collision detection time = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << " [us]" << std::endl;
    size_t collIdx=0;
    for (auto coll : collisionTable){
        if (coll)
            std::cout << collIdx << " is with collision\n";
        else
            std::cout << collIdx << " no collision\n";
        collIdx++;
    }
    begin = std::chrono::steady_clock::now();
    walkers::Robot::ConstraintType constrViol;
    bool isColl = robotMat->checkCollision(refAnglesKinem, constrViol);
    if (isColl)
        std::cout << " is collision\n";
    else
        std::cout << " no collisions\n";
    end = std::chrono::steady_clock::now();
    std::cout << "Collision detection time = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << " [us]" << std::endl;
}

/// draw text status
void QGLVisualizer::drawTextStatus(){
    qglColor(foregroundColor());
    int minutesFromStart = (int)realTime.stop()/60000;
    int secondsFromStart = ((int)realTime.stop()/1000)%60;
    std::string realTimeTxt = "Real time: " + std::string( 2-std::to_string(minutesFromStart).length(), '0').append(std::to_string(minutesFromStart)) + ":" + std::string( 2-std::to_string(secondsFromStart).length(), '0').append(std::to_string(secondsFromStart));
    //this->setD
    drawText(10, this->height()-4,QString(realTimeTxt.c_str()));
    mtxSimTime.lock();
    minutesFromStart = (int)simulationTime/60000;
    secondsFromStart = ((int)simulationTime/1000)%60;
    std::string simTimeTxt = "Sim. time: " + std::string( 2-std::to_string(minutesFromStart).length(), '0').append(std::to_string(minutesFromStart)) + ":" + std::string( 2-std::to_string(secondsFromStart).length(), '0').append(std::to_string(secondsFromStart));
    //this->setD
    drawText(10, this->height()-15,QString(simTimeTxt.c_str()));
    mtxSimTime.unlock();
}

void QGLVisualizer::updateSimTime(int simTime){
    mtxSimTime.lock();
    simulationTime = simTime;
    mtxSimTime.unlock();
}

void QGLVisualizer::renderScene() {
    mtxObjects.lock();
    if (meshes3D.size()>0){
        if(init3Dmesh) {
            init3Dmesh = false;
            size_t meshNo=0;
            meshIds.resize(meshes3D.size());
            size_t objects3Dsize = object3dsKinem.objects.size()+3;
            for (size_t id = objects3Dsize; id < objects3Dsize + meshes3D.size(); id++){
                glNewList(int(id+1), GL_COMPILE);
//                meshes3D[meshNo].ObjLoad(meshes3D[meshNo].filenames[meshNo]);
                meshes3D[meshNo].Object3DSinit(0);
                meshIds[meshNo] = id+1;
                glEndList();
                meshNo++;
            }
            std::cout << "donene\n";
        }
        size_t meshNo = 0;
        for (const auto& id : meshIds){
            glPushMatrix();
            walkers::Mat34 mat;
            if (meshNo < robotAndMapPose.size()){
                mat = robotAndMapPose[meshNo];
            }
            else
                mat = walkers::Mat34::Identity();
            double GLmat[16]={mat(0,0), mat(1,0), mat(2,0), mat(3,0),
                                mat(0,1), mat(1,1), mat(2,1), mat(3,1),
                                mat(0,2), mat(1,2), mat(2,2), mat(3,2),
                                mat(0,3), mat(1,3), mat(2,3), mat(3,3)};
            glMultMatrixd(GLmat);
//                glScaled(object.scaleX,object.scaleY,object.scaleZ);
            glCallList(int(id+1));
            glPopMatrix();
            meshNo++;
        }
    }
    if(kinematicObj.size()>0) {
        //std::cout<<"Debug: dynamicObj | kinematicObj\n";
        if(init3DSModel) {
            init3DSModel = false;
            for(size_t i=0; i<object3ds->objects.size(); ++i) {
                glNewList(int(i+4), GL_COMPILE);
                object3ds->Object3DSinit(int(i));
                glEndList();
            }
        }
        if(init3DSModelKinem) {
           // std::cout << "init kinematic model\n";
            init3DSModelKinem = false;
           // std::cout << "object3ds->objects.size() " << object3ds->objects.size() << " object3dsKinem.objects.size()" << object3dsKinem.objects.size() << "\n";
            for(size_t i=object3ds->objects.size(); i<object3dsKinem.objects.size(); ++i) {
                glNewList(int(i+4), GL_COMPILE);
                object3dsKinem.Object3DSinit(int(i));
                glEndList();
            }
            //std::cout << "init kinematic model done\n";
        }
        for(unsigned int i=0;i<kinematicObj.size();++i) {
            glColor4d(kinematicObj[i].color[0],kinematicObj[i].color[1],kinematicObj[i].color[2],kinematicObj[i].color[3]);
            renderPhysxObject(kinematicObj[i]);
        }
    }
    mtxObjects.unlock();
}

/// draw objects
void QGLVisualizer::animate(){
//    qglviewer::Vec position;
//    animateTime = animateTime+0.01;
//    double radius = 3.5;
//    position.setValue(float(radius * sin(animateTime)), float(radius * cos(animateTime)), 1.75);
//    camera()->setPosition(position);
//    qglviewer::Vec view(float(-radius * sin(animateTime)), float(-radius * cos(animateTime)), -1.75);
//    camera()->setOrientation(0.0,-1.57);
//    camera()->setViewDirection(view);

//    qglviewer::Vec position;
//    animateTime = animateTime+0.01;
//    double radius = 0.075;
//    position.setValue(float(radius * sin(animateTime)), float(-0.075), float(radius * cos(animateTime)));
//    camera()->setPosition(position);
//    qglviewer::Vec view(float(-radius * sin(animateTime)), float(0.075), float(-radius * cos(animateTime)));
//    camera()->setOrientation(1.57,0);
//    camera()->setViewDirection(view);
}

/// initialize visualizer
void QGLVisualizer::init(){
    // Light setup
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 128.0);
    GLfloat specular_color[4] = { 0.5f, 0.5f, 0.5f, 1.0f };
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,  specular_color);

    //Set global ambient light
    GLfloat black[] = {0.5f, 0.5f, 0.5f, 1.0f};
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, black);

    glEnable(GL_AUTO_NORMAL);
    glEnable(GL_NORMALIZE);
    // Restore previous viewer state.
    restoreStateFromFile();

    camera()->setZNearCoefficient(0.00001f);
    camera()->setZClippingCoefficient(100.0);

    setBackgroundColor(config.backgroundColor);

    glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

     // Opens help window
    help();


    startAnimation();
}

/// capture the key
void QGLVisualizer::keyPressEvent(QKeyEvent* key){
    if ((key->key() == Qt::Key_C)) {
        if (selectedLeg<refAnglesKinem.size()-2)
            selectedLeg++;
    }
    if ((key->key() == Qt::Key_V)) {
        if (selectedLeg>0)
            selectedLeg--;
    }
    if ((key->key() == Qt::Key_E)) {
        refAnglesKinem[selectedLeg*3+0]+=1.0*M_PI/180.0;
        std::cout << "refAnglesKinem[selectedLeg*3+0] " << refAnglesKinem[selectedLeg*3+0] << "\n";
        updateStateKinemModel = true;
    }
    if ((key->key() == Qt::Key_D)) {
        refAnglesKinem[selectedLeg*3+0]-=1.0*M_PI/180.0;
        std::cout << "refAnglesKinem[selectedLeg*3+0] " << refAnglesKinem[selectedLeg*3+0] << "\n";
        updateStateKinemModel = true;
    }
    if ((key->key() == Qt::Key_R)) {
        refAnglesKinem[selectedLeg*3+1]+=1.0*M_PI/180.0;
        std::cout << "refAnglesKinem[selectedLeg*3+1] " << refAnglesKinem[selectedLeg*3+1] << "\n";
        updateStateKinemModel = true;
    }
    if ((key->key() == Qt::Key_F)) {
        refAnglesKinem[selectedLeg*3+1]-=1.0*M_PI/180.0;
        std::cout << "refAnglesKinem[selectedLeg*3+1] " << refAnglesKinem[selectedLeg*3+1] << "\n";
        updateStateKinemModel = true;
    }
    if ((key->key() == Qt::Key_T)) {
        refAnglesKinem[selectedLeg*3+2]+=1.0*M_PI/180.0;
        std::cout << "refAnglesKinem[selectedLeg*3+2] " << refAnglesKinem[selectedLeg*3+2] << "\n";
        updateStateKinemModel = true;
    }
    if ((key->key() == Qt::Key_G)) {
        refAnglesKinem[selectedLeg*3+2]-=1.0*M_PI/180.0;
        std::cout << "refAnglesKinem[selectedLeg*3+2] " << refAnglesKinem[selectedLeg*3+2] << "\n";
        updateStateKinemModel = true;
    }
    if ((key->key() == Qt::Key_X)) {
        config.drawAxis = (config.drawAxis) ? false : true;
    }
    if ((key->key() == Qt::Key_B)) {
        config.visualizeCollisions = (config.visualizeCollisions) ? false : true;
    }
    if (key->key() == Qt::Key_O){
        kinematicObj[0].mat = kinematicObj[0].mat* walkers::toRotationMat(walkers::Vec3(0,0,0.1));
    }
    else
        QGLViewer::keyPressEvent(key);
}


/// generate help string
std::string QGLVisualizer::help() const{
    std::string text("S i m p l e V i e w e r");
    text += "Use the mouse to move the camera around the object. ";
    text += "You can respectively revolve around, zoom and translate with the three mouse buttons. ";
    text += "Left and middle buttons pressed together rotate around the camera view direction axis<br><br>";
    text += "Pressing <b>Alt</b> and one of the function keys (<b>F1</b>..<b>F12</b>) defines a camera keyFrame. ";
    text += "Simply press the function key again to restore it. Several keyFrames define a ";
    text += "camera path. Paths are saved when you quit the application and restored at next start.<br><br>";
    text += "Press <b>F</b> to display the frame rate, <b>A</b> for the world axis, ";
    text += "<b>Alt+Return</b> for full screen mode and <b>Control+S</b> to save a snapshot. ";
    text += "See the <b>Keyboard</b> tab in this window for a complete shortcut list.<br><br>";
    text += "Double clicks automates single click actions: A left button double click aligns the closer axis with the camera (if close enough). ";
    text += "A middle button double click fits the zoom of the camera and the right button re-centers the scene.<br><br>";
    text += "A left button double click while holding right button pressed defines the camera <i>Revolve Around Point</i>. ";
    text += "See the <b>Mouse</b> tab and the documentation web pages for details.<br><br>";
    text += "Press <b>Escape</b> to exit the viewer.";
    return text;
}


void QGLVisualizer::renderPhysxObject(RenderObject& object)
{
    double GLmat[16]={object.mat(0,0), object.mat(1,0), object.mat(2,0), object.mat(3,0),
                        object.mat(0,1), object.mat(1,1), object.mat(2,1), object.mat(3,1),
                        object.mat(0,2), object.mat(1,2), object.mat(2,2), object.mat(3,2),
                        object.mat(0,3), object.mat(1,3), object.mat(2,3), object.mat(3,3)};

    if(object.type == RenderObjectType::BOX)
    {
        glPushMatrix();
        glMultMatrixd(GLmat);
        glScaled(object.x, object.y, object.z);
        glutSolidCube(2.0);
        glPopMatrix();
    }
    else if(object.type == RenderObjectType::SPHERE)
    {
        glPushMatrix();
        glMultMatrixd(GLmat);
        glutSolidSphere(object.x, 10, 10);
        glPopMatrix();
    }
    else if(object.type == RenderObjectType::LINE)
    {
        glPushMatrix();
        glLineWidth(1.5);
        glBegin(GL_LINES);
        glVertex3d(object.x, object.y, object.z);
        glVertex3d(object.scaleX, object.scaleY, object.scaleZ);
        glEnd();
        glPopMatrix();
    }
    else if(object.type == RenderObjectType::MODEL3D)
    {
        glPushMatrix();
        glMultMatrixd(GLmat);
        glScaled(object.scaleX,object.scaleY,object.scaleZ);
        glCallList((uint)object.id+4);
        glPopMatrix();
        //std::cout<<"renderphysxobject model3d\n";
    }
    else
    {

    }
}

