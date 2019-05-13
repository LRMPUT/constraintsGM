/**********************************************************
*
*		author: Tomasz Augustyn
* 
**********************************************************/

#include "CollisionDetection/CollisionDetectionFCL.h"
#include <stdexcept>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>

using namespace coldet;

/// A single instance of CollisionDetectionFCL
CollisionDetectionFCL::Ptr collisionDetectionFCL;

CollisionDetectionFCL::CollisionDetectionFCL(void) : CollisionDetection("CollisionDetectionFCL", TYPE_FCL) {

}

/// Overloaded Constructor with vectors initialization
CollisionDetectionFCL::CollisionDetectionFCL(std::string configFilename) : CollisionDetection("CollisionDetectionFCL", TYPE_FCL){
    tinyxml2::XMLDocument config;
    setlocale(LC_NUMERIC,"C");
    std::string filename = "../../resources/" + configFilename;
    config.LoadFile(filename.c_str());
    if (config.ErrorID()) {
        std::cout << "unable to load coldet config file: " << filename << ".\n";
    }
    else {
//                tinyxml2::XMLElement* coldet = config.FirstChildElement("Coldet");

//                robotConfig = coldet->FirstChildElement( "Robot" )->FirstChildElement("config")->GetText();
//                robotType = coldet->FirstChildElement( "Robot" )->FirstChildElement("type")->GetText();
    }
}

/// initialize
void CollisionDetectionFCL::initializeMeshModelWalker(const Objects3DS& _object3dsKinem, size_t modelsNo, size_t elementsNo,
                                                const std::vector<walkers::Vec3>& scales){
    object3dsKinem = _object3dsKinem;
    // create collision models
    meshModels.resize(_object3dsKinem.objects.size());
    collTable.resize(elementsNo);
    for (size_t modNo=0; modNo<_object3dsKinem.objects.size();modNo++){
        initCollisionModel(modNo, scales[modNo%modelsNo]);//
    }
    collObjects.resize(elementsNo);
    fcl::Transform3d tf(fcl::Transform3d::Identity());
    collObjects[0].reset(new fcl::CollisionObject<double>(meshModels[0], tf));
    size_t meshModelNo=1;
    for (size_t legNo=0;legNo<(elementsNo-1)/(modelsNo-1);legNo++) {//init legs
        for (size_t jointNo=0;jointNo<modelsNo-1;jointNo++) {
            collObjects[meshModelNo].reset(new fcl::CollisionObject<double>(meshModels[(legNo*(modelsNo-1)+jointNo)%(_object3dsKinem.objects.size()-1)+1], tf));
            meshModelNo++;
        }
    }
}

/// initialize
void CollisionDetectionFCL::initializeMeshModel(const Objects3DS& _object3dsKinem, size_t modelsNo, const std::vector<walkers::Vec3>& scales){
    object3dsKinem = _object3dsKinem;
    // create collision models
    meshModels.resize(modelsNo);
    collTable.resize(modelsNo);
    for (size_t modNo=0; modNo<meshModels.size();modNo++)
        initCollisionModel(modNo, scales[modNo]);//
    collObjects.resize(modelsNo);
    fcl::Transform3d tf(fcl::Transform3d::Identity());
    for (size_t modNo=0; modNo<meshModels.size();modNo++)
        collObjects[modNo].reset(new fcl::CollisionObject<double>(meshModels[modNo], tf));
}

/// initialize terrain model
void CollisionDetectionFCL::initializeTerrain(const Objects3DS& _object3dsKinem){
    if (_object3dsKinem.filenames.size()!=1||_object3dsKinem.objects.size()!=1){
        throw std::runtime_error("FCL terrain initialization failed. Incorrect number of collision models\n");
    }
    object3dsKinem.filenames.push_back(_object3dsKinem.filenames.back());
    object3dsKinem.objects.push_back(_object3dsKinem.objects.back());
    // create collision models
    meshModels.resize(meshModels.size()+1);
    collTable.resize(collTable.size()+1);
    initCollisionModel(meshModels.size()-1, walkers::Vec3(1.0,1.0,1.0));// init terrain
    collObjects.resize(collObjects.size()+1);
    fcl::Transform3d tf(fcl::Transform3d::Identity());
    collObjects[collObjects.size()-1].reset(new fcl::CollisionObject<double>(meshModels[meshModels.size()-1], tf));
}

CollisionDetectionFCL::~CollisionDetectionFCL(void) {
}

/// initialize collisions model
void CollisionDetectionFCL::initCollisionModel(size_t modelNo, const walkers::Vec3& scale){
    std::vector<fcl::Vector3<double>> vertices;
    std::vector<fcl::Triangle> triangles;
    //std::cout << "object3dsKinem.objects[modelNo].vertices.size() " << object3dsKinem.objects[modelNo].vertices.size() << "\n";
    // code to set the vertices and triangles
    if (object3dsKinem.objects[modelNo].vertices.size()==0||object3dsKinem.objects[modelNo].vertices.size()>65000){
        std::cout << "filename: " << object3dsKinem.filenames[modelNo] << "\n";
        std::cout << "num of objects: " << object3dsKinem.objects.size() << "\n";
        std::cout << "name of the objects:\n" << object3dsKinem.objects[modelNo].name << "\n";
        std::cout << "object3dsKinem.objects[modelNo].vertices.size() " << object3dsKinem.objects[modelNo].vertices.size() << "\n";
        std::cout << "something is wrong with 3ds load\n";
    }
    if (object3dsKinem.objects[modelNo].polygons.size()==0||object3dsKinem.objects[modelNo].polygons.size()>65000){
        std::cout << "filename: " << object3dsKinem.filenames[modelNo] << "\n";
        std::cout << "num of objects: " << object3dsKinem.objects.size() << "\n";
        std::cout << "name of the objects:\n" << object3dsKinem.objects[modelNo].name << "\n";
        std::cout << "object3dsKinem.objects[modelNo].polygons.size() " << object3dsKinem.objects[modelNo].polygons.size() << "\n";
        std::cout << "something is wrong with 3ds load\n";
    }
    for (size_t j=0;j<object3dsKinem.objects[modelNo].vertices.size();j++) {
        fcl::Vector3<double> vert(object3dsKinem.objects[modelNo].vertices[j].vertex.x*scale.x(), object3dsKinem.objects[modelNo].vertices[j].vertex.y*scale.y(), object3dsKinem.objects[modelNo].vertices[j].vertex.z*scale.z());
        vertices.push_back(vert);
    }
    for (size_t j=0;j<object3dsKinem.objects[modelNo].polygons.size();j++) {
        fcl::Triangle tr(object3dsKinem.objects[modelNo].polygons[j].a, object3dsKinem.objects[modelNo].polygons[j].b, object3dsKinem.objects[modelNo].polygons[j].c);
        triangles.push_back(tr);
    }
    // BVHModel is a template class for mesh geometry, for default OBBRSS template is used
    // add the mesh data into the BVHModel structure
    meshModels[modelNo].reset(new Model);
    meshModels[modelNo]->beginModel();
    meshModels[modelNo]->addSubModel(vertices, triangles);
    meshModels[modelNo]->endModel();
}

/// check collisions
bool CollisionDetectionFCL::checkCollision(const std::vector<simulator::RenderObject>& kinemObjects2render, std::vector<bool>& collisionTable){
    for (size_t colNo=0; colNo<collTable.size();colNo++)
        collTable[colNo] = false;
    int objectNo=0;
    for (auto& obj : kinemObjects2render){
        // transform is configured according to R and T
        fcl::Transform3d transform;
        transform = (fcl::Transform3d)obj.mat.matrix();
        collObjects[objectNo]->setTransform(transform);
        objectNo++;
    }
    if (object3dsKinem.filenames.back()=="terrain.map"){
        fcl::Transform3d transform(fcl::Transform3d::Identity());
        collObjects.back()->setTransform(transform);
    }

    // check all posible collisions
    for(size_t j=0; j<collObjects.size(); j++){
        for(size_t i=0; i<collObjects.size(); i++){
            if(i!=j){
                // set the collision request structure, here we just use the default setting
                fcl::CollisionRequest<double> request;
                // result will be returned via the collision result structure
                fcl::CollisionResult<double> result;
                // perform collision test
                fcl::collide(collObjects[j].get(), collObjects[i].get(), request, result);
                if (result.isCollision()){
                    collTable[j]=true; collTable[i]=true;
                }
            }
        }
    }

    collisionTable = collTable;
    for (size_t colNo=0; colNo<collTable.size();colNo++){
        if (collTable[colNo]==true)
            return true;
    }
	return false;
}

/// Check collisions
bool CollisionDetectionFCL::checkCollision(const std::vector<simulator::RenderObject>& kinemObjects2render,  std::set<int> objects2check, std::vector<bool>& collisionTable){
    for (size_t colNo=0; colNo<collTable.size();colNo++)
        collTable[colNo] = false;
    int objectNo=0;
    for (auto& obj : kinemObjects2render){
        // transform is configured according to R and T
        fcl::Transform3d transform;
        transform = (fcl::Transform3d)obj.mat.matrix();
        collObjects[objectNo]->setTransform(transform);
        objectNo++;
    }
    if (object3dsKinem.filenames.back()=="terrain.map"){
        fcl::Transform3d transform(fcl::Transform3d::Identity());
        collObjects.back()->setTransform(transform);
    }

    // check all posible collisions
    for(const auto object1 : objects2check){
        for(const auto object2 : objects2check){
            if(object1!=object2){
                // set the collision request structure, here we just use the default setting
                fcl::CollisionRequest<double> request;
                // result will be returned via the collision result structure
                fcl::CollisionResult<double> result;
                // perform collision test
                fcl::collide(collObjects[object2].get(), collObjects[object1].get(), request, result);
                if (result.isCollision()){
                    collTable[object1]=true; collTable[object2]=true;
                }
            }
        }
    }

    collisionTable = collTable;
    for(const auto objectId : objects2check){
        if (collTable[objectId]==true)
            return true;
    }
    return false;
}

std::unique_ptr<CollisionDetection> coldet::createCollisionDetectionFCL(void) {
    return walkers::make_unique<CollisionDetectionFCL>();
}

std::unique_ptr<CollisionDetection> coldet::createCollisionDetectionFCL(std::string configFile) {
    return walkers::make_unique<CollisionDetectionFCL>(configFile);
}
