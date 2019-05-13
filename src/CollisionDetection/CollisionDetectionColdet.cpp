/**********************************************************
*
*		author: Tomasz Augustyn
* 
**********************************************************/

#include "CollisionDetection/CollisionDetectionColdet.h"
#include <stdexcept>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>

using namespace coldet;

/// A single instance of CollisionDetectionColdet
CollisionDetectionColdet::Ptr collisionDetectionColdet;

CollisionDetectionColdet::CollisionDetectionColdet(void) : CollisionDetection("CollisionDetectionColdet", TYPE_COLDET) {

}

/// Overloaded Constructor with vectors initialization
CollisionDetectionColdet::CollisionDetectionColdet(std::string configFilename) : CollisionDetection("CollisionDetectionColdet", TYPE_COLDET){
    tinyxml2::XMLDocument config;
    std::string filename = "../../resources/" + configFilename;
    setlocale(LC_NUMERIC,"C");
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

CollisionDetectionColdet::~CollisionDetectionColdet(void) {
}

/// initialize
void CollisionDetectionColdet::initializeMeshModelWalker(const Objects3DS& _object3dsKinem, size_t modelsNo, size_t elementsNo,
                         const std::vector<walkers::Vec3>& scales){
    object3dsKinem = _object3dsKinem;
    // create collision model
    meshModel.push_back(newCollisionModel3D());//body
    for (size_t i=0;i<(elementsNo-1)/(modelsNo-1);i++) {
        for (size_t j=0;j<modelsNo-1;j++) {
            meshModel.push_back(newCollisionModel3D());
        }
    }
    collTable.resize(meshModel.size());
    initCollisionModel(0, *meshModel[0], scales[0]);// init body
    size_t meshModelNo=1;
    for (size_t legNo=0;legNo<(elementsNo-1)/(modelsNo-1);legNo++) {//init legs
        for (size_t jointNo=0;jointNo<modelsNo-1;jointNo++) {
            initCollisionModel((legNo*(modelsNo-1)+jointNo)%(_object3dsKinem.objects.size()-1)+1, *meshModel[meshModelNo], scales[jointNo+1]);
            meshModelNo++;
        }
    }
}

/// initialize
void CollisionDetectionColdet::initializeMeshModel(const Objects3DS& _object3dsKinem, size_t modelsNo, const std::vector<walkers::Vec3>& scales){
    object3dsKinem = _object3dsKinem;
    // create collision model
    for (size_t j=0;j<modelsNo;j++)
        meshModel.push_back(newCollisionModel3D());
    collTable.resize(meshModel.size());
    for (size_t j=0;j<modelsNo;j++)
        initCollisionModel(j, *meshModel[j], scales[j]);// init body
}

/// initialize terrain model
void CollisionDetectionColdet::initializeTerrain(const Objects3DS& _object3dsKinem){
    if (_object3dsKinem.filenames.size()!=1||_object3dsKinem.objects.size()!=1){
        throw std::runtime_error("Coldet terrain initialization failed. Incorrect number of collision models\n");
    }
    object3dsKinem.filenames.push_back(_object3dsKinem.filenames.back());
    object3dsKinem.objects.push_back(_object3dsKinem.objects.back());
    // create collision models
    meshModel.push_back(newCollisionModel3D());
    collTable.resize(collTable.size()+1);
    initCollisionModel(object3dsKinem.objects.size()-1, *meshModel[meshModel.size()-1],walkers::Vec3(1.0,1.0,1.0));// init terrain
}

///create collsion model
void CollisionDetectionColdet::initCollisionModel(size_t objectNo, CollisionModel3D& model, const walkers::Vec3 scale) {
    model.setTriangleNumber((int)object3dsKinem.objects[objectNo].polygons.size());
    for (size_t j=0;j<object3dsKinem.objects[objectNo].polygons.size();j++) {
        model.addTriangle(	(float)(object3dsKinem.objects[objectNo].vertices[ object3dsKinem.objects[objectNo].polygons[j].a ].vertex.x*scale.x()), (float)(object3dsKinem.objects[objectNo].vertices[ object3dsKinem.objects[objectNo].polygons[j].a ].vertex.y*scale.y()), (float)(object3dsKinem.objects[objectNo].vertices[ object3dsKinem.objects[objectNo].polygons[j].a ].vertex.z*scale.z()),
                            (float)(object3dsKinem.objects[objectNo].vertices[ object3dsKinem.objects[objectNo].polygons[j].b ].vertex.x*scale.x()), (float)(object3dsKinem.objects[objectNo].vertices[ object3dsKinem.objects[objectNo].polygons[j].b ].vertex.y*scale.y()), (float)(object3dsKinem.objects[objectNo].vertices[ object3dsKinem.objects[objectNo].polygons[j].b ].vertex.z*scale.z()),
                            (float)(object3dsKinem.objects[objectNo].vertices[ object3dsKinem.objects[objectNo].polygons[j].c ].vertex.x*scale.x()), (float)(object3dsKinem.objects[objectNo].vertices[ object3dsKinem.objects[objectNo].polygons[j].c ].vertex.y*scale.y()), (float)(object3dsKinem.objects[objectNo].vertices[ object3dsKinem.objects[objectNo].polygons[j].c ].vertex.z*scale.z()));
    }
    model.finalize();
}

/// tranform Mat34 format to coldet format
void CollisionDetectionColdet::Mat34toTable(const walkers::Mat34& src, float * dest) const{
    for (int i=0;i<4;i++){
        for (int j=0;j<4;j++){
            dest[i+4*j]=(float)src(i,j);
        }
    }
}

/// check collisions
bool CollisionDetectionColdet::checkCollision(const std::vector<simulator::RenderObject>& kinemObjects2render, std::vector<bool>& collisionTable){
//	DrawRobot(pose, config);

    for (size_t colNo=0; colNo<collTable.size();colNo++)
        collTable[colNo] = false;

    int objectNo=0;
    for (auto& obj : kinemObjects2render){
        //std::cout << "pose " << objectNo << "\n" << obj.mat.matrix()<<"\n";
        float transform[16];
        Mat34toTable(obj.mat,transform);
        meshModel[objectNo]->setTransform (transform);
        objectNo++;
    }
    if (object3dsKinem.filenames.back()=="terrain.map"){
        float transform[16];
        Mat34toTable(walkers::Mat34::Identity(),transform);
        meshModel.back()->setTransform(transform);
    }

    // check all posible collisions
    for(size_t j=0; j<meshModel.size(); j++){
        for(size_t i=0; i<meshModel.size(); i++){
            if(i!=j){
                if (meshModel[j]->collision(meshModel[i])){
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
bool CollisionDetectionColdet::checkCollision(const std::vector<simulator::RenderObject>& kinemObjects2render,  std::set<int> objects2check, std::vector<bool>& collisionTable){
    for (size_t colNo=0; colNo<collTable.size();colNo++)
        collTable[colNo] = false;

    int objectNo=0;
    for (auto& obj : kinemObjects2render){
        //std::cout << "pose " << objectNo << "\n" << obj.mat.matrix()<<"\n";
        float transform[16];
        Mat34toTable(obj.mat,transform);
        meshModel[objectNo]->setTransform (transform);
        objectNo++;
    }
    if (object3dsKinem.filenames.back()=="terrain.map"){
        float transform[16];
        Mat34toTable(walkers::Mat34::Identity(),transform);
        meshModel.back()->setTransform(transform);
    }

    // check all posible collisions
    for(const auto& object1 : objects2check){
        for(const auto& object2 : objects2check){
            if(object1!=object2){
                if (meshModel[object1]->collision(meshModel[object2])){
                    collTable[object1]=true; collTable[object2]=true;
                }
            }
        }
    }

    collisionTable = collTable;
    for (const auto& objectId : objects2check){
        if (collTable[objectId]==true)
            return true;
    }
    return false;
}

/// get collision points
void CollisionDetectionColdet::getCollisionPoints(std::vector<walkers::Vec3>& points){
    points.clear();
    for(size_t j=0; j<meshModel.size(); j++){
        if (collTable[j]){
            float point[3];
            meshModel[j]->getCollisionPoint(point,false);
            walkers::Vec3 pp(point[0], point[1], point[2]);
            points.push_back(pp);
        }
    }
}

/// get collision points
void CollisionDetectionColdet::getCollisionTriangles(std::vector<std::vector<walkers::Vec3>>& triangles){
    triangles.clear();
    for(size_t j=0; j<meshModel.size(); j++){
        if (collTable[j]){
            float t1[9]; float t2[9];
            meshModel[j]->getCollidingTriangles(t1, t2, false);
            walkers::Vec3 p11(t1[0],t1[1],t1[2]); walkers::Vec3 p12(t1[3],t1[4],t1[5]); walkers::Vec3 p13(t1[6],t1[7],t1[8]);
            walkers::Vec3 p21(t2[0],t2[1],t2[2]); walkers::Vec3 p22(t2[3],t2[4],t2[5]); walkers::Vec3 p23(t2[6],t2[7],t2[8]);
            std::vector<walkers::Vec3> triangle1 = {p11, p12, p13};
            triangles.push_back(triangle1);
            std::vector<walkers::Vec3> triangle2 = {p21, p22, p23};
            triangles.push_back(triangle2);
        }
    }
}

std::unique_ptr<CollisionDetection> coldet::createCollisionDetectionColdet(void) {
    return walkers::make_unique<CollisionDetectionColdet>();
}

std::unique_ptr<CollisionDetection> coldet::createCollisionDetectionColdet(std::string configFile) {
    return walkers::make_unique<CollisionDetectionColdet>(configFile);
}
