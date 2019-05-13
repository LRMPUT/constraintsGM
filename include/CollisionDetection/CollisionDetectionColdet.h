/**
* @author Dominik Belter
*
*/

#ifndef COLLISIONDETECTIONCOLDET_H_INCLUDED
#define COLLISIONDETECTIONCOLDET_H_INCLUDED

#include "CollisionDetection/CollisionDetection.h"
#include "../3rdParty/coldet/coldet.h"
#include "tinyXML/tinyxml2.h"
#include <memory>
#include <iostream>
#include <array>


namespace coldet {
	/// create a single CollisionDetectionColdet
    std::unique_ptr<CollisionDetection> createCollisionDetectionColdet(void);
    std::unique_ptr<CollisionDetection> createCollisionDetectionColdet(std::string configFile);
}

/// CollisionDetection implementation
class CollisionDetectionColdet : public coldet::CollisionDetection {
    public:
        /// Pointer
        typedef std::unique_ptr<CollisionDetectionColdet> Ptr;

        /// Constructor
        CollisionDetectionColdet (void);

        /// Overloaded Constructor with vectors initialization
        CollisionDetectionColdet(std::string configFilename);

		/// Destructor
        ~CollisionDetectionColdet (void);

        /// initialize
        void initializeMeshModelWalker(const Objects3DS& _object3dsKinem, size_t modelsNo, size_t elementsNo,
                                 const std::vector<walkers::Vec3>& scales);

        /// initialize
        void initializeMeshModel(const Objects3DS& _object3dsKinem, size_t modelsNo, const std::vector<walkers::Vec3>& scales);

        /// initialize terrain model
        void initializeTerrain(const Objects3DS& _object3dsKinem);

		/// Check collisions
        bool checkCollision(const std::vector<simulator::RenderObject>& kinemObjects2render, std::vector<bool>& collisionTable);

        /// Check collisions
        bool checkCollision(const std::vector<simulator::RenderObject>& kinemObjects2render,  std::set<int> objects2check, std::vector<bool>& collisionTable);

        /// get collision points
        void getCollisionPoints(std::vector<walkers::Vec3>& points);

        /// get collision points
        void getCollisionTriangles(std::vector<std::vector<walkers::Vec3>>& traingles);

    private:
        ///create collision model
        void initCollisionModel(size_t objectNo, CollisionModel3D& model, const walkers::Vec3 scale);
        /// tranform Mat34 format to coldet format
        void Mat34toTable(const walkers::Mat34& src, float * dest) const;

        /// kinematic model of the robot
        Objects3DS object3dsKinem;
        /// colision models
        std::vector<CollisionModel3D*> meshModel;  /// model 3DS
        /// collsion table
        std::vector<bool> collTable;
};

#endif // COLLISIONDETECTIONCOLDET_H_INCLUDED
