/**
* @author Dominik Belter
*
*/

#ifndef COLLISIONDETECTIONFCL_H_INCLUDED
#define COLLISIONDETECTIONFCL_H_INCLUDED

#include "CollisionDetection/CollisionDetection.h"
#include "Defs/fcl.h"
//#include <fcl/fcl.h>
//#include <fcl/collision.h>
#include "tinyXML/tinyxml2.h"
#include <memory>
#include <iostream>
#include <array>

namespace coldet {
    /// create a single CollisionDetectionFCL
    std::unique_ptr<CollisionDetection> createCollisionDetectionFCL(void);
    std::unique_ptr<CollisionDetection> createCollisionDetectionFCL(std::string configFile);
}

/// CollisionDetection implementation
class CollisionDetectionFCL : public coldet::CollisionDetection {
    public:
        /// Pointer
        typedef std::unique_ptr<CollisionDetectionFCL> Ptr;
        typedef fcl::BVHModel<fcl::OBBRSS<double>> Model;

        /// Constructor
        CollisionDetectionFCL(void);

        /// Overloaded Constructor with vectors initialization
        CollisionDetectionFCL(std::string configFilename);

		/// Destructor
        ~CollisionDetectionFCL(void);

		/// Check collisions
        bool checkCollision(const std::vector<simulator::RenderObject>& kinemObjects2render, std::vector<bool>& collisionTable);
       
        /// Check collisions
        bool checkCollision(const std::vector<simulator::RenderObject>& kinemObjects2render,  std::set<int> objects2check, std::vector<bool>& collisionTable);

        /// get objects to render
        void getObjects2Render(std::vector<simulator::RenderObject>& kinemObjects, const std::vector<double>& config);

        /// initialize
        void initializeMeshModelWalker(const Objects3DS& _object3dsKinem, size_t modelsNo, size_t elementsNo,
                                 const std::vector<walkers::Vec3>& scales);

        /// initialize
        void initializeMeshModel(const Objects3DS& _object3dsKinem, size_t modelsNo, const std::vector<walkers::Vec3>& scales);

        /// initialize terrain model
        void initializeTerrain(const Objects3DS& _object3dsKinem);

    private:
        /// initialize collisions model
        void initCollisionModel(size_t modelNo, const walkers::Vec3& scale);

        /// kinematic model of the robot
        Objects3DS object3dsKinem;
        /// mesh models
        std::vector<std::shared_ptr<Model>> meshModels;  /// mesh models
        /// collision models
        std::vector<std::shared_ptr<fcl::CollisionObject<double>>> collObjects;
        /// collsion table
        std::vector<bool> collTable;
};

#endif // CollisionDetectionFCL_H_INCLUDED
