/**
* @author Dominik Belter
*
*/

#ifndef _COLLISIONDETECTION_H_
#define _COLLISIONDETECTION_H_

#include "Defs/defs.h"
#include "Defs/simulator_defs.h"
#include <iostream>
#include <string>
#include <vector>
#include <set>
#include "Utilities/3dsloader.h"
#include "Utilities/objects3DS.h"

namespace coldet{
	//CollisionDetection interface
	class CollisionDetection {
        public:
            /// CollisinDetection type
            enum Type {
                /// based on coldet library (see 3rdParty folder)
                TYPE_COLDET,
                /// based on the voxels
                TYPE_VOXEL,
                /// based on the FCL library (https://github.com/flexible-collision-library/fcl)
                TYPE_FCL,
            };

			/// overloaded constructor
            CollisionDetection(const std::string _name, Type _type) : name(_name), type(_type) {}

			/// Name of the structure
			virtual const std::string& getName() const {
                return name;
			}

            /// Name of the structure
            virtual const Type& getType() const {
                return type;
            }

			/// Initialize robot structure
//			virtual void initStructures(void) = 0;

			/// Draw robot using openGL
//            virtual void GLDrawRobot (const walkers::Mat34& pose, const std::vector<double>& config,  std::vector<bool>& collision_table) const = 0;

			/// Check collisions
            virtual bool checkCollision(const std::vector<simulator::RenderObject>& kinemObjects2render,  std::vector<bool>& collisionTable) = 0;

            /// Check collisions
            virtual bool checkCollision(const std::vector<simulator::RenderObject>& kinemObjects2render,  std::set<int> objects2check, std::vector<bool>& collisionTable) = 0;

            /// initialize
            virtual void initializeMeshModelWalker(const Objects3DS& _object3dsKinem, size_t modelsNo, size_t elementsNo,
                                     const std::vector<walkers::Vec3>& scales) = 0;
            /// initialize
            virtual void initializeMeshModel(const Objects3DS& _object3dsKinem, size_t modelsNo, const std::vector<walkers::Vec3>& scales) = 0;

            /// initialize terrain model
            virtual void initializeTerrain(const Objects3DS& _object3dsKinem) = 0;

            /// Virtual descrutor
            virtual ~CollisionDetection() {}

        protected:
            /// CollisonDetection name
            const std::string name;
            /// CollisonDetection type
            Type type;
	};
}

#endif // _COLLISIONDETECTION_H_
