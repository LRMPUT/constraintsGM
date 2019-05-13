/** @file kinematic.h
 *
 * Kinematic interface
 *
 * @author Adam Czeszejkowski
 * @author Norbert Werblinski
 *
 */
 

#ifndef _KINEMATIC_H_
#define _KINEMATIC_H_

#include "Defs/defs.h"
#include "3rdParty/tinyXML/tinyxml2.h"
#include <iostream>
#include <string>
#include <vector>

namespace controller {
    /// Kinematic interface
    class Kinematic {
        public:

            /// Kinematic type
            enum Type {
                    /// Lie groups
                    TYPE_LIE,
                    /// Denavit-Hartenberg
                    TYPE_DENAVIT_HARTNBERG,
            };

            /// overloaded constructor
            Kinematic(const std::string _name, Type _type) : name(_name), type(_type) {}
			/** Kinematic constructor
			*/

            virtual const std::string& getName() const { return name; }
            
            /** Compute forward kinematic, default (-1) -- the last joint
             *
             * 
             *@param [in] configuration Vector of displacements and joint angles
             *@param [in] linkNo Link number
             *@return Matrix of the position, orientation
             *
             */
            
            virtual walkers::Mat34 forwardKinematic(const std::vector<double>& configuration, int linkNo=-1) = 0;

             /** Compute inverse kinematic, default (-1) -- the last joint
              *
              * 
              *@param [in] linkPose Matrix of the position, orientation
              *@param [in] linkNo Link number
              *@return Vector of displacements and joint angles
              *
              */
            virtual std::vector<double> inverseKinematic(const walkers::Mat34& linkPose, unsigned int linkNo=-1) = 0;

             /** Return set of link's poses
             *
             * 
             *@param [in] configuration Vector of displacements and joint angles
             *@return Vector of Matrix of the position, orientation for each joint
             *
             */
            virtual std::vector<walkers::Mat34> getState(const std::vector<double>& configuration) = 0;

            /// Virtual descrutor
			virtual ~Kinematic() {};

        protected:
            /// Board name
            const std::string name;
            /// Board type
            Type type;
    };
}

#endif // _KINEMATIC_H_
