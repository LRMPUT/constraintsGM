/** @file leg.h
 *
 * Leg interface
 *
 */

#ifndef _LEG_H_
#define _LEG_H_

#include "Defs/defs.h"
#include "RobotModelMat/kinematic.h"
#include "3rdParty/tinyXML/tinyxml2.h"
#include <iostream>
#include <string>
#include <vector>

namespace controller {
    /// Kinematic interface
    class Leg {
        public:

            /// Leg type
            enum Type {
                /// Insect
                TYPE_INSECT,
                /// Mammal
                TYPE_MAMMAL,
            };

            /// overloaded constructor
            Leg(const std::string _name, Type _type) : name(_name), type(_type) {}

            /// Name of the leg model
            virtual const std::string& getName() const { return name; }

            ///Compute torque in each joint for given the force applied in the foot
            virtual std::vector<double> computLoad(walkers::Vec3& force, std::vector<double> config) = 0;

            ///Compute torque in each joint for given the force applied in the foot
            virtual std::vector<double> computLoad(walkers::Vec3& force, std::vector<double> config, bool is_leg_left) = 0;

            /// Compute forward kinematic, default (-1) -- the last joint
            virtual walkers::Mat34 forwardKinematic(const std::vector<double>& configuration, int linkNo=-1, bool is_leg_left = false) = 0;

            /// Compute forward kinematic, default (-1) -- the last joint
            virtual std::vector<double> inverseKinematic(const walkers::Mat34& linkPose, bool& isMotionPossible, int linkNo=-1) = 0;

            /// is robot reference legs poses inside robot workspace
            virtual bool isInsideWorkspace(const walkers::Mat34& linkPose) = 0;

            /// is robot reference legs pose inside robot workspace
            virtual bool isInsideWorkspace(const std::vector<double>& legConf) const = 0;

            /** Returns number of links in leg
            * @return int number of links
            */
            int getLinksNo() const { return linksNo; }

            ///get mass of the link
            double getLinkMass(size_t linkNo) const {
                if (linkNo<(size_t)linksNo)
                    return massLinks[linkNo];
                else
                    throw std::runtime_error("incorrect number of joint.\n");
                return 0;
            }

            /** Returns number of joints in leg
            * @return int number of joints
            */
            int getJointsNo() const { return jointsNo; }

            /// get 3d model scale
            walkers::Vec3 getLegModelScale(size_t linkNo) {return model3dScale[linkNo];}

            void get3DmodelScales(std::vector<walkers::Vec3>& _model3dScales) {_model3dScales = model3dScale;}

            /// get joint limits
            std::pair<double,double> getJointLimits(size_t jointNo){
                if (jointNo>=(size_t)jointsNo)
                    throw std::runtime_error("incorrect number of joint.\n");
                return jointLimits[jointNo];
            }

            /// get joint limits
            std::vector<std::pair<double,double>> getJointLimits(void) const{
                return jointLimits;
            }

            inline double getFootholdSearchRange(void) {return footholdSearchRange;}

            /// Virtual descrutor
            virtual ~Leg() {}

        protected:
            /// Leg name
            const std::string name;
            /// Leg type
            Type type;

            /// number of joints
            int jointsNo;

            /// number of links
            int linksNo;

            /// footholdSearchRange
            double footholdSearchRange;

            /// scale 3D model
            std::vector<walkers::Vec3> model3dScale;

            /// joint limits
            std::vector<std::pair<double,double>> jointLimits;
            /// mass of links
            std::vector<double> massLinks;
    };
}

#endif // _LEG_H_
