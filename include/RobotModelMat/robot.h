/** @file robot.h
 * Robot interface
 * Dominik Belter
 * Emil Waledziak
 * Jerzy Wiatrow
 * Maciej Okoniewski
 * Marcin Zgolinski	
 */

#ifndef _ROBOT_H_
#define _ROBOT_H_

#include "Defs/defs.h"
#include "RobotModelMat/leg.h"
#include "3rdParty/tinyXML/tinyxml2.h"
#include "Utilities/objects3DS.h"
#include "Defs/simulator_defs.h"
#include "Optimizer/cmaes.h"
#include "CollisionDetection/CollisionDetectionColdet.h"
#ifdef BUILD_WITH_FCL
#include "CollisionDetection/CollisionDetectionFCL.h"
#endif
#include <iostream>
#include <string>
#include <vector>
#include <stdexcept>
#include <functional>

namespace walkers {

    /// Kinematic interface
    class Robot {
        public:

        class Statistics{
            public:
            size_t collCheckGMNo;
            size_t collCheckFCLNo;
            size_t stabCheckNo;
            size_t kinemMargCheckNo;
            size_t workspaceCheckNo;
            size_t footholdsNo;
            size_t postureOptNo;
            size_t collisionWithGroundNo;
            size_t dist2workspaceNo;

            Statistics(void): collCheckGMNo(0), collCheckFCLNo(0), stabCheckNo(0), kinemMargCheckNo(0), workspaceCheckNo(0),
            footholdsNo(0), postureOptNo(0), collisionWithGroundNo(0), dist2workspaceNo(0){}

            void reset(void){
                collCheckGMNo = 0; collCheckFCLNo = 0; stabCheckNo = 0;
                kinemMargCheckNo = 0; workspaceCheckNo = 0; footholdsNo = 0;
                postureOptNo = 0; collisionWithGroundNo = 0; dist2workspaceNo = 0;
            }
        };

            /// Robot type
            enum Type {
                /// Messor Robot
                TYPE_MESSOR,
                /// Messor2 Robot
                TYPE_MESSOR2,
                /// StarlETH robot
                TYPE_STARLETH,
                /// Galgo robot
                TYPE_GALGO,
                /// PhantomX robot
                TYPE_PHANTOMX,
                /// Atlas robot
                TYPE_ATLAS,
                /// Anymal robot
                TYPE_ANYMAL

            };

            /// Leg Motion type
            enum LegMotionType {
                /// w.r.t. neutral leg configuration
                LEG_MOTION_NEUTRAL,
                /// w.r.t. current leg configuration
                LEG_MOTION_CURRENT,
                /// w.r.t. given leg configuration
                LEG_MOTION_GIVEN_CONF,
            };

            /// Constraint type
            enum ConstraintType {
                /// No constraint violated
                CONSTRAINT_NO,
                /// Outside workspace
                CONSTRAINT_WORKSPACE,
                /// Self collisions
                CONSTRAINT_SELF_COLLISIONS,
                /// Collisions between neighbouring legs
                CONSTRAINT_NEIGH_COLLISIONS,
                /// Collisions with ground
                CONSTRAINT_GROUND_COLL,
            };

            /// overloaded constructor
            Robot(const std::string _name, Type _type);

            /// Name of the robot model
            virtual const std::string& getName() const { return name; }

            /// type of the robot model
            virtual const Type& getType() const { return type; }

            ///Compute configuration of the leg for the reference motion of the platform
            /**
            * @param motion - specified motion
            * @return reference values for servomotors
            */
            //virtual std::vector<double> computeLegConfiguration(int legNo, const walkers::Mat34& bodyMotion, const std::vector<double>& startConfiguration, bool& isMotionPossible) = 0;
            ///Compute configuration of the leg for the reference motion of the platform
            /**
            * @param motion - specified motion
            * @return reference values for servomotors
            */
            std::vector<double> computeLegConfiguration(int legNo, const walkers::Mat34& bodyMotion, const std::vector<double>& startConfiguration, bool& isMotionPossible);

            ///Compute configuration of the leg for the pose defined in the platform coordinates
            /**
            * @param motion - specified pose
            * @return reference values for servomotors
            */
            std::vector<double> computeLegConfiguration(int legNo, const walkers::Mat34& pose, bool& isMotionPossible);

            /// new method: computes forward kinematics for each leg and returns position of each link of the robot (body is [0,0,0]^T)
            //virtual std::vector<walkers::Mat34> computeLinksPosition(const std::vector<double>& configuration) = 0;
            /// new method: computes forward kinematics for each leg and returns position of each link of the robot (body is [0,0,0]^T)
            std::vector<walkers::Mat34> computeLinksPosition(const std::vector<double>& configuration);

            //virtual walkers::Mat34 legCPos(const std::vector<double>& configuration, int legNo) = 0;
            walkers::Mat34 legCPos(const std::vector<double>& configuration, int legNo);

            /// returns foot pose in Leg coordinate system
            walkers::Mat34 getLegMountPoint(size_t legNo);

            /// returns neutral position of the foot
            //virtual walkers::Mat34 footPose(const walkers::Mat34& robotPose, int legNo) = 0;
            /// returns neutral position of the foot
            walkers::Mat34 footPose(const walkers::Mat34& robotPose, int legNo);

            /// returns current position of the foot
            //virtual walkers::Mat34 footPose(const walkers::Mat34& robotPose, const std::vector<double>& conf, int legNo) = 0;
            walkers::Mat34 footPose(const walkers::Mat34& robotPose, const std::vector<double>& conf, int legNo);

            /// returns foot pose in Leg coordinate system
            //virtual walkers::Mat34 footPose(const walkers::Mat34& robotPose, const std::vector<double>& conf, int legNo) = 0;
            walkers::Mat34 footPose(const walkers::Mat34& robotPoseGlob, const walkers::Mat34& footPoseGlob, int legNo){
                return (robotPoseGlob*legMountPoints[legNo]).inverse()*footPoseGlob;
            }

            /// returns foot pose in GLOBAL coordinate system
            walkers::Mat34 footPoseGlob(const walkers::Mat34& robotPoseGlob, const walkers::Mat34& footPoseLeg, int legNo){
                return robotPoseGlob*legMountPoints[legNo]*footPoseLeg;
            }

            virtual void load3Dobjects(Objects3DS& _objects3DS) = 0;

            virtual size_t getLegsNo(void) const {return (size_t)legsNo;}
            virtual size_t getLegJointsNo(size_t legNo) const {return legModels[legNo]->getJointsNo();}
            virtual size_t getLegLinksNo(size_t legNo) const {return legModels[legNo]->getLinksNo();}

            /// get kinematic objects
            virtual std::vector<simulator::RenderObject> getObjectsToRender(const std::vector<double>& configuration) const = 0;

            /// get 3d model scale
            virtual walkers::Vec3 getModelScale() const {return model3dScale;}

            /// get 3d model scale
            walkers::Vec3 getLegModelScale(size_t legNo, size_t linkNo) const {return legModels[legNo]->getLegModelScale(linkNo);}

            /// get joint limits
            std::pair<double,double> getJointLimits(size_t legNo, size_t jointNo) const {
                if (legNo>=(size_t)legsNo){
                    throw std::runtime_error("incorrect number of leg.\n");
                }
                return legModels[legNo]->getJointLimits(jointNo);
            }

            /// is robot reference legs poses inside robot workspace (leg coord)
            bool isInsideWorkspaceLeg(int legNo, const walkers::Mat34& linkPose) const{
                stats.workspaceCheckNo++;
                return legModels[legNo]->isInsideWorkspace(linkPose);
            }

            /// is robot reference legs poses inside robot workspace (robot coord)
            bool isInsideWorkspaceRobot(int legNo, const walkers::Mat34& linkPose) const{
                stats.workspaceCheckNo++;
                return legModels[legNo]->isInsideWorkspace(legMountPoints[legNo].inverse()*linkPose);
            }

            /// is robot reference legs poses inside robot workspace (global coord)
            bool isInsideWorkspaceGlobal(int legNo, const walkers::Mat34& robotPose, const walkers::Mat34& linkPose) const{
                stats.workspaceCheckNo++;
                return legModels[legNo]->isInsideWorkspace((robotPose*legMountPoints[legNo]).inverse()*linkPose);
            }

            /// is robot reference legs poses inside robot workspace (global coord)
            bool isInsideWorkspaceGlobal(const walkers::Mat34& robotPose, const std::vector<walkers::Mat34>& feetPoses);

            /// is robot reference legs pose inside robot workspace
            bool isInsideWorkspace(size_t legNo, const std::vector<double>& legConf) const{
                stats.workspaceCheckNo++;
                return legModels[legNo]->isInsideWorkspace(legConf);
            }

            /// compute forward kinematic for the robot
            std::vector<Mat34> forwardKinematic(const std::vector<double>& conf) const;

            /// compute forward kinematic
            Mat34 forwardKinematic(size_t legNo, const std::vector<double>& legConf) const{
                int side = 0;//(legNo<3) ? 0 : 1;
                return legModels[legNo]->forwardKinematic(legConf, -1, side);
            }

            /// compute forward kinematic in body coordinate system
            Mat34 forwardKinematicBody(size_t legNo, const std::vector<double>& legConf) const{
                int side = 0;
                return legMountPoints[legNo]*legModels[legNo]->forwardKinematic(legConf, -1, side);
            }

            /// get neutral pose of the foot
            walkers::Mat34 getFootNeutralPose(size_t legNo) const {
                return feetNeutralPosition[legNo];
            }

            /// compute inverse kinematic
            std::vector<double> inverseKinematicLeg(size_t legNo, bool& isMotionPossible, const walkers::Mat34& footPose) const{
                return legModels[legNo]->inverseKinematic(footPose, isMotionPossible, -1);
            }

            /// compute inverse kinematic
            std::vector<double> inverseKinematicRobot(size_t legNo, bool& isMotionPossible, const walkers::Mat34& footPose) const{
                return legModels[legNo]->inverseKinematic(legMountPoints[legNo].inverse()*footPose, isMotionPossible, -1);
            }

            /// compute inverse kinematic
            std::vector<double> inverseKinematicGlobal(size_t legNo, bool& isMotionPossible, const walkers::Mat34& robotPose, const walkers::Mat34& footPose) const{
                return legModels[legNo]->inverseKinematic((robotPose*legMountPoints[legNo]).inverse()*footPose, isMotionPossible, -1);
            }

            /// compute inverse kinematic
            std::vector<double> inverseKinematicGlobal(bool& isMotionPossible, const walkers::Mat34& robotPose, const std::vector<walkers::Mat34>& feetPoses) const;

            /// check collision for leg
            virtual bool checkCollision(size_t legNo, const std::vector<double>& conf) = 0;

            /// check collision for the robot
            virtual bool checkCollision(const std::vector<double>& conf, ConstraintType& constrViolated) = 0;

            /// check collision for two legs
            virtual bool checkCollision(size_t leg1No, const std::vector<double>& conf1, size_t leg2No, const std::vector<double>& conf2) = 0;

            /// check collision for two legs
            virtual bool checkCollision(size_t leg1No, const std::vector<double>& conf1, size_t leg2No, const std::vector<double>& conf2, ConstraintType& constrViolated) = 0;

            /// check collision for the robot
            bool checkCollision(const std::vector<double>& conf);

            /// compute kinematic margin for the leg (considers workspace and collisions)
            double kinematicMargin(size_t legNo, const std::vector<double>& conf);

            /// compute kinematic margin for the robot (considers workspace and collisions)
            double kinematicMargin(const std::vector<double>& conf);

            /// compute kinematic margin for the leg (considers workspace and collisions)
            double kinematicMargin(size_t legNo, const walkers::Mat34& footPose);

            /// compute distance to wrokspace for the leg
            double distance2workspace(size_t legNo, const walkers::Mat34& footPose);

            /// get kinematic range for the leg
            std::vector<std::pair<double,double>> getJointLimits(size_t legNo) const {
                return legModels[legNo]->getJointLimits();
            }

            /// get statistics
            Statistics getStats(void);

            /// Virtual descrutor
            virtual ~Robot() {}

        protected:
            /// Robot name
            const std::string name;
            /// Robot type
            Type type;

            /// legs number
            int legsNo;

            /// Mounting points of legs
            std::vector<walkers::Mat34> legMountPoints;

            /// Feet neutral position
            std::vector<walkers::Mat34> feetNeutralPosition;

            /// leg models
            std::vector<std::shared_ptr<controller::Leg>> legModels;

            std::vector<double> configurationStart;
            std::vector<double> configurationCurr;

            RobotState currentStateOpt;
            RobotState prevStateOpt;
            std::vector<size_t> movingLegsOpt;
            /// optimizer inverse kinematic
            std::unique_ptr<Optimizer> optimizerInvKin;
            /// optimizer inverse kinematic
            std::unique_ptr<Optimizer> optimizerPosture;
            /// 3D model scale
            walkers::Vec3 model3dScale;
            ///collision detection
            std::string coldetConfig;
            std::string coldetType;
            std::unique_ptr<coldet::CollisionDetection> collisionChecker;
            ///mass of the body
            double massBody;

            /// statistics
            mutable Statistics stats;

            /// check reference values
            void checkReferenceAngles(std::vector<double>& refValues) const;

            /// compute fitness
            double checkLegsMotion(const Mat34& bodyMotion, const walkers::RobotState& currentState, const walkers::RobotState& prevState, const std::vector<size_t>& movingLegs);

            double checkLegsMotionOpt(const Eigen::VectorXd& x);

            /// compute area of the trianlge
            double triangleArea2D(const Mat34& vertA, const Mat34& vertB, const Mat34& vertC) const;

            /// clear stats
            void clearStats(void);
    };
}

#endif // _ROBOT_H_
