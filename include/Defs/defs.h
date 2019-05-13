/** @file defs.h
* @author: Dominik Belter
* Walkers definitions
*
*/

#ifndef DEFS_H_INCLUDED
#define DEFS_H_INCLUDED

#include <cstdint>
#include <vector>
#include <memory>
#include <cmath>
#include "eigen3.h"
#include <iostream>

/// putslam name space
namespace walkers {

    /// 3 element vector class
    typedef Eigen::Translation<double,3> Vec3;

    /// Matrix representation of SO(3) group of rotations
    typedef Eigen::Matrix<double,3,3> Mat33;

    /// Vec6
    typedef Eigen::Matrix<double,6,1> Vec6;

    /// Quaternion representation of SO(3) group of rotations
    typedef Eigen::Quaternion<double> Quaternion;

    /// Homogeneous representation of SE(3) rigid body transformations
    typedef Eigen::Transform<double, 3, Eigen::Affine> Mat34;

    /// configuration of the robot
    typedef std::vector<double> RobotConfiguration;

    /// Point cloud
    typedef std::vector<Vec3> PointVec;

    /// compute roll/pitch/yaw from rotation matrix
    Vec3 fromRotationMat(const Mat34& pose);

    /// to homogenous transformation
    Mat34 toSE3(const Eigen::Vector3d& pos, const walkers::Quaternion& rot);

    /// compute rotation matrix from RPY angles
    Mat33 toRotationMat(const Vec3& rpy);

    /// compute rotation matrix from RPY angles
    Mat33 toRotationMat(const Quaternion& quat);

    /// compute quaternion from rotation matrix
    Quaternion rotMat2quat(const Mat34& pose);

    /// compute roll/pitch/yaw from quaternion
    Vec3 fromQuaternion(const Quaternion& quat);

    /// compute roll/pitch/yaw from quaternion (in robot frame)
    Vec3 fromQuaternionRobot(const Quaternion& quat);

    /// get content from string
    void getTokenizedContent(std::string content, std::vector<double>& collectedData);

    /// get content from string
    void getTokenizedContent(std::vector<std::string> content, std::vector<Vec3>& collectedData);

    /// get content from string
    void getTokenizedContent(std::string content, Vec3& collectedData);

    /// returns skew-symetric matrix created from vector omega
    Mat33 skewSymetric(const Vec3& omega);

    /// inverse skew-symetric operator
    Vec3 invSkewSymetric(const Mat33& skewSymetric);

    /// expmap
    Mat33 expmap(const Vec3& omega);

    /// logmap
    Vec3 logmap(const Mat33& R);

    /// return x y z roll pitch yaw
    Vec6 toTranslRPY(const Mat34& pose);

    /// Mat34 from translation and RPY
    Mat34 fromTranslRPY(const Vec6& xyzRPY);

    /// compute difference between current and previous pose (rpy parametrization)
    Vec6 computeDeltaPose(const Mat34& currentPose, const Mat34& prevPose);

    /// joint info for observer
    struct JointInfo{
        int type;
        //Mat34 pos_mat;
        Vec3 anchor, pt1, pt2;
        Vec3 axis;
        JointInfo(){}
    };

    struct ObjectInfo{
        int type;
        int modelID;
        Mat34 pos_mat;
        Vec3 scale;
        double radius;
        ObjectInfo(){}
        void print(){
            std::cout<<"ObjectODE id: "<<modelID; std::cout<<" scale "<<scale.x()<<", "<<scale.y()<<", "<<scale.z()<<std::endl;
        }
    };

    class TorqueForce {
    public:
        ///force values
        Vec3 force;

        ///torque values
        Vec3 torque;

        ///susceptibility values
        std::vector<double> susceptibility;

    };

    class RobotState {
    public:
        /// set of Robot States
        typedef std::vector<RobotState> Seq;

        /// construction
        RobotState(void){
        }

        /// construction
        RobotState(size_t _jointsNo, size_t _legsNo) : jointsNo(_jointsNo), legsNo(_legsNo),
            refValues(jointsNo,0), currentValues(jointsNo,0), refSpeed(jointsNo), contactSensors(legsNo,0){
        }

        /// joints No
        size_t jointsNo;
        /// legs No
        size_t legsNo;
        /// ref values
        std::vector<double> refValues;
        /// measured values
        std::vector<double> currentValues;
        /// reference joint speed
        std::vector<double> refSpeed;
        /// contact sensors
        std::vector<bool> contactSensors;
    };

    double computeMean(const std::vector<double>& vector);

    double computeStd(const std::vector<double>& vector);

    template<typename T, typename ...Args>
    std::unique_ptr<T> make_unique( Args&& ...args ){
        return std::unique_ptr<T>( new T( std::forward<Args>(args)... ) );
    }

//    template<typename T, typename ...Args>
//    std::shared_ptr<T> make_shared( Args&& ...args ){
//        return std::shared_ptr<T>( new T( std::forward<Args>(args)... ) );
//    }
}

#endif // DEFS_H_INCLUDED
