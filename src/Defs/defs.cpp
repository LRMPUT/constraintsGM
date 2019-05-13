/** @file defs.cpp
* @author: Dominik Belter
* Walkers definitions
*
*/

#include "Defs/defs.h"
#include <numeric>
#include <iostream>

namespace walkers{

    /// compute roll/pitch/yaw from rotation matrix
    Vec3 fromRotationMat(const Mat34& pose){
        Vec3 rpy(Vec3::Identity());
        rpy.x() = atan2(pose(2,1), pose(2,2));
        rpy.y() = -asin(pose(2,0));
        rpy.z() = atan2(pose(1,0), pose(0,0));
        return rpy;
    }

    /// compute rotation matrix from RPY angles
    Mat33 toRotationMat(const Vec3& rpy){
        Mat33 mat(Mat33::Identity());
        double fi = rpy.x(); double theta=rpy.y(); double psi = rpy.z();
        mat(0,0) = cos(theta)*cos(psi);
        mat(0,1) = sin(fi)*sin(theta)*cos(psi)-cos(fi)*sin(psi);
        mat(0,2) = cos(fi)*sin(theta)*cos(psi)+sin(fi)*sin(psi);

        mat(1,0) = cos(theta)*sin(psi);
        mat(1,1) = sin(fi)*sin(theta)*sin(psi)+cos(fi)*cos(psi);
        mat(1,2) = cos(fi)*sin(theta)*sin(psi)-sin(fi)*cos(psi);

        mat(2,0) = -sin(theta);
        mat(2,1) = sin(fi)*cos(theta);
        mat(2,2) = cos(fi)*cos(theta);
        return mat;
    }

    /// to homogenous transformation
    Mat34 toSE3(const Eigen::Vector3d& pos, const walkers::Quaternion& rot){
        Mat34 pose;
        pose(0,3) = pos(0); pose(1,3) = pos(1); pose(2,3) = pos(2);
        pose.matrix().block<3,3>(0,0) = rot.matrix();
        return pose;
    }

    /// compute rotation matrix from RPY angles
    Mat33 toRotationMat(const Quaternion& quat){
        Mat33 mat(Mat33::Identity());
        double q0 = quat.w(); double q1 = quat.x(); double q2 = quat.y(); double q3 = quat.z();
        mat(0,0) = 2*pow(q0,2.0)+2*pow(q1,2.0)-1;
        mat(0,1) = 2*q1*q2-2*q0*q3;
        mat(0,2) = 2*q1*q3+2*q0*q2;

        mat(1,0) = 2*q1*q2+2*q0*q3;
        mat(1,1) = 2*pow(q0,2.0)+2*pow(q2,2.0)-1;
        mat(1,2) = 2*q2*q3-2*q0*q1;

        mat(2,0) = 2*q1*q3-2*q0*q2;
        mat(2,1) = 2*q2*q3+2*q0*q1;
        mat(2,2) = 2*pow(q0,2.0)+2*pow(q3,2.0)-1;
        return mat;
    }

    /// compute quaternion from rotation matrix
    Quaternion rotMat2quat(const Mat34& pose){
        Quaternion quat(pose.rotation());
        return quat;
    }

    /// compute roll/pitch/yaw from quaternion
    Vec3 fromQuaternion(const Quaternion& quat){
        Vec3 rpy(Vec3::Identity());
        double q0 = quat.w(); double q1 = quat.x(); double q2 = quat.y(); double q3 = quat.z();
        rpy.x() = atan2(2*q2*q3+2*q0*q1, 2*pow(q0,2.0)+2*pow(q3,2.0)-1);
        rpy.y() = -asin(2*q1*q3-2*q0*q2);
        rpy.z() = atan2(2*q1*q2+2*q0*q3, 2*pow(q0,2.0)+2*pow(q1,2.0)-1);
        return rpy;
    }

    /// compute roll/pitch/yaw from quaternion
    Vec3 fromQuaternionRobot(const Quaternion& quat){
        Vec3 rpy(Vec3::Identity());
        double q0 = quat.w(); double q1 = quat.x(); double q2 = quat.y(); double q3 = quat.z();
        rpy.x() = atan2(2*q1*q2+2*q0*q3, 2*pow(q0,2.0)+2*pow(q1,2.0)-1);
        rpy.y() = asin(2*q1*q3-2*q0*q2);
        rpy.z() = -atan2(2*q2*q3+2*q0*q1, 2*pow(q0,2.0)+2*pow(q3,2.0)-1);
        return rpy;
    }

    void getTokenizedContent(std::string content, std::vector<double>& collectedData) {
        std::stringstream str(content);
        std::string token;
        while (str.good ()) {
            str >> token;;
            collectedData.push_back(std::stod(token));
        }
    }

    void getTokenizedContent(std::vector<std::string> content, std::vector<Vec3>& collectedData) {
        for(uint i = 0; i < content.size(); i++) {
            std::stringstream str(content.at(i));
            std::string token;
            Vec3 parameters;
            for (int j = 0; j < 3; j++) {
                str >> token;
                parameters.vector()(j) = std::stof(token);
            }
            collectedData.push_back(parameters);
        }
    }

    void getTokenizedContent(std::string content, Vec3& collectedData){
        std::stringstream str(content);
        std::string token;
        Vec3 parameters;
        for (int j = 0; j < 3; j++){
            str >> token;
            parameters.vector()(j) = std::stof(token);
        }
        collectedData = parameters;
    }

    /// returns skew-symetric matrix created from vector omega
    Mat33 skewSymetric(const Vec3& omega){
        Mat33 omegaRot(Mat33::Zero());
        omegaRot(0,1)=-omega.z(); omegaRot(0,2)=omega.y();
        omegaRot(1,0)=omega.z(); omegaRot(1,2)=-omega.x();
        omegaRot(2,0)=-omega.y(); omegaRot(2,1)=omega.x();
        return omegaRot;
    }

    /// inverse skew-symetric operator
    Vec3 invSkewSymetric(const Mat33& skewSymetric){
        return Vec3(skewSymetric(2,1),skewSymetric(0,2),skewSymetric(1,0));
    }

    /// expmap
    Mat33 expmap(const Vec3& omega){
        double theta = sqrt(pow(omega.x(),2.0)+pow(omega.y(),2.0)+pow(omega.z(),2.0));
        Mat33 omegaRot(skewSymetric(omega));
        if (theta<1e-5)
            return Mat33::Identity()+(1+(pow(theta,2.0)/6.0)+(pow(theta,4.0)/120.0))*omegaRot+(0.5-(pow(theta,2.0)/24.0)+(pow(theta,4.0)/720.0))*(omegaRot*omegaRot);
        else
            return Mat33::Identity()+(sin(theta)/theta)*omegaRot+((1-cos(theta))/pow(theta,2.0))*(omegaRot*omegaRot);
    }

    /// logmap
    Vec3 logmap(const Mat33& R){
        double theta = acos((R.trace()-1)/2);
        double coeff=1;
        if (theta<1e-5)
            coeff=1.0;
        else
            coeff = (theta/(2*sin(theta)));
        Mat33 lnR = coeff*(R-R.transpose());
        return invSkewSymetric(lnR);
    }

    /// return x y z roll pitch yaw
    Vec6 toTranslRPY(const Mat34& pose){
        Vec6 xyzRPY;
        xyzRPY(0) = pose(0,3);
        xyzRPY(1) = pose(1,3);
        xyzRPY(2) = pose(2,3);
        Vec3 rpy = fromRotationMat(pose);
        xyzRPY(3) = rpy.x(); xyzRPY(4) = rpy.y(); xyzRPY(5) = rpy.z();
        return xyzRPY;
    }

    /// Mat34 from translation and RPY
    Mat34 fromTranslRPY(const Vec6& xyzRPY){
        Mat34 pose(Mat34::Identity());
        pose.matrix().block<3,3>(0,0) = toRotationMat(Vec3(xyzRPY(3),xyzRPY(4),xyzRPY(5)));
        pose(0,3) = xyzRPY(0); pose(1,3) = xyzRPY(1); pose(2,3) = xyzRPY(2);
        return pose;
    }

    /// compute difference between current and previous pose (rpy parametrization)
    Vec6 computeDeltaPose(const Mat34& currentPose, const Mat34& prevPose){
        return toTranslRPY(currentPose) - toTranslRPY(prevPose);
    }

    double computeMean(const std::vector<double>& vector){
        double sum = std::accumulate(vector.begin(), vector.end(), 0.0);
        double mean = sum / double(vector.size());
        return mean;
    }

    double computeStd(const std::vector<double>& vector){
        double sq_sum = std::inner_product(vector.begin(), vector.end(), vector.begin(), 0.0);
        double mean = computeMean(vector);
        double stdev = std::sqrt(sq_sum / double(vector.size()) - mean * mean);
        return stdev;
    }
}
