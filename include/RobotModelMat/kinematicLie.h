/** @file kinematicLie.h
 *
 * implementation - Kinematic model which uses Lie's groups
 *
 * @author Adam Czeszejkowski
 * @author Norbert Werblinski
 *
 */
 

#ifndef KINEMATIC_LIE_H_INCLUDED
#define KINEMATIC_LIE_H_INCLUDED

#include "kinematic.h"
#include <iostream>
#include <chrono>
#include <memory>
#include <mutex>

namespace controller {
     /** create a single kinematic model (Lie's groups)
     */
    std::unique_ptr<Kinematic> createKinematicLie(void);
    
     /** create a single kinematic model (Lie's groups) and load configuration from file
    */
    std::unique_ptr<Kinematic> createKinematicLie(std::string filename);
}

/// Kinematic implementation
class KinematicLie : public controller::Kinematic {
    public:
           
        /** Type Ptr definition
        */   
        
        typedef std::unique_ptr<KinematicLie> Ptr;

        /// Construction
        KinematicLie(void);

        /// Construction
		KinematicLie(std::string configFilename);

        /// Destructor
        ~KinematicLie(void);

        /// Name of the kienematic model
        const std::string& getName() const { return name; }
        
         /** Compute forward kinematic, default (-1) -- the last joint
         *
         * 
         *@param [in] configuration Vector of displacements and joint angles
         *@param [in] linkNo Link number
         *@return Matrix of the position, orientation
         *
         */
        walkers::Mat34 forwardKinematic(const std::vector<double>& configuration, int linkNo=-1);
        
          /** Compute inverse kinematic, default (-1) -- the last joint
         *
         * 
         *@param [in] linkPose Matrix of the position, orientation
         *@param [in] linkNo Link number
         *@return Vector of displacements and joint angles
         *
         */
        std::vector<double> inverseKinematic(const walkers::Mat34& linkPose, unsigned int linkNo=-1);

          /** Return set of link's poses
         *
         *
         *@param [in] configuration Vector of displacements and joint angles
         *@return Vector of Matrix of the position, orientation for each joint
         *
         */
        std::vector<walkers::Mat34> getState(const std::vector<double>& configuration);

    private:
        Eigen::Matrix4d createEMatrix(const std::vector<double>& E, double angle)
		{
			Eigen::Matrix4d e;
			Eigen::Matrix3d w_, ewO, I;
			Eigen::Vector3d w(3), v(3), T(3);
			w << E[3], E[4], E[5];
			v << E[0], E[1], E[2];
			w_ << 0, -E[5], E[4], E[5], 0, -E[3], -E[4], E[3], 0;
			I.setIdentity();
			ewO = I + w_*sin(angle) + w_*w_*(1 - cos(angle));
			T = (I - ewO)*w.cross(v);
			e.setIdentity();
			e.topLeftCorner(3, 3) = ewO;
			e.col(3) << T[0], T[1], T[2],1;
			return e;
		}
        Eigen::Matrix4d createGMatrix(const std::vector<double>& g)
		{
			Eigen::Matrix4d e;
			e.setIdentity();
			e.col(3) << g[0], g[1], g[2], 1;
			return e;
		}
		/** ksi tables
		* contains vector of ksi vectors loaded from file legModel.xml
		*/
        std::vector<std::vector<double>> ksi;
		/** g0 table
		* contains g0 vector loaded from file legModel.xml
		*/
        std::vector<double> g0;
		/** number of joints
		* loaded from file legModel.xml
		*/
		unsigned int jointsNo;
		/** Number of links
		* loaded from file legModel.xml
		*/
        unsigned int linksNo;
};

#endif // KINEMATICLIE_H_INCLUDED
