/** @file insectLeg.h
*
* Leg interface for insects
*
* @author Lukasz Mejlun
* @author Wojciech Nowakowski
*/

#ifndef _INSECTLEG_H_INCLUDED
#define _INSECTLEG_H_INCLUDED

#include "leg.h"
#include <iostream>
#include <chrono>
#include <memory>
#include <mutex>

namespace controller 
{
	/** Constructor without arguments of Leg object*
	 * @return controller::Leg* indicator to the Leg object
	 */
    std::unique_ptr<Leg> createInsectLeg(void);
	/** Constructor of Leg object which argument is location of configuration file
	 * @param [in] filename relative path to acces the file
	 * @return controller::Leg* indicator to the Leg object
	 */
    std::unique_ptr<Leg> createInsectLeg(std::string filename);
}

/// Insect Leg implementation
class InsectLeg : public controller::Leg
{
	public:
		/// Pointer
		typedef std::unique_ptr<InsectLeg> Ptr;
	
		/** Constructor without arguments of Leg object*
		 * @return controller::Leg* indicator to the Leg object
		 */
		InsectLeg(void);
			
		/** Constructor of Leg object which argument is location of configuration file
		 * @param [in] configFilename relative path to acces the file
		 * @return controller::Leg* indicator to the Leg object
		 */
		InsectLeg(std::string configFilename);
	
		/// Destructor
		~InsectLeg(void);
	
		/** Returns name of the leg model
		 * @return (const std::string&) name of the leg model
		 */
		const std::string& getName() const { return name; }

		/** Compute torque in each joint for given the force applied in the foot
		* @param [in] force Indicator to the force vector which works in x,y,z axis
		* @param [in] config vector of joints parameters of leg
        * @return std::vector<double> load vector in individual nodes
		*/
        std::vector<double> computLoad(walkers::Vec3& force, std::vector<double> config);

		/** Compute torque in each joint for given the force applied in the foot
		* @param [in] force Indicator to the force vector which works in x,y,z axis
		* @param [in] config vector of joints parameters of leg
		* @param [in] is_leg_left is the leg on the left side of robot
        * @return std::vector<double> load vector in individual nodes
		*/
        std::vector<double> computLoad(walkers::Vec3& force, std::vector<double> config, bool is_leg_left);

		/** Compute forward kinematic, default (-1) -- the last joint
		* @param [in] configuration configuration variables legs
		* @param [in] linkNo the number of nodes kinematic
		* @return Mat34 homogeneous matrix legs
		*/
        walkers::Mat34 forwardKinematic(const std::vector<double>& configuration, int linkNo = -1, bool is_leg_left = false);

		/** Compute inverse kinematic, default (-1) -- the last joint
		* @param [in] linkPose homogeneous matrix legs
		* @param [in] linkNo the number of nodes kinematic
        * @return std::vector<double> configuration variables legs
		*/
        std::vector<double> inverseKinematic(const walkers::Mat34& linkPose, bool& motionPossible, int linkNo = -1);

        /// is robot reference legs poses inside robot workspace
        bool isInsideWorkspace(const walkers::Mat34& linkPose);

        /// is robot reference legs pose inside robot workspace
        bool isInsideWorkspace(const std::vector<double>& legConf) const;

        void get3DmodelNames(std::vector<std::string>& _model3dNames) {_model3dNames = model3dName;}

        /// get model scales
        void get3DmodelScales(std::vector<walkers::Vec3>& _model3dScales);

        ///get 3d model poses
        void get3DmodelPoses(std::vector<walkers::Mat34>& _model3dPoses);

    private:
		/// kinematic model of leg
        std::unique_ptr<controller::Kinematic> legKine;

        ///Model 3D parameters
        std::vector<std::string> model3dName;
        std::vector<bool> model3dFlag;
        std::vector<uint> model3dId;
        std::vector<walkers::Vec3> model3dOrientation;
        std::vector<walkers::Vec3> model3dPosition;

        /// kinematic margin GM
        std::string kinemMargGMFilename;

        /// kinematic margin GM outside workspace
        std::string kinemOutMargGMFilename;

		/// lengths of legs
        double lengths[3];

		/// Jacobian of Messor leg
        walkers::Mat33 computeJacobian_transposed(std::vector<double> config);

};


#endif	// _INSECTLEG_H_INCLUDED
