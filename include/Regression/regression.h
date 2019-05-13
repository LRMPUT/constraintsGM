/** @file regression.h
 * Regression module interface
 * Dominik Belter
 */

#ifndef _REGRESSION_H_
#define _REGRESSION_H_

#include "Defs/defs.h"
#include "3rdParty/tinyXML/tinyxml2.h"
#include <iostream>
#include <string>
#include <vector>
#include <stdexcept>
#include <functional>

namespace regression {

    /// Regression interface
    class Regression {
        public:

            /// Robot type
            enum Type {
                /// Gaussian mixture
                TYPE_GAUSSIAN_MIXTURE
            };

            /// overloaded constructor
            Regression(const std::string _name, Type _type) : name(_name), type(_type) {}

            /// Name of the regression
            virtual const std::string& getName() const { return name; }

            /// get type
            virtual const Type& getType() const { return type; }

            /// Initialize training
            virtual void initializeTraining(void) = 0;

            /// Initialize training
            virtual void initializeTraining(const std::string& trainFilename, const std::string& verifFilename, const std::string& testFilename) = 0;

            /// train the model
            virtual void train() = 0;

            /// compute output for trained function
            virtual double computeOutput(const Eigen::MatrixXd& input, int outNo) const = 0;

            /// compute gradient of trained function
            virtual void computeGradient(const Eigen::MatrixXd& input, Eigen::MatrixXd& grad) const = 0;

            /// store results
            virtual void storeResult(std::string filename) = 0;

            /// store results
            virtual void load(std::string filename) = 0;

            /// write summary
            virtual void writeSummary(const std::string& filenameTrain, const std::string& filenameTest) = 0;

            /// Virtual descrutor
            virtual ~Regression() {}

        protected:
            /// Regreession name
            const std::string name;
            /// Regreession type
            Type type;
    };
}

#endif // _REGRESSION_H_
