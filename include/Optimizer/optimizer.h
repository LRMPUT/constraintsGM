/// Author:
/// Dominik Belter
///

#ifndef optimizer_h
#define optimizer_h
#include <functional>
#include <map>
#include "Defs/eigen3.h"
#include "Defs/defs.h"

class Optimizer {
	public:

		// Optimizer type
        enum Type {
            TYPE_CMAES,
            TYPE_PSO
		};

    	// overloaded constructor
        Optimizer(const std::string _name, Type _type) : type(_type), name(_name), injectSolutionFlag(false), noProgressIter(0) {}

      	// Name of the optimizer
      	virtual const std::string& getName() const = 0;

      	virtual void setParameters() = 0;

        virtual void Optimize(std::map<std::string, std::function<double(const Eigen::VectorXd&)>>  funcMap, std::string function) = 0;

        /// print output
		virtual void output() = 0;

        /// get result
        virtual void injectSolution(const Eigen::VectorXd& _injectedSolution) {injectedSolution=_injectedSolution; injectSolutionFlag=true;}

        /// get result
        virtual void getResult(Eigen::VectorXd& xmin) = 0;

        /// set boundaries
        virtual void setBoundaries(const std::vector<std::pair<double,double>>& _range) = 0;

		// Virtual descrutor
		virtual ~Optimizer() {}

	protected:
        /// Optimizer type
      	Type type;

        /// Optimizer name
       	const std::string name;

        /// Injected solution
        Eigen::VectorXd injectedSolution;

        ///injectSolution
        bool injectSolutionFlag;

        /// no progres iter
        size_t noProgressIter;
};

#endif
