/// PSO implementation
/// @author: Dominik Belter

#ifndef PSO_H
#define PSO_H
#include <iostream>
#include <cmath>
#include <cstdio>
#include <stdlib.h>
#include <algorithm>
#include "time.h"
#include <string>
#include "eigen3.h"
#include "../3rdParty/tinyXML/tinyxml2.h"
#include "Optimizer/optimizer.h"
#include <memory>
#include <random>

std::unique_ptr<Optimizer> createPSO(void);
std::unique_ptr<Optimizer> createPSO(std::string configFile);

class PSO : public Optimizer
{
    private:
        /// dimensionality of the problem
        int N;
        /// search range for each axis
        std::vector<std::pair<double,double>> range;
        double c1,c2;
        double stopfitness;
        /// breakIfNoProgressFor
        int breakIfNoProgressFor;
        int particlesNo;
        int maxEpoch;
        int verbose;

        class Particle{
        public:
            ///particle best solution
            Eigen::VectorXd bestSolution;
            ///particle solution
            Eigen::VectorXd solution;
            /// best fitness
            double bestFitness;
            /// current fitness
            double fitness;
            /// speed
            Eigen::VectorXd speed;
        };

        std::vector<Particle> particles;
        size_t bestParticle;
        /// best fitness
        double bestFitness;
        /// speed limit
        double speedLimit;

        /// std::mt19937 mt
        std::mt19937 mt;
        std::vector<std::uniform_real_distribution<double>> distInit;
        std::uniform_real_distribution<double> distUnit;

        void moveParticles(void);

    public:
        /// Pointer
        typedef std::unique_ptr<PSO> Ptr;

        /// Construction
        PSO(void);

        /// Construction
        PSO(std::string configFilename);

        virtual const std::string& getName() const;
        /// set initial parameters
        void setParameters();
        /// optimize
        void Optimize(std::map<std::string, std::function<double(const Eigen::VectorXd&)>>  funcMap, std::string function);
        /// print results
        void output();
        /// get result
        void getResult(Eigen::VectorXd& xmin);
        /// set boundaries
        void setBoundaries(const std::vector<std::pair<double,double>>& _range);
};

#endif
