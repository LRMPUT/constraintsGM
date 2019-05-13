/// CMA-ES implementation
/// @author: Dominik Belter

#ifndef CMAES_H
#define CMAES_H
#include <iostream>
#include <cmath>
#include <cstdio>
#include <stdlib.h>     
#include <algorithm>
#include "time.h"
#include <string>
#include "Defs/eigen3.h"
#include "../3rdParty/tinyXML/tinyxml2.h"
#include "Optimizer/optimizer.h"
#include <memory>
#include <random>

std::unique_ptr<Optimizer> createCMAES(void);
std::unique_ptr<Optimizer> createCMAES(std::string configFile);

class Cmaes : public Optimizer
{
	private:
        /// dimensionality of the problem
		int N;
        /// search range for each axis
        std::vector<std::pair<double,double>> range;
		double sigma;
		double stopfitness;
		double stopeval;
		int counteval;
		int eigeneval;
        /// breakIfNoProgressFor
        int breakIfNoProgressFor;
		double chiN;
		int lambda;
		int mu;
		double mueff;
		double cc, cs, c1, cmu, cmu_a, cmu_b, damps;
		double ps_norm, hsig;
        int verbose;
		
        Eigen::VectorXd xmean;
        Eigen::VectorXd bestSolution;
        double bestFitness;
        Eigen::VectorXd weights;
        Eigen::MatrixXd arindex;
        Eigen::MatrixXd arx;
        Eigen::MatrixXd arfitness;
        Eigen::MatrixXd arx_N_x_mu;
        Eigen::VectorXd xold;
        Eigen::VectorXd pc; // evolution paths for C and sigma
        Eigen::VectorXd ps;
        Eigen::MatrixXd B; // B defines the coordinate system
        Eigen::VectorXd D; // diagonal D defines the scaling
        Eigen::MatrixXd C; // covariance matrix C
        Eigen::MatrixXd invsqrtC;

        Eigen::MatrixXd sort(Eigen::MatrixXd& table, Eigen::MatrixXd& index);
        std::pair<Eigen::MatrixXd, Eigen::VectorXd> eig(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B);
        void genOffspring(std::map<std::string, std::function<double(const Eigen::VectorXd&)>>  funcMap, std::string function);
		void updateXmean();
		void updateEvoPath();
		void updateMatrixC();
		void updateOtherMatrices();
		
	public:
		/// Pointer
		typedef std::unique_ptr<Cmaes> Ptr;

       	/// Construction
       	Cmaes(void);

       	/// Construction
        Cmaes(std::string configFilename);

		virtual const std::string& getName() const;
      	void setParameters();
        void Optimize(std::map<std::string, std::function<double(const Eigen::VectorXd&)>>  funcMap, std::string function);
        /// print results
		void output();
        /// get result
        void getResult(Eigen::VectorXd& xmin);
        /// set boundaries
        void setBoundaries(const std::vector<std::pair<double,double>>& _range);
};

#endif
