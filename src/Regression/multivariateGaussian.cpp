//
//  @ Project : Gaussian Mixture Regression library
//  @ File Name : MultivariateGaussian.cpp
//  @ Date : 2009-07-16
//  @ Author : Dominik Belter
//
//

#include "Regression/multivariateGaussian.h"
#include <chrono>
#include <random>
#include <fstream>

/// default constructor
MultivariateGaussian::MultivariateGaussian(){
    long int seed = std::chrono::system_clock::now().time_since_epoch().count();
    generator.seed(seed);
}

/// default destructor
MultivariateGaussian::~MultivariateGaussian(){
}

/// initialize
void MultivariateGaussian::setParameters(unsigned int _dim){
    dim=_dim;
    gaussians.resize(dim);
    //gaussianBoundariesChange.resize(dim);
}

/// set boundaries of function's parameters
void MultivariateGaussian::setBoundaries(const std::vector<std::pair<double,double>>& _gaussianBoundaries, const std::vector<std::pair<double,double>>& _inputDomain){
    //gaussianBoundaries = _gaussianBoundaries;
    //inputDomain = _inputDomain;
    initializeGauss(_gaussianBoundaries, _inputDomain);
}

/// initialize 1D Gaussian
void MultivariateGaussian::initializeGauss(const std::vector<std::pair<double,double>>& _gaussianBoundaries, const std::vector<std::pair<double,double>>& _inputDomain){
    gaussians.resize(dim);
    for (size_t i=0;i<dim;i++){
        std::uniform_real_distribution<double> distributionCentroid (_inputDomain[i].first,_inputDomain[i].second);
        std::uniform_real_distribution<double> distributionWidth (_gaussianBoundaries[i].first, _gaussianBoundaries[i].second);
        gaussians[i].setParameters(distributionCentroid(generator), distributionWidth(generator));

		//PSO
        gaussians[i].modifyPositionChange(0,0);
        //gaussianBoundariesChange[i]=0;
	}
}

/// compue value of gene point must be row vector
double MultivariateGaussian::computeValue(const Eigen::MatrixXd& point) const{
    double result=0;
    for (size_t i=0;i<dim;i++){
        result+=gaussians[i].computeValue(point(0,i));
    }
    result/=double(dim);
    return exp(result);
}

/// compue gradient. Point must be column vector
void MultivariateGaussian::computeGradient(const Eigen::MatrixXd& point, Eigen::MatrixXd& grad) const{
    for (size_t i=0;i<dim;i++){
        grad(0,i) = gaussians[i].computeDerivative(point(0,i));
    }
    grad = computeValue(point)*grad;
}

///save function to file
void MultivariateGaussian::save2file(std::ofstream& ofstr, int type){
    ofstr << "exp((";
    for (size_t i =0;i<dim;i++) {
        gaussians[i].save2file(ofstr,(unsigned int)i, type);
		if (i<(dim-1)){
            ofstr << "+";
		}
	}
    ofstr << ")/"<< dim << ")";
}

/// set gaussians
void MultivariateGaussian::setGaussians(const std::vector<double> params){
    for (size_t i=0;i<dim;i++){
        gaussians[i].setParameters(params[i*2+0], params[i*2+1]);
    }
}
