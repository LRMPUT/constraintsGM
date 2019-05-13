//
//
//
//  @ Project : Approximation library Untitled
//  @ File Name : Individual.h
//  @ Date : 2009-07-16
//  @ Author : Dominik Belter
//
//
#ifndef _INDIVIDUAL_H
#define _INDIVIDUAL_H

#include "multivariateGaussian.h"
#include <iostream>

class Individual {
public:
	/// default constructor
    Individual();
	/// default destructor
    ~Individual();
	/// initialize
    void setParameters(unsigned int _gaussiansNo, unsigned int dim, unsigned int _outputsNo);
	/// set boundaries of function's parameters
    void setBoundaries(const std::vector<std::pair<double,double>>& _gaussianBoundaries, const std::vector<std::pair<double,double>>& _inputDomain);
    /// set boundaries of function's parameters
    void setGaussianBoundaries(const std::vector<std::pair<double,double>>& _gaussianBoundaries);
	/// create random chromosome
    void createChromosome();
	/// create gene
    void initializeGaussian(int gaussianNo);
    /// PSO change the best position
    void changeBestPosition();
	/// move individual
    void moveIndividual(const Individual& bestParticle);
	/// compute value of polynomial represented by individual; point must be a column vector
    double computeValue(const Eigen::MatrixXd& point, const Eigen::MatrixXd& coefficient, int outNo) const;
    /// compute gradient of polynomial represented by individual; point must be a column vector
    void computeGradient(const Eigen::MatrixXd& point, Eigen::MatrixXd& grad, const Eigen::MatrixXd& coefficient, int outNo) const;
    /// compute value of polynomial represented by individual; point must be a column vector
    double computeValue(int gaussNo, const Eigen::MatrixXd& point) const;
	/// compute fitness value
    double computeFitness(const Eigen::MatrixXd& points, const Eigen::MatrixXd& expectedOutput, const Eigen::MatrixXd& polyCoef);
	/// get gene value
    MultivariateGaussian getGeneValue(unsigned int geneNo);
	/// set gene value
    void setGeneValue(const MultivariateGaussian& gene, unsigned int geneNo);
	///save function to file
    void save2file(const std::string& filename, const Eigen::MatrixXd& coefficient, int type);
    /// Get fitness value
    inline double getFitnessValue() const {return fitnessValue;}
    /// Get Gaussian Boundaries
    inline std::vector<std::pair<double,double>> getGaussianBoundaries() const {return gaussianBoundaries;}
    /// Get Gaussians no
    inline int getGaussiansNo() const {return gaussiansNo;}
    /// set individual
    void setIndividual(const Eigen::VectorXd& x);
    ///store solution
    void store(const std::string& filename, const Eigen::MatrixXd& coefficient, const std::vector<std::pair<double,double>>& outputDomain);
    ///store solution
    void load(const std::string& filename, Eigen::MatrixXd& coefficient, std::vector<std::pair<double,double>>& outputDomain);

private:
    /// Random number generator
    std::default_random_engine generator;
    ///number of elements in polynomial
    int gaussiansNo;
    /// chromosome
    std::vector<MultivariateGaussian> chromosome;
    /// chromosome
    std::vector<MultivariateGaussian> bestPosition;
    ///boundaries n-coefficient for functions - two-column vector [min, max]
    std::vector<std::pair<double,double>> gaussianBoundaries;
    ///boundaries a-coefficient for functions - two-column vector [min, max]
    std::vector<std::pair<double,double>> inputDomain;
    /// fitness value
    double fitnessValue;
    /// best fitness value
    double bestFitnessValue;
    /// dimension of the approximation space
    unsigned int dim;
    /// number of outputs
    unsigned int outputsNo;
    ///PSO - c1
    double c1;
    ///PSO - c2
    double c2;
    ///max change of the particle position [0-1]
    double maxChange;
    /// gaussianBoundaries change
    std::vector<double> gaussianBoundariesChange;
    /// best gaussianBoundaries value
    std::vector<double> bestGaussianBoundaries;
};

#endif  //_Individual_H
