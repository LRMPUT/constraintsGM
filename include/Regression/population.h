//
//
//  @ Project : Approximation library Untitled
//  @ File Name : Population.h
//  @ Date : 2009-07-16
//  @ Author : Dominik Belter
//
//


#ifndef _POPULATION_H
#define _POPULATION_H

#include "individual.h"
#include <iostream>
#include <list>

class Population {
public:
	///constructor
    Population();
	///destructor
    ~Population();
	/// initialize
    void setParameters(unsigned int _populationSize, unsigned int gaussiansNo, unsigned int dim, unsigned int _outputsNo);
	/// set boundaries of function's parameters
    void setBoundaries(const std::vector<std::pair<double,double>>& _gaussianBoundaries, const std::vector<std::pair<double,double>>& _inputDomain);
	/// creates new individual and adds it into the population
	void createNewIndividual();
	/// remove individual fom population
    void removeIndividual(unsigned int individualNo);
	/// creates population
	void createPopulation();
	/// randomly selects individual
	unsigned int selectIndividual();
	/// get individual from population
    Individual getIndividual(unsigned int individualNo) const;
	/// compute average fitness
    double computeAverageFitness();
	/// compute individual fitness
    double computeIndividualFitness(const Eigen::MatrixXd& input, const Eigen::MatrixXd& output, const Eigen::MatrixXd& coef, unsigned int individualNo);
	/// get individual from population
	void copyIndividual(unsigned int src, unsigned int dest);
    /// Get best solution
    inline unsigned int getBestIndividualNo() const {return bestIndividual;}
    /// Set best solution
    inline void setBestIndividualNo(unsigned int individual) {bestIndividual = individual;}
    /// Set best solution
    inline void setIndividual(unsigned int individualNo, Individual& individual) {population[individualNo] = individual;}
    /// set individual
    void setIndividual(const Eigen::VectorXd& x, size_t individualNo);
    /// Get Gaussian Boundaries
    inline std::vector<std::pair<double,double>> getGaussianBoundaries(unsigned int individualNo) const {return population[individualNo].getGaussianBoundaries();}
    /// Get population size
    inline unsigned int getPopulationSize() const {return populationSize;}
    /// Get Gaussians no
    inline unsigned int getGaussiansNo(unsigned int gaussianNo) const {return population[gaussianNo].getGaussiansNo();}
    /// Get average fitness
    inline double getAverageFitness() const {return averageFitness;}
    /// Move individual
    inline void moveIndividual(unsigned int individualNo, Individual& _bestIndividual) {population[individualNo].moveIndividual(_bestIndividual);}

private:
    /// population of individuals
    std::vector<Individual> population;
    /// population size
    unsigned int populationSize;
    /// the best individual
    unsigned int bestIndividual;
    /// fitness of the best individual
    double bestFitness;
    /// average fitness
    double averageFitness;
    ///boundaries n-coefficient for functions - two-column vector [min, max]
    std::vector<std::pair<double,double>> gaussianBoundaries;
    ///boundaries a-coefficient for functions - two-column vector [min, max]
    std::vector<std::pair<double,double>> inputDomain;
};

#endif  //_POPULATION_H
