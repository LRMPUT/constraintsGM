//
//
//
//  @ Project : Approximation Library
//  @ File Name : Population.cpp
//  @ Date : 2009-07-16
//  @ Author : Dominik Belter
//
//

#include "Regression/population.h"

///constructor
Population::Population(){
}

///destructor
Population::~Population(){
    //cvReleaseMat(&gaussianBoundaries);
    //cvReleaseMat(&inputDomain);
	/*
    for (int i=0;i<max_populationSize;i++)
		delete population[i];
	delete [] population;
	*/
}

/// initialize
void Population::setParameters(unsigned int _populationSize, unsigned int gaussiansNo, unsigned int dim, unsigned int _outputsNo){
    //list<Individual>::iterator it;
    populationSize=_populationSize;
    createPopulation();
    for (size_t i=0;i<populationSize;i++){
        population[i].setBoundaries(gaussianBoundaries,inputDomain);
        population[i].setParameters(gaussiansNo, dim, _outputsNo);
    }
}

/// set boundaries of function's parameters
void Population::setBoundaries(const std::vector<std::pair<double,double>>& _gaussianBoundaries, const std::vector<std::pair<double,double>>& _inputDomain){
    //gaussianBoundaries = MatrixXd::Zero(_gaussianBoundaries.rows(), 2);
    //inputDomain = MatrixXd::Zero(_inputDomain.rows(), 2);
    gaussianBoundaries = _gaussianBoundaries;
    inputDomain = _inputDomain;
}

/// creates new individual and adds it into the population
void Population::createNewIndividual() {
    //population[populationSize].setBoundaries(gaussianBoundaries,inputDomain);
    //population[populationSize].setParameters(population[0].poly_elements_no,population[0].dim);
    populationSize++;
}

/// remove individual fom population
void Population::removeIndividual(unsigned int individualNo) {
    Individual tmp(population[individualNo]);
    for (size_t i=0;i<populationSize-1;i++){
        if (i>=individualNo)
			population[i]=population[i+1];
	}
    population[populationSize-1]=tmp;
    populationSize--;
}

/// creates population
void Population::createPopulation() {
    population.resize(populationSize);
}

/// get individual from population
Individual Population::getIndividual(unsigned int individualNo) const {
    return population[individualNo];
}

/// compute average fitness
double Population::computeAverageFitness(){
    double average=0;
    for (size_t i=0;i<populationSize;i++){
        average+=population[i].getFitnessValue();
	}
    average=average/(double)populationSize;

    averageFitness=average;
	return average;
}

/// compute individual fitness
double Population::computeIndividualFitness(const Eigen::MatrixXd& input, const Eigen::MatrixXd& output, const Eigen::MatrixXd& coef, unsigned int individualNo){
    return population[individualNo].computeFitness(input, output, coef);
}

/// set individual
void Population::setIndividual(const Eigen::VectorXd& x, size_t individualNo){
    population[individualNo].setIndividual(x);
}
