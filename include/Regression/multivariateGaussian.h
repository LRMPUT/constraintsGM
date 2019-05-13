//
//  @ Project : Gaussian Mixture Regression library
//  @ File Name : multivariateGaussian.h
//  @ Date : 2009-07-16
//  @ Author : Dominik Belter



#ifndef _MULTIVARIATEGAUSSIAN_H
#define _MULTIVARIATEGAUSSIAN_H

#include "function.h"
#include "Defs/eigen3.h"

#include <random>
#include <iostream>

class MultivariateGaussian {
public:
	/// default constructor
    MultivariateGaussian();
	/// default destructor
    ~MultivariateGaussian();
	/// initialize
    void setParameters(unsigned int _dim);
	/// set boundaries of function's parameters
    void setBoundaries(const std::vector<std::pair<double,double>>& _gaussianBoundaries, const std::vector<std::pair<double,double>>& _inputDomain);
    /// create multivariate Gaussian
    void createMultivariateGaussian();
	/// compue value of gene point must be column vector
    double computeValue(const Eigen::MatrixXd& point) const;
    /// compue gradient. Point must be column vector
    void computeGradient(const Eigen::MatrixXd& point, Eigen::MatrixXd& grad) const;
	///save function to file
    void save2file(std::ofstream& ofstr, int type);
    /// initialize 1D Gaussian
    void initializeGauss(const std::vector<std::pair<double,double>>& _gaussianBoundaries, const std::vector<std::pair<double,double>>& _inputDomain);
    /// set gaussians
    void setGaussians(const std::vector<double> params);
    /// Get Gaussian width
    inline double getWidth(size_t gaussianNo) const {return gaussians[gaussianNo].getWidth();}
    /// Get centroid
    inline double getCentroid(size_t gaussianNo) const {return gaussians[gaussianNo].getCentroid();}
    /// Get width change
    inline double getWidthChange(size_t gaussianNo) const {return gaussians[gaussianNo].getWidthChange();}
    /// Get centroid change
    inline double getCentroidChange(size_t gaussianNo) const {return gaussians[gaussianNo].getCentroidChange();}
    /// Set width
    inline void setWidth(size_t gaussianNo, double _width) {gaussians[gaussianNo].setWidth(_width);}
    /// Set centroid
    inline void setCentroid(size_t gaussianNo, double _centroid) {gaussians[gaussianNo].setCentroid(_centroid);}
    /// Set width change
    inline void setWidthChange(size_t gaussianNo, double _widthChange) {gaussians[gaussianNo].setWidthChange(_widthChange);}
    /// Set centroid change
    inline void setCentroidChange(size_t gaussianNo, double _centroidChange) {gaussians[gaussianNo].setCentroidChange(_centroidChange);}

private:

    /// random number generator
    std::default_random_engine generator;
    /// functions in gene
    std::vector<Function> gaussians;
    /// dimension of the approximation space
    unsigned int dim;
    ///boundaries n-coefficient for functions - two-column vector [min, max]
    //std::vector<std::pair<double,double>> gaussianBoundaries;
    ///boundaries a-coefficient for functions - two-column vector [min, max]
    //std::vector<std::pair<double,double>> inputDomain;
    /// n_bound change
    //std::vector<double> gaussianBoundariesChange;
};

#endif  //_MULTIVARIATEGAUSSIAN_H
