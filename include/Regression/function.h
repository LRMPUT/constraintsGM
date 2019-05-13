//  @ Project : Gaussian Mixture Regression library
//  @ File Name : function.h
//  @ Date : 2009-07-16
//  @ Author : Dominik Belter
//
//

#include <iostream>

#ifndef _FUNCTION_H
#define _FUNCTION_H

class Function {
public:
	///compute value of function
    double computeValue(double x) const;
    ///compute derivative value of function
    double computeDerivative(double x) const;
	///save function to file
    void save2file(std::ofstream& ofstr, unsigned int dim, int type) const;
    ///set Gaussian parameters
    void setParameters(double centroid, double width);
    ///PSO -- modify positoin change
    void modifyPositionChange(double _centroidChange, double _widthChange);
    /// Get width
    inline double getWidth() const {return width;}
    /// Get centroid
    inline double getCentroid() const {return centroid;}
    /// Get width change
    inline double getWidthChange() const {return widthChange;}
    /// Get centroid change
    inline double getCentroidChange() const {return centroidChange;}
    /// Set width
    inline void setWidth(double _width) {width = _width;}
    /// Set centroid
    inline void setCentroid(double _centroid) {centroid = _centroid;}
    /// Set width change
    inline void setWidthChange(double _widthChange) {widthChange = _widthChange;}
    /// Set centroid change
    inline void setCentroidChange(double _centroidChange) {centroidChange = _centroidChange;}

private:
    /// centroid
    double centroid;
    /// Gaussian width
    double width;
    /// x offset - velocity for PSO
    double centroidChange;
    /// width - velocity for PSO
    double widthChange;
};

#endif  //_CFUNCTION_H
