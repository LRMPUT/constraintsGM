//
//
//
//  @ Project : Approximation library
//  @ File Name : cgene.cpp
//  @ Date : 2009-07-16
//  @ Author : Dominik Belter
//
//

#include "Regression/function.h"
#include <cmath>
#include <fstream>
#include <iostream>

///compute value of function
double Function::computeValue(double x) const{
    return (-width*pow((x-centroid),2.0));//gaussian element
}

///compute derivative value of function
double Function::computeDerivative(double x) const{
    return (-width*2.0*(x-centroid));//gaussian element
}

///set Gaussian parameters
void Function::setParameters(double _centroid, double _width){
    centroid = _centroid;
    width=_width;
}

///PSO -- modify position change
void Function::modifyPositionChange(double _centroidChange, double _widthChange){
    centroidChange = _centroidChange;
    widthChange = _widthChange;
}

///save function to file
void Function::save2file(std::ofstream& ofstr, unsigned int dim, int type) const{
    if (type==1) {
        ofstr << "exp(-(" << width << ")*(input(" << dim+1 << ")-(" << centroid << ")))";
	}
    if (type==2) {
        ofstr << "(-(" << width << ")*pow((in[" << dim << "]-(" << centroid << ")),2.0))";
    }
}
