#ifndef H_OBJECTS3DS
#define H_OBJECTS3DS

#include <stdlib.h>
#include <GL/glut.h>
#include "3dsloader.h"
#include <cstdint>
#include <iostream>
#include <vector>
#include <math.h>

class Objects3DS {
public:
    Objects3DS();
    ~Objects3DS();
	
    /// openGL initialization
    void Object3DSinit(int objQty);
    /// load object
    size_t ObjLoad(const std::string& objectFilename);

    /// compute normal vector
    void calcNormal(double** v, double* out);
    /// reduce vector to unit
    void ReduceToUnit(double vector[3]);
//    /// check if object is loaded
//    bool isObject(const std::string& objectFilename);
//    /// getId of the model
//    size_t getId(const std::string& objectFilename);

    std::vector<ObjType> objects;
    std::vector<std::string> filenames;
};
#endif
