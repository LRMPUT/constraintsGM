/** @file defs.h
*
* Simulator definitions
*
*/

#ifndef SIMULATOR_DEFS_H_INCLUDED
#define SIMULATOR_DEFS_H_INCLUDED

#include "defs.h"
#include <cstdint>
#include <vector>
#include <memory>
#include <cmath>

/// putslam name space
namespace simulator {

    class RogidBody {
    public:
        /// mass
        double mass;

        /// size
        walkers::Vec3 size;

        /// center
        walkers::Vec3 center;

        /// orientation
        walkers::Quaternion orientation;
    };
    //exception class goes here

    enum RenderObjectType{
        PLANE, BOX, SPHERE, CAPSULE, HEIGHTFIELD, LINE, MODEL3D
    };

    class RenderObject {
        public:
            /// set of RenderObjects
            typedef std::vector<RenderObject> Seq;
            /// construction
            RenderObject(void) : color{0.4,0.4,0.4,1.0}{
            }

            /// position and rotation
            walkers::Mat34 mat;

            /// type
            RenderObjectType type;

            /// size
            double x,y,z;
            /// scale factors
            double scaleX,scaleY,scaleZ;

            /// identifier
            size_t id;

            /// color
            std::vector<double> color;

            /// mass
            double mass;
    };

    class RenderObjectHeightmap: public RenderObject {
        public:
            std::vector<std::vector<double>> heightmap;
            std::vector<std::vector<std::vector<double>>> heightmapColors;
    };

}

#endif // SIMULATOR__H_INCLUDED
