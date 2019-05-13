/*
 * ---------------- www.spacesimulator.net --------------
 *   ---- Space simulators and 3d engine tutorials ----
 *
 *  Author: Damiano Vitulli <info@spacesimulator.net>
 *
 * ALL RIGHTS RESERVED
 *
 *
 * Tutorial 4: 3d engine - 3ds models loader
 * 
 * File header: 3dsloader.h
 *  
 */

#ifndef _LOADER3DS_H_
#define _LOADER3DS_H_

/**********************************************************
 *
 * FUNCTION Load3DS (obj_type_ptr, char *)
 *
 * This function loads a mesh from a 3ds file.
 * Please note that we are loading only the vertices, polygons and mapping lists.
 * If you need to load meshes with advanced features as for example:
 * multi objects, materials, lights and so on, you must insert other chunk parsers.
 *
 *********************************************************/
#include <vector>
#include <string>

//#define MAX_VERTICES 60000	// Max number of vertices (for each object) old - 10000
//#define MAX_POLYGONS 60000	// Max number of polygons (for each object) old - 10000
//#define MAX_OBJECTS 100		// Max number of objects

// Our vertex type
struct VertexType {
    float x,y,z;
};

// The polygon (triangle), 3 numbers that aim 3 vertices
struct PolygonType {
    size_t a,b,c;
};

// The mapcoord type, 2 texture coordinates for each vertex
struct MapcoordType {
    float u,v;
};

struct Vertex {
    VertexType		vertex;
    VertexType		normal;
    MapcoordType	mapcoord;
};

struct ObjType{

    std::string name;		// Name of the object

    std::vector<Vertex> vertices;
    std::vector<PolygonType> polygons;
    //VertexType		vertex[MAX_VERTICES];	// Array of vertices
    //VertexType		normal[MAX_VERTICES];	// Array of the vertices' normals
    //PolygonType	polygon[MAX_POLYGONS];	// Array of polygons (numbers that point to the vertices' list)
    //MapcoordType	mapcoord[MAX_VERTICES]; // Array of U,V coordinates for texture mapping
};

char Load3DS (ObjType* pObject, const char *pFilename);

#endif // _LOADER3DS_H_
