#include "Utilities/objects3DS.h"

Objects3DS::Objects3DS() {
    this->filenames.clear();
    this->objects.clear();
}

Objects3DS::~Objects3DS() {
    //#ifdef DEBUG
    //std::cout << "Objects3Ds destructor\n";
    //#endif
}

void Objects3DS::Object3DSinit(int objQty) {
#ifdef BUILD_VISUALIZER
    double normal[3];
    double ** vert = new double*[3];
	for(int i = 0; i < 3; i++)
        vert[i] = new double[3];
    glBegin(GL_TRIANGLES);
    for (size_t j=0;j<objects[objQty].polygons.size();j++) {

        vert[0][0]=(objects[objQty].vertices[ objects[objQty].polygons[j].a ].vertex.x);
        vert[0][1]=(objects[objQty].vertices[ objects[objQty].polygons[j].a ].vertex.y);
        vert[0][2]=(objects[objQty].vertices[ objects[objQty].polygons[j].a ].vertex.z);

        vert[1][0]=(objects[objQty].vertices[ objects[objQty].polygons[j].b ].vertex.x);
        vert[1][1]=(objects[objQty].vertices[ objects[objQty].polygons[j].b ].vertex.y);
        vert[1][2]=(objects[objQty].vertices[ objects[objQty].polygons[j].b ].vertex.z);

        vert[2][0]=(objects[objQty].vertices[ objects[objQty].polygons[j].c ].vertex.x);
        vert[2][1]=(objects[objQty].vertices[ objects[objQty].polygons[j].c ].vertex.y);
        vert[2][2]=(objects[objQty].vertices[ objects[objQty].polygons[j].c ].vertex.z);
		
		calcNormal(vert, normal);
		glNormal3d (normal[0],normal[1],normal[2]);

        glVertex3d(vert[0][0], vert[0][1], vert[0][2]);
        glVertex3d(vert[1][0], vert[1][1], vert[1][2]);
        glVertex3d(vert[2][0], vert[2][1], vert[2][2]);
	}
    glEnd();
	for(int i = 0; i < 3; i++)
		delete [] vert[i];
	delete [] vert;	
#endif
}

///// check if object is loaded
//bool Objects3DS::isObject(const std::string& objectFilename) {
//    for(size_t i=0; i<objects.size(); ++i) {
//        if(!filenames[i].compare(objectFilename)) {
//            return true;
//        }
//    }
//    return false;
//}

///// getId of the model
//size_t Objects3DS::getId(const std::string& objectFilename){
//    for(size_t i=0; i<objects.size(); ++i) {
//        if(!filenames[i].compare(objectFilename)) {
//            return i+1;
//        }
//    }
//    return 0;
//}

size_t Objects3DS::ObjLoad(const std::string& objectFilename) {
    for(size_t i=0; i<objects.size(); ++i) {
        if(!filenames[i].compare(objectFilename)) {
            return i+1;
        }
    }
    std::string sources = "../../resources/"+objectFilename;
    objects.resize(objects.size()+1);
    if (Load3DS(&objects[objects.size()-1],sources.c_str())==0) return(0);
    filenames.push_back(objectFilename.c_str());
    return (objects.size());
}

void Objects3DS::calcNormal(double **v, double *out) {
    double v1[3],v2[3];
    const int x = 0;
    const int y = 1;
    const int z = 2;

    v1[x] = v[0][x] - v[1][x];
    v1[y] = v[0][y] - v[1][y];
    v1[z] = v[0][z] - v[1][z];

    v2[x] = v[1][x] - v[2][x];
    v2[y] = v[1][y] - v[2][y];
    v2[z] = v[1][z] - v[2][z];

    out[x] = v1[y]*v2[z] - v1[z]*v2[y];
    out[y] = v1[z]*v2[x] - v1[x]*v2[z];
    out[z] = v1[x]*v2[y] - v1[y]*v2[x];

    ReduceToUnit(out);
}

void Objects3DS::ReduceToUnit(double vector[3]) {
    double length = sqrt(  (vector[0]*vector[0]) + (vector[1]*vector[1]) + (vector[2]*vector[2]));
    if(length == 0.0)
        length = 1.0;

    vector[0] /= length;
    vector[1] /= length;
    vector[2] /= length;
}
