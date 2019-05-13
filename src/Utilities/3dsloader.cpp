/*
 * ---------------- www.spacesimulator.net --------------
 *   ---- Space simulators and 3d engine tutorials ----
 *
 * Author: Damiano Vitulli <info@spacesimulator.net>
 *
 * ALL RIGHTS RESERVED
 *
 *
 * Tutorial 4: 3d engine - 3ds models loader
 * 
 * Include File: 3dsloader.cpp
 *  
 */
#define _CRT_SECURE_NO_DEPRECATE

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Utilities/3dsloader.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <stdexcept>
#include <iostream>

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

char Load3DS (ObjType* pObject, const char *pFilename)
{
	int i; //Index variable
	
	FILE *l_file; //File pointer
	
	unsigned short l_chunk_id; //Chunk identifier
	unsigned int l_chunk_lenght; //Chunk lenght

	unsigned char l_char; //Char variable
	unsigned short l_qty; //Number of elements in each chunk

	unsigned short l_face_flags; //Flag that stores some face information

	struct stat filestatus;
    stat( pFilename, &filestatus );
	
    if ((l_file=fopen (pFilename, "rb"))== NULL) return 0; //Open the file

	
	while (ftell (l_file) < filestatus.st_size) //Loop to scan the whole file 
	{
		//getche(); //Insert this command for debug (to wait for keypress for each chuck reading)

        if (fread (&l_chunk_id, 2, 1, l_file)==0)
            std::cout << "read problem\n"; //Read the chunk header
		//printf("ChunkID: %x\n",l_chunk_id); 
        if (fread (&l_chunk_lenght, 4, 1, l_file)==0)
                std::cout << "read problem\n";//Read the lenght of the chunk
		//printf("ChunkLenght: %x\n",l_chunk_lenght);

        switch (l_chunk_id) {
			//----------------- MAIN3DS -----------------
			// Description: Main chunk, contains all the other chunks
			// Chunk ID: 4d4d 
			// Chunk Lenght: 0 + sub chunks
			//-------------------------------------------
			case 0x4d4d: 
			break;    

			//----------------- EDIT3DS -----------------
			// Description: 3D Editor chunk, objects layout info 
			// Chunk ID: 3d3d (hex)
			// Chunk Lenght: 0 + sub chunks
			//-------------------------------------------
			case 0x3d3d:
			break;
			
			//--------------- EDIT_OBJECT ---------------
			// Description: Object block, info for each object
			// Chunk ID: 4000 (hex)
			// Chunk Lenght: len(object name) + sub chunks
			//-------------------------------------------
			case 0x4000: 
				i=0;
                do{
                    if (fread (&l_char, 1, 1, l_file)==0)
                        std::cout << "read problem\n";
                    //pObject->name[i]=l_char;
                    const char ch(l_char);
                    pObject->name.append(&ch);
					i++;
                }while(l_char != '\0' && i<20);
			break;

			//--------------- OBJ_TRIMESH ---------------
			// Description: Triangular mesh, contains chunks for 3d mesh info
			// Chunk ID: 4100 (hex)
			// Chunk Lenght: 0 + sub chunks
			//-------------------------------------------
			case 0x4100:
			break;
			
			//--------------- TRI_VERTEXL ---------------
			// Description: Vertices list
			// Chunk ID: 4110 (hex)
			// Chunk Lenght: 1 x unsigned short (number of vertices) 
            //             + 3 x float (vertex coordinates) x (number of vertices)
			//             + sub chunks
			//-------------------------------------------
			case 0x4110: 
                if (fread (&l_qty, sizeof (unsigned short), 1, l_file)==0)
                    std::cout << "read problem\n";
                //pObject->verticesQty = l_qty;
                pObject->vertices.resize(l_qty);
                //printf("Number of vertices: %d\n",l_qty);
                for (i=0; i<l_qty; i++)
                {
                    if (fread (&pObject->vertices[i].vertex.x, sizeof(float), 1, l_file)==0)
                        std::cout << "read problem\n";
 					//printf("Vertices list x: %f\n",p_object->vertex[i].x);
                    if (fread (&pObject->vertices[i].vertex.y, sizeof(float), 1, l_file)==0)
                        std::cout << "read problem\n";
 					//printf("Vertices list y: %f\n",p_object->vertex[i].y);
                    if (fread (&pObject->vertices[i].vertex.z, sizeof(float), 1, l_file)==0)
                        std::cout << "read problem\n";
                    if (isnan(pObject->vertices[i].vertex.x)||isnan(pObject->vertices[i].vertex.y)||isnan(pObject->vertices[i].vertex.z))
                        throw std::runtime_error("3ds file incorrect (vertex)\n");
                    if (!isfinite(pObject->vertices[i].vertex.x)||!isfinite(pObject->vertices[i].vertex.y)||!isfinite(pObject->vertices[i].vertex.z))
                        throw std::runtime_error("3ds file incorrect (vertex coord is not finite)\n");
 					//printf("Vertices list z: %f\n",p_object->vertex[i].z);
				}
				break;

			//--------------- TRI_FACEL1 ----------------
			// Description: Polygons (faces) list
			// Chunk ID: 4120 (hex)
			// Chunk Lenght: 1 x unsigned short (number of polygons) 
			//             + 3 x unsigned short (polygon points) x (number of polygons)
			//             + sub chunks
			//-------------------------------------------
			case 0x4120:
                if (fread (&l_qty, sizeof (unsigned short), 1, l_file)==0)
                    std::cout << "read problem\n";
                //pObject->polygonsQty = l_qty;
                pObject->polygons.resize(l_qty);
               // printf("Number of polygons: %d\n",l_qty); 
                for (i=0; i<l_qty; i++) {
                    unsigned short readVal;
                    if (fread (&readVal, sizeof (unsigned short), 1, l_file)==0)
                        std::cout << "read problem\n";
                    pObject->polygons[i].a = readVal;
					//printf("Polygon point a: %d\n",p_object->polygon[i].a);
                    if (fread (&readVal, sizeof (unsigned short), 1, l_file)==0)
                        std::cout << "read problem\n";
                    pObject->polygons[i].b = readVal;
					//printf("Polygon point b: %d\n",p_object->polygon[i].b);
                    if (fread (&readVal, sizeof (unsigned short), 1, l_file)==0)
                        std::cout << "read problem\n";
                    pObject->polygons[i].c = readVal;
					//printf("Polygon point c: %d\n",p_object->polygon[i].c);
                    if (fread (&l_face_flags, sizeof (unsigned short), 1, l_file)==0)
                        std::cout << "read problem\n";
					//printf("Face flags: %x\n",l_face_flags);
				}
                break;

			//------------- TRI_MAPPINGCOORS ------------
			// Description: Vertices list
			// Chunk ID: 4140 (hex)
			// Chunk Lenght: 1 x unsigned short (number of mapping points) 
            //             + 2 x float (mapping coordinates) x (number of mapping points)
			//             + sub chunks
			//-------------------------------------------
			case 0x4140:
                if (fread (&l_qty, sizeof (unsigned short), 1, l_file)==0)
                    std::cout << "read problem\n";
				for (i=0; i<l_qty; i++)
				{
                    if (fread (&pObject->vertices[i].mapcoord.u, sizeof (float), 1, l_file)==0)
                        std::cout << "read problem\n";
					//printf("Mapping list u: %f\n",p_object->mapcoord[i].u);
                    if (fread (&pObject->vertices[i].mapcoord.v, sizeof (float), 1, l_file)==0)
                        std::cout << "read problem\n";
					//printf("Mapping list v: %f\n",p_object->mapcoord[i].v);
				}
                break;

			//----------- Skip unknow chunks ------------
			//We need to skip all the chunks that currently we don't use
			//We use the chunk lenght information to set the file pointer
			//to the same level next chunk
			//-------------------------------------------
			default:
				 fseek(l_file, l_chunk_lenght-6, SEEK_CUR);
        } 
	}
	fclose (l_file); // Closes the file stream
	return (1); // Returns ok
}

