/*******************************************************************************
 *  CLASS PLYIO
 *
 *  This class is used to read and write data from/to ply files.
 *
 *  Ferran Roure (froure@eia.udg.edu)
 *  ViCOROB Research Group
 *  University of Girona
 ******************************************************************************/

#ifndef PLYIO_H
#define PLYIO_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <algorithm>
#include <iterator>
#include "point.h"
#include "IManagerIO.h"


using namespace std;

class PlyIO : public IManagerIO
{
public:

    // Methods --------------------------------------------------------------------------
    PlyIO();                                                // Constructor
    ~PlyIO();                                               // Destructor

    vector<Point>* readFile(string path);

    void writeFile(string in_path,
                   string out_path,
                   vector<Point> *lin);

    void writeFile(string in_path,
                   string out_path,
                   vector<Point> *lin, vector<vector3D> *norm);

    void writeFile(string out_path,
                   vector<Point> *lin);

    void writeFile(string out_path,
                    vector<Point*> *lin,
                    bool withNormals=0,
                    bool withColor=0);

    void writeBase(string out_path,
                   vector<Point> *lin);


};

#endif // PLYIO_H
