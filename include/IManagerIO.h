/*******************************************************************************
 *  INTERFACE IManagerIO
 *
 *  This interface defines the contract for any class that wants to read or write
 *  any point cloud file.
 *
 *  Ferran Roure (froure@eia.udg.edu)
 *  ViCOROB Research Group
 *  University of Girona
 ******************************************************************************/

#ifndef IMANAGERIO_H
#define IMANAGERIO_H


#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <algorithm>
#include <iterator>
#include <vector>
#include "point.h"

class IManagerIO
{
public:

    // Methods -------------------------------------------------
    virtual vector<Point>* readFile(string path) = 0;                   // Read input file.

    virtual void writeFile(string in_path,                              // Write points to an output file according to
                           string out_path,                             //      the configuration of the input file (triangles, connections...).
                           vector<Point> *lin) = 0;

    virtual void writeFile(string in_path,                              // Write points and normals to an outpuf file according to
                           string out_path,                             //      the configuration of the input file.
                           vector<Point> *lin,
                           vector<vector3D> *norm) = 0;

    virtual void writeFile(string out_path,                             // Write only points without other data.
                           vector<Point> *lin) = 0;

    virtual void writeBase(string out_path,                             // Write a Base in a single file.
                           vector<Point> *lin) = 0;
};


#endif // IMANAGERIO_H
