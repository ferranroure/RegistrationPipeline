/*******************************************************************************
 *  CLASS POINT
 *
 *  This is the representation of the 3d Point. Optionally there are a pointer
 *  to a descriptor element of the point
 *
 *  Ferran Roure (froure@eia.udg.edu)
 *  ViCOROB Research Group
 *  University of Girona
 ******************************************************************************/

#ifndef POINT_H
#define POINT_H

#include <iostream>
//#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include "IDescriptor.h"
#include "../external/vector3D.h"

using namespace std;

//#define tole 0.000001
#define tole 0.00001

struct pointColor {

    int red;
    int green;
    int blue;
    int alpha;
};

class Point
{
private:
    // Elements ------------------------------------------------------------------------
    double x;                                       // Point coordinates.
    double y;
    double z;
    int index;
    IDescriptor * descriptor;                          // Pointer to descriptor class.
    vector3D * normal;
    pointColor color;

public:
    // Methods -------------------------------------------------------------------------
    Point();                                        // Constructor
    Point(double X, double Y, double Z);            // Constructor.
    Point(const Point &p);                          // Copy Constructor
    ~Point();                                       // Destructor.
    void addNoise(double nx, double ny,             // Adding noise to a point with a certain threshold.
                  double nz, double threshold);
    bool isInside(float xmin,
                  float xmax,
                  float ymin,
                  float ymax,
                  float zmin,
                  float zmax);                      // Return true if this point is inside de bounding box.
    void print(bool withNormals=0, bool withColor=0);                                   // Print a point information.
    void printWithDesc();
    int getIndex();
    double getX() const;
    double getY() const;
    double getZ() const;
    int getDescSize();
    void setIndex(int ind);
    IDescriptor * getDescriptor();
    void addCoords(double X, double Y, double Z);


    void update(Point p);
    // Operators -----------------------------------------------------------------------
    void operator=(Point p); 						// Assignement
    Point operator+(vector3D v);
    Point operator-(vector3D v);
    vector3D operator-(Point p);

    double dist(Point *a); 							// Distance between two points
    double sqrtDist(Point *a);
    Point distVector(Point a);
    void setDescriptor(IDescriptor *desc);
    void copyDescriptor(IDescriptor *desc);
    bool availableDescriptor() const;

    vector3D * getNormal();
    void setNormal(vector3D * norm);
    void setNormal(double nx, double ny, double nz);
    bool availableNormal() const;

    int getRed() const;
    int getGreen() const;
    int getBlue() const;
    int getAlpha() const;
    void setColor(int r, int g, int b, int a=255);

    //Comparison operators
    bool operator ==(Point p)const;
    bool operator <(Point p)const;
    bool operator !=(Point p)const;
    bool operator >(Point p)const;
    bool operator <=(Point p)const;
    bool operator >=(Point p)const;

};

#endif // POINT_H
