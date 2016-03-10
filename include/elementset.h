/*******************************************************************************
 *  CLASS ELEMENT SET
 *
 *  This class is a container of the elements of the point cloud.
 *
 *  Ferran Roure (froure@eia.udg.edu)
 *  ViCOROB Research Group
 *  University of Girona
 ******************************************************************************/

#ifndef ELEMENTSET_H
#define ELEMENTSET_H

#include <iostream>
#include <string>
#include <vector>
#include <stdlib.h>
#include <time.h>
#include <float.h>
#include <math.h>
#include "point.h"
#include "plyio.h"
#include "DataStructures/kdtree/mykdtree.h"
#include "mypcl.h"
#include "descdist.h"
#include "base.h"
#include "../external/motion3D.h"
#include "DataStructures/octree/myOctree.h"
#include "DataStructures/trihash/myTriHash.h"
#include "DataStructures/octree/Octree.h"
#include "DataStructures/gridTree/myGridTree.h"

using namespace std;


class ElementSet
{
private:


private:
    // Elements --------------------------------------------------------------------------------------------------------
    vector<Point> *points;                                          // Vector of points.

    float xmin, xmax, ymin, ymax, zmin, zmax;                       // Bounding Box.
    float diagonal;                                                 // Diagonal of the bounding box.
    Point center;                                                   // Approx center of mass.
    IDataStructure *dataStruct;                                     // KdTree.
    Octree *octree; // DEPRECATED
    float MMD;                                                      // Mean Minimum Distance of point of this cloud.
    float GTresidue;                                                // Ground Truth residue.
    vector<vector3D> *normals;
    string dataStructureType;

public:
    // Constructors  ---------------------------------------------------------------------------------------------------
    vector<Point*> *workpoints;                                     // Vector of keypoints.
    vector<Point*> *allpoints;
    ElementSet();                                                   // Constructor NULL.
    ElementSet(ElementSet &ES);                                     // Copy constructor.
    ElementSet(vector<Point> *lin, string _DSType);                                 // Constructor from vector<Point>.
    ElementSet(string file, string _DSType, float normFactor=1);    // Constructor from specific file.
    ~ElementSet();                                                  // Destructor.


    // Getting information from this ElementSet ------------------------------------------------------------------------
    Point * getPoint(int pos);                                      // Get a point located at position POS
    Point * findWidePoint(Point *p, bool useDetectors);
    Point * getRandomPoint(bool useDetectors=false);
    Base  * getRandomBase(bool useDetectors=false);
    Point * findPoint(Point &p);
    int     nPoints();                                              // Return number of points.
    void    print(bool withNormals=0, bool withColor=0);            // Print all element set
    bool    availableDescriptors();

    vector<Point *> *findCorrespondences(Point *p, int nCorr, bool usePCL, string method);
    vector<Point *> findCoplanarPoints(vector3D norm, float d, Point *p, float dist);


    // Compute data ----------------------------------------------------------------------------------------------------
    double  calcDiagonal();                                         // Calculate diagonal distance of our point cloud.
    void    calcMMD();                                              // Calculates the Mean Minimum Distance of points.
    Point   calcCentroid();
    double  calcNN(vector<Point> *Q, double percOfPoints,
                float errorFactor, int &pairedPoints);              // Calculates the Nearest Neighbor
    vector<returnData> calcNneigh(Point *q, int nNeigh);


    void    calcDescriptor(string method, float radiusNormal,
                float radiusSearch);
    void    calcDetector(string method);
    Eigen::Matrix3f *calcCovarianceMatrix();
    void    calcNormals(float radiusNormal);

    vector<pair<Point *, Point *> > findPairs(float distance, float thrs);
    vector<pair<Point *, Point *> > findPairsOctree(float distance, float thrs);
    int     calcOutliers(float thrs);


    // Modify information ----------------------------------------------------------------------------------------------
    void    update();                                               // Update external data structures after a modification.
    void    addNoise(double threshold);                             // Add noise to this elementSet
    void    transform(motion3D *m);                                 // Tranform the point cloud according to the tranfrmation m
    void    createFileFromData(string infile, string outfile, bool withNormals=0);      // Create PLY file from data element
    void    createFileFromData(string outfile, bool withNormals=0, bool withColor=0);
    void    createFileFromDescriptors(string outfile);
    void    createDataStructure();                                  // Create Kdtree
    vector<DescDist> *sortPoints(Point *p);
    void    setPointDescriptor(int pos, IDescriptor *desc);
    void    createWorkingStructures();

    void    initRandomMachine();                                    // Initiates the random generator
    void    scalePoints(float normFactor);
    void    addPoint(Point *p);

    void    updateDataStructure(IDataStructure *ed);


    // Getters and Setters ---------------------------------------------------------------------------------------------

    const string &getDataStructureType() const {
        return dataStructureType;
    }

    void setDataStructureType(const string &dataStructureType) {
        ElementSet::dataStructureType = dataStructureType;
    }

    vector<Point> *getPoints() const{
        return points;
    }

    vector<Point *> *getWorkpoints() const {
        return workpoints;
    }

    vector<Point *> *getAllpoints() const {
        return allpoints;
    }

    float getXmin() const {
        return xmin;
    }

    float getXmax() const {
        return xmax;
    }

    float getYmin() const {
        return ymin;
    }

    float getYmax() const {
        return ymax;
    }

    float getZmin() const {
        return zmin;
    }

    float getZmax() const {
        return zmax;
    }

    float getDiagonal() const {
        return diagonal;
    }

    Point const &getCenter() const {
        return center;
    }

    IDataStructure *getDataStruct() const {
        return dataStruct;
    }

    float getMMD() const {
        return MMD;
    }

    float getGTresidue() const {
        return GTresidue;
    }

    void setPoints(vector<Point> *points) {
        ElementSet::points = points;
    }

    void setWorkpoints(vector<Point *> *workpoints) {
        ElementSet::workpoints = workpoints;
    }

    void setXmin(float xmin) {
        ElementSet::xmin = xmin;
    }

    void setXmax(float xmax) {
        ElementSet::xmax = xmax;
    }

    void setYmin(float ymin) {
        ElementSet::ymin = ymin;
    }

    void setYmax(float ymax) {
        ElementSet::ymax = ymax;
    }

    void setZmin(float zmin) {
        ElementSet::zmin = zmin;
    }

    void setZmax(float zmax) {
        ElementSet::zmax = zmax;
    }

    void setDiagonal(float diagonal) {
        ElementSet::diagonal = diagonal;
    }

    void setCenter(Point const &center) {
        ElementSet::center = center;
    }

    void setDataStruct(IDataStructure *dataStruct) {
        ElementSet::dataStruct = dataStruct;
    }

    void setMMD(float MMD) {
        ElementSet::MMD = MMD;
    }

    void setGTresidue(float GTresidue) {
        ElementSet::GTresidue = GTresidue;
    }

    vector<vector3D> *getNormals() const {
        return normals;
    }

    void setNormals(vector<vector3D> *normals) {
        ElementSet::normals = normals;
    }

};

#endif // ELEMENTSET_H
