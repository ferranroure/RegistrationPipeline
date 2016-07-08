/*******************************************************************************
 *  CLASS DATA
 *
 *  This class contains all the data needed in the pipeline. This class is
 *  accessible from all the steps of the pipeline. Each element is can be modified
 *  by any class of the pipeline.
 *
 *  Ferran Roure (froure@eia.udg.edu)
 *  ViCOROB Research Group
 *  University of Girona
 ******************************************************************************/

#ifndef DATA_H
#define DATA_H

#include <iostream>
#include <vector>
#include <stdlib.h>
#include <time.h>
#include <float.h>
#include <math.h>
#include "include/point.h"
#include "include/plyio.h"
#include "include/elementset.h"
#include "external/tinyxml2.h"
#include <pcl/io/ply_io.h>

using namespace std;
using namespace tinyxml2;

struct parameters{

    bool realData;                      // 0 = Syntetic data; 1 = Real data.

    string infile;                      // Input object file.
    string infile2;                     // Second input object file.
    string infileTemp;                  // Input temporal object file.
    string outfile;                     // Output object file.
    string outres;                      // Output results file.

    bool useDetection;                  // Use Detection step.
    bool useDescription;                // Use Description step.
    bool useSS;                         // Use Searching Strategies step.
    bool useRefinement;                 // Use Refinement step.

    string detectMethod;                // Detector method.
    string descMethod;                  // Descriptor method used.
    string SSMethod;                    // Searching Strategy method.
    string refineMethod;                // Refinement method.

    // parameters for the 4points data structure
    bool fourPUseCmdLineP;                   // Turn this on or of at will
    float thr;                          // threshold for sets to be considered matched
    int nPoints;                       // Number of points sampled at the start of the 4points algorithm
    float normDiff;                    // difference between normals?
    float delta;                        // distance allowed for two points to be considered neighbors also works with base points
    float overlap;                      // expected matching overlap between sets, condition what bases are accepted

    int nLevels;                        // Number of levels for Grid3D method. MODIFIED, CURRENTLY THE NUMBERS OF LEVELS OF THE HNSS PYRAMIDS
    int nCells;                         // Number of cells x dimension for Grid3D method.
    float percOfPoints;                 // % of points of the object used to search NN residue.
    int nSamples;                       // Number of sample points extracted in detection step. In the case of HNSS it is the size of the dataset at the top of the pyramid
   // int FourPCSSample;                     // the number of points sampled by the 4PCS
    bool normalizeModels;               // Boolean about if models must be normalized or not. (divide points by biggest diagonal).
    string dataStructure;               // Data Structure used for Nearest Neighbour searching and Residue computation.

    // All next parameters are the multipling factor for Mean Minimum Distance (MMD).
    float nnErrorFactor;                // Multipling factor for the error distance on NN search.
    float percOfNoise;                  // Multipling factor to apply noise in the object.
    float radiusNormalFactor;           // Multipling factor for normal search.
    float radiusSearchFactor;           // Multipling factor for descriptors search.
    int   nNeighbours;                  // Number of neighbours used for the search for correspondences.
    int   thrsFactor;                   // Multipling factor for checkDistanceThreshold.

    float MMD;                          // Mean Minimum Distance of point from main point cloud.
    int   nOutliersA;                   // # of outliers of A
    int   nOutliersB;                   // # of outliers of B
    float GTdescThrs;                   // Threshold to compare descriptors. Is calculated using a ground truth experimentation.
    float GTminDescDist;                // Minimum difference between two descriptors.
    float GTmaxDescDist;                // Maximum difference between two descriptors.
    float GTresidue;                    // GroundTruth residue.
    float GTpercPairedPoints;           // % of paired points from groundTruth.
};


class Data
{
public:

    // Elements -------------------------------------------------------------------
    ElementSet *A;                              // First point cloud.
    ElementSet *B;                              // Second point cloud.
    ElementSet *result;                         // vector of final result.
    motion3D *cM;                               // Coarse transformation matrix.
    motion3D *fM;                               // Fine transformation matrix.
    parameters params;                          // Parameters for the execution.


    // Methods --------------------------------------------------------------------
    Data();                                     // Constructor.
    Data(char *paramsfile);                     // Constructor by parameters file.
    ~Data();                                    // Destructor.

    void printParams();                         // Print all the parameters.
    void setParameters(char * paramsfile);      // Set struct params with the information of paramsfile.
    void setParametersXML(char * paramsfile);   // Set struct params with the information of XML paramsfile.
    bool toBool(string value);                  // Transform an string to bool.
    void crearteFileFromBase(string path, Point x, Point y, Point z);

};
#endif // DATA_H
