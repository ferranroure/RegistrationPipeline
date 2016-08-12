/*******************************************************************************
 *  CLASS PIPELINE
 *
 *  This class is the center of the project. Using Pipeline we can manage all
 *  the steps of the pipeline.
 *
 *  Ferran Roure (froure@eia.udg.edu)
 *  ViCOROB Research Group
 *  University of Girona
 ******************************************************************************/

#ifndef PIPELINE_H
#define PIPELINE_H

#include "data.h"
#include "inputhandler.h"
#include "IDetection.h"
#include "IDescription.h"
#include "ISearchingStrategy.h"
#include "IRefinement.h"
#include "outputhandler.h"
#include "include/timer.h"

#include "methods/det_iss.h"
#include "methods/det_RandomSampling.h"
#include "methods/det_NormalSpaceSampling.h"
#include "methods/det_HierarchicalNormalSpaceSampling.h"
#include "methods/det_ColorSpaceSampling.h"
#include "methods/det_KinectSuperSampling.h"
#include "methods/det_DistConnectedComponents.h"
#include "methods/det_FPFHSignature33.h"

#include "methods/des_shot.h"
#include "methods/des_sp.h"
#include "methods/ss_SmartForce.h"
#include "methods/ss_4PCS.h"
#include "methods/ss_3PS.h"
#include "methods/ss_Grid3D.h"
#include "methods/ref_icp.h"
#include "methods/ss_HNSS.h"
#include "methods/ss_pclFeatureBased.h"


#define COARSE 1
#define FINE 2

class Pipeline
{
public:

    // Elements --------------------------------------------------------------
    // All steps of the pipeline.
    InputHandler input;
    IDetection *detection;
    IDescription *description;
    ISearchingStrategy *searching;
    IRefinement *refinement;
    OutputHandler output;
    Data *data;                             // Pointer to Data object.


    // Methods ---------------------------------------------------------------
    Pipeline();                             // Constructor.
    Pipeline(char *paramsfile);             // Constructor with parameters file.
    ~Pipeline();                            // Destructor.
    void createMethods();                   // Creation of each method depending on the params file.
    void execute();                         // Execution of the pipeline.
    void executeTest();                     // Execution for tests without any cout.
    void executeResidueComputation();       // Execution of residue computation test.
    void computeResidue(bool test=false);   // Compute residue and % of paired points between views.
    void syntheticComputeResidue();         // Residue computation tests for synthetic models.
    vector<motion3D> readMatrices(const char *file);

    void calcGroundTruth(ElementSet *X,
                        IDescription *desc);// Compute groundtruth of a given ElementSet.
    void applyMovement(int type);           // Apply computed movement to ElementSet B.

};

#endif // PIPELINE_H

