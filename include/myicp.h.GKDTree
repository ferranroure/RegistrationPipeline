/*******************************************************************************
 *  CLASS MYICP
 *
 *  This class is a bridge between data structures from pipeline project and
 *  the external ICP classes.
 *  The external ICP class is:
 *
 *      The ICP method implemented by Rusinkiewicz et.al.
 *      http://www.cs.princeton.edu/~smr/papers/fasticp/
 *
 *  Ferran Roure (froure@eia.udg.edu)
 *  ViCOROB Research Group
 *  University of Girona
 ******************************************************************************/

#ifndef MYICP_H
#define MYICP_H

#include <iostream>
#include "../external/FineMatching/fineMatching.h"
#include "../external/FineMatching/TriMesh.h"
#include "../external/FineMatching/Vec.h"
#include "../external/motion3D.h"
#include "../include/elementset.h"
#include "../include/point.h"


class myICP
{
public:
    // Elements ---------------------------------------------------------------
    fineMatching *fm;

    // Methods ----------------------------------------------------------------
    myICP();                                    // Constructor.
    myICP(ElementSet *A, ElementSet *B);        // Constructor with the initial pointsets.
    myICP(string infile1, string infile2,
        IDataStructure *ds1, IDataStructure *ds2);      // Constructor with initial files.
    ~myICP();                                   // Desctructor.
    motion3D *align();                          // Calulate fine alignment.


};

#endif // MYICP_H
