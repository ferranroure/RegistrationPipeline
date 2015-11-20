/*******************************************************************************
 *  INTERFACE IREFINEMENT
 *
 *  This interface defines the contract for refinement classes.
 *
 *  Ferran Roure (froure@eia.udg.edu)
 *  ViCOROB Research Group
 *  University of Girona
 ******************************************************************************/

#ifndef IREFINEMENT_H
#define IREFINEMENT_H

#include <iostream>
#include "include/myicp.h"
#include "data.h"

class IRefinement
{

protected:
    Data *data;

public:


    // Methods ------------------------------------------------
    virtual void setData(Data *d) = 0;                          // Set Data element in order to access and modify it.
    virtual void execute() = 0;                                 // Execute the instructions of this part of the pipline.
};

#endif // IREFINEMENT_H
