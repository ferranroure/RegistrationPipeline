/*******************************************************************************
 *  CLASS OUTPUT HANDLER
 *
 *  This class manage the results of the pipeline to be published in a output
 *  file.
 *
 *  Ferran Roure (froure@eia.udg.edu)
 *  ViCOROB Research Group
 *  University of Girona
 ******************************************************************************/

#ifndef OUTPUTHANDLER_H
#define OUTPUTHANDLER_H

#include "data.h"

class OutputHandler
{
public:

    // Elements ---------------------------------------------------------------
    Data *data;                                     // Pointer to Data object.

    // Methods ----------------------------------------------------------------
    OutputHandler();                                // Constructor.
    ~OutputHandler();                               // Destructor.
    void setData(Data *d);                          // Set Data element in order to access and modify it.
    void execute();                                 // Execute the instructions of this part of the pipline.

};

#endif // OUTPUTHANDLER_H
