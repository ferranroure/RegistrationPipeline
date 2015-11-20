/*******************************************************************************
 *  CLASS INPUT HANDLER
 *
 *  This class manage and prepare the input data to be operated in the rest of the
 *  pipeline.
 *
 *  Ferran Roure (froure@eia.udg.edu)
 *  ViCOROB Research Group
 *  University of Girona
 ******************************************************************************/

#ifndef INPUTHANDLER_H
#define INPUTHANDLER_H

#include <iostream>
#include "data.h"

using namespace std;

//# define PI           3.14159265358979323846


class InputHandler
{
public:

    // Elements ----------------------------------------------------------------
    Data *data;                                     // Pointer to Data object.

    // Methods -----------------------------------------------------------------
    InputHandler();                                 // Constructor.
    ~InputHandler();                                // Destructor.
    void setData(Data *d);                          // Set Data element in order to access and modify it.
    void execute();                                 // Execute the instructions of this part of the pipline.
};

#endif // INPUTHANDLER_H
