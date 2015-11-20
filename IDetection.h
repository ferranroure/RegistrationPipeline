#ifndef DETECTION_H
#define DETECTION_H

#include <iostream>
#include "data.h"
#include "include/mypcl.h"
#include "include/elementset.h"

class IDetection
{
protected:
	// Elements -------------------------------------------------
    Data *data;

public:
	// Methods --------------------------------------------------

    virtual void setData(Data *d) = 0;
    virtual void execute() = 0;
};

#endif // DETECTION_H
