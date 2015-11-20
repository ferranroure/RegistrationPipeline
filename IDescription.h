#ifndef IDESCRIPTION_H
#define IDESCRIPTION_H

#include <iostream>
#include "data.h"
#include "include/mypcl.h"
#include "include/elementset.h"

class IDescription
{
protected:
    // Elements -------------------------------------------------
    Data *data;

public:
    // Methods --------------------------------------------------

    virtual void setData(Data *d) = 0;
    virtual void execute() = 0;
    virtual void calcDescriptors(ElementSet *X) = 0;
};

#endif // IDESCRIPTION_H
