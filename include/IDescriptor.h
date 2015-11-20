/*******************************************************************************
 *  Interface IDescriptor
 *
 *  Contract of Descriptor types.
 *
 *  Ferran Roure (froure@eia.udg.edu)
 *  ViCOROB Research Group
 *  University of Girona
 ******************************************************************************/

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <algorithm>
#include <iterator>
#include "point.h"
#include "cmath"

#ifndef IDESCRIPTOR_H
#define IDESCRIPTOR_H

using namespace std;

class IDescriptor
{
public:

    // Methods ---------------------------------------------------------------
    virtual void print() = 0;
    virtual bool isAvailable() const = 0;
    virtual float compare(IDescriptor *D) = 0;
    virtual string toString() = 0;
    virtual int getSize() const = 0;
    virtual float getValue(int pos) const = 0;
    virtual void addElement(float f) = 0;
    virtual IDescriptor * clone() const = 0;
};

#endif // IDESCRIPTOR_H
