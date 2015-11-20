/*******************************************************************************
 *  CLASS HISTOGRAM
 *
 *  This object represents a histogram point descriptor.
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
#include "IDescriptor.h"

#ifndef DESCHISTOGRAM_H
#define DESCHISTOGRAM_H

using namespace std;

class DescHistogram : public IDescriptor
{
public:

    // Elements --------------------------------------------------------------
    vector<float> desc;

    // Methods ---------------------------------------------------------------
    DescHistogram();
    DescHistogram(const DescHistogram &dh);
    DescHistogram(vector<float> vd);
    ~DescHistogram();
    void print();
    bool isAvailable() const;
    float compare(IDescriptor *D);
    string toString();
    int getSize() const;
    float getValue(int pos) const;
    void addElement(float f);
    virtual DescHistogram * clone() const;
};

#endif // DESCRIPTOR_H
