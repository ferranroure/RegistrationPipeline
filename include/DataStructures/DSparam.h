//
// Created by ferran on 5/08/16.
//

#ifndef PIPELINE_DSPARAM_H
#define PIPELINE_DSPARAM_H

#include <iostream>
#include <string>

using namespace std;

template <class T>
class DSparam {

private:
    T param;
    string name;

public:

    DSparam(T _value);
    ~DSparam();

    T getValue();
    void printName();
};


#endif //PIPELINE_DSPARAM_H
