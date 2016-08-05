//
// Created by ferran on 5/08/16.
//

#include "DSparam.h"

template <class T>
DSparam<T>::DSparam(T _value) {

    param = _value;
}

template <class T>
DSparam<T>::~DSparam() { }

template <class T>
T DSparam<T>::getValue() {

    return param;
}

template <class T>
void DSparam<T>::printName() {

    cout << " " << name << " ";
}