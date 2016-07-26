#include <iostream>
#include <fstream>
#include <string>
#include "pipeline.h"

using namespace std;

int main(int argc, char** argv)
{

    // Reading parameters
    Pipeline pipeline(argv[1]);

//    pipeline.execute();
    pipeline.executeResidueComputation((bool)atoi(argv[2]), argv[3]);
//    pipeline.syntheticComputeResidue();

    return 0;
}
