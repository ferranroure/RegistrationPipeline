#include <iostream>
#include <fstream>
#include <string>
#include "pipeline.h"

using namespace std;

int main(int argc, char** argv)
{
    // Reading parameters
    Pipeline pipeline(argv[1]);

    int repetitions=10000;

    for(int i=0;i<repetitions;i++)
    {
        //    pipeline.execute();
        pipeline.executeResidueComputation();
    }
    return 0;
}
