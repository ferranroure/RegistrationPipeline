#include <iostream>
#include <fstream>
#include <string>
#include "pipeline.h"

using namespace std;

int main(int argc, char** argv)
{
    // Reading parameters

    int repetitions=100000000000;
cout<<"enter the sandman"<<endl;
    for(int i=0;i<repetitions;i++)
    {

        Pipeline pipeline(argv[1]);

        //    pipeline.execute();
       // pipeline.executeResidueComputation();
    }
    return 0;
}
