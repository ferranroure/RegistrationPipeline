//
// Created by yago on 16/06/21.
//

#include "ss_HNSS.h"

ss_HNSS::ss_HNSS() {
    data = NULL;
}

ss_HNSS::~ss_HNSS() {

}

void ss_HNSS::setData(Data *d) {
    data = d;
   // innerSearchStrategy->setData(d);
}

double ss_HNSS::execute()
{
    // loop through the inner search strategy

    double acceptablePercentage=data->params.overlap; // us the same as teh secondary structure
    // here we could define a step variable to make it easier to find matching in lower levels.
    // we could modify the parameter at the inner data structure modifying
//    data->params.overlap=
 //   innerSearchStrategy->setData(data);
 // careful with order!!!!

    bool finished=false;
    int i=detAccess->getPiramidALevels()-1;

    while(!finished&&(i>0))
    {
        // choose the new data set
        detAccess->extractHierarchicalNormalSpaceLevel(i);
        //innerSearchStrategy->setData(data);

        double matchedPercentage=innerSearchStrategy->execute();

        cout<<"                                                                                          :::::::::::::::::::::::::HNSS level, matched percentage: "<<matchedPercentage<<" acceptable at this point is "<<acceptablePercentage<<endl;

        if(matchedPercentage>acceptablePercentage) finished=true;
        else
        {
           // acceptablePercentage=acceptablePercentage-step;
            i = i - 1;
        }
    }

}

ss_HNSS::ss_HNSS(ISearchingStrategy *inner, det_HierarchicalNormalSpaceSampling *det)
{
    innerSearchStrategy=inner;
    detAccess=det;
}
