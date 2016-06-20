//
// Created by yago on 16/06/15.
//

#ifndef PIPELINE_DET_HIERARCHICALNORMALSPACESAMPING_H
#define PIPELINE_DET_HIERARCHICALNORMALSPACESAMPING_H

#include "../IDetection.h"
#include "../include/Converters/converterCGAL.h"
#include <CGAL/pca_estimate_normals.h>

vector<Point* >* randomSample(vector<Point* >* v,int numSamples);


class det_HierarchicalNormalSpaceSampling : public IDetection {

    bool initialised;
    vector<vector<Point*>* > pyramid; // this structure will contain the points at every level // the last level is implicit and contains all the points (aproximadament) data->B->allPoints

    vector<vector<vector<Point*> > > buckets; //this structure will contain the points divided by angle buckets according to their normals.

    int numLevels;
    int numBucketsDimension;
    int minNumber; // the number of elements in the upper level. In intermediate levels, it increases exponentially (num_elements_level_i = minNumber + a^i where a=pow(|B|-minNumber,1/

public:
    det_HierarchicalNormalSpaceSampling()
    {
        numBucketsDimension=10; // fixed as we fancied

        numLevels=0;
        minNumber=-1; // the number of elements in the upper level. In intermediate levels, it increases exponentially (num_elements_level_i = minNumber + a^i where a=pow(|B|-minNumber,1/
        initialised=false;
        data=NULL;
        buckets=vector<vector<vector<Point*> > >(numBucketsDimension);
        for(int i=0;i<numBucketsDimension;i++){
            buckets[i]=vector<vector<Point*> >(numBucketsDimension);
            for(int j=0;j<numBucketsDimension;j++){
                buckets[i][j]= vector<Point*>();
            }
        }


    }

    ~det_HierarchicalNormalSpaceSampling() { }

    void setData(Data *d);
    void execute();

    void extractHierarchicalNormalSpaceSampling(int sample);

private:
    void hierarchizeSet(ElementSet *X);
    vector<vector<int> >*  contributionPerBucket(int totalContribution, vector<vector<vector<Point*> > > *elementsInBucket);


};


#endif //PIPELINE_DET_HIERARCHICALNORMALSPACESAMPING_H
