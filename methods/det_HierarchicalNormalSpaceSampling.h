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
    vector<vector<Point*>* > pyramidA; // this structure will contain the points at every level // the last level is implicit and contains all the points (aproximadament) data->A->allPoints
    vector<vector<Point*>* > pyramidB; // this structure will contain the points at every level // the last level is implicit and contains all the points (aproximadament) data->B->allPoints

    vector<vector<vector<Point*> > > bucketsA; //this structure will contain the points divided by angle buckets according to their normals.
    vector<vector<vector<Point*> > > bucketsB; //this structure will contain the points divided by angle buckets according to their normals.

    int numLevels;
    int numBucketsDimension;
    int minNumber; // the number of elements in the upper level. In intermediate levels, it increases exponentially (num_elements_level_i = minNumber + a^i where a=pow(|B|-minNumber,1/
    int step; // the ratio of elements between a level and the following one

public:
    ~det_HierarchicalNormalSpaceSampling() { }
    det_HierarchicalNormalSpaceSampling()
    {
        step=2;
        numBucketsDimension=15; // fixed as we fancied

        numLevels=0;
        minNumber=-1; // the number of elements in the upper level. In intermediate levels, it increases exponentially (num_elements_level_i = minNumber + a^i where a=pow(|B|-minNumber,1/
        initialised=false;
        data=NULL;
        bucketsA=vector<vector<vector<Point*> > >(numBucketsDimension);
        for(int i=0;i<numBucketsDimension;i++){
            bucketsA[i]=vector<vector<Point*> >(numBucketsDimension);
            for(int j=0;j<numBucketsDimension;j++){
                bucketsA[i][j]= vector<Point*>();
            }
        }

        bucketsB=vector<vector<vector<Point*> > >(numBucketsDimension);
        for(int i=0;i<numBucketsDimension;i++){
            bucketsB[i]=vector<vector<Point*> >(numBucketsDimension);
            for(int j=0;j<numBucketsDimension;j++){
                bucketsB[i][j]= vector<Point*>();
            }
        }


    }

    void extractHierarchicalNormalSpaceSampling(int sample);
    void extractHierarchicalNormalSpaceLevel(int level);

    void setData(Data *d);
    void execute();
    int getPiramidALevels(){return pyramidA.size();}


private:
    void hierarchizeSet(ElementSet *X, vector<vector<Point*>* > *pyramid, vector<vector<vector<Point*> > > *buckets);
    vector<vector<int> >*  contributionPerBucket(int totalContribution, vector<vector<vector<Point*> > > *elementsInBucket);


};


#endif //PIPELINE_DET_HIERARCHICALNORMALSPACESAMPING_H
