//
// Created by yago on 16/06/15.
//

#include "det_HierarchicalNormalSpaceSampling.h"

void det_HierarchicalNormalSpaceSampling::setData(Data *d)
{
    data = d;
    int veryMinimum=5;
    // at this point, determine the parameters of the hierarchy
    numLevels=data->params.nLevels;
    minNumber= (data->B->nPoints()/pow(4,numLevels)< veryMinimum) ? veryMinimum : data->B->nPoints()/pow(4,numLevels); // the number of elements in the upper level. In intermediate levels, it increases exponentially (num_elements_level_i = minNumber + a^i where a=pow(|B|-minNumber,1/

 //   cout<<"numElements "<<data->B->nPoints()<<endl;
 //   cout << "HierarchicalNormalSpaceSampling setting data with "<<numLevels<<" "<<minNumber<<endl;
}

void det_HierarchicalNormalSpaceSampling::execute()
{
 //   cout<<"det_HierarchicalNormalSpaceSampling:: executing"<<endl;
    if(!initialised)
    {
        // call hierarchize
        initialised=true;
        hierarchizeSet(data->B);
    }

//    extractHierarchicalNormalSpaceSampling(data->A, data->params.nSamples);
    extractHierarchicalNormalSpaceSampling(data->params.nSamples);
 //   cout<<"det_HierarchicalNormalSpaceSampling:: finished executing"<<endl;

}


void det_HierarchicalNormalSpaceSampling::hierarchizeSet(ElementSet *X) {

  //  cout << "HierarchicalNormalSpaceSampling running..., Hierarchizing with "<<    data->params.nLevels<<" levels " << endl;
    const double PI = 3.141592653589793;

    // First, compute normals
    converterCGAL acgal;
    list<PointVectorPair> points = acgal.points2CGAL_list(X->getAllpoints());

    vector<Point *> *allThePoints = X->allpoints;

    // Estimates normals direction.
    // Note: pca_estimate_normals() requires an iterator over points
    // as well as property maps to access each point's position and normal.
    const int nb_neighbors = 3; // K-nearest neighbors = 3 rings
    CGAL::pca_estimate_normals(points.begin(), points.end(), CGAL::First_of_pair_property_map<PointVectorPair>(), CGAL::Second_of_pair_property_map<PointVectorPair>(), nb_neighbors);

    X->setNormals(acgal.CGAL2normals(points));

    // Bin Normals
    list<PointVectorPair>::iterator it;
    double radius,theta,phi,x,y,z;
    int binX,binY;
    double binSize=PI/numBucketsDimension;

    int compt=0;
    for(it=points.begin();it!=points.end();it++)        // for each normal,
     {
         // compute its polar coordinates
         x=(*it).second.x();
         y=(*it).second.y();
         z=(*it).second.z();

         //radius = sqrt(x * x + y * y + z * z);
         radius=1;
         phi = acos(z / radius); // angle in radiants from 0 to Pi
         theta = atan2(y, x); // angle in radians form -Pi to Pi

         // put it in the right bin

         binX = floor(phi/binSize);   //corresponds to angle phi
         binY = floor((theta+PI)/(2*binSize));   //corresponds to angle phi

         if(binX==numBucketsDimension)binX--;
         if(binY==numBucketsDimension)binY--;

         // Add it to the Bin
         buckets[binX][binY].push_back( (*allThePoints)[compt] );
         //cout << "HierarchicalNormalSpaceSampling Listing points and normals. "<<(*it).first<<" and "<<(*it).second<<" has angles "<<phi<<","<<theta<<" and radius "<<radius<<" goes to bins ("<<binX<<","<<binY<<") this bucket has this many points "<<buckets[binX][binY].size()<<endl;
         compt++;
     }

    // Build piramid with the proper points
    // The lowermost set is the whole of X
    pyramid=vector<vector<Point*>* >();
    pyramid.push_back(allThePoints);

    double currentSize=X->nPoints()/4;
    int level=1;
    int totalBuckets=numBucketsDimension*numBucketsDimension;
    while((level<numLevels)&&(currentSize>minNumber))
    {
        vector<vector<int> >* contributions=contributionPerBucket(currentSize, &buckets);
   //     cout<<"Starting LEVEL "<<level<<" need "<<currentSize<<" going as far down as "<<minNumber<<endl;

        // now go over the bucket and sample randomly the proper number in each bucket
        // for every bucket
        vector<Point *> *pointsInThisLevel= new vector<Point *>();
        for(int i=0;i<numBucketsDimension;i++)
        {
            for(int j=0;j<numBucketsDimension;j++)
            {
                // Get random dampling of the assgined contributions per bucket in this bucket
                vector<Point *> *randSampling=randomSample(&buckets[i][j],(*contributions)[i][j]);

                // add the elements in the uniform sampling to cAux
                for (int l = 0; l < randSampling->size(); l++) {
                    pointsInThisLevel->push_back((*randSampling)[l]);
                }
            }
        }

        pyramid.push_back(pointsInThisLevel);
 //       cout<<"FINISHED LEVEL "<<level<<" contributed "<<pointsInThisLevel->size()<<endl;
        level++;
        currentSize=currentSize/4;

    }

    if(level!=numLevels)numLevels=level;// adjust the number of levels if not all full
   // cout << "HierarchicalNormalSpaceSampling running..., finished Hierarchizing with "<<numLevels<<" levels " << endl;
}



// compute the contribution per bucket
vector<vector<int> >*  det_HierarchicalNormalSpaceSampling::contributionPerBucket(int totalContribution, vector<vector<vector<Point*> > > *elementsInBucket)
{

//    cout<<"Searching cont per bucket, we want to get  "<<totalContribution<<endl;

    vector<vector<int> > *contributionPerBucket =  new vector<vector<int> >(numBucketsDimension);
    for(int i=0;i<numBucketsDimension;i++)
    {
        (*contributionPerBucket)[i]=vector<int>(numBucketsDimension);
    }

    // first, decide orientative contribution
    int tentContribSlot = totalContribution/((double)numBucketsDimension*numBucketsDimension);

 //   cout<<"tentative is  "<<tentContribSlot<<endl;


    // store total contribution
    int contributionsSoFar=0;

    // decide contributions and store them in the contribution matrix.
    int i,j,compt;
    compt=0;
    while (contributionsSoFar < totalContribution)
    {
 //       cout<<"Contribution loop "<<contributionsSoFar<<" of "<<totalContribution<<endl;

        i=0;
        while(i<numBucketsDimension)
        {
            j=0;
            while(j<numBucketsDimension)
            {
                //can the current slot contribute more?
                if ((*elementsInBucket)[i][j].size() > (*contributionPerBucket)[i][j])
                {
                   // cout<<"bucket   ("<<i<<","<<j<<") can contribute more, so far "<<(*contributionPerBucket)[i][j]<<" "<<contributionsSoFar<<" of "<<totalContribution<<endl;

                    // this slot can contribute more
                    int contribution = tentContribSlot + compt;

                    // Beware of the contibution asked
                    if (contribution - (*contributionPerBucket)[i][j] > (totalContribution-contributionsSoFar) )
                    {
                     //   cout<<"bucket   ("<<i<<","<<j<<") adjusting congtribution because we are finishing to "<<(totalContribution-contributionsSoFar)<<endl;

                        contribution = totalContribution-contributionsSoFar;
                    }

                    // beware of the maximum number of elements in the slot
                    if (contribution > (*elementsInBucket)[i][j].size())
                    {
                    //    cout<<"bucket   ("<<i<<","<<j<<") adjusting congtribution because of bucket size "<< (*elementsInBucket)[i][j].size()<<endl;
                        contribution = (*elementsInBucket)[i][j].size();
                    }


                    // we now have decided the new contribution, adjust total contributionsSoFar and contributionPerSlot[i][j][k]
                    contributionsSoFar = contributionsSoFar - (*contributionPerBucket)[i][j] + contribution;
                    (*contributionPerBucket)[i][j] = contribution;

                }

                // check that we are not already finished
                if (contributionsSoFar >= totalContribution) {
                   // cout<<"going to return after having contributed  "<<contributionsSoFar<<endl;
                    return contributionPerBucket;
                }
                else {
                    j++;
                }
            }
            i++;
        }

        // we gradually increase contributions in 1 each time
        compt++;
     //  cout<<"increasing contribution to  "<<tentContribSlot + compt<<endl;

    }
}



void det_HierarchicalNormalSpaceSampling::extractHierarchicalNormalSpaceSampling(int sample) {
//cout<<"det_HierarchicalNormalSpaceSampling::extractHierarchicalNormalSpaceSampling, we want to extract "<<sample<<" points"<<endl;
// FIRST find proper level
    int i=pyramid.size()-1;
    bool finished=false;
    while( (i>0)&&!finished)
    {
  //   cout<<"Trying to sample at level "<<i<<" here we have this many points "<<pyramid[i]->size()<<" we need "<<sample<<endl;
        if(sample<pyramid[i]->size())
        {
            vector<Point*> *v=data->B->getWorkpoints();
            v  = randomSample(pyramid[i], sample);
            finished=true;
        }
        else{

            i--;
    //        cout<<"decreased "<<i<<endl;

        }
    }

//    X->createFileFromData("../models/test4Scaled.ply", "../models/provanormals.ply", true);

}

vector<Point *> * randomSample(vector<Point *> *v, int numSamples)
{
//    if(v->size()!=0) cout<<"RANDOM SAMPLING A BUCKET FOR THIS MANY POINTS  "<<numSamples<<" there are this many points here "<<v->size()<<endl;

    double randomDraw=((double)numSamples)/v->size();
    if(randomDraw>1)throw("at det_HierarchicalNormalSpaceSampling, trying to oversample ");
    vector<Point* >* returnVector= new vector<Point* >();
    int samplesRemaining=numSamples;

    srand (time(NULL));

    for(int i=0;i<v->size();i++)
    {
        if(samplesRemaining==0) return returnVector;
        if( ((rand()/RAND_MAX)>randomDraw)||( v->size()-i>=samplesRemaining ) )
        {
           // cout<<"Pushing point "<<(*(*v)[i])<<" remaining "<<samplesRemaining<<endl;
            returnVector->push_back((*v)[i]);
            samplesRemaining--;
        }
    }
    if(samplesRemaining==0) return returnVector;// to prevent the case when we completed the vector in the last step

    return NULL;
}
