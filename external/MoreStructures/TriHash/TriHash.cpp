#include "TriHash.h"
#include <cstdlib>
#include <float.h>

TriHash::TriHash(vector<Element *> vec, int numC, double iTol)
{
	slotsPerDimension = numC;
	tol=iTol;

	numElems=vec.size();

	if(slotsPerDimension==-1) // in the case where the number of cells was not specified, make charge factor to be near 1
	{
		 slotsPerDimension = pow(vec.size(),1/(3.));
		 if(slotsPerDimension<2) {slotsPerDimension=2;}	// 8 cells is the minimum possible
	}
	
	// First, find numeric limits

	//cout<<"TriHash::TriHash adjustet slot number: "<<slotsPerDimension<<endl;

	//initialize limits using three vectors of two doubles (min and max)
	limits	= vector< vector<double> >();

	limits.push_back(vector<double>());
	limits.push_back(vector<double>());
	limits.push_back(vector<double>());

	if(vec.size()<1)
	{
		//cout<<"TriHash::TriHash smalish vector???? "<<vec.size()<<endl;
		throw("TriHash::TriHash smalish vector???? ");
	}

	//cout<<"TriHash::TriHash computing limits"<<endl;

	// initialize maximum and minimum values to the coordinates of the first point	
	limits[0].push_back(vec[0]->getPoint().getX()); //minim en X
	limits[0].push_back(vec[0]->getPoint().getX()); //minim en X
	limits[1].push_back(vec[0]->getPoint().getY()); //minim en X
	limits[1].push_back(vec[0]->getPoint().getY()); //minim en X
	limits[2].push_back(vec[0]->getPoint().getZ()); //minim en X
	limits[2].push_back(vec[0]->getPoint().getZ()); //minim en X

	//cout<<"TriHash::TriHash limits computed"<<endl;
	
	// traverse grid updating limits
	vector<Element *>::iterator it;
	for(it=vec.begin();it!=vec.end();it++)
	{
		point3D currentP = (*it)->getPoint();
		
		if(currentP.getX() < limits[0][0]) limits[0][0] = currentP.getX();
		if(currentP.getX() > limits[0][1]) limits[0][1] = currentP.getX();

		if(currentP.getY() < limits[1][0]) limits[1][0] = currentP.getY();
		if(currentP.getY() > limits[1][1]) limits[1][1] = currentP.getY();

		if(currentP.getZ() < limits[2][0]) limits[2][0] = currentP.getZ();
		if(currentP.getZ() > limits[2][1]) limits[2][1] = currentP.getZ();
	}

	//cout<<"TriHash::TriHash initialized limits: <<endl<<"x: ("<<limits[0][0]<<" , "<<limits[0][1]<<")"<<endl;
	//cout<<"y: ("<<limits[1][0]<<" , "<<limits[1][1]<<")"<<endl;
	//cout<<"z: ("<<limits[2][0]<<" , "<<limits[2][1]<<")"<<endl;

	// now that we know its dimensions, distribute the points throughout the grid
	// memory initialization
	elements = vector<vector<vector<vector<Element *> > > >(slotsPerDimension);

	for(int i=0;i<slotsPerDimension;i++)
	{
		elements[i] = vector< vector<vector<Element *> > >(slotsPerDimension); 
		for(int j=0;j<slotsPerDimension;j++)
		{
			elements[i][j]=vector< vector<Element *> >(slotsPerDimension);
		}
	}
	//cout<<"TriHash::TriHash grid initialized"<<endl;

// alternative initialization
/*for(int i=0;i<slotsPerDimension;i++)
	{
		grid.push_back(vector<vector<vector<Element *> > > () );

		for(int j=0;j<slotsPerDimension;j++)
		{
			grid[i].push_back(vector<vector<Element *> >  () );
	
			for(int k=0;k<slotsPerDimension;k++)
			{
				grid[i][j].push_back(vector<Element *>() );
			}
		}
	}*/

	// distribute points in grid slots
	for(it=vec.begin();it!=vec.end();it++)
	{
		int x,y,z;
		point3D currentP = (*it)->getPoint();

		x=findSlot(currentP.getX(),'x');
		y=findSlot(currentP.getY(),'y');
		z=findSlot(currentP.getZ(),'z');
	
		//cout<<"TriHash::TriHash putting "<<*(*it)<<" in slot "<<x<<y<<z<<endl;

		elements[x][y][z].push_back(*it);
	}

	//cout<<"TriHash::TriHash points distributed"<<endl;
}

int TriHash::findSlot(double val, char type,bool margin)
{
	double min,max;
	int returnValue;

	//cout<<"TriHash::findSlot limits "<<endl<<"x: ("<<limits[0][0]<<" , "<<limits[0][1]<<")"<<endl;
	//cout<<"y: ("<<limits[1][0]<<" , "<<limits[1][1]<<")"<<endl;
	//cout<<"z: ("<<limits[2][0]<<" , "<<limits[2][1]<<")"<<endl;

	switch( type )
	{
		case 'x' : 
			   min = limits[0][0];	
			   max = limits[0][1];			
			   break;
		case 'y' : 
			   min = limits[1][0];	
			   max = limits[1][1];			
			   break;
		case 'z' : 
			   min = limits[2][0];	
			   max = limits[2][1];			
			   break;

		default  : cout<<"TriHash::findSlot(double val, char type) wrong slot type???? "<<endl;
			   throw("TriHash::findSlot(double val, char type) wrong slot type???? ");
			   break;
	}			


	//check for extreme cases 
	if(fabs(max-val)<tol) returnValue=slotsPerDimension-1;
	else if(fabs(min-val)<tol) returnValue=0;
	else
	{
		double pas = (fabs(max-min)/slotsPerDimension);

		returnValue = (int)( (val-min) /pas);
	}	

	if( (returnValue>=slotsPerDimension) || (returnValue<0) )
	{
		if(!margin)
		{
			cout<<"TriHash::findSlot(double val, char type) wrong slot? "<<returnValue<<endl;
			throw("TriHash::findSlot(double val, char type) wrong slot? ");
		}
		else 	// set to the last slot out-of-bound queries (for example, for sentinel-guided searches
		{
			if(returnValue>=slotsPerDimension) returnValue=slotsPerDimension-1;
			else returnValue=0;
		}
	}

	//cout<<"TriHash::findSlot finished "<<returnValue<<endl<<endl<<endl<<endl;

	return returnValue ;

}


double TriHash::slotLowerBound(int c, char type)
{
	double min,max;

	switch( type )
	{
		case 'x' : 
			   min = limits[0][0];	
			   max = limits[0][1];			
			   break;
		case 'y' : 
			   min = limits[1][0];	
			   max = limits[1][1];			
			   break;
		case 'z' : 
			   min = limits[2][0];	
			   max = limits[2][1];			
			   break;

		default  : cout<<"TriHash::slotLowerBound wrong slot type???? "<<endl;
			   throw("TriHash::slotLowerBound wrong slot type???? ");
			   break;
	}			

	double pas = (fabs(max-min)/slotsPerDimension);

	return (min+pas*c);
}

double TriHash::slotUpperBound(int c, char type)
{
	return slotLowerBound(c+1,type);
}

// return the minimum and maximum index of the slots affected
vector<int> TriHash::slotsTouched(double min, double max, char type)
{
	vector<int> returnValue = vector<int>(2);

	returnValue[0] = findSlot(min, type,true); 
	returnValue[1] = findSlot(max, type,true);

	return returnValue;		
}

vector<Element *> TriHash::neigbors(point3D p, double eps)
{
	//cout<<"TriHash::neigbors neighbors search for "<<p<<" at distance "<<eps<<endl;
	// find points in a query cube and then choose the ones inside the query sphere
	vector<Element *> returnValue = vector<Element *>();

	vector<int> limitsX = slotsTouched(p.getX()-eps, p.getX()+eps, 'x');
	vector<int> limitsY = slotsTouched(p.getY()-eps, p.getY()+eps, 'y');
	vector<int> limitsZ = slotsTouched(p.getZ()-eps, p.getZ()+eps, 'z');

	//cout<<"TriHash::neigbors limits values found: "<<endl;
	//cout<<"x: ("<<limitsX[0]<<" , "<<limitsX[1]<<")"<<endl;
	//cout<<"y: ("<<limitsY[0]<<" , "<<limitsY[1]<<")"<<endl;
	//cout<<"z: ("<<limitsZ[0]<<" , "<<limitsZ[1]<<")"<<endl;


	for(int i=limitsX[0];i<=limitsX[1];i++)
	{	
		for(int j=limitsY[0];j<=limitsY[1];j++)
		{	
			for(int k=limitsZ[0];k<=limitsZ[1];k++)
			{
				vector<Element *>::iterator it;
				for(it=(elements[i][j][k]).begin();it!=(elements[i][j][k]).end();it++)
				{	
					point3D currentP=(*it)->getPoint();
					double dist = currentP.dist(p);

					if( p!=currentP && dist<eps)
					{
						returnValue.push_back(*it);
					}
				}
			}
		}		
	}



	return returnValue;
}



Element * TriHash::nearestNeighbor(point3D p){

	Element *res = new Element();
	float bestDist = FLT_MAX;

	int i = findSlot(p.getX(), 'x', true);
	int j = findSlot(p.getY(), 'y', true);
	int k = findSlot(p.getZ(), 'z', true);

	vector<Element *> slot = elementsSlot(i, j, k);

	// more points than itself.
	if(slot.size()>1){

		for (int l = 0; l < slot.size(); ++l) {
			if(p!=slot.at(l)->getPoint()) { // skip itself
				float dist = p.dist(slot.at(l)->getPoint());

				if (dist < bestDist) {

					res->setPoint(slot.at(l)->getPoint());
					bestDist = dist;
				}
			}
		}
	}

	// find in the close slots using the bestDist as eps. If there is a closest point,
	// we'll find it. If not, the current point is the nearest neighbour.

	vector<Element *> neighs = neigbors(p, bestDist);
	if(!neighs.empty()){
		for (int l = 0; l < neighs.size(); ++l) {
			float dist = p.dist(neighs.at(l)->getPoint());

			if(p!=neighs.at(l)->getPoint() && dist < bestDist){
				res->setPoint(neighs.at(l)->getPoint());
				bestDist = dist;
			}
		}
	}
	return res;

}

// given a contribution, return points from the different slots as uniformly as possible
vector<Element *> TriHash::uniformSampling(int totalContribution)
{
	// define return value
	vector<Element *> ret = vector<Element *>();

	//cout<<"vector<Element *> TriHash::uniformSampling(int totalContribution) Begin"<<endl;

	// if we need to contribute more grid than what we have, we throw an exception
	if(totalContribution > numElems )
	{
		cout<<"vector<Element *> TriHash::uniformSampling(int totalContribution) Sampling too big! contribution "<<totalContribution<<" number of grid "<<numElems<<endl;
		throw("vector<Element *> TriHash::uniformSampling(int totalContribution) Sampling too big!");
	}

	// the main problem is deciding how many points will be contributed by each slot.
	// to aid in this task, we define an int to keep current Contribution and a matrix to keep current slot contributions.
	
	vector< vector< vector<int> > > contributionPerSlot = vector< vector< vector<int> > >(slotsPerDimension);
	// initialize the matrix
	//cout<<"vector<Element *> TriHash::uniformSampling(int totalContribution) contributionPerSlot begin initialization"<<endl;

	for(int i=0;i<slotsPerDimension;i++)
	{
		contributionPerSlot[i] = vector< vector<int> >(slotsPerDimension);
		for(int j=0;j<slotsPerDimension;j++)
		{
			contributionPerSlot[i][j] = vector<int>(slotsPerDimension,0);
		}
	}

	//cout<<"vector<Element *> TriHash::uniformSampling(int totalContribution) contributionPerSlot initialized"<<endl;

	int contributionsSoFar=0;

	// first, decide orientative contribution
	int tentContribSlot = totalContribution/((double)pow(slotsPerDimension,3));

	// decide contributions and store them in the contribution matrix.
	int i,j,k,compt;
	compt=0;
	while (contributionsSoFar < totalContribution)	
	{
		i=0;
		while(i<slotsPerDimension)
		{
			j=0;
			while(j<slotsPerDimension)
			{		
				k=0;
				while(k<slotsPerDimension)
				{		
					//cout<<"vector<Element *> TriHash::uniformSampling(int totalContribution) computing contributionPerSlot "<<i<<j<<k<<" "<<contributionsSoFar<<" total contribution "<<totalContribution<<" compt "<<compt<<endl;

					//can the current slot contribute more?
					if(elements[i][j][k].size() > contributionPerSlot[i][j][k])
					{
						//cout<<"vector<Element *> TriHash::uniformSampling(int totalContribution)                    can contribute more!!!!!!"<<endl;
						// this slot can contribute more
						int contribution = tentContribSlot + compt;

						// beware of the maximum number of grid in the slot
						if(contribution > elements[i][j][k].size()) contribution = elements[i][j][k].size();
						 
						// we now have decided the new contribution, adjust total contributionsSoFar and contributionPerSlot[i][j][k]
						contributionsSoFar = contributionsSoFar - contributionPerSlot[i][j][k] + contribution;
						contributionPerSlot[i][j][k] = contribution;	

					}
//					else
//					{
//						cout<<"vector<Element *> TriHash::uniformSampling(int totalContribution)                    canNOT contribute more!!!!!!"<<endl;
//					}
					
					// chechk that we are not already finished				
					if( contributionsSoFar >= totalContribution)
					{
						i=slotsPerDimension;
						j=slotsPerDimension;
						k=slotsPerDimension;
					}	
					else
					{	
						k++;
					}
				}
			j++;
			}
		i++;
		}
		// we gradually increase contributions in 1 each time
		compt++;
	}

//	cout<<"vector<Element *> TriHash::uniformSampling(int totalContribution) build return value "<<endl;
	
	// once we have decided the contributions, build the return value
	for(int i=0;i<slotsPerDimension;i++)
	{
		for(int j=0;j<slotsPerDimension;j++)
		{
			for(int k=0;k<slotsPerDimension;k++)
			{
				//cout<<"vector<Element *> TriHash::uniformSampling(int totalContribution) build return value,slot "<<i<<j<<k<<" contribution size: "<<contributionPerSlot[i][j][k]<<endl;

				// take the corresponding contribution from the slot
				//vector<Element *> aux = firstElementsInSlot(i,j,k,contributionPerSlot[i][j][k]);
				vector<Element *> aux = randomElementsInSlot(i,j,k,contributionPerSlot[i][j][k]);

				//append to the result vector
				ret.insert(ret.end(),aux.begin(),aux.end());
			}
		}
	}

	//cout<<"vector<Element *> TriHash::uniformSampling(int totalContribution) end"<<endl;
	return ret;
} 


vector<Element *> TriHash::firstElementsInSlot(int i, int j, int k, int numElements )
{
	vector<Element *> ret = vector<Element *>();
	if(elements[i][j][k].size()<numElements)
	{
		cout<<"vector<Element *> TriHash::firstElementsInSlot(int i, int j, int k, int numElements ), exception, you ask for too many grid! "<<endl;
		throw("vector<Element *> TriHash::firstElementsInSlot(int i, int j, int k, int numElements ), exception, you ask for too many grid! ");
	}

	for(int l=0;l<numElements;l++) ret.push_back(elements[i][j][k][l]);
	

	return ret;
}

vector<Element *> TriHash::randomElementsInSlot(int i, int j, int k, int numElements )
{

	vector<Element *> ret = vector<Element *>();

	if(elements[i][j][k].size()<numElements)
	{
		cout<<"vector<Element *> TriHash::firstElementsInSlot(int i, int j, int k, int numElements ), exception, you ask for too many grid! "<<endl;
		throw("vector<Element *> TriHash::firstElementsInSlot(int i, int j, int k, int numElements ), exception, you ask for too many grid! ");
	}

	// create a random permutation of the 
	vector<int> permutation = vector<int>(elements[i][j][k].size());

	// first the vector is sorted
	for(int l=0;l<permutation.size();l++) permutation[l]=l;

	// now create random permutation
	for(int l=0;l<permutation.size();l++) 
	{
		// choose a random number within the range
		int randomValue = (permutation.size()-1)*(((double)rand())/(double)RAND_MAX);
		//swap the values of l and randomValue
		int aux = permutation[l];
		permutation[l]=permutation[randomValue];
		permutation[randomValue]=aux;
	}

	for(int l=0;l<numElements;l++) ret.push_back(elements[i][j][k][permutation[l]]);
	

	return ret;
}

int TriHash::getNumElems() {

	return numElems;
}

int TriHash::getSlotsPerDimension() {

	return slotsPerDimension;
}
