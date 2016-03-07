#include "../AuxiliaryClasses/Element.h"
#include "../AuxiliaryClasses/point3D.h"


#include <vector>

#ifndef _TRIHASH
#define _TRIHASH_

class TriHash
{
	
	int slotsPerDimension; //number of equally spaced subdivisions in each dimension
	vector< vector<double> > limits; // three rows (x,y,z) and two columns(min,max), keeps the information on limits in each dimension

	vector<vector<vector<vector<Element *> > > > elements;

	double tol; // tolerance to prevent numerical representation errors

	int numElems;

	public:

	// creator
	TriHash(vector<Element *> vec, int numC=-1, double iTol=0.0000000001);

	// destructor
	~TriHash(){};

	int getNumElems();
	int getSlotsPerDimension();

	int findSlot(double val, char type, bool checkOutOfBounds=false); // type=x,y,z returns the slot (for the givenn) where value val falls into. "checkOutOfBounds" indictes if we get out of bonds querys back IN bounds or if we throw an exception.
  
	double slotLowerBound(int c, char type); // type=x,y,z returns starting value of slot number c
	double slotUpperBound(int c, char type); // type=x,y,z returns finishing value of slot number c 

	vector<int> slotsTouched(double min, double max, char type); // returns minimum and maximum slots touched by an interval in a dimension x,y o z (indicated by type)

	vector<Element *> neigbors(point3D p, double eps); // returns all neigbors at distance at most eps from p, if it finds p it does not return it
	Element * nearestNeighbor(point3D p);

	vector<Element *> elementsSlot(int i,int j, int k){ return (elements[i][j][k]);}

	vector<Element *> uniformSampling(int totalContribution); 

	// function to return all points further than a given distance from a certain point TO DO!!!!
	
	int slots(){return slotsPerDimension;}

	friend ostream& operator<<(ostream& os,TriHash c) 
	{	
		os<<"Trihash :: Writing trihash of "<<c.slotsPerDimension<<" at each coordinate: "<<endl;

		for(int i=0;i<c.slotsPerDimension;i++)
		{	
			os<<" Points in slot "<<i<<" in x, ( "<<c.slotLowerBound(i,'x')<<","<<c.slotUpperBound(i,'x')<<")"<<endl;
			for(int j=0;j<c.slotsPerDimension;j++)
			{	
				os<<"\t\t Points in slot "<<j<<" in y, ("<<c.slotLowerBound(j,'y')<<","<<c.slotUpperBound(j,'y')<<")"<<endl;
				for(int k=0;k<c.slotsPerDimension;k++)
				{
					if((c.elements[i][j][k]).size()>0)
					{
						os<<"\t\t\t\t Points in slot "<<k<<" in z, ("<<c.slotLowerBound(k,'z')<<","<<c.slotUpperBound(k,'z')<<")"<<endl;
						
						vector<Element *>::iterator it;
						for(it=(c.elements[i][j][k]).begin();it!=(c.elements[i][j][k]).end();it++)
						{	
							os<<"\t\t\t\t\t\t"<<(*it)->getPoint()<<endl;
						}
					}
				}
			}		
		}
		os<<"Trihash:: Finished writing  "<<endl;
		return os;
	}


	private: 
		vector<Element *> firstElementsInSlot(int i, int j, int k, int numElements );
		vector<Element *> randomElementsInSlot(int i, int j, int k, int numElements );
};

#endif
