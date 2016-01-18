#ifndef _COMPRESSED_CANDIDATE_ZONE_
#define _COMPRESSED_CANDIDATE_ZONE_

#include <iostream>
#include <list>

#include "./CompressedONode.h"

class CompressedCandidateZone
{
	private:
		int zone_type;	//1.- one node
				//2.- two nodes
				//4.- quartet
				//8.- Octet
				//0.- not_initialized
		int nivellMaxim;
		
		CompressedONode *q1;
		CompressedONode *q2;
		CompressedONode *q3;
		CompressedONode *q4;
		CompressedONode *q5;
		CompressedONode *q6;
		CompressedONode *q7;
		CompressedONode *q8;

	public:
		CompressedCandidateZone( int init_type=0 , CompressedONode *init_q1=NULL, CompressedONode *init_q2=NULL, CompressedONode *init_q3=NULL,	CompressedONode *init_q4=NULL, CompressedONode *init_q5=NULL, CompressedONode *init_q6=NULL, CompressedONode *init_q7=NULL, CompressedONode *init_q8=NULL );
		~CompressedCandidateZone( );
		
		bool in_zone_query(point3D p); //Asks if a point is inside a certain candidate zone
		// returns the ith element in the candidate zone.
		Element getIthElement(unsigned int i);
		
		// returns the ith point in the candidate zone.
		point3D getIthPoint(unsigned int i);
	
		
	// NEW!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		int cardinal();
				
		//copy constructor
		CompressedCandidateZone(const CompressedCandidateZone &c); 

		// assignment operator
		CompressedCandidateZone& operator=(const CompressedCandidateZone &c); 

		
		// range searching function (caldria ferla tambe per a quadtree).
		list<Element> weightedNeighbors(Element *e,double epsilon);
		
		// delete operation 
		void deleteElement(Element e);	
		
		void write(ostream& os)
		{
			if (zone_type == 1)
			{
				os<<endl;			
				os<< "TIPUS 1:"<<endl;
				if(q1!=NULL)
				{				
					q1->write2(os);
				}
				else
				{
					cout<<"ZONA DE TIPUS 1 BUIDA"<<endl;
				}
			}
			else if (zone_type == 2)
			{
				os<<endl;			
				os<< "TIPUS 2:"<<endl;
				os<< "Primer node:"<<endl;
				if(q1!=NULL)
				{				
					q1->write2(os);
				}
				else
				{
					cout<<"NODE 1 BUIT"<<endl;
				}
				
				os<< "Segon node:"<<endl;
				if(q2!=NULL)
				{				
					q2->write2(os);
				}
				else
				{
					cout<<"NODE 2 BUIT"<<endl;
				}
			}
			else if ( (zone_type == 4) )
			{
				os<<endl;			
				os<< "TIPUS 4:"<<endl;
				os<< "Primer node:"<<endl;
				if(q1!=NULL)
				{				
					q1->write2(os);
				}
				else
				{
					cout<<"NODE 1 BUIT"<<endl;
				}
				os<< "Segon node:"<<endl;
				if(q2!=NULL)
				{				
					q2->write2(os);
				}
				else
				{
					cout<<"NODE 2 BUIT"<<endl;
				}
				os<< "Tercer node:"<<endl;
				if(q3!=NULL)
				{				
					q3->write2(os);
				}
				else
				{
					cout<<"NODE 3 BUIT"<<endl;
				}
				
				os<< "Quart node:"<<endl;
				if(q4!=NULL)
				{				
					q4->write2(os);
				}
				else
				{
					cout<<"NODE 4 BUIT"<<endl;
				}
			}
			else // zone_type == 8
			{
				os<<endl;			
				os<< "TIPUS 8:"<<zone_type<<endl;
				os<< "Primer node:"<<endl;
				if(q1!=NULL)
				{				
					q1->write2(os);
				}
				else
				{
					cout<<"NODE 1 BUIT "<<endl;
				}
				os<< "Segon node:"<<endl;
				if(q2!=NULL)
				{				
					q2->write2(os);
				}
				else
				{
					cout<<"NODE 2 BUIT "<<endl;
				}
				os<< "Tercer node:"<<endl;
				if(q3!=NULL)
				{				
					q3->write2(os);
				}
				else
				{
					cout<<"NODE 3 BUIT "<<endl;
				}
				os<< "Quart node:"<<endl;
				if(q4!=NULL)
				{				
					q4->write2(os);
				}
				else
				{
					cout<<"NODE 4 BUIT "<<endl;
				}
				os<< "Cinque node:"<<endl;
				if(q5!=NULL)
				{				
					q5->write2(os);
				}
				else
				{
					cout<<"NODE 5 BUIT "<<endl;
				}
				os<< "Sise node:"<<endl;
				if(q6!=NULL)
				{				
					q6->write2(os);
				}
				else
				{
					cout<<"NODE 6 BUIT "<<endl;
				}
				os<< "Sete node:"<<endl;
				if(q7!=NULL)
				{				
					q7->write2(os);
				}
				else
				{
					cout<<"NODE 7 BUIT "<<endl;
				}
				os<< "Vuite node:"<<endl;
				if(q8!=NULL)
				{				
					q8->write2(os);
				}
				else
				{
					cout<<"NODE 8 BUIT "<<endl;
				}
			}
		}
		
		friend ostream& operator<<(ostream& os,CompressedCandidateZone c) 
		{
			if(!c.buida())
			{
				os<<endl;			
				os<<"VAIG A write UNA ZONA CANDIDATA COMPRESSED"<<endl;
				c.write(os);
			}
			else
			{
				os<<"Zona candidata buida"<<endl;
				
			} 
			return os;
		}
		
		//bool empty(){return q1==NULL;}
	
		int type();	
		
		bool buida();
		
		void anulaNode(CompressedONode *p);

		bool acceptable(CompressedInformacioGeometrica info,double epsilon);
		
		vector<Element *> elementsZona();
		void marcarElement(Element * e);
		void mostraMides()
		{

			if (zone_type == 1)
			{
				cout<< "TIPUS 1: "<<endl;
				if(q1!=NULL)
				{				
					cout<<"Mida Node únic: "<<q1->getMida()<<endl;
				}
				else
				{
					cout<<"ZONA DE TIPUS 1 BUIDA"<<endl;
				}
			}
			else if (zone_type == 2)
			{
				cout<< "TIPUS 2: "<<endl;
				if(q1!=NULL)
				{				
					cout<<"Mida Primer Node : "<<q1->getMida()<<endl;
				}
				else
				{
					cout<<"NODE 1 BUIT"<<endl;
				}
				
				if(q2!=NULL)
				{				
					cout<<"Mida segon Node : "<<q2->getMida()<<endl;
				}
				else
				{
					cout<<"NODE 2 BUIT"<<endl;
				}
			}
			else if (zone_type == 4)
			{
				cout<< "TIPUS 4: "<<zone_type<<endl;
				if(q1!=NULL)
				{				
					cout<<"Mida Primer Node : "<<q1->getMida()<<endl;
				}
				else
				{
					cout<<"NODE 1 BUIT "<<endl;
				}
				if(q2!=NULL)
				{				
					cout<<"Mida Segon Node : "<<q2->getMida()<<endl;
				}
				else
				{
					cout<<"NODE 2 BUIT "<<endl;
				}
				if(q3!=NULL)
				{				
					cout<<"Mida Tercer Node : "<<q3->getMida()<<endl;
				}
				else
				{
					cout<<"NODE 3 BUIT "<<endl;
				}
				if(q4!=NULL)
				{				
					cout<<"Mida Quart Node : "<<q4->getMida()<<endl;
				}
				else
				{
					cout<<"NODE 4 BUIT "<<endl;
				}
			}

			else // zone_type == 8
			{
				cout<< "TIPUS 8: "<<zone_type<<endl;
				if(q1!=NULL)
				{				
					cout<<"Mida Primer Node : "<<q1->getMida()<<endl;
				}
				else
				{
					cout<<"NODE 1 BUIT "<<endl;
				}
				if(q2!=NULL)
				{				
					cout<<"Mida Segon Node : "<<q2->getMida()<<endl;
				}
				else
				{
					cout<<"NODE 2 BUIT "<<endl;
				}
				if(q3!=NULL)
				{				
					cout<<"Mida Tercer Node : "<<q3->getMida()<<endl;
				}
				else
				{
					cout<<"NODE 3 BUIT "<<endl;
				}
				if(q4!=NULL)
				{				
					cout<<"Mida Quart Node : "<<q4->getMida()<<endl;
				}
				else
				{
					cout<<"NODE 4 BUIT "<<endl;
				}
				if(q5!=NULL)
				{				
					cout<<"Mida Cinque Node : "<<q5->getMida()<<endl;
				}
				else
				{
					cout<<"NODE 5 BUIT "<<endl;
				}
				if(q6!=NULL)
				{				
					cout<<"Mida Sise Node : "<<q6->getMida()<<endl;
				}
				else
				{
					cout<<"NODE 6 BUIT "<<endl;
				}
				if(q7!=NULL)
				{				
					cout<<"Mida Sete Node : "<<q7->getMida()<<endl;
				}
				else
				{
					cout<<"NODE 7 BUIT "<<endl;
				}
				if(q8!=NULL)
				{				
					cout<<"Mida Vuite Node : "<<q8->getMida()<<endl;
				}
				else
				{
					cout<<"NODE 8 BUIT "<<endl;
				}
			}
		}

		int getCandidateZoneType(){return zone_type;}
		
};

#endif
