#include "CompressedCandidateZone.h"
#include <vector>
#include <iostream>

#include <iostream>

using namespace std;

const int MAX_LEVEL = 7;

CompressedCandidateZone::CompressedCandidateZone( int init_type , CompressedONode *init_q1, CompressedONode *init_q2, CompressedONode *init_q3, CompressedONode *init_q4,CompressedONode *init_q5, CompressedONode *init_q6, CompressedONode *init_q7, CompressedONode *init_q8 ) 
{

	zone_type = init_type;
	nivellMaxim = MAX_LEVEL;
	q1 = init_q1;
	q2 = init_q2;
	q3 = init_q3;
	q4 = init_q4;
	q5 = init_q5;
	q6 = init_q6;
	q7 = init_q7;
	q8 = init_q8;
}

CompressedCandidateZone::~CompressedCandidateZone() 
{
	// ja es destruira el corresponent octree, si vol
	//zone_type = -1;
	//nivellMaxim = MAX_LEVEL;
	q1 = NULL;
	q2 = NULL;
	q3 = NULL;
	q4 = NULL;
	q5 = NULL;
	q6 = NULL;
	q7 = NULL;
	q8 = NULL;
}

bool CompressedCandidateZone::in_zone_query(point3D p)
{
	bool b1=false, b2=false, b3=false, b4=false, b5=false, b6=false, b7=false, b8=false;
	
	//We search in all necessary nodes until we find our point
	if (q1!=NULL){b1=q1->formaPart(p);}
	if (!b1&&(q2!=NULL)){b2=q2->formaPart(p);}
	if (!b1&&!b2&&(q3!=NULL)){b3=q3->formaPart(p);}
	if (!b1&&!b2&&!b3&&(q4!=NULL)){b4=q4->formaPart(p);}
	if (!b1&&!b2&&!b3&&!b4&&(q5!=NULL)){b5=q5->formaPart(p);}
	if (!b1&&!b2&&!b3&&!b4&&!b5&&(q6!=NULL)){b6=q6->formaPart(p);}
	if (!b1&&!b2&&!b3&&!b4&&!b5&&!b6&&(q7!=NULL)){b7=q7->formaPart(p);}
	if (!b1&&!b2&&!b3&&!b4&&!b5&&!b6&&!b7&&(q8!=NULL)){b8=q8->formaPart(p);}

	return ( b1||b2||b3||b4||b5||b6||b7||b8 );
}


// NEW!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

// returns the number of elements inside a candidate zone.
int CompressedCandidateZone::cardinal()
{
	int res=0;

	if(this!=NULL)
	{
		if (zone_type == 1)
		{
			if(q1!=NULL)
			{
				res=q1->getLlistaElements().size();
			}
			else
			{
				res=0;
			}
		}
		else if (zone_type == 2)
		{
			if(q1!=NULL)
			{
				res= q1->getLlistaElements().size();
			}
			else
			{
				res=0;
			} 
			
			if(q2!=NULL)
			{
				res= res + q2->getLlistaElements().size();
			}
		}
		else if( zone_type == 4)
		{
			if(q1!=NULL)
			{
				res= q1->getLlistaElements().size();
			}
			else
			{
				res=0;
			} 
			if(q2!=NULL)
			{
				res= res + q2->getLlistaElements().size();
			}
			if(q3!=NULL)
			{
				res= res + q3->getLlistaElements().size();
			}
			if(q4!=NULL)
			{
				res= res + q4->getLlistaElements().size();
			}
			
		}
		else // zone_type==8
		{
			if(q1!=NULL)
			{
				res= q1->getLlistaElements().size();
			}
			else
			{
				res=0;
			} 
			if(q2!=NULL)
			{
				res= res + q2->getLlistaElements().size();
			}
			if(q3!=NULL)
			{
				res= res + q3->getLlistaElements().size();
			}
			if(q4!=NULL)
			{
				res= res + q4->getLlistaElements().size();
			}
			if(q5!=NULL)
			{
				res= res + q5->getLlistaElements().size();
			}
			if(q6!=NULL)
			{
				res= res + q6->getLlistaElements().size();
			}
			if(q7!=NULL)
			{
				res= res + q7->getLlistaElements().size();
			}
			if(q8!=NULL)
			{
				res= res + q8->getLlistaElements().size();
			}
		}
	}
	else
	{
		cout<<"ERROR (CompressedCandidateZone.cpp::cardinal) : This es nul!"<<endl;
		exit(-1);
	}	
	return res;
}

// returns the ith element in the candidate zone.

// Precondition, this contains at least i elements
Element CompressedCandidateZone::getIthElement(unsigned int i) 
{
	if (zone_type == 1)
	{
		int unsigned sq1;
		
		// posem les mides de tothom
		if(q1!=NULL)
		{
			sq1=q1->getLlistaElements().size();
		}
		else
		{
			sq1=0;
		}

		// llencem la cerca que toqui!
		
		if(i<sq1)
		{
			return *(q1->getLlistaElements()[i]);

		}
		else
		{
			cout<<"CCZ: getIth 1 fora de rang!!!!!!!!!!!!!!"<<endl;
			exit(-1);
		}
	}
	else if (zone_type == 2)
	{
		unsigned int sq1,sq2;
		
		// posem les mides de tothom
		if(q1!=NULL)
		{
			sq1=q1->getLlistaElements().size();
		}
		else
		{
			sq1=0;
		}

		if(q2!=NULL)
		{
			sq2=q2->getLlistaElements().size();
		}
		else
		{
			sq2=0;
		}
		
		// llencem la cerca que toqui!
		
		if(i<sq1)
		{
			return *(q1->getLlistaElements()[i]);

		}
		else if( i< sq1 + sq2 )
		{
			return *(q2->getLlistaElements()[ i - sq1 ]);
		} 
		else
		{
			cout<<"CCZ: getIth 2 fora de rang!!!!!!!!!!!!!!"<<endl;
			exit(-1);
		}
		
	
	}
	else if (zone_type == 4)
	{
		unsigned int sq1,sq2,sq3,sq4;
		
		// posem les mides de tothom
		if(q1!=NULL)
		{
			sq1=q1->getLlistaElements().size();
		}
		else
		{
			sq1=0;
		}

		if(q2!=NULL)
		{
			sq2=q2->getLlistaElements().size();
		}
		else
		{
			sq2=0;
		}
		
		if(q3!=NULL)
		{
			sq3=q3->getLlistaElements().size();
		}
		else
		{
			sq3=0;
		}
		
		if(q4!=NULL)
		{
			sq4=q4->getLlistaElements().size();
		}
		else
		{
			sq4=0;
		}

		// llencem la cerca que toqui!
		
		if(i<sq1)
		{
			return *(q1->getLlistaElements()[i]);

		}
		else if( i< sq1 + sq2 )
		{
			return *(q2->getLlistaElements()[ i - sq1 ]);
		} 
		else if( i< sq1 + sq2 + sq3 )
		{
			return *(q3->getLlistaElements()[ i - sq1 -sq2]);
		}
		else if( i < sq1 + sq2 + sq3 + sq4)
		{
			return *(q4->getLlistaElements()[ i - sq1 -sq2 -sq3]);
		}
		else
		{
			cout<<"CCZ: getIth 4 fora de rang!!!!!!!!!!!!!!"<<endl;
			exit(-1);
		}
	}
	else  //zone_type == 8
	{
		unsigned int sq1,sq2,sq3,sq4,sq5,sq6,sq7,sq8;
		
		// posem les mides de tothom
		if(q1!=NULL)
		{
			sq1=q1->getLlistaElements().size();
		}
		else
		{
			sq1=0;
		}

		if(q2!=NULL)
		{
			sq2=q2->getLlistaElements().size();
		}
		else
		{
			sq2=0;
		}
		
		if(q3!=NULL)
		{
			sq3=q3->getLlistaElements().size();
		}
		else
		{
			sq3=0;
		}
		
		if(q4!=NULL)
		{
			sq4=q4->getLlistaElements().size();
		}
		else
		{
			sq4=0;
		}

		if(q5!=NULL)
		{
			sq5=q5->getLlistaElements().size();
		}
		else
		{
			sq5=0;
		}

		if(q6!=NULL)
		{
			sq6=q6->getLlistaElements().size();
		}
		else
		{
			sq6=0;
		}
		
		if(q7!=NULL)
		{
			sq7=q7->getLlistaElements().size();
		}
		else
		{
			sq7=0;
		}
		
		if(q8!=NULL)
		{
			sq8=q8->getLlistaElements().size();
		}
		else
		{
			sq8=0;
		}

		// llencem la cerca que toqui!
		
		if(i<sq1)
		{
			return *(q1->getLlistaElements()[i]);

		}
		else if( i< sq1 + sq2 )
		{
			return *(q2->getLlistaElements()[ i - sq1 ]);
		} 
		else if( i< sq1 + sq2 + sq3 )
		{
			return *(q3->getLlistaElements()[ i - sq1 -sq2]);
		}
		else if( i < sq1 + sq2 + sq3 + sq4)
		{
			return *(q4->getLlistaElements()[ i - sq1 -sq2 -sq3]);
		}
		else if( i < sq1 + sq2 + sq3 + sq4 + sq5)
		{
			return *(q5->getLlistaElements()[ i - sq1 -sq2 -sq3 -sq4 ]);
		}
		else if( i < sq1 + sq2 + sq3 + sq4 + sq5 + sq6)
		{
			return *(q6->getLlistaElements()[ i - sq1 -sq2 -sq3 -sq4 -sq5]);
		}
		else if( i < sq1 + sq2 + sq3 + sq4 + sq5 + sq6 + sq7)
		{
			return *(q7->getLlistaElements()[ i - sq1 -sq2 -sq3 -sq4 -sq5 -sq6]);
		}
		else if( i < sq1 + sq2 + sq3 + sq4 + sq5 + sq6 + sq7 + sq8)
		{
			return *(q8->getLlistaElements()[ i - sq1 -sq2 -sq3 -sq4 -sq5 -sq6 -sq7]);
		}
		else
		{
			cout<<"CCZ: getIth 8 fora de rang!!!!!!!!!!!!!!"<<endl;
			exit(-1);
		}
	}

}


// returns the ith point in the candidate zone.

// Precondition, this contains at least i points

point3D CompressedCandidateZone::getIthPoint(unsigned int i)
{
	return getIthElement(i).getPoint();
}

// produces a weighted range searching in the candidate zone: output all neighbors
// with adequate related weight

list<Element> CompressedCandidateZone::weightedNeighbors(Element *e,double epsilon)
{
	list<Element> ret = list<Element>();
	list<Element> retAux = list<Element>();

	if (zone_type == 1)
	{
		if(q1!=NULL)
		{
			ret = q1->weightedNeighbors(e,epsilon);
		}
	}
	else if (zone_type == 2)
	{
		if(q1!=NULL)
		{
			ret = q1->weightedNeighbors(e,epsilon);
		}
		if(q2!=NULL)
		{
			retAux = q2->weightedNeighbors(e,epsilon);
			ret.insert(ret.end(),retAux.begin(),retAux.end());// aqui es podria fer servir "splice!!!!"
		}
	}
	else if (zone_type == 4)
	{
		if(q1!=NULL)
		{
			ret = q1->weightedNeighbors(e,epsilon);
		}
		if(q2!=NULL)
		{
			retAux = q2->weightedNeighbors(e,epsilon);
			ret.insert(ret.end(),retAux.begin(),retAux.end());//  "splice!!!!"
		}
		if(q3!=NULL)
		{
			retAux = q3->weightedNeighbors(e,epsilon);
			ret.insert(ret.end(),retAux.begin(),retAux.end());//  "splice!!!!"
		}
		if(q4!=NULL)
		{
			retAux = q4->weightedNeighbors(e,epsilon);
			ret.insert(ret.end(),retAux.begin(),retAux.end());//  "splice!!!!"
		}
	}
	else //zone_type == 8
	{
		if(q1!=NULL)
		{
			ret = q1->weightedNeighbors(e,epsilon);
		}
		if(q2!=NULL)
		{
			retAux = q2->weightedNeighbors(e,epsilon);
			ret.insert(ret.end(),retAux.begin(),retAux.end());//  "splice!!!!"
		}
		if(q3!=NULL)
		{
			retAux = q3->weightedNeighbors(e,epsilon);
			ret.insert(ret.end(),retAux.begin(),retAux.end());//  "splice!!!!"
		}
		if(q4!=NULL)
		{
			retAux = q4->weightedNeighbors(e,epsilon);
			ret.insert(ret.end(),retAux.begin(),retAux.end());//  "splice!!!!"
		}
		if(q5!=NULL)
		{
			retAux = q5->weightedNeighbors(e,epsilon);
			ret.insert(ret.end(),retAux.begin(),retAux.end());//  "splice!!!!"
		}
		if(q6!=NULL)
		{
			retAux = q6->weightedNeighbors(e,epsilon);
			ret.insert(ret.end(),retAux.begin(),retAux.end());//  "splice!!!!"
		}
		if(q7!=NULL)
		{
			retAux = q7->weightedNeighbors(e,epsilon);
			ret.insert(ret.end(),retAux.begin(),retAux.end());//  "splice!!!!"
		}
		if(q8!=NULL)
		{
			retAux = q8->weightedNeighbors(e,epsilon);
			ret.insert(ret.end(),retAux.begin(),retAux.end());//  "splice!!!!"
		}
	}

	return ret;
}


void CompressedCandidateZone::deleteElement(Element e)
{
	if (zone_type == 1)
	{
		if( (q1!=NULL)&&(!q1->nodeBuit()) )
		{
			if(q1->formaPart(e.getPoint()))
			{
				q1->esborrarElement(&e,nivellMaxim,NULL);
				
				if((q1->tipus())==0)
				{
					delete q1;
					q1=NULL;
				}
			}
			else
			{
				//aixo seria una excepcio
				cout<<"ERROR (CompressedCandidateZone::deleteElement) : 1 INTENTES ESBORRAR ALGU QUE NO ES A LA ZONA CANDIDATA"<<endl;
				exit(-1);
			}
		}
		else
		{
			cout<<"1 PASSO D'ESBORRAR PERQUE LA ZONA ES BUIDA"<<endl;
		}		
	}
	else if (zone_type == 2)
	{
		bool trobat=false;
		if( (q1!=NULL)&&(!q1->nodeBuit()) )
		{
			if(q1->formaPart(e.getPoint()))
			{
				q1->esborrarElement(&e,nivellMaxim,NULL);
				
				trobat=true;
				
				if((q1->tipus())==0)
				{
					delete q1;
					q1=NULL;
				}
			
			}
		}
		else
		{
			//cout<<"21 PASSO D'ESBORRAR PERQUE LA ZONA ES BUIDA"<<endl;
		}
		
		if ( (!trobat) &&(q2!=NULL)&&(!q2->nodeBuit()) )
		{
			if(q2->formaPart(e.getPoint()))
			{
				q2->esborrarElement(&e,nivellMaxim,NULL);
							
				if((q2->tipus())==0)
				{
					delete q2;
					q2=NULL;
				}
			}
			else
			{
				//aixo seria una excepcio
				cout<<"ERROR (CompressedCandidateZone::deleteElement) : 2 INTENTES ESBORRAR ALGU QUE NO ES A LA ZONA CANDIDATA"<<endl;
				exit(-1);
			}
		}
		else
		{
			//cout<<"22 PASSO D'ESBORRAR PERQUE LA ZONA ES BUIDA (o ja l'he trobat) "<<trobat<<endl;
		}
	}
	else if ( (zone_type == 4))// zone_type == 4
	{
		bool trobat=false;
		if( (q1!=NULL)&&(!q1->nodeBuit()) )
		{

			if(q1->formaPart(e.getPoint()))
			{
				q1->esborrarElement(&e,nivellMaxim,NULL);
				
				trobat=true;
				
				if((q1->tipus())==0)
				{
					delete q1;
					q1=NULL;
				}
			
			}
		}
		else
		{
			//cout<<"41 PASSO D'ESBORRAR PERQUE LA ZONA ES BUIDA"<<endl;
		}
		
		if ( (!trobat) &&(q2!=NULL)&&(!q2->nodeBuit()) )
		{
			if(q2->formaPart(e.getPoint()))
			{
				q2->esborrarElement(&e,nivellMaxim,NULL);
				trobat=true;
							
				if((q2->tipus())==0)
				{
					delete q2;
					q2=NULL;
				}
			}
		}
		else
		{
			//cout<<"42 PASSO D'ESBORRAR PERQUE LA ZONA ES BUIDA (o ja l'he trobat) "<<trobat<<endl;
		}

		if ( (!trobat) &&(q3!=NULL)&&(!q3->nodeBuit()) )
		{
			if(q3->formaPart(e.getPoint()))
			{
				q3->esborrarElement(&e,nivellMaxim,NULL);
				trobat=true;
							
				if((q3->tipus())==0)
				{
					delete q3;
					q3=NULL;
				}
			}
		}
		else
		{
			//cout<<"43 PASSO D'ESBORRAR PERQUE LA ZONA ES BUIDA (o ja l'he trobat) "<<trobat<<endl;
		}
		
		if ( (!trobat) &&(q4!=NULL)&&(!q4->nodeBuit()) )
		{
			if(q4->formaPart(e.getPoint()))
			{
				q4->esborrarElement(&e,nivellMaxim,NULL);
							
				if((q4->tipus())==0)
				{
					delete q4;
					q4=NULL;
				}
			}
			else
			{
				//aixo seria una excepcio
				cout<<"ERROR (CompressedCandidateZone::deleteElement) : 4 INTENTES ESBORRAR ALGU QUE NO ES A LA ZONA CANDIDATA: "<<e<<endl;
				exit(-1);		
			}
		}
		else
		{
			//cout<<"44 PASSO D'ESBORRAR PERQUE LA ZONA ES BUIDA (o ja l'he trobat) "<<trobat<<endl;
		}
	}
	else if ( (zone_type == 8))// zone_type == 8
	{
		bool trobat=false;
		if( (q1!=NULL)&&(!q1->nodeBuit()) )
		{
			if(q1->formaPart(e.getPoint()))
			{
				q1->esborrarElement(&e,nivellMaxim,NULL);
				
				trobat=true;
				
				if((q1->tipus())==0)
				{
					delete q1;
					q1=NULL;
				}
			
			}
		}
		else
		{
			//cout<<"81 PASSO D'ESBORRAR PERQUE LA ZONA ES BUIDA"<<endl;
		}
		
		if ( (!trobat) &&(q2!=NULL)&&(!q2->nodeBuit()) )
		{
			if(q2->formaPart(e.getPoint()))
			{
				q2->esborrarElement(&e,nivellMaxim,NULL);
				trobat=true;
							
				if((q2->tipus())==0)
				{
					delete q2;
					q2=NULL;
				}
			}
		}
		else
		{
			//cout<<"82 PASSO D'ESBORRAR PERQUE LA ZONA ES BUIDA (o ja l'he trobat) "<<trobat<<endl;
		}

		if ( (!trobat) &&(q3!=NULL)&&(!q3->nodeBuit()) )
		{
			if(q3->formaPart(e.getPoint()))
			{
				q3->esborrarElement(&e,nivellMaxim,NULL);
				trobat=true;
							
				if((q3->tipus())==0)
				{
					delete q3;
					q3=NULL;
				}
			}
		}
		else
		{
			//cout<<"83 PASSO D'ESBORRAR PERQUE LA ZONA ES BUIDA (o ja l'he trobat) "<<trobat<<endl;
		}
		
		if ( (!trobat) &&(q4!=NULL)&&(!q4->nodeBuit()) )
		{
			if(q4->formaPart(e.getPoint()))
			{
				q4->esborrarElement(&e,nivellMaxim,NULL);
							
				if((q4->tipus())==0)
				{
					delete q4;
					q4=NULL;
				}
			}
		}
		else
		{
			//cout<<"84 PASSO D'ESBORRAR PERQUE LA ZONA ES BUIDA (o ja l'he trobat) "<<trobat<<endl;
		}

		if ( (!trobat) &&(q5!=NULL)&&(!q5->nodeBuit()) )
		{
			if(q5->formaPart(e.getPoint()))
			{
				q5->esborrarElement(&e,nivellMaxim,NULL);
				trobat=true;
							
				if((q5->tipus())==0)
				{
					delete q5;
					q5=NULL;
				}
			}
		}
		else
		{
			//cout<<"85 PASSO D'ESBORRAR PERQUE LA ZONA ES BUIDA (o ja l'he trobat) "<<trobat<<endl;
		}

		if ( (!trobat) &&(q6!=NULL)&&(!q6->nodeBuit()) )
		{
			if(q6->formaPart(e.getPoint()))
			{
				q6->esborrarElement(&e,nivellMaxim,NULL);
				trobat=true;
							
				if((q6->tipus())==0)
				{
					delete q6;
					q6=NULL;
				}
			}
		}
		else
		{
			//cout<<"86 PASSO D'ESBORRAR PERQUE LA ZONA ES BUIDA (o ja l'he trobat) "<<trobat<<endl;
		}

		if ( (!trobat) &&(q7!=NULL)&&(!q7->nodeBuit()) )
		{
			if(q7->formaPart(e.getPoint()))
			{
				q7->esborrarElement(&e,nivellMaxim,NULL);
				trobat=true;
							
				if((q7->tipus())==0)
				{
					delete q7;
					q7=NULL;
				}
			}
		}
		else
		{
			//cout<<"87 PASSO D'ESBORRAR PERQUE LA ZONA ES BUIDA (o ja l'he trobat) "<<trobat<<endl;
		}
		
		if ( (!trobat) &&(q8!=NULL)&&(!q8->nodeBuit()) )
		{
			if(q8->formaPart(e.getPoint()))
			{
				q8->esborrarElement(&e,nivellMaxim,NULL);
							
				if((q8->tipus())==0)
				{
					delete q8;
					q8=NULL;
				}
			}
			else
			{
				//aixo seria una excepcio
				cout<<"ERROR (CompressedCandidateZone::deleteElement) : 9 INTENTES ESBORRAR ALGU QUE NO ES A LA ZONA CANDIDATA: "<<e<<endl;
				exit(-1);		
			}
		}
		else
		{
			//cout<<"88 PASSO D'ESBORRAR PERQUE LA ZONA ES BUIDA (o ja l'he trobat) "<<trobat<<endl;
		}
	}
	else
	{
			cout<<"ERROR (CompressedCandidateZone::deleteElement) : Hi ha alguna zona candidata que te el tipus malament"<<endl;
			exit(-1);
	}
}

// copy constructor
CompressedCandidateZone::CompressedCandidateZone(const CompressedCandidateZone &c) 
{
	//cout<<"papapa entro a constructor per copia"<<endl;
	
	if (this != &c) 
	{  // make sure not same object
		
		//cout<<"no soc el mateix"<<endl;
		zone_type=c.zone_type;	
		if( (zone_type==1) )
		{
			//cout<<"tipus 1"<<endl;
			if(c.q1!=NULL)
			{
				q1 = new CompressedONode(*c.q1);
			}
			else
			{
				q1=NULL;
			}
			q2=NULL;
			q3=NULL;
			q4=NULL;
			q5=NULL;
			q6=NULL;
			q7=NULL;
			q8=NULL;
		}
		else if(zone_type==2)
		{
			if( c.q1!=NULL)
			{
				q1=new CompressedONode(*c.q1);
			}
			else
			{
				q1=NULL;
			}
			
			if( c.q2!=NULL)
			{
				q2=new CompressedONode(*c.q2);
			}
			else
			{
				q2=NULL;
			}
	
			q3=NULL;
			q4=NULL;
			q5=NULL;
			q6=NULL;
			q7=NULL;
			q8=NULL;
		}
		else if(zone_type==4)
		{
			if( c.q1!=NULL)
			{
				q1=new CompressedONode(*c.q1);
			}
			else
			{
				q1=NULL;
			}
			
			if( c.q2!=NULL)
			{
				q2=new CompressedONode(*c.q2);
			}
			else
			{
				q2=NULL;
			}

			if( c.q3!=NULL)
			{
				q3=new CompressedONode(*c.q3);
			}
			else
			{
				q3=NULL;
			}
			
			if( c.q4!=NULL)
			{
				q4=new CompressedONode(*c.q4);
			}
			else
			{
				q4=NULL;
			}
			q5=NULL;
			q6=NULL;
			q7=NULL;
			q8=NULL;
		} 
		else //(zone_type == 8)
		{
			if( c.q1!=NULL)
			{
				q1=new CompressedONode(*c.q1);
			}
			else
			{
				q1=NULL;
			}
			
			if( c.q2!=NULL)
			{
				q2=new CompressedONode(*c.q2);
			}
			else
			{
				q2=NULL;
			}

			if( c.q3!=NULL)
			{
				q3=new CompressedONode(*c.q3);
			}
			else
			{
				q3=NULL;
			}
			
			if( c.q4!=NULL)
			{
				q4=new CompressedONode(*c.q4);
			}
			else
			{
				q4=NULL;
			}

			if( c.q5!=NULL)
			{
				q5=new CompressedONode(*c.q5);
			}
			else
			{
				q5=NULL;
			}
			
			if( c.q6!=NULL)
			{
				q6=new CompressedONode(*c.q6);
			}
			else
			{
				q6=NULL;
			}

			if( c.q7!=NULL)
			{
				q7=new CompressedONode(*c.q7);
			}
			else
			{
				q7=NULL;
			}
			
			if( c.q8!=NULL)
			{
				q8=new CompressedONode(*c.q8);
			}
			else
			{
				q8=NULL;
			}
		}
	}
	
	//cout<<"surto del constructor per copia"<<endl;
}


// assignement operator
//nomes volem que copii els punters!
CompressedCandidateZone& CompressedCandidateZone::operator=(const CompressedCandidateZone& c) 
{
	//cout<<"entro a operador d'assignacio "<<endl;
	
	
	if (this != &c) 
	{  // make sure not same object
		
		//cout<<"no soc el mateix"<<endl;
		zone_type=c.zone_type;	
		if( (zone_type==1) )
		{
			//cout<<"tipus 1"<<endl;
			if(c.q1!=NULL)
			{
				//q1 = new CompressedONode(*c.q1);
				q1=c.q1;
			}
			else
			{
				q1=NULL;
			}
			q2=NULL;
			q3=NULL;
			q4=NULL;
			q5=NULL;
			q6=NULL;
			q7=NULL;
			q8=NULL;
		}
		else if(zone_type==2)
		{
			if( c.q1!=NULL)
			{
				//q1=new CompressedONode(*c.q1);
				q1=c.q1;
			}
			else
			{
				q1=NULL;
			}
			
			if( c.q2!=NULL)
			{
				//q2=new CompressedONode(*c.q2);
				q2=c.q2;
			}
			else
			{
				q2=NULL;
			}
	
			q3=NULL;
			q4=NULL;
			q5=NULL;
			q6=NULL;
			q7=NULL;
			q8=NULL;
		}
		else if(zone_type==4)
		{
			if( c.q1!=NULL)
			{
//				q1=new CompressedONode(*c.q1);
				q1=c.q1;
			}
			else
			{
				q1=NULL;
			}
			
			if( c.q2!=NULL)
			{
//				q2=new CompressedONode(*c.q2);
				q2=c.q2;
			}
			else
			{
				q2=NULL;
			}

			if( c.q3!=NULL)
			{
//				q3=new CompressedONode(*c.q3);
				q3=c.q3;
			}
			else
			{
				q3=NULL;
			}
			
			if( c.q4!=NULL)
			{
//				q4=new CompressedONode(*c.q4);
				q4=c.q4;
			}
			else
			{
				q4=NULL;
			}
			q5=NULL;
			q6=NULL;
			q7=NULL;
			q8=NULL;
		} 
		else //(zone_type == 8)
		{
			if( c.q1!=NULL)
			{
//				q1=new CompressedONode(*c.q1);
				q1=c.q1;
			}
			else
			{
				q1=NULL;
			}
			
			if( c.q2!=NULL)
			{
//				q2=new CompressedONode(*c.q2);
				q2=c.q2;
			}
			else
			{
				q2=NULL;
			}

			if( c.q3!=NULL)
			{
//				q3=new CompressedONode(*c.q3);
				q3=c.q3;
			}
			else
			{
				q3=NULL;
			}
			
			if( c.q4!=NULL)
			{
//				q4=new CompressedONode(*c.q4);
				q4=c.q4;
			}
			else
			{
				q4=NULL;
			}

			if( c.q5!=NULL)
			{
//				q5=new CompressedONode(*c.q5);
				q5=c.q5;
			}
			else
			{
				q5=NULL;
			}
			
			if( c.q6!=NULL)
			{
//				q6=new CompressedONode(*c.q6);
				q6=c.q6;
			}
			else
			{
				q6=NULL;
			}

			if( c.q7!=NULL)
			{
//				q7=new CompressedONode(*c.q7);
				q7=c.q7;
			}
			else
			{
				q7=NULL;
			}
			
			if( c.q8!=NULL)
			{
//				q8=new CompressedONode(*c.q8);
				q8=c.q8;
			}
			else
			{
				q8=NULL;
			}
		}
	}

	//cout<<"surto de l'operador d'assignacio "<<endl;
	
	return *this;    // Return ref for multiple assignment
}//end operator=

int CompressedCandidateZone::type()
{
	return zone_type;
}


bool CompressedCandidateZone::buida()
{
	bool ret=false;
	//cout<<"entro a buida"<<endl;
	if(this!=NULL)
	{
	
		if (zone_type == 1)
		{
			if( (q1==NULL)||(q1->nodeBuit()) )
			{
				ret=true;			
			}
		}
		else if (zone_type == 2)
		{
			if( (q1==NULL)||(q1->nodeBuit()) )
			{
				if ( (q2==NULL)||(q2->nodeBuit()) )
				{
					ret=true;
				}	
			}
			
		}
		else if ( (zone_type == 4))// zone_type == 4
		{
			if( (q1==NULL)||(q1->nodeBuit()) )
			{
				if ( (q2==NULL)||(q2->nodeBuit()) )
				{
					if( (q3==NULL)||(q3->nodeBuit()) )
					{
						if ( (q4==NULL)||(q4->nodeBuit()) )
						{
							ret=true;
						}	
					}	
				}	
			}
		}
		else if ( (zone_type == 8))// zone_type == 8
		{
			if( (q1==NULL)||(q1->nodeBuit()) )
			{
				if ( (q2==NULL)||(q2->nodeBuit()) )
				{
					if( (q3==NULL)||(q3->nodeBuit()) )
					{
						if ( (q4==NULL)||(q4->nodeBuit()) )
						{
							if( (q5==NULL)||(q5->nodeBuit()) )
							{
								if ( (q6==NULL)||(q6->nodeBuit()) )
								{
									if( (q7==NULL)||(q7->nodeBuit()) )
									{
										if ( (q8==NULL)||(q8->nodeBuit()) )
										{
											ret=true;
										}
									}
								}
							}
						}
					}	
				}	
			}
		}
		else
		{
				cout<<"ERROR (CompressedCandidateZone.cpp::buida) : El tipus de la zona candidata no es 1,2,4 ni 8!"<<endl;
				exit(-1);
		}
	}
	else
	{
		ret=true;
	}
	//cout<<"surto de buida "<<ret<<endl;

	return ret;
}



void CompressedCandidateZone::anulaNode(CompressedONode *p)
{
	if (zone_type == 1)
	{
		if( q1==p )
		{
			q1=NULL;
		}
		else
		{
			cout<<"1 no puc anular el node com tocaria!"<<endl;
		}
	}
	else if (zone_type == 2)
	{
		if( q1==p )
		{
			q1=NULL;
		}
		else if( q2==p )
		{
			q2=NULL;
		}
		else 
		{
			cout<<"2 no puc anular el node com tocaria!"<<endl;
		}
	}
	else if (zone_type == 4)// zone_type == 4
	{
		if( q1==p )
		{
			q1=NULL;
		}
		else if( q2==p )
		{
			q2=NULL;
		}
		else if( q3==p )
		{
			q3=NULL;
		}
		else if( q4==p )
		{
			q4=NULL;
		}
		else 
		{
			cout<<"4 no puc anular el node com tocaria!"<<endl;
		}
	}
	else if (zone_type == 8)// zone_type == 8
	{
		if( q1==p )
		{
			q1=NULL;
		}
		else if( q2==p )
		{
			q2=NULL;
		}
		else if( q3==p )
		{
			q3=NULL;
		}
		else if( q4==p )
		{
			q4=NULL;
		}
		else if( q5==p )
		{
			q5=NULL;
		}
		else if( q6==p )
		{
			q6=NULL;
		}
		else if( q7==p )
		{
			q7=NULL;
		}
		else if( q8==p )
		{
			q8=NULL;
		}
		else 
		{
			cout<<"8 no puc anular el node com tocaria!"<<endl;
		}
	}
	else
	{
		cout<<"ERROR (CompressedCandidateZone.cpp::anulaNode) : El tipus de la zona candidata no es 1,2,4 ni 8!" <<endl;
		exit(-1);
	
	}
	

}

bool CompressedCandidateZone::acceptable(CompressedInformacioGeometrica info,double epsilon)
{	
	bool ret=false;
	
	// comptem el numero de no-marcats.

	if( buida()==false)
	{

		int numNoMarcats=0;
		vector<Element *> v; 
		vector<CompressedONode *> vNodes= vector<CompressedONode *>(8);	
		// primer, recollim tots els possibles nodes
		vNodes[0]=q1;		
		vNodes[1]=q2;		
		vNodes[2]=q3;		
		vNodes[3]=q4;		
		vNodes[4]=q5;		
		vNodes[5]=q6;		
		vNodes[6]=q7;		
		vNodes[7]=q8;		

		int i=0;

		// recollim tots els elements
		vector<Element > vAux = vector<Element >();
		while( i<8 )
		{
			v.clear();
			if(vNodes[i]!=NULL)
			{
				v=vNodes[i]->getLlistaElements();
				vector<Element *>::iterator it;
				it=v.begin();
				while( it!=v.end() )
				{
					if( (*it)->getMarcat() == false ) vAux.push_back( *(*it) );
					it++;
				}		
			}
			i++;				
		}
		
		//cout<<"CCZ: acceptable, Comparo "<<cardA<<" i "<<numNoMarcats<<endl;

		CompressedInformacioGeometrica infoZona=CompressedInformacioGeometrica(vAux);

		// busquem un node no nul per llençar compatible:
		i=0;
		while( i<8 )
		{
			if(vNodes[i]!=NULL)
			{
				ret=vNodes[i]->compatible(info,&infoZona,true,true,true,epsilon);
				i=8;			
			}		
			i++;				
		}

	}	
	
	
	return ret;
	
}

vector<Element *> CompressedCandidateZone::elementsZona()
{
	// EP! perd memoria?
	// potser caldria fer un clear de aux despres de insertar-lo cada cop
	 
	if (zone_type == 1)
	{
		if( q1!=NULL )
		{
			return q1->getLlistaElements();
		}
		else
		{
			return *(new vector<Element *>());
		}
	}
	else if (zone_type == 2)
	{
		vector<Element *> ret = vector<Element *> ();
		vector<Element *> aux;
		
		if( q1!=NULL )
		{
			aux = q1->getLlistaElements();
			ret.insert(ret.end(),aux.begin(),aux.end());
		}

		if( q2!=NULL )
		{
			aux = q2->getLlistaElements();
			ret.insert(ret.end(),aux.begin(),aux.end());
		}
		
		return ret;
	
	}
	else if ( (zone_type == 4))// zone_type == 4
	{
		vector<Element *> ret = vector<Element *> ();
		vector<Element *> aux;
		
		if( q1!=NULL )
		{
			aux = q1->getLlistaElements();
			ret.insert(ret.end(),aux.begin(),aux.end());
		}

		if( q2!=NULL )
		{
			aux = q2->getLlistaElements();
			ret.insert(ret.end(),aux.begin(),aux.end());
		}

		if( q3!=NULL )
		{
			aux = q3->getLlistaElements();
			ret.insert(ret.end(),aux.begin(),aux.end());
		}

		if( q4!=NULL )
		{
			aux = q4->getLlistaElements();
			ret.insert(ret.end(),aux.begin(),aux.end());
		}

		return ret;
	}
	else if ( (zone_type == 8))// zone_type == 8
	{
		vector<Element *> ret = vector<Element *> ();
		vector<Element *> aux;
		
		if( q1!=NULL )
		{
			aux = q1->getLlistaElements();
			ret.insert(ret.end(),aux.begin(),aux.end());
		}

		if( q2!=NULL )
		{
			aux = q2->getLlistaElements();
			ret.insert(ret.end(),aux.begin(),aux.end());
		}

		if( q3!=NULL )
		{
			aux = q3->getLlistaElements();
			ret.insert(ret.end(),aux.begin(),aux.end());
		}

		if( q4!=NULL )
		{
			aux = q4->getLlistaElements();
			ret.insert(ret.end(),aux.begin(),aux.end());
		}

		if( q5!=NULL )
		{
			aux = q5->getLlistaElements();
			ret.insert(ret.end(),aux.begin(),aux.end());
		}

		if( q6!=NULL )
		{
			aux = q6->getLlistaElements();
			ret.insert(ret.end(),aux.begin(),aux.end());
		}

		if( q7!=NULL )
		{
			aux = q7->getLlistaElements();
			ret.insert(ret.end(),aux.begin(),aux.end());
		}

		if( q8!=NULL )
		{
			aux = q8->getLlistaElements();
			ret.insert(ret.end(),aux.begin(),aux.end());
		}

		return ret;
	}
	else
	{
		cout<<"ERROR (CompressedCandidateZone.cpp::elementsZona) : El tipus de la zona candidata no es 1,2,4 ni 8!" <<endl;
		exit(-1);
	
	}

}

void CompressedCandidateZone::marcarElement(Element *e)
{
	bool b1=false, b2=false, b3=false, b4=false, b5=false, b6=false, b7=false, b8=false;

 	if (q1!=NULL)
	{
		b1=q1->formaPart(e->getPoint());
		try
		{
			if(b1) q1->marcarElement(e);
		}
		catch(int i){}// de moment aqui no fem res amb la excepcio	
	}
	if (!b1&&(q2!=NULL))
	{
		b2=q2->formaPart(e->getPoint());
		try
		{
			if(b2) q2->marcarElement(e);
		}
		catch(int i){}// de moment aqui no fem res amb la excepcio	
	}
	if (!b1&&!b2&&(q3!=NULL))
	{
		b3=q3->formaPart(e->getPoint());
		try
		{
			if(b3) q3->marcarElement(e);
		}
		catch(int i){}// de moment aqui no fem res amb la excepcio	
	}
	if (!b1&&!b2&&!b3&&(q4!=NULL))
	{
		b4=q4->formaPart(e->getPoint());
		try
		{
			if(b4) q4->marcarElement(e);
		}
		catch(int i){}// de moment aqui no fem res amb la excepcio	
	}
	if (!b1&&!b2&&!b3&&!b4&&(q5!=NULL))
	{
		b5=q5->formaPart(e->getPoint());
		try
		{
			if(b5) q5->marcarElement(e);
		}
		catch(int i){}// de moment aqui no fem res amb la excepcio	

	}
	if (!b1&&!b2&&!b3&&!b4&&!b5&&(q6!=NULL))
	{
		b6=q6->formaPart(e->getPoint());
		try
		{
			if(b6) q6->marcarElement(e);
		}
		catch(int i){}// de moment aqui no fem res amb la excepcio	
	}
	if (!b1&&!b2&&!b3&&!b4&&!b5&&!b6&&(q7!=NULL))
	{
		b7=q7->formaPart(e->getPoint());
		try
		{
			if(b7) q7->marcarElement(e);
		}
		catch(int i){}// de moment aqui no fem res amb la excepcio	
	}
	if (!b1&&!b2&&!b3&&!b4&&!b5&&!b6&&!b7&&(q8!=NULL))
	{
		b8=q8->formaPart(e->getPoint());
		try
		{
			if(b8) q8->marcarElement(e);
		}
		catch(int i){}// de moment aqui no fem res amb la excepcio	
	}

	if (!(b1||b2||b3||b4||b5||b6||b7||b8))
	{
		//cout<<"ERROR (CompressedCandidateZone.cpp::marcarElement) : Marcar element de zona candidata, no hi era!"<<endl;
		//cout<<"CCZ: suposem error d'arrodoniment, busquem a pinyó"<<endl;
		
		bool solucionat=false;
		vector<Element *> v; 
		vector<CompressedONode *> vNodes= vector<CompressedONode *>(8);	
		// primer, recollim tots els possibles nodes
		vNodes[0]=q1;		
		vNodes[1]=q2;		
		vNodes[2]=q3;		
		vNodes[3]=q4;		
		vNodes[4]=q5;		
		vNodes[5]=q6;		
		vNodes[6]=q7;		
		vNodes[7]=q8;		

		int i=0;
		while( (!solucionat)&&(i<8))
		{
			v.clear();
			if(vNodes[i]!=NULL)
			{
				v=vNodes[i]->getLlistaElements();
				vector<Element *>::iterator it;
				it=v.begin();
				while( !solucionat && (it!=v.end()) )
				{
					if( (*(*it)) == (*e) ) solucionat=true;
					it++;
				}		
			}
			i++;				
		}
		
		if(!solucionat)
		{	
			cout<<"CCZ: No ho hem pas pogut arreglar!"<<endl;	
			exit(-1);	
		}
	}
}
