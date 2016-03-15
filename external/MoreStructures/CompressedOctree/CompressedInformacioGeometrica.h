#ifndef _COMPRESSED_INFORMACIO_GEOMETRICA_
#define _COMPRESSED_INFORMACIO_GEOMETRICA_

#include <vector>
#include <iostream>
#include <cstdlib>

#include "../AuxiliaryClasses/Element.h"

//tupla per la llista de radis
struct  Interval {
	vector<Element *> llistaElements; //Llista d'elements
	int numElements; //Numero de radis que tenen el valor 'r'
	double distMax; //Distancia maxima entre dos elements
	double distMin; //Distancia minima entre dos elements
	
	
	Interval() 
	{
		numElements = 0;
		distMax = -1;
		distMin = -1;
	}
	
	Interval(const Interval &i)
	{
		llistaElements.insert(llistaElements.end(),i.llistaElements.begin(),i.llistaElements.end());
		numElements = i.numElements;
		distMax = i.distMax;
		distMin = i.distMin;
	}

	~Interval() {}
	
	int getNumElements() 
	{
		return numElements;
	}
	
	double getDistMax() 
	{
		return distMax;
	}
	
	double getDistMin() 
	{
		return distMin;
	}

	vector<Element *> getLlistaElements() 
	{
		return llistaElements;
	}
		
	void afegir(Element *e) 
	{
		//Actualitzem distancia minima i distancia maxima
		double distActual;
		for(int i=0; i<(int)llistaElements.size(); i++) {
			distActual = llistaElements[i]->getPoint().dist(e->getPoint());
			if ( (distActual < distMin) || (distMin == -1) ) distMin = distActual;
			if ( distActual > distMax ) distMax = distActual;
		}
		
		//Actualitzem numero d'elements
		numElements++;
		llistaElements.push_back(e);
	}

	void afegir(Interval i)
	{
		//Afegim els elements de l'interval "i" a l'interval actual
		vector<Element *>::iterator it;
		for (it=i.llistaElements.begin(); it!=i.llistaElements.end(); it++) {
			afegir(*it);
		}
	}

	void inicialitzar() 
	{
		llistaElements.clear();
		numElements = 0;
		distMax = -1;
		distMin = -1;
	}
};

class CompressedInformacioGeometrica
{
	private:
 		int numElements; //Numero d'esferes
		double midaNode; //Mida del node que conte la IG.
 		vector<Interval> histograma; //Llista de radis

	public:
		CompressedInformacioGeometrica(); //Constructor per defecte
		CompressedInformacioGeometrica(double,vector<Element *>); //Constructor amb pas de parametres
		CompressedInformacioGeometrica(vector<Element>);//constructor per als targetSets		
		CompressedInformacioGeometrica(const CompressedInformacioGeometrica &info); //Constructor copia
		~CompressedInformacioGeometrica(); //Destructor

		int getNumElements();
		double getMidaNode();
		vector<Interval> getHistograma();
		vector<int> getNumElementsHistograma();
		vector<double> getDistMinHistograma();
		vector<double> getDistMaxHistograma();

		void afegir(CompressedInformacioGeometrica *info); //Metode per fusionar 2 informacions geometriques

		void actualitzar(vector<Element *>); //Metode per actualitzar la informacio geometrica
 		
		bool buida();

 		void escriure();
        	friend ostream& operator<<(ostream& os,CompressedInformacioGeometrica r) 
                {
                        if(!r.buida()) {
                                os << "Cig Ostream: numEsferes: " << r.numElements<<" mida: "<<r.midaNode<<endl << "radis:              (";
                                for (int i=0; i<(int)(r.histograma.size()-1); i++) {
                                os << r.histograma[i].getNumElements() << ",";
                                }
                                os << r.histograma[r.histograma.size()-1].getNumElements() << ")" << endl;

                                os << "Distancies minimes: (";
                                for (int i=0; i<(int)(r.histograma.size()-1); i++) {
                                os << r.histograma[i].getDistMin() << ",";
                                }
                                os << r.histograma[r.histograma.size()-1].getDistMin() << ")" << endl;

                                os << "Distancies maximes: (";
                                for (int i=0; i<(int)(r.histograma.size()-1); i++) {
                                os << r.histograma[i].getDistMax() << ",";
                                }
                                os << r.histograma[r.histograma.size()-1].getDistMax() << ")" << endl;

				// no entenc perque esta separat lultim tros, canviar? FALTA

                        } else {
                                os<<"Rep Info buida"<<endl;
                        }
                        return os;
                }

		// precondicio! i>0!
		Element * getHistogramsIthPosition(int i) //inline!
		{
			//considerar la possibilitat de muntar una segona estructura que sigui un vector.
			
			// sha de recorrer tot el vector d'intervals, anar saltant els que hi hagi a cada interval i finalment retornar el que toqui, que probablement sera un element.
	
			if (i<numElements)
			{
				cout<<"CompInfGeom::getHistogramsIthPosition, Funcio no testada!"<<endl;
				vector<Interval>::iterator it=histograma.begin();
				while(it!=histograma.end())
				{
					//si estem a l'interval que toca retornem, sino saltem
					if(i< ((int)((*it).llistaElements).size()) ) 
					{
						if(i>=0)return ((*it).llistaElements)[i];
						else 				cout<<"Error CompInfGeom::getHistogramsIthPosition, t'has perdut!"<<endl;
					}
					else
					{
						i=i-((*it).llistaElements).size();
					}
					it++;
				}
				
				
			}
			else
			{
				cout<<"Error CompInfGeom::getHistogramsIthPosition, no hi ha tants elements!"<<endl;
				exit(-1);	
			}	
		}

		void setHistogramsIthPosition(int i, Element* e)
		{
			//per treure els warnings!
			i=i;
			e=e;

			cout<<"CompInfGeom::setHistogramsIthPosition, crec que aquesta funcio ja no cal, de moment nomes talla l'execució!"<<endl;
			exit(-1);
			// si s'hagues de refer nomes cal tenir en compte el que diu l'anterior!
		}		
};

#endif
