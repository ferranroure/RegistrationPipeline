#include "CompressedInformacioGeometrica.h"

#include <iostream>

#define MAX_RADI 20

CompressedInformacioGeometrica::CompressedInformacioGeometrica()
{
	midaNode = 0;
 	numElements = 0;
	
	//Omplim l'histograma:  [0,1),[1,2),[2,3),..
	for (int i=0; i<MAX_RADI; i++) {
		histograma.push_back( Interval() );
	}
}

CompressedInformacioGeometrica::CompressedInformacioGeometrica(double mida, vector<Element *> llistaElements)
{
	midaNode = mida;
	numElements = llistaElements.size();
	
	//Omplim l'histograma:  [0,1),[1,2),[2,3),..
	for (int i=0; i<MAX_RADI; i++) {
		histograma.push_back( Interval() );
	}
	histograma.push_back( Interval() ); //Per si algun interval es superior a 100

	//Calculem la distancia minima i maxima entre 2 esferes i emplenem la llista de radis
	for(int i=0; i<(int)llistaElements.size(); i++) {
		//Incrementem el valor a la llista de radis
		int numInterval = (int)(llistaElements[i]->getRadi());
		if (numInterval>MAX_RADI) numInterval = MAX_RADI;
		histograma[numInterval].afegir(llistaElements[i]);
	}
}

CompressedInformacioGeometrica::CompressedInformacioGeometrica(vector<Element> llistaElements)
{
	// ep! calcula la mida!!
	double distancia=-1;
	for(unsigned int i=0;i<llistaElements.size();i++)
	{
		for(unsigned int j=i+1;j<llistaElements.size();j++)
		{
			if( (llistaElements[i].getPoint()).dist(llistaElements[j].getPoint()) > distancia)
			{
				distancia= (llistaElements[i].getPoint()).dist(llistaElements[j].getPoint());
				//cout<<"miro distancia: "<<llistaElements[i].getPoint()<<" "<<llistaElements[j].getPoint()<<"---->"<<distancia<<endl;
			}
		}
	}

	midaNode = distancia;

	numElements = llistaElements.size();
	
	//Omplim l'histograma:  [0,1),[1,2),[2,3),..
	for (int i=0; i<MAX_RADI; i++) {
		histograma.push_back( Interval() );
	}
	histograma.push_back( Interval() ); //Per si algun interval es superior a 100

	//Calculem la distancia minima i maxima entre 2 esferes i emplenem la llista de radis
	for(int i=0; i<(int)llistaElements.size(); i++) {
		//Incrementem el valor a la llista de radis
		int numInterval = (int)(llistaElements[i].getRadi());
		if (numInterval>MAX_RADI) numInterval = MAX_RADI;
		// oju memoria!
		histograma[numInterval].afegir(new Element(llistaElements[i]));
	}
}



CompressedInformacioGeometrica::CompressedInformacioGeometrica(const CompressedInformacioGeometrica &info)
{
	midaNode = info.midaNode;
	numElements = info.numElements;

	//Omplim l'histograma:  [0,1),[1,2),[2,3),..
	for (int i=0;i<info.histograma.size(); i++) {
		histograma.push_back( Interval(info.histograma[i]) );
	}
}

CompressedInformacioGeometrica::~CompressedInformacioGeometrica() {}

int CompressedInformacioGeometrica::getNumElements()
{
	return numElements;
}

double CompressedInformacioGeometrica::getMidaNode()
{
	return midaNode;
}

vector<Interval> CompressedInformacioGeometrica::getHistograma()
{
	return histograma;
}

vector<int> CompressedInformacioGeometrica::getNumElementsHistograma()
{
	vector<int> resultat = vector<int>();

	vector<Interval>::iterator i;
	for (i=histograma.begin(); i!=histograma.end(); i++) {
		resultat.push_back( (*i).getNumElements() );
	}

	return resultat;
}

vector<double> CompressedInformacioGeometrica::getDistMinHistograma()
{
	vector<double> resultat = vector<double>();

	vector<Interval>::iterator i;
	for (i=histograma.begin(); i!=histograma.end(); i++) {
		resultat.push_back( (*i).getDistMin() );
	}

	return resultat;
}

vector<double> CompressedInformacioGeometrica::getDistMaxHistograma()
{
	vector<double> resultat = vector<double>();

	vector<Interval>::iterator i;
	for (i=histograma.begin(); i!=histograma.end(); i++) {
		resultat.push_back( (*i).getDistMax() );
	}

	return resultat;
}

void CompressedInformacioGeometrica::afegir(CompressedInformacioGeometrica *info)
{
	numElements+=info->numElements;
	if (midaNode<info->midaNode) midaNode = info->midaNode;
	
	//Afegim els elements de "info" a l'histograma
	vector<Interval>::iterator i1,i2;
	i2=info->histograma.begin();
	for (i1=histograma.begin(); i1!=histograma.end(); i1++) {
		(*i1).afegir(*i2);
		i2++;
	}
}

void CompressedInformacioGeometrica::actualitzar(vector<Element *> llistaElements)
{
	numElements = llistaElements.size();
	
	//Resetegem l'histograma
	for(int i=0; i<(int)histograma.size(); i++) {
		histograma[i].inicialitzar();
	}

	//Calculem la distancia minima i maxima entre 2 esferes i emplenem la llista de radis
	for(int i=0; i<(int)llistaElements.size(); i++) {
		//Incrementem el valor a la llista de radis
		int numInterval = (int)(llistaElements[i]->getRadi());
		if (numInterval>MAX_RADI) numInterval = MAX_RADI;
		histograma[numInterval].afegir(llistaElements[i]);
	}
}

bool CompressedInformacioGeometrica::buida()
{
        return (numElements==0);
}

void CompressedInformacioGeometrica::escriure()
{
	cout << "numEsferes: " << numElements << " radis: (";
	for (int i=0; i<(int)(histograma.size()-1); i++) {
		cout << histograma[i].getNumElements() << ",";
	}
	cout << histograma[histograma.size()-1].getNumElements() << ")" << endl;
}
