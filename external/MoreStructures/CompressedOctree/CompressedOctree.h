#ifndef _COMPRESSED_OCTREE_
#define _COMPRESSED_OCTREE_

#include <vector>
#include <list>
#include <iostream>
#include <math.h>

#include "./CompressedONode.h"
#include "./CompressedCandidateZone.h"
#include "../AuxiliaryClasses/point3D.h"

class CompressedOctree
{
	private:
		point3D ancoratge; //Punt d'ancoratge
		double mida; //Mida del costat del cub
		int nivellActual; //Nivell de profunditat actual de l'CompressedOctree (l'arrel te nivell 0)
		int nivellMaxim; //Nivell de profunditat maxim que pot tenir l'octree
		double epsilon;
		
		vector<Element *> llistaElements; //Llista d'esferes de l'octree
		
		CompressedONode *arrel; //CompressedONode arrel de l'octree

		vector<CompressedCandidateZone *> candidates; //Vector for the candidate zones
		
		void calcularCub(); //Metode que calcula el punt d'ancoratge i la mida del cub que forma l'octree

		// Functions to compare information
		void intern_search(CompressedInformacioGeometrica info, bool bNum, bool bHisto, bool bDist);
		void intern_search_1(CompressedInformacioGeometrica info, CompressedONode *  t, bool bNum, bool bHisto, bool bDist);   
		void intern_search_2x(CompressedInformacioGeometrica info, CompressedONode *  t1, CompressedONode *  t2, bool bNum, bool bHisto, bool bDist);
		void intern_search_2y(CompressedInformacioGeometrica info, CompressedONode *  t1, CompressedONode *  t2, bool bNum, bool bHisto, bool bDist);
		void intern_search_2z(CompressedInformacioGeometrica info, CompressedONode *  t1, CompressedONode *  t2, bool bNum, bool bHisto, bool bDist);
		void intern_search_4xy(CompressedInformacioGeometrica info, CompressedONode *  t1, CompressedONode *  t2,CompressedONode *  t3, CompressedONode *  t4, bool bNum, bool bHisto, bool bDist);
		void intern_search_4yz(CompressedInformacioGeometrica info, CompressedONode *  t1, CompressedONode *  t2,CompressedONode *  t3, CompressedONode *  t4, bool bNum, bool bHisto, bool bDist);
		void intern_search_4xz(CompressedInformacioGeometrica info, CompressedONode *  t1, CompressedONode *  t2,CompressedONode *  t3, CompressedONode *  t4, bool bNum, bool bHisto, bool bDist);
		void intern_search_8(CompressedInformacioGeometrica info, CompressedONode *  t1, CompressedONode *  t2,CompressedONode *  t3, CompressedONode *  t4,  CompressedONode *  t5, CompressedONode *  t6,CompressedONode *  t7, CompressedONode *  t8, bool bNum, bool bHisto, bool bDist);
		void call_intern_search_4(CompressedInformacioGeometrica info, bool ok1, bool ok2, bool ok3, bool ok4,CompressedONode *t1,CompressedONode *t2,CompressedONode *t3,CompressedONode *t4, bool bNum, bool bHisto, bool bDist, int tipus);
		void call_intern_search_8(CompressedInformacioGeometrica info, bool ok1, bool ok2, bool ok3, bool ok4, bool ok5, bool ok6, bool ok7, bool ok8,CompressedONode *t1,CompressedONode *t2,CompressedONode *t3,CompressedONode *t4, CompressedONode *t5,CompressedONode *t6,CompressedONode *t7,CompressedONode *t8, bool bNum, bool bHisto, bool bDist);
		bool esToquen2(CompressedInformacioGeometrica info, CompressedONode *t1, CompressedONode *t2);
		bool deCostat2(CompressedONode * t1, CompressedONode * t2, int tipus); //Tipus -> 1:x, 2:y, 3:z
		int quiEscombra2(CompressedONode * t1, CompressedONode * t2, int tipus); //Tipus -> 1:x, 2:y, 3:z; Retorna el fill que escombra (si ningu escombra, retorna -1)
		
	public:
		CompressedOctree(); //Constructor per defecte
		CompressedOctree(double eps);
		CompressedOctree(vector<Element *>,double eps); //Constructor amb pas de parametres
		~CompressedOctree(); //Destructor
		
		point3D getAncoratge(); //Metode que retorna el punt d'ancoratge de l'octree
		double getMida(); //Metode que retorna la mida del costat del cub de que forma l'octree
		int getNivellActual(); //Metode que retorna el nivell de profunditat actual
		int getNivellMaxim(); //Metode que retorna el nivell de profunditat maxim
		void setNivellMaxim(int); //Metode que introdueix el nivell maxim
		
		void afegirElement(Element *); //Metode que posa una esfera a l'octree, retorna el nivell de l'octree
		void esborrarElement(Element *); //Metode que treu l'esfera de l'octree, retorna el nivell de l'octree
		void marcarElement(Element *); //Metode que busca un element i li canvia l'atribut "marcat"
		
		void actualitzarInfGeo(); //Serveix per actualitzar la informacio geometrica

		list<Element> weightedNeighbors(Element *e,double eps); //Weighted Neighbors

		CompressedInformacioGeometrica* calcularInfo2(CompressedInformacioGeometrica *info1, CompressedInformacioGeometrica *info2); //Sumar 2 info geometriques (força bruta)
		CompressedInformacioGeometrica* calcularInfo4(CompressedInformacioGeometrica *info1, CompressedInformacioGeometrica *info2, CompressedInformacioGeometrica *info3, CompressedInformacioGeometrica *info4); //Sumar 4 info geometriques (força bruta)
		CompressedInformacioGeometrica* calcularInfo8(CompressedInformacioGeometrica *info1, CompressedInformacioGeometrica *info2, CompressedInformacioGeometrica *info3, CompressedInformacioGeometrica *info4, CompressedInformacioGeometrica *info5, CompressedInformacioGeometrica *info6, CompressedInformacioGeometrica *info7, CompressedInformacioGeometrica *info8); //Sumar 8 info geometriques (força bruta)
		void search(CompressedInformacioGeometrica info, bool bNum, bool bHisto, bool bDist); //Search
	
		void pintar(); //Metode per pintar l'octree
		void write(); //Metode que escriu per la terminal l'octree en POSTORDRE (arrel,f1,f2,f3,...) 

		//metode inline per retornar l'i-essim element d'un CompressedOctree.
		Element getIthElement(int i){return *llistaElements[i];}

		//metode inline per retornar el nombre d'elements d'un CompressedOctree.
		int cardinal(){return llistaElements.size();}
		
		//metode inline per retornar una zona candidata que conte tot el compressedOctree
		CompressedCandidateZone* returnCandidateZone(){return new CompressedCandidateZone(1,arrel);}

		//metode inline per recollir les zones candidates
		vector<CompressedCandidateZone *> getCandidates(){return candidates;}

		// metode per netejar les zones candidates si es volen fer mes cerques
		void clearCandidates() {candidates.clear();}

		// metode auxiliar per simplificar les cerques, descomprimeix els fills del node t i ho empalma a nAux
		vector<bool> descomprimeix(CompressedONode *t,CompressedONode *nAux);
		// aquest altre ho torna a comprimir			
		void recomprimeix(vector<bool> fillsTocats, CompressedONode * nAux);

		vector<Element *> getLlistaElements(){return llistaElements;}

};

#endif
