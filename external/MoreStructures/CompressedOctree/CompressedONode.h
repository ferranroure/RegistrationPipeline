#ifndef _COMPRESSED_ONODE_
#define _COMPRESSED_ONODE_

#include <vector>
#include <list>
#include <iostream>

#include "../AuxiliaryClasses/Element.h"
#include "./CompressedInformacioGeometrica.h"
//#include "./CompressedCandidateZone.h"

#define INSIDE 1
#define OUTSIDE 0
#define INTERSECT 2
#define CONTAINS 3

class CompressedCandidateZone;

class CompressedONode
{
	private:
		point3D ancoratge; //Punt d'ancoratge del cub al que es refereix el node
		long double mida; //Mida del costat del cub al que es refereix el node
		int nivell; //Nivell que es troba el node
		vector<Element *> llistaElements; //Llista d'esferes que conte el node (tant si es fulla com si no)
	
		CompressedInformacioGeometrica *infGeo; //Informacio geometrica del node	
	
		CompressedONode *f1,*f2,*f3,*f4,*f5,*f6,*f7,*f8; //Els 8 fills

		vector<CompressedCandidateZone *> nodeZones; //List of candidate zones rooted at this CompressedONode
		
		int afegirElementFillCorresponent(Element *,int); //Metode que calcula el fill que s'ha de posar l'esfera i crida el metode "afegirElement" amb el fill calculat. L'enter es el nivell maxim que pot tenir un node. Retorna el nivell que insereix l'esfera
		
	public:
		CompressedONode(); //Constructor per defecte
		CompressedONode(point3D,double,int); //Constructor per un node "fulla blanca" (ancoratge,mida,nivell)
		CompressedONode(const CompressedONode &n); //Constructor copia
		~CompressedONode(); //Destructor
		
		CompressedONode& operator=(const CompressedONode &n);

		point3D getAncoratge();
		double getMida();
		int getNivell();
		CompressedONode* getFill1();
		CompressedONode* getFill2();
		CompressedONode* getFill3();
		CompressedONode* getFill4();
		CompressedONode* getFill5();
		CompressedONode* getFill6();
		CompressedONode* getFill7();
		CompressedONode* getFill8();
		vector<Element *> getLlistaElements();
		CompressedInformacioGeometrica* getInfGeo();
		void setInfGeo(CompressedInformacioGeometrica*);

		void setAncoratge(point3D);
		void setMida(double);
		void setNivell(int);
		void setLlistaElements(vector<Element *>);
		void setFill1(CompressedONode *);
		void setFill2(CompressedONode *);
		void setFill3(CompressedONode *);
		void setFill4(CompressedONode *);
		void setFill5(CompressedONode *);
		void setFill6(CompressedONode *);
		void setFill7(CompressedONode *);
		void setFill8(CompressedONode *);
		void setNodeZones(CompressedCandidateZone * cand); //Enllaça el CompressedONode que toca amb la zona que toca
		vector<CompressedCandidateZone *> getNodeZones(){return nodeZones;}

		void copiarAtributs(CompressedONode *);

		int afegirElement(Element *,int, CompressedONode *); //Metode per afegir una esfera al node passant com a parametre l'esfera, el nivell maxim que pot tenir un node i el pare. Retorna el nivell que ha posat l'esfera
		void esborrarElement(Element *,int,CompressedONode *); //Metode per esborrar un element del compressed octree
		
		int tipus(); //Retorna quin es el seu tipus de node (fulla blanca, negre o node parcial).
		bool esFulla(); //Retorna cert si el node no te fills
		bool nodeBuit(); //Retorna cert si es fulla blanca
		int crearFills(int); //Metode que crea els fills d'un node a partir dels 2 elements que hi ha.
		bool pertany(Element *e); 
		vector<bool> aQuinsFills(Element *e);
		int aQuinFill(Element *e); //Retorna el numero de fill que pertany l'element e.
		int aQuinFill(CompressedONode *n); //Retorna el numero de fill que pertany el node n.
		int fillNoFullaBlanca(); //Retorna el numero de fill que no es fulla blanca (en cas que nomes n'hi hagi un)
		bool formaPart(point3D p); //Retorna cert si el punt forma part del node, fals altrament.
		void marcarElement(Element *e); //Metode que busca un element i li canvia l'atribut "marcat"

		void actualitzarInfGeo(); //Metode per actualitzar la informacio geometrica

		list<Element*> * weightedNeighbors(Element *e,double epsilon); //Retorna la llista d'elements que formen part de la circumferencia "e-epsilon"
		bool contained(Element *e,double r); //Retorna cert si el node forma part de la circumferencia zona "e-r", altrament fals
		bool stabbingSimple(Element *e, double r);
		bool stabbing(Element *e,double r); //Retorna cert si alguna part del node forma part de la circumferencia "e-r", altrament fals
		int checkIntersection(double p_xmin, double p_xmax, double p_ymin, double p_ymax, double p_zmin, double p_zmax);

		list<Element*> * report(Element *e); //Report all Elements in the node matching the parameters radius
		list<Element*> *  reportIf(Element *e,double r); //Report all Elements in the node matching the parameters radius and distance requirement


		static bool compatible(CompressedInformacioGeometrica,CompressedInformacioGeometrica*,bool bNum,bool bHisto,bool bDist,double eps); //Operation for atribute compatibility
		static bool compatible_num_elements(CompressedInformacioGeometrica,CompressedInformacioGeometrica*); //First attribute: number of elements
		static bool compatible_weights(CompressedInformacioGeometrica,CompressedInformacioGeometrica*); //Second attribute: weights
		static bool compatible_distances(CompressedInformacioGeometrica,CompressedInformacioGeometrica*,double eps); //Third attribute: distances

		void pintar(); //Metode per pintar 2 plans al node (si te fills)
		void write(); //Metode que escriu per la terminal l'octree en POSTORDRE (arrel,f1,f2,f3,...)
		void write2(ostream& os); //Metode per write a traves d'un ostream
};

#endif
