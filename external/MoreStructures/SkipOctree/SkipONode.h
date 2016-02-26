//
// Created by yago on 16/02/26.
//

#ifndef _SKIP_ONODE_
#define _SKIP_ONODE_

#include <vector>
#include <list>
#include <iostream>

#include "../AuxiliaryClasses/Element.h"

#define INSIDE 1
#define OUTSIDE 0
#define INTERSECT 2
#define CONTAINS 3

class SkipONode
{
private:
    point3D ancoratge; //Punt d'ancoratge del cub al que es refereix el node
    long double mida; //Mida del costat del cub al que es refereix el node
    int nivell; //Nivell que es troba el node
    vector<Element *> llistaElements; //Llista d'esferes que conte el node (tant si es fulla com si no)

    SkipONode *f1,*f2,*f3,*f4,*f5,*f6,*f7,*f8; //Els 8 fills

    int afegirElementFillCorresponent(Element *,int); //Metode que calcula el fill que s'ha de posar l'esfera i crida el metode "afegirElement" amb el fill calculat. L'enter es el nivell maxim que pot tenir un node. Retorna el nivell que insereix l'esfera

public:
    SkipONode(); //Constructor per defecte
    SkipONode(point3D,double,int); //Constructor per un node "fulla blanca" (ancoratge,mida,nivell)
    SkipONode(const SkipONode &n); //Constructor copia
    ~SkipONode(); //Destructor

    SkipONode& operator=(const SkipONode &n);

    point3D getAncoratge();
    double getMida();
    int getNivell();
    SkipONode* getFill1();
    SkipONode* getFill2();
    SkipONode* getFill3();
    SkipONode* getFill4();
    SkipONode* getFill5();
    SkipONode* getFill6();
    SkipONode* getFill7();
    SkipONode* getFill8();
    vector<Element *> getLlistaElements();

    void setAncoratge(point3D);
    void setMida(double);
    void setNivell(int);
    void setLlistaElements(vector<Element *>);
    void setFill1(SkipONode *);
    void setFill2(SkipONode *);
    void setFill3(SkipONode *);
    void setFill4(SkipONode *);
    void setFill5(SkipONode *);
    void setFill6(SkipONode *);
    void setFill7(SkipONode *);
    void setFill8(SkipONode *);

    void copiarAtributs(SkipONode *);

    int afegirElement(Element *,int, SkipONode *); //Metode per afegir una esfera al node passant com a parametre l'esfera, el nivell maxim que pot tenir un node i el pare. Retorna el nivell que ha posat l'esfera
    void esborrarElement(Element *,int,SkipONode *); //Metode per esborrar un element del Skip octree

    int tipus(); //Retorna quin es el seu tipus de node (fulla blanca, negre o node parcial).
    bool esFulla(); //Retorna cert si el node no te fills
    bool nodeBuit(); //Retorna cert si es fulla blanca
    int crearFills(int); //Metode que crea els fills d'un node a partir dels 2 elements que hi ha.
    bool pertany(Element *e);
    vector<bool> aQuinsFills(Element *e);
    int aQuinFill(Element *e); //Retorna el numero de fill que pertany l'element e.
    int aQuinFill(SkipONode *n); //Retorna el numero de fill que pertany el node n.
    int fillNoFullaBlanca(); //Retorna el numero de fill que no es fulla blanca (en cas que nomes n'hi hagi un)
    bool formaPart(point3D p); //Retorna cert si el punt forma part del node, fals altrament.
    void marcarElement(Element *e); //Metode que busca un element i li canvia l'atribut "marcat"

    list<Element*> * weightedNeighbors(Element *e,double epsilon); //Retorna la llista d'elements que formen part de la circumferencia "e-epsilon"
    bool contained(Element *e,double r); //Retorna cert si el node forma part de la circumferencia zona "e-r", altrament fals
    bool stabbingSimple(Element *e, double r);
    bool stabbing(Element *e,double r); //Retorna cert si alguna part del node forma part de la circumferencia "e-r", altrament fals
    int checkIntersection(double p_xmin, double p_xmax, double p_ymin, double p_ymax, double p_zmin, double p_zmax);

    list<Element*> * report(Element *e); //Report all Elements in the node matching the parameters radius
    list<Element*> *  reportIf(Element *e,double r); //Report all Elements in the node matching the parameters radius and distance requirement

    void write(); //Metode que escriu per la terminal l'octree en POSTORDRE (arrel,f1,f2,f3,...)
    void write2(ostream& os); //Metode per write a traves d'un ostream
};

#endif