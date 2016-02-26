//
// Created by yago on 16/02/26.
//

#ifndef _SKIP_OCTREE_
#define _SKIP_OCTREE_

#include <vector>
#include <list>
#include <iostream>
#include <math.h>

#include "./SkipONode.h"
#include "../AuxiliaryClasses/point3D.h"

class SkipOctree
{
private:
    point3D ancoratge; //Punt d'ancoratge
    double mida; //Mida del costat del cub
    int nivellActual; //Nivell de profunditat actual de l'SkipOctree (l'arrel te nivell 0)
    int nivellMaxim; //Nivell de profunditat maxim que pot tenir l'octree
    double epsilon;

    vector<Element *> llistaElements; //Llista d'esferes de l'octree

    SkipONode *arrel; //SkipONode arrel de l'octree

    void calcularCub(); //Metode que calcula el punt d'ancoratge i la mida del cub que forma l'octree

    bool deCostat2(SkipONode * t1, SkipONode * t2, int wtipus); //Tipus -> 1:x, 2:y, 3:z
    int quiEscombra2(SkipONode * t1, SkipONode * t2, int tipus); //Tipus -> 1:x, 2:y, 3:z; Retorna el fill que escombra (si ningu escombra, retorna -1)

public:
    SkipOctree(); //Constructor per defecte
    SkipOctree(double eps);
    SkipOctree(vector<Element *>,double eps); //Constructor amb pas de parametres
    ~SkipOctree(); //Destructor

    point3D getAncoratge(); //Metode que retorna el punt d'ancoratge de l'octree
    double getMida(); //Metode que retorna la mida del costat del cub de que forma l'octree
    int getNivellActual(); //Metode que retorna el nivell de profunditat actual
    int getNivellMaxim(); //Metode que retorna el nivell de profunditat maxim
    void setNivellMaxim(int); //Metode que introdueix el nivell maxim

    void afegirElement(Element *); //Metode que posa una esfera a l'octree, retorna el nivell de l'octree
    void esborrarElement(Element *); //Metode que treu l'esfera de l'octree, retorna el nivell de l'octree

    list<Element*> * weightedNeighbors(Element *e,double eps); //Weighted Neighbors

    void write(); //Metode que escriu per la terminal l'octree en POSTORDRE (arrel,f1,f2,f3,...)

    //metode inline per retornar l'i-essim element d'un SkipOctree.
    Element getIthElement(int i){return *llistaElements[i];}

    //metode inline per retornar el nombre d'elements d'un SkipOctree.
    int cardinal(){return llistaElements.size();}

    // metode auxiliar per simplificar les cerques, descomprimeix els fills del node t i ho empalma a nAux
    vector<bool> descomprimeix(SkipONode *t,SkipONode *nAux);
    // aquest altre ho torna a comprimir
    void recomprimeix(vector<bool> fillsTocats, SkipONode * nAux);

    vector<Element *> getLlistaElements(){return llistaElements;}

};

#endif