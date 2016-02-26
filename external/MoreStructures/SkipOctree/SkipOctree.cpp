//
// Created by yago on 16/02/26.
//

#include "SkipOctree.h"
#include <iostream>
#include <stdlib.h>

#define TOLERANCIA_CALCULAR_CUB 0.000001
const int MAX_LEVEL = 7;
//const double EPSILON = 0.05;

SkipOctree::SkipOctree()
{
    nivellMaxim = MAX_LEVEL;
    epsilon = -1;

    calcularCub(); //Calculem la mida i el punt d'ancoratge

    arrel = new SkipONode(ancoratge,mida,0); //Creem l'arrel amb el punt d'ancoratge, la mida i el nivell
    nivellActual = 0;

}

SkipOctree::SkipOctree(double eps)
{
    nivellMaxim = MAX_LEVEL;
    epsilon = eps;

    calcularCub(); //Calculem la mida i el punt d'ancoratge

    arrel = new SkipONode(ancoratge,mida,0); //Creem l'arrel amb el punt d'ancoratge, la mida i el nivell
    nivellActual = 0;
}

SkipOctree::SkipOctree(vector<Element *> llE, double eps)
{
//cout << "SkipOctree.cpp --> CREEM Skip OCTREE AMB "<< llE.size() << " ELEMENTS!" << endl;
llistaElements = llE;
nivellMaxim = MAX_LEVEL;
epsilon = eps;

calcularCub(); //Calculem la mida i el punt d'ancoratge

arrel = new SkipONode(ancoratge,mida,0); //Creem l'arrel amb el punt d'ancoratge, la mida i el nivell
nivellActual = 0;

llistaElements.clear();
for(int i=0; i<(int)llE.size(); i++) {
afegirElement(llE[i]);
}

}

SkipOctree::~SkipOctree()
{
    if(arrel!=NULL) delete arrel;

}


//CalcularCub: Busco distancia maxima entre cada coordenada
void SkipOctree::calcularCub()
{
    if (llistaElements.empty()) {
        //Si no hi ha cap esfera, creem un cub de mida 2 amb centre (0,0,0)
        ancoratge = point3D(-1,1,1);
        mida = 2;
    } else {
        //Primer calculem les tres coordenades per obtenir el punt d'ancoratge
        //i la distancia maxima entre dos elements
        double xMin,yMin,zMin;
        double xMax,yMax,zMax;

        point3D aux = llistaElements[0]->getPoint();

        xMin = aux.getX(); yMin = aux.getY(); zMin = aux.getZ();
        xMax = aux.getX(); yMax = aux.getY(); zMax = aux.getZ();

        for(int i=0; i<(int)llistaElements.size(); i++)
        {
            //Busquem distancies minimes i maximes entre coordenades
            aux = llistaElements[i]->getPoint();
            if (aux.getX() < xMin) xMin = aux.getX(); else if(aux.getX() > xMax) xMax = aux.getX();
            if (aux.getY() < yMin) yMin = aux.getY(); else if(aux.getY() > yMax) yMax = aux.getY();
            if (aux.getZ() < zMin) zMin = aux.getZ(); else if(aux.getZ() > zMax) zMax = aux.getZ();
        }

        //Calculem el punt d'ancoratge i la mida
        double distMax = fabs(xMax-xMin);
        if (distMax < fabs(yMax-yMin)) distMax = fabs(yMax-yMin);
        if (distMax < fabs(zMax-zMin)) distMax = fabs(zMax-zMin);

        if (distMax!=0.0) {
            mida = distMax + (2.0*epsilon) + TOLERANCIA_CALCULAR_CUB;
            ancoratge = point3D(xMin,yMax,zMax);

        } else {
            mida = 2.0 + (2.0*epsilon) + TOLERANCIA_CALCULAR_CUB;
            ancoratge = point3D(xMin-(mida/2.0),yMax+(mida/2.0),zMax+(mida/2.0));
        }
    }
}


point3D SkipOctree::getAncoratge()
{
    return ancoratge;
}

double SkipOctree::getMida()
{
    return mida;
}

int SkipOctree::getNivellActual()
{
    return nivellActual;
}

int SkipOctree::getNivellMaxim()
{
    return nivellMaxim;
}

void SkipOctree::setNivellMaxim(int n)
{
    nivellMaxim = n;
}

void SkipOctree::afegirElement(Element *e)
{
    llistaElements.push_back(e);
    point3D centre = e->getPoint();
    int aux;

    //Mirem si el centre de l'esfera forma part del cub de l'octree
    //if ( (centre.getX() >= ancoratge.getX()) && (centre.getX() < ancoratge.getX()+mida) &&
    //(centre.getY() <= ancoratge.getY()) && (centre.getY() > ancoratge.getY()-mida) &&
    //(centre.getZ() <= ancoratge.getZ()) && (centre.getZ() > ancoratge.getZ()-mida) ) {
    if(arrel->formaPart(centre))
    {
        //La esfera forma part del cub
        aux = arrel->afegirElement(e,nivellMaxim,NULL);
        if (aux > nivellActual) nivellActual = aux;
    } else {
        //La esfera no forma part del cub, per tant borrem l'octree i el tornem a crear
        delete arrel;
        calcularCub();
        arrel = new SkipONode(ancoratge,mida,0); //Creem l'arrel amb el punt d'ancoratge, la mida i el nivell
        nivellActual = 0;

        for(int i=0; i<(int)llistaElements.size(); i++) {
            //bool b= arrel->formaPart(llistaElements[i]->getPoint() );
            //if(!b)
            //{
            //	cout<<"COC calcular cub?"<<endl;
            //	exit(-1);
            //}
            //cout<<"COC vaig a reafegir "<<llistaElements[i]->getPoint()<<endl;
            aux = arrel->afegirElement(llistaElements[i],nivellMaxim,NULL);
            if (aux > nivellActual) nivellActual = aux;
        }
    }
}

void SkipOctree::esborrarElement(Element *e)
{
    bool trobat = false;
    int i = 0;

    vector<Element *>::iterator it = llistaElements.begin();

    //Primer de tot esborrem la esfera de la llista d'esferes
    while ( (i<(int)llistaElements.size()) && (!trobat) ) {
        if ( e == llistaElements[i] ) {
            trobat = true;
            llistaElements.erase(it);
        }
        i++; it++;
    }

    arrel->esborrarElement(e,nivellMaxim,NULL);
}


list<Element*> * SkipOctree::weightedNeighbors(Element *e,double eps)
{
    list<Element*> *ret = NULL;

    ret = arrel->weightedNeighbors(e,eps);

    return ret;
}



bool SkipOctree::deCostat2(SkipONode * t1, SkipONode * t2, int tipus)
{
    double tolerancia = 1.0e-6;
    point3D punt1 = t1->getAncoratge();
    point3D punt2 = t2->getAncoratge();
    bool ok1,ok2;

    switch (tipus) {
        case 1:
            //De costat per l'eix X
            ok1 = ( fabs(punt1.getY()-punt2.getY())<tolerancia );
            ok2 = ( fabs(punt1.getZ()-punt2.getZ())<tolerancia );
            if (ok1 && ok2) return true; else return false;
            break;
        case 2:
            //De costat per l'eix Y
            ok1 = ( fabs(punt1.getX()-punt2.getX())<tolerancia );
            ok2 = ( fabs(punt1.getZ()-punt2.getZ())<tolerancia );
            if (ok1 && ok2) return true; else return false;
            break;
        case 3:
            //De costat per l'eix Z
            ok1 = ( fabs(punt1.getX()-punt2.getX())<tolerancia );
            ok2 = ( fabs(punt1.getY()-punt2.getY())<tolerancia );
            if (ok1 && ok2) return true; else return false;
            break;
        default:
            cout << "ERROR (SkipOctree::deCostat) : No s'ha trobat el tipus" << endl;
            exit(-1);
            //per treure un warning
            return false;
            break;
    }

}

//Metode que busca quin fill (de t1 o de t2) escombra l'altre node.
//Precondicio: t1 i t2 no tenen la mateixa mida
int SkipOctree::quiEscombra2(SkipONode * t1, SkipONode * t2, int tipus)
{
    double tolerancia = 1.0e-6;

    point3D punt1 = t1->getAncoratge();
    point3D punt2 = t2->getAncoratge();

    double mida1 = t1->getMida();
    double mida2 = t2->getMida();

    switch (tipus) {
        case 1:
            //De costat per l'eix X
            if (mida1>mida2) {
                //El node 1 es mes gran que el node 2
                if ( (punt2.getY()<=punt1.getY()+tolerancia) && (punt2.getY()>punt1.getY()-(mida1/2.0)+tolerancia) ) {

                    if ( (punt2.getZ()<=punt1.getZ()+tolerancia) && (punt2.getZ()>punt1.getZ()-(mida1/2.0)+tolerancia) ) {
                        //Qui escombra es el fill 2 del node 1
                        return 2;
                    } else if ( (punt2.getZ()<=punt1.getZ()-(mida1/2.0)+tolerancia) && (punt2.getZ()>punt1.getZ()-mida1+tolerancia) ) {
                        //Qui escombra es el fill 6 del node 1
                        return 6;
                    } else {
                        return -1;
                    }

                } else if ( (punt2.getY()<=punt1.getY()-(mida1/2.0)+tolerancia) && (punt2.getY()>punt1.getY()-mida1+tolerancia) ) {

                    if ( (punt2.getZ()<=punt1.getZ()+tolerancia) && (punt2.getZ()>punt1.getZ()-(mida1/2.0)+tolerancia) ) {
                        //Qui escombra es el fill 4 del node 1
                        return 4;
                    } else if ( (punt2.getZ()<=punt1.getZ()-(mida1/2.0)+tolerancia) && (punt2.getZ()>punt1.getZ()-mida1+tolerancia) ) {
                        //Qui escombra es el fill 8 del node 1
                        return 8;
                    } else {
                        return -1;
                    }

                } else {
                    return -1;
                }
            } else {
                //El node 2 es mes gran que el node 1
                if ( (punt1.getY()<=punt2.getY()+tolerancia) && (punt1.getY()>punt2.getY()-(mida2/2.0)+tolerancia) ) {

                    if ( (punt1.getZ()<=punt2.getZ()+tolerancia) && (punt1.getZ()>punt2.getZ()-(mida2/2.0)+tolerancia) ) {
                        //Qui escombra es el fill 1 del node 2
                        return 1;
                    } else if ( (punt1.getZ()<=punt2.getZ()-(mida2/2.0)+tolerancia) && (punt1.getZ()>punt2.getZ()-mida2+tolerancia) ) {
                        //Qui escombra es el fill 5 del node 2
                        return 5;
                    } else {
                        return -1;
                    }

                } else if ( (punt1.getY()<=punt2.getY()-(mida2/2.0)+tolerancia) && (punt1.getY()>punt2.getY()-mida2+tolerancia) ) {

                    if ( (punt1.getZ()<=punt2.getZ()+tolerancia) && (punt1.getZ()>punt2.getZ()-(mida2/2.0)+tolerancia) ) {
                        //Qui escombra es el fill 3 del node 2
                        return 3;
                    } else if ( (punt1.getZ()<=punt2.getZ()-(mida2/2.0)+tolerancia) && (punt1.getZ()>punt2.getZ()-mida2+tolerancia) ) {
                        //Qui escombra es el fill 7 del node 2
                        return 7;
                    } else {
                        return -1;
                    }

                } else {
                    return -1;
                }
            }
        case 2:
            //De costat per l'eix Y
            if (mida1>mida2) {
                //El node 1 es mes gran que el node 2
                if ( (punt2.getX()>=punt1.getX()-tolerancia) && (punt2.getX()<punt1.getX()+(mida1/2.0)-tolerancia) ) {

                    if ( (punt2.getZ()<=punt1.getZ()+tolerancia) && (punt2.getZ()>punt1.getZ()-(mida1/2.0)+tolerancia) ) {
                        //Qui escombra es el fill 3 del node 1
                        return 3;
                    } else if ( (punt2.getZ()<=punt1.getZ()-(mida1/2.0)+tolerancia) && (punt2.getZ()>punt1.getZ()-mida1+tolerancia) ) {
                        //Qui escombra es el fill 7 del node 1
                        return 7;
                    } else {
                        return -1;
                    }

                } else if ( (punt2.getX()>=punt1.getX()+(mida1/2.0)-tolerancia) && (punt2.getX()<punt1.getX()+mida1-tolerancia) ) {

                    if ( (punt2.getZ()<=punt1.getZ()+tolerancia) && (punt2.getZ()>punt1.getZ()-(mida1/2.0)+tolerancia) ) {
                        //Qui escombra es el fill 4 del node 1
                        return 4;
                    } else if ( (punt2.getZ()<=punt1.getZ()-(mida1/2.0)+tolerancia) && (punt2.getZ()>punt1.getZ()-mida1+tolerancia) ) {
                        //Qui escombra es el fill 8 del node 1
                        return 8;
                    } else {
                        return -1;
                    }

                } else {
                    return -1;
                }
            } else {
                //El node 2 es mes gran que el node 1
                if ( (punt2.getX()>=punt1.getX()-tolerancia) && (punt2.getX()<punt1.getX()+(mida1/2.0)-tolerancia) ) {

                    if ( (punt2.getZ()<=punt1.getZ()+tolerancia) && (punt2.getZ()>punt1.getZ()-(mida1/2.0)+tolerancia) ) {
                        //Qui escombra es el fill 1 del node 2
                        return 1;
                    } else if ( (punt2.getZ()<=punt1.getZ()-(mida1/2.0)+tolerancia) && (punt2.getZ()>punt1.getZ()-mida1+tolerancia) ) {
                        //Qui escombra es el fill 5 del node 2
                        return 5;
                    } else {
                        return -1;
                    }

                } else if ( (punt2.getX()>=punt1.getX()+(mida1/2.0)-tolerancia) && (punt2.getX()<punt1.getX()+mida1-tolerancia) ) {

                    if ( (punt2.getZ()<=punt1.getZ()+tolerancia) && (punt2.getZ()>punt1.getZ()-(mida1/2.0)+tolerancia) ) {
                        //Qui escombra es el fill 2 del node 2
                        return 2;
                    } else if ( (punt2.getZ()<=punt1.getZ()-(mida1/2.0)+tolerancia) && (punt2.getZ()>punt1.getZ()-mida1+tolerancia) ) {
                        //Qui escombra es el fill 6 del node 2
                        return 6;
                    } else {
                        return -1;
                    }

                } else {
                    return -1;
                }
            }
            break;
        case 3:
            //De costat per l'eix Z
            if (mida1>mida2) {
                //El node 1 es mes gran que el node 2
                if ( (punt2.getX()>=punt1.getX()-tolerancia) && (punt2.getX()<punt1.getX()+(mida1/2.0)-tolerancia) ) {

                    if ( (punt2.getY()<=punt1.getY()+tolerancia) && (punt2.getY()>punt1.getY()-(mida1/2.0)+tolerancia) ) {
                        //Qui escombra es el fill 5 del node 1
                        return 5;
                    } else if ( (punt2.getY()<=punt1.getY()-(mida1/2.0)+tolerancia) && (punt2.getY()>punt1.getY()-mida1+tolerancia) ) {
                        //Qui escombra es el fill 7 del node 1
                        return 7;
                    } else {
                        return -1;
                    }

                } else if ( (punt2.getX()>=punt1.getX()+(mida1/2.0)-tolerancia) && (punt2.getX()<punt1.getX()+mida1-tolerancia) ) {

                    if ( (punt2.getY()<=punt1.getY()+tolerancia) && (punt2.getY()>punt1.getY()-(mida1/2.0)+tolerancia) ) {
                        //Qui escombra es el fill 6 del node 1
                        return 6;
                    } else if ( (punt2.getY()<=punt1.getY()-(mida1/2.0)+tolerancia) && (punt2.getY()>punt1.getY()-mida1+tolerancia) ) {
                        //Qui escombra es el fill 8 del node 1
                        return 8;
                    } else {
                        return -1;
                    }

                } else {
                    return -1;
                }
            } else {
                //El node 2 es mes gran que el node 1
                if ( (punt2.getX()>=punt1.getX()-tolerancia) && (punt2.getX()<punt1.getX()+(mida1/2.0)-tolerancia) ) {

                    if ( (punt2.getY()<=punt1.getY()+tolerancia) && (punt2.getY()>punt1.getY()-(mida1/2.0)+tolerancia) ) {
                        //Qui escombra es el fill 1 del node 2
                        return 1;
                    } else if ( (punt2.getY()<=punt1.getY()-(mida1/2.0)+tolerancia) && (punt2.getY()>punt1.getY()-mida1+tolerancia) ) {
                        //Qui escombra es el fill 3 del node 2
                        return 3;
                    } else {
                        return -1;
                    }

                } else if ( (punt2.getX()>=punt1.getX()+(mida1/2.0)-tolerancia) && (punt2.getX()<punt1.getX()+mida1-tolerancia) ) {

                    if ( (punt2.getY()<=punt1.getY()+tolerancia) && (punt2.getY()>punt1.getY()-(mida1/2.0)+tolerancia) ) {
                        //Qui escombra es el fill 2 del node 2
                        return 2;
                    } else if ( (punt2.getY()<=punt1.getY()-(mida1/2.0)+tolerancia) && (punt2.getY()>punt1.getY()-mida1+tolerancia) ) {
                        //Qui escombra es el fill 4 del node 2
                        return 4;
                    } else {
                        return -1;
                    }

                } else {
                    return -1;
                }
            }
            break;
        default:
            cout << "ERROR (SkipOctree::deCostat) : No s'ha trobat el tipus" << endl;
            break;
    }

    return -1;
}


void SkipOctree::write()
{
    cout << "-------------------------------------------------------------------" << endl;
    cout << "OCTREE: RECORREGUT EN POSTORDRE (arrel,fill 1, fill 2, fill 3, ...)" << endl;
    cout << "-------------------------------------------------------------------" << endl;
    if (arrel != NULL) {
        arrel->write();
    } else {
        cout << "L'octree no conte cap element!" << endl;
    }
    cout << "-------------------------------------------------------------------" << endl;
    cout << "                     OCTREE: FINAL RECORREGUT" << endl;
    cout << "-------------------------------------------------------------------" << endl;
}


vector<bool> SkipOctree::descomprimeix(SkipONode *t,SkipONode *nAux)
{
    // per aquells fils de t que no siguin de "la mida inmediatament inferior a t" hi afegirem un node intermig d'aquesta mida i mantindrem els antics fills de t com a nodes d'un nivell inferior ("nets"). Tota l'estructura nova penjara del node auxiliar nAux per facilitar despres la seva eliminacio sense perdre memoria ni esborrar res que "no toqui"

    //cout<<"surto de descomprimir amb t: "<<endl;
    //t->write();
    //cout<<"i nAux: "<<endl;
    //nAux->write();

    double mida=t->getMida();
    double tolerancia= 0.00000000001;

    // primer, posem tots els fills en un vector i vigilem quins han de ser expandits
    vector<SkipONode * > vFills = vector<SkipONode * >(8);
    vector<bool> quinsFills	=vector<bool>(8,false);

    // posem cada node al vector per tal de poder-los tractar amb un bucle
    vFills[0]=t->getFill1();
    vFills[1]=t->getFill2();
    vFills[2]=t->getFill3();
    vFills[3]=t->getFill4();
    vFills[4]=t->getFill5();
    vFills[5]=t->getFill6();
    vFills[6]=t->getFill7();
    vFills[7]=t->getFill8();

    // cada node que no sigui de la mida inmediatament inferior
    // s'ha de descomprimir, el vector de booleans ens servira despres per "recomprimir"
    for (unsigned int i=0;i!=vFills.size();i++)
    {
        // si no es NULL ni fulla blanca
        if( (vFills[i]!=NULL) && (vFills[i]->tipus()!=0 ) )
        {
            double midaFill=vFills[i]->getMida();
            // mirem si te mida menor de la que toca, apliquem una petita tolerancia
            if( midaFill*2. < (mida-tolerancia) )
            {
                // aquest node cal descomprimirlo

                // primer de tot, ho apuntem
                quinsFills[i]=true;

                // ara hem de crear un node fill intermig ("adoptiu")
                // ha de tenir com a punt d'ancoratge el que tindria l'(i-1)-essim
                // fill de t de mida exactament la meitat que la mida de t, mida t i
                // ha de tenir tots els seus fill nulls excepte el que ja no era null de t
                // la informacio geometrica i la llista de nodes han de ser les del fill de t

                point3D ancoratgeFillAdoptiu=point3D(t->getAncoratge());
                switch (i) { //Mirem a quin fill corresponen i baixem de nivell
                    //case 1: No s'ha de fer res ja que el punt no varia
                    case 2:
                        ancoratgeFillAdoptiu.setX( ancoratgeFillAdoptiu.getX()+(mida/2) );
                        break;
                    case 3:
                        ancoratgeFillAdoptiu.setY( ancoratgeFillAdoptiu.getY()-(mida/2) );
                        break;
                    case 4:
                        ancoratgeFillAdoptiu.setX( ancoratgeFillAdoptiu.getX()+(mida/2) );
                        ancoratgeFillAdoptiu.setY( ancoratgeFillAdoptiu.getY()-(mida/2) );
                        break;
                    case 5:
                        ancoratgeFillAdoptiu.setZ( ancoratgeFillAdoptiu.getZ()-(mida/2) );
                        break;
                    case 6:
                        ancoratgeFillAdoptiu.setX( ancoratgeFillAdoptiu.getX()+(mida/2) );
                        ancoratgeFillAdoptiu.setZ( ancoratgeFillAdoptiu.getZ()-(mida/2) );
                        break;
                    case 7:
                        ancoratgeFillAdoptiu.setY( ancoratgeFillAdoptiu.getY()-(mida/2) );
                        ancoratgeFillAdoptiu.setZ( ancoratgeFillAdoptiu.getZ()-(mida/2) );
                        break;
                    case 8:
                        ancoratgeFillAdoptiu.setX( ancoratgeFillAdoptiu.getX()+(mida/2) );
                        ancoratgeFillAdoptiu.setY( ancoratgeFillAdoptiu.getY()-(mida/2) );
                        ancoratgeFillAdoptiu.setZ( ancoratgeFillAdoptiu.getZ()-(mida/2) );
                        break;
                }

                SkipONode *fillAdoptiu = new SkipONode(ancoratgeFillAdoptiu,mida/2,t->getNivell()+1);
                // li posem la llista d'elements del que sera el seu fill
                fillAdoptiu->setLlistaElements( vFills[i]->getLlistaElements());


                // ara fillAdoptiu te tots els seus fills nulls, falta penjar vFills[i]
                // buscarem a quin fill cau i el penjarem d'allà

                switch(fillAdoptiu->aQuinFill(vFills[i]))
                {
                    case 1:
                        fillAdoptiu->setFill1(vFills[i]);
                        break;
                    case 2:
                        fillAdoptiu->setFill2(vFills[i]);
                        break;
                    case 3:
                        fillAdoptiu->setFill3(vFills[i]);
                        break;
                    case 4:
                        fillAdoptiu->setFill4(vFills[i]);
                        break;
                    case 5:
                        fillAdoptiu->setFill5(vFills[i]);
                        break;
                    case 6:
                        fillAdoptiu->setFill6(vFills[i]);
                        break;
                    case 7:
                        fillAdoptiu->setFill7(vFills[i]);
                        break;
                    case 8:
                        fillAdoptiu->setFill8(vFills[i]);
                        break;
                    default:
                        cout<<"SkipOctree:Descomprimir, error"<<endl;
                        exit(-1);
                }

                // ara ja esta correcte, ara el fillAdoptiu es el que considerarem com a nou fill
                vFills[i]=fillAdoptiu;
            }
        }

    }

    //ara actualitzem el node auxiliar amb els nous fills que han sortit del proces:
    // els nodes null o fulles blanques segueixen igual
    // els altres:
    // els de mida adequada (la meitat que el seu pare) tambe segueixen igual
    // els de mida "no adequada" han estat substituits per fills adoptius

    nAux->setFill1(	vFills[0]);
    nAux->setFill2(	vFills[1]);
    nAux->setFill3(	vFills[2]);
    nAux->setFill4(	vFills[3]);
    nAux->setFill5(	vFills[4]);
    nAux->setFill6(	vFills[5]);
    nAux->setFill7(	vFills[6]);
    nAux->setFill8(	vFills[7]);

    //cout<<"surto de descomprimir amb t: "<<endl;
    //t->write();
    //cout<<"i nAux: "<<endl;
    //nAux->write();

    return quinsFills;
}

void SkipOctree::recomprimeix(vector<bool> fillsTocats, SkipONode * nAux)
{

    //cout<<"reccomprimir "<<endl;
    //nAux->write();

    // primer, posem tots els fills en un vector i vigilem quins han de ser desenganxats de l'estructura
    vector<SkipONode * > vFills = vector<SkipONode * >(8);
    vector<bool> quinsFills	=vector<bool>(8,false);

    // controlem que no estiguin passant coses rares
    if(nAux==NULL)
    {
        cout<<"CO: Recomprimeix: Error de node NULL!"<<endl;
        exit(-1);
    }


    // posem cada node al vector per tal de poder-los tractar amb un bucle
    vFills[0]=nAux->getFill1();
    vFills[1]=nAux->getFill2();
    vFills[2]=nAux->getFill3();
    vFills[3]=nAux->getFill4();
    vFills[4]=nAux->getFill5();
    vFills[5]=nAux->getFill6();
    vFills[6]=nAux->getFill7();
    vFills[7]=nAux->getFill8();

    for (unsigned int i=0;i!=fillsTocats.size();i++)
    {
        if(fillsTocats[i])
        {
            //aquest si que l'haviem tocat, hi ha un node intermig que s'ha de desenganxar de
            //l'estructura i mantenir enganxat a nodeAux perque s'alliberi en morir nAux, tambe cal vigilar
            //la informacio geometrica que conte

            // vigilem que no fos part de cap zona candidata:
        }
        else
        {
            // aquest no l'haviem tocat, simplement desfem el punter doblat per desenganxar nAux de l'estructura
            vFills[i]=NULL;

        }

    }

    nAux->setFill1(vFills[0]);
    nAux->setFill2(vFills[1]);
    nAux->setFill3(vFills[2]);
    nAux->setFill4(vFills[3]);
    nAux->setFill5(vFills[4]);
    nAux->setFill6(vFills[5]);
    nAux->setFill7(vFills[6]);
    nAux->setFill8(vFills[7]);


    //cout<<"surto de reccomprimir "<<endl;
    //nAux->write();
}