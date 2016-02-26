//
// Created by yago on 16/02/26.
//

#include "SkipONode.h"
#include "../../../include/timer.h"
#include <cstdlib>
#include <iostream>
#include <cmath>


//#define EPSILON 0.5
#define TOLERANCIA_DETECTAR_PERILL_A_ESBORRAR_NODE 0.00001
#define TOLERANCIA_METODE_A_QUIN_FILL 0.00001
#define NUMERO_PUNTS_GRAELLA_STABBING 10
#define MAX_ELEMENTS_SEGUIR_BUSCANT 10
#define PI 3.14159265358979323846

SkipONode::SkipONode()
{
    ancoratge = point3D(-1,1,1);
    mida = 2;
    nivell = 0;


    f1 = NULL;
    f2 = NULL;
    f3 = NULL;
    f4 = NULL;
    f5 = NULL;
    f6 = NULL;
    f7 = NULL;
    f8 = NULL;

}

SkipONode::SkipONode(point3D a,double m,int n)
{
    ancoratge = a;
    mida = m;
    nivell = n;

    //Com que estem creant un node fulla, tots els seus fills = NULL
    f1 = NULL;
    f2 = NULL;
    f3 = NULL;
    f4 = NULL;
    f5 = NULL;
    f6 = NULL;
    f7 = NULL;
    f8 = NULL;

}

SkipONode::SkipONode(const SkipONode &n)
{
    if (this != &n)
    {  // make sure not same object

        ancoratge=n.ancoratge;

        mida=n.mida;

        nivell=n.nivell;

        llistaElements=n.llistaElements;


        f1=n.f1;
        f2=n.f2;
        f3=n.f3;
        f4=n.f4;
        f5=n.f5;
        f6=n.f6;
        f7=n.f7;
        f8=n.f8;

    }
}

SkipONode::~SkipONode()
{
    //cout<<"CON: Estic alliberant "<<this->getAncoratge()<<" "<<this->getMida()<<endl;
    //cout<<"amb fills: "<<f1<<f2<<f3<<f4<<f5<<f6<<f7<<f8<<endl;

    delete f1;
    delete f2;
    delete f3;
    delete f4;
    delete f5;
    delete f6;
    delete f7;
    delete f8;

    f1=NULL;
    f2=NULL;
    f3=NULL;
    f4=NULL;
    f5=NULL;
    f6=NULL;
    f7=NULL;
    f8=NULL;

}

SkipONode& SkipONode::operator=(const SkipONode &n)
{
    if (this != &n)
    {  // make sure not same object

        ancoratge=n.ancoratge;

        mida=n.mida;

        nivell=n.nivell;

        llistaElements=n.llistaElements;


        f1=n.f1;
        f2=n.f2;
        f3=n.f3;
        f4=n.f4;
        f5=n.f5;
        f6=n.f6;
        f7=n.f7;
        f8=n.f8;

    }

    return *this;
}

point3D SkipONode::getAncoratge()
{
    return ancoratge;
}

double SkipONode::getMida()
{
    return mida;
}

int SkipONode::getNivell()
{
    return nivell;
}

SkipONode* SkipONode::getFill1()
{
    return f1;
}

SkipONode* SkipONode::getFill2()
{
    return f2;
}

SkipONode* SkipONode::getFill3()
{
    return f3;
}

SkipONode* SkipONode::getFill4()
{
    return f4;
}

SkipONode* SkipONode::getFill5()
{
    return f5;
}

SkipONode* SkipONode::getFill6()
{
    return f6;
}

SkipONode* SkipONode::getFill7()
{
    return f7;
}

SkipONode* SkipONode::getFill8()
{
    return f8;
}

vector<Element *> SkipONode::getLlistaElements()
{
    //cout<<" CON getllistaELements "<<llistaElements.size()<<endl;
    return llistaElements;
}


void SkipONode::setAncoratge(point3D a)
{
    ancoratge = a;
}

void SkipONode::setMida(double m)
{
    mida = m;
}

void SkipONode::setNivell(int n)
{
    nivell = n;
}

void SkipONode::setLlistaElements(vector<Element *> llE)
{
    llistaElements = llE;
}

void SkipONode::setFill1(SkipONode *n)
{
    f1 = n;
}

void SkipONode::setFill2(SkipONode *n)
{
    f2 = n;
}

void SkipONode::setFill3(SkipONode *n)
{
    f3 = n;
}

void SkipONode::setFill4(SkipONode *n)
{
    f4 = n;
}

void SkipONode::setFill5(SkipONode *n)
{
    f5 = n;
}

void SkipONode::setFill6(SkipONode *n)
{
    f6 = n;
}

void SkipONode::setFill7(SkipONode *n)
{
    f7 = n;
}

void SkipONode::setFill8(SkipONode *n)
{
    f8 = n;
}


void SkipONode::copiarAtributs(SkipONode *n)
{
    ancoratge = n->ancoratge;
    mida = n->mida;
    nivell = n->nivell;
    llistaElements = n->llistaElements;

    f1 = n->f1;
    f2 = n->f2;
    f3 = n->f3;
    f4 = n->f4;
    f5 = n->f5;
    f6 = n->f6;
    f7 = n->f7;
    f8 = n->f8;
}

int SkipONode::afegirElement(Element *e,int nivellMaxim,SkipONode *pare)
{
    int res = 0;

    switch (tipus()) {
        case 0: //Fulla blanca
            //cout<<"CON AFEGIR, fulla blanca "<<" nivell: "<<nivell<<" pare: "<<pare<<" centre "<<(*e).getPoint()<<endl;
            //Un octree comprimit nomes tindra una fulla blanca quan estigui buit (nomes hi hagi l'arrel).
            //Recordem que els octrees comprimits tenen nul en comptes de fulla blanca.
            llistaElements.push_back(e);
            res = nivell;
            break;

        case 1: //Fulla negre (i no nivell maxim)
            //cout<<"CON AFEGIR, fulla negra "<<" nivell: "<<nivell<<" pare: "<<pare<<" centre "<<(*e).getPoint()<<endl;
            //Transformem el node actual en node parcial i hi introduim els 2 elements.
            if (nivell < nivellMaxim) {
                llistaElements.push_back(e);
                res = crearFills(nivellMaxim);
                break;
            }
            //Si es nivell maxim anira al case 2 i reestructurara l'octree si es necessari

        case 2: //Node parcial
            //cout<<"CON AFEGIR, parcial "<<" nivell: "<<nivell<<" pare: "<<pare<<" centre "<<(*e).getPoint()<<endl;

            //Comprovem que l'element formi part del node
            if (formaPart(e->getPoint())) {
                llistaElements.push_back(e);
                res = afegirElementFillCorresponent(e,nivellMaxim);
            } else {
                //Tenim que l'element ha d'anar a un node que no esta creat i
                //en el seu lloc hi ha un altre node d'un nivell inferior.
                //El que hem de fer es crear un nou node parcial i afegir-hi
                //l'element actual i el node de nivell inferior com a fills seus.
                SkipONode *aux = NULL; //Nou node parcial

                //cout<<"CON "<<pare<<endl;

                double pareX = (pare->getAncoratge()).getX();
                double pareY = (pare->getAncoratge()).getY();
                double pareZ = (pare->getAncoratge()).getZ();
                double pareMida = pare->getMida();
                int pareNivell = pare->getNivell();
                switch (pare->aQuinFill(e)) {
                    case 1:
                        aux = new SkipONode(pare->getAncoratge(),pareMida/2,pareNivell+1);
                        pare->setFill1(aux);
                        break;
                    case 2:
                        aux = new SkipONode(point3D(pareX+(pareMida/2),pareY,pareZ),pareMida/2,pareNivell+1);
                        pare->setFill2(aux);
                        break;
                    case 3:
                        aux = new SkipONode(point3D(pareX,pareY-(pareMida/2),pareZ),pareMida/2,pareNivell+1);
                        pare->setFill3(aux);
                        break;
                    case 4:
                        aux = new SkipONode(point3D(pareX+(pareMida/2),pareY-(pareMida/2),pareZ),pareMida/2,pareNivell+1);
                        pare->setFill4(aux);
                        break;
                    case 5:
                        aux = new SkipONode(point3D(pareX,pareY,pareZ-(pareMida/2)),pareMida/2,pareNivell+1);
                        pare->setFill5(aux);
                        break;
                    case 6:
                        aux = new SkipONode(point3D(pareX+(pareMida/2),pareY,pareZ-(pareMida/2)),pareMida/2,pareNivell+1);
                        pare->setFill6(aux);
                        break;
                    case 7:
                        aux = new SkipONode(point3D(pareX,pareY-(pareMida/2),pareZ-(pareMida/2)),pareMida/2,pareNivell+1);
                        pare->setFill7(aux);
                        break;
                    case 8:
                        aux = new SkipONode(point3D(pareX+(pareMida/2),pareY-(pareMida/2),pareZ-(pareMida/2)),pareMida/2,pareNivell+1);
                        pare->setFill8(aux);
                        break;
                }

                //Afegim els elements al nou node
                vector<Element *> llistaAux = llistaElements;
                llistaAux.push_back(e);
                aux->setLlistaElements(llistaAux);

                //Ara, igual que en el metode create sons, mirem a quin fill
                //hem de posar l'element actual i el node parcial. En cas que
                //anessin al mateix fill, transformarem el node 'aux' en un
                //nivell inferior i repetirem el procés.
                int fillNodeAntic = aux->aQuinFill(this);
                int fillNouElement = aux->aQuinFill(e);

                while ( (fillNodeAntic==fillNouElement) && (aux->getNivell()<nivellMaxim) ) { //Mentre els 2 elements hagin d'anar al mateix fill
                    // YY no estiguem al nivell maxim -1 (posem el -1 perque quan afegim els 2 elements baixarem
                    // un altre nivell)

                    //Primer canviem el punt d'ancoratge (basePoint)
                    switch (fillNouElement) { //Mirem a quin fill corresponen i baixem de nivell
                        //case 1: No s'ha de fer res ja que el punt no varia
                        case 2:
                            aux->ancoratge.setX( aux->ancoratge.getX()+(aux->mida/2.) );
                            break;
                        case 3:
                            aux->ancoratge.setY( aux->ancoratge.getY()-(aux->mida/2.) );
                            break;
                        case 4:
                            aux->ancoratge.setX( aux->ancoratge.getX()+(aux->mida/2.) );
                            aux->ancoratge.setY( aux->ancoratge.getY()-(aux->mida/2.) );
                            break;
                        case 5:
                            aux->ancoratge.setZ( aux->ancoratge.getZ()-(aux->mida/2.) );
                            break;
                        case 6:
                            aux->ancoratge.setX( aux->ancoratge.getX()+(aux->mida/2.) );
                            aux->ancoratge.setZ( aux->ancoratge.getZ()-(aux->mida/2.) );
                            break;
                        case 7:
                            aux->ancoratge.setY( aux->ancoratge.getY()-(aux->mida/2.) );
                            aux->ancoratge.setZ( aux->ancoratge.getZ()-(aux->mida/2.) );
                            break;
                        case 8:
                            aux->ancoratge.setX( aux->ancoratge.getX()+(aux->mida/2.) );
                            aux->ancoratge.setY( aux->ancoratge.getY()-(aux->mida/2.) );
                            aux->ancoratge.setZ( aux->ancoratge.getZ()-(aux->mida/2.) );
                            break;
                    }
                    aux->mida = aux->mida/2; //La mida del node sera la meitat
                    aux->nivell++; //El node tindra un nivell mes

                    //Actualitzem el fill que han d'anar els elements
                    fillNodeAntic = aux->aQuinFill(this);
                    fillNouElement = aux->aQuinFill(e);
                }

                //Mirem si hem arribat al nivell maxim. En cas d'haver-hi arribat,
                //no cal que fem res (ja hi ha els elements dins el node aux).
                //Altrament, afegim el node actual i l'element com a fills del node aux.
                if (aux->getNivell() < nivellMaxim) {
                    //Afegim el node
                    switch (fillNodeAntic) {
                        case 1:
                            aux->setFill1(this);
                            break;
                        case 2:
                            aux->setFill2(this);
                            break;
                        case 3:
                            aux->setFill3(this);
                            break;
                        case 4:
                            aux->setFill4(this);
                            break;
                        case 5:
                            aux->setFill5(this);
                            break;
                        case 6:
                            aux->setFill6(this);
                            break;
                        case 7:
                            aux->setFill7(this);
                            break;
                        case 8:
                            aux->setFill8(this);
                            break;
                    }

                    //Afegim l'element
                    res = aux->afegirElementFillCorresponent(e,nivellMaxim);
                }
            }
            break;
    }
    //}
    return res;
}


void SkipONode::esborrarElement(Element *e,int nivellMaxim,SkipONode *pare)
{
    vector<Element*>::iterator i;
    bool final;

    //Mirem si hem de seguir buscant pels fills
    switch (tipus())
    {
        case 0:
            cout << "ERROR (esborrarElement) : He anat a petar a una fulla blanca!" << endl;
            exit(-1);
            break;
        case 1:

            //Si hi es, esborra'l de la llista
            i = llistaElements.end();
            final = false;
            while( (i!=llistaElements.begin()) && (!final) )
            {
                i--;
                if( ((*i)->getRadi()==e->getRadi()) && ( (*i)->getPoint()==e->getPoint() ) )
                {
                    final = true;
                }
            }

            if(final)
            {
                llistaElements.erase(i);
            }
            else
            {
                cout << "ERROR (esborrarElement) : Soc una fulla negre i intentes esborrar un que no hi es!" << endl;
                exit(-1);
            }

            //Ara hem de comprovar si s'ha quedat com una fulla negre amb un sol element
            if (llistaElements.size() == 1) {

                if ( (pare!=NULL) && (pare->getNivell()<nivell-1) ){
                    //Creem un node d'un nivell inferior al pare
                    double pareX = (pare->getAncoratge()).getX();
                    double pareY = (pare->getAncoratge()).getY();
                    double pareZ = (pare->getAncoratge()).getZ();
                    double pareMida = pare->getMida();
                    int pareNivell = pare->getNivell();
                    switch (pare->aQuinFill(this)) {
                        case 1:
                            ancoratge.setX( pareX );
                            ancoratge.setY( pareY );
                            ancoratge.setZ( pareZ );
                        case 2:
                            ancoratge.setX( pareX+(pareMida/2.) );
                            ancoratge.setY( pareY );
                            ancoratge.setZ( pareZ );
                            break;
                        case 3:
                            ancoratge.setX( pareX );
                            ancoratge.setY( pareY-(pareMida/2.) );
                            ancoratge.setZ( pareZ );
                            break;
                        case 4:
                            ancoratge.setX( pareX+(pareMida/2.) );
                            ancoratge.setY( pareY-(pareMida/2.) );
                            ancoratge.setZ( pareZ );
                            break;
                        case 5:
                            ancoratge.setX( pareX );
                            ancoratge.setY( pareY );
                            ancoratge.setZ( pareZ-(pareMida/2.) );
                            break;
                        case 6:
                            ancoratge.setX( pareX+(pareMida/2.) );
                            ancoratge.setY( pareY );
                            ancoratge.setZ( pareZ-(pareMida/2.) );
                            break;
                        case 7:
                            ancoratge.setX( pareX );
                            ancoratge.setY( pareY-(pareMida/2.) );
                            ancoratge.setZ( pareZ-(pareMida/2.) );
                            break;
                        case 8:
                            ancoratge.setX( pareX+(pareMida/2.) );
                            ancoratge.setY( pareY-(pareMida/2.) );
                            ancoratge.setZ( pareZ-(pareMida/2.) );
                            break;
                    }
                    nivell = pareNivell+1;
                    mida = pareMida/2.;
                }
            }
            break;
        case 2:
            vector<bool> possiblesFills = aQuinsFills(e);
            vector<SkipONode *> fills = vector<SkipONode *>(8);
            fills[0]=f1;
            fills[1]=f2;
            fills[2]=f3;
            fills[3]=f4;
            fills[4]=f5;
            fills[5]=f6;
            fills[6]=f7;
            fills[7]=f8;
            int numFillsPossibles = 0;
            for (int f=0;f<8;f++) {if (possiblesFills[f]) {numFillsPossibles++;}}

            int fillBo;
            if (numFillsPossibles==1) {
                //cout<<"CON: un sol fill possible "<<endl;				
                //Només un fill candidat
                int actual = 0;
                bool trobat = false;
                while (!trobat) {
                    if (possiblesFills[actual]) {fillBo = actual; trobat = true;}
                    actual++;
                }

            } else if (numFillsPossibles > 1) {
                //cout<<"CON: mes dun fill possible "<<endl;				
                //Hi ha varis fills candidats, busquem quin és el que conte l'element e.

                //Primerament, comencem a buscar pel fill que teoricament hauria d'estar
                fillBo = aQuinFill(e)-1;

                if ( (fills[fillBo]==NULL) || ( (fills[fillBo]!=NULL)&& (!fills[fillBo]->pertany(e)) ) ) {

                    //No l'hem trobat, ara busquem pels altres!
                    possiblesFills[fillBo] = false; //Aquest ja l'hem buscat!
                    int act = 0;
                    bool trob = false;

                    while ( (!trob) && (act<8) ) {
                        if ( (possiblesFills[act]) && (fills[act]!=NULL) && (fills[act]->pertany(e)) ) {
                            fillBo = act;
                            trob = true;
                        }
                        act++;
                    }

                    if (!trob) {
                        cout << "ERROR (SkipONode::esborrarElement()) : No he trobat l'element a cap fill" << endl;
                        exit(-1);
                    }
                }

            } else {
                cout << "ERROR (SkipONode::esborrarElement()) : No he trobat cap possible fill on hi hagi e" << endl;
                exit(-1);
            }

            //Tenim el fill que conte l'element (fillBo)
            if (fills[fillBo]!=NULL) {
                fills[fillBo]->esborrarElement(e,nivellMaxim,this);
                if (fills[fillBo]->tipus()==0) { //Si s'ha convertit en fulla blanca
                    switch (fillBo) {
                        case 0: delete f1; f1=NULL; break;
                        case 1: delete f2; f2=NULL; break;
                        case 2: delete f3; f3=NULL; break;
                        case 3: delete f4; f4=NULL; break;
                        case 4: delete f5; f5=NULL; break;
                        case 5: delete f6; f6=NULL; break;
                        case 6: delete f7; f7=NULL; break;
                        case 7: delete f8; f8=NULL; break;
                    }
                }
            } else {
                cout << "ERROR (SkipONode::esborrarElement()) : He anat a petar a un node NULL" << endl;
                cout<<"pare "<<pare<<endl;
                cout<<" pertany al pare? "<<pare->pertany(e)<<endl;
                cout<<" pertany a mi ? "<<pertany(e)<<endl;


                //  miro els possibles
                for(int iP=0;iP<8;iP++)
                {
                    cout<<iP<<" fill possible? "<<possiblesFills[iP]<<endl;
                    if( (fills[iP]!=NULL) ) cout<<"pertany? "<<fills[iP]->pertany(e)<<endl;
                    else cout<<"fill nul "<<endl;
                }

                //pinto l'element
                e->write();

                //em pinto
                cout << endl << endl;
                write();

                exit(-1);
            }

            //Eliminem l'element de la llista
            i = llistaElements.end();
            final = false;
            while( (i!=llistaElements.begin()) && (!final) )
            {
                i--;
                if( ( (*i)->getRadi()==e->getRadi()) && ((*i)->getPoint()==e->getPoint()) )
                {
                    final = true;
                }
            }

            if(final)
            {
                llistaElements.erase(i);
            }
            else
            {
                cout << "ERROR (SkipONode::esborrarElement()) : Soc un node parcial i intentes esborrar un que no hi es!" << endl;
                exit(-1);
            }

            //Refem l'estructura de l'arbre en cas que ho necessitem
            switch ( fillNoFullaBlanca() )
            {
                case 1:
                    switch (f1->tipus())
                    {
                        case 1:
                            //Comprovem quants elements hi ha
                            if (f1->getLlistaElements().size()>1) {
                                //Més d'un element = Nivell Maxim
                                //Transformem el node actual com el node fill i eliminem el fill
                                SkipONode *aux = f1;
                                copiarAtributs(f1);
                                delete aux;
                            } else {
                                if ( (pare!=NULL) && (pare->getNivell()<nivell-1) ){
                                    //Creem un node d'un nivell inferior al pare					
                                    double pareX = (pare->getAncoratge()).getX();
                                    double pareY = (pare->getAncoratge()).getY();
                                    double pareZ = (pare->getAncoratge()).getZ();
                                    double pareMida = pare->getMida();
                                    int pareNivell = pare->getNivell();
                                    switch (pare->aQuinFill(f1)) {
                                        case 1:
                                            setAncoratge(pare->getAncoratge());
                                            break;
                                        case 2:
                                            setAncoratge(point3D(pareX+(pareMida/2),pareY,pareZ));
                                            break;
                                        case 3:
                                            setAncoratge(point3D(pareX,pareY-(pareMida/2),pareZ));
                                            break;
                                        case 4:
                                            setAncoratge(point3D(pareX+(pareMida/2),pareY-(pareMida/2),pareZ));
                                            break;
                                        case 5:
                                            setAncoratge(point3D(pareX,pareY,pareZ-(pareMida/2)));
                                            break;
                                        case 6:
                                            setAncoratge(point3D(pareX+(pareMida/2),pareY,pareZ-(pareMida/2)));
                                            break;
                                        case 7:
                                            setAncoratge(point3D(pareX,pareY-(pareMida/2),pareZ-(pareMida/2)));
                                            break;
                                        case 8:
                                            setAncoratge(point3D(pareX+(pareMida/2),pareY-(pareMida/2),pareZ-(pareMida/2)));
                                            break;
                                    }
                                    mida = pareMida/2.0;
                                    nivell = pareNivell+1;
                                    //Afegim els elements al nou node
                                    llistaElements = f1->getLlistaElements();
                                    delete f1; f1=NULL;
                                } else {
                                    llistaElements = f1->getLlistaElements();
                                    delete f1; f1=NULL;
                                }
                            }
                            break;
                        case 2:
                            if (nivell!=0) { //Si no es el node arrel
                                //Transformem el node actual com el node fill i eliminem el fill
                                SkipONode *aux = f1;
                                copiarAtributs(f1);
                                aux->setFill1(NULL);
                                aux->setFill2(NULL);
                                aux->setFill3(NULL);
                                aux->setFill4(NULL);
                                aux->setFill5(NULL);
                                aux->setFill6(NULL);
                                aux->setFill7(NULL);
                                aux->setFill8(NULL);
                                delete aux;
                            }
                            break;
                        default:
                            cout << "ERROR (SkipONode::esborrarElement()) : Refent el node!" << endl;
                            exit(-1);
                            break;
                    }
                    break;
                case 2:
                    switch (f2->tipus())
                    {
                        case 1:
                            //Comprovem quants elements hi ha
                            if (f2->getLlistaElements().size()>1) {
                                //Més d'un element = Nivell Maxim
                                //Transformem el node actual com el node fill i eliminem el fill
                                SkipONode *aux = f2;
                                copiarAtributs(f2);
                                delete aux;
                            } else {
                                if ( (pare!=NULL) && (pare->getNivell()<nivell-1) ){
                                    //Creem un node d'un nivell inferior al pare

                                    double pareX = (pare->getAncoratge()).getX();
                                    double pareY = (pare->getAncoratge()).getY();
                                    double pareZ = (pare->getAncoratge()).getZ();
                                    double pareMida = pare->getMida();
                                    int pareNivell = pare->getNivell();
                                    switch (pare->aQuinFill(f2)) {
                                        case 1:
                                            setAncoratge(pare->getAncoratge());
                                            break;
                                        case 2:
                                            setAncoratge(point3D(pareX+(pareMida/2),pareY,pareZ));
                                            break;
                                        case 3:
                                            setAncoratge(point3D(pareX,pareY-(pareMida/2),pareZ));
                                            break;
                                        case 4:
                                            setAncoratge(point3D(pareX+(pareMida/2),pareY-(pareMida/2),pareZ));
                                            break;
                                        case 5:
                                            setAncoratge(point3D(pareX,pareY,pareZ-(pareMida/2)));
                                            break;
                                        case 6:
                                            setAncoratge(point3D(pareX+(pareMida/2),pareY,pareZ-(pareMida/2)));
                                            break;
                                        case 7:
                                            setAncoratge(point3D(pareX,pareY-(pareMida/2),pareZ-(pareMida/2)));
                                            break;
                                        case 8:
                                            setAncoratge(point3D(pareX+(pareMida/2),pareY-(pareMida/2),pareZ-(pareMida/2)));
                                            break;
                                    }
                                    mida = pareMida/2.0;
                                    nivell = pareNivell+1;
                                    //Afegim els elements al nou node
                                    llistaElements = f2->getLlistaElements();
                                    delete f2; f2=NULL;
                                } else {
                                    llistaElements = f2->getLlistaElements();
                                    delete f2; f2=NULL;
                                }
                            }
                            break;
                        case 2:
                            if (nivell!=0) { //Si no es el node arrel
                                //Transformem el node actual com el node fill i eliminem el fill
                                SkipONode *aux = f2;
                                copiarAtributs(f2);
                                aux->setFill1(NULL);
                                aux->setFill2(NULL);
                                aux->setFill3(NULL);
                                aux->setFill4(NULL);
                                aux->setFill5(NULL);
                                aux->setFill6(NULL);
                                aux->setFill7(NULL);
                                aux->setFill8(NULL);
                                delete aux;
                            }
                            break;
                        default:
                            cout << "ERROR (SkipQNode::esborrarElement()) : Refent el node!" << endl;
                            exit(-1);
                            break;
                    }
                    break;
                case 3:
                    switch (f3->tipus())
                    {
                        case 1:
                            //Comprovem quants elements hi ha
                            if (f3->getLlistaElements().size()>1) {
                                //Més d'un element = Nivell Maxim
                                //Transformem el node actual com el node fill i eliminem el fill
                                SkipONode *aux = f3;
                                copiarAtributs(f3);
                                delete aux;
                            } else {
                                if ( (pare!=NULL) && (pare->getNivell()<nivell-1) ){
                                    //Creem un node d'un nivell inferior al pare

                                    double pareX = (pare->getAncoratge()).getX();
                                    double pareY = (pare->getAncoratge()).getY();
                                    double pareZ = (pare->getAncoratge()).getZ();
                                    double pareMida = pare->getMida();
                                    int pareNivell = pare->getNivell();
                                    switch (pare->aQuinFill(f3)) {
                                        case 1:
                                            setAncoratge(pare->getAncoratge());
                                            break;
                                        case 2:
                                            setAncoratge(point3D(pareX+(pareMida/2),pareY,pareZ));
                                            break;
                                        case 3:
                                            setAncoratge(point3D(pareX,pareY-(pareMida/2),pareZ));
                                            break;
                                        case 4:
                                            setAncoratge(point3D(pareX+(pareMida/2),pareY-(pareMida/2),pareZ));
                                            break;
                                        case 5:
                                            setAncoratge(point3D(pareX,pareY,pareZ-(pareMida/2)));
                                            break;
                                        case 6:
                                            setAncoratge(point3D(pareX+(pareMida/2),pareY,pareZ-(pareMida/2)));
                                            break;
                                        case 7:
                                            setAncoratge(point3D(pareX,pareY-(pareMida/2),pareZ-(pareMida/2)));
                                            break;
                                        case 8:
                                            setAncoratge(point3D(pareX+(pareMida/2),pareY-(pareMida/2),pareZ-(pareMida/2)));
                                            break;
                                    }
                                    mida = pareMida/2.0;
                                    nivell = pareNivell+1;
                                    //Afegim els elements al nou node
                                    llistaElements = f3->getLlistaElements();
                                    delete f3; f3=NULL;
                                } else {
                                    llistaElements = f3->getLlistaElements();
                                    delete f3; f3=NULL;
                                }
                            }
                            break;
                        case 2:
                            if (nivell!=0) { //Si no es el node arrel
                                //Transformem el node actual com el node fill i eliminem el fill
                                SkipONode *aux = f3;
                                copiarAtributs(f3);
                                aux->setFill1(NULL);
                                aux->setFill2(NULL);
                                aux->setFill3(NULL);
                                aux->setFill4(NULL);
                                aux->setFill5(NULL);
                                aux->setFill6(NULL);
                                aux->setFill7(NULL);
                                aux->setFill8(NULL);
                                delete aux;
                            }
                            break;
                        default:
                            cout << "ERROR (SkipQNode::esborrarElement()) : Refent el node!" << endl;
                            exit(-1);
                            break;
                    }
                    break;
                case 4:
                    switch (f4->tipus())
                    {
                        case 1:
                            //Comprovem quants elements hi ha
                            if (f4->getLlistaElements().size()>1) {
                                //Més d'un element = Nivell Maxim
                                //Transformem el node actual com el node fill i eliminem el fill
                                SkipONode *aux = f4;
                                copiarAtributs(f4);
                                delete aux;
                            } else {
                                if ( (pare!=NULL) && (pare->getNivell()<nivell-1) ){
                                    //Creem un node d'un nivell inferior al pare

                                    double pareX = (pare->getAncoratge()).getX();
                                    double pareY = (pare->getAncoratge()).getY();
                                    double pareZ = (pare->getAncoratge()).getZ();
                                    double pareMida = pare->getMida();
                                    int pareNivell = pare->getNivell();
                                    switch (pare->aQuinFill(f4)) {
                                        case 1:
                                            setAncoratge(pare->getAncoratge());
                                            break;
                                        case 2:
                                            setAncoratge(point3D(pareX+(pareMida/2),pareY,pareZ));
                                            break;
                                        case 3:
                                            setAncoratge(point3D(pareX,pareY-(pareMida/2),pareZ));
                                            break;
                                        case 4:
                                            setAncoratge(point3D(pareX+(pareMida/2),pareY-(pareMida/2),pareZ));
                                            break;
                                        case 5:
                                            setAncoratge(point3D(pareX,pareY,pareZ-(pareMida/2)));
                                            break;
                                        case 6:
                                            setAncoratge(point3D(pareX+(pareMida/2),pareY,pareZ-(pareMida/2)));
                                            break;
                                        case 7:
                                            setAncoratge(point3D(pareX,pareY-(pareMida/2),pareZ-(pareMida/2)));
                                            break;
                                        case 8:
                                            setAncoratge(point3D(pareX+(pareMida/2),pareY-(pareMida/2),pareZ-(pareMida/2)));
                                            break;
                                    }
                                    mida = pareMida/2.0;
                                    nivell = pareNivell+1;
                                    //Afegim els elements al nou node
                                    llistaElements = f4->getLlistaElements();
                                    delete f4; f4=NULL;
                                } else {
                                    llistaElements = f4->getLlistaElements();
                                    delete f4; f4=NULL;
                                }
                            }
                            break;
                        case 2:
                            if (nivell!=0) { //Si no es el node arrel
                                //Transformem el node actual com el node fill i eliminem el fill
                                SkipONode *aux = f4;
                                copiarAtributs(f4);
                                aux->setFill1(NULL);
                                aux->setFill2(NULL);
                                aux->setFill3(NULL);
                                aux->setFill4(NULL);
                                aux->setFill5(NULL);
                                aux->setFill6(NULL);
                                aux->setFill7(NULL);
                                aux->setFill8(NULL);
                                delete aux;
                            }
                            break;
                        default:
                            cout << "ERROR (SkipQNode::esborrarElement()) : Refent el node!" << endl;
                            exit(-1);
                            break;
                    }
                    break;
                case 5:
                    switch (f5->tipus())
                    {
                        case 1:
                            //Comprovem quants elements hi ha
                            if (f5->getLlistaElements().size()>1) {
                                //Més d'un element = Nivell Maxim
                                //Transformem el node actual com el node fill i eliminem el fill
                                SkipONode *aux = f5;
                                copiarAtributs(f5);
                                delete aux;
                            } else {
                                if ( (pare!=NULL) && (pare->getNivell()<nivell-1) ){
                                    //Creem un node d'un nivell inferior al pare

                                    double pareX = (pare->getAncoratge()).getX();
                                    double pareY = (pare->getAncoratge()).getY();
                                    double pareZ = (pare->getAncoratge()).getZ();
                                    double pareMida = pare->getMida();
                                    int pareNivell = pare->getNivell();
                                    switch (pare->aQuinFill(f5)) {
                                        case 1:
                                            setAncoratge(pare->getAncoratge());
                                            break;
                                        case 2:
                                            setAncoratge(point3D(pareX+(pareMida/2),pareY,pareZ));
                                            break;
                                        case 3:
                                            setAncoratge(point3D(pareX,pareY-(pareMida/2),pareZ));
                                            break;
                                        case 4:
                                            setAncoratge(point3D(pareX+(pareMida/2),pareY-(pareMida/2),pareZ));
                                            break;
                                        case 5:
                                            setAncoratge(point3D(pareX,pareY,pareZ-(pareMida/2)));
                                            break;
                                        case 6:
                                            setAncoratge(point3D(pareX+(pareMida/2),pareY,pareZ-(pareMida/2)));
                                            break;
                                        case 7:
                                            setAncoratge(point3D(pareX,pareY-(pareMida/2),pareZ-(pareMida/2)));
                                            break;
                                        case 8:
                                            setAncoratge(point3D(pareX+(pareMida/2),pareY-(pareMida/2),pareZ-(pareMida/2)));
                                            break;
                                    }
                                    mida = pareMida/2.0;
                                    nivell = pareNivell+1;
                                    //Afegim els elements al nou node
                                    llistaElements = f5->getLlistaElements();
                                    delete f5; f5=NULL;
                                } else {
                                    llistaElements = f5->getLlistaElements();
                                    delete f5; f5=NULL;
                                }
                            }
                            break;
                        case 2:
                            if (nivell!=0) { //Si no es el node arrel
                                //Transformem el node actual com el node fill i eliminem el fill
                                SkipONode *aux = f5;
                                copiarAtributs(f5);
                                aux->setFill1(NULL);
                                aux->setFill2(NULL);
                                aux->setFill3(NULL);
                                aux->setFill4(NULL);
                                aux->setFill5(NULL);
                                aux->setFill6(NULL);
                                aux->setFill7(NULL);
                                aux->setFill8(NULL);
                                delete aux;
                            }
                            break;
                        default:
                            cout << "ERROR (SkipQNode::esborrarElement()) : Refent el node!" << endl;
                            exit(-1);
                            break;
                    }
                    break;
                case 6:
                    switch (f6->tipus())
                    {
                        case 1:
                            //Comprovem quants elements hi ha
                            if (f6->getLlistaElements().size()>1) {
                                //Més d'un element = Nivell Maxim
                                //Transformem el node actual com el node fill i eliminem el fill
                                SkipONode *aux = f6;
                                copiarAtributs(f6);
                                delete aux;
                            } else {
                                if ( (pare!=NULL) && (pare->getNivell()<nivell-1) ){
                                    //Creem un node d'un nivell inferior al pare

                                    double pareX = (pare->getAncoratge()).getX();
                                    double pareY = (pare->getAncoratge()).getY();
                                    double pareZ = (pare->getAncoratge()).getZ();
                                    double pareMida = pare->getMida();
                                    int pareNivell = pare->getNivell();
                                    switch (pare->aQuinFill(f6)) {
                                        case 1:
                                            setAncoratge(pare->getAncoratge());
                                            break;
                                        case 2:
                                            setAncoratge(point3D(pareX+(pareMida/2),pareY,pareZ));
                                            break;
                                        case 3:
                                            setAncoratge(point3D(pareX,pareY-(pareMida/2),pareZ));
                                            break;
                                        case 4:
                                            setAncoratge(point3D(pareX+(pareMida/2),pareY-(pareMida/2),pareZ));
                                            break;
                                        case 5:
                                            setAncoratge(point3D(pareX,pareY,pareZ-(pareMida/2)));
                                            break;
                                        case 6:
                                            setAncoratge(point3D(pareX+(pareMida/2),pareY,pareZ-(pareMida/2)));
                                            break;
                                        case 7:
                                            setAncoratge(point3D(pareX,pareY-(pareMida/2),pareZ-(pareMida/2)));
                                            break;
                                        case 8:
                                            setAncoratge(point3D(pareX+(pareMida/2),pareY-(pareMida/2),pareZ-(pareMida/2)));
                                            break;
                                    }
                                    mida = pareMida/2.0;
                                    nivell = pareNivell+1;
                                    //Afegim els elements al nou node
                                    llistaElements = f6->getLlistaElements();
                                    delete f6; f6=NULL;
                                } else {
                                    llistaElements = f6->getLlistaElements();
                                    delete f6; f6=NULL;
                                }
                            }
                            break;
                        case 2:
                            if (nivell!=0) { //Si no es el node arrel
                                //Més d'un element = Nivell Maxim
                                //Transformem el node actual com el node fill i eliminem el fill
                                SkipONode *aux = f6;
                                copiarAtributs(f6);
                                aux->setFill1(NULL);
                                aux->setFill2(NULL);
                                aux->setFill3(NULL);
                                aux->setFill4(NULL);
                                aux->setFill5(NULL);
                                aux->setFill6(NULL);
                                aux->setFill7(NULL);
                                aux->setFill8(NULL);
                                delete aux;
                            }
                            break;
                        default:
                            cout << "ERROR (SkipQNode::esborrarElement()) : Refent el node!" << endl;
                            exit(-1);
                            break;
                    }
                    break;
                case 7:
                    switch (f7->tipus())
                    {
                        case 1:
                            //Comprovem quants elements hi ha
                            if (f7->getLlistaElements().size()>1) {
                                //Més d'un element = Nivell Maxim
                                //Transformem el node actual com el node fill i eliminem el fill
                                SkipONode *aux = f7;
                                copiarAtributs(f7);
                                delete aux;
                            } else {
                                if ( (pare!=NULL) && (pare->getNivell()<nivell-1) ){
                                    //Creem un node d'un nivell inferior al pare

                                    double pareX = (pare->getAncoratge()).getX();
                                    double pareY = (pare->getAncoratge()).getY();
                                    double pareZ = (pare->getAncoratge()).getZ();
                                    double pareMida = pare->getMida();
                                    int pareNivell = pare->getNivell();
                                    switch (pare->aQuinFill(f7)) {
                                        case 1:
                                            setAncoratge(pare->getAncoratge());
                                            break;
                                        case 2:
                                            setAncoratge(point3D(pareX+(pareMida/2),pareY,pareZ));
                                            break;
                                        case 3:
                                            setAncoratge(point3D(pareX,pareY-(pareMida/2),pareZ));
                                            break;
                                        case 4:
                                            setAncoratge(point3D(pareX+(pareMida/2),pareY-(pareMida/2),pareZ));
                                            break;
                                        case 5:
                                            setAncoratge(point3D(pareX,pareY,pareZ-(pareMida/2)));
                                            break;
                                        case 6:
                                            setAncoratge(point3D(pareX+(pareMida/2),pareY,pareZ-(pareMida/2)));
                                            break;
                                        case 7:
                                            setAncoratge(point3D(pareX,pareY-(pareMida/2),pareZ-(pareMida/2)));
                                            break;
                                        case 8:
                                            setAncoratge(point3D(pareX+(pareMida/2),pareY-(pareMida/2),pareZ-(pareMida/2)));
                                            break;
                                    }
                                    mida = pareMida/2.0;
                                    nivell = pareNivell+1;
                                    //Afegim els elements al nou node
                                    llistaElements = f7->getLlistaElements();
                                    delete f7; f7=NULL;
                                } else {
                                    llistaElements = f7->getLlistaElements();
                                    delete f7; f7=NULL;
                                }
                            }
                            break;
                        case 2:
                            if (nivell!=0) { //Si no es el node arrel
                                //Transformem el node actual com el node fill i eliminem el fill
                                SkipONode *aux = f7;
                                copiarAtributs(f7);
                                aux->setFill1(NULL);
                                aux->setFill2(NULL);
                                aux->setFill3(NULL);
                                aux->setFill4(NULL);
                                aux->setFill5(NULL);
                                aux->setFill6(NULL);
                                aux->setFill7(NULL);
                                aux->setFill8(NULL);
                                delete aux;
                            }
                            break;
                        default:
                            cout << "ERROR (SkipQNode::esborrarElement()) : Refent el node!" << endl;
                            exit(-1);
                            break;
                    }
                    break;
                case 8:
                    switch (f8->tipus())
                    {
                        case 1:
                            //Comprovem quants elements hi ha
                            if (f8->getLlistaElements().size()>1) {
                                //Més d'un element = Nivell Maxim
                                //Transformem el node actual com el node fill i eliminem el fill
                                SkipONode *aux = f8;
                                copiarAtributs(f8);
                                delete aux;
                            } else {
                                if ( (pare!=NULL) && (pare->getNivell()<nivell-1) ){
                                    //Creem un node d'un nivell inferior al pare

                                    double pareX = (pare->getAncoratge()).getX();
                                    double pareY = (pare->getAncoratge()).getY();
                                    double pareZ = (pare->getAncoratge()).getZ();
                                    double pareMida = pare->getMida();
                                    int pareNivell = pare->getNivell();
                                    switch (pare->aQuinFill(f8)) {
                                        case 1:
                                            setAncoratge(pare->getAncoratge());
                                            break;
                                        case 2:
                                            setAncoratge(point3D(pareX+(pareMida/2),pareY,pareZ));
                                            break;
                                        case 3:
                                            setAncoratge(point3D(pareX,pareY-(pareMida/2),pareZ));
                                            break;
                                        case 4:
                                            setAncoratge(point3D(pareX+(pareMida/2),pareY-(pareMida/2),pareZ));
                                            break;
                                        case 5:
                                            setAncoratge(point3D(pareX,pareY,pareZ-(pareMida/2)));
                                            break;
                                        case 6:
                                            setAncoratge(point3D(pareX+(pareMida/2),pareY,pareZ-(pareMida/2)));
                                            break;
                                        case 7:
                                            setAncoratge(point3D(pareX,pareY-(pareMida/2),pareZ-(pareMida/2)));
                                            break;
                                        case 8:
                                            setAncoratge(point3D(pareX+(pareMida/2),pareY-(pareMida/2),pareZ-(pareMida/2)));
                                            break;
                                    }
                                    mida = pareMida/2.0;
                                    nivell = pareNivell+1;
                                    //Afegim els elements al nou node
                                    llistaElements = f8->getLlistaElements();
                                    delete f8; f8=NULL;
                                } else {
                                    llistaElements = f8->getLlistaElements();
                                    delete f8; f8=NULL;
                                }
                            }
                            break;
                        case 2:
                            if (nivell!=0) { //Si no es el node arrel
                                //Transformem el node actual com el node fill i eliminem el fill
                                SkipONode *aux = f8;
                                copiarAtributs(f8);
                                aux->setFill1(NULL);
                                aux->setFill2(NULL);
                                aux->setFill3(NULL);
                                aux->setFill4(NULL);
                                aux->setFill5(NULL);
                                aux->setFill6(NULL);
                                aux->setFill7(NULL);
                                aux->setFill8(NULL);
                                delete aux;
                            }
                            break;
                        default:
                            cout << "ERROR (SkipQNode::esborrarElement()) : Refent el node!" << endl;
                            exit(-1);
                            break;
                    }
                    break;
            }
            break;
    }
}

//Retorna el tipus de node que es (0: Fulla blanca, 1: Fulla negre, 2: Node parcial)
//Precondicio: El node es no null.
int SkipONode::tipus()
{
    // Retorna:
    // 0 : fulla blanca
    // 1 : fulla negre
    // 2 : node parcial

    if (this == NULL)
    {
        cout << "ERROR a SkipONode::tipus() : Node nul!" << endl;
        exit(-1);
    }
    else
    {
        if ( (f1 != NULL) || (f2 != NULL) || (f3 != NULL) || (f4 != NULL) || (f5 != NULL) || (f6 != NULL) || (f7 != NULL) || (f8 != NULL) ) {
            return 2;
        } else {
            if ( llistaElements.empty() )
            {
                return 0;
            } else {
                return 1;
            }
        }
    }
}

bool SkipONode::esFulla()
{
    return ( (tipus()==0)||(tipus()==1)  );
}

bool SkipONode::nodeBuit()
{
    return ( (f1==NULL)&&(f2==NULL)&&(f3==NULL)&&(f4==NULL)&&(f5==NULL)&&(f6==NULL)&&(f7==NULL)&&(f8==NULL)&&(llistaElements.size()==0) );
}

//Crea els fills d'un node a partir dels 2 elements que hi ha dins d'ell
int SkipONode::crearFills(int nivellMaxim)
{
    //Hem de crear els fills del node depenent dels 2 elements que te
    //Per fer-ho, primer mirem a quins fills han d'anar cada un dels dos elements:
    //--> Si els elements van a 2 fills diferents, simplement creem aquests 2 fills
    //--> Altrament (van al mateix fill), hem d'anar saltant de nivell fins que vagin a dos
    //	fills diferents o arribem al nivell maxim del quadtree
    int res;

    int fill1 = aQuinFill(llistaElements[0]);
    int fill2 = aQuinFill(llistaElements[1]);

    while ( (fill1==fill2) && (nivell<nivellMaxim) && (nivell>0) ) { //Mentre els 2 elements hagin d'anar al mateix fill
        // YY no estiguem al nivell maxim -1 (posem el -1 perque quan afegim els 2 elements baixarem
        // un altre nivell) YY el nivell no sigui 0 (ja que si es l'arrel no li canviarem la seva mida)

        //Primer canviem el punt d'ancoratge (basePoint)
        switch (fill1) { //Mirem a quin fill corresponen i baixem de nivell
            //case 1: No s'ha de fer res ja que el punt no varia
            case 2:
                ancoratge.setX( ancoratge.getX()+(mida/2) );
                break;
            case 3:
                ancoratge.setY( ancoratge.getY()-(mida/2) );
                break;
            case 4:
                ancoratge.setX( ancoratge.getX()+(mida/2) );
                ancoratge.setY( ancoratge.getY()-(mida/2) );
                break;
            case 5:
                ancoratge.setZ( ancoratge.getZ()-(mida/2) );
                break;
            case 6:
                ancoratge.setX( ancoratge.getX()+(mida/2) );
                ancoratge.setZ( ancoratge.getZ()-(mida/2) );
                break;
            case 7:
                ancoratge.setY( ancoratge.getY()-(mida/2) );
                ancoratge.setZ( ancoratge.getZ()-(mida/2) );
                break;
            case 8:
                ancoratge.setX( ancoratge.getX()+(mida/2) );
                ancoratge.setY( ancoratge.getY()-(mida/2) );
                ancoratge.setZ( ancoratge.getZ()-(mida/2) );
                break;
        }
        mida = mida/2; //La mida del node sera la meitat
        nivell++; //El node tindra un nivell mes

        //Actualitzem el fill que han d'anar els elements
        fill1 = aQuinFill(llistaElements[0]);
        fill2 = aQuinFill(llistaElements[1]);

    }

    if (nivell<nivellMaxim) {
        //Creem els 2 fills i hi posem l'element corresponent a cada un
        // --> El metode addToCorrespondingSonSkip ja crea els fills si aquests valen NULL
        afegirElementFillCorresponent(llistaElements[0],nivellMaxim);
        res = afegirElementFillCorresponent(llistaElements[1],nivellMaxim);
    } else {
        //No cal fer res perque estem al nivell maxim
        res = nivell;
    }

    return res;
}

//Retorna cert si el node conté l'element e
bool SkipONode::pertany(Element *e)
{
    //cout << "\t dins el pertany, num elements: " << llistaElements.size() << endl;
    vector<Element*>::iterator i = llistaElements.end();
    while (i!=llistaElements.begin())
    {
        //cout << "\tcomparem amb una esfera" << endl;
        i--;
        if( ((*i)->getRadi()==e->getRadi()) && ( (*i)->getPoint()==e->getPoint() ) )
        {
            //cout << "\the trobat una element igual!! Retorno true!" << endl;
            return true;
        }
    }
    //cout << "\tNo he trobat ningú, marxo amb false" << endl;
    return false;
}

//Retorna un vector de booleans indicant a quins fills pot estar l'element e
vector<bool> SkipONode::aQuinsFills(Element *e)
{
    vector<bool> res = vector<bool>(8,false);
    double tolerancia = TOLERANCIA_DETECTAR_PERILL_A_ESBORRAR_NODE;
    point3D centre = e->getPoint();

    if ( centre.getZ() > (ancoratge.getZ()-(mida/2.0)-tolerancia) ) { //Pertany a f1,f2,f3 o f4
        if ( centre.getY() > (ancoratge.getY()-(mida/2.0)-tolerancia) ) { //Pertany a f1 o f2 
            if ( centre.getX() < (ancoratge.getX()+(mida/2.0)+tolerancia) ) { //Pertany a f1
                res[0] = true;
            }
            if ( centre.getX() > (ancoratge.getX()+(mida/2.0)-tolerancia) ) { //Pertany a f2
                res[1] = true;
            }
        }
        if ( centre.getY() < (ancoratge.getY()-(mida/2.0)+tolerancia) ) { //Pertany a f3 o f4
            if ( centre.getX() < (ancoratge.getX()+(mida/2.0)+tolerancia) ) { //Pertany a f3
                res[2] = true;
            }
            if ( centre.getX() > (ancoratge.getX()+(mida/2.0)-tolerancia) ) { //Pertany a f4
                res[3] = true;
            }
        }
    }

    if ( centre.getZ() < (ancoratge.getZ()-(mida/2.0)+tolerancia) ) { //Pertany a f5,f6,f7 o f8
        if ( centre.getY() > (ancoratge.getY()-(mida/2.0)-tolerancia) ) { //Pertany a f5 o f6
            if ( centre.getX() < (ancoratge.getX()+(mida/2.0)+tolerancia) ) { //Pertany a f5
                res[4] = true;
            }
            if ( centre.getX() > (ancoratge.getX()+(mida/2.0)-tolerancia) ) { //Pertany a f6
                res[5] = true;
            }
        }
        if ( centre.getY() < (ancoratge.getY()-(mida/2.0)+tolerancia) ) { //Pertany a f7 o f8
            if ( centre.getX() < (ancoratge.getX()+(mida/2.0)+tolerancia) ) { //Pertany a f7
                res[6] = true;
            }
            if ( centre.getX() > (ancoratge.getX()+(mida/2.0)-tolerancia) ) { //Pertany a f8
                res[7] = true;
            }
        }
    }
    return res;
}

//Retorna el numero de fill que pertany l'element e
int SkipONode::aQuinFill(Element *e)
{
    int res = 0;
    point3D centre = e->getPoint();
    if ( centre.getZ() > (ancoratge.getZ()-(mida/2)) ) { //Pertany a f1,f2,f3 o f4
        if ( centre.getY() > (ancoratge.getY()-(mida/2)) ) { //Pertany a f1 o f2
            if ( centre.getX() < (ancoratge.getX()+(mida/2)) ) { //Pertany a f1
                return 1;
            } else { //Pertany a f2
                return 2;
            }
        } else { //Pertany a f3 o f4
            if ( centre.getX() < (ancoratge.getX()+(mida/2)) ) { //Pertany a f3
                return 3;
            } else { //Pertany a f4
                return 4;
            }
        }
    } else { //Pertany a f5,f6,f7 o f8
        if ( centre.getY() > (ancoratge.getY()-(mida/2)) ) { //Pertany a f5 o f6
            if ( centre.getX() < (ancoratge.getX()+(mida/2)) ) { //Pertany a f5
                return 5;
            } else { //Pertany a f6
                return 6;
            }
        } else { //Pertany a f7 o f8
            if ( centre.getX() < (ancoratge.getX()+(mida/2)) ) { //Pertany a f7
                return 7;
            } else { //Pertany a f8
                return 8;
            }
        }
    }
    return res;
}

//Retorna el numero de fill que pertany el node n
int SkipONode::aQuinFill(SkipONode *n)
{
    int res = 0;
    point3D ancora = n->getAncoratge();
    double ancoraX = ancora.getX()+TOLERANCIA_METODE_A_QUIN_FILL;
    double ancoraY = ancora.getY()-TOLERANCIA_METODE_A_QUIN_FILL;
    double ancoraZ = ancora.getZ()-TOLERANCIA_METODE_A_QUIN_FILL;

    if ( ancoraZ > (ancoratge.getZ()-(mida/2.)) ) { //Pertany a f1,f2,f3 o f4
        if ( ancoraY > (ancoratge.getY()-(mida/2.)) ) { //Pertany a f1 o f2
            if ( ancoraX < (ancoratge.getX()+(mida/2.)) ) { //Pertany a f1
                return 1;
            } else { //Pertany a f2
                return 2;
            }
        } else { //Pertany a f3 o f4
            if ( ancoraX < (ancoratge.getX()+(mida/2.)) ) { //Pertany a f3
                return 3;
            } else { //Pertany a f4
                return 4;
            }
        }
    } else { //Pertany a f5,f6,f7 o f8
        if ( ancoraY > (ancoratge.getY()-(mida/2.)) ) { //Pertany a f5 o f6
            if ( ancoraX < (ancoratge.getX()+(mida/2.)) ) { //Pertany a f5
                return 5;
            } else { //Pertany a f6
                return 6;
            }
        } else { //Pertany a f7 o f8
            if ( ancoraX < (ancoratge.getX()+(mida/2.)) ) { //Pertany a f7
                return 7;
            } else { //Pertany a f8
                return 8;
            }
        }
    }
    return res;
}

int SkipONode::fillNoFullaBlanca()
{
    //Retorna 0 si hi ha més d'un fill que no és una fulla blanca.
    //Altrament, retorna el numero de fill que és fulla blanca.
    int res = -1;
    int noFullesBlanques = 0;

    if ( (f1!=NULL) && (f1->tipus()!=0) ) { //El fill 1 no es fulla blanca
        res = 1;
        noFullesBlanques++;
    }

    if ( (f2!=NULL) && (f2->tipus()!=0) ) { //El fill 2 no es fulla blanca
        res = 2;
        noFullesBlanques++;
    }

    if ( (f3!=NULL) && (f3->tipus()!=0) ) { //El fill 3 no es fulla blanca
        res = 3;
        noFullesBlanques++;
    }

    if ( (f4!=NULL) && (f4->tipus()!=0) ) { //El fill 4 no es fulla blanca
        res = 4;
        noFullesBlanques++;
    }

    if ( (f5!=NULL) && (f5->tipus()!=0) ) { //El fill 5 no es fulla blanca
        res = 5;
        noFullesBlanques++;
    }

    if ( (f6!=NULL) && (f6->tipus()!=0) ) { //El fill 6 no es fulla blanca
        res = 6;
        noFullesBlanques++;
    }

    if ( (f7!=NULL) && (f7->tipus()!=0) ) { //El fill 7 no es fulla blanca
        res = 7;
        noFullesBlanques++;
    }

    if ( (f8!=NULL) && (f8->tipus()!=0) ) { //El fill 8 no es fulla blanca
        res = 8;
        noFullesBlanques++;
    }

    switch (noFullesBlanques) {
        case 0: //Tots els fills son fulles blanques
            //Hi ha algun problema que s'havia d'haver resolt abans
            cout << "ERROR (SkipONode::fillNoFullaBlanca() : Tots els fills son fulles blanques!" << endl;
            break;
        case 1: //Hi ha nomes un fill no fulla blanca
            //No cal fer res
            break;
        default: //Hi ha mes d'un fill no fulla blanca
            res = 0;
            break;
    }

    return res;
}

int SkipONode::afegirElementFillCorresponent(Element *e,int nivellMaxim)
{
    int res = 0;

    double nouX = ancoratge.getX();
    double nouY = ancoratge.getY();
    double nouZ = ancoratge.getZ();
    point3D centre = e->getPoint();
    if ( centre.getZ() > (ancoratge.getZ()-(mida/2.)) ) { //Pertany a f1,f2,f3 o f4
        if ( centre.getY() > (ancoratge.getY()-(mida/2.)) ) { //Pertany a f1 o f2
            if ( centre.getX() < (ancoratge.getX()+(mida/2.)) ) { //Pertany a f1
                if (f1 == NULL) f1 = new SkipONode(ancoratge,mida/2.,nivell+1);
                res = f1->afegirElement(e,nivellMaxim,this);
            } else { //Pertany a f2
                if (f2 == NULL) f2 = new SkipONode(point3D(nouX+(mida/2.),nouY,nouZ),mida/2.,nivell+1);
                res = f2->afegirElement(e,nivellMaxim,this);
            }
        } else { //Pertany a f3 o f4
            if ( centre.getX() < (ancoratge.getX()+(mida/2.)) ) { //Pertany a f3
                if (f3 == NULL) f3 = new SkipONode(point3D(nouX,nouY-(mida/2.),nouZ),mida/2.,nivell+1);
                res = f3->afegirElement(e,nivellMaxim,this);
            } else { //Pertany a f4
                if (f4 == NULL) f4 = new SkipONode(point3D(nouX+(mida/2.),nouY-(mida/2.),nouZ),mida/2.,nivell+1);
                res = f4->afegirElement(e,nivellMaxim,this);
            }
        }
    } else { //Pertany a f5,f6,f7 o f8
        if ( centre.getY() > (ancoratge.getY()-(mida/2.)) ) { //Pertany a f5 o f6
            if ( centre.getX() < (ancoratge.getX()+(mida/2.)) ) { //Pertany a f5
                if (f5 == NULL) f5 = new SkipONode(point3D(nouX,nouY,nouZ-(mida/2.)),mida/2.,nivell+1);
                res = f5->afegirElement(e,nivellMaxim,this);
            } else { //Pertany a f6
                if (f6 == NULL) f6 = new SkipONode(point3D(nouX+(mida/2.),nouY,nouZ-(mida/2.)),mida/2.,nivell+1);
                res = f6->afegirElement(e,nivellMaxim,this);
            }
        } else { //Pertany a f7 o f8
            if ( centre.getX() < (ancoratge.getX()+(mida/2.)) ) { //Pertany a f7
                if (f7 == NULL) f7 = new SkipONode(point3D(nouX,nouY-(mida/2.),nouZ-(mida/2.)),mida/2.,nivell+1);
                res = f7->afegirElement(e,nivellMaxim,this);
            } else { //Pertany a f8
                if (f8 == NULL) f8 = new SkipONode(point3D(nouX+(mida/2.),nouY-(mida/2.),nouZ-(mida/2.)),mida/2.,nivell+1);
                res = f8->afegirElement(e,nivellMaxim,this);
            }
        }
    }
    return res;
}

bool SkipONode::formaPart(point3D p)
{
//	cout<<"SkipONode::formaPart punt "<<p<<" mida "<<mida<<endl;
//	cout<<"SkipONode::formaPart extrems X (major) "<< ancoratge.getX()<<" i "<<ancoratge.getX()+mida<<endl;
//	cout<<"SkipONode::formaPart extrems Y (menor) "<< ancoratge.getY()<<" i "<<ancoratge.getY()-mida<<endl;
//	cout<<"SkipONode::formaPart extrems Z (menor) "<< ancoratge.getZ()<<" i "<<ancoratge.getZ()-mida<<endl;

    if ( (p.getX() >= ancoratge.getX()) && (p.getX() < ancoratge.getX()+mida) &&
         (p.getY() <= ancoratge.getY()) && (p.getY() > ancoratge.getY()-mida) &&
         (p.getZ() <= ancoratge.getZ()) && (p.getZ() > ancoratge.getZ()-mida) ) {
        return true;
    } else {
        return false;
    }
}

//Retorna la llista d'elements que formen part de la circumferencia "e-eps"
list<Element*> * SkipONode::weightedNeighbors(Element *e,double eps)
{
    list<Element*> *ret = NULL;
    point3D p = e->getPoint();


    //cout<<"SkipONode::weightedNeighbors, entro al node amb punt d'ancoratge "<<ancoratge<<" de nivell "<<nivell<<" i mida: "<<mida<<" estic buscant veins de  "<<e->getPoint()<<" a distancia "<<eps<<" amb punts aqui dintre: "<<endl;
    //vector<Element *>::iterator it; 	
    //for(it=llistaElements.begin();it!=llistaElements.end();it++)
    //{
    //	cout<<*(*it)<<endl;
    //}

//	if(stabbing(e,eps) == stabbingSimple(e, eps)) cout << "equal" << endl;
//	else cout << "different" << endl;


    // si estem en un nivell de l'octree on n'hi ha prou pocs, passem de seguir buscant.
    if(llistaElements.size()<MAX_ELEMENTS_SEGUIR_BUSCANT)
    {
        //cout<<"SkipONode::weightedNeighbors, passsant de l'octree!!!!"<<endl;
        ret=reportIf(e,eps);
    }
    else
    {

//		if(contained(e,eps))
        if(INSIDE == checkIntersection(p.getX()-eps, p.getX()+eps,
                                       p.getY()-eps, p.getY()+eps,
                                       p.getZ()-eps, p.getZ()+eps))
        {
            //cout<<"SkipONode::weightedNeighbors	contained "<<endl;
            ret = report(e);
        }
        else if(stabbingSimple(e,eps))
        {
//			cout<<"SkipONode::weightedNeighbors	stabbing "<<endl;
            if(tipus()==1)//Black leaf
            {
                //cout<<"SkipONode::weightedNeighbors	stabbing, black leaf "<<endl;

                ret=reportIf(e,eps);
            }
            else if (tipus()==2)//Non-leaf node
            {
                list<Element*> * retAux = NULL;
                ret = new list<Element*>();
                //cout<<"SkipONode::weightedNeighbors	stabbing, non leaf "<<f1<<f2<<f3<<f4<<f5<<f6<<f7<<f8<<endl;

                //Recursive call
                if (f1!=NULL){
                    retAux=f1->weightedNeighbors(e,eps);
                    if(retAux!=NULL) {
                        ret->splice(ret->end(), *retAux);
                    }
                }

                if (f2!=NULL) {
                    retAux=f2->weightedNeighbors(e,eps);
                    if(retAux!=NULL) {
                        ret->splice(ret->end(), *retAux);
                    }				}

                if (f3!=NULL) {
                    retAux=f3->weightedNeighbors(e,eps);
                    if(retAux!=NULL) {
                        ret->splice(ret->end(), *retAux);
                    }				}

                if (f4!=NULL) {
                    retAux=f4->weightedNeighbors(e,eps);
                    if(retAux!=NULL) {
                        ret->splice(ret->end(), *retAux);
                    }				}

                if (f5!=NULL) {
                    retAux=f5->weightedNeighbors(e,eps);
                    if(retAux!=NULL) {
                        ret->splice(ret->end(), *retAux);
                    }				}

                if (f6!=NULL) {
                    retAux=f6->weightedNeighbors(e,eps);
                    if(retAux!=NULL) {
                        ret->splice(ret->end(), *retAux);
                    }				}

                if (f7!=NULL) {
                    retAux=f7->weightedNeighbors(e,eps);
                    if(retAux!=NULL) {
                        ret->splice(ret->end(), *retAux);
                    }				}

                if (f8!=NULL) {
                    retAux=f8->weightedNeighbors(e,eps);
                    if(retAux!=NULL) {
                        ret->splice(ret->end(), *retAux);
                    }				}
                //retAux.clear();
                delete retAux;
            }
        }
        //else
        //{
        //cout<<"SkipONode::weightedNeighbors	ni contained ni stabbing "<<endl;
        //}
    }

//	if(ret==NULL){
//		cout << tipus() << endl;
//		cout << "soc nul" << endl;
//	}

    return ret;
}
//



//Retorna cert si el node forma part de la circumferencia "e-r", altrament fals
bool SkipONode::contained(Element *e,double r)
{
    bool b;

    point3D p=e->getPoint();
    point3D p1,p2,p3,p4,p5,p6,p7,p8;

    //Agafem els 8 punts extrems
    p1 = point3D(ancoratge.getX(),ancoratge.getY(),ancoratge.getZ());
    p2 = point3D(ancoratge.getX()+mida,ancoratge.getY(),ancoratge.getZ());
    p3 = point3D(ancoratge.getX(),ancoratge.getY()-mida,ancoratge.getZ());
    p4 = point3D(ancoratge.getX()+mida,ancoratge.getY()-mida,ancoratge.getZ());
    p5 = point3D(ancoratge.getX(),ancoratge.getY(),ancoratge.getZ()-mida);
    p6 = point3D(ancoratge.getX()+mida,ancoratge.getY(),ancoratge.getZ()-mida);
    p7 = point3D(ancoratge.getX(),ancoratge.getY()-mida,ancoratge.getZ()-mida);
    p8 = point3D(ancoratge.getX()+mida,ancoratge.getY()-mida,ancoratge.getZ()-mida);

    //Mirem si cauen tots vuit a dins
    b= ( (p.dist(p1)<=r) && (p.dist(p2)<r) && (p.dist(p3)<r) && (p.dist(p4)<r)
         && (p.dist(p5)<r) && (p.dist(p6)<r) && (p.dist(p7)<r) && (p.dist(p8)<r) );

    return b;
}

// Be carefull! This concept is invers! We gonna check if the current node is inside the ball!!!
bool SkipONode::stabbingSimple(Element *e, double r){

    point3D p = e->getPoint();


    double xmin = p.getX() - r;
    double xmax = p.getX() + r;
    double ymin = p.getY() - r;
    double ymax = p.getY() + r;
    double zmin = p.getZ() - r;
    double zmax = p.getZ() + r;


    // Check if the current node is INSIDE, INTERSECT or CONTAINS the ball of radius r centered at p.
    int res = this->checkIntersection(xmin, xmax, ymin, ymax, zmin, zmax);

    if ( res == INSIDE || res == INTERSECT || res == CONTAINS ){

        return true;
    }
    else{
        return false;
    }
}

// Check intersection between parameter point and the current node.
int SkipONode:: checkIntersection(double p_xmin, double p_xmax,
                                  double p_ymin, double p_ymax,
                                  double p_zmin, double p_zmax) {

    int stateX = -1;
    int stateY = -1;
    int stateZ = -1;


    // This distribution is used to use the same "point distribution" of Skip octree class.
    double xmin = ancoratge.getX();
    double ymin = ancoratge.getY()-mida;
    double zmin = ancoratge.getZ()-mida;
    double xmax = ancoratge.getX()+mida;
    double ymax = ancoratge.getY();
    double zmax = ancoratge.getZ();

//	cout << "node" << endl;
//	cout << xmin << " " << ymin << " " << zmin << endl;
//	cout << xmax << " " << ymax << " " << zmax << endl;
//	cout << "point" << endl;
//	cout << p_xmin << " " << p_ymin << " " << p_zmin << endl;
//	cout << p_xmax << " " << p_ymax << " " << p_zmax << endl;

    // Find the state for each axis.
    // Current node is inside the ball of radius r centered at p.
    if(p_xmin <= xmin && xmax <= p_xmax) stateX = INSIDE;
    if(p_ymin <= ymin && ymax <= p_ymax) stateY = INSIDE;
    if(p_zmin <= zmin && zmax <= p_zmax) stateZ = INSIDE;

    // Current node contain the ball of radius r centered at p.
    if(xmin < p_xmin && p_xmax < xmax) stateX = CONTAINS;
    if(ymin < p_ymin && p_ymax < ymax) stateY = CONTAINS;
    if(zmin < p_zmin && p_zmax < zmax) stateZ = CONTAINS;

    // Current node is outside of the ball of radius r centered at p.
    if ((xmax < p_xmin) || (p_xmax < xmin)) stateX = OUTSIDE;
    if ((ymax < p_ymin) || (p_ymax < ymin)) stateY = OUTSIDE;
    if ((zmax < p_zmin) || (p_zmax < zmin)) stateZ = OUTSIDE;

    if ((xmin < p_xmin && p_xmin < xmax && xmax < p_xmax) || (p_xmin < xmin && xmin < p_xmax && p_xmax < xmax)) stateX = INTERSECT;
    if ((ymin < p_ymin && p_ymin < ymax && ymax < p_ymax) || (p_ymin < ymin && ymin < p_ymax && p_ymax < ymax)) stateY = INTERSECT;
    if ((zmin < p_zmin && p_zmin < zmax && zmax < p_zmax) || (p_zmin < zmin && zmin < p_zmax && p_zmax < zmax)) stateZ = INTERSECT;

//	cout << "Axis states: " << stateX << " " << stateY << " " << stateZ << endl;

    // MERGING STATES
    if(stateX==OUTSIDE || stateY==OUTSIDE || stateZ==OUTSIDE) return OUTSIDE;

    // a partir d'aquí és impossible que un eix estigui fora.
    if(stateX==INTERSECT || stateY==INTERSECT || stateZ==INTERSECT) return INTERSECT;

    // A partir d'aquí només pot quedar INSIDE i CONTAINS
    // If one of those are different from each other the node must INTERSECT.
    if(stateX != stateY || stateY != stateZ || stateZ != stateX) return INTERSECT;

    if(stateX==INSIDE && stateY==INSIDE && stateZ==INSIDE) return INSIDE;

    if(stateX==CONTAINS && stateY==CONTAINS && stateZ==CONTAINS) return CONTAINS;

//	cout << "BOOOOOOX. " << endl;

    cerr << "CheckIntersection::Something wrong happens. I can't compute the intersection!!" << endl;
    exit(EXIT_FAILURE);

    return -1;
}

//Retorna cert si alguna part del node forma part de la circumferencia "e-r", altrament fals
// pre: Contained es fals
bool SkipONode::stabbing(Element *e, double r)
{
    bool b;


    point3D p = e->getPoint();

    point3D p1,p2,p3,p4,p5,p6,p7,p8;
    point3D pc1,pc2,pc3,pc4,pc5,pc6;

    double ancX = ancoratge.getX();
    double ancY = ancoratge.getY();
    double ancZ = ancoratge.getZ();

    //Agafem els 8 punts extrems del node
    p1 = point3D(ancX,ancY,ancZ);
    p2 = point3D(ancX+mida,ancY,ancZ);
    p3 = point3D(ancX,ancY-mida,ancZ);
    p4 = point3D(ancX+mida,ancY-mida,ancZ);
    p5 = point3D(ancX,ancY,ancZ-mida);
    p6 = point3D(ancX+mida,ancY,ancZ-mida);
    p7 = point3D(ancX,ancY-mida,ancZ-mida);
    p8 = point3D(ancX+mida,ancY-mida,ancZ-mida);


    //Mirem si algun cau a dins
    b = ( (p.dist(p1)<=r) || (p.dist(p2)<r) || (p.dist(p3)<r) || (p.dist(p4)<r) ||
          (p.dist(p5)<r) || (p.dist(p6)<r) || (p.dist(p7)<r) || (p.dist(p8)<r) );


//	cout<<" SkipONode::stabbing punts extrems i distancies: p1 "<<p1<<" dist: "<< p.dist(p1) <<" r: "<<r<<endl;
//	cout<<" SkipONode::stabbing punts extrems i distancies: p2 "<<p2<<" dist: "<< p.dist(p2) <<" r: "<<r<<endl;
//	cout<<" SkipONode::stabbing punts extrems i distancies: p3 "<<p3<<" dist: "<< p.dist(p3) <<" r: "<<r<<endl;
//	cout<<" SkipONode::stabbing punts extrems i distancies: p4 "<<p4<<" dist: "<< p.dist(p4) <<" r: "<<r<<endl;
//	cout<<" SkipONode::stabbing punts extrems i distancies: p5 "<<p5<<" dist: "<< p.dist(p5) <<" r: "<<r<<endl;
//	cout<<" SkipONode::stabbing punts extrems i distancies: p6 "<<p6<<" dist: "<< p.dist(p6) <<" r: "<<r<<endl;
//	cout<<" SkipONode::stabbing punts extrems i distancies: p7 "<<p7<<" dist: "<< p.dist(p7) <<" r: "<<r<<endl;
//	cout<<" SkipONode::stabbing punts extrems i distancies: p8 "<<p8<<" dist: "<< p.dist(p8) <<" r: "<<r<<endl;

//	cout<<" SkipONode::stabbing algu a dins de la query region? "<<b<<endl;

//	cout << "node" << endl;
//	cout << ancoratge.getX() << " " << ancoratge.getY() << " " << ancoratge.getZ() << endl;
//	cout << ancoratge.getX()+mida << " " << ancoratge.getY()+mida  << " " << ancoratge.getZ()+mida  << endl;


    //Mirem tambe si el query point esta a dins del node
    b = b || formaPart(p);

    //cout<<" SkipONode::stabbing forma part? "<<" b: "<<b<<endl;

    //Mirem tambe si talla per algun costat (creem els 6 punts extrems de la circumferencia
    // i vigilem si entren al node
    pc1 = point3D(p.getX()-r,p.getY(),p.getZ());
    pc2 = point3D(p.getX()+r,p.getY(),p.getZ());
    pc3 = point3D(p.getX(),p.getY()-r,p.getZ());
    pc4 = point3D(p.getX(),p.getY()+r,p.getZ());
    pc5 = point3D(p.getX(),p.getY(),p.getZ()-r);
    pc6 = point3D(p.getX(),p.getY(),p.getZ()+r);

    b = b || formaPart(pc1) || formaPart(pc2) || formaPart(pc3) || formaPart(pc4) || formaPart(pc5) || formaPart(pc6);

    // si fins ara no hem vist si esta stabbing, mirem tambe uns quants punts a l'atzar
    if(!b)
    {
        double theta,phi;

        theta=0;
        bool trobat=false;

        while(theta<2*PI && !trobat)
        {
            phi=0;
            while(phi<PI && !trobat)
            {
                //cout<<"SkipONode::stabbing tirant uns quants punts "<<theta<<" "<<phi<<endl;
                // pillem el punt que toca de l'esfera de centre e->getPoint() i radi r
                point3D pSample = point3D( (e->getPoint()).getX() + r*sin(phi)*cos(theta) , (e->getPoint()).getY() + r*sin(phi)*sin(theta) , (e->getPoint()).getZ() + r*cos(theta)   );

                if(formaPart(pSample)) trobat=true;

                phi=phi+(PI)/(NUMERO_PUNTS_GRAELLA_STABBING);
            }


            theta=theta+(2*PI)/(NUMERO_PUNTS_GRAELLA_STABBING);
        }

        if(trobat) b=true;
    }



    //cout<<" SkipONode::stabbing talla per algun costat? "<<b<<endl;

    return b;

}

//Report all Elements in the node matching the parameters radius
list<Element*> * SkipONode::report(Element *e)
{
    list<Element*> *ret = new list<Element*>();

    //Report all Matching elements in the node
    vector<Element*>::iterator it;
    for(it=llistaElements.begin();it!=llistaElements.end();it++)
    {
        if ( ((*it)->getRadi() == e->getRadi()) && (!(*it)->getMarcat()) )
        {
            ret->push_back(*it);
        }
    }

    return ret;
}

//Report all Elements in the node matching the parameters radius and distance requirement
list<Element*> * SkipONode::reportIf(Element *e,double r)
{
    list<Element*> *ret = new list<Element*>();

    //Report all Matching elements in the node
    vector<Element*>::iterator it;
    for(it =llistaElements.begin(); it != llistaElements.end(); it++)
    {

        bool aux1 = (*it)->getRadi() == e->getRadi();
        bool aux2 = !(*it)->getMarcat();
        bool aux3 = ((*it)->getPoint()).dist(e->getPoint()) < r ;
        bool aux4 = aux1 && aux2 && aux3;

        if( aux4 )
        {
            ret->push_back(*it);
        }
    }

    return ret;
}

void SkipONode::write()
{
    if (this == NULL) {
        cout << "Null!" << endl;
    } else if (tipus()!=2) { //Si es un node fulla
        if (llistaElements.empty()) {
            cout << "Fulla blanca! "<<getAncoratge() <<" mida: "<<getMida()<< endl;
        } else {
            cout << "Fulla Negra! "<<getAncoratge() <<" mida: "<<getMida()<< endl;
            for(int i=0; i<(int)llistaElements.size(); i++)
            {
                if (i!=0) for(int j=0; j<nivell; j++) cout << "  ";
                cout << "Element " << i << ": ";
                llistaElements[i]->write();
                cout << " - Nivell: " << nivell << endl;
            }
        }
    } else {
        cout << "Intermig! "<<getAncoratge() <<" mida: "<<getMida()<< endl;
        for(int i=0; i<nivell; i++) cout << "  "; cout << "FILL 1: "; f1->write();
        for(int i=0; i<nivell; i++) cout << "  "; cout << "FILL 2: "; f2->write();
        for(int i=0; i<nivell; i++) cout << "  "; cout << "FILL 3: "; f3->write();
        for(int i=0; i<nivell; i++) cout << "  "; cout << "FILL 4: "; f4->write();
        for(int i=0; i<nivell; i++) cout << "  "; cout << "FILL 5: "; f5->write();
        for(int i=0; i<nivell; i++) cout << "  "; cout << "FILL 6: "; f6->write();
        for(int i=0; i<nivell; i++) cout << "  "; cout << "FILL 7: "; f7->write();
        for(int i=0; i<nivell; i++) cout << "  "; cout << "FILL 8: "; f8->write();
    }
}

void SkipONode::write2(ostream& os)
{
    if (this == NULL) {
        os << "Null!" << endl;
    } else if (tipus()!=2) { //Si es un node fulla
        if (llistaElements.empty()) {
            os << "Fulla blanca!" << endl;
        } else {
            for(int i=0; i<(int)llistaElements.size(); i++)
            {
                if (i!=0) for(int j=0; j<nivell; j++) os << " ";
                os << "Element " << i << ": ";
                os<<(*llistaElements[i]);
                os << " - Nivell: " << nivell << endl;
            }
        }
    } else {
        for(int i=0; i<nivell; i++) os << "  "; os << "FILL 1: "; f1->write2(os);
        for(int i=0; i<nivell; i++) os << "  "; os << "FILL 2: "; f2->write2(os);
        for(int i=0; i<nivell; i++) os << "  "; os << "FILL 3: "; f3->write2(os);
        for(int i=0; i<nivell; i++) os << "  "; os << "FILL 4: "; f4->write2(os);
        for(int i=0; i<nivell; i++) os << "  "; os << "FILL 5: "; f5->write2(os);
        for(int i=0; i<nivell; i++) os << "  "; os << "FILL 6: "; f6->write2(os);
        for(int i=0; i<nivell; i++) os << "  "; os << "FILL 7: "; f7->write2(os);
        for(int i=0; i<nivell; i++) os << "  "; os << "FILL 8: "; f8->write2(os);
    }
}