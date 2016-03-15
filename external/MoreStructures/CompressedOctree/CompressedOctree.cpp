#include "CompressedOctree.h"

#include <iostream>


#define TOLERANCIA_CALCULAR_CUB 0.000001
const int MAX_LEVEL = 7;
//const double EPSILON = 0.05;

CompressedOctree::CompressedOctree()
{
	nivellMaxim = MAX_LEVEL;
	epsilon = -1;
	
	calcularCub(); //Calculem la mida i el punt d'ancoratge
	
	arrel = new CompressedONode(ancoratge,mida,0); //Creem l'arrel amb el punt d'ancoratge, la mida i el nivell
	nivellActual = 0;

	candidates = vector<CompressedCandidateZone *>();
}

CompressedOctree::CompressedOctree(double eps)
{
	nivellMaxim = MAX_LEVEL;
	epsilon = eps;
	
	calcularCub(); //Calculem la mida i el punt d'ancoratge
	
	arrel = new CompressedONode(ancoratge,mida,0); //Creem l'arrel amb el punt d'ancoratge, la mida i el nivell
	nivellActual = 0;

	candidates = vector<CompressedCandidateZone *>();
}

CompressedOctree::CompressedOctree(vector<Element *> llE, double eps)
{
//cout << "CompressedOctree.cpp --> CREEM COMPRESSED OCTREE AMB "<< llE.size() << " ELEMENTS!" << endl;
	llistaElements = llE;
	nivellMaxim = MAX_LEVEL;
	epsilon = eps;
	
	calcularCub(); //Calculem la mida i el punt d'ancoratge
	
	arrel = new CompressedONode(ancoratge,mida,0); //Creem l'arrel amb el punt d'ancoratge, la mida i el nivell
	nivellActual = 0;
	
	llistaElements.clear();
	for(int i=0; i<(int)llE.size(); i++) {
		afegirElement(llE[i]);
	}

	candidates = vector<CompressedCandidateZone *>();
}

CompressedOctree::~CompressedOctree()
{
	if(arrel!=NULL) delete arrel;

	// les zones candidates no son mes que trossets de l'octree, no cal alliberar-les explicitament
	vector<CompressedCandidateZone *>::iterator it;
	for(it=candidates.begin();it!=candidates.end();it++)
	{
		delete *it;
	}

	for (int i = 0; i < llistaElements.size(); ++i) {
		delete llistaElements[i];
	}
	llistaElements.clear();
}


//CalcularCub: Busco distancia maxima entre cada coordenada
void CompressedOctree::calcularCub()
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


point3D CompressedOctree::getAncoratge()
{
	return ancoratge;
}

double CompressedOctree::getMida()
{
	return mida;
}

int CompressedOctree::getNivellActual()
{
	return nivellActual;
}

int CompressedOctree::getNivellMaxim()
{
	return nivellMaxim;
}

void CompressedOctree::setNivellMaxim(int n)
{
	nivellMaxim = n;
}

void CompressedOctree::afegirElement(Element *e) 
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
		arrel = new CompressedONode(ancoratge,mida,0); //Creem l'arrel amb el punt d'ancoratge, la mida i el nivell
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

void CompressedOctree::esborrarElement(Element *e) 
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

void CompressedOctree::marcarElement(Element *e)
{
	try
	{
		arrel->marcarElement(e);
	}
	catch (int i)
	{
		//cout<<"Marcar de CO, mirant d'adobar-ho"<<endl;

		bool solucionat=false;
		vector<Element *> v; 
		vector<CompressedONode *> vNodes= vector<CompressedONode *>(8);	
		// primer, recollim tots els possibles nodes
		vNodes[0]=arrel->getFill1();		
		vNodes[1]=arrel->getFill2();		
		vNodes[2]=arrel->getFill3();		
		vNodes[3]=arrel->getFill4();		
		vNodes[4]=arrel->getFill5();		
		vNodes[5]=arrel->getFill6();		
		vNodes[6]=arrel->getFill7();		
		vNodes[7]=arrel->getFill8();		

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
			cout<<"CO: No ho hem pas pogut arreglar!"<<endl;	
			exit(-1);	
		}


	}	
}

void CompressedOctree::actualitzarInfGeo()
{
	arrel->actualitzarInfGeo();
}

list<Element*> * CompressedOctree::weightedNeighbors(Element *e,double eps)
{
	list<Element*> *ret = NULL;
	
	ret = arrel->weightedNeighbors(e,eps);
	
	return ret;
}


// Pre: cap de les info es un punter null
CompressedInformacioGeometrica* CompressedOctree::calcularInfo2(CompressedInformacioGeometrica *info1, CompressedInformacioGeometrica *info2)
{	
	
	CompressedInformacioGeometrica *infoResultat = new CompressedInformacioGeometrica(*info1);
	infoResultat->afegir(info2);

	return infoResultat;
}

CompressedInformacioGeometrica* CompressedOctree::calcularInfo4(CompressedInformacioGeometrica *info1, CompressedInformacioGeometrica *info2, CompressedInformacioGeometrica *info3, CompressedInformacioGeometrica *info4)
{
	CompressedInformacioGeometrica *infoResultat=new CompressedInformacioGeometrica();

	// vigilem que les informacions que ens passen poden ser punters nulls
	if(info1!= NULL ) infoResultat->afegir(info1);	
	if(info2!= NULL ) infoResultat->afegir(info2);	
	if(info3!= NULL ) infoResultat->afegir(info3);	
	if(info4!= NULL ) infoResultat->afegir(info4);	

	return infoResultat;
}

CompressedInformacioGeometrica* CompressedOctree::calcularInfo8(CompressedInformacioGeometrica *info1, CompressedInformacioGeometrica *info2, CompressedInformacioGeometrica *info3, CompressedInformacioGeometrica *info4, CompressedInformacioGeometrica *info5, CompressedInformacioGeometrica *info6, CompressedInformacioGeometrica *info7, CompressedInformacioGeometrica *info8)
{
	CompressedInformacioGeometrica *infoResultat=new CompressedInformacioGeometrica();

	// vigilem que les informacions que ens passen poden ser punters nulls
	if(info1!= NULL ) infoResultat->afegir(info1);	
	if(info2!= NULL ) infoResultat->afegir(info2);	
	if(info3!= NULL ) infoResultat->afegir(info3);	
	if(info4!= NULL ) infoResultat->afegir(info4);	
	if(info5!= NULL ) infoResultat->afegir(info5);	
	if(info6!= NULL ) infoResultat->afegir(info6);	
	if(info7!= NULL ) infoResultat->afegir(info7);	
	if(info8!= NULL ) infoResultat->afegir(info8);	
	
	return infoResultat;
}

//Function that returns the areas where the compressed octree follows
//the restrictions imposed by a certain information
void CompressedOctree::search(CompressedInformacioGeometrica info, bool bNum, bool bHisto, bool bDist)
{
	//Call the intern function to compare every atribute of the representation
	this->intern_search(info, bNum,  bHisto, bDist);
}

// Function to search for the information
void CompressedOctree::intern_search(CompressedInformacioGeometrica info, bool bNum, bool bHisto, bool bDist)
{
	if( arrel == NULL )
	{
		cout<<"ERROR (CompressedOctree.cpp::intern_search) : Triyng to search in a Null Compressed Octree"<<endl;
		exit(-1);
	}
	else
	{
		//Creem o actualitzem la informacio geometrica de l'octree
		arrel->actualitzarInfGeo();

		//Comencem la cerca
		intern_search_1(info, arrel, bNum, bHisto, bDist);
	}
}

/* Intern search functions:
1- Following the structure of the octree
2- Pairs of neighbors
4- Quartets
8- Octets
*/
// Function that searches following the structure of the octree
// Precondition, the CompressedONode t is not null (granted in extern function)
// Searches in compatible nodes until it finds a leaf or reaches the correct size.
void CompressedOctree::intern_search_1(CompressedInformacioGeometrica info, CompressedONode *t, bool bNum, bool bHisto, bool bDist)
{
	//cout<<endl<<endl<<endl;
	//cout<<"entro a search_1 amb node, mida i info: "<<t->getAncoratge()<<" "<<t->getMida()<<" "<<*(t->getInfGeo())<<endl;
	
//char c;
//cin>>c;

	//Search following the octree structure
	if (CompressedONode::compatible( info, t->getInfGeo(), bNum, bHisto, bDist ,epsilon))
	{
		// Compatible node, test if further exploration is needed (leaf node or correct size)
		if ( ( (t->getMida()/2.) < info.getMidaNode() )  || (t->tipus()==1)  )
		{
			//Store the candidate zone and keep a pointer to it here
			CompressedCandidateZone *cand = new CompressedCandidateZone(1,t);
			candidates.push_back(cand);
			t->setNodeZones(cand);
		
			//cout<<"                                                                         ";
			//cout<<"1REPORTO!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
			//cout<<"Compatible QNode ! "<<endl;
			//cout<<"Base Point: "<<t->getAncoratge()<<" Size: "<<t->getMida()<<endl;

  			//Aqui hi aniria dibuixar la zona candidata FALTA!
		}
		else //Recursively call Search functions:
		{
			//Avoid recursive calls in NULL sons and white leaves
			if (t->tipus()==2)
			{
				bool ok1,ok2,ok3,ok4,ok5,ok6,ok7,ok8;

				ok1 = (t->getFill1()!=NULL) && ((t->getFill1())->tipus()!=0);
				ok2 = (t->getFill2()!=NULL) && ((t->getFill2())->tipus()!=0);
				ok3 = (t->getFill3()!=NULL) && ((t->getFill3())->tipus()!=0);
				ok4 = (t->getFill4()!=NULL) && ((t->getFill4())->tipus()!=0);
				ok5 = (t->getFill5()!=NULL) && ((t->getFill5())->tipus()!=0);
				ok6 = (t->getFill6()!=NULL) && ((t->getFill6())->tipus()!=0);
				ok7 = (t->getFill7()!=NULL) && ((t->getFill7())->tipus()!=0);
				ok8 = (t->getFill8()!=NULL) && ((t->getFill8())->tipus()!=0);
				
				//cout<<"     Crides recursives a search_1 desde 1"<<endl;
				//1. Following octree structure
				if (ok1) intern_search_1(info,t->getFill1(),bNum,bHisto,bDist);
				if (ok2) intern_search_1(info,t->getFill2(),bNum,bHisto,bDist);
				if (ok3) intern_search_1(info,t->getFill3(),bNum,bHisto,bDist);
				if (ok4) intern_search_1(info,t->getFill4(),bNum,bHisto,bDist);
				if (ok5) intern_search_1(info,t->getFill5(),bNum,bHisto,bDist);
				if (ok6) intern_search_1(info,t->getFill6(),bNum,bHisto,bDist);
				if (ok7) intern_search_1(info,t->getFill7(),bNum,bHisto,bDist);
				if (ok8) intern_search_1(info,t->getFill8(),bNum,bHisto,bDist);
				
				//cout<<"     Crides recursives a search_2 desde 1"<<endl;
				//2. Search pairs of X neighbors
				// Beware of white leaved couples
				if ( (ok1) && (ok2) ) intern_search_2x(info,t->getFill1(),t->getFill2(),bNum,bHisto,bDist);
				if ( (ok3) && (ok4) ) intern_search_2x(info,t->getFill3(),t->getFill4(),bNum,bHisto,bDist);
				if ( (ok5) && (ok6) ) intern_search_2x(info,t->getFill5(),t->getFill6(),bNum,bHisto,bDist);
				if ( (ok7) && (ok8) ) intern_search_2x(info,t->getFill7(),t->getFill8(),bNum,bHisto,bDist);
				// Search pairs of Y neighbors
				// Beware of white leaved couples
				if ( (ok1) && (ok3) ) intern_search_2y(info,t->getFill1(),t->getFill3(),bNum,bHisto,bDist);
				if ( (ok2) && (ok4) ) intern_search_2y(info,t->getFill2(),t->getFill4(),bNum,bHisto,bDist);
				if ( (ok5) && (ok7) ) intern_search_2y(info,t->getFill5(),t->getFill7(),bNum,bHisto,bDist);
				if ( (ok6) && (ok8) ) intern_search_2y(info,t->getFill6(),t->getFill8(),bNum,bHisto,bDist);
				// Search pairs of Z neighbors
				// Beware of white leaved couples
				if ( (ok1) && (ok5) ) intern_search_2z(info,t->getFill1(),t->getFill5(),bNum,bHisto,bDist);
				if ( (ok2) && (ok6) ) intern_search_2z(info,t->getFill2(),t->getFill6(),bNum,bHisto,bDist);
				if ( (ok3) && (ok7) ) intern_search_2z(info,t->getFill3(),t->getFill7(),bNum,bHisto,bDist);
				if ( (ok4) && (ok8) ) intern_search_2z(info,t->getFill4(),t->getFill8(),bNum,bHisto,bDist);
			
				// abans de llençar les cerques, descomprimim				
				CompressedONode nodeAux = CompressedONode(t->getAncoratge(),t->getMida(),t->getNivell() );
				vector<bool> fillsTocats = descomprimeix(t,&nodeAux);

				// ara totes les cerques es llençaran sobre els fills de nodeAux

				//3. Search quartets
			
				call_intern_search_4(info,ok1,ok2,ok3,ok4,nodeAux.getFill1(),nodeAux.getFill2(),nodeAux.getFill3(),nodeAux.getFill4(),bNum,bHisto,bDist,1);
				call_intern_search_4(info,ok5,ok6,ok7,ok8,nodeAux.getFill5(),nodeAux.getFill6(),nodeAux.getFill7(),nodeAux.getFill8(),bNum,bHisto,bDist,1);
				call_intern_search_4(info,ok1,ok2,ok5,ok6,nodeAux.getFill1(),nodeAux.getFill2(),nodeAux.getFill5(),nodeAux.getFill6(),bNum,bHisto,bDist,2);
				call_intern_search_4(info,ok3,ok4,ok7,ok8,nodeAux.getFill3(),nodeAux.getFill4(),nodeAux.getFill7(),nodeAux.getFill8(),bNum,bHisto,bDist,2);
				call_intern_search_4(info,ok1,ok3,ok5,ok7,nodeAux.getFill1(),nodeAux.getFill3(),nodeAux.getFill5(),nodeAux.getFill7(),bNum,bHisto,bDist,3);
				call_intern_search_4(info,ok2,ok4,ok6,ok8,nodeAux.getFill2(),nodeAux.getFill4(),nodeAux.getFill6(),nodeAux.getFill8(),bNum,bHisto,bDist,3);

				//4. Search octet
				call_intern_search_8(info,ok1,ok2,ok3,ok4,ok5,ok6,ok7,ok8,nodeAux.getFill1(),nodeAux.getFill2(),nodeAux.getFill3(),nodeAux.getFill4(),nodeAux.getFill5(),nodeAux.getFill6(),nodeAux.getFill7(),nodeAux.getFill8(),bNum,bHisto,bDist);

				// Recomprimeix per no perdre memoria
				recomprimeix(fillsTocats, &nodeAux);
				// ara nAux es mor i l'estructura "auxiliar" queda alliberada sense perdre memoria.
				
			}
		}
	}
	//cout<<"surto de 1, zones trobades fins ara"<<candidates.size()<<endl;
	//cout<<endl<<endl<<endl;
	//for(unsigned int i=0;i<candidates.size();i++) cout<<*candidates[i]<<endl;

}

void CompressedOctree::call_intern_search_4(CompressedInformacioGeometrica info,bool ok1,bool ok2,bool ok3,bool ok4,CompressedONode *t1,CompressedONode *t2,CompressedONode *t3,CompressedONode *t4, bool bNum, bool bHisto, bool bDist,int tipus)
{
	bool left = ok1 || ok3;
	bool right = ok2 || ok4;
	bool up = ok1 || ok2;
	bool down = ok3 || ok4;

	if (left && right && up && down) {
		switch (tipus) {
			case 1:
				intern_search_4xy(info,t1,t2,t3,t4,bNum,bHisto,bDist);
				break;
			case 2:
				intern_search_4xz(info,t1,t2,t3,t4,bNum,bHisto,bDist);
				break;
			case 3:
				intern_search_4yz(info,t1,t2,t3,t4,bNum,bHisto,bDist);
				break;
			default:
				cout << "ERROR (CompressedOctree::call_intern_search_4) : No s'ha trobat el tipus" << endl;
				break;
			
		}
	}
}

void CompressedOctree::call_intern_search_8(CompressedInformacioGeometrica info, bool ok1, bool ok2, bool ok3, bool ok4, bool ok5, bool ok6, bool ok7, bool ok8,CompressedONode *t1,CompressedONode *t2,CompressedONode *t3,CompressedONode *t4, CompressedONode *t5,CompressedONode *t6,CompressedONode *t7,CompressedONode *t8, bool bNum, bool bHisto, bool bDist)
{
	bool front = ok1 || ok2 || ok3 || ok4;
	bool back = ok5 || ok6 || ok7 || ok8;
	bool left = ok1 || ok5 || ok3 || ok7;
	bool right = ok2 || ok6 || ok4 || ok8;
	bool up = ok1 || ok5 || ok2 || ok6;
	bool down = ok3 || ok7 || ok4 || ok8;

	if (front && back && left && right && up && down) {
		intern_search_8(info,t1,t2,t3,t4,t5,t6,t7,t8,bNum,bHisto,bDist);
	}
}

bool CompressedOctree::esToquen2(CompressedInformacioGeometrica info, CompressedONode * t1, CompressedONode * t2)
{
	//Busquem els punts que estiguin més a prop dels dos nodes
	double mida1 = t1->getMida();
	point3D punt1 = t1->getAncoratge();
	double mida2 = t2->getMida();
	point3D punt2 = t2->getAncoratge();

	point3D centre1 = point3D(punt1.getX()+(mida1/2.0),punt1.getY()-(mida1/2.0),punt1.getZ()-(mida1/2.0));
	point3D centre2 = point3D(punt2.getX()+(mida2/2.0),punt2.getY()-(mida2/2.0),punt2.getZ()-(mida2/2.0));
	double dif = (mida1+mida2)/2.0;
	
	double varX = centre2.getX()-centre1.getX();
	if (varX>=dif) {
		punt1.setX(punt1.getX()+mida1);
	} else if (varX<=-dif) {
		punt2.setX(punt2.getX()+mida2);
	} else {
		if (mida1>=mida2) {
			if (varX>0.0) punt1.setX(punt1.getX()+mida1);
			if ((punt1.getX()-centre2.getX()) > 0.0) punt2.setX(punt2.getX()+mida2);
		} else {
			if (varX<0.0) punt2.setX(punt2.getX()+mida2);
			if ((punt2.getX()-centre1.getX()) > 0.0) punt1.setX(punt1.getX()+mida1);
		}
	}
	
	double varY = centre2.getY()-centre1.getY();
	if (varY>=dif) {
		punt2.setY(punt2.getY()-mida2);
	} else if (varY<=-dif) {
		punt1.setY(punt1.getY()-mida1);
	} else {
		if (mida1>=mida2) {
			if (varY<0.0) punt1.setY(punt1.getY()-mida1);
			if ((punt1.getY()-centre2.getY()) < 0.0) punt2.setY(punt2.getY()-mida2);
		} else {
			if (varY>0.0) punt2.setY(punt2.getY()-mida2);
			if ((punt2.getY()-centre1.getY()) < 0.0) punt1.setY(punt1.getY()-mida1);
		}
	}
	
	double varZ = centre2.getZ()-centre1.getZ();
	if (varZ>=dif) {
		punt2.setZ(punt2.getZ()-mida2);
	} else if (varZ<=-dif) {
		punt1.setZ(punt1.getZ()-mida1);
	} else {
		if (mida1>=mida2) {
			if (varZ<0.0) punt1.setZ(punt1.getZ()-mida1);
			if ((punt1.getZ()-centre2.getZ()) < 0.0) punt2.setZ(punt2.getZ()-mida2);
		} else {
			if (varZ>0.0) punt2.setZ(punt2.getZ()-mida2);
			if ((punt2.getZ()-centre1.getZ()) < 0.0) punt1.setZ(punt1.getZ()-mida1);
		}
	}

	//Un cop tenim els dos punts que estan més a prop (punt1 i punt2), mirem si la info pot interseccionar amb els dos
	varX = fabs(punt1.getX()-punt2.getX());
	varY = fabs(punt1.getY()-punt2.getY());
	varZ = fabs(punt1.getZ()-punt2.getZ());
	
	double maxim = varX;
	if (varY>maxim) maxim = varY;
	if (varZ>maxim) maxim = varZ;
	
	if (info.getMidaNode()>maxim) {
		return true;
	} else {
		return false;
	}
}

bool CompressedOctree::deCostat2(CompressedONode * t1, CompressedONode * t2, int tipus)
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
			cout << "ERROR (CompressedOctree::deCostat) : No s'ha trobat el tipus" << endl;
			exit(-1);
			//per treure un warning
			return false;
			break;
	}

}

//Metode que busca quin fill (de t1 o de t2) escombra l'altre node.
//Precondicio: t1 i t2 no tenen la mateixa mida
int CompressedOctree::quiEscombra2(CompressedONode * t1, CompressedONode * t2, int tipus)
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
			cout << "ERROR (CompressedOctree::deCostat) : No s'ha trobat el tipus" << endl;
			break;
	}

	return -1;
}

//Function that searches in pairs of X neighbors 1 2
//Precondition: t1 and t2 must be not NULL
void CompressedOctree::intern_search_2x(CompressedInformacioGeometrica info, CompressedONode * t1, CompressedONode * t2, bool bNum, bool bHisto, bool bDist)
{
	//cout<<endl<<endl<<endl;
	//cout<<"entro a search_2x amb node1 i mida1: "<<t1->getAncoratge()<<" "<<t1->getMida()<<" i node2 i mida 2  "<<t2->getAncoratge()<<" "<<t2->getMida()<<endl;

//char c;
//cin>>c;

	//Test for compatibility
	if ( (esToquen2(info,t1,t2)) && (CompressedONode::compatible(info,calcularInfo2(t1->getInfGeo(),t2->getInfGeo()), bNum, bHisto, bDist,epsilon)) ) {
		
		//Agafem la mida mes gran dels que siguin parcials
		bool parcial1 = t1->tipus()==2;
		bool parcial2 = t2->tipus()==2;
		double mida = 0.0;
		if (parcial1) mida = t1->getMida();
		if ((parcial2) && (t2->getMida()>mida)) mida = t2->getMida();

		if ( (mida/2.0)<info.getMidaNode() ) {

			// Let's store the candidate zone
			CompressedCandidateZone *cand = new CompressedCandidateZone(2,t1,t2);
			candidates.push_back(cand);	
			t1->setNodeZones(cand);
			t2->setNodeZones(cand);
		
			//cout<<"                                                                         ";
			//cout<<"REPORTO!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
			//cout<<"Compatible side Pair ! "<<endl;
			//cout<<"Base Point1: "<<t1->getAncoratge()<<" Size1: "<<t1->getMida()<<endl;
			//cout<<"Base Point2: "<<t2->getAncoratge()<<" Size2: "<<t2->getMida()<<endl;
			
			//aqui tb falta pintar	
		}
		else
		{
			// oju arrodoniments!
			if ( t1->getMida()==t2->getMida() ) {
				//Tenen la mateixa mida
				
				if ( deCostat2(t1,t2,1) ) {
					//Estan de costat
	
					if (parcial1 && parcial2) {
						//Tots 2 parcials

						//Recursively call Search function
						bool ok1,ok2,ok3,ok4,ok5,ok6,ok7,ok8;
			
						ok1 = (t1->getFill2()!=NULL) && ((t1->getFill2())->tipus()!=0);
						ok2 = (t2->getFill1()!=NULL) && ((t2->getFill1())->tipus()!=0);
						ok3 = (t1->getFill4()!=NULL) && ((t1->getFill4())->tipus()!=0);
						ok4 = (t2->getFill3()!=NULL) && ((t2->getFill3())->tipus()!=0);
						ok5 = (t1->getFill6()!=NULL) && ((t1->getFill6())->tipus()!=0);
						ok6 = (t2->getFill5()!=NULL) && ((t2->getFill5())->tipus()!=0);
						ok7 = (t1->getFill8()!=NULL) && ((t1->getFill8())->tipus()!=0);
						ok8 = (t2->getFill7()!=NULL) && ((t2->getFill7())->tipus()!=0);
						
						//cout<<"     Crides recursives a search_2 desde 2"<<endl;
						//2. Search pairs of X neighbors
						// Beware of white leaved couples
						if ( (ok1) && (ok2) ) intern_search_2x(info,t1->getFill2(),t2->getFill1(),bNum,bHisto,bDist);
						if ( (ok3) && (ok4) ) intern_search_2x(info,t1->getFill4(),t2->getFill3(),bNum,bHisto,bDist);
						if ( (ok5) && (ok6) ) intern_search_2x(info,t1->getFill6(),t2->getFill5(),bNum,bHisto,bDist);
						if ( (ok7) && (ok8) ) intern_search_2x(info,t1->getFill8(),t2->getFill7(),bNum,bHisto,bDist);
						
						// Abans de llençar les cerques, descomprimim				
        					CompressedONode nodeAux1 = CompressedONode(t1->getAncoratge(),t1->getMida(),t1->getNivell() );
        					vector<bool> fillsTocats1 = descomprimeix(t1,&nodeAux1);	
        					CompressedONode nodeAux2 = CompressedONode(t2->getAncoratge(),t2->getMida(),t2->getNivell() );	
        					vector<bool> fillsTocats2 = descomprimeix(t2,&nodeAux2);
	
						//3. Search quartets
						call_intern_search_4(info,ok1,ok2,ok3,ok4,nodeAux1.getFill2(),nodeAux2.getFill1(),nodeAux1.getFill4(),nodeAux2.getFill3(),bNum,bHisto,bDist,1);
						call_intern_search_4(info,ok5,ok6,ok7,ok8,nodeAux1.getFill6(),nodeAux2.getFill5(),nodeAux1.getFill8(),nodeAux2.getFill7(),bNum,bHisto,bDist,1);
						call_intern_search_4(info,ok1,ok2,ok5,ok6,nodeAux1.getFill2(),nodeAux2.getFill1(),nodeAux1.getFill6(),nodeAux2.getFill5(),bNum,bHisto,bDist,2);
						call_intern_search_4(info,ok3,ok4,ok7,ok8,nodeAux1.getFill4(),nodeAux2.getFill3(),nodeAux1.getFill8(),nodeAux2.getFill7(),bNum,bHisto,bDist,2);
			
						//4. Search octet
						call_intern_search_8(info,ok1,ok2,ok3,ok4,ok5,ok6,ok7,ok8,nodeAux1.getFill2(),nodeAux2.getFill1(),nodeAux1.getFill4(),nodeAux2.getFill3(),nodeAux1.getFill6(),nodeAux2.getFill5(),nodeAux1.getFill8(),nodeAux2.getFill7(),bNum,bHisto,bDist);
					
        					// Recomprimeix per no perdre memoria
        					recomprimeix(fillsTocats1, &nodeAux1);
        					recomprimeix(fillsTocats2, &nodeAux2);
					
					} else if (parcial1) {
						//Nomes 1r parcial

						//Mirem si creant un node orfe al costat del node parcial
						//aconseguim "atrapar" algun element
						double dif = (t2->getAncoratge().getX()-t1->getAncoratge().getX());
						double difMax = (2.0*t1->getMida())-1.0e-6;
	
						if ( (dif<difMax) && (deCostat2(t1,t2,1)) ) {
							//CALDRIA FER NODE ORFEEEEEEEEEEEEEEEEEEEEEEEEE!!!!!!!!!!!!!!!!!!!!!!!!!!
							// Let's store the candidate zone
							CompressedCandidateZone *cand = new CompressedCandidateZone(2,t1,t2);
							candidates.push_back(cand);	
							t1->setNodeZones(cand);
							t2->setNodeZones(cand);
						
					//cout<<"                                                                         ";
							//cout<<"CO: 1Reporto irregular "<<endl;
							//cout<<"REPORTO!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
							//cout<<"Compatible side Pair ! "<<endl;
							//cout<<"Base Point1: "<<t1->getAncoratge()<<" Size1: "<<t1->getMida()<<endl;
							//cout<<"Base Point2: "<<t2->getAncoratge()<<" Size2: "<<t2->getMida()<<endl;
						} else {
							// Let's store the candidate zone
							CompressedCandidateZone *cand = new CompressedCandidateZone(2,t1,t2);
							candidates.push_back(cand);	
							t1->setNodeZones(cand);
							t2->setNodeZones(cand);
						
				//cout<<"                                                                         ";
							//cout<<"CO: 2Reporto irregular "<<endl;
							//cout<<"REPORTO!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
							//cout<<"Compatible side Pair ! "<<endl;
							//cout<<"Base Point1: "<<t1->getAncoratge()<<" Size1: "<<t1->getMida()<<endl;
							//cout<<"Base Point2: "<<t2->getAncoratge()<<" Size2: "<<t2->getMida()<<endl;
						}

					} else if (parcial2) {
						//Nomes 2n parcial

						//Mirem si creant un node orfe al costat del node parcial
						//aconseguim "atrapar" algun element
						double dif = (t2->getAncoratge().getX()-t1->getAncoratge().getX());
						double difMax = t2->getMida()+1.0e-6;
	
						if ( (dif<=difMax) && (deCostat2(t1,t2,1)) ) {
							//CALDRIA FER NODE ORFEEEEEEEEEEEEEEEEEEEEEEEEE!!!!!!!!!!!!!!!!!!!!!!!!!!
							// Let's store the candidate zone
							CompressedCandidateZone *cand = new CompressedCandidateZone(2,t1,t2);
							candidates.push_back(cand);	
							t1->setNodeZones(cand);
							t2->setNodeZones(cand);

			//cout<<"                                                                         ";
							//cout<<"CO: 3Reporto irregular "<<endl;
							//cout<<"REPORTO!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
							//cout<<"Compatible side Pair ! "<<endl;
							//cout<<"Base Point1: "<<t1->getAncoratge()<<" Size1: "<<t1->getMida()<<endl;
							//cout<<"Base Point2: "<<t2->getAncoratge()<<" Size2: "<<t2->getMida()<<endl;
						} else {
							// Let's store the candidate zone
							CompressedCandidateZone *cand = new CompressedCandidateZone(2,t1,t2);
							candidates.push_back(cand);	
							t1->setNodeZones(cand);
							t2->setNodeZones(cand);

			//cout<<"                                                                         ";
							//cout<<"CO: 4Reporto irregular "<<endl;
							//cout<<"REPORTO!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
							//cout<<"Compatible side Pair ! "<<endl;
							//cout<<"Base Point1: "<<t1->getAncoratge()<<" Size1: "<<t1->getMida()<<endl;
							//cout<<"Base Point2: "<<t2->getAncoratge()<<" Size2: "<<t2->getMida()<<endl;
						}

					} else {
						//Cap parcial (2 fulles negres)

						// Let's store the candidate zone
						CompressedCandidateZone *cand = new CompressedCandidateZone(2,t1,t2);
						candidates.push_back(cand);	
						t1->setNodeZones(cand);
						t2->setNodeZones(cand);
					cout<<"                                                                         ";
						//cout<<"REPORTO!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
						//cout<<"Compatible side Pair ! "<<endl;
						//cout<<"Base Point1: "<<t1->getAncoratge()<<" Size1: "<<t1->getMida()<<endl;
						//cout<<"Base Point2: "<<t2->getAncoratge()<<" Size2: "<<t2->getMida()<<endl;
					}
				} else {
					//No estan de costat
					
					//CALDRIA MIRAR ELS 12 CASOS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

					// Let's store the candidate zone
					CompressedCandidateZone *cand = new CompressedCandidateZone(2,t1,t2);
					candidates.push_back(cand);	
					t1->setNodeZones(cand);
					t2->setNodeZones(cand);

			//cout<<"                                                                         ";
					//cout<<"CO: Reporto irregular "<<endl;
					//cout<<"REPORTO!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
					//cout<<"Compatible side Pair ! "<<endl;
					//cout<<"Base Point1: "<<t1->getAncoratge()<<" Size1: "<<t1->getMida()<<endl;
					//cout<<"Base Point2: "<<t2->getAncoratge()<<" Size2: "<<t2->getMida()<<endl;
				}

			} else {
				//No tenen la mateixa mida

				if (t1->getMida()>t2->getMida()) {
					//t1 es mes gran
					
					int escombra = quiEscombra2(t1,t2,1);
					if ( escombra!=-1 ) {
						//Estan de costat (algun fill de t1 escombra a t2)
						
						if (parcial1) {
							//t1 es parcial

							bool ok1,ok2,ok3,ok4,ok5,ok6,ok7,ok8;
			
							ok1 = (t1->getFill2()!=NULL) && ((t1->getFill2())->tipus()!=0);
							ok2 = false;
							ok3 = (t1->getFill4()!=NULL) && ((t1->getFill4())->tipus()!=0);
							ok4 = false;
							ok5 = (t1->getFill6()!=NULL) && ((t1->getFill6())->tipus()!=0);
							ok6 = false;
							ok7 = (t1->getFill8()!=NULL) && ((t1->getFill8())->tipus()!=0);
							ok8 = false;

							CompressedONode *t2_1 = NULL;
							CompressedONode *t2_3 = NULL;
							CompressedONode *t2_5 = NULL;
							CompressedONode *t2_7 = NULL;

							switch (escombra) {
								case 2:
									//El fill 2 del node 1 escombra el node 2
									ok2 = true;
									t2_1 = t2;
									break;
								case 4:
									//El fill 4 del node 1 escombra el node 2
									ok4 = true;
									t2_3 = t2;
									break;
								case 6:
									//El fill 6 del node 1 escombra el node 2
									ok6 = true;
									t2_5 = t2;
									break;
								case 8:
									//El fill 8 del node 1 escombra el node 2
									ok8 = true;
									t2_7 = t2;
									break;
								default:
									cout << "ERROR (CompressedOctree ::intern_search_2x) : La escombra s'ha equivocat!" << endl;
									exit(-1);
									break;
							}
							
							//Cridem search_2
							if ( (ok1) && (ok2) ) intern_search_2x(info,t1->getFill2(),t2_1,bNum,bHisto,bDist);
							if ( (ok3) && (ok4) ) intern_search_2x(info,t1->getFill4(),t2_3,bNum,bHisto,bDist);
							if ( (ok5) && (ok6) ) intern_search_2x(info,t1->getFill6(),t2_5,bNum,bHisto,bDist);
							if ( (ok7) && (ok8) ) intern_search_2x(info,t1->getFill8(),t2_7,bNum,bHisto,bDist);
							
							//Cridem search_4
							call_intern_search_4(info,ok1,ok2,ok3,ok4,t1->getFill2(),t2_1,t1->getFill4(),t2_3,bNum,bHisto,bDist,1);
							call_intern_search_4(info,ok5,ok6,ok7,ok8,t1->getFill6(),t2_5,t1->getFill8(),t2_7,bNum,bHisto,bDist,1);
							call_intern_search_4(info,ok1,ok2,ok5,ok6,t1->getFill2(),t2_1,t1->getFill6(),t2_5,bNum,bHisto,bDist,2);
							call_intern_search_4(info,ok3,ok4,ok7,ok8,t1->getFill4(),t2_3,t1->getFill8(),t2_7,bNum,bHisto,bDist,2);
				
							//Cridem search_8
							call_intern_search_8(info,ok1,ok2,ok3,ok4,ok5,ok6,ok7,ok8,t1->getFill2(),t2_1,t1->getFill4(),t2_3,t1->getFill6(),t2_5,t1->getFill8(),t2_7,bNum,bHisto,bDist);
					
						} else if (parcial2) {
							//Nomes t2 es parcial

							//Mirem si creant un node orfe al costat del node parcial
							//aconseguim "atrapar" algun element
							double dif = (t2->getAncoratge().getX()-t1->getAncoratge().getX());
							double difMax = t2->getMida()+1.0e-6;
		
							if ( (dif<=difMax) && (quiEscombra2(t1,t2,1)!=-1) ) {
								//CALDRIA FER NODE ORFEEEEEEEEEEEEEEEEEEEEEEEEE!!!!!!!!!!!!!!!!!!!!!!!!!!
								// Let's store the candidate zone
								CompressedCandidateZone *cand = new CompressedCandidateZone(2,t1,t2);
								candidates.push_back(cand);	
								t1->setNodeZones(cand);
								t2->setNodeZones(cand);
						//cout<<"                                                                         ";
								//cout<<"REPORTO!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
								//cout<<"Compatible side Pair ! "<<endl;
								//cout<<"Base Point1: "<<t1->getAncoratge()<<" Size1: "<<t1->getMida()<<endl;
								//cout<<"Base Point2: "<<t2->getAncoratge()<<" Size2: "<<t2->getMida()<<endl;
							} else {
								// Let's store the candidate zone
								CompressedCandidateZone *cand = new CompressedCandidateZone(2,t1,t2);
								candidates.push_back(cand);	
								t1->setNodeZones(cand);
								t2->setNodeZones(cand);

			//cout<<"                                                                         ";
					
								//cout<<"REPORTO!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
								//cout<<"Compatible side Pair ! "<<endl;
								//cout<<"Base Point1: "<<t1->getAncoratge()<<" Size1: "<<t1->getMida()<<endl;
								//cout<<"Base Point2: "<<t2->getAncoratge()<<" Size2: "<<t2->getMida()<<endl;
							}
						} else {
							//Cap parcial (2 fulles negres)
	
							// Let's store the candidate zone
							CompressedCandidateZone *cand = new CompressedCandidateZone(2,t1,t2);
							candidates.push_back(cand);	
							t1->setNodeZones(cand);
								t2->setNodeZones(cand);
						
			//cout<<"                                                                         ";
							//cout<<"REPORTO!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
							//cout<<"Compatible side Pair ! "<<endl;
							//cout<<"Base Point1: "<<t1->getAncoratge()<<" Size1: "<<t1->getMida()<<endl;
							//cout<<"Base Point2: "<<t2->getAncoratge()<<" Size2: "<<t2->getMida()<<endl;
						}
					} else {
						//No estan de costat
					
						//CALDRIA MIRAR ELS 12 CASOS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	
						// Let's store the candidate zone
						CompressedCandidateZone *cand = new CompressedCandidateZone(2,t1,t2);
						candidates.push_back(cand);	
						t1->setNodeZones(cand);
						t2->setNodeZones(cand);
					
			//cout<<"                                                                         ";
						//cout<<"REPORTO!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
						//cout<<"Compatible side Pair ! "<<endl;
						//cout<<"Base Point1: "<<t1->getAncoratge()<<" Size1: "<<t1->getMida()<<endl;
						//cout<<"Base Point2: "<<t2->getAncoratge()<<" Size2: "<<t2->getMida()<<endl;
					}

				} else {
					//t2 es mes gran
					
					int escombra = quiEscombra2(t1,t2,1);
					if ( escombra!=-1 ) {
						//Estan de costat (algun fill de t2 escombra a t1)
						
						if (parcial2) {
							//t2 es parcial

							bool ok1,ok2,ok3,ok4,ok5,ok6,ok7,ok8;
			
							ok1 = false;
							ok2 = (t2->getFill1()!=NULL) && ((t2->getFill1())->tipus()!=0);
							ok3 = false;
							ok4 = (t2->getFill3()!=NULL) && ((t2->getFill3())->tipus()!=0);
							ok5 = false;
							ok6 = (t2->getFill5()!=NULL) && ((t2->getFill5())->tipus()!=0);
							ok7 = false;
							ok8 = (t2->getFill7()!=NULL) && ((t2->getFill7())->tipus()!=0);

							CompressedONode *t1_2 = NULL;
							CompressedONode *t1_4 = NULL;
							CompressedONode *t1_6 = NULL;
							CompressedONode *t1_8 = NULL;

							switch (escombra) {
								case 1:
									//El fill 1 del node 2 escombra el node 1
									ok1 = true;
									t1_2 = t1;
									break;
								case 3:
									//El fill 3 del node 2 escombra el node 1
									ok3 = true;
									t1_4 = t1;
									break;
								case 5:
									//El fill 5 del node 2 escombra el node 1
									ok5 = true;
									t1_6 = t1;
									break;
								case 7:
									//El fill 7 del node 2 escombra el node 1
									ok7 = true;
									t1_8 = t1;
									break;
								default:
									cout << "ERROR (CompressedOctree ::intern_search_2x) : La escombra s'ha equivocat!" << endl;
									exit(-1);
									break;
							}
							
							//Cridem search_2
							if ( (ok1) && (ok2) ) intern_search_2x(info,t1_2,t2->getFill1(),bNum,bHisto,bDist);
							if ( (ok3) && (ok4) ) intern_search_2x(info,t1_4,t2->getFill3(),bNum,bHisto,bDist);
							if ( (ok5) && (ok6) ) intern_search_2x(info,t1_6,t2->getFill5(),bNum,bHisto,bDist);
							if ( (ok7) && (ok8) ) intern_search_2x(info,t1_8,t2->getFill7(),bNum,bHisto,bDist);
							
							//Cridem search_4
							call_intern_search_4(info,ok1,ok2,ok3,ok4,t1_2,t2->getFill1(),t1_4,t2->getFill3(),bNum,bHisto,bDist,1);
							call_intern_search_4(info,ok5,ok6,ok7,ok8,t1_6,t2->getFill5(),t1_8,t2->getFill7(),bNum,bHisto,bDist,1);
							call_intern_search_4(info,ok1,ok2,ok5,ok6,t1_2,t2->getFill1(),t1_6,t2->getFill5(),bNum,bHisto,bDist,2);
							call_intern_search_4(info,ok3,ok4,ok7,ok8,t1_4,t2->getFill3(),t1_8,t2->getFill7(),bNum,bHisto,bDist,2);
				
							//Cridem search_8
							call_intern_search_8(info,ok1,ok2,ok3,ok4,ok5,ok6,ok7,ok8,t1_2,t2->getFill1(),t1_4,t2->getFill3(),t1_6,t2->getFill5(),t1_8,t2->getFill7(),bNum,bHisto,bDist);
					
						} else if (parcial1) {
							//Nomes t1 es parcial

							//Mirem si creant un node orfe al costat del node parcial
							//aconseguim "atrapar" algun element
							double dif = (t2->getAncoratge().getX()-t1->getAncoratge().getX());
							double difMax = (2.0*t1->getMida())-1.0e-6;
		
							if ( (dif<difMax) && (quiEscombra2(t1,t2,1)!=-1) ) {
								//CALDRIA FER NODE ORFEEEEEEEEEEEEEEEEEEEEEEEEE!!!!!!!!!!!!!!!!!!!!!!!!!!
								// Let's store the candidate zone
								CompressedCandidateZone *cand = new CompressedCandidateZone(2,t1,t2);
								candidates.push_back(cand);	
								t1->setNodeZones(cand);
								t2->setNodeZones(cand);
							
			//cout<<"                                                                         ";
								//cout<<"REPORTO!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
								//cout<<"Compatible side Pair ! "<<endl;
								//cout<<"Base Point1: "<<t1->getAncoratge()<<" Size1: "<<t1->getMida()<<endl;
								//cout<<"Base Point2: "<<t2->getAncoratge()<<" Size2: "<<t2->getMida()<<endl;
							} else {
								// Let's store the candidate zone
								CompressedCandidateZone *cand = new CompressedCandidateZone(2,t1,t2);
								candidates.push_back(cand);	
								t1->setNodeZones(cand);
								t2->setNodeZones(cand);
							
			//cout<<"                                                                         ";
								//cout<<"REPORTO!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
								//cout<<"Compatible side Pair ! "<<endl;
								//cout<<"Base Point1: "<<t1->getAncoratge()<<" Size1: "<<t1->getMida()<<endl;
								//cout<<"Base Point2: "<<t2->getAncoratge()<<" Size2: "<<t2->getMida()<<endl;
							}

						} else {
							//Cap parcial (2 fulles negres)
	
							// Let's store the candidate zone
							CompressedCandidateZone *cand = new CompressedCandidateZone(2,t1,t2);
							candidates.push_back(cand);	
							t1->setNodeZones(cand);
								t2->setNodeZones(cand);
						
			//cout<<"                                                                         ";
							//cout<<"REPORTO!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
							//cout<<"Compatible side Pair ! "<<endl;
							//cout<<"Base Point1: "<<t1->getAncoratge()<<" Size1: "<<t1->getMida()<<endl;
							//cout<<"Base Point2: "<<t2->getAncoratge()<<" Size2: "<<t2->getMida()<<endl;
						}
					} else {
						//No estan de costat
					
						//CALDRIA MIRAR ELS 12 CASOS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	
						// Let's store the candidate zone
						CompressedCandidateZone *cand = new CompressedCandidateZone(2,t1,t2);
						candidates.push_back(cand);	
						t1->setNodeZones(cand);
						t2->setNodeZones(cand);
					
			//cout<<"                                                                         ";
						//cout<<"REPORTO!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
						//cout<<"Compatible side Pair ! "<<endl;
						//cout<<"Base Point1: "<<t1->getAncoratge()<<" Size1: "<<t1->getMida()<<endl;
						//cout<<"Base Point2: "<<t2->getAncoratge()<<" Size2: "<<t2->getMida()<<endl;
					}
				}
			}
		}
	}

//cout<<"surto de search 2x"<<endl;
}

/* Function that searches in pairs of Y neighbors
1
2
*/
void CompressedOctree::intern_search_2y(CompressedInformacioGeometrica info, CompressedONode * t1, CompressedONode * t2, bool bNum, bool bHisto, bool bDist)
{
	//cout<<endl<<endl<<endl;
	//cout<<"entro a search_2y amb node1 i mida1: "<<t1->getAncoratge()<<" "<<t1->getMida()<<" i node2 i mida 2  "<<t2->getAncoratge()<<" "<<t2->getMida()<<endl;

//char c;
//cin>>c;

	//Test for compatibility
	if ( (esToquen2(info,t1,t2)) && (CompressedONode::compatible(info,calcularInfo2(t1->getInfGeo(),t2->getInfGeo()), bNum, bHisto, bDist,epsilon)) ) {
		
		//Agafem la mida mes gran dels que siguin parcials
		bool parcial1 = t1->tipus()==2;
		bool parcial2 = t2->tipus()==2;
		double mida = 0.0;
		if (parcial1) mida = t1->getMida();
		if ((parcial2) && (t2->getMida()>mida)) mida = t2->getMida();

		if ( (mida/2.0)<info.getMidaNode() ) {

			// Let's store the candidate zone
			CompressedCandidateZone *cand = new CompressedCandidateZone(2,t1,t2);
			candidates.push_back(cand);	
			t1->setNodeZones(cand);
			t2->setNodeZones(cand);
		
			//cout<<"REPORTO!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
			//cout<<"Compatible side Pair ! "<<endl;
			//cout<<"Base Point1: "<<t1->getAncoratge()<<" Size1: "<<t1->getMida()<<endl;
			//cout<<"Base Point2: "<<t2->getAncoratge()<<" Size2: "<<t2->getMida()<<endl;
			
			//aqui tb falta pintar	
		}
		else
		{
			if ( t1->getMida()==t2->getMida() ) {
				//Tenen la mateixa mida
				
				if ( deCostat2(t1,t2,2) ) {
					//Estan de costat
	
					if (parcial1 && parcial2) {
						//Tots 2 parcials

						//Recursively call Search function
						bool ok1,ok2,ok3,ok4,ok5,ok6,ok7,ok8;
			
						ok1 = (t1->getFill3()!=NULL) && ((t1->getFill3())->tipus()!=0);
						ok2 = (t1->getFill4()!=NULL) && ((t1->getFill4())->tipus()!=0);
						ok3 = (t2->getFill1()!=NULL) && ((t2->getFill1())->tipus()!=0);
						ok4 = (t2->getFill2()!=NULL) && ((t2->getFill2())->tipus()!=0);
						ok5 = (t1->getFill7()!=NULL) && ((t1->getFill7())->tipus()!=0);
						ok6 = (t1->getFill8()!=NULL) && ((t1->getFill8())->tipus()!=0);
						ok7 = (t2->getFill5()!=NULL) && ((t2->getFill5())->tipus()!=0);
						ok8 = (t2->getFill6()!=NULL) && ((t2->getFill6())->tipus()!=0);
						
						//cout<<"     Crides recursives a search_2 desde 2"<<endl;
						// Search pairs of Y neighbors
						// Beware of white leaved couples
						if ( (ok1) && (ok3) ) intern_search_2y(info,t1->getFill3(),t2->getFill1(),bNum,bHisto,bDist);
						if ( (ok2) && (ok4) ) intern_search_2y(info,t1->getFill4(),t2->getFill2(),bNum,bHisto,bDist);
						if ( (ok5) && (ok7) ) intern_search_2y(info,t1->getFill7(),t2->getFill5(),bNum,bHisto,bDist);
						if ( (ok6) && (ok8) ) intern_search_2y(info,t1->getFill8(),t2->getFill6(),bNum,bHisto,bDist);
						
						// Abans de llençar les cerques, descomprimim				
        				CompressedONode nodeAux1 = CompressedONode(t1->getAncoratge(),t1->getMida(),t1->getNivell() );
        				vector<bool> fillsTocats1 = descomprimeix(t1,&nodeAux1);	
        				CompressedONode nodeAux2 = CompressedONode(t2->getAncoratge(),t2->getMida(),t2->getNivell() );
        				vector<bool> fillsTocats2 = descomprimeix(t2,&nodeAux2);
        				
						//3. Search quartets
						call_intern_search_4(info,ok1,ok2,ok3,ok4,nodeAux1.getFill3(),nodeAux1.getFill4(),nodeAux2.getFill1(),nodeAux2.getFill2(),bNum,bHisto,bDist,1);
						call_intern_search_4(info,ok5,ok6,ok7,ok8,nodeAux1.getFill7(),nodeAux1.getFill8(),nodeAux2.getFill5(),nodeAux2.getFill6(),bNum,bHisto,bDist,1);
						call_intern_search_4(info,ok1,ok3,ok5,ok7,nodeAux1.getFill3(),nodeAux2.getFill1(),nodeAux1.getFill7(),nodeAux2.getFill5(),bNum,bHisto,bDist,3);
						call_intern_search_4(info,ok2,ok4,ok6,ok8,nodeAux1.getFill4(),nodeAux2.getFill2(),nodeAux1.getFill8(),nodeAux2.getFill6(),bNum,bHisto,bDist,3);
			
						//4. Search octet
						call_intern_search_8(info,ok1,ok2,ok3,ok4,ok5,ok6,ok7,ok8,nodeAux1.getFill3(),nodeAux1.getFill4(),nodeAux2.getFill1(),nodeAux2.getFill2(),nodeAux1.getFill7(),nodeAux1.getFill8(),nodeAux2.getFill5(),nodeAux2.getFill6(),bNum,bHisto,bDist);
					
        				// Recomprimeix per no perdre memoria
        				recomprimeix(fillsTocats1, &nodeAux1);
        				recomprimeix(fillsTocats2, &nodeAux2);
        				
					} else if (parcial1) {
						//Nomes 1r parcial

						//Mirem si creant un node orfe al costat del node parcial
						//aconseguim "atrapar" algun element
						double dif = (t1->getAncoratge().getY()-t2->getAncoratge().getY());
						double difMax = t1->getMida()+1.0e-6;
	
						if ( (dif<=difMax) && (deCostat2(t1,t2,2)) ) {
							//CALDRIA FER NODE ORFEEEEEEEEEEEEEEEEEEEEEEEEE!!!!!!!!!!!!!!!!!!!!!!!!!!
							// Let's store the candidate zone
							CompressedCandidateZone *cand = new CompressedCandidateZone(2,t1,t2);
							candidates.push_back(cand);	
							t1->setNodeZones(cand);
							t2->setNodeZones(cand);
						
							//cout<<"REPORTO!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
							//cout<<"Compatible side Pair ! "<<endl;
							//cout<<"Base Point1: "<<t1->getAncoratge()<<" Size1: "<<t1->getMida()<<endl;
							//cout<<"Base Point2: "<<t2->getAncoratge()<<" Size2: "<<t2->getMida()<<endl;
						} else {
							// Let's store the candidate zone
							CompressedCandidateZone *cand = new CompressedCandidateZone(2,t1,t2);
							candidates.push_back(cand);	
							t1->setNodeZones(cand);
							t2->setNodeZones(cand);
						
							//cout<<"REPORTO!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
							//cout<<"Compatible side Pair ! "<<endl;
							//cout<<"Base Point1: "<<t1->getAncoratge()<<" Size1: "<<t1->getMida()<<endl;
							//cout<<"Base Point2: "<<t2->getAncoratge()<<" Size2: "<<t2->getMida()<<endl;
						}

					} else if (parcial2) {
						//Nomes 2n parcial

						//Mirem si creant un node orfe al costat del node parcial
						//aconseguim "atrapar" algun element
						double dif = (t1->getAncoratge().getY()-t2->getAncoratge().getY());
						double difMax = (2.0*t2->getMida())-1.0e-6;
	
						if ( (dif<difMax) && (deCostat2(t1,t2,2)) ) {
							//CALDRIA FER NODE ORFEEEEEEEEEEEEEEEEEEEEEEEEE!!!!!!!!!!!!!!!!!!!!!!!!!!
							// Let's store the candidate zone
							CompressedCandidateZone *cand = new CompressedCandidateZone(2,t1,t2);
							candidates.push_back(cand);	
							t1->setNodeZones(cand);
							t2->setNodeZones(cand);
						
							//cout<<"REPORTO!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
							//cout<<"Compatible side Pair ! "<<endl;
							//cout<<"Base Point1: "<<t1->getAncoratge()<<" Size1: "<<t1->getMida()<<endl;
							//cout<<"Base Point2: "<<t2->getAncoratge()<<" Size2: "<<t2->getMida()<<endl;
						} else {
							// Let's store the candidate zone
							CompressedCandidateZone *cand = new CompressedCandidateZone(2,t1,t2);
							candidates.push_back(cand);	
							t1->setNodeZones(cand);
							t2->setNodeZones(cand);
						
							//cout<<"REPORTO!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
							//cout<<"Compatible side Pair ! "<<endl;
							//cout<<"Base Point1: "<<t1->getAncoratge()<<" Size1: "<<t1->getMida()<<endl;
							//cout<<"Base Point2: "<<t2->getAncoratge()<<" Size2: "<<t2->getMida()<<endl;
						}

					} else {
						//Cap parcial (2 fulles negres)

						// Let's store the candidate zone
						CompressedCandidateZone *cand = new CompressedCandidateZone(2,t1,t2);
						candidates.push_back(cand);	
						t1->setNodeZones(cand);
						t2->setNodeZones(cand);
					
						//cout<<"REPORTO!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
						//cout<<"Compatible side Pair ! "<<endl;
						//cout<<"Base Point1: "<<t1->getAncoratge()<<" Size1: "<<t1->getMida()<<endl;
						//cout<<"Base Point2: "<<t2->getAncoratge()<<" Size2: "<<t2->getMida()<<endl;
					}
				} else {
					//No estan de costat
					
					//CALDRIA MIRAR ELS 12 CASOS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

					// Let's store the candidate zone
					CompressedCandidateZone *cand = new CompressedCandidateZone(2,t1,t2);
					candidates.push_back(cand);	
					t1->setNodeZones(cand);
					t2->setNodeZones(cand);
				
					//cout<<"REPORTO!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
					//cout<<"Compatible side Pair ! "<<endl;
					//cout<<"Base Point1: "<<t1->getAncoratge()<<" Size1: "<<t1->getMida()<<endl;
					//cout<<"Base Point2: "<<t2->getAncoratge()<<" Size2: "<<t2->getMida()<<endl;
				}

			} else {
				//No tenen la mateixa mida

				if (t1->getMida()>t2->getMida()) {
					//t1 es mes gran
					
					int escombra = quiEscombra2(t1,t2,2);
					if ( escombra!=-1 ) {
						//Estan de costat (algun fill de t1 escombra a t2)
						
						if (parcial1) {
							//t1 es parcial
							
							bool ok1,ok2,ok3,ok4,ok5,ok6,ok7,ok8;
			
							ok1 = (t1->getFill3()!=NULL) && ((t1->getFill3())->tipus()!=0);
							ok2 = (t1->getFill4()!=NULL) && ((t1->getFill4())->tipus()!=0);
							ok3 = false;
							ok4 = false;
							ok5 = (t1->getFill7()!=NULL) && ((t1->getFill7())->tipus()!=0);
							ok6 = (t1->getFill8()!=NULL) && ((t1->getFill8())->tipus()!=0);
							ok7 = false;
							ok8 = false;

							CompressedONode *t2_1 = NULL;
							CompressedONode *t2_2 = NULL;
							CompressedONode *t2_5 = NULL;
							CompressedONode *t2_6 = NULL;

							switch (escombra) {
								case 3:
									//El fill 2 del node 1 escombra el node 2
									ok3 = true;
									t2_1 = t2;
									break;
								case 4:
									//El fill 4 del node 1 escombra el node 2
									ok4 = true;
									t2_2 = t2;
									break;
								case 7:
									//El fill 7 del node 1 escombra el node 2
									ok7 = true;
									t2_5 = t2;
									break;
								case 8:
									//El fill 8 del node 1 escombra el node 2
									ok8 = true;
									t2_6 = t2;
									break;
								default:
									cout << "ERROR (CompressedOctree ::intern_search_2y) : La escombra s'ha equivocat!" << endl;
									exit(-1);
									break;
							}
							
							//Cridem search_2
							if ( (ok1) && (ok3) ) intern_search_2y(info,t1->getFill3(),t2_1,bNum,bHisto,bDist);
    						if ( (ok2) && (ok4) ) intern_search_2y(info,t1->getFill4(),t2_2,bNum,bHisto,bDist);
    						if ( (ok5) && (ok7) ) intern_search_2y(info,t1->getFill7(),t2_5,bNum,bHisto,bDist);
    						if ( (ok6) && (ok8) ) intern_search_2y(info,t1->getFill8(),t2_6,bNum,bHisto,bDist);
    						
    						//Cridem search_4
    						call_intern_search_4(info,ok1,ok2,ok3,ok4,t1->getFill3(),t1->getFill4(),t2_1,t2_2,bNum,bHisto,bDist,1);
    						call_intern_search_4(info,ok5,ok6,ok7,ok8,t1->getFill7(),t1->getFill8(),t2_5,t2_6,bNum,bHisto,bDist,1);
    						call_intern_search_4(info,ok1,ok3,ok5,ok7,t1->getFill3(),t2_1,t1->getFill7(),t2_5,bNum,bHisto,bDist,3);
    						call_intern_search_4(info,ok2,ok4,ok6,ok8,t1->getFill4(),t2_2,t1->getFill8(),t2_6,bNum,bHisto,bDist,3);
    			
    						//Cridem search_8
    						call_intern_search_8(info,ok1,ok2,ok3,ok4,ok5,ok6,ok7,ok8,t1->getFill3(),t1->getFill4(),t2_1,t2_2,t1->getFill7(),t1->getFill8(),t2_5,t2_6,bNum,bHisto,bDist);
					
						} else if (parcial2) {
							//Nomes t2 es parcial

							//Mirem si creant un node orfe al costat del node parcial
							//aconseguim "atrapar" algun element
							double dif = (t1->getAncoratge().getY()-t2->getAncoratge().getY());
							double difMax = (2.0*t2->getMida())-1.0e-6;
		
							if ( (dif<difMax) && (quiEscombra2(t1,t2,2)!=-1) ) {
								//CALDRIA FER NODE ORFEEEEEEEEEEEEEEEEEEEEEEEEE!!!!!!!!!!!!!!!!!!!!!!!!!!
								// Let's store the candidate zone
								CompressedCandidateZone *cand = new CompressedCandidateZone(2,t1,t2);
								candidates.push_back(cand);	
								t1->setNodeZones(cand);
								t2->setNodeZones(cand);
							
								//cout<<"REPORTO!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
								//cout<<"Compatible side Pair ! "<<endl;
								//cout<<"Base Point1: "<<t1->getAncoratge()<<" Size1: "<<t1->getMida()<<endl;
								//cout<<"Base Point2: "<<t2->getAncoratge()<<" Size2: "<<t2->getMida()<<endl;
							} else {
								// Let's store the candidate zone
								CompressedCandidateZone *cand = new CompressedCandidateZone(2,t1,t2);
								candidates.push_back(cand);	
								t1->setNodeZones(cand);
								t2->setNodeZones(cand);
							
								//cout<<"REPORTO!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
								//cout<<"Compatible side Pair ! "<<endl;
								//cout<<"Base Point1: "<<t1->getAncoratge()<<" Size1: "<<t1->getMida()<<endl;
								//cout<<"Base Point2: "<<t2->getAncoratge()<<" Size2: "<<t2->getMida()<<endl;
							}
						} else {
							//Cap parcial (2 fulles negres)
	
							// Let's store the candidate zone
							CompressedCandidateZone *cand = new CompressedCandidateZone(2,t1,t2);
							candidates.push_back(cand);	
							t1->setNodeZones(cand);
								t2->setNodeZones(cand);
						
							//cout<<"REPORTO!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
							//cout<<"Compatible side Pair ! "<<endl;
							//cout<<"Base Point1: "<<t1->getAncoratge()<<" Size1: "<<t1->getMida()<<endl;
							//cout<<"Base Point2: "<<t2->getAncoratge()<<" Size2: "<<t2->getMida()<<endl;
						}
					} else {
						//No estan de costat
					
						//CALDRIA MIRAR ELS 12 CASOS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	
						// Let's store the candidate zone
						CompressedCandidateZone *cand = new CompressedCandidateZone(2,t1,t2);
						candidates.push_back(cand);	
						t1->setNodeZones(cand);
						t2->setNodeZones(cand);
					
						//cout<<"REPORTO!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
						//cout<<"Compatible side Pair ! "<<endl;
						//cout<<"Base Point1: "<<t1->getAncoratge()<<" Size1: "<<t1->getMida()<<endl;
						//cout<<"Base Point2: "<<t2->getAncoratge()<<" Size2: "<<t2->getMida()<<endl;
					}

				} else {
					//t2 es mes gran
					
					int escombra = quiEscombra2(t1,t2,2);
					if ( escombra!=-1 ) {
						//Estan de costat (algun fill de t2 escombra a t1)
						
                    	if (parcial2) {
							//t2 es parcial

							bool ok1,ok2,ok3,ok4,ok5,ok6,ok7,ok8;
			
							ok1 = false;
							ok2 = false;
							ok3 = (t2->getFill1()!=NULL) && ((t2->getFill1())->tipus()!=0);
							ok4 = (t2->getFill2()!=NULL) && ((t2->getFill2())->tipus()!=0);
							ok5 = false;
							ok6 = false;
							ok7 = (t2->getFill5()!=NULL) && ((t2->getFill5())->tipus()!=0);
							ok8 = (t2->getFill6()!=NULL) && ((t2->getFill6())->tipus()!=0);

							CompressedONode *t1_3 = NULL;
							CompressedONode *t1_4 = NULL;
							CompressedONode *t1_7 = NULL;
							CompressedONode *t1_8 = NULL;

							switch (escombra) {
								case 1:
									//El fill 1 del node 2 escombra el node 1
									ok1 = true;
									t1_3 = t1;
									break;
								case 2:
									//El fill 2 del node 2 escombra el node 1
									ok2 = true;
									t1_4 = t1;
									break;
								case 5:
									//El fill 5 del node 2 escombra el node 1
									ok5 = true;
									t1_7 = t1;
									break;
								case 6:
									//El fill 6 del node 2 escombra el node 1
									ok6 = true;
									t1_8 = t1;
									break;
								default:
									cout << "ERROR (CompressedOctree ::intern_search_2y) : La escombra s'ha equivocat!" << endl;
									exit(-1);
									break;
							}
							
							
    						//Cridem search_2
    						if ( (ok1) && (ok3) ) intern_search_2y(info,t1_3,t2->getFill1(),bNum,bHisto,bDist);
    						if ( (ok2) && (ok4) ) intern_search_2y(info,t1_4,t2->getFill2(),bNum,bHisto,bDist);
    						if ( (ok5) && (ok7) ) intern_search_2y(info,t1_7,t2->getFill5(),bNum,bHisto,bDist);
    						if ( (ok6) && (ok8) ) intern_search_2y(info,t1_8,t2->getFill6(),bNum,bHisto,bDist);
    						
    						//Cridem search_4
    						call_intern_search_4(info,ok1,ok2,ok3,ok4,t1_3,t1_4,t2->getFill1(),t2->getFill2(),bNum,bHisto,bDist,1);
    						call_intern_search_4(info,ok5,ok6,ok7,ok8,t1_7,t1_8,t2->getFill5(),t2->getFill6(),bNum,bHisto,bDist,1);
    						call_intern_search_4(info,ok1,ok3,ok5,ok7,t1_3,t2->getFill1(),t1_7,t2->getFill5(),bNum,bHisto,bDist,3);
    						call_intern_search_4(info,ok2,ok4,ok6,ok8,t1_4,t2->getFill2(),t1_8,t2->getFill6(),bNum,bHisto,bDist,3);
    			
    						//Cridem search_8
    						call_intern_search_8(info,ok1,ok2,ok3,ok4,ok5,ok6,ok7,ok8,t1_3,t1_4,t2->getFill1(),t2->getFill2(),t1_7,t1_8,t2->getFill5(),t2->getFill6(),bNum,bHisto,bDist);
					
						} else if (parcial1) {
							//Nomes t1 es parcial

							//Mirem si creant un node orfe al costat del node parcial
							//aconseguim "atrapar" algun element
							double dif = (t1->getAncoratge().getY()-t2->getAncoratge().getY());
							double difMax = t1->getMida()+1.0e-6;
		
							if ( (dif<=difMax) && (quiEscombra2(t1,t2,2)!=-1) ) {
								//CALDRIA FER NODE ORFEEEEEEEEEEEEEEEEEEEEEEEEE!!!!!!!!!!!!!!!!!!!!!!!!!!
								// Let's store the candidate zone
								CompressedCandidateZone *cand = new CompressedCandidateZone(2,t1,t2);
								candidates.push_back(cand);	
								t1->setNodeZones(cand);
								t2->setNodeZones(cand);
							
								//cout<<"REPORTO!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
								//cout<<"Compatible side Pair ! "<<endl;
								//cout<<"Base Point1: "<<t1->getAncoratge()<<" Size1: "<<t1->getMida()<<endl;
								//cout<<"Base Point2: "<<t2->getAncoratge()<<" Size2: "<<t2->getMida()<<endl;
							} else {
								// Let's store the candidate zone
								CompressedCandidateZone *cand = new CompressedCandidateZone(2,t1,t2);
								candidates.push_back(cand);	
								t1->setNodeZones(cand);
								t2->setNodeZones(cand);
							
								//cout<<"REPORTO!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
								//cout<<"Compatible side Pair ! "<<endl;
								//cout<<"Base Point1: "<<t1->getAncoratge()<<" Size1: "<<t1->getMida()<<endl;
								//cout<<"Base Point2: "<<t2->getAncoratge()<<" Size2: "<<t2->getMida()<<endl;
							}

						} else {
							//Cap parcial (2 fulles negres)
	
							// Let's store the candidate zone
							CompressedCandidateZone *cand = new CompressedCandidateZone(2,t1,t2);
							candidates.push_back(cand);	
							t1->setNodeZones(cand);
								t2->setNodeZones(cand);
						
							//cout<<"REPORTO!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
							//cout<<"Compatible side Pair ! "<<endl;
							//cout<<"Base Point1: "<<t1->getAncoratge()<<" Size1: "<<t1->getMida()<<endl;
							//cout<<"Base Point2: "<<t2->getAncoratge()<<" Size2: "<<t2->getMida()<<endl;
						}
					} else {
						//No estan de costat
					
						//CALDRIA MIRAR ELS 12 CASOS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	
						// Let's store the candidate zone
						CompressedCandidateZone *cand = new CompressedCandidateZone(2,t1,t2);
						candidates.push_back(cand);	
						t1->setNodeZones(cand);
						t2->setNodeZones(cand);
					
						//cout<<"REPORTO!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
						//cout<<"Compatible side Pair ! "<<endl;
						//cout<<"Base Point1: "<<t1->getAncoratge()<<" Size1: "<<t1->getMida()<<endl;
						//cout<<"Base Point2: "<<t2->getAncoratge()<<" Size2: "<<t2->getMida()<<endl;
					}
				}
			}
		}
	}

//cout<<"surto de search 2y"<<endl;
}

/* Function that searches in pairs of Z neighbors
 2 
1
*/
void CompressedOctree::intern_search_2z(CompressedInformacioGeometrica info, CompressedONode * t1, CompressedONode * t2, bool bNum, bool bHisto, bool bDist)
{
	//cout<<endl<<endl<<endl;
	//cout<<"entro a search_2z amb node1 i mida1: "<<t1->getAncoratge()<<" "<<t1->getMida()<<" i node2 i mida 2  "<<t2->getAncoratge()<<" "<<t2->getMida()<<endl;

//char c;
//cin>>c;

	//Test for compatibility
	if ( (esToquen2(info,t1,t2)) && (CompressedONode::compatible(info,calcularInfo2(t1->getInfGeo(),t2->getInfGeo()), bNum, bHisto, bDist,epsilon)) ) {
		
		//Agafem la mida mes gran dels que siguin parcials
		bool parcial1 = t1->tipus()==2;
		bool parcial2 = t2->tipus()==2;
		double mida = 0.0;
		if (parcial1) mida = t1->getMida();
		if ((parcial2) && (t2->getMida()>mida)) mida = t2->getMida();

		if ( (mida/2.0)<info.getMidaNode() ) {

			// Let's store the candidate zone
			CompressedCandidateZone *cand = new CompressedCandidateZone(2,t1,t2);
			candidates.push_back(cand);	
			t1->setNodeZones(cand);
			t2->setNodeZones(cand);
		
			//cout<<"REPORTO!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
			//cout<<"Compatible side Pair ! "<<endl;
			//cout<<"Base Point1: "<<t1->getAncoratge()<<" Size1: "<<t1->getMida()<<endl;
			//cout<<"Base Point2: "<<t2->getAncoratge()<<" Size2: "<<t2->getMida()<<endl;
			
			//aqui tb falta pintar	
		}
		else
		{
			if ( t1->getMida()==t2->getMida() ) {
				//Tenen la mateixa mida
				
				if ( deCostat2(t1,t2,3) ) {
					//Estan de costat
	
					if (parcial1 && parcial2) {
						//Tots 2 parcials

						//Recursively call Search function
						bool ok1,ok2,ok3,ok4,ok5,ok6,ok7,ok8;
			
						ok1 = (t1->getFill5()!=NULL) && ((t1->getFill5())->tipus()!=0);
						ok2 = (t1->getFill6()!=NULL) && ((t1->getFill6())->tipus()!=0);
						ok3 = (t1->getFill7()!=NULL) && ((t1->getFill7())->tipus()!=0);
						ok4 = (t1->getFill8()!=NULL) && ((t1->getFill8())->tipus()!=0);
						ok5 = (t2->getFill1()!=NULL) && ((t2->getFill1())->tipus()!=0);
						ok6 = (t2->getFill2()!=NULL) && ((t2->getFill2())->tipus()!=0);
						ok7 = (t2->getFill3()!=NULL) && ((t2->getFill3())->tipus()!=0);
						ok8 = (t2->getFill4()!=NULL) && ((t2->getFill4())->tipus()!=0);
						
						//cout<<"     Crides recursives a search_2 desde 2"<<endl;
						// Search pairs of Z neighbors
						// Beware of white leaved couples
						if ( (ok1) && (ok5) ) intern_search_2z(info,t1->getFill5(),t2->getFill1(),bNum,bHisto,bDist);
						if ( (ok2) && (ok6) ) intern_search_2z(info,t1->getFill6(),t2->getFill2(),bNum,bHisto,bDist);
						if ( (ok3) && (ok7) ) intern_search_2z(info,t1->getFill7(),t2->getFill3(),bNum,bHisto,bDist);
						if ( (ok4) && (ok8) ) intern_search_2z(info,t1->getFill8(),t2->getFill4(),bNum,bHisto,bDist);
			
			            // Abans de llençar les cerques, descomprimim				
        				CompressedONode nodeAux1 = CompressedONode(t1->getAncoratge(),t1->getMida(),t1->getNivell() );
        				vector<bool> fillsTocats1 = descomprimeix(t1,&nodeAux1);	
        				CompressedONode nodeAux2 = CompressedONode(t2->getAncoratge(),t2->getMida(),t2->getNivell() );
        				vector<bool> fillsTocats2 = descomprimeix(t2,&nodeAux2);
        				
						//3. Search quartets
						call_intern_search_4(info,ok1,ok2,ok5,ok6,nodeAux1.getFill5(),nodeAux1.getFill6(),nodeAux2.getFill1(),nodeAux2.getFill2(),bNum,bHisto,bDist,2);
						call_intern_search_4(info,ok3,ok4,ok7,ok8,nodeAux1.getFill7(),nodeAux1.getFill8(),nodeAux2.getFill3(),nodeAux2.getFill4(),bNum,bHisto,bDist,2);
						call_intern_search_4(info,ok1,ok3,ok5,ok7,nodeAux1.getFill5(),nodeAux1.getFill7(),nodeAux2.getFill1(),nodeAux2.getFill3(),bNum,bHisto,bDist,3);
						call_intern_search_4(info,ok2,ok4,ok6,ok8,nodeAux1.getFill6(),nodeAux1.getFill8(),nodeAux2.getFill2(),nodeAux2.getFill4(),bNum,bHisto,bDist,3);
			
						//4. Search octet
						call_intern_search_8(info,ok1,ok2,ok3,ok4,ok5,ok6,ok7,ok8,nodeAux1.getFill5(),nodeAux1.getFill6(),nodeAux1.getFill7(),nodeAux1.getFill8(),nodeAux2.getFill1(),nodeAux2.getFill2(),nodeAux2.getFill3(),nodeAux2.getFill4(),bNum,bHisto,bDist);
			            
        				// Recomprimeix per no perdre memoria
        				recomprimeix(fillsTocats1, &nodeAux1);
        				recomprimeix(fillsTocats2, &nodeAux2);
        				
					} else if (parcial1) {
						//Nomes 1r parcial

						//Mirem si creant un node orfe al costat del node parcial
						//aconseguim "atrapar" algun element
						double dif = (t1->getAncoratge().getZ()-t2->getAncoratge().getZ());
						double difMax = t1->getMida()+1.0e-6;
	
						if ( (dif<=difMax) && (deCostat2(t1,t2,3)) ) {
							//CALDRIA FER NODE ORFEEEEEEEEEEEEEEEEEEEEEEEEE!!!!!!!!!!!!!!!!!!!!!!!!!!
							// Let's store the candidate zone
							CompressedCandidateZone *cand = new CompressedCandidateZone(2,t1,t2);
							candidates.push_back(cand);	
							t1->setNodeZones(cand);
							t2->setNodeZones(cand);
						
							//cout<<"REPORTO!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
							//cout<<"Compatible side Pair ! "<<endl;
							//cout<<"Base Point1: "<<t1->getAncoratge()<<" Size1: "<<t1->getMida()<<endl;
							//cout<<"Base Point2: "<<t2->getAncoratge()<<" Size2: "<<t2->getMida()<<endl;
						} else {
							// Let's store the candidate zone
							CompressedCandidateZone *cand = new CompressedCandidateZone(2,t1,t2);
							candidates.push_back(cand);	
							t1->setNodeZones(cand);
							t2->setNodeZones(cand);
						
							//cout<<"REPORTO!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
							//cout<<"Compatible side Pair ! "<<endl;
							//cout<<"Base Point1: "<<t1->getAncoratge()<<" Size1: "<<t1->getMida()<<endl;
							//cout<<"Base Point2: "<<t2->getAncoratge()<<" Size2: "<<t2->getMida()<<endl;
						}

					} else if (parcial2) {
						//Nomes 2n parcial

						//Mirem si creant un node orfe al costat del node parcial
						//aconseguim "atrapar" algun element
						double dif = (t1->getAncoratge().getZ()-t2->getAncoratge().getZ());
						double difMax = (2.0*t2->getMida())-1.0e-6;
	
						if ( (dif<difMax) && (deCostat2(t1,t2,3)) ) {
							//CALDRIA FER NODE ORFEEEEEEEEEEEEEEEEEEEEEEEEE!!!!!!!!!!!!!!!!!!!!!!!!!!
							// Let's store the candidate zone
							CompressedCandidateZone *cand = new CompressedCandidateZone(2,t1,t2);
							candidates.push_back(cand);	
							t1->setNodeZones(cand);
							t2->setNodeZones(cand);
						
							//cout<<"REPORTO!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
							//cout<<"Compatible side Pair ! "<<endl;
							//cout<<"Base Point1: "<<t1->getAncoratge()<<" Size1: "<<t1->getMida()<<endl;
							//cout<<"Base Point2: "<<t2->getAncoratge()<<" Size2: "<<t2->getMida()<<endl;
						} else {
							// Let's store the candidate zone
							CompressedCandidateZone *cand = new CompressedCandidateZone(2,t1,t2);
							candidates.push_back(cand);	
							t1->setNodeZones(cand);
							t2->setNodeZones(cand);
						
							//cout<<"REPORTO!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
							//cout<<"Compatible side Pair ! "<<endl;
							//cout<<"Base Point1: "<<t1->getAncoratge()<<" Size1: "<<t1->getMida()<<endl;
							//cout<<"Base Point2: "<<t2->getAncoratge()<<" Size2: "<<t2->getMida()<<endl;
						}

					} else {
						//Cap parcial (2 fulles negres)

						// Let's store the candidate zone
						CompressedCandidateZone *cand = new CompressedCandidateZone(2,t1,t2);
						candidates.push_back(cand);	
						t1->setNodeZones(cand);
						t2->setNodeZones(cand);
					
						//cout<<"REPORTO!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
						//cout<<"Compatible side Pair ! "<<endl;
						//cout<<"Base Point1: "<<t1->getAncoratge()<<" Size1: "<<t1->getMida()<<endl;
						//cout<<"Base Point2: "<<t2->getAncoratge()<<" Size2: "<<t2->getMida()<<endl;
					}
				} else {
					//No estan de costat
					
					//CALDRIA MIRAR ELS 12 CASOS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

					// Let's store the candidate zone
					CompressedCandidateZone *cand = new CompressedCandidateZone(2,t1,t2);
					candidates.push_back(cand);	
					t1->setNodeZones(cand);
					t2->setNodeZones(cand);
				
					//cout<<"REPORTO!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
					//cout<<"Compatible side Pair ! "<<endl;
					//cout<<"Base Point1: "<<t1->getAncoratge()<<" Size1: "<<t1->getMida()<<endl;
					//cout<<"Base Point2: "<<t2->getAncoratge()<<" Size2: "<<t2->getMida()<<endl;
				}

			} else {
				//No tenen la mateixa mida

				if (t1->getMida()>t2->getMida()) {
					//t1 es mes gran
					
					int escombra = quiEscombra2(t1,t2,3);
					if ( escombra!=-1 ) {
						//Estan de costat (algun fill de t1 escombra a t2)
						
						if (parcial1) {
							//t1 es parcial
							
							bool ok1,ok2,ok3,ok4,ok5,ok6,ok7,ok8;
			
							ok1 = (t1->getFill5()!=NULL) && ((t1->getFill5())->tipus()!=0);
							ok2 = (t1->getFill6()!=NULL) && ((t1->getFill6())->tipus()!=0);
							ok3 = (t1->getFill7()!=NULL) && ((t1->getFill7())->tipus()!=0);
							ok4 = (t1->getFill8()!=NULL) && ((t1->getFill8())->tipus()!=0);
							ok5 = false;
							ok6 = false;
							ok7 = false;
							ok8 = false;

							CompressedONode *t2_1 = NULL;
							CompressedONode *t2_2 = NULL;
							CompressedONode *t2_3 = NULL;
							CompressedONode *t2_4 = NULL;

							switch (escombra) {
								case 5:
									//El fill 5 del node 1 escombra el node 2
									ok5 = true;
									t2_1 = t2;
									break;
								case 6:
									//El fill 6 del node 1 escombra el node 2
									ok6 = true;
									t2_2 = t2;
									break;
								case 7:
									//El fill 7 del node 1 escombra el node 2
									ok7 = true;
									t2_3 = t2;
									break;
								case 8:
									//El fill 8 del node 1 escombra el node 2
									ok8 = true;
									t2_4 = t2;
									break;
								default:
									cout << "ERROR (CompressedOctree ::intern_search_2z) : La escombra s'ha equivocat!" << endl;
									exit(-1);
									break;
							}
							
							//Cridem search_2
							if ( (ok1) && (ok5) ) intern_search_2z(info,t1->getFill5(),t2_1,bNum,bHisto,bDist);
    						if ( (ok2) && (ok6) ) intern_search_2z(info,t1->getFill6(),t2_2,bNum,bHisto,bDist);
    						if ( (ok3) && (ok7) ) intern_search_2z(info,t1->getFill7(),t2_3,bNum,bHisto,bDist);
    						if ( (ok4) && (ok8) ) intern_search_2z(info,t1->getFill8(),t2_4,bNum,bHisto,bDist);
 						
    						//3. Search quartets
    						call_intern_search_4(info,ok1,ok2,ok5,ok6,t1->getFill5(),t1->getFill6(),t2_1,t2_2,bNum,bHisto,bDist,2);
    						call_intern_search_4(info,ok3,ok4,ok7,ok8,t1->getFill7(),t1->getFill8(),t2_3,t2_4,bNum,bHisto,bDist,2);
    						call_intern_search_4(info,ok1,ok3,ok5,ok7,t1->getFill5(),t1->getFill7(),t2_1,t2_3,bNum,bHisto,bDist,3);
    						call_intern_search_4(info,ok2,ok4,ok6,ok8,t1->getFill6(),t1->getFill8(),t2_2,t2_4,bNum,bHisto,bDist,3);
    			
    						//4. Search octet
    						call_intern_search_8(info,ok1,ok2,ok3,ok4,ok5,ok6,ok7,ok8,t1->getFill5(),t1->getFill6(),t1->getFill7(),t1->getFill8(),t2_1,t2_2,t2_3,t2_4,bNum,bHisto,bDist);
					
						} else if (parcial2) {
							//Nomes t2 es parcial

							//Mirem si creant un node orfe al costat del node parcial
							//aconseguim "atrapar" algun element
							double dif = (t1->getAncoratge().getZ()-t2->getAncoratge().getZ());
							double difMax = (2.0*t2->getMida())-1.0e-6;
		
							if ( (dif<difMax) && (quiEscombra2(t1,t2,3)!=-1) ) {
								//CALDRIA FER NODE ORFEEEEEEEEEEEEEEEEEEEEEEEEE!!!!!!!!!!!!!!!!!!!!!!!!!!
								// Let's store the candidate zone
								CompressedCandidateZone *cand = new CompressedCandidateZone(2,t1,t2);
								candidates.push_back(cand);	
								t1->setNodeZones(cand);
								t2->setNodeZones(cand);
							
								//cout<<"REPORTO!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
								//cout<<"Compatible side Pair ! "<<endl;
								//cout<<"Base Point1: "<<t1->getAncoratge()<<" Size1: "<<t1->getMida()<<endl;
								//cout<<"Base Point2: "<<t2->getAncoratge()<<" Size2: "<<t2->getMida()<<endl;
							} else {
								// Let's store the candidate zone
								CompressedCandidateZone *cand = new CompressedCandidateZone(2,t1,t2);
								candidates.push_back(cand);	
								t1->setNodeZones(cand);
								t2->setNodeZones(cand);
							
								//cout<<"REPORTO!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
								//cout<<"Compatible side Pair ! "<<endl;
								//cout<<"Base Point1: "<<t1->getAncoratge()<<" Size1: "<<t1->getMida()<<endl;
								//cout<<"Base Point2: "<<t2->getAncoratge()<<" Size2: "<<t2->getMida()<<endl;
							}
						} else {
							//Cap parcial (2 fulles negres)
	
							// Let's store the candidate zone
							CompressedCandidateZone *cand = new CompressedCandidateZone(2,t1,t2);
							candidates.push_back(cand);	
							t1->setNodeZones(cand);
								t2->setNodeZones(cand);
						
							//cout<<"REPORTO!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
							//cout<<"Compatible side Pair ! "<<endl;
							//cout<<"Base Point1: "<<t1->getAncoratge()<<" Size1: "<<t1->getMida()<<endl;
							//cout<<"Base Point2: "<<t2->getAncoratge()<<" Size2: "<<t2->getMida()<<endl;
						}
					} else {
						//No estan de costat
					
						//CALDRIA MIRAR ELS 12 CASOS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	
						// Let's store the candidate zone
						CompressedCandidateZone *cand = new CompressedCandidateZone(2,t1,t2);
						candidates.push_back(cand);	
						t1->setNodeZones(cand);
						t2->setNodeZones(cand);
					
						//cout<<"REPORTO!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
						//cout<<"Compatible side Pair ! "<<endl;
						//cout<<"Base Point1: "<<t1->getAncoratge()<<" Size1: "<<t1->getMida()<<endl;
						//cout<<"Base Point2: "<<t2->getAncoratge()<<" Size2: "<<t2->getMida()<<endl;
					}

				} else {
					//t2 es mes gran
					
					int escombra = quiEscombra2(t1,t2,3);
					if ( escombra!=-1 ) {
						//Estan de costat (algun fill de t2 escombra a t1)
					
                    	if (parcial2) {
							//t2 es parcial

							bool ok1,ok2,ok3,ok4,ok5,ok6,ok7,ok8;
			
							ok1 = false;
							ok2 = false;
							ok3 = false;
							ok4 = false;
							ok5 = (t2->getFill1()!=NULL) && ((t2->getFill1())->tipus()!=0);
							ok6 = (t2->getFill2()!=NULL) && ((t2->getFill2())->tipus()!=0);
							ok7 = (t2->getFill3()!=NULL) && ((t2->getFill3())->tipus()!=0);
							ok8 = (t2->getFill4()!=NULL) && ((t2->getFill4())->tipus()!=0);

							CompressedONode *t1_5 = NULL;
							CompressedONode *t1_6 = NULL;
							CompressedONode *t1_7 = NULL;
							CompressedONode *t1_8 = NULL;

							switch (escombra) {
								case 1:
									//El fill 1 del node 2 escombra el node 1
									ok1 = true;
									t1_5 = t1;
									break;
								case 2:
									//El fill 2 del node 2 escombra el node 1
									ok2 = true;
									t1_6 = t1;
									break;
								case 3:
									//El fill 3 del node 2 escombra el node 1
									ok3 = true;
									t1_7 = t1;
									break;
								case 4:
									//El fill 4 del node 2 escombra el node 1
									ok4 = true;
									t1_8 = t1;
									break;
								default:
									cout << "ERROR (CompressedOctree ::intern_search_2<) : La escombra s'ha equivocat!" << endl;
									exit(-1);
									break;
							}
							
							//Cridem search_2
							if ( (ok1) && (ok5) ) intern_search_2z(info,t1_5,t2->getFill1(),bNum,bHisto,bDist);
    						if ( (ok2) && (ok6) ) intern_search_2z(info,t1_6,t2->getFill2(),bNum,bHisto,bDist);
    						if ( (ok3) && (ok7) ) intern_search_2z(info,t1_7,t2->getFill3(),bNum,bHisto,bDist);
    						if ( (ok4) && (ok8) ) intern_search_2z(info,t1_8,t2->getFill4(),bNum,bHisto,bDist);
    						
    						//Cridem search_4
    						call_intern_search_4(info,ok1,ok2,ok5,ok6,t1_5,t1_6,t2->getFill1(),t2->getFill2(),bNum,bHisto,bDist,2);
    						call_intern_search_4(info,ok3,ok4,ok7,ok8,t1_7,t1_8,t2->getFill3(),t2->getFill4(),bNum,bHisto,bDist,2);
    						call_intern_search_4(info,ok1,ok3,ok5,ok7,t1_5,t1_7,t2->getFill1(),t2->getFill3(),bNum,bHisto,bDist,3);
    						call_intern_search_4(info,ok2,ok4,ok6,ok8,t1_6,t1_8,t2->getFill2(),t2->getFill4(),bNum,bHisto,bDist,3);
    			
    						//Cridem search_8
    						call_intern_search_8(info,ok1,ok2,ok3,ok4,ok5,ok6,ok7,ok8,t1_5,t1_6,t1_7,t1_8,t2->getFill1(),t2->getFill2(),t2->getFill3(),t2->getFill4(),bNum,bHisto,bDist);
    			
						} else if (parcial1) {
							//Nomes t1 es parcial

							//Mirem si creant un node orfe al costat del node parcial
							//aconseguim "atrapar" algun element
							double dif = (t1->getAncoratge().getZ()-t2->getAncoratge().getZ());
							double difMax = t1->getMida()+1.0e-6;
		
							if ( (dif<=difMax) && (quiEscombra2(t1,t2,3)!=-1) ) {
								//CALDRIA FER NODE ORFEEEEEEEEEEEEEEEEEEEEEEEEE!!!!!!!!!!!!!!!!!!!!!!!!!!
								// Let's store the candidate zone
								CompressedCandidateZone *cand = new CompressedCandidateZone(2,t1,t2);
								candidates.push_back(cand);	
								t1->setNodeZones(cand);
								t2->setNodeZones(cand);
							
								//cout<<"REPORTO!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
								//cout<<"Compatible side Pair ! "<<endl;
								//cout<<"Base Point1: "<<t1->getAncoratge()<<" Size1: "<<t1->getMida()<<endl;
								//cout<<"Base Point2: "<<t2->getAncoratge()<<" Size2: "<<t2->getMida()<<endl;
							} else {
								// Let's store the candidate zone
								CompressedCandidateZone *cand = new CompressedCandidateZone(2,t1,t2);
								candidates.push_back(cand);	
								t1->setNodeZones(cand);
								t2->setNodeZones(cand);
							
								//cout<<"REPORTO!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
								//cout<<"Compatible side Pair ! "<<endl;
								//cout<<"Base Point1: "<<t1->getAncoratge()<<" Size1: "<<t1->getMida()<<endl;
								//cout<<"Base Point2: "<<t2->getAncoratge()<<" Size2: "<<t2->getMida()<<endl;
							}

						} else {
							//Cap parcial (2 fulles negres)
	
							// Let's store the candidate zone
							CompressedCandidateZone *cand = new CompressedCandidateZone(2,t1,t2);
							candidates.push_back(cand);	
							t1->setNodeZones(cand);
								t2->setNodeZones(cand);
						
							//cout<<"REPORTO!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
							//cout<<"Compatible side Pair ! "<<endl;
							//cout<<"Base Point1: "<<t1->getAncoratge()<<" Size1: "<<t1->getMida()<<endl;
							//cout<<"Base Point2: "<<t2->getAncoratge()<<" Size2: "<<t2->getMida()<<endl;
						}
					} else {
						//No estan de costat
					
						//CALDRIA MIRAR ELS 12 CASOS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	
						// Let's store the candidate zone
						CompressedCandidateZone *cand = new CompressedCandidateZone(2,t1,t2);
						candidates.push_back(cand);	
						t1->setNodeZones(cand);
						t2->setNodeZones(cand);
					
						//cout<<"REPORTO!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
						//cout<<"Compatible side Pair ! "<<endl;
						//cout<<"Base Point1: "<<t1->getAncoratge()<<" Size1: "<<t1->getMida()<<endl;
						//cout<<"Base Point2: "<<t2->getAncoratge()<<" Size2: "<<t2->getMida()<<endl;
					}
				}
			}
		}
	}

//cout<<"surto de search 2z"<<endl;
}

// Search in Quartets
/*
1 2   5 6
3 4 ó 7 8
*/
// Pre: de moment, tot el que es crida es fa sobre nodes de la mida que toca o nulls
void CompressedOctree::intern_search_4xy(CompressedInformacioGeometrica info, CompressedONode * t1, CompressedONode * t2,CompressedONode * t3, CompressedONode * t4, bool bNum, bool bHisto, bool bDist)
{

/*	cout<<endl<<endl<<endl;
	cout<<"entro a search_4xy amb nodes i mides: "<<endl;
	cout<<" 1: ";
	if(t1!=NULL)cout<<t1->getAncoratge()<<" "<<t1->getMida()<<endl;
	else cout<<"NULL!"<<endl;

	cout<<" 2: ";
	if(t2!=NULL)cout<<t2->getAncoratge()<<" "<<t2->getMida()<<endl;
	else cout<<"NULL!"<<endl;

	cout<<" 3: ";
	if(t3!=NULL)cout<<t3->getAncoratge()<<" "<<t3->getMida()<<endl;
	else cout<<"NULL!"<<endl;

	cout<<" 4: ";
	if(t4!=NULL)cout<<t4->getAncoratge()<<" "<<t4->getMida()<<endl;
	else cout<<"NULL!"<<endl;

char c;
cin>>c;*/

	CompressedInformacioGeometrica* infoT1=NULL;
	CompressedInformacioGeometrica* infoT2=NULL;
	CompressedInformacioGeometrica* infoT3=NULL;
	CompressedInformacioGeometrica* infoT4=NULL;

	// primer els posem tots a null, per assegurar
	//infoT1=NULL;
	//infoT2=NULL;
	//infoT3=NULL;
	//infoT4=NULL;

	// recollim la informacio geometrica de tots els nodes que no siguin nuls o fulla blanca	
	if( (t1!=NULL)&&(t1->tipus()!=0)) infoT1=t1->getInfGeo();
	if( (t2!=NULL)&&(t2->tipus()!=0)) infoT2=t2->getInfGeo();
	if( (t3!=NULL)&&(t3->tipus()!=0)) infoT3=t3->getInfGeo();
	if( (t4!=NULL)&&(t4->tipus()!=0)) infoT4=t4->getInfGeo();

	// Test for compatibility
	// atencio, compatible ha passat a ser estatica!
	if ( (CompressedONode::compatible(info,calcularInfo4(infoT1,infoT2,infoT3,infoT4), bNum, bHisto, bDist,epsilon)) ) 
	{
		
		//Mirem la mida dels que siguin parcials i comprovem si queda algun parcial
		bool quedenParcials=false;
		double mida = 0.0;		

		if(t1!=NULL) 
		{ 
			if(t1->tipus()==2)
			{
				 quedenParcials=true;
				if (mida<t1->getMida()) mida=t1->getMida(); 
			}
		}

		if(t2!=NULL) 
		{ 
			if(t2->tipus()==2)
			{
				 quedenParcials=true;
				if (mida<t2->getMida()) mida=t2->getMida(); 
			}
		}

		if(t3!=NULL) 
		{ 
			if(t3->tipus()==2)
			{
				 quedenParcials=true;
				if (mida<t3->getMida()) mida=t3->getMida(); 
			}
		}

		if(t4!=NULL) 
		{ 
			if(t4->tipus()==2)
			{
				 quedenParcials=true;
				if (mida<t4->getMida()) mida=t4->getMida(); 
			}
		}

		// si no queden parcials o la mida ja es massa petita, reportem
		if ( (mida/2.0)<info.getMidaNode()||!quedenParcials ) 
		{

			// Let's store the candidate zone
			CompressedCandidateZone *cand = new CompressedCandidateZone(4,t1,t2,t3,t4);
			candidates.push_back(cand);	
			if(t1!=NULL) t1->setNodeZones(cand);
			if(t2!=NULL) t2->setNodeZones(cand);
			if(t3!=NULL) t3->setNodeZones(cand);
			if(t4!=NULL) t4->setNodeZones(cand);

			/*cout<<"                                                                         ";
			cout<<"REPORTO!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
			cout<<"Compatible quartet xy"<<endl;

			cout<<" 1: ";
			if(t1!=NULL)cout<<t1->getAncoratge()<<" "<<t1->getMida()<<endl;
			else cout<<"NULL!"<<endl;

			cout<<" 2: ";
			if(t2!=NULL)cout<<t2->getAncoratge()<<" "<<t2->getMida()<<endl;
			else cout<<"NULL!"<<endl;

			cout<<" 3: ";
			if(t3!=NULL)cout<<t3->getAncoratge()<<" "<<t3->getMida()<<endl;
			else cout<<"NULL!"<<endl;

			cout<<" 4: ";
			if(t4!=NULL)cout<<t4->getAncoratge()<<" "<<t4->getMida()<<endl;
			else cout<<"NULL!"<<endl;*/
		
			//aqui tb falta pintar	
		}
		else// hem de seguir tirant
		{
			// queden parcial i la mida encara es correcta
			// ara tots son de mida adequada

			// declararem quatre nodes nous i adaptarem les crides per:
			// repetir les fulles negres i blanques i "baixar" per els que no ho siguin
			// vigilar de baixar nomes un "grao", descomprimint si cal
			// els "aux" serveixen per descomprimir i els nous per fer la nova crida 
			// sense que emprenyin les fulles negres.
			CompressedONode nodeAux1,nodeAux2,nodeAux3,nodeAux4,nodeAux5,nodeAux6,nodeAux7,nodeAux8;
			vector<bool> fillsTocats1, fillsTocats2, fillsTocats3, fillsTocats4, fillsTocats5, fillsTocats6,fillsTocats7, fillsTocats8;	

			CompressedONode *nouT1_4,*nouT1_8,*nouT2_3,*nouT2_7,*nouT3_2,*nouT3_6,*nouT4_1,*nouT4_5;

			// abans de llençar les cerques, descomprimim
			// si es parcial, el descomprimim si cal			
			if( (t1!=NULL)&&(t1->tipus()==2))
			{							
				nodeAux1 = CompressedONode(t1->getAncoratge(),t1->getMida(),t1->getNivell() );
				fillsTocats1 = descomprimeix(t1,&nodeAux1);
	
				// baixem pel fill que toca
				nouT1_4=nodeAux1.getFill4();
				nouT1_8=nodeAux1.getFill8();
			}
			else 
			{
				// tant si es fulla negra, com blanca, com null hem de cridar search4 amb ell
				nouT1_4=t1;
				nouT1_8=t1;
				// ep! aixo portara problemes amb la crida a search_8, 
				// abans de ferla caldra treure aquesta repetició posant-ne un a null 
			} 
	
			if( (t2!=NULL)&&(t2->tipus()==2))
			{							
				nodeAux2 = CompressedONode(t2->getAncoratge(),t2->getMida(),t2->getNivell() );
				fillsTocats2 = descomprimeix(t2,&nodeAux2);
	
				// baixem pel fill que toca
				nouT2_3=nodeAux2.getFill3();
				nouT2_7=nodeAux2.getFill7();
			}
			else 
			{
				// tant si es fulla negra, com blanca, com null hem de cridar search4 amb ell
				nouT2_3=t2;
				nouT2_7=t2;
				// ep! aixo portara problemes amb la crida a search_8, 
				// abans de ferla caldra treure aquesta repetició posant-ne un a null 
			} 

			if( (t3!=NULL)&&(t3->tipus()==2))
			{							
				nodeAux3 = CompressedONode(t3->getAncoratge(),t3->getMida(),t3->getNivell() );
				fillsTocats3 = descomprimeix(t3,&nodeAux3);
	
				// baixem pel fill que toca
				nouT3_2=nodeAux3.getFill2();
				nouT3_6=nodeAux3.getFill6();
			}
			else 
			{
				// tant si es fulla negra, com blanca, com null hem de cridar search4 amb ell
				nouT3_2=t3;
				nouT3_6=t3;
				// ep! aixo portara problemes amb la crida a search_8, 
				// abans de ferla caldra treure aquesta repetició posant-ne un a null 
			} 

			if( (t4!=NULL)&&(t4->tipus()==2))
			{							
				nodeAux4 = CompressedONode(t4->getAncoratge(),t4->getMida(),t4->getNivell() );
				fillsTocats4 = descomprimeix(t4,&nodeAux4);
	
				// baixem pel fill que toca
				nouT4_1=nodeAux4.getFill1();
				nouT4_5=nodeAux4.getFill5();
			}
			else 
			{
				// tant si es fulla negra, com blanca, com null hem de cridar search4 amb ell
				nouT4_1=t4;
				nouT4_5=t4;
				// ep! aixo portara problemes amb la crida a search_8, 
				// abans de ferla caldra treure aquesta repetició posant-ne un a null 
			} 

			// ara totes les cerques es llençaran sobre els fills dels nodeAuxi si cal 
			// Nomes falta vigilar que no tinguem massa fulles blanques.
			bool ok1,ok2,ok3,ok4,ok5,ok6,ok7,ok8;
	
			ok1 = (nouT1_4!=NULL) && (nouT1_4->tipus()!=0);
			ok2 = (nouT2_3!=NULL) && (nouT2_3->tipus()!=0);
			ok3 = (nouT3_2!=NULL) && (nouT3_2->tipus()!=0);
			ok4 = (nouT4_1!=NULL) && (nouT4_1->tipus()!=0);
			ok5 = (nouT1_8!=NULL) && (nouT1_8->tipus()!=0);
			ok6 = (nouT2_7!=NULL) && (nouT2_7->tipus()!=0);
			ok7 = (nouT3_6!=NULL) && (nouT3_6->tipus()!=0);
			ok8 = (nouT4_5!=NULL) && (nouT4_5->tipus()!=0);
			
			// aqui falten les crides dels quartets.

			call_intern_search_4(info,ok1,ok2,ok3,ok4,nouT1_4,nouT2_3,nouT3_2,nouT4_1,bNum,bHisto,bDist,1);
			call_intern_search_4(info,ok5,ok6,ok7,ok8,nouT1_8,nouT2_7,nouT3_6,nouT4_5,bNum,bHisto,bDist,1);

	
			//4. Search octet
			
			// si resulta que tenim "repetits" (en el cas de les fulles negres, en fem un de null)
			// vigilem pero els oks! de moment no els toquem
			if((t1!=NULL)&&(t1->tipus()==1)) nouT1_8 = NULL;
			if((t2!=NULL)&&(t2->tipus()==1)) nouT2_7 = NULL;
			if((t3!=NULL)&&(t3->tipus()==1)) nouT3_2 = NULL;
			if((t4!=NULL)&&(t4->tipus()==1)) nouT4_1 = NULL;

			call_intern_search_8(info,ok1,ok2,ok3,ok4,ok5,ok6,ok7,ok8,nouT1_4,nouT2_3,nouT3_2,nouT4_1,nouT1_8,nouT2_7,nouT3_6,nouT4_5,bNum,bHisto,bDist);

			// recomprimeix per no perdre memoria tots els que hagin estat descomprimits
			if(fillsTocats1.size()!=0) recomprimeix(fillsTocats1, &nodeAux1);
			if(fillsTocats2.size()!=0) recomprimeix(fillsTocats2, &nodeAux2);
			if(fillsTocats3.size()!=0) recomprimeix(fillsTocats3, &nodeAux3);
			if(fillsTocats4.size()!=0) recomprimeix(fillsTocats4, &nodeAux4);

			// ara els nAux moren i les estructures "auxiliars" queden alliberades sense perdre memoria.
		
		}// tanca l'else de seguir tirant
	}// tanca l'if de compatible

//cout<<"surto de search 4xy"<<endl;




}

// Search in Quartets
/*
1 5   3 7
2 6 ó 4 8
*/
void CompressedOctree::intern_search_4xz(CompressedInformacioGeometrica info, CompressedONode * t1, CompressedONode * t2,CompressedONode * t3, CompressedONode * t4, bool bNum, bool bHisto, bool bDist)
{
	/*cout<<endl<<endl<<endl;
	cout<<"entro a search_4xz amb nodes i mides: "<<endl;
	cout<<" 1: ";
	if(t1!=NULL)cout<<t1->getAncoratge()<<" "<<t1->getMida()<<endl;
	else cout<<"NULL!"<<endl;

	cout<<" 2: ";
	if(t2!=NULL)cout<<t2->getAncoratge()<<" "<<t2->getMida()<<endl;
	else cout<<"NULL!"<<endl;

	cout<<" 3: ";
	if(t3!=NULL)cout<<t3->getAncoratge()<<" "<<t3->getMida()<<endl;
	else cout<<"NULL!"<<endl;

	cout<<" 4: ";
	if(t4!=NULL)cout<<t4->getAncoratge()<<" "<<t4->getMida()<<endl;
	else cout<<"NULL!"<<endl;

char c;
cin>>c;*/

	CompressedInformacioGeometrica* infoT1;
	CompressedInformacioGeometrica* infoT2;
	CompressedInformacioGeometrica* infoT3;
	CompressedInformacioGeometrica* infoT4;

	// primer els posem tots a null, per assegurar
	infoT1=NULL;
	infoT2=NULL;
	infoT3=NULL;
	infoT4=NULL;

	// recollim la informacio geometrica de tots els nodes que no siguin nuls o fulla blanca	
	if( (t1!=NULL)&&(t1->tipus()!=0)) infoT1=t1->getInfGeo();
	if( (t2!=NULL)&&(t2->tipus()!=0)) infoT2=t2->getInfGeo();
	if( (t3!=NULL)&&(t3->tipus()!=0)) infoT3=t3->getInfGeo();
	if( (t4!=NULL)&&(t4->tipus()!=0)) infoT4=t4->getInfGeo();

	// Test for compatibility
	// atencio, compatible ha passat a ser estatica!
	CompressedInformacioGeometrica *info2 = calcularInfo4(infoT1,infoT2,infoT3,infoT4);
	

	if ( (CompressedONode::compatible(info, info2, bNum, bHisto, bDist,epsilon)) ) 
	{
		
		//Mirem la mida dels que siguin parcials i comprovem si queda algun parcial
		bool quedenParcials=false;
		double mida = 0.0;		

		if(t1!=NULL) 
		{ 
			if(t1->tipus()==2)
			{
				 quedenParcials=true;
				if (mida<t1->getMida()) mida=t1->getMida(); 
			}
		}

		if(t2!=NULL) 
		{ 
			if(t2->tipus()==2)
			{
				 quedenParcials=true;
				if (mida<t2->getMida()) mida=t2->getMida(); 
			}
		}

		if(t3!=NULL) 
		{ 
			if(t3->tipus()==2)
			{
				 quedenParcials=true;
				if (mida<t3->getMida()) mida=t3->getMida(); 
			}
		}

		if(t4!=NULL) 
		{ 
			if(t4->tipus()==2)
			{
				 quedenParcials=true;
				if (mida<t4->getMida()) mida=t4->getMida(); 
			}
		}

		// si no queden parcials o la mida ja es massa petita, reportem
		if ( (mida/2.0)<info.getMidaNode()||!quedenParcials ) 
		{

			// Let's store the candidate zone
			CompressedCandidateZone *cand = new CompressedCandidateZone(4,t1,t2,t3,t4);
			candidates.push_back(cand);	
			if(t1!=NULL) t1->setNodeZones(cand);
			if(t2!=NULL) t2->setNodeZones(cand);
			if(t3!=NULL) t3->setNodeZones(cand);
			if(t4!=NULL) t4->setNodeZones(cand);
		
			/*cout<<"                                                                         ";
			cout<<"REPORTO!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
			cout<<"Compatible quartet xz"<<endl;

			cout<<" 1: ";
			if(t1!=NULL)cout<<t1->getAncoratge()<<" "<<t1->getMida()<<endl;
			else cout<<"NULL!"<<endl;

			cout<<" 2: ";
			if(t2!=NULL)cout<<t2->getAncoratge()<<" "<<t2->getMida()<<endl;
			else cout<<"NULL!"<<endl;

			cout<<" 3: ";
			if(t3!=NULL)cout<<t3->getAncoratge()<<" "<<t3->getMida()<<endl;
			else cout<<"NULL!"<<endl;

			cout<<" 4: ";
			if(t4!=NULL)cout<<t4->getAncoratge()<<" "<<t4->getMida()<<endl;
			else cout<<"NULL!"<<endl;*/
		
			//aqui tb falta pintar	
		}
		else// hem de seguir tirant
		{
			// queden parcial i la mida encara es correcta
			// ara tots son de mida adequada

			// declararem quatre nodes nous i adaptarem les crides per:
			// repetir les fulles negres i blanques i "baixar" per els que no ho siguin
			// vigilar de baixar nomes un "grao", descomprimint si cal
			// els "aux" serveixen per descomprimir i els nous per fer la nova crida 
			// sense que emprenyin les fulles negres.
			CompressedONode nodeAux1,nodeAux2,nodeAux3,nodeAux4,nodeAux5,nodeAux6,nodeAux7,nodeAux8;
			vector<bool> fillsTocats1, fillsTocats2, fillsTocats3, fillsTocats4, fillsTocats5, fillsTocats6,fillsTocats7, fillsTocats8;	

			CompressedONode *nouT1_6,*nouT1_8,*nouT2_5,*nouT2_7,*nouT3_2,*nouT3_4,*nouT4_1,*nouT4_3;

			// abans de llençar les cerques, descomprimim
			// si es parcial, el descomprimim si cal			
			if( (t1!=NULL)&&(t1->tipus()==2))
			{							
				nodeAux1 = CompressedONode(t1->getAncoratge(),t1->getMida(),t1->getNivell() );
				fillsTocats1 = descomprimeix(t1,&nodeAux1);
	
				// baixem pel fill que toca
				nouT1_6=nodeAux1.getFill6();
				nouT1_8=nodeAux1.getFill8();
			}
			else 
			{
				// tant si es fulla negra, com blanca, com null hem de cridar search4 amb ell
				nouT1_6=t1;
				nouT1_8=t1;
				// ep! aixo portara problemes amb la crida a search_8, 
				// abans de ferla caldra treure aquesta repetició posant-ne un a null 
			} 
	
			if( (t2!=NULL)&&(t2->tipus()==2))
			{							
				nodeAux2 = CompressedONode(t2->getAncoratge(),t2->getMida(),t2->getNivell() );
				fillsTocats2 = descomprimeix(t2,&nodeAux2);
	
				// baixem pel fill que toca
				nouT2_5=nodeAux2.getFill5();
				nouT2_7=nodeAux2.getFill7();
			}
			else 
			{
				// tant si es fulla negra, com blanca, com null hem de cridar search4 amb ell
				nouT2_5=t2;
				nouT2_7=t2;
				// ep! aixo portara problemes amb la crida a search_8, 
				// abans de ferla caldra treure aquesta repetició posant-ne un a null 
			} 

			if( (t3!=NULL)&&(t3->tipus()==2))
			{							
				nodeAux3 = CompressedONode(t3->getAncoratge(),t3->getMida(),t3->getNivell() );
				fillsTocats3 = descomprimeix(t3,&nodeAux3);
	
				// baixem pel fill que toca
				nouT3_2=nodeAux3.getFill2();
				nouT3_4=nodeAux3.getFill4();
			}
			else 
			{
				// tant si es fulla negra, com blanca, com null hem de cridar search4 amb ell
				nouT3_2=t3;
				nouT3_4=t3;
				// ep! aixo portara problemes amb la crida a search_8, 
				// abans de ferla caldra treure aquesta repetició posant-ne un a null 
			} 

			if( (t4!=NULL)&&(t4->tipus()==2))
			{							
				nodeAux4 = CompressedONode(t4->getAncoratge(),t4->getMida(),t4->getNivell() );
				fillsTocats4 = descomprimeix(t4,&nodeAux4);
	
				// baixem pel fill que toca
				nouT4_1=nodeAux4.getFill1();
				nouT4_3=nodeAux4.getFill3();
			}
			else 
			{
				// tant si es fulla negra, com blanca, com null hem de cridar search4 amb ell
				nouT4_1=t4;
				nouT4_3=t4;
				// ep! aixo portara problemes amb la crida a search_8, 
				// abans de ferla caldra treure aquesta repetició posant-ne un a null 
			} 

			// ara totes les cerques es llençaran sobre els fills dels nodeAuxi si cal 
			// Nomes falta vigilar que no tinguem massa fulles blanques.
			bool ok1,ok2,ok3,ok4,ok5,ok6,ok7,ok8;
	
			ok1 = (nouT1_6!=NULL) && (nouT1_6->tipus()!=0);
			ok2 = (nouT2_5!=NULL) && (nouT2_5->tipus()!=0);
			ok3 = (nouT3_2!=NULL) && (nouT3_2->tipus()!=0);
			ok4 = (nouT4_1!=NULL) && (nouT4_1->tipus()!=0);
			ok5 = (nouT1_8!=NULL) && (nouT1_8->tipus()!=0);
			ok6 = (nouT2_7!=NULL) && (nouT2_7->tipus()!=0);
			ok7 = (nouT3_4!=NULL) && (nouT3_4->tipus()!=0);
			ok8 = (nouT4_3!=NULL) && (nouT4_3->tipus()!=0);
			
			// aqui falten les crides dels quartets.

			call_intern_search_4(info,ok1,ok2,ok3,ok4,nouT1_6,nouT2_5,nouT3_2,nouT4_1,bNum,bHisto,bDist,2);
			call_intern_search_4(info,ok5,ok6,ok7,ok8,nouT1_8,nouT2_7,nouT3_4,nouT4_3,bNum,bHisto,bDist,2);

	
			//4. Search octet
			
			// si resulta que tenim "repetits" (en el cas de les fulles negres, en fem un de null
			if((t1!=NULL)&&(t1->tipus()==1)) nouT1_8 = NULL;
			if((t2!=NULL)&&(t2->tipus()==1)) nouT2_7 = NULL;
			if((t3!=NULL)&&(t3->tipus()==1)) nouT3_4 = NULL;
			if((t4!=NULL)&&(t4->tipus()==1)) nouT4_3 = NULL;

			call_intern_search_8(info,ok1,ok2,ok5,ok6,ok3,ok4,ok7,ok8,nouT1_6,nouT2_5,nouT1_8,nouT2_7,nouT3_2,nouT4_1,nouT3_4,nouT4_3,bNum,bHisto,bDist);

			// recomprimeix per no perdre memoria tots els que hagin estat descomprimits
			if(fillsTocats1.size()!=0) recomprimeix(fillsTocats1, &nodeAux1);
			if(fillsTocats2.size()!=0) recomprimeix(fillsTocats2, &nodeAux2);
			if(fillsTocats3.size()!=0) recomprimeix(fillsTocats3, &nodeAux3);
			if(fillsTocats4.size()!=0) recomprimeix(fillsTocats4, &nodeAux4);

			// ara els nAux moren i les estructures "auxiliars" queden alliberades sense perdre memoria.
		
		}// tanca l'else de seguir tirant
	}// tanca l'if de compatible

delete info2;

//cout<<"surto de search 4xz"<<endl;

}

// Search in Quartets
/*
1 5   2 6
3 7 ó 4 8
*/
void CompressedOctree::intern_search_4yz(CompressedInformacioGeometrica info, CompressedONode * t1, CompressedONode * t2,CompressedONode * t3, CompressedONode * t4, bool bNum, bool bHisto, bool bDist)
{

	/*cout<<endl<<endl<<endl;
	cout<<"entro a search_4yz amb nodes i mides: "<<endl;
	cout<<" 1: ";
	if(t1!=NULL)cout<<t1->getAncoratge()<<" "<<t1->getMida()<<endl;
	else cout<<"NULL!"<<endl;

	cout<<" 2: ";
	if(t2!=NULL)cout<<t2->getAncoratge()<<" "<<t2->getMida()<<endl;
	else cout<<"NULL!"<<endl;

	cout<<" 3: ";
	if(t3!=NULL)cout<<t3->getAncoratge()<<" "<<t3->getMida()<<endl;
	else cout<<"NULL!"<<endl;

	cout<<" 4: ";
	if(t4!=NULL)cout<<t4->getAncoratge()<<" "<<t4->getMida()<<endl;
	else cout<<"NULL!"<<endl;

char c;
cin>>c;*/

	CompressedInformacioGeometrica* infoT1;
	CompressedInformacioGeometrica* infoT2;
	CompressedInformacioGeometrica* infoT3;
	CompressedInformacioGeometrica* infoT4;

	// primer els posem tots a null, per assegurar
	infoT1=NULL;
	infoT2=NULL;
	infoT3=NULL;
	infoT4=NULL;

	// recollim la informacio geometrica de tots els nodes que no siguin nuls o fulla blanca	
	if( (t1!=NULL)&&(t1->tipus()!=0)) infoT1=t1->getInfGeo();
	if( (t2!=NULL)&&(t2->tipus()!=0)) infoT2=t2->getInfGeo();
	if( (t3!=NULL)&&(t3->tipus()!=0)) infoT3=t3->getInfGeo();
	if( (t4!=NULL)&&(t4->tipus()!=0)) infoT4=t4->getInfGeo();

	// Test for compatibility
	// atencio, compatible ha passat a ser estatica!
	if ( (CompressedONode::compatible(info,calcularInfo4(infoT1,infoT2,infoT3,infoT4), bNum, bHisto, bDist,epsilon)) ) 
	{
		
		//Mirem la mida dels que siguin parcials i comprovem si queda algun parcial
		bool quedenParcials=false;
		double mida = 0.0;		

		if(t1!=NULL) 
		{ 
			if(t1->tipus()==2)
			{
				 quedenParcials=true;
				if (mida<t1->getMida()) mida=t1->getMida(); 
			}
		}

		if(t2!=NULL) 
		{ 
			if(t2->tipus()==2)
			{
				 quedenParcials=true;
				if (mida<t2->getMida()) mida=t2->getMida(); 
			}
		}

		if(t3!=NULL) 
		{ 
			if(t3->tipus()==2)
			{
				 quedenParcials=true;
				if (mida<t3->getMida()) mida=t3->getMida(); 
			}
		}

		if(t4!=NULL) 
		{ 
			if(t4->tipus()==2)
			{
				 quedenParcials=true;
				if (mida<t4->getMida()) mida=t4->getMida(); 
			}
		}

		// si no queden parcials o la mida ja es massa petita, reportem
		if ( (mida/2.0)<info.getMidaNode()||!quedenParcials ) 
		{

			// Let's store the candidate zone
			CompressedCandidateZone *cand = new CompressedCandidateZone(4,t1,t2,t3,t4);
			candidates.push_back(cand);	
			if(t1!=NULL) t1->setNodeZones(cand);
			if(t2!=NULL) t2->setNodeZones(cand);
			if(t3!=NULL) t3->setNodeZones(cand);
			if(t4!=NULL) t4->setNodeZones(cand);
		
			/*cout<<"                                                                         ";
			cout<<"REPORTO!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
			cout<<"Compatible quartet yz"<<endl;

			cout<<" 1: ";
			if(t1!=NULL)cout<<t1->getAncoratge()<<" "<<t1->getMida()<<endl;
			else cout<<"NULL!"<<endl;

			cout<<" 2: ";
			if(t2!=NULL)cout<<t2->getAncoratge()<<" "<<t2->getMida()<<endl;
			else cout<<"NULL!"<<endl;

			cout<<" 3: ";
			if(t3!=NULL)cout<<t3->getAncoratge()<<" "<<t3->getMida()<<endl;
			else cout<<"NULL!"<<endl;

			cout<<" 4: ";
			if(t4!=NULL)cout<<t4->getAncoratge()<<" "<<t4->getMida()<<endl;
			else cout<<"NULL!"<<endl;*/
		
			//aqui tb falta pintar	
		}
		else// hem de seguir tirant
		{
			// queden parcial i la mida encara es correcta
			// ara tots son de mida adequada

			// declararem quatre nodes nous i adaptarem les crides per:
			// repetir les fulles negres i blanques i "baixar" per els que no ho siguin
			// vigilar de baixar nomes un "grao", descomprimint si cal
			// els "aux" serveixen per descomprimir i els nous per fer la nova crida 
			// sense que emprenyin les fulles negres.
			CompressedONode nodeAux1,nodeAux2,nodeAux3,nodeAux4,nodeAux5,nodeAux6,nodeAux7,nodeAux8;
			vector<bool> fillsTocats1, fillsTocats2, fillsTocats3, fillsTocats4, fillsTocats5, fillsTocats6,fillsTocats7, fillsTocats8;	

			CompressedONode *nouT1_7,*nouT1_8,*nouT2_5,*nouT2_6,*nouT3_3,*nouT3_4,*nouT4_1,*nouT4_4;

			// abans de llençar les cerques, descomprimim
			// si es parcial, el descomprimim si cal			
			if( (t1!=NULL)&&(t1->tipus()==2))
			{							
				nodeAux1 = CompressedONode(t1->getAncoratge(),t1->getMida(),t1->getNivell() );
				fillsTocats1 = descomprimeix(t1,&nodeAux1);
	
				// baixem pel fill que toca
				nouT1_7=nodeAux1.getFill7();
				nouT1_8=nodeAux1.getFill8();
			}
			else 
			{
				// tant si es fulla negra, com blanca, com null hem de cridar search4 amb ell
				nouT1_7=t1;
				nouT1_8=t1;
				// ep! aixo portara problemes amb la crida a search_8, 
				// abans de ferla caldra treure aquesta repetició posant-ne un a null 
			} 
	
			if( (t2!=NULL)&&(t2->tipus()==2))
			{							
				nodeAux2 = CompressedONode(t2->getAncoratge(),t2->getMida(),t2->getNivell() );
				fillsTocats2 = descomprimeix(t2,&nodeAux2);
	
				// baixem pel fill que toca
				nouT2_5=nodeAux2.getFill5();
				nouT2_6=nodeAux2.getFill6();
			}
			else 
			{
				// tant si es fulla negra, com blanca, com null hem de cridar search4 amb ell
				nouT2_5=t2;
				nouT2_6=t2;
				// ep! aixo portara problemes amb la crida a search_8, 
				// abans de ferla caldra treure aquesta repetició posant-ne un a null 
			} 

			if( (t3!=NULL)&&(t3->tipus()==2))
			{							
				nodeAux3 = CompressedONode(t3->getAncoratge(),t3->getMida(),t3->getNivell() );
				fillsTocats3 = descomprimeix(t3,&nodeAux3);
	
				// baixem pel fill que toca
				nouT3_3=nodeAux3.getFill3();
				nouT3_4=nodeAux3.getFill4();
			}
			else 
			{
				// tant si es fulla negra, com blanca, com null hem de cridar search4 amb ell
				nouT3_3=t3;
				nouT3_4=t3;
				// ep! aixo portara problemes amb la crida a search_8, 
				// abans de ferla caldra treure aquesta repetició posant-ne un a null 
			} 

			if( (t4!=NULL)&&(t4->tipus()==2))
			{							
				nodeAux4 = CompressedONode(t4->getAncoratge(),t4->getMida(),t4->getNivell() );
				fillsTocats4 = descomprimeix(t4,&nodeAux4);
	
				// baixem pel fill que toca
				nouT4_1=nodeAux4.getFill1();
				nouT4_4=nodeAux4.getFill4();
			}
			else 
			{
				// tant si es fulla negra, com blanca, com null hem de cridar search4 amb ell
				nouT4_1=t4;
				nouT4_4=t4;
				// ep! aixo portara problemes amb la crida a search_8, 
				// abans de ferla caldra treure aquesta repetició posant-ne un a null 
			} 

			// ara totes les cerques es llençaran sobre els fills dels nodeAuxi si cal 
			// Nomes falta vigilar que no tinguem massa fulles blanques.
			bool ok1,ok2,ok3,ok4,ok5,ok6,ok7,ok8;
	
			ok1 = (nouT1_7!=NULL) && (nouT1_7->tipus()!=0);
			ok2 = (nouT2_5!=NULL) && (nouT2_5->tipus()!=0);
			ok3 = (nouT3_3!=NULL) && (nouT3_3->tipus()!=0);
			ok4 = (nouT4_1!=NULL) && (nouT4_1->tipus()!=0);
			ok5 = (nouT1_8!=NULL) && (nouT1_8->tipus()!=0);
			ok6 = (nouT2_6!=NULL) && (nouT2_6->tipus()!=0);
			ok7 = (nouT3_4!=NULL) && (nouT3_4->tipus()!=0);
			ok8 = (nouT4_4!=NULL) && (nouT4_4->tipus()!=0);
			
			// aqui falten les crides dels quartets.

			call_intern_search_4(info,ok1,ok2,ok3,ok4,nouT1_7,nouT2_5,nouT3_3,nouT4_1,bNum,bHisto,bDist,3);
			call_intern_search_4(info,ok5,ok6,ok7,ok8,nouT1_8,nouT2_6,nouT3_4,nouT4_4,bNum,bHisto,bDist,3);

			//4. Search octet
			
			// si resulta que tenim "repetits" (en el cas de les fulles negres, en fem un de null
			if((t1!=NULL)&&(t1->tipus()==1)) nouT1_8 = NULL;
			if((t2!=NULL)&&(t2->tipus()==1)) nouT2_6 = NULL;
			if((t3!=NULL)&&(t3->tipus()==1)) nouT3_4 = NULL;
			if((t4!=NULL)&&(t4->tipus()==1)) nouT4_1 = NULL;

			call_intern_search_8(info,ok1,ok5,ok2,ok6,ok3,ok7,ok4,ok8,nouT1_7,nouT1_8,nouT2_5,nouT2_6,nouT3_3,nouT3_4,nouT4_1,nouT4_4,bNum,bHisto,bDist);

			// recomprimeix per no perdre memoria tots els que hagin estat descomprimits
			if(fillsTocats1.size()!=0) recomprimeix(fillsTocats1, &nodeAux1);
			if(fillsTocats2.size()!=0) recomprimeix(fillsTocats2, &nodeAux2);
			if(fillsTocats3.size()!=0) recomprimeix(fillsTocats3, &nodeAux3);
			if(fillsTocats4.size()!=0) recomprimeix(fillsTocats4, &nodeAux4);

			// ara els nAux moren i les estructures "auxiliars" queden alliberades sense perdre memoria.
		
		}// tanca l'else de seguir tirant
	}// tanca l'if de compatible

//cout<<"surto de search 4xz"<<endl;



}

// Search in Octets
void CompressedOctree::intern_search_8(CompressedInformacioGeometrica info, CompressedONode * t1, CompressedONode * t2,CompressedONode * t3, CompressedONode * t4, CompressedONode * t5, CompressedONode * t6,CompressedONode * t7, CompressedONode * t8, bool bNum, bool bHisto, bool bDist)
{

	/*cout<<endl<<endl<<endl;
	cout<<"entro a search_8 amb nodes i mides: "<<endl;
	cout<<" 1: ";
	if(t1!=NULL)cout<<t1->getAncoratge()<<" "<<t1->getMida()<<endl;
	else cout<<"NULL!"<<endl;

	cout<<" 2: ";
	if(t2!=NULL)cout<<t2->getAncoratge()<<" "<<t2->getMida()<<endl;
	else cout<<"NULL!"<<endl;

	cout<<" 3: ";
	if(t3!=NULL)cout<<t3->getAncoratge()<<" "<<t3->getMida()<<endl;
	else cout<<"NULL!"<<endl;

	cout<<" 4: ";
	if(t4!=NULL)cout<<t4->getAncoratge()<<" "<<t4->getMida()<<endl;
	else cout<<"NULL!"<<endl;

	cout<<" 5: ";
	if(t5!=NULL)cout<<t5->getAncoratge()<<" "<<t5->getMida()<<endl;
	else cout<<"NULL!"<<endl;

	cout<<" 6: ";
	if(t6!=NULL)cout<<t6->getAncoratge()<<" "<<t6->getMida()<<endl;
	else cout<<"NULL!"<<endl;

	cout<<" 7: ";
	if(t7!=NULL)cout<<t7->getAncoratge()<<" "<<t7->getMida()<<endl;
	else cout<<"NULL!"<<endl;

	cout<<" 8: ";
	if(t8!=NULL)cout<<t8->getAncoratge()<<" "<<t8->getMida()<<endl;
	else cout<<"NULL!"<<endl;


char c;
cin>>c;*/

	CompressedInformacioGeometrica* infoT1;
	CompressedInformacioGeometrica* infoT2;
	CompressedInformacioGeometrica* infoT3;
	CompressedInformacioGeometrica* infoT4;
	CompressedInformacioGeometrica* infoT5;
	CompressedInformacioGeometrica* infoT6;
	CompressedInformacioGeometrica* infoT7;
	CompressedInformacioGeometrica* infoT8;

	// primer els posem tots a null, per assegurar
	infoT1=NULL;
	infoT2=NULL;
	infoT3=NULL;
	infoT4=NULL;
	infoT5=NULL;
	infoT6=NULL;
	infoT7=NULL;
	infoT8=NULL;

	// recollim la informacio geometrica de tots els nodes que no siguin nuls o fulla blanca	
	if( (t1!=NULL)&&(t1->tipus()!=0)) infoT1=t1->getInfGeo();
	if( (t2!=NULL)&&(t2->tipus()!=0)) infoT2=t2->getInfGeo();
	if( (t3!=NULL)&&(t3->tipus()!=0)) infoT3=t3->getInfGeo();
	if( (t4!=NULL)&&(t4->tipus()!=0)) infoT4=t4->getInfGeo();
	if( (t5!=NULL)&&(t5->tipus()!=0)) infoT5=t5->getInfGeo();
	if( (t6!=NULL)&&(t6->tipus()!=0)) infoT6=t6->getInfGeo();
	if( (t7!=NULL)&&(t7->tipus()!=0)) infoT7=t7->getInfGeo();
	if( (t8!=NULL)&&(t8->tipus()!=0)) infoT8=t8->getInfGeo();

	// Test for compatibility
	// atencio, compatible ha passat a ser estatica!
	if ( (CompressedONode::compatible(info,calcularInfo8(infoT1,infoT2,infoT3,infoT4,infoT5,infoT6,infoT7,infoT8), bNum, bHisto, bDist,epsilon)) ) 
	{
		
		//Mirem la mida dels que siguin parcials i comprovem si queda algun parcial
		bool quedenParcials=false;
		double mida = 0.0;		

		if(t1!=NULL) 
		{ 
			if(t1->tipus()==2)
			{
				 quedenParcials=true;
				if (mida<t1->getMida()) mida=t1->getMida(); 
			}
		}

		if(t2!=NULL) 
		{ 
			if(t2->tipus()==2)
			{
				 quedenParcials=true;
				if (mida<t2->getMida()) mida=t2->getMida(); 
			}
		}

		if(t3!=NULL) 
		{ 
			if(t3->tipus()==2)
			{
				 quedenParcials=true;
				if (mida<t3->getMida()) mida=t3->getMida(); 
			}
		}

		if(t4!=NULL) 
		{ 
			if(t4->tipus()==2)
			{
				 quedenParcials=true;
				if (mida<t4->getMida()) mida=t4->getMida(); 
			}
		}

		if(t5!=NULL) 
		{ 
			if(t5->tipus()==2)
			{
				 quedenParcials=true;
				if (mida<t5->getMida()) mida=t5->getMida(); 
			}
		}

		if(t6!=NULL) 
		{ 
			if(t6->tipus()==2)
			{
				 quedenParcials=true;
				if (mida<t6->getMida()) mida=t6->getMida(); 
			}
		}

		if(t7!=NULL) 
		{ 
			if(t7->tipus()==2)
			{
				 quedenParcials=true;
				if (mida<t7->getMida()) mida=t7->getMida(); 
			}
		}

		if(t8!=NULL) 
		{ 
			if(t8->tipus()==2)
			{
				 quedenParcials=true;
				if (mida<t8->getMida()) mida=t8->getMida(); 
			}
		}

		// si no queden parcials o la mida ja es massa petita, reportem
		if ( (mida/2.0)<info.getMidaNode()||!quedenParcials ) 
		{

			// Let's store the candidate zone
			CompressedCandidateZone *cand = new CompressedCandidateZone(8,t1,t2,t3,t4,t5,t6,t7,t8);
			candidates.push_back(cand);	
			
			if(t1!=NULL) t1->setNodeZones(cand);
			if(t2!=NULL) t2->setNodeZones(cand);
			if(t3!=NULL) t3->setNodeZones(cand);
			if(t4!=NULL) t4->setNodeZones(cand);
			if(t5!=NULL) t5->setNodeZones(cand);
			if(t6!=NULL) t6->setNodeZones(cand);
			if(t7!=NULL) t7->setNodeZones(cand);
			if(t8!=NULL) t8->setNodeZones(cand);
		
			/*cout<<"                                                                         ";
			cout<<"REPORTO!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
			cout<<"Compatible octet"<<endl;

			cout<<" 1: ";
			if(t1!=NULL)cout<<t1->getAncoratge()<<" "<<t1->getMida()<<endl;
			else cout<<"NULL!"<<endl;

			cout<<" 2: ";
			if(t2!=NULL)cout<<t2->getAncoratge()<<" "<<t2->getMida()<<endl;
			else cout<<"NULL!"<<endl;

			cout<<" 3: ";
			if(t3!=NULL)cout<<t3->getAncoratge()<<" "<<t3->getMida()<<endl;
			else cout<<"NULL!"<<endl;

			cout<<" 4: ";
			if(t4!=NULL)cout<<t4->getAncoratge()<<" "<<t4->getMida()<<endl;
			else cout<<"NULL!"<<endl;

			cout<<" 5: ";
			if(t5!=NULL)cout<<t5->getAncoratge()<<" "<<t5->getMida()<<endl;
			else cout<<"NULL!"<<endl;

			cout<<" 6: ";
			if(t6!=NULL)cout<<t6->getAncoratge()<<" "<<t6->getMida()<<endl;
			else cout<<"NULL!"<<endl;

			cout<<" 7: ";
			if(t7!=NULL)cout<<t7->getAncoratge()<<" "<<t7->getMida()<<endl;
			else cout<<"NULL!"<<endl;

			cout<<" 8: ";
			if(t8!=NULL)cout<<t8->getAncoratge()<<" "<<t8->getMida()<<endl;
			else cout<<"NULL!"<<endl;*/
			
			//cout<<"Mostro la zona candidata!"<<*cand<<endl;

			//aqui tb falta pintar	
		}
		else// hem de seguir tirant
		{

			// queden parcial i la mida encara es correcta
			// ara tenim parcials de mira adequada i fulles negres

			// declararem vuit nodes nous i adaptarem les crides per:
			// repetir les fulles negres i blanques i "baixar" per els que no ho siguin
			// vigilar de baixar nomes un "grao", descomprimint si cal
			// els "aux" serveixen per descomprimir i els nous per fer la nova crida 
			// sense que emprenyin les fulles negres.
			CompressedONode nodeAux1,nodeAux2,nodeAux3,nodeAux4,nodeAux5,nodeAux6,nodeAux7,nodeAux8;
			vector<bool> fillsTocats1, fillsTocats2, fillsTocats3, fillsTocats4, fillsTocats5, fillsTocats6,fillsTocats7, fillsTocats8;	

			CompressedONode *nouT1,*nouT2,*nouT3,*nouT4,*nouT5,*nouT6,*nouT7,*nouT8;

			// abans de llençar les cerques, descomprimim
			// si es parcial, el descomprimim			
			if( (t1!=NULL)&&(t1->tipus()==2))
			{							
				nodeAux1 = CompressedONode(t1->getAncoratge(),t1->getMida(),t1->getNivell() );
				fillsTocats1 = descomprimeix(t1,&nodeAux1);
	
				// baixem pel fill que toca
				nouT1=nodeAux1.getFill8();
			}
			else 
			{
				// tant si es fulla negra, com blanca, com null hem de cridar amb ell
				nouT1=t1;
			} 
	

			if( (t2!=NULL)&&(t2->tipus()==2))
			{							
				nodeAux2 = CompressedONode(t2->getAncoratge(),t2->getMida(),t2->getNivell() );
				fillsTocats2 = descomprimeix(t2,&nodeAux2);
	
				// baixem pel fill que toca
				nouT2=nodeAux2.getFill7();
			}
			else 
			{
				// tant si es fulla negra, com blanca, com null hem de cridar amb ell
				nouT2=t2;
			} 
	
			if( (t3!=NULL)&&(t3->tipus()==2))
			{							
				nodeAux3 = CompressedONode(t3->getAncoratge(),t3->getMida(),t3->getNivell() );
				fillsTocats3 = descomprimeix(t3,&nodeAux3);
	
				// baixem pel fill que toca
				nouT3=nodeAux3.getFill6();
			}
			else 
			{
				// tant si es fulla negra, com blanca, com null hem de cridar amb ell
				nouT3=t3;
			} 
	
			if( (t4!=NULL)&&(t4->tipus()==2))
			{							
				nodeAux4 = CompressedONode(t4->getAncoratge(),t4->getMida(),t4->getNivell() );
				fillsTocats4 = descomprimeix(t4,&nodeAux4);
	
				// baixem pel fill que toca
				nouT4=nodeAux4.getFill5();
			}
			else 
			{
				// tant si es fulla negra, com blanca, com null hem de cridar amb ell
				nouT4=t4;
			} 
	
			if( (t5!=NULL)&&(t5->tipus()==2))
			{							
				nodeAux5 = CompressedONode(t5->getAncoratge(),t5->getMida(),t5->getNivell() );
				fillsTocats5 = descomprimeix(t5,&nodeAux5);
	
				// baixem pel fill que toca
				nouT5=nodeAux5.getFill4();
			}
			else 
			{
				// tant si es fulla negra, com blanca, com null hem de cridar amb ell
				nouT5=t5;
			} 
	
			if( (t6!=NULL)&&(t6->tipus()==2))
			{							
				nodeAux6 = CompressedONode(t6->getAncoratge(),t6->getMida(),t6->getNivell() );
				fillsTocats6 = descomprimeix(t6,&nodeAux6);
	
				// baixem pel fill que toca
				nouT6=nodeAux6.getFill3();
			}
			else 
			{
				// tant si es fulla negra, com blanca, com null hem de cridar amb ell
				nouT6=t6;
			} 
	
			if( (t7!=NULL)&&(t7->tipus()==2))
			{							
				nodeAux7 = CompressedONode(t7->getAncoratge(),t7->getMida(),t7->getNivell() );
				fillsTocats7 = descomprimeix(t7,&nodeAux7);
	
				// baixem pel fill que toca
				nouT7=nodeAux7.getFill2();
			}
			else 
			{
				// tant si es fulla negra, com blanca, com null hem de cridar amb ell
				nouT7=t7;
			} 
	
			if( (t8!=NULL)&&(t8->tipus()==2))
			{							
				nodeAux8 = CompressedONode(t8->getAncoratge(),t8->getMida(),t8->getNivell() );
				fillsTocats8 = descomprimeix(t8,&nodeAux8);
	
				// baixem pel fill que toca
				nouT8=nodeAux8.getFill1();
			}
			else 
			{
				// tant si es fulla negra, com blanca, com null hem de cridar amb ell
				nouT8=t8;
			} 
	
			// ara totes les cerques es llençaran sobre els fills dels nodeAuxi si cal 
			// Nomes falta vigilar que no tinguem massa fulles blanques.
			bool ok1,ok2,ok3,ok4,ok5,ok6,ok7,ok8;
	
			ok1 = (nouT1!=NULL) && (nouT1->tipus()!=0);
			ok2 = (nouT2!=NULL) && (nouT2->tipus()!=0);
			ok3 = (nouT3!=NULL) && (nouT3->tipus()!=0);
			ok4 = (nouT4!=NULL) && (nouT4->tipus()!=0);
			ok5 = (nouT5!=NULL) && (nouT5->tipus()!=0);
			ok6 = (nouT6!=NULL) && (nouT6->tipus()!=0);
			ok7 = (nouT7!=NULL) && (nouT7->tipus()!=0);
			ok8 = (nouT8!=NULL) && (nouT8->tipus()!=0);
				
			//4. Search octet
			call_intern_search_8(info,ok1,ok2,ok3,ok4,ok5,ok6,ok7,ok8,nouT1,nouT2,nouT3,nouT4,nouT5,nouT6,nouT7,nouT8,bNum,bHisto,bDist);

			// recomprimeix per no perdre memoria tots els que hagin estat descomprimits
			if(fillsTocats1.size()!=0) recomprimeix(fillsTocats1, &nodeAux1);
			if(fillsTocats2.size()!=0) recomprimeix(fillsTocats2, &nodeAux2);
			if(fillsTocats3.size()!=0) recomprimeix(fillsTocats3, &nodeAux3);
			if(fillsTocats4.size()!=0) recomprimeix(fillsTocats4, &nodeAux4);
			if(fillsTocats5.size()!=0) recomprimeix(fillsTocats5, &nodeAux5);
			if(fillsTocats6.size()!=0) recomprimeix(fillsTocats6, &nodeAux6);
			if(fillsTocats7.size()!=0) recomprimeix(fillsTocats7, &nodeAux7);
			if(fillsTocats8.size()!=0) recomprimeix(fillsTocats8, &nodeAux8);

			// ara els nAux moren i les estructures "auxiliars" queden alliberades sense perdre memoria.
		
		}// tanca l'else de seguir tirant

	}// tanca l'if de compatible

	//cout<<"surto de 8, zones trobades fins ara "<<candidates.size()<<endl;
	//cout<<endl<<endl<<endl;

	//for(unsigned int i=0;i<candidates.size();i++) cout<<*candidates[i]<<endl;



}

void CompressedOctree::pintar()
{
	if (arrel != NULL) arrel->pintar();
}

void CompressedOctree::write()
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


vector<bool> CompressedOctree::descomprimeix(CompressedONode *t,CompressedONode *nAux)
{
	// per aquells fils de t que no siguin de "la mida inmediatament inferior a t" hi afegirem un node intermig d'aquesta mida i mantindrem els antics fills de t com a nodes d'un nivell inferior ("nets"). Tota l'estructura nova penjara del node auxiliar nAux per facilitar despres la seva eliminacio sense perdre memoria ni esborrar res que "no toqui"

	//cout<<"surto de descomprimir amb t: "<<endl;
	//t->write();
	//cout<<"i nAux: "<<endl;
	//nAux->write();

	double mida=t->getMida();
	double tolerancia= 0.00000000001;

	// primer, posem tots els fills en un vector i vigilem quins han de ser expandits
	vector<CompressedONode * > vFills = vector<CompressedONode * >(8);
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
			
				CompressedONode *fillAdoptiu = new CompressedONode(ancoratgeFillAdoptiu,mida/2,t->getNivell()+1);
				// li posem la llista d'elements del que sera el seu fill
				fillAdoptiu->setLlistaElements( vFills[i]->getLlistaElements());
						
				// la informacio geometrica tambe la copiem igual
				fillAdoptiu->setInfGeo( vFills[i]->getInfGeo() );

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
						cout<<"CompressedOctree:Descomprimir, error"<<endl;
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

void CompressedOctree::recomprimeix(vector<bool> fillsTocats, CompressedONode * nAux)
{

	//cout<<"reccomprimir "<<endl;
	//nAux->write();
	
	// primer, posem tots els fills en un vector i vigilem quins han de ser desenganxats de l'estructura
	vector<CompressedONode * > vFills = vector<CompressedONode * >(8);
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
			if((vFills[i]->getNodeZones()).size()!=0)
			{
				// el node intermig resulta que es part d'una zona candidata, 					// desenganxem-lo perque no se'ns esborri.
				// aqui perdem memoria!
				vFills[i]=NULL;					
			}
			else
			{		

				// desenganxa la informacio geometrica			
				vFills[i]->setInfGeo(NULL);
			
				// tots els fills d'aques han de passar a ser NULL per no esborrar res que no toqui quan ell	
				// mori (de fet tots excepte un ja eren NULL)
				vFills[i]->setFill1(NULL);
				vFills[i]->setFill2(NULL);
				vFills[i]->setFill3(NULL);
				vFills[i]->setFill4(NULL);
				vFills[i]->setFill5(NULL);
				vFills[i]->setFill6(NULL);
				vFills[i]->setFill7(NULL);
				vFills[i]->setFill8(NULL);
			}
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

CompressedONode * CompressedOctree::getArrel() {

	return arrel;
}
