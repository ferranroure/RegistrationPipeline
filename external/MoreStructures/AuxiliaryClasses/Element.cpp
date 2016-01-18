#include "Element.h"

#include <iostream>
#include <cstdlib>


void Element::write(ostream& os)
{
	os << "Centre: "; centre.write(os);
	os << " - Radi:" << radi << " - Categoria: "<<categoria; 
	if(marcat)os<<" MARCAT!"<<endl;
	else os<<" LLIURE!"<<endl;
	// Pintar tambe el descriptor, si ha estat calcular
	if(desc==NULL) os<<" Element no descrit"<<endl;
	//else os<<"Element descrit, no hi ha collons de pintar el descriptor aqui"<<endl;
	//desc->write(os);
}

Element::Element()
{
	centre = point3D();
	radi = 0;
	marcat = false;
	categoria="";
	desc=NULL;
}

Element::Element(double x, double y, double z, double r,string s)
{
	point3D p = point3D(x,y,z);
	centre = p;
	radi = r;
	marcat = false;
	categoria=s;
	desc=NULL;
}

Element::Element(point3D p, double r)
{
	centre = p;
	radi = r;
	marcat = false;
	categoria="";
	desc=NULL;
}


// constructor per copia!
// EP!!!! el descriptor nomes copia el punter!!
Element::Element(const Element& e) 
{
	if (this != &e) //evitar auto-assignacions  
	{ 
		centre = e.centre;
		radi = e.radi;
		marcat = e.marcat;
		categoria=e.categoria;
		desc=e.desc;
	}
}

//operador d'assignacio!
void Element::operator=(const Element& e) 
{
	if (this != &e) //evitar auto-assignacions  
	{ 
		centre = e.centre;
		radi = e.radi;
		marcat = e.marcat;
		categoria=e.categoria;
		desc=e.desc;
	}
}

point3D Element::getPoint() const
{
	return centre;
}

void Element::setPoint(point3D p)
{
	centre=p;
}

double Element::getRadi() const
{
	return radi;
}

bool Element::getMarcat() const
{
	return marcat;
}

void Element::setMarcat(bool m)
{
	if (marcat && m) {
		cout << "ERROR (Element.cpp::setMarcat) : Intentes marcar un Element que ja estava marcat!" << endl;
		exit(-1);
	}

	if (!marcat && !m) {
		cout << "ERROR (Element.cpp::setMarcat) : Intentes desmarcar un element que ja estava desmarcat!" << endl;
		exit(-1);
	}

	marcat = m;
}

bool Element::operator==(Element e) const 
{
	point3D centreE = e.getPoint();
	
	//Agafem els dos centres i comparem component a component
	return ( (centre.getX()==centreE.getX() ) && (centre.getY()==centreE.getY()) );
}

bool Element::operator<(Element e)const 
{
	return (centre < e.getPoint());
}

bool Element::operator !=(Element e) const
{	
	return( !(*this == e) );
}

bool Element::operator <=(Element e)  const
{
	return ( (*this<e)||(*this==e) );
}

bool Element::operator >(Element e) const
{
	return ( !(*this<=e) );
}

bool Element::operator >=(Element e) const
{
	return ( !(*this<e) );
}



/*bool Element::intersecta(Raig r, point3D &p)
{
	bool res = false;
	
	double a = 1.0;
	double b = 2*(r.getVector().prodEscalar(r.getOrigen()-centre));
	double c = pow((r.getOrigen()-centre).modul(),2) - pow(radi,2);
	
	if(b*b-(4*a*c)>=0) { //Ens assegurem que l'arrel quadrada no sigui negativa
		double resultat1 = (-b+sqrt(b*b-(4*a*c)))/2*a;
		double resultat2 = (-b-sqrt(b*b-(4*a*c)))/2*a;
		if(resultat1 >=0 || resultat2 >=0) { //Si hi ha alguna solucio correcte busquem la interseccio
			res=true;
			//Intersecció = Vd*resultat+P0 on Vd i P0 són el vector director i l'origen del Raig
			if ( (resultat1==resultat2) || (resultat1>=0 && resultat2<0) ) { //Els dos resultats son iguals o nomes resultat 1 es correcte
				p = r.getOrigen() + (r.getVector()*resultat1);
			} else if (resultat2>=0 && resultat1<0) { //Nomes el resultat2 es correcte
				p = r.getOrigen() + (r.getVector()*resultat2);
			} else { //Els dos resultats son correctes i diferents (en aquest cas hem de buscar el que esta mes a prop de l'observador)
				point3D interseccio1 = r.getOrigen() + r.getVector()*resultat1;
				point3D interseccio2 = r.getOrigen() + r.getVector()*resultat2;
				if ( interseccio1.dist(r.getOrigen()) < interseccio2.dist(r.getOrigen()) ) {
					p = interseccio1;
				} else {
					p = interseccio2;
				}
			}
		}
	}
	return res;
}*/

void Element::write()
{
	cout << "Centre: "; centre.write();
	cout << " - Radi:" << radi;
	cout<<" Categoria "<<categoria;
}

string Element::getCategoria()
{
	return categoria;
}

void Element::setCategoria(string s)
{
	categoria=s;
}


void Element::setIndex(int i) {

	index = i;
}

int Element::getIndex() {

	return index;
}
