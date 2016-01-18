#ifndef _ELEMENT_
#define _ELEMENT_

#include <math.h>
#include <limits>

#include "./point3D.h"
#include "../../vector3D.h"

class Descriptor;

class Element
{
	protected:
		point3D centre;

		// Adding index for Pipeline project
		int index;

		// Unused parameters, for graphical and categorization purposes		
		double radi;
		string categoria;
		bool marcat;

		// Each element contains a Descriptor pointer 
		// it is initialised to NULL, whenever the descriptor is instantiated, this pointer should be updated
		Descriptor *desc;

		// normal vetor to the surface that the element is part of, angle between this vector and (1,0,0)
		vector3D n;
		double angle;

	public:
		Element(); //Default constructor
		Element(double x, double y, double z, double r, string s); // constructor with parameters, unused parameters get default values 
		Element(point3D p, double r); 	 // constructor with parameters, unused parameters get default values 	
	
		void operator=(const Element& e); //assignment
		Element(const Element& e); //Copy constructor  		

		//accessor methods	
		point3D getPoint()const;  
		void setPoint(point3D p); 
		double getRadi()const; 
		bool getMarcat()const;
		void setMarcat(bool); 
		string getCategoria();
		void setCategoria(string s);

		Descriptor* getDescriptor(){return desc;}
		void setDescriptor(Descriptor* d){desc=d;}

		
		bool operator ==(Element e)const;
		bool operator <(Element e)const;
		bool operator !=(Element e)const;
		bool operator >(Element e)const;
		bool operator <=(Element e)const;
		bool operator >=(Element e)const;
			
		void write(); //Metode que escriu el centre, radi de l'esfera pel terminal
		void write(ostream& os);

		void setIndex(int i);
		int getIndex();

		friend ostream& operator<<(ostream& os,Element e) 
		{
			e.write(os);
			return os;
		}
};

#endif
