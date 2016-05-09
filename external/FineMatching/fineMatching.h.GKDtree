#ifndef _FINE_MATCHING_
#define _FINE_MATCHING_

//#include "../AuxiliaryClasses/Element.h"
//#include "../ElementSet/elementSet.h"
//#include "../AuxiliaryClasses/vector3D.h"

#include "XForm.h"
#include "TriMesh.h"
#include "TriMesh_algo.h"
#include "ICP.h"
#include "../../include/DataStructures/IDataStructure.h"

//#include "../AuxiliaryClasses/timer.h"

class fineMatching
{
	// definir dos trimesh llegir dos arxius al crear i posar de manera que es modifiquin com toca (potser guardar original 2)

	// sets to be matched
	TriMesh * mesh1;
	TriMesh * mesh2;
	IDataStructure *ds1;
	IDataStructure *ds2;

	public:
		// creator function (llegint dos arxius ply)
		fineMatching(const char *file1,const char *file2, IDataStructure *_ds1, IDataStructure *_ds2);
        fineMatching(TriMesh *A, TriMesh *B);
        ~fineMatching();

		// given an initial motion, icp sets
		double iCP(xform &xf1,xform &xf2);
	private:



};

#endif
