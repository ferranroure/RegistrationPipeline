#include "./fineMatching.h"
#include "../../include/DataStructures/kdtree/mykdtree.h"


fineMatching::fineMatching(const char *filename1,const char *filename2, IDataStructure *_ds1, IDataStructure *_ds2)
{
	mesh1 = TriMesh::read(filename1);
	mesh2 = TriMesh::read(filename2);
	ds1 = _ds1;
	ds2 = _ds2;
}

fineMatching::fineMatching(TriMesh *A, TriMesh *B){

    mesh1 = A;
    mesh2 = B;
}

fineMatching::~fineMatching(){

    delete mesh1;
    delete mesh2;
	delete ds1;
	delete ds2;
}


double fineMatching::iCP(xform &xf1,xform &xf2)
{

	int verbose = 1;
	bool do_scale = false;
	bool do_affine = false;
	bool bulkmode = false;

	//OJO!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	//xf1=xform(); xf2=xform();


	/*int c;
	while ((c = getopt(argc, argv, "harsvb")) != EOF) {
		switch (c) {
			case 'a': do_affine = true; do_scale = false; break;
			case 'r': do_affine = do_scale = false; break;
			case 's': do_scale = true; do_affine = false; break;
			case 'v': verbose = 2; break;
			case 'b': bulkmode = true; break;
			default: usage(argv[0]);
		}
	}*/

	TriMesh::set_verbose(verbose);

	/*xform xf1;
	string xffilename1 = xfname(filename1);
	xf1.read(xffilename1);

	xform xf2;
	string xffilename2 = xfname(filename2);
	xf2.read(xffilename2);*/


	if(ds1 == NULL || ds2 == NULL){
		cerr << "ICP::DataStructures are NULL, fix it! " << endl;
		exit(EXIT_FAILURE);
	}

//	KDtree *kd1 = new KDtree(mesh1->vertices);
//	KDtree *kd2 = new KDtree(mesh2->vertices);
	vector<float> weights1, weights2;

	// I commented this beacuse bulkmode is setted to false!!
//	if (bulkmode) {
//		float area1 = mesh1->stat(TriMesh::STAT_TOTAL, TriMesh::STAT_FACEAREA);
//		float area2 = mesh2->stat(TriMesh::STAT_TOTAL, TriMesh::STAT_FACEAREA);
//		float overlap_area, overlap_dist;
//		find_overlap(mesh1, mesh2, xf1, xf2, ds1, ds2, overlap_area, overlap_dist);
//		float frac_overlap = overlap_area / min(area1, area2);
//
//		if (frac_overlap < 0.1f) {
////            TriMesh::eprintf("finematching::Insufficient overlap\n");
//			throw("fineMatching::ICP Insufficient overlap\n\n");
//			//exit(1);
//		} else {
////            TriMesh::dprintf("%.1f%% overlap\n",
////                frac_overlap * 100.0f);
//		}
//	}


	float err = ICP(mesh1, mesh2, xf1, xf2, ds1, ds2, weights1, weights2, 0, verbose, do_scale, do_affine);
	if (err >= 0.0f)
		err = ICP(mesh1, mesh2, xf1, xf2, ds1, ds2, weights1, weights2, 0, verbose, do_scale, do_affine);

	if (err < 0.0f) {
//		TriMesh::eprintf("fineMatching::ICP failed\n");
		throw("fineMatching::ICP failed\n");
		//exit(1);
	}

//    TriMesh::eprintf("fineMatching::ICP succeeded - distance = %f\n", err);
	

	// what the fuck is that?
/*	if (bulkmode) {
		string xffilename12 = filename1;
		size_t dot = xffilename12.rfind(".", xffilename12.length());
		if (dot != string::npos)
			xffilename12.erase(dot);
		xffilename12 += string("--") + replace_ext(filename2, "xf");
		xform xf12 = inv(xf2) * xf1;
		xf12.write(xffilename12);
	} else {
		xf2.write(xffilename2);
	}*/

//    delete kd1;
//    delete kd2;


} 
