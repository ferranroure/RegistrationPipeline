#include "data.h"
#include "IDescription.h"
#include "methods/des_shot.h"


string processFileByFormat(string file) // take a file name and, if it is not in ply format, change to ply and return the new name
{

    string suffix= file.substr (file.size()-4);

    if(suffix.compare(".ply")==0){return file;} //if the format is ply, do nothing
    else {
        cout << "READING FILE in " << suffix << " format " << file << endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZ>);

        // read pcd file
        pcl::PCDReader reader;
        if (reader.read(file, *cloud_input) < 0) {
            cout << "Error opening file: " << file << endl;
            throw ("Error opening file: %s.\n", file);
        }

        // also possible to write the shit in ply format
        pcl::PLYWriter writer;
        string newFileName = file.substr(0,file.size()-4)+".ply";
        writer.write(newFileName, *cloud_input);

        return newFileName;


    }

}

/* CONSTRUCTOR -------------------------------------------------------------
 *
 */
Data::Data(){

    A = NULL;
    B = NULL;
    result = NULL;
    cM = NULL;
    fM = NULL;
}


/* CONSTRUCTOR -------------------------------------------------------------
 *
 *  Constructor via parameters' file.
 */
Data::Data(char * paramsfile){

    setParametersXML(paramsfile);
    float normFactor = 1;


    A = new ElementSet(params.infile, params.dataStructure);

    // If no real data are used, we create a copy of target point cloud.
    if( ! params.realData){
        B = new ElementSet(*A);
        params.infile2 = params.infile;
    }
    else{
        B = new ElementSet(params.infile2, params.dataStructure);
    }


    // Resize (normalize) if is specified on params
    if(params.normalizeModels){

        float digA = A->getDiagonal();
        float digB = B->getDiagonal();

        if (digA >= digB) normFactor = digA;
        else normFactor = digB;

        motion3D *motionA = new motion3D(-(A->getCenter().getX()), -(A->getCenter().getY()), -(A->getCenter().getZ()));
        motion3D *motionB = new motion3D(-(B->getCenter().getX()), -(B->getCenter().getY()), -(B->getCenter().getZ()));

        A->transform(motionA);
        B->transform(motionB);

        delete motionA;
        delete motionB;

        A->scalePoints(normFactor);
        B->scalePoints(normFactor);

        A->createFileFromData("../models/testLab00.ply", false, false);
        B->createFileFromData("../models/testLab30.ply", false, false);
        exit(0);
    }



/*
    // B.size shoud be bigger than A.size.
    if(B->nPoints()<A->nPoints()){

        cout << "Switching models because B should be bigger than A!!" << endl;
        ElementSet *aux = A;
        A = B;
        B = aux;
        string saux = params.infile;
        params.infile = params.infile2;
        params.infile2 = saux;
    }
*/
    // If only refinement is used and no transformation applied to B
    if( ! params.useSS && ! params.useDescription && ! params.useDetection ){
        params.infileTemp = params.infile2;
    }

    result = NULL;
    cM = NULL;
    fM = NULL;

    params.MMD = A->getMMD();
//    params.nOutliersA = A->calcOutliers(10*A->getMMD());
//    params.nOutliersB = B->calcOutliers(10*B->getMMD());


//    A->calcNormals(params.radiusNormalFactor);
//    B->calcNormals(params.radiusNormalFactor);
}


/* DESTRUCTOR --------------------------------------------------------
 *
 */
Data::~Data(){

    delete A;
    delete B;
    delete result;
    delete cM;
    delete fM;
}




/* PRINT PARAMS --------------------------------------------------------------
 *
 */
void Data::printParams(){

    cout << endl;
    cout << "PARAMETERS OF THIS EXECUTION:  " << endl;
    cout << "Input file:                    " << params.infile << " (" << A->nPoints() << " points)" << endl;
    cout << "Input file 2                   " << params.infile2 << " (" << B->nPoints() << " points)" << endl;
    cout << "Temporal file:                 " << params.infileTemp << endl;
    cout << "Output file:                   " << params.outfile << endl;
    cout << "Results file:                  " << params.outres << endl;
    cout << "Real Data:                     " << params.realData << endl;
    cout << endl;
    cout << "Use Detection step:            " << params.useDetection << endl;
    cout << "Use Description step:          " << params.useDescription << endl;
    cout << "Use Srch. Strat. step:         " << params.useSS << endl;
    cout << "Use Refinement step:           " << params.useRefinement << endl;
    cout << endl;
    cout << "% of points for NNS:           " << params.percOfPoints << endl;
    cout << "% of noise:                    " << params.percOfNoise << endl;
    cout << "Mult. error factor nn:         " << params.nnErrorFactor << endl;
    cout << "Radius Normal Factor:          " << params.radiusNormalFactor << endl;
    cout << "Radius Search Factor:          " << params.radiusSearchFactor << endl;
    cout << "# of neighbours for corr:      " << params.nNeighbours << endl;
    cout << "Grid3D nCells:                 " << params.nCells << endl;
    cout << "Thrs Factor:                   " << params.thrsFactor << endl;
    cout << "# of samples detected:         "; if(!params.useDetection) cout << "--"; else cout << params.nSamples; cout << endl;
    cout << "Normalize Models:              " << params.normalizeModels << endl;
    cout << "Data Structure:                " << params.dataStructure << endl;

    cout << endl;
    cout << "MMD:                           " << params.MMD << endl;
//    cout << "# outliers A                   " << params.nOutliersA << endl;
//    cout << "# outliers B                   " << params.nOutliersB << endl;
    cout << "GT Descript. Thrs:             " << params.GTdescThrs << endl;
    cout << "GT minDescDist:                " << params.GTminDescDist << endl;
    cout << "GT maxDescDist:                " << params.GTmaxDescDist << endl;
    cout << "GT Residue:                    " << params.GTresidue << endl;
    cout << "GT % of paried points:         " << params.GTpercPairedPoints << endl;
    cout << endl;
    cout << "Detection Method:              "; if(!params.useDetection) cout << "--"; else cout << params.detectMethod; cout << endl;
    cout << "Description Method:            "; if(!params.useDescription) cout << "--"; else cout << params.descMethod; cout << endl;
    cout << "Search. Strat. Method:         "; if(!params.useSS) cout << "--"; else cout << params.SSMethod; cout << endl;
    cout << "Refinement Method:             "; if(!params.useRefinement) cout << "--"; else cout << params.refineMethod; cout << endl;
    cout << endl;

    cout << endl;
}


///* SET PARAMETERS --------------------------------------------------------------
// *
// *  Set the parameters of the execution reading the parameters' file.
// */
//void Data::setParameters(char * paramsfile){
//
//    string line;
//    ifstream myfile (paramsfile);
//
//    if (myfile.is_open()){
//
//        getline(myfile, line);  params.realData = atof(line.c_str());
//
//        getline(myfile, line);  params.infile = line;
//        getline(myfile, line);  params.infile2 = line;
//        getline(myfile, line);  params.infileTemp = line;
//        getline(myfile, line);  params.outfile = line;
//        getline(myfile, line);  params.outres = line;
//
//        getline(myfile, line);  params.detectMethod = line;
//        getline(myfile, line);  params.descMethod = line;
//        getline(myfile, line);  params.SSMethod = line;
//        getline(myfile, line);  params.refineMethod = line;
//
//        getline(myfile, line);  params.nLevels = atof(line.c_str());
//        getline(myfile, line);  params.nCells = atof(line.c_str());
//        getline(myfile, line);  params.percOfPoints = atof(line.c_str());
//
//        getline(myfile, line);  params.nnErrorFactor = atof(line.c_str());
//        getline(myfile, line);  params.percOfNoise = atof(line.c_str());
//        getline(myfile, line);  params.radiusNormalFactor = atof(line.c_str());
//        getline(myfile, line);  params.radiusSearchFactor = atof(line.c_str());
//
//        getline(myfile, line);  params.useDetection = atof(line.c_str());
//        getline(myfile, line);  params.useDescription = atof(line.c_str());
//        getline(myfile, line);  params.useSS = atof(line.c_str());
//        getline(myfile, line);  params.useRefinement = atof(line.c_str());
//
//        params.GTdescThrs = 0;
//        params.GTminDescDist = 0;
//        params.GTmaxDescDist = 0;
//        params.GTpercPairedPoints = 0;
//        params.GTresidue = 0;
//        params.MMD = 0;
//
//        myfile.close();
//    }
//    else{
//        cout << "Error opening the parameter's file!" << endl << endl;
//        exit(0);
//    }
//}

/* SET PARAMETERS XML --------------------------------------------------------------
 *
 *  Set the parameters of the execution reading the XML parameters' file.
 */
void Data::setParametersXML(char * paramsfile){

    XMLDocument doc;
    doc.LoadFile(paramsfile);

    XMLElement *parameters = doc.FirstChildElement("params");
    XMLElement *files = parameters->FirstChildElement("files");
    XMLElement *methods = parameters->FirstChildElement("methods");
    XMLElement *generalProps = parameters->FirstChildElement("generalProperties");

    // FILES
    params.realData = toBool( files->FirstChildElement("realData")->GetText() );
    params.infile = files->FirstChildElement("infile")->GetText();
    params.infile2 = files->FirstChildElement("infile2")->GetText();

  // check if it is ply or not and fix if necessary
    params.infile=processFileByFormat(params.infile);
    params.infile2=processFileByFormat(params.infile2);

    params.infileTemp = files->FirstChildElement("infileTemp")->GetText();
    params.outfile = files->FirstChildElement("outfile")->GetText();
    params.outres = files->FirstChildElement("outres")->GetText();

    // METHODS
    XMLElement *det = methods->FirstChildElement("detection");
    params.detectMethod = det->FirstChildElement("method")->GetText();
    params.useDetection = toBool(det->Attribute("use"));
    XMLElement *detProp =  det->FirstChildElement("properties");
    params.nSamples = atoi( detProp->FirstChildElement("nSamples")->GetText() );
    params.nLevels = atoi( detProp->FirstChildElement("nLevels")->GetText() );


    XMLElement *desc = methods->FirstChildElement("description");
    params.descMethod = desc->FirstChildElement("method")->GetText();
    params.useDescription = toBool(desc->Attribute("use"));
    XMLElement *descProp =  desc->FirstChildElement("properties");
    params.radiusNormalFactor = atof( descProp->FirstChildElement("radiusNormalFactor")->GetText() );
    params.radiusSearchFactor = atof( descProp->FirstChildElement("radiusSearchFactor")->GetText() );
    params.nNeighbours = atoi( descProp->FirstChildElement("nNeighbours")->GetText() );

    XMLElement *SS = methods->FirstChildElement("searchingStrategies");
    params.SSMethod = SS->FirstChildElement("method")->GetText();
    params.useSS = toBool(SS->Attribute("use"));
    XMLElement *SSProp =  SS->FirstChildElement("properties");
    params.nCells = atoi( SSProp->FirstChildElement("nCells")->GetText() );
    params.thrsFactor = atoi( SSProp->FirstChildElement("thrsFactor")->GetText() );
    XMLElement *FourPParams = SS->FirstChildElement("cmdLineParam");
    params.fourPUseCmdLineP = toBool(FourPParams->Attribute("use"));
    params.thr = atof(FourPParams->FirstChildElement("thr")->GetText());
    params.nPoints = atoi(FourPParams->FirstChildElement("n_points")->GetText());
    params.normDiff = atof(FourPParams->FirstChildElement("norm_diff")->GetText());
    params.delta = atof(FourPParams->FirstChildElement("delta")->GetText());
    params.overlap = atof(FourPParams->FirstChildElement("overlap")->GetText());

    XMLElement *ref = methods->FirstChildElement("refinement");
    params.refineMethod = ref->FirstChildElement("method")->GetText();
    params.useRefinement = toBool(ref->Attribute("use"));

    // GENERAL PROPERTIES
    params.percOfPoints = atof( generalProps->FirstChildElement("percOfPoints")->GetText() );
    params.nnErrorFactor = atof( generalProps->FirstChildElement("nnErrorFactor")->GetText() );
    params.percOfNoise = atof( generalProps->FirstChildElement("percOfNoise")->GetText() );
    params.normalizeModels = toBool( generalProps->FirstChildElement("normalizeModels")->GetText() );
    params.dataStructure = generalProps->FirstChildElement("dataStructure")->GetText();

    params.GTdescThrs = 0;
    params.GTminDescDist = 0;
    params.GTmaxDescDist = 0;
    params.GTpercPairedPoints = 0;
    params.GTresidue = 0;
    params.MMD = 0;

}

bool Data::toBool(string value){

    if(value == "TRUE" || value == "True" || value == "true"){

        return true;
    }
    else if(value == "FALSE" || value == "False" || value == "false"){

        return false;
    }
    else{

        cerr << "Boolean value is not correct." << endl;
        exit(0);
    }
}

void Data::crearteFileFromBase(string path, Point x, Point y, Point z){

    PlyIO plyio;
    vector<Point> *lin = new vector<Point>();
    lin->push_back(x);
    lin->push_back(y);
    lin->push_back(z);
    plyio.writeBase(path, lin);
}


