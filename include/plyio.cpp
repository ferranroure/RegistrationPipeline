#include "plyio.h"

/* CONSTRUCTOR -------------------------------------------------------------
 *
 */
PlyIO::PlyIO(){

}


/* DESTRUCTOR -------------------------------------------------------------
 *
 */
PlyIO::~PlyIO(){

}


/*  READ PLY ---------------------------------------------------------------
 *
 *  This method reads a *.ply file. The vertices are extracted and stored
 *  in a vector<Point>.
 */
vector<Point>* PlyIO::readFile(string path){

    string line;
    ifstream myfile (path.c_str());
    vector<Point> *points = new vector<Point>();
    int nVertex = 0;
    bool readColor = false;
    bool readAlpha = false;

    if (myfile.is_open())
    {
        string one, two, three;

        while(!myfile.eof())
        {
            getline(myfile, line);

            istringstream iss(line);
            iss >> one >> two >> three;

            if(one=="element" && two=="vertex") {
                istringstream (three) >> nVertex;
            }

            // Read pointColor
            if(one=="property" && two=="uchar" && three=="red"){
                readColor = true;
            }

            //Read alpha
            if(one=="property" && two=="uchar" && three=="alpha"){
                readAlpha = true;
            }

            if(line=="end_header") break;
        }

        for(int i=0; i<nVertex; i++)
        {
            getline(myfile, line);
            if(myfile.eof()) break;

            if(!line.empty())
            {
                istringstream iss(line);
                double x, y, z;
                iss >> x >> y >> z;

                Point p(x, y, z);

                if(readColor){
                    int r, g, b, a;
                    a = 255;
                    iss >> r >> g >> b;
                    if(readAlpha){
                        iss >> a;
                    }
                    p.setColor(r, g, b, a);
                }

                p.setIndex(i);
                points->push_back(p);
            }
        }

        myfile.close();
    }

    else {cerr << "ERROR opening the file: " << path << endl; exit(0);}

    return points;
}


/* WRITE PLY -------------------------------------------------------------
 *
 *  Write a ply file from a point vector, using trianglulation information of
 *  in_path file.
 */
void PlyIO::writeFile(string in_path, string out_path, vector<Point> *lin){

    string line;
    ifstream infile (in_path.c_str());
    ofstream outfile (out_path.c_str());
    double x,y,z;

    if(lin == NULL)
    {
        cerr << "ERROR: The vector of points is empty!" << endl;
        exit(EXIT_FAILURE);
    }
    else if (!infile.is_open())
    {
        cerr << "ERROR: I can't open the file" << endl;
        exit(EXIT_FAILURE);
    }
    else
    {
        while(!infile.eof())
        {
            getline(infile, line);
            outfile << line << endl;

            if(line=="end_header") break;
        }

        string prop;
        for(vector<Point>::iterator it=lin->begin(); it!=lin->end(); ++it)
        {

            getline(infile, line);
            istringstream iss(line);
            iss >> x >> y >> z;

            outfile << it->getX() << " " << it->getY() << " " << it->getZ();

            // printing original normals or other properites
            // HEM D'IMPRIMIR LES NORMALS CANVIADES, NO LES DEL FITXER ORIGINAL!!!!
            while(iss >> prop) outfile << " " << prop;

            outfile << endl;
        }

        while(!infile.eof())
        {
            getline(infile, line);
            outfile << line << endl;

            if(infile.eof()) break;
        }

        infile.close();
        outfile.close();
    }
}


/* WRITE PLY POINTS -------------------------------------------------------------
 *
 *  Writes a ply files only with points, without triangulation information.
 */
void PlyIO::writeFile(string out_path, vector<Point> *lin){

    ofstream outfile (out_path.c_str());

    // Printing the header
    outfile << "ply" << endl;
    outfile << "format ascii 1.0 " << endl;
    outfile << "element vertex " << lin->size() << endl;
    outfile << "property float32 x" << endl;
    outfile << "property float32 y" << endl;
    outfile << "property float32 z" << endl;
    outfile << "end_header" << endl;

    for(vector<Point>::iterator it=lin->begin(); it!=lin->end(); ++it){

        outfile << it->getX() << " " << it->getY() << " " << it->getZ() << endl;
    }

    outfile.close();
}

/* WRITE PLY POINTS NORMALS -------------------------------------------------------------
 *
 *  Writes a ply files only with points and normals, without triangulation information.
 */
void PlyIO::writeFile(string in_path, string out_path, vector<Point> *lin, vector<vector3D> *norm){


    string line;
    ifstream infile (in_path.c_str());
    ofstream outfile (out_path.c_str());

    if(lin == NULL)
    {
        cerr << "ERROR: The vector of points is empty!" << endl;
        exit(EXIT_FAILURE);
    }
    else if (!infile.is_open())
    {
        cerr << "ERROR: I can't open the file" << endl;
        exit(EXIT_FAILURE);
    }
    else
    {
        while(!infile.eof())
        {
            getline(infile, line);
            outfile << line << endl;

            if (line=="property float32 z" || line=="property float z"){

                outfile << "property float32 nx" << endl;
                outfile << "property float32 ny" << endl;
                outfile << "property float32 nz" << endl;
            }

            if(line=="end_header") break;
        }


        int i = 0;
        for(vector<Point>::iterator it=lin->begin(); it!=lin->end(); ++it){

            outfile << it->getX() << " " << it->getY() << " " << it->getZ() << " ";
            outfile << norm->at(i).getX() << " " << norm->at(i).getY() << " " << norm->at(i).getZ() << endl;
            i++;
            getline(infile, line);
        }

        while(!infile.eof()){

            getline(infile, line);
            outfile << line << endl;
        }

        outfile.close();
    }
}

void PlyIO::writeBase(string out_path, vector<Point> *lin){

    ofstream outfile (out_path.c_str());

    // Printing the header
    outfile << "ply" << endl;
    outfile << "format ascii 1.0" << endl;
    outfile << "element vertex " << lin->size() << endl;
    outfile << "property float32 x" << endl;
    outfile << "property float32 y" << endl;
    outfile << "property float32 z" << endl;
//    if(lin->size()==3){
        outfile << "element face 1" << endl;
        outfile << "property list uchar int vertex_indices" << endl;
//    }
//    if(lin->size()==4){
//        outfile << "element edge 2" << endl;
//        outfile << "property int vertex1" << endl;
//        outfile << "property int vertex2" << endl;
//        outfile << "property uchar red" << endl;
//        outfile << "property uchar green" << endl;
//        outfile << "property uchar blue" << endl;
//    }

    outfile << "end_header" << endl;

    for(vector<Point>::iterator it=lin->begin(); it!=lin->end(); ++it){

        outfile << it->getX() << " " << it->getY() << " " << it->getZ() << endl;
    }

    if(lin->size() == 3) {
        outfile << "3 0 1 2" << endl;
    }
    if(lin->size() == 4){
//        outfile << "0 1 255 0 0" << endl;
//        outfile << "2 3 255 0 0" << endl;
        outfile << "4 0 1 2 3" << endl;
    }

    outfile.close();
}

/* WRITE PLY POINTS -------------------------------------------------------------
 *
 *  Writes a ply files only with points, without triangulation information.
 */
void PlyIO::writeFile(string out_path, vector<Point*> *lin, bool withNormals, bool withColor){

    ofstream outfile (out_path.c_str());

    // Printing the header
    outfile << "ply" << endl;
    outfile << "format ascii 1.0 " << endl;
    outfile << "element vertex " << lin->size() << endl;
    outfile << "property float32 x" << endl;
    outfile << "property float32 y" << endl;
    outfile << "property float32 z" << endl;
    if(withNormals){
        outfile << "property float32 nx" << endl;
        outfile << "property float32 ny" << endl;
        outfile << "property float32 nz" << endl;
    }
    if(withColor){
        outfile << "property uchar red" << endl;
        outfile << "property uchar green" << endl;
        outfile << "property uchar blue" << endl;
        outfile << "property uchar alpha" << endl;
    }
    outfile << "end_header" << endl;

    for(vector<Point*>::iterator it=lin->begin(); it!=lin->end(); ++it){

        outfile << (*it)->getX() << " " << (*it)->getY() << " " << (*it)->getZ();

        if(withNormals) outfile << " " << (*it)->getNormal()->getX() << " " << (*it)->getNormal()->getY() << " " << (*it)->getNormal()->getZ();

        if(withColor) outfile << " " << (*it)->getRed() << " " << (*it)->getGreen() << " " << (*it)->getBlue() << " " << (*it)->getAlpha();

        outfile << endl;
    }

    outfile.close();
}


