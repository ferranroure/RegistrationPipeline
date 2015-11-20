#include "inputhandler.h"

/* CONSTRUCTOR --------------------------------------------------
 *
 */
InputHandler::InputHandler(){
	data = NULL;
}


/* DESTRUCTOR --------------------------------------------------------
 *
 */
InputHandler::~InputHandler(){

}


/* SET DATA -----------------------------------------------------
 *
 * This method sets Data variable.
 */
void InputHandler::setData(Data *d){

    data = d;
}

/*  EXECUTE -----------------------------------------------------
 *
 *  Execution of the input step of the pipline:
 *  Reads point clouds from ply files and storing into
 *  vector of points.
 */
void InputHandler::execute(){

//    data->A->addNoise(data->params.percOfNoise);
//    data->B->addNoise(data->params.percOfNoise);

    if( ! data->params.realData){

        double dig = data->A->getDiagonal();
        motion3D *mT = new motion3D(dig, dig, 0);

        // Translation
        data->B->transform(mT);
        delete mT;


        // Rotation
        motion3D m1(0.78, 1);
        motion3D m2(1.0, 2);
        motion3D m3(0.5, 3);

        motion3D *M = new motion3D(m3*m2*m1);
        data->B->transform(M);

        delete M;

        // Applying noise
        //data->B->addNoise(data->params.percOfNoise);

        // Save the copied model in a temporal file.
        data->B->createFileFromData("models/prova/inicial.ply");
        data->B->createFileFromData(data->params.infileTemp);

    }
}
