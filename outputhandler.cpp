#include "outputhandler.h"

/* CONSTRUCTOR --------------------------------------------------
 *
 */
OutputHandler::OutputHandler(){

}


/* DESTRUCTOR --------------------------------------------------------
 *
 */
OutputHandler::~OutputHandler(){

}


/* SET DATA -----------------------------------------------------
 *
 * This method sets Data variable.
 */
void OutputHandler::setData(Data *d){

    data = d;
}


/*  EXECUTE -----------------------------------------------------
 *
 *  Execution of the output step of the pipline:
 *  Creates an output file with the result of the computation.
 */
void OutputHandler::execute(){

    data->B->createFileFromData(data->params.outfile, false, true);
}
