#include "ref_icp.h"

/* CONSTRUCTOR -----------------------------------------------------------
 *
 */
ref_ICP::ref_ICP(){

}


/* DESTRUCTOR -----------------------------------------------------------
 *
 */
ref_ICP::~ref_ICP(){

}


/* SET DATA -----------------------------------------------------
 *
 * This method sets Data variable.
 */
void ref_ICP::setData(Data *d){

    data = d;
}


/* EXECUTE --------------------------------------------------------------
 *
 *  This method executes desired ref_ICP method choosen by params file.
 *
void ref_ICP::execute(){

    if(data->params.RefineMethod == "ICP"){

        executeICP();
    }
    else{
        cerr << "No ref_ICP method is selected!" << endl;
    }
}
*/

/* EXECUTE ICP --------------------------------------------------------------
 *
 *  This method executes Rusinkiewicz's ICP.
 */
void ref_ICP::execute(){

    myICP *icp = new myICP(data->params.infile, data->params.infileTemp, data->A->getDataStruct(), data->B->getDataStruct());

    data->fM = icp->align();

    delete icp;
}
