#include "descHistogram.h"

/* CONSTRUCTOR -----------------------------------------------------------
 *
 */
DescHistogram::DescHistogram(){
}

DescHistogram::DescHistogram(const DescHistogram &dh) {

    for (int i = 0; i < dh.getSize(); ++i) {

        desc.push_back(dh.getValue(i));
    }
}


/* CONSTRUCTOR -----------------------------------------------------------
 *
 */
DescHistogram::DescHistogram(vector<float> vd){

    desc = vd;
}


/* DESTRUCTOR ------------------------------------------------------------
 *
 */
DescHistogram::~DescHistogram(){

}


/* PRINT -----------------------------------------------------------------
 *
 */
void DescHistogram::print(){

    for(vector<float>::iterator it = desc.begin(); it != desc.end(); ++it){

        cout << *it << ", ";
    }
    cout << endl;
}

string DescHistogram::toString(){

    //string out;

    std::ostringstream out;

    for(vector<float>::iterator it = desc.begin(); it != desc.end(); ++it){

        out << *it << " ";
        //out = out + to_string(*it) + " ";
    }

    string s(out.str());

    return s;
}


/* IS AVAILABLE -----------------------------------------------------------
 *
 *  Returns true if vector desc contains some values.
 */
bool DescHistogram::isAvailable() const{

    return !(desc.empty());
}


/* COMPARE ----------------------------------------------------------------
 *
 *  Comparing two descriptors according RMSD of his descriptors.
 */
float DescHistogram::compare(IDescriptor *D){

    DescHistogram *DH = dynamic_cast<DescHistogram*>(D);

    vector<float>::iterator it2 = DH->desc.begin();

    float sum = 0;
	
    for(vector<float>::iterator it=desc.begin(); it!=desc.end(); ++it){
	    
        sum += pow((*it)-(*it2),2);
        ++it2;
    }

    float MSD = sum / desc.size();

    return sqrt(MSD);
}

int DescHistogram::getSize() const{

    return desc.size();
}

float DescHistogram::getValue(int pos) const{

    return desc.at(pos);
}

void DescHistogram::addElement(float f) {

    desc.push_back(f);
}

DescHistogram * DescHistogram::clone() const {

    return new DescHistogram(*this);
}
