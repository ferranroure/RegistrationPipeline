#include "base.h"

Base::Base(){
}

Base::Base(Point *pi, Point *pj, Point *pk){

//    cout << "New Base: " << endl;
//    pi->print();
    addPoint(pi);
//    pj->print();
    addPoint(pj);
//    pk->print();
    addPoint(pk);
//    cout << "New Base finished" << endl << endl;

    a = pi;
    b = pj;
    c = pk;

    computeProperties();
}

Base::~Base(){
}

void Base::addPoint(Point *p){

    // Check if the point is already in the base

    if(find(base.begin(), base.end(), p)!=base.end()){ // AIXÒ JA ES CONTROLA AMB EL METODE EXISTREPEATEDPOINTS()

//        cout << endl << endl;
//        cout << "candidate: "; p->print();
//        print();
//        cout << endl << endl;
        //cout << "Base :: The point " << p->index << " is already in this Base! Maybe you are using too few points :P" << endl;
    }

    base.push_back(p);

}

void Base::addBase(Point *pi, Point *pj, Point *pk){

    addPoint(pi);
    addPoint(pj);
    addPoint(pk);

    computeProperties();
}

Point * Base::getPoint(int pos){

    return base.at(pos);
}

int Base::size(){

    return base.size();
}

void Base::removeLastPoint(){

    base.pop_back();
}

void Base::clear(){

    base.clear();
}

void Base::print(){

    for(vector<Point*>::iterator it = base.begin(); it!=base.end(); ++it){
        (*it)->print();
    }
}

bool Base::existRepeatedPoints(){

    for(int i=0; i<base.size(); i++){

        int count = std::count(base.begin(), base.end(), base.at(i));

        if (count>1) return true;
    }

    return false;
}

void Base::computeProperties() {

    dist_ab = a->dist(b);
    dist_bc = b->dist(c);
    dist_ca = c->dist(a);

    // Using law of cosinus and then law of sinus.
    gamma = acos((dist_ab * dist_ab - dist_bc * dist_bc - dist_ca * dist_ca) / (-2* dist_bc * dist_ca));

    alpha = asin((sin(gamma) * dist_bc) / (dist_ab));
    beta = M_PI - gamma - alpha;

//    cout << "alpha: " << alpha << " beta: " << beta << " gamma: " << gamma << endl;
    double s = (dist_ab + dist_bc + dist_ca)/2;
    area = sqrt(s*(s- dist_ab)*(s- dist_bc)*(s- dist_ca));
}
