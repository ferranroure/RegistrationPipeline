#include "point.h"

/* CONSTRUCTOR --------------------------------------------------------
 *
 */
Point::Point(){

    x = 0;
    y = 0;
    z = 0;
    index = -1;
    descriptor = NULL;
    normal = NULL;
    color.red = 255;
    color.green = 255;
    color.blue = 255;
    color.alpha = 255;
}


/* CONSTRUCTOR --------------------------------------------------------
 *
 */
Point::Point(double X, double Y, double Z){

    x = X;
    y = Y;
    z = Z;
    index = -1;
    descriptor = NULL;
    normal = NULL;
    color.red = 255;
    color.green = 255;
    color.blue = 255;
    color.alpha = 255;
}


/* CONSTRUCTOR --------------------------------------------------------
 *
 */
Point::Point(const Point &p){

    x = p.x;
    y = p.y;
    z = p.z;
    index = p.index;
    if(p.availableDescriptor()) {
        copyDescriptor(p.descriptor);
    }
    else{
        descriptor = NULL;
    }

    if(p.availableNormal()) {
        normal = new vector3D(*p.normal);
    }
    else{
        normal = NULL;
    }

    setColor(p.getRed(), p.getGreen(), p.getBlue(), p.getAlpha());
}

/* DESTRUCTOR --------------------------------------------------------
 *
 */
Point::~Point(){

    if(descriptor != NULL){

        delete descriptor;
    }

    if(normal != NULL){
        delete normal;
    }
}

double Point::getX() const{

    return x;
}

double Point::getY() const{

    return y;
}

double Point::getZ() const{

    return z;
}

int Point::getIndex(){

    return index;
}

void Point::setIndex(int ind){

    index = ind;
}

int Point::getDescSize() {

    return descriptor->getSize();
}

/* ADD NOISE ----------------------------------------------------------
 *
 *  Adds noise to a point, pondered with a certain threshold.
 */
void Point::addNoise(double nx, double ny, double nz, double threshold){

    x = x + nx * threshold;
    y = y + ny * threshold;
    z = z + nz * threshold;
}



/* ISINSIDE ------------------------------------------------------------
 *
 *  This method returns true if this point is located inside the bounding
 *  box formed by the incoming parameters. Returns false otherwise.
*/
bool Point::isInside(float xmin, float xmax, float ymin, float ymax, float zmin, float zmax){

    if( (xmin <= x && x <=xmax) && (ymin <= y && y <=ymax) && (zmin <= z && z <=zmax) ){
        return true;
    }
    else{
        return false;
    }
}

void Point::addCoords(double X, double Y, double Z){

    x = X;
    y = Y;
    z = Z;
}



/* SET DESCRIPTOR --------------------------------------------------------
 *
 *  Sets de descriptor object with other descriptor.
 */
void Point::setDescriptor(IDescriptor *desc){

    descriptor = desc;
}

void Point::copyDescriptor(IDescriptor *desc) {

    descriptor = desc->clone();

}

bool Point::availableDescriptor() const{

    if(descriptor != NULL) return true;
    else return false;
}

bool Point::availableNormal() const{

    if(normal != NULL) return true;
    else return false;
}


/* PRINT ---------------------------------------------------------------
 *
 *  This method prints a point.
 */
void Point::print(bool withNormals, bool withColor){

    cout << "(" << x << ",\t" << y << ",\t" << z << ") Index: " << index;
    if(withNormals) cout << " N: (" << normal->getX() << "," << normal->getY() << "," << normal->getZ() << ")";
    if(withColor) cout << " C: (" << getRed() << "," << getGreen() << "," << getBlue() << "," << getAlpha() << ")";

    cout << endl;
}


/* PRINT WITH DESC ---------------------------------------------------------------
 *
 *  This method prints a point with its descriptor.
 */
void Point::printWithDesc(){

    cout << "Point: (" << x << ",\t" << y << ",\t" << z << ") Index: " << index  << endl;
    cout << "Descriptor: ";
    descriptor->print();
}


/* UPDATE ---------------------------------------------------------------
 *
 *  Update the point coordinates.
 */
void Point::update(Point p){

    x = p.x;
    y = p.y;
    z = p.z;
}


/* OPERATORS  ---------------------------------------------------------------------- */

void Point::operator=(Point p)
{
    x = p.x;
    y = p.y;
    z = p.z;
    descriptor = p.descriptor;
}

Point Point::operator+(vector3D v)
{
    Point c;

    c.x = x+v.getX();
    c.y = y+v.getY();
    c.z = z+v.getZ();

    return c;
}

Point Point::operator-(vector3D v)
{
    Point c;

    c.x = x-v.getX();
    c.y = y-v.getY();
    c.z = z-v.getZ();

    return c;
}

vector3D Point::operator-(Point p)
{
    vector3D c;

    c.setX( x-p.x );
    c.setY( y-p.y );
    c.setZ( z-p.z );

    return c;
}

double Point::dist(Point *a)
{
    double res = 0;
    res=sqrt( pow(a->getX()-x,2) + pow(a->getY()-y,2) + pow(a->getZ()-z,2) );
    return res;
}

double Point::sqrDist(Point *a)
{
    double res = 0;
    res=( pow(a->getX()-x,2) + pow(a->getY()-y,2) + pow(a->getZ()-z,2) );
    return res;
}

Point Point::distVector(Point a){

    double vx = x - a.x;
    double vy = y - a.y;
    double vz = z - a.z;

    Point v(vx, vy, vz);

    return v;
}

bool Point::operator==(Point p) const
{
    // account for small variations attributed to noise or rounding errors
    return ( (fabs(x-p.x )<tole) && (fabs(z-p.z )<tole) && (fabs(z-p.z )<tole) );
}

bool Point::operator<(Point p) const //lexicographyc order
{
if ( x < p.x ) {
        return 1;
    } else {
        if ( fabs(x - p.x)<tole ) {
            if (y < p.y) {
                return 1;
            } else {
                if ( fabs(y - p.y)<tole ) {
                    if (z < p.z) {
                        return 1;
                    } else {
                        return 0;
                    }
                } else {
                    return 0;
                }
            }
        } else {
            return 0;
        }
    }
}

bool Point::operator !=(Point p) const
{
    return( !(*this == p) );
}

bool Point::operator <=(Point p)  const
{
    return ( (*this<p)||(*this==p) );
}

bool Point::operator >(Point p) const
{
    return ( !(*this<=p) );
}

bool Point::operator >=(Point p) const
{
    return ( !(*this<p) );
}

vector3D * Point::getNormal() {

    return normal;
}

void Point::setNormal(vector3D *norm) {

    if(availableNormal()){
        delete normal;
    }
    else{
        normal = new vector3D(norm->getX(), norm->getY(), norm->getZ());
    }
}

void Point::setNormal(double nx, double ny, double nz) {

    if(availableNormal()){
        delete normal;
    }
    else{
        normal = new vector3D(nx, ny, nz);
    }
}

IDescriptor * Point::getDescriptor() {

    return descriptor;
}


int Point::getRed() const {

    return color.red;
}

int Point::getGreen() const {

    return color.green;
}

int Point::getBlue() const {

    return color.blue;
}

int Point::getAlpha() const {

    return color.alpha;
}

void Point::setColor(int r, int g, int b, int a) {

    color.red = r;
    color.green = g;
    color.blue = b;
    color.alpha = a;
}
