#include "LineCallbackPort.hpp"

namespace roboticslab {

/************************************************************************/
void LineCallbackPort::setSharedArea(SharedArea* _sharedArea) {
    sharedArea = _sharedArea;
}

/************************************************************************/
void LineCallbackPort::onRead(yarp::os::Bottle& b) {
    if (b.size() != 2) {
        fprintf(stderr,"[error] LineCallbackPort currently needs 2 lists to parse.\n");
        return;
    }
    double lineCoords[6];
/*    for(int i=0;i<6;i++) lineCoords[i] = b.get(i).asDouble(); */
///////////////////////////////////////////////////
    for (int i=0; i<b.size(); i++) {  // Parse each of the 2 elements
        if(!b.get(i).isList()) {  // Check if element is a list
            fprintf(stderr,"[error] LineCallbackPort needs elements to be lists to parse\n");
            return;
        }
        yarp::os::Bottle* inListElement = b.get(i).asList();
        if(inListElement->size()!=3) {  // case: inListElement is a point 3-vector
            fprintf(stderr,"[error] for now only parsing 3-double lists\n");
            return;  // case: other --> still not implemented
        }
        if(i==0) for(int i=0;i<3;i++) lineCoords[i] = inListElement->get(i).asDouble();
        else for(int i=0;i<3;i++) lineCoords[i+3] = inListElement->get(i).asDouble();
    }
///////////////////////////////////////////////
    printf("[LineCallbackPort] coord1: %f %f %f\t",lineCoords[0],lineCoords[1],lineCoords[2]);
    printf("[LineCallbackPort] coord2: %f %f %f\n",lineCoords[3],lineCoords[4],lineCoords[5]);
    sharedArea->setLC(lineCoords);


}

/************************************************************************/
}  // namespace roboticslab

