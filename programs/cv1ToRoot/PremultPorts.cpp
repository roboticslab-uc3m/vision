
#include "PremultPorts.hpp"

/************************************************************************/

void PremultPorts::setOutPort(Port* _outPort) {
    outPort = _outPort;
}

/************************************************************************/

void PremultPorts::onRead(Bottle& b) {
    //printf("[PremultPorts] Got %s\n", b.toString().c_str());
    if(b.size() != 3) {
        fprintf(stderr,"[error] for now only parsing 3-double lists\n");
        exit(1);  // case: other --> still not implemented
    }

    yarp::sig::Matrix H_root_hip(4,4);
    H_root_hip.eye();

    yarp::sig::Matrix H_hip_neck(4,4);
    H_hip_neck.eye();

    yarp::sig::Matrix H_neck_head(4,4);
    H_neck_head.eye();

    yarp::sig::Matrix H_head_rgb(4,4);
    H_head_rgb.eye();

    yarp::sig::Matrix H_rgb(4,4);
    H_rgb.eye();
    H_rgb(0,3) = b.get(0).asDouble();
    H_rgb(1,3) = b.get(1).asDouble();
    H_rgb(2,3) = b.get(2).asDouble();

    yarp::sig::Matrix H_root = H_rgb;  // Needs all elems
    Bottle outB;
    outB.addDouble(H_root(0,3));
    outB.addDouble(H_root(1,3));
    outB.addDouble(H_root(2,3));
    outPort->write(outB);
}

/************************************************************************/

