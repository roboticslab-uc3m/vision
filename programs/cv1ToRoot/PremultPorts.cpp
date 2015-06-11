
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

    yarp::sig::Matrix H_root_hip(4,4);  // TRUNK
    H_root_hip.eye();

    yarp::sig::Matrix H_hip_neck(4,4);  // fixed
    H_hip_neck.eye();

    yarp::sig::Matrix H_neck_head(4,4);  // HEAD
    H_neck_head.eye();

    yarp::sig::Matrix H_head_rgb(4,4);  // fixed
    H_head_rgb.eye();

    yarp::sig::Matrix P_rgb(4,1);
    P_rgb(0,1) = b.get(0).asDouble();
    P_rgb(1,1) = b.get(1).asDouble();
    P_rgb(2,1) = b.get(2).asDouble();
    P_rgb(3,1) = 1.0;

    yarp::sig::Matrix P_root = H_root_hip * H_hip_neck * H_neck_head * H_head_rgb * P_rgb;  // Needs all elems
    Bottle outB;
    outB.addDouble(P_root(0,1));
    outB.addDouble(P_root(1,1));
    outB.addDouble(P_root(2,1));
    outPort->write(outB);
}

/************************************************************************/

