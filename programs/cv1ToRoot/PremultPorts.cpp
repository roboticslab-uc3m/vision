
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

    double l1 = 305;
    double l3 = 59.742;
    double l14 = 18;

    //-- H_root_hip (TRUNK) --
    KDL::Frame H_root_hip;

    //-- H_hip_neck (fixed) --
    KDL::Frame H_hip_neck_m1;
    H_hip_neck_m1.p.data[1] = -l1;

    KDL::Frame H_hip_neck_m2;
    H_hip_neck_m2.M = KDL::Rotation::RotX(M_PI);

    KDL::Frame H_hip_neck = H_hip_neck_m1 * H_hip_neck_m2;

    //-- H_neck_head (HEAD) --
    KDL::Frame H_neck_head;

    //-- H_head_rgb (fixed) --
    KDL::Frame H_head_rgb_m1;
    H_head_rgb_m1.p.data[2] = -l14;
    H_head_rgb_m1.p.data[1] = l3;

    KDL::Frame H_head_rgb_m2;
    H_head_rgb_m2.M = KDL::Rotation::RotY(M_PI/2.0);

    KDL::Frame H_head_rgb_m3;
    H_head_rgb_m2.M = KDL::Rotation::RotZ(M_PI);

    KDL::Frame H_head_rgb = H_head_rgb_m1 * H_head_rgb_m2 * H_head_rgb_m3;

    //--
    KDL::Frame H_rgb;
    H_rgb.p.data[0] = b.get(0).asDouble();
    H_rgb.p.data[1] = b.get(1).asDouble();
    H_rgb.p.data[2] = b.get(2).asDouble();

    KDL::Frame H_root = H_root_hip * H_hip_neck * H_neck_head * H_head_rgb * H_rgb;
    Bottle outB;
    outB.addDouble(H_root.p.x());
    outB.addDouble(H_root.p.y());
    outB.addDouble(H_root.p.z());
    outPort->write(outB);
}

/************************************************************************/

