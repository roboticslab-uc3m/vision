// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CallbackPort.hpp"

namespace
{
    inline void scaleXY(int width1, int height1, const yarp::sig::Image & frame2, double px1, double py1, double * px2, double * py2)
    {
        if (width1 != frame2.width() || height1 != frame2.height())
        {
            *px2 = px1 * ((double)frame2.width() / (double)width1);
            *py2 = py1 * ((double)frame2.height() / (double)height1);
        }
        else
        {
            *px2 = px1;
            *py2 = py1;
        }
    }
}


/************************************************************************/
void CallbackPort::setParams(double _fx, double _fy, double _cx, double _cy) {
    fx = _fx;
    fy = _fy;
    cx = _cx;
    cy = _cy;
}

/************************************************************************/
void CallbackPort::setDepthSensorHandle(yarp::dev::IRGBDSensor * _irgbdSensor) {
    irgbdSensor = _irgbdSensor;
}

/************************************************************************/
void CallbackPort::setOutPort(yarp::os::Port* _outPort) {
    outPort = _outPort;
}

/************************************************************************/

void CallbackPort::onRead(yarp::os::Bottle& b) {
    //fprintf(stdout,"[CallbackPort] Got %s\n", b.toString().c_str());
    yarp::os::Bottle outLists;
    for (int i=0; i<b.size(); i++) {
        yarp::os::Bottle* pxCoords = b.get(i).asList();
        int pxX = pxCoords->get(0).asFloat64();
        int pxY = pxCoords->get(1).asFloat64();
        int width = pxCoords->get(2).asInt32();
        int height = pxCoords->get(3).asInt32();
        yarp::sig::ImageOf<yarp::sig::PixelFloat> depth;
        if (!irgbdSensor->getDepthImage(depth)) {
            printf("[CallbackPort] No depth image yet.\n");
            continue;
        }
        double depthX, depthY;
        scaleXY(width, height, depth, pxX, pxY, &depthX, &depthY);
        double mmZ = depth.pixel(int(depthX), int(depthY));  // maybe better do a mean around area?
        fprintf(stdout,"[CallbackPort] depth at (%d,%d) is %f.\n",pxX,pxY,mmZ);
        yarp::os::Bottle mmOut;
        double mmX = 1000.0 * (pxX - (cx * mmZ/1000.0)) / fx;
        double mmY = 1000.0 * (pxY - (cy * mmZ/1000.0)) / fy;
        mmOut.addFloat64(mmX);
        mmOut.addFloat64(mmY);
        mmOut.addFloat64(mmZ);
        if(mmZ != 0) outLists.addList() = mmOut;
    }
    outPort->write(outLists);
}

/************************************************************************/
