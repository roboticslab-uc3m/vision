// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "VisionSegmentor.hpp"

namespace roboticslab
{

/************************************************************************/

bool VisionSegmentor::configure(ResourceFinder &rf) {

    //printf("--------------------------------------------------------------\n");
    if (rf.check("help")) {
        printf("VisionSegmentor options:\n");
        printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        // Do not exit: let last layer exit so we get help from the complete chain.
    }
    printf("VisionSegmentor using no additional special options.\n");

    segmentorThread.setInImg(&inImg);
    segmentorThread.setOutImg(&outImg);
    segmentorThread.setOutPort(&outPort);

    segmentorThread.init(rf);

    //-----------------OPEN LOCAL PORTS------------//
    inImg.open("/colorRegionDetection2D/img:i");
    outImg.open("/colorRegionDetection2D/img:o");
    outPort.open("/colorRegionDetection2D/features:o");

    return true;
}

/*****************************************************************/
double VisionSegmentor::getPeriod() {
    return 2.0;  // Fixed, in seconds, the slow thread that calls updateModule below
}

/************************************************************************/

bool VisionSegmentor::updateModule() {
    printf("VisionSegmentor alive...\n");
    return true;
}

/************************************************************************/

bool VisionSegmentor::interruptModule() {
    printf("VisionSegmentor closing...\n");
    outPort.interrupt();
    inImg.interrupt();
    outPort.close();
    inImg.close();
    return true;
}

/************************************************************************/

}  // namespace roboticslab
