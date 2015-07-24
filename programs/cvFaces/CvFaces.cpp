// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CvFaces.hpp"

namespace teo
{

/************************************************************************/
bool CvFaces::configure(ResourceFinder &rf) {

    cropSelector = DEFAULT_CROP_SELECTOR;
    ConstString strKinectDevice = DEFAULT_KINECT_DEVICE;
    ConstString strKinectLocal = DEFAULT_KINECT_LOCAL;
    ConstString strKinectRemote = DEFAULT_KINECT_REMOTE;
    watchdog = DEFAULT_WATCHDOG;  // double

    fprintf(stdout,"--------------------------------------------------------------\n");
    if (rf.check("help")) {
        printf("CvFaces options:\n");
        printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        printf("\t--cropSelector (default: \"%d\")\n",cropSelector);
        printf("\t--kinectDevice (device we create, default: \"%s\")\n",strKinectDevice.c_str());
        printf("\t--kinectLocal (if accesing remote, local port name, default: \"%s\")\n",strKinectLocal.c_str());
        printf("\t--kinectRemote (if accesing remote, remote port name, default: \"%s\")\n",strKinectRemote.c_str());
        printf("\t--watchdog ([s] default: \"%f\")\n",watchdog);
        // Do not exit: let last layer exit so we get help from the complete chain.
    }
    if(rf.check("cropSelector")) cropSelector = rf.find("cropSelector").asInt();
    printf("CvFaces using cropSelector: %d.\n",cropSelector);
    if(rf.check("kinectDevice")) strKinectDevice = rf.find("kinectDevice").asString();
    if(rf.check("kinectLocal")) strKinectLocal = rf.find("kinectLocal").asString();
    if(rf.check("kinectRemote")) strKinectRemote = rf.find("kinectRemote").asString();
    if(rf.check("watchdog")) watchdog = rf.find("watchdog").asDouble();
    printf("CvFaces using kinectDevice: %s, kinectLocal: %s, kinectRemote: %s.\n",
        strKinectDevice.c_str(), strKinectLocal.c_str(), strKinectRemote.c_str());
    printf("CvFaces using watchdog: %f.\n",watchdog);

    if (!rf.check("help")) {
        Property options;
        options.put("device",strKinectDevice);
        options.put("localName",strKinectLocal);  //
        options.put("remoteName",strKinectRemote);  //
        if(rf.check("noMirror")) options.put("noMirror",1);
        while(!dd.open(options)) {
            printf("Waiting for kinectDevice \"%s\"...\n",strKinectDevice.c_str());
            Time::delay(1);
        }
        printf("[CvFaces] success: kinectDevice available.\n");
        if (! dd.view(kinect) ) fprintf(stderr,"[CvFaces] warning: kinectDevice bad view.\n");
        else printf("[CvFaces] success: kinectDevice ok view.\n");

        segmentorThread.setIKinectDeviceDriver(kinect);
        segmentorThread.setOutImg(&outImg);
        segmentorThread.setOutPort(&outPort);

        segmentorThread.setCropSelector(cropSelector);
        if(cropSelector != 0) {
            segmentorThread.setOutCropSelectorImg(&outCropSelectorImg);
            segmentorThread.setInCropSelectorPort(&inCropSelectorPort);
        }
    }

    segmentorThread.init(rf);

    //-----------------OPEN LOCAL PORTS------------//
    outImg.open(strKinectLocal + "/img:o");
    outPort.open(strKinectLocal + "/state:o");
    if(cropSelector != 0) {
        outCropSelectorImg.open(strKinectLocal + "/cropSelector/img:o");
        inCropSelectorPort.open(strKinectLocal + "/cropSelector/state:i");
    }
    return true;
}

/*****************************************************************/
double CvFaces::getPeriod() {
    return watchdog;  // [s]
}

/************************************************************************/

bool CvFaces::updateModule() {
    printf("CvFaces alive...\n");
    return true;
}

/************************************************************************/

bool CvFaces::interruptModule() {
    printf("CvFaces closing...\n");
    segmentorThread.stop();
    outImg.interrupt();
    outPort.interrupt();
    if(cropSelector != 0) {
        outCropSelectorImg.interrupt();
        inCropSelectorPort.interrupt();
    }
    dd.close();
    outImg.close();
    outPort.close();
    if(cropSelector != 0) {
        outCropSelectorImg.close();
        inCropSelectorPort.close();
    }
    return true;
}

/************************************************************************/

}  // namespace teo
