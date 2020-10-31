#!/usr/bin/env python

import yarp
import roboticslab_vision

#yarp.Network.init()
#if not yarp.Network.checkNetwork():
#    print("[error] Please try running yarp server")
#    quit()

detectorOptions = yarp.Property()
detectorOptions.put("device","HaarDetector")
detectorDevice = yarp.PolyDriver(detectorOptions)

iDetector = roboticslab_vision.viewIDetector(detectorDevice)

yarpImgRgb = yarp.ImageRgb()

rf = yarp.ResourceFinder()
rf.setDefaultContext("HaarDetector")
faceFullName = rf.findFileByName("tests/face-nc.pgm")
yarp.read(yarpImgRgb, faceFullName, yarp.FORMAT_PGM)

yarpImgFlex = yarp.FlexImage()
yarpImgFlex.setPixelCode(yarpImgRgb.getPixelCode())
yarpImgFlex.setQuantum(yarpImgRgb.getQuantum())
yarpImgFlex.setExternal(yarpImgRgb.getRawImage(), yarpImgRgb.width(), yarpImgRgb.height())

print("detect()")
detectedObjects = iDetector.detect(yarpImgFlex)

print(detectedObjects[0].find("tlx").asInt32()) # 90
print(detectedObjects[0].find("brx").asInt32()) # 168
print(detectedObjects[0].find("tly").asInt32()) # 68
print(detectedObjects[0].find("bry").asInt32()) # 146

detectorDevice.close()
