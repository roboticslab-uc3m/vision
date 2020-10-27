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

yarpImg = yarp.ImageRgb()

rf = yarp.ResourceFinder()
rf.setDefaultContext("HaarDetector")
faceFullName = rf.findFileByName("tests/face-nc.pgm")
yarp.read(yarpImg, faceFullName, yarp.FORMAT_PGM)

print("detect()")
detectedObjects = iDetector.detect(yarpImg)

print(detectedObjects[0].find("tlx").asInt32()) # 90
print(detectedObjects[0].find("brx").asInt32()) # 168
print(detectedObjects[0].find("tly").asInt32()) # 68
print(detectedObjects[0].find("bry").asInt32()) # 146

detectorDevice.close()
