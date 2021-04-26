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

print("detect()")
detectedObjects = yarp.Bottle()

if not iDetector.detect(yarpImgRgb, detectedObjects):
    print("Detector failed")
    raise SystemExit

print(detectedObjects.get(0).asDict().find("tlx").asInt32()) # 90
print(detectedObjects.get(0).asDict().find("brx").asInt32()) # 168
print(detectedObjects.get(0).asDict().find("tly").asInt32()) # 68
print(detectedObjects.get(0).asDict().find("bry").asInt32()) # 146

detectorDevice.close()
