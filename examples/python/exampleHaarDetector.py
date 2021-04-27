#!/usr/bin/env python3

import yarp
import roboticslab_vision

detectorOptions = yarp.Property()
detectorOptions.put("device", "HaarDetector")
detectorDevice = yarp.PolyDriver(detectorOptions)

if not detectorDevice.isValid():
    print("Device not available")
    raise SystemExit

iDetector = roboticslab_vision.viewIDetector(detectorDevice)

rf = yarp.ResourceFinder()
rf.setDefaultContext("HaarDetector")
faceFullName = rf.findFileByName("tests/face-nc.pgm")
yarpImgRgb = yarp.ImageRgb()

if not yarp.read(yarpImgRgb, faceFullName, yarp.FORMAT_PGM):
    print("Image file not available")
    raise SystemExit

print("detect()")
detectedObjects = yarp.Bottle()

if not iDetector.detect(yarpImgRgb, detectedObjects) or detectedObjects.size() == 0:
    print("Detector failed")
    raise SystemExit

detectedObject = detectedObjects.get(0).asDict()

print("tlx: %d" % detectedObject.find("tlx").asInt32()) # 90
print("tly: %d" % detectedObject.find("brx").asInt32()) # 168
print("brx: %d" % detectedObject.find("tly").asInt32()) # 68
print("bry: %d" % detectedObject.find("bry").asInt32()) # 146
