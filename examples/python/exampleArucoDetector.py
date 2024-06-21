#!/usr/bin/env python3

import yarp
import roboticslab_vision

detectorOptions = yarp.Property()
detectorOptions.put("device", "ArucoDetector")
detectorDevice = yarp.PolyDriver(detectorOptions)

if not detectorDevice.isValid():
    print("Device not available")
    raise SystemExit

iDetector = roboticslab_vision.viewIDetector(detectorDevice)

rf = yarp.ResourceFinder()
rf.setDefaultContext("ArucoDetector")
arucoFullName = rf.findFileByName("tests/rdaruco.png")
yarpImgRgb = yarp.ImageRgb()

if not yarp.read(yarpImgRgb, arucoFullName, yarp.FORMAT_PNG):
    print("Image file not available")
    raise SystemExit

print("detect()")
detectedObjects = yarp.Bottle()

if not iDetector.detect(yarpImgRgb, detectedObjects):
    print("Detector failed")
    raise SystemExit

for i in range(detectedObjects.size()):
    tlX = detectedObjects.get(i).asDict().find("tlx").asFloat32()
    tlY = detectedObjects.get(i).asDict().find("tly").asFloat32()
    trX = detectedObjects.get(i).asDict().find("trx").asFloat32()
    trY = detectedObjects.get(i).asDict().find("try").asFloat32()
    brX = detectedObjects.get(i).asDict().find("brx").asFloat32()
    brY = detectedObjects.get(i).asDict().find("bry").asFloat32()
    blX = detectedObjects.get(i).asDict().find("blx").asFloat32()
    blY = detectedObjects.get(i).asDict().find("bly").asFloat32()
    text = detectedObjects.get(i).asDict().find("text").asInt32()

    print("aruco%d [[%f,%f],[%f,%f],[%f,%f],[%f,%f]]: \"%d\"" % (i, tlX, tlY, trX, trY, brX, brY, blX, blY, text))
