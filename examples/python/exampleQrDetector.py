#!/usr/bin/env python

import yarp
import roboticslab_vision

detectorOptions = yarp.Property()
detectorOptions.put("device", "QrDetector")
detectorDevice = yarp.PolyDriver(detectorOptions)

iDetector = roboticslab_vision.viewIDetector(detectorDevice)

yarpImgRgb = yarp.ImageRgb()

rf = yarp.ResourceFinder()
rf.setDefaultContext("QrDetector")
qrFullName = rf.findFileByName("tests/rdqr.png")
yarp.read(yarpImgRgb, qrFullName, yarp.FORMAT_PNG)

print("detect()")
detectedObjects = yarp.Bottle()

if not iDetector.detect(yarpImgRgb, detectedObjects):
    print("Detection failed")
    raise SystemExit

for i in range(detectedObjects.size()):
    tlX = detectedObjects.get(i).asDict().find("tlx").asInt32()
    tlY = detectedObjects.get(i).asDict().find("tly").asInt32()
    trX = detectedObjects.get(i).asDict().find("trx").asInt32()
    trY = detectedObjects.get(i).asDict().find("try").asInt32()
    brX = detectedObjects.get(i).asDict().find("brx").asInt32()
    brY = detectedObjects.get(i).asDict().find("bry").asInt32()
    blX = detectedObjects.get(i).asDict().find("blx").asInt32()
    blY = detectedObjects.get(i).asDict().find("bly").asInt32()
    text = detectedObjects.get(i).asDict().find("text").asString()

    print("qr%d [[%d,%d],[%d,%d],[%d,%d],[%d,%d]]: \"%s\"" % (i, tlX, tlY, trX, trY, brX, brY, blX, blY, text))

detectorDevice.close()
