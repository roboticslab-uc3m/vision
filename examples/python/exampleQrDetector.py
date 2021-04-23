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
detectedObjects = iDetector.detect(yarpImgRgb)

for i in range(len(detectedObjects)):
    tlX = detectedObjects[i].find("tlx").asInt32()
    tlY = detectedObjects[i].find("tly").asInt32()
    trX = detectedObjects[i].find("trx").asInt32()
    trY = detectedObjects[i].find("try").asInt32()
    brX = detectedObjects[i].find("brx").asInt32()
    brY = detectedObjects[i].find("bry").asInt32()
    blX = detectedObjects[i].find("blx").asInt32()
    blY = detectedObjects[i].find("bly").asInt32()
    text = detectedObjects[i].find("text").asString()

    print("qr%d [[%d,%d],[%d,%d],[%d,%d],[%d,%d]]: \"%s\"" % (i, tlX, tlY, trX, trY, brX, brY, blX, blY, text))

detectorDevice.close()
